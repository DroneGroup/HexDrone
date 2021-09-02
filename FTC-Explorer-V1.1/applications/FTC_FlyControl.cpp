/******************** (C) COPYRIGHT 2015 FTC ***************************
 * 作者		 ：FTC
 * 文件名  ：FTC_FlyControl.cpp
 * 描述    ：飞行控制
**********************************************************************************/
#include "FTC_FlyControl.h"

FTC_FlyControl fc;

FTC_FlyControl::FTC_FlyControl()
{
	rollPitchRate = 150;
	yawRate = 50;

	altHoldDeadband = 100;

	//重置PID参数
	PID_Reset();

	nowState = locked;
	upThrottle = 1750;
	downThrottle = 1500;
	slowLandThrottle = 1650;
}

//重置PID参数
void FTC_FlyControl::PID_Reset(void)
{
	pid[PIDROLL].set_pid(0.15, 0.15, 0.02, 200);
	pid[PIDPITCH].set_pid(0.15, 0.15, 0.02, 200);
	pid[PIDYAW].set_pid(0.8, 0.45, 0, 200);
	pid[PIDANGLE].set_pid(5, 0, 0, 0);
	pid[PIDMAG].set_pid(2, 0, 0, 0);
	pid[PIDVELZ].set_pid(1.5, 0.5, 0.002, 150);
	pid[PIDALT].set_pid(1.2, 0, 0, 200);
}

//飞行器姿态外环控制
void FTC_FlyControl::Attitude_Outter_Loop(void)
{
	int32_t errorAngle[2];
	Vector3f Gyro_ADC;

	switch (nowState)
	{
	case locked:
		if (ftc.f.ARMED && rc.rawData[AUX2] < RC_MINCHECK)
			nowState = standBy;
		break;
	case standBy:
		if (!ftc.f.ARMED)
			nowState = locked;
		if (imu.Acc_lpf.z > (1.7 * ACC_1G))
		{
			startCnt = 0;
			nowState = goingUP;
			rc.Command[ROLL] = 0;
			rc.Command[PITCH] = 0;
			rc.Command[YAW] = 0;
			//upThrottle += 0.01 * (imu.Acc_lpf.z - 1.7 * ACC_1G);
			//upThrottle = constrain_int16(upThrottle, RC_MINCHECK, RC_MAXCHECK);
			timeIncrease = 0.05 * (imu.Acc_lpf.z - 1.7 * ACC_1G);
			timeIncrease = constrain_int16(timeIncrease, 0, 1000);
		}
		else if (rc.rawData[AUX2] > RC_MINCHECK)
		{
			startCnt = 0;
			nowState = goingUP;
			rc.Command[ROLL] = 0;
			rc.Command[PITCH] = -5;
			rc.Command[YAW] = 0;
			timeIncrease = 0;
		}
		break;
	case goingUP:
	case goingDown:
	case slowLand:
		rc.Command[ROLL] = 0;
		rc.Command[PITCH] = 0;
		rc.Command[YAW] = 0;
		break;
	case autoUpEnd:
		if (rc.rawData[AUX2] > RC_MINCHECK)
			nowState = locked;
		break;
	default:
		break;
	}

	//计算角度误差值
	errorAngle[ROLL] = constrain_int32((rc.Command[ROLL] * 2), -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - imu.angle.x * 10;
	errorAngle[PITCH] = constrain_int32((rc.Command[PITCH] * 2), -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - imu.angle.y * 10;
	errorAngle[ROLL] = applyDeadband(errorAngle[ROLL], 2);
	errorAngle[PITCH] = applyDeadband(errorAngle[PITCH], 2);

	//获取角速度
	Gyro_ADC = imu.Gyro_lpf / 4.0f;

	//得到外环PID输出
	RateError[ROLL] = pid[PIDANGLE].get_p(errorAngle[ROLL]) - Gyro_ADC.x;
	RateError[PITCH] = pid[PIDANGLE].get_p(errorAngle[PITCH]) - Gyro_ADC.y;
	RateError[YAW] = ((int32_t)(yawRate)*rc.Command[YAW]) / 32 - Gyro_ADC.z;
}

//飞行器姿态内环控制
void FTC_FlyControl::Attitude_Inner_Loop(void)
{
	int32_t PIDTerm[3];
	float tiltAngle = constrain_float(max(abs(imu.angle.x), abs(imu.angle.y)), 0, 20);

	//My modification
	//Manually taking control
	if (nowState != autoUpEnd && nowState > locked && rc.rawData[THROTTLE] > RC_MINCHECK)
		nowState = autoUpEnd;

	switch (nowState)
	{
	case goingUP:
		useThrottle = upThrottle;
		startCnt++;
		if (startCnt > 1000 + timeIncrease)
		{
			nowState = goingDown;
			startCnt = 0;
		}
		break;
	case goingDown:
		useThrottle = downThrottle;
		startCnt++;
		if (startCnt > 700 + timeIncrease)
		{
			nowState = slowLand;
			startCnt = 0;
		}
		break;
	case slowLand:
		useThrottle = slowLandThrottle;
		startCnt++;
		if (startCnt > 350)
		{
			nowState = autoUpEnd;
			startCnt = 0;
		}
		break;
	default:
		useThrottle = rc.Command[THROTTLE];
		break;
	}

	for (u8 i = 0; i < 3; i++)
	{
		//当油门低于检查值时积分清零
		if (fc.nowState != goingDown && fc.nowState != goingUP && (rc.rawData[THROTTLE]) < RC_MINCHECK)
			pid[i].reset_I();

		//得到内环PID输出
		PIDTerm[i] = pid[i].get_pid(RateError[i], PID_INNER_LOOP_TIME * 1e-6);
	}

	PIDTerm[YAW] = -constrain_int32(PIDTerm[YAW], -300 - abs(rc.Command[YAW]), +300 + abs(rc.Command[YAW]));

	//油门倾斜补偿
	//if(!ftc.f.ALTHOLD)
	useThrottle = (useThrottle - 1000) / cosf(radians(tiltAngle)) + 1000;

	//PID输出转为电机控制量
	motor.writeMotor(useThrottle, PIDTerm[ROLL], PIDTerm[PITCH], PIDTerm[YAW]);
}

//飞行器高度外环控制
void FTC_FlyControl::Altitude_Outter_Loop(void)
{
	//to do
}

//飞行器高度内环控制
void FTC_FlyControl::Altitude_Inner_Loop(void)
{
	//to do
}

void FTC_FlyControl::AltHoldReset(void)
{
	AltHold = nav.position.z;
}

/************************ (C) COPYRIGHT 2015 FTC *****END OF FILE**********************/

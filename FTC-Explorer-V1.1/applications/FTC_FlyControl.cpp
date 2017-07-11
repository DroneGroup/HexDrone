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
	//to do
	Vector3f temp;
	temp = Vector3f(rc.Command[ROLL], rc.Command[PITCH], rc.Command[YAW]) - imu.angle;

	outterOut.x =  pid[PIDROLL].get_pid(temp.x, PID_OUTER_LOOP_TIME*1e-6);
	outterOut.y =  pid[PIDROLL].get_pid(temp.y, PID_OUTER_LOOP_TIME*1e-6);
	outterOut.z =  pid[PIDROLL].get_pid(temp.z, PID_OUTER_LOOP_TIME*1e-6);
}

//飞行器姿态内环控制
void FTC_FlyControl::Attitude_Inner_Loop(void)
{
	//to do
	RateError[ROLL] = outterOut.x  - imu.Gyro.x;
	RateError[PITCH] = outterOut.y - imu.Gyro.y;
	RateError[YAW]= outterOut.z - imu.Gyro.z;

	if(rc.Command[THROTTLE] < RC_MINCHECK)
	{
		pid[PIDROLL].reset_I();
		pid[PIDPITCH].reset_I();
		pid[PIDYAW].reset_I();
	}
	
	velPIDTerm.x = pid[PIDROLL].get_pid(RateError[ROLL], PID_INNER_LOOP_TIME * 1e-6);
	velPIDTerm.y = pid[PIDPITCH].get_pid(RateError[PITCH], PID_INNER_LOOP_TIME * 1e-6);
	velPIDTerm.z = pid[PIDYAW].get_pid(RateError[YAW], PID_INNER_LOOP_TIME * 1e-6);

	maxAngle = imu.angle.x > imu.angle.y ? imu.angle.x : imu.angle.y;

	motor.writeMotor(rc.Command[THROTTLE]/cosf(maxAngle), velPIDTerm.x, velPIDTerm.y, velPIDTerm.z);	
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

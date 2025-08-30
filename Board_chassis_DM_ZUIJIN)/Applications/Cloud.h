/**
 * @file Cloud.h
 * @author ZHY
 * @brief 
 * @version 0.1
 * @date 2025-08-30
 * 
 * @copyright 
 * 
 */
#ifndef __CLOUD_H
#define __CLOUD_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "FuzzyPID.h"
#include "board_comm.h"
#include "BSP_fdcan.h"
#include "kalman_filter.h"
#include "motor_pid.h"
#include "GM6020.h"
#include "ins.h"
#include "M2006.h"
//重新安装电机或移用代码时需要重新测量这些值（toalAngle）后再允许运动。

#define Cloud_Yaw_Center 4060
#define Cloud_Yaw_ReverseCenter 7809
#define pi 3.1415926
#ifdef __cplusplus
class Class_Cloud
{
public:
    void Cloud_Init(void);
    void Motor_Init();
    void Cloud_Sport_Out(void);
	void Cloud_Yaw_Angle_Set(void);
	void PID_Clear_Yaw(void);
    void Cloud_SetTargetYaw(float _target_yaw){Target_Yaw = _target_yaw;}
    Class_GM6020 YAW;
    Class_M2006 Dial_2006;
    KalmanFilter_t Cloud_YawMotorAngle_Error_Kalman, Cloud_YAWODKalman, Cloud_YawCurrent_Kalman, Cloud_YawCurrent_Kalman_manul;
    Class_MotorPid M6020s_YawIPID;
    Class_MotorPid M6020s_Yaw_SpeedPID;
    Class_MotorPid M6020s_YawOPID;
    Class_MotorPid AutoAim_M6020s_YawIPID;
    Class_MotorPid AutoAim_M6020s_YawOPID;
    Class_MotorPid M2006s;
    float Target_Yaw; 		    //云台目标yaw轴
private:
    float Yaw_Raw; 			      //yaw轴原始数据
    float Pitch_Raw;   		    //pitch轴原始数据

    float Target_Pitch;     	//云台目标pitch轴
	float Vision_Yaw_Delta;		//视觉Yaw轴数据(差值)
	float Vision_Pitch_Delta;	//视觉Pitch轴数据(差值)
};

extern Class_Cloud Cloud;
#endif
#endif
/**
 * @file Cloud.h
 * @author Cyx
 * @brief 
 * @version 0.1
 * @date 2023-08-15
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
#include "N100.h"
#include "kalman_filter.h"
#include "motor_pid.h"
#include "GM6020.h"
//重新安装电机或移用代码时需要重新测量这些值（toalAngle）后再允许运动。

#define Cloud_Yaw_Center 4060
#define Cloud_Yaw_ReverseCenter 7809
#define pi 3.1415926
class Class_Cloud
{
public:
    void Cloud_Init(void);
    void Cloud_Sport_Out(void);
	void Cloud_Yaw_Angle_Set(void);
	void PID_Clear_Yaw(void);
    Class_GM6020 YAW;
    KalmanFilter_t Cloud_YawMotorAngle_Error_Kalman, Cloud_YAWODKalman, Cloud_YawCurrent_Kalman, Cloud_YawCurrent_Kalman_manul;
    Class_MotorPid M6020s_YawIPID;
    Class_MotorPid M6020s_Yaw_SpeedPID;
    Class_MotorPid M6020s_YawOPID;
    Class_MotorPid AutoAim_M6020s_YawIPID;
    Class_MotorPid AutoAim_M6020s_YawOPID;
private:
    float Yaw_Raw; 			      //yaw轴原始数据
    float Pitch_Raw;   		    //pitch轴原始数据
    float Target_Yaw; 		    //云台目标yaw轴
    float Target_Pitch;     	//云台目标pitch轴
	float Vision_Yaw_Delta;		//视觉Yaw轴数据(差值)
	float Vision_Pitch_Delta;	//视觉Pitch轴数据(差值)
};

extern IMUData_Packet_t IMUData_Packet;
extern AHRSData_Packet_t AHRSData_Packet;
extern Class_Cloud Cloud;
#endif
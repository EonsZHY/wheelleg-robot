/**
 * @file motor_pid.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "math.h"
#include "FuzzyPID.h"
// pid控制类型
typedef enum
{
	pid_control_increase,
	pid_control_normal,
	pid_control_frontfeed,
	pid_control_frontfuzzy,
	pid_control_angle  // 舵轮舵向电机专用PID(解决就近转位问题)
}pid_control;

class Class_MotorPid
{
public:
    void Incremental_PID();
    void Position_PID();
    void Angle_PID();
    void Incremental_PIDInit(float _Kp, float _Ki, float _Kd, float _ecd_max, uint32_t _MaxOutput, uint32_t _IntegralLimit);
    void Position_PIDInit(float _Kp, float _Ki, float _Kd, float _Kf, float _ecd_max, float _MaxOutput, float _Integral_Separation, float _IntegralLimit);
    void Clear_PositionPIDData();
    void Clear_IncrementalPIDData();
    void Position_PID_Yaw(FUZZYPID_Data_t *fuzzy_t);
    void SetTarget(float _target);
    void SetMeasured(float _measured);
private:
    float Target;     //目标值
    float Measured;   //测量值
    float err;        //误差
    float err_last;   //前一次的误差
    float err_change; 
	float error_target;
    float err_beforeLast;   
	float last_set_point; 
    float Kp;
    float Ki;
    float Kd; 
	float Kf;         
    float p_out;
    float i_out;
    float d_out;               
	float f_out;               
    float pwm;               //输出值  
    float MaxOutput;           //最大输出值限制
    float Integral_Separation; //积分项分离阈值
    float IntegralLimit;       //积分项限幅
    float ecd_max;       //被控电机编码器最大值（例如8191对应360度）




};



#endif
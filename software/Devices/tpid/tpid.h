/**
 * @file PID.h
 * @author xyz
 * @brief pid相关处理
 * @version 1.1
 * @date 2024-09-18
 * @copyright Transistor BUAA
 */

#ifndef TPID_H
#define TPID_H

/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx_hal.h"


/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/


/**
 * @brief PID处理结构体
 */
struct Struct_PID_Manage_Object
{
   float kp;
   float ki;
   float kd;
   float kf;

   float error;
   float last_error;
   float before_last_error;
   float integral_error; // 误差积分累积
   float error_target;   // 前馈控制

   float set_point;      // 目标值
   float now_point;      // 当前值
   float last_set_point; // 上一次目标值

   float integral_limit; // 积分限幅
   float output_limit;   // 输出限幅
   float deadzone;       // 死区

   float p_out;  // p输出
   float i_out;  // i输出
   float d_out;  // d输出
   float f_out;  // 前馈f输出
   float output; // 总输出
	 
	 //模糊PID计算的基准参数
};

/* Exported variables --------------------------------------------------------*/
extern void TPID_Init(struct Struct_PID_Manage_Object *pid, float _kp, float _ki, float _kd, float _kf, float _integral_limit, float _output_limit, float _deadzone);
extern float T_Update(struct Struct_PID_Manage_Object *pid, float _now_point, float _set_point);
extern void pid_change(struct Struct_PID_Manage_Object *pid, float _kp, float _ki, float _kd, float _kf);
extern void Position_PIDInit(struct Struct_PID_Manage_Object *pid, float _kp, float _ki, float _kd, float _kf, float _output_limit, float _integral_limit);
#ifdef __cplusplus
extern "C" {
#endif
extern float Position_PID(struct Struct_PID_Manage_Object *pid, float _now_point, float _set_point);

#ifdef __cplusplus
}
#endif

#endif

#ifndef __TASK_MANAGER_H__
#define __TASK_MANAGER_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "board_comm.h"
extern void motor_init();
extern void DM_MotorTask();
extern void WheelMotorTask();
extern void YawMotorTask();
extern void ChassisCalcTask();
extern void boardCommunicateTask();
#ifdef __cplusplus
}
#endif

#endif
/**
 *******************************************************************************
 * @file      : INS.h
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INS_TASK_H
#define __INS_TASK_H

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/

#include "stdint.h"
#define RAD_2_DEGREE 57.2957795f     // 180/pi
#define DEGREE_2_RAD 0.01745329252f  // pi/180
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/


#define INS_TASK_PERIOD 1

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    float q[4];  // 四元数估计值

    float Gyro[3];  // 角速度
    float Accel[3];          // 加速度
    float MotionAccel_b[3];  // 机体坐标加速度
    float MotionAccel_n[3];  // 绝对系加速度

    
    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} INS_t;

typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
extern INS_t INS;

void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

#endif

#ifdef __cplusplus
extern "C" {
#endif
#include "Saber_C3.h"
void INS_Init(void);
void INS_Task(void);
#ifdef __cplusplus
}
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/




#endif /* __INS_TASK_H */

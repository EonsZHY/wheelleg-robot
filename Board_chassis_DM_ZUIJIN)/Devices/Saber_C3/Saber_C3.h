/**
 * @file Saber_C3.h
 * @author lxr(784457420@qq.com)
 * @brief 
 * @version 1.0
 * @date 2023-8-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */
 #ifndef __SABER_C3_H
 #define __SABER_C3_H
 
 #include "main.h"
 #include "usart.h"
typedef struct Saber_Angle_t
 {
float Acc_x ;
float Acc_y ;
float	Acc_z ;
		
float	Gyro_x ;
float Gyro_y ;
float	Gyro_z ;
		
float q1 ;
float q2;
float q3;
float q4;
		
float RoLL ;
float Pitch ;
float Yaw ;
 }Saber_Angle_t;
 
/* 陀螺仪DMA和拼接缓冲区大小,以及帧尾的位置 */
#define Saber_Data_Length 76
#define Saber_Data_Buffer 152
#define Frame_End 75

 extern uint8_t Saber_Rxbuffer[Saber_Data_Length];
 extern uint8_t Saber_Montage[Saber_Data_Buffer];

 extern Saber_Angle_t Saber_Angle;
  /**
  * @brief          IMU初始化任务，打开串口
  * @retval         none
  */
extern void Saber_Init(void);
 /**
  * @brief          IMU欧拉角读取任务
  * @retval         none
  */
extern void Saber_Read(void);
   /**
  * @brief          帧头帧尾判断
  * @parameter[in]  接收数据首地址
  * @retval         检查符合协议返回1，否则返回0。
  */
extern uint8_t Saber_Cheak(uint8_t *p);
 
#endif

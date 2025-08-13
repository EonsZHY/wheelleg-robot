/**
 * @file N100.h
 * @author zhy
 * @brief 
 * @version 1.0
 * @date 2025-8-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef __N100_H
#define __N100_H


#include "main.h"
#include "usart.h"

#define N100_Data_length 
#define FRAME_HEAD      0Xfc 
#define FRAME_TAIL      0Xfd
#define TYPE_IMU        0x40
#define TYPE_AHRS       0x41
#define SEND_DATA_SIZE    24
#define RECEIVE_DATA_SIZE 11
#define IMU_RS 64
#define AHRS_RS 56
#define INSGPS_RS 80
#define IMU_LEN  0x38   //56+8  8组数据
#define AHRS_LEN 0x30   //48+8  7组数据

extern uint8_t N100_Rxbuffer;
extern uint8_t N100_ReImu[IMU_RS];
extern uint8_t N100_ReAhrs[AHRS_RS];
extern uint8_t N100_tmpData[IMU_RS];
extern uint8_t Count;
extern uint8_t last_rsnum;
extern int Flag_Ahrs ;
extern int Flag_Imu ;
extern int Handle_Ahrs;
extern int Handle_Imu;

typedef struct IMUData_Packet_t{
    float gyroscope_x;          //unit: rad/s
    float gyroscope_y;          //unit: rad/s
    float gyroscope_z;          //unit: rad/s
    float accelerometer_x;      //m/s^2
    float accelerometer_y;      //m/s^2
    float accelerometer_z;      //m/s^2
    float magnetometer_x;       //mG
    float magnetometer_y;       //mG
    float magnetometer_z;       //mG
    float imu_temperature;      //C
    float Pressure;             //Pa
    float pressure_temperature; //C
    uint32_t Timestamp;          //us
} IMUData_Packet_t;

typedef struct AHRSData_Packet_t
{
    float RollSpeed;   //unit: rad/s
    float PitchSpeed;  //unit: rad/s
    float HeadingSpeed;//unit: rad/s
    float Roll;        //unit: rad
    float Pitch;       //unit: rad
    float Heading;     //unit: rad
    float Qw;//w          //Quaternion
    float Qx;//x
    float Qy;//y
    float Qz;//z
    long long Timestamp; //unit: us
    uint8_t zero_flag;
} AHRSData_Packet_t;


extern IMUData_Packet_t IMUData_Packet;
extern AHRSData_Packet_t AHRSData_Packet;

#ifdef __cplusplus
extern "C" {
#endif

void N100_Init(void);
void N100_Read(void);
uint8_t N100_Cheak(uint8_t *p);
float DATA_Trans(uint8_t Data_1,uint8_t Data_2,uint8_t Data_3,uint8_t Data_4);
long long timestamp(uint8_t Data_1,uint8_t Data_2,uint8_t Data_3,uint8_t Data_4,uint8_t Data_5,uint8_t Data_6,uint8_t Data_7,uint8_t Data_8);


#ifdef __cplusplus
}
#endif

#endif

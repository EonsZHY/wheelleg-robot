/**
 ******************************************************************************
 * @file    N100.h
 * @author  (您的姓名)
 * @brief   N100 IMU传感器驱动程序接口
 * @date    2025-09-20
 ******************************************************************************
 * @attention
 * 本驱动程序使用UART逐字节中断方式接收数据，适用于连续数据流场景。
 * 需在CubeMX中为对应的UART (例如UART7) 开启全局中断(NVIC)。
 ******************************************************************************
 */

#ifndef __N100_H
#define __N100_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Public Defines ------------------------------------------------------------*/
// --- 粘贴这份干净的版本 ---
// 数据帧常量
#define FRAME_HEAD      0xFC
#define FRAME_TAIL      0xFD

// 数据包类型
#define TYPE_IMU        0x40
#define TYPE_AHRS       0x41

// 数据包原始数据长度 (包含帧头帧尾等)
#define IMU_RS          64
#define AHRS_RS         56

// 数据包纯数据长度 (用于数据包内的长度字段校验)
#define IMU_LEN         0x38  // 56字节
#define AHRS_LEN        0x30  // 48字节

/* Public types --------------------------------------------------------------*/
// IMU数据包结构体
typedef struct 
{
    float gyroscope_x;        // 单位: rad/s
    float gyroscope_y;        // 单位: rad/s
    float gyroscope_z;        // 单位: rad/s
    float accelerometer_x;    // 单位: m/s^2
    float accelerometer_y;    // 单位: m/s^2
    float accelerometer_z;    // 单位: m/s^2
    float magnetometer_x;     // 单位: mG
    float magnetometer_y;     // 单位: mG
    float magnetometer_z;     // 单位: mG
    long long Timestamp;      // 单位: us
} IMUData_Packet_t;

// AHRS数据包结构体
typedef struct
{
    float RollSpeed;    // 单位: rad/s
    float PitchSpeed;   // 单位: rad/s
    float HeadingSpeed; // 单位: rad/s
    float Roll;         // 单位: rad
    float Pitch;        // 单位: rad
    float Heading;      // 单位: rad
    float Qw;           // 四元数 W
    float Qx;           // 四元数 X
    float Qy;           // 四元数 Y
    float Qz;           // 四元数 Z
    long long Timestamp;// 单位: us
} AHRSData_Packet_t;

/* External variables --------------------------------------------------------*/
// -- 解析后的数据包 --
extern IMUData_Packet_t IMUData_Packet;
extern AHRSData_Packet_t AHRSData_Packet;

// -- 接收缓冲区 --
// 最终存放有效AHRS数据的缓冲区
extern uint8_t N100_ReAhrs[AHRS_RS];
// 最终存放有效IMU数据的缓冲区
extern uint8_t N100_ReImu[IMU_RS];
// 软件组包缓冲区
extern uint8_t N100_assembly_buf[IMU_RS]; 
// UART硬件接收单字节的物理缓冲区
extern uint8_t N100_rx_byte;

// -- 处理标志位 --
// volatile 防止编译器优化，确保在主循环和中断中能正确读写
extern volatile int Handle_Ahrs; 
extern volatile int Handle_Imu;

/* Public function prototypes ------------------------------------------------*/
/**
 * @brief 初始化N100模块，启动UART接收中断
 */
void N100_Init(void);

/**
 * @brief 在主循环中调用，处理已接收到的完整数据帧
 */
void N100_Read(void);

#ifdef __cplusplus
}
#endif

#endif /* __N100_H */
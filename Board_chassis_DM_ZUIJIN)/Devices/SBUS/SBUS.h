/**
 * @file    SBUS.h
 * @author  ZHY (由 Gemini 重构)
 * @brief   ET16S遥控器SBUS协议驱动程序头文件。
 * 该驱动包含精确的16通道解析、鲁棒的双缓冲DMA接收机制，
 * 以及用于平滑控制的斜坡处理功能。
 * @version 1.2
 * @date    2025-09-18
 */

#ifndef __SBUS_H
#define __SBUS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h" // 引入STM32 HAL库定义

/* Public Defines ------------------------------------------------------------*/
// 注意：以下常量定义了遥控器通道到具体物理量的映射关系，可根据需求修改
#define SBUS_DATA_SIZE      25.0f   // SBUS每帧数据大小为25字节
#define DEADZONE_THRESHOLD  20.0f   // SBUS通道中值的死区阈值，防止摇杆不回中
#define SPEED_MAX           2.0f    // 映射后的最大前进/后退速度 (m/s)
#define W_SPEED_MAX         4.0f    // 映射后的最大横移速度 (m/s)
#define YAW_SPEED_MAX       4.0f    // 映射后的最大偏航角速度 (rad/s)
#define MID_LEN             0.3f    // 腿长中间值 (m)
#define MAX_LEN             0.1f    // 腿长最大变化量 (m)
#define TORQUE_MAX          7.0f   // 最大扭矩 (Nm)
// 档位判断容差（避免数据抖动导致跳变）
#define TOLERANCE 50
// 斜坡处理参数配置 (数值越大，响应越快；数值越小，响应越平滑)
#define RAMP_RATE_SPEED     2.0f    // 速度斜坡率 (m/s^2)
#define RAMP_RATE_W_SPEED   3.0f    // 横移速度斜坡率 (m/s^2)
#define RAMP_RATE_YAW       4.0f    // 偏航角速度斜坡率 (rad/s^2)
#define RAMP_RATE_LEN       1.0f    // 腿长斜坡率 (m/s)
#define RAMP_RATE_TORQUE    1.0f
/* Public Structs ------------------------------------------------------------*/

/**
 * @brief SBUS最终输出的控制数据包
 * @note  这是应用程序应该直接使用的结构体，其中包含了所有经过平滑处理的最终控制值。
 */
typedef struct {
    float speed;        // 前进/后退速度 (m/s)
    float w_speed;      // 横移速度 (m/s)
    float len;          // 腿长 (m)
    float yaw_speed;    // 偏航角速度 (rad/s)
    float jump_flag;    // 跳跃标志 (0.0f 或 1.0f)
    float status_flag;  //设置机器人状态标志（1：自瞄模式  2：操作手状态 3：失能状态）
	float reset_flag;   //整车复位标志
    float torque;
} SBUS_RevPack_t;


/* Public Variables ----------------------------------------------------------*/

// 声明一个全局变量，用于在其他文件中访问最终的遥控数据
extern SBUS_RevPack_t sbus_rx_data;

extern UART_HandleTypeDef* sbus_huart;              // 保存SBUS所使用的UART句柄
/* Public Function Prototypes ------------------------------------------------*/

/**
 * @brief 初始化SBUS接收
 * @note  此函数应在主函数初始化阶段调用一次。
 * 它会配置UART并启动第一次DMA接收。
 * @param huart 指向目标UART的句柄 (例如 &huart5)。
 */
void SBUS_Init(UART_HandleTypeDef* huart);

/**
 * @brief SBUS数据处理函数
 * @note  此函数应在主循环 (while(1)) 中被周期性调用。
 * 它会检查是否有新的SBUS数据，并执行解析、映射和斜坡处理。
 */
void SBUS_Handle(void);
void SBUS_RX_Callback_Handler(void);
void SBUS_Open();

#ifdef __cplusplus
}
#endif

#endif // __SBUS_H
/**
 ******************************************************************************
 * @file    N100.c
 * @author  (您的姓名)
 * @brief   N100 IMU传感器驱动程序实现
 * @date    2025-09-20
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "N100.h"
#include "usart.h" // 假设您的huart7在此文件中定义
#include <string.h>
#include <math.h>

/* Private variables ---------------------------------------------------------*/
// -- 解析后的数据包定义 --
IMUData_Packet_t IMUData_Packet;
AHRSData_Packet_t AHRSData_Packet;

// -- 接收缓冲区定义 --
uint8_t N100_ReAhrs[AHRS_RS];
uint8_t N100_ReImu[IMU_RS];
uint8_t N100_assembly_buf[IMU_RS]; // 用于在软件中拼接数据帧
uint8_t N100_rx_byte;             // 用于接收UART硬件传来的单个字节

// -- 处理标志位定义 --
volatile int Handle_Ahrs = 0;
volatile int Handle_Imu = 0;

/* Private function prototypes ---------------------------------------------*/
static float DATA_Trans(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4);
static long long timestamp_Trans(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4, 
                                 uint8_t Data_5, uint8_t Data_6, uint8_t Data_7, uint8_t Data_8);

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  初始化N100模块，启动UART逐字节接收中断
 * @param  None
 * @retval None
 */
void N100_Init(void)
{
    // 清零所有缓冲区和标志位
    memset(N100_ReImu, 0, sizeof(N100_ReImu));
    memset(N100_ReAhrs, 0, sizeof(N100_ReAhrs));
    memset(N100_assembly_buf, 0, sizeof(N100_assembly_buf));
    Handle_Ahrs = 0;
    Handle_Imu = 0;
    
    // 启动UART接收中断，每次只接收1个字节到N100_rx_byte中
    // 这是整个接收机制的起点
    // 启动UART的DMA接收，每次只接收1个字节到N100_rx_byte中
    if (HAL_UART_Receive_DMA(&huart7, &N100_rx_byte, 1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  在主循环中调用，用于处理已完整接收的数据
 * @param  None
 * @retval None
 */
void N100_Read(void)
{
    // 检查AHRS数据处理标志位
    if (Handle_Ahrs == 1)
    {
        // 双重校验，确保数据有效
        if (N100_ReAhrs[0] == FRAME_HEAD && N100_ReAhrs[1] == TYPE_AHRS && N100_ReAhrs[2] == AHRS_LEN)
        {   
            AHRSData_Packet.RollSpeed    = DATA_Trans(N100_ReAhrs[7], N100_ReAhrs[8], N100_ReAhrs[9], N100_ReAhrs[10]);
            AHRSData_Packet.PitchSpeed   = DATA_Trans(N100_ReAhrs[11], N100_ReAhrs[12], N100_ReAhrs[13], N100_ReAhrs[14]);
            AHRSData_Packet.HeadingSpeed = DATA_Trans(N100_ReAhrs[15], N100_ReAhrs[16], N100_ReAhrs[17], N100_ReAhrs[18]);
            AHRSData_Packet.Roll         = DATA_Trans(N100_ReAhrs[19], N100_ReAhrs[20], N100_ReAhrs[21], N100_ReAhrs[22]);
            AHRSData_Packet.Pitch        = DATA_Trans(N100_ReAhrs[23], N100_ReAhrs[24], N100_ReAhrs[25], N100_ReAhrs[26]);
            AHRSData_Packet.Heading      = DATA_Trans(N100_ReAhrs[27], N100_ReAhrs[28], N100_ReAhrs[29], N100_ReAhrs[30]);
            AHRSData_Packet.Qw           = DATA_Trans(N100_ReAhrs[31], N100_ReAhrs[32], N100_ReAhrs[33], N100_ReAhrs[34]);
            AHRSData_Packet.Qx           = DATA_Trans(N100_ReAhrs[35], N100_ReAhrs[36], N100_ReAhrs[37], N100_ReAhrs[38]);
            AHRSData_Packet.Qy           = DATA_Trans(N100_ReAhrs[39], N100_ReAhrs[40], N100_ReAhrs[41], N100_ReAhrs[42]);
            AHRSData_Packet.Qz           = DATA_Trans(N100_ReAhrs[43], N100_ReAhrs[44], N100_ReAhrs[45], N100_ReAhrs[46]);
            AHRSData_Packet.Timestamp    = timestamp_Trans(N100_ReAhrs[47], N100_ReAhrs[48], N100_ReAhrs[49], N100_ReAhrs[50], N100_ReAhrs[51], N100_ReAhrs[52], N100_ReAhrs[53], N100_ReAhrs[54]);
        }
        Handle_Ahrs = 0; // 清除标志位，等待下一帧
    }

    // 检查IMU数据处理标志位
    if (Handle_Imu == 1)
    {
        if (N100_ReImu[0] == FRAME_HEAD && N100_ReImu[1] == TYPE_IMU && N100_ReImu[2] == IMU_LEN)
        {
            IMUData_Packet.gyroscope_x    = DATA_Trans(N100_ReImu[7], N100_ReImu[8], N100_ReImu[9], N100_ReImu[10]);
            IMUData_Packet.gyroscope_y    = DATA_Trans(N100_ReImu[11], N100_ReImu[12], N100_ReImu[13], N100_ReImu[14]);
            IMUData_Packet.gyroscope_z    = DATA_Trans(N100_ReImu[15], N100_ReImu[16], N100_ReImu[17], N100_ReImu[18]);
            IMUData_Packet.accelerometer_x= DATA_Trans(N100_ReImu[19], N100_ReImu[20], N100_ReImu[21], N100_ReImu[22]);
            IMUData_Packet.accelerometer_y= DATA_Trans(N100_ReImu[23], N100_ReImu[24], N100_ReImu[25], N100_ReImu[26]);
            IMUData_Packet.accelerometer_z= DATA_Trans(N100_ReImu[27], N100_ReImu[28], N100_ReImu[29], N100_ReImu[30]);
            IMUData_Packet.magnetometer_x = DATA_Trans(N100_ReImu[31], N100_ReImu[32], N100_ReImu[33], N100_ReImu[34]);
            IMUData_Packet.magnetometer_y = DATA_Trans(N100_ReImu[35], N100_ReImu[36], N100_ReImu[37], N100_ReImu[38]);
            IMUData_Packet.magnetometer_z = DATA_Trans(N100_ReImu[39], N100_ReImu[40], N100_ReImu[41], N100_ReImu[42]);
            IMUData_Packet.Timestamp      = timestamp_Trans(N100_ReImu[55], N100_ReImu[56], N100_ReImu[57], N100_ReImu[58], N100_ReImu[59], N100_ReImu[60], N100_ReImu[61], N100_ReImu[62]);
        }
        Handle_Imu = 0; // 清除标志位，等待下一帧
    }
}




/* Private functions ---------------------------------------------------------*/

/**
 * @brief  将4个u8数据转换为float (使用memcpy，更安全可靠)
 */
static float DATA_Trans(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4)
{
    uint8_t source[4] = {Data_1, Data_2, Data_3, Data_4};
    float tmp = 0.0f;
    memcpy(&tmp, source, sizeof(float));
    return tmp;
}

/**
 * @brief  将8个u8数据转换为long long
 */
static long long timestamp_Trans(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4, 
                                 uint8_t Data_5, uint8_t Data_6, uint8_t Data_7, uint8_t Data_8)
{
    long long transition_64 = 0;
    
    transition_64 |= (long long)Data_8 << 56;
    transition_64 |= (long long)Data_7 << 48;
    transition_64 |= (long long)Data_6 << 40;
    transition_64 |= (long long)Data_5 << 32;
    transition_64 |= (long long)Data_4 << 24;
    transition_64 |= (long long)Data_3 << 16;
    transition_64 |= (long long)Data_2 << 8;
    transition_64 |= (long long)Data_1;

    return transition_64;
}
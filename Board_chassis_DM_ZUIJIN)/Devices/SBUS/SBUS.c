/**
 * @file    SBUS.c
 * @author  ZHY (由 Gemini 重构)
 * @brief   ET16S遥控器SBUS协议驱动程序实现。
 * @version 1.2
 * @date    2025-09-18
 */

#include "SBUS.h"
#include <string.h> // for memcpy
#include <stdlib.h> // for abs

/* Private Structs -----------------------------------------------------------*/

// 用于存放解析后的16通道11位原始数据
typedef struct
{
    uint16_t ch[16];
} SBUS_RawData_t;

// 用于斜坡处理的内部数据结构
typedef struct {
    float speed_target, w_speed_target, yaw_speed_target, len_target;
    float speed_current, w_speed_current, yaw_speed_current, len_current;
} RampControl_t;

/* Private Defines -----------------------------------------------------------*/

// SBUS原始值范围定义 (根据遥控器实际输出进行校准)
static const uint16_t SBUS_MIN = 321;
static const uint16_t SBUS_MAX = 1663;
static const uint16_t SBUS_MID = 992;

/* Private Variables ---------------------------------------------------------*/

// 文件内静态变量，实现信息隐藏
UART_HandleTypeDef* sbus_huart;              // 保存SBUS所使用的UART句柄
static SBUS_RawData_t sbus_raw;                     // 存放解析后的原始数据
static RampControl_t ramp_control;                  // 斜坡处理的内部变量

static volatile uint8_t sbus_rx_completed = 0;      // 接收完成标志 (volatile关键字防止编译器优化)
static uint8_t dma_rx_buf[25];                      // 缓冲区1: DMA硬件接收专用
static uint8_t sbus_decode_buf[25];                 // 缓冲区2: 主循环解码专用

/* Public Variables ----------------------------------------------------------*/

// 定义在.h中声明的全局变量
SBUS_RevPack_t sbus_rx_data;

/* Private Function Prototypes -----------------------------------------------*/

static float map_sbus_to_range(uint16_t sbus_value, float min_out, float max_out);
static float Ramp_Handle(float current, float target, float ramp_rate, float dt);

/* Public Function Implementations -------------------------------------------*/
/**
 * @brief 11-12通道档位转换：353→1，1695→2
 * @param sbus_val: SBUS原始通道值
 * @retval 1/2档位
 */
 static float map_to_2levels(int16_t sbus_val) 
{
    if (sbus_val <= SBUS_MIN + TOLERANCE) {
        return 1;
    } else if (sbus_val >= SBUS_MAX - TOLERANCE) {
        return 2;
    }
    return 0; 
}
/**
 * @brief 5-10通道档位转换：353→1，1024→2，1695→3
 * @param sbus_val: SBUS原始通道值
 * @retval 1/2/3档位
 */
static float map_to_3levels(int16_t sbus_val) {
    if (sbus_val <= SBUS_MIN + TOLERANCE) {
        return 1;
    } else if (sbus_val >= SBUS_MAX - TOLERANCE) {
        return 3;
    } else if (sbus_val >= SBUS_MID - TOLERANCE && sbus_val <= SBUS_MID + TOLERANCE) {
        return 2;
    }
    return 0;
}
void SBUS_Init(UART_HandleTypeDef* huart)
{
    sbus_huart = huart;
    // 启动DMA接收，数据目标是dma_rx_buf
    HAL_UARTEx_ReceiveToIdle_DMA(sbus_huart, dma_rx_buf, 25);
}


/**
 * @brief SBUS接收中断开启
 * @note  
 *
*/
void SBUS_Open()
{
	HAL_UARTEx_ReceiveToIdle_DMA(sbus_huart, dma_rx_buf, 25);
}


/**
 * @brief SBUS 接收中断回调处理函数
 * @note  这是一个公共接口，专门给外部中断服务程序调用，用于设置接收完成标志。
 */
void SBUS_RX_Callback_Handler(void)
{
    // 将DMA接收缓冲区的数据安全地复制到解码缓冲区
    memcpy(sbus_decode_buf, dma_rx_buf, 25);

    // 设置接收完成标志
    sbus_rx_completed = 1;
}
void SBUS_Handle(void)
{
    if (sbus_rx_completed)
    {
        sbus_rx_completed = 0; // 清除标志，避免重复处理

        // 计算时间差dt，用于斜坡函数
        static uint32_t last_time = 0;
        uint32_t current_time = HAL_GetTick();
        float dt = (current_time - last_time) / 1000.0f;
        if (dt <= 0.0f || dt > 0.1f) dt = 0.02f; // 防止dt异常, SBUS周期约为14ms
        last_time = current_time;

        // 对解码缓冲区sbus_decode_buf中的安全数据进行解析
        if (sbus_decode_buf[0] == 0x0F && sbus_decode_buf[24] == 0x00)
        {
            // 精确的16通道解析
            sbus_raw.ch[0]  = ((sbus_decode_buf[1]      | sbus_decode_buf[2] << 8) & 0x07FF); //J1（右手左右）
            sbus_raw.ch[1]  = ((sbus_decode_buf[2] >> 3 | sbus_decode_buf[3] << 5) & 0x07FF); //J3（左手上下）
            sbus_raw.ch[2]  = ((sbus_decode_buf[3] >> 6 | sbus_decode_buf[4] << 2 | sbus_decode_buf[5] << 10) & 0x07FF); //J2（右手上下）
            sbus_raw.ch[3]  = ((sbus_decode_buf[5] >> 1 | sbus_decode_buf[6] << 7) & 0x07FF); //J4（左手左右）
            sbus_raw.ch[4]  = ((sbus_decode_buf[6] >> 4 | sbus_decode_buf[7] << 4) & 0x07FF); //SA
            sbus_raw.ch[5]  = ((sbus_decode_buf[7] >> 7 | sbus_decode_buf[8] << 1 | sbus_decode_buf[9] << 9) & 0x07FF); //SB
            sbus_raw.ch[6]  = ((sbus_decode_buf[9] >> 2 | sbus_decode_buf[10] << 6) & 0x07FF); //SC
            sbus_raw.ch[7]  = ((sbus_decode_buf[10] >> 5| sbus_decode_buf[11] << 3) & 0x07FF); //SD
            sbus_raw.ch[8]  = ((sbus_decode_buf[12]     | sbus_decode_buf[13] << 8) & 0x07FF); //SE
            sbus_raw.ch[9]  = ((sbus_decode_buf[13] >> 3| sbus_decode_buf[14] << 5) & 0x07FF); //SF
            sbus_raw.ch[10] = ((sbus_decode_buf[14] >> 6| sbus_decode_buf[15] << 2 | sbus_decode_buf[16] << 10) & 0x07FF);//SG
            sbus_raw.ch[11] = ((sbus_decode_buf[16] >> 1| sbus_decode_buf[17] << 7) & 0x07FF); //SH
            sbus_raw.ch[12] = ((sbus_decode_buf[17] >> 4| sbus_decode_buf[18] << 4) & 0x07FF); //LS
            sbus_raw.ch[13] = ((sbus_decode_buf[18] >> 7| sbus_decode_buf[19] << 1 | sbus_decode_buf[20] << 9) & 0x07FF); //RS
            sbus_raw.ch[14] = ((sbus_decode_buf[20] >> 2| sbus_decode_buf[21] << 6) & 0x07FF); //LO
            sbus_raw.ch[15] = ((sbus_decode_buf[21] >> 5| sbus_decode_buf[22] << 3) & 0x07FF); //RO

            // --- 通道映射与斜坡处理 ---
            // 根据遥感拨杆习惯，将通道映射到具体功能 (ch1-ch4)
            // 右手摇杆上下: Ch2 -> speed
            // 左手摇杆左右: Ch4 -> w_speed
            // 左手摇杆上下: Ch3 -> len
            // 右手摇杆左右: Ch1 -> yaw_speed
            ramp_control.speed_target     = map_sbus_to_range(sbus_raw.ch[1], -SPEED_MAX, SPEED_MAX);
            ramp_control.w_speed_target   = map_sbus_to_range(sbus_raw.ch[3], -W_SPEED_MAX, W_SPEED_MAX);
            ramp_control.len_target       = -map_sbus_to_range(sbus_raw.ch[13], -MAX_LEN, MAX_LEN) + MID_LEN;
            ramp_control.yaw_speed_target = map_sbus_to_range(sbus_raw.ch[0], -YAW_SPEED_MAX, YAW_SPEED_MAX);

            // 应用斜坡处理
            ramp_control.speed_current     = Ramp_Handle(ramp_control.speed_current, ramp_control.speed_target, RAMP_RATE_SPEED, dt);
            ramp_control.w_speed_current   = Ramp_Handle(ramp_control.w_speed_current, ramp_control.w_speed_target, RAMP_RATE_W_SPEED, dt);
            ramp_control.len_current       = Ramp_Handle(ramp_control.len_current, ramp_control.len_target, RAMP_RATE_LEN, dt);
            ramp_control.yaw_speed_current = Ramp_Handle(ramp_control.yaw_speed_current, ramp_control.yaw_speed_target, RAMP_RATE_YAW, dt);

            // 更新最终的外部数据结构
            sbus_rx_data.speed     = ramp_control.speed_current;
            sbus_rx_data.w_speed   = ramp_control.w_speed_current;
            sbus_rx_data.len       = ramp_control.len_current;
            sbus_rx_data.yaw_speed = ramp_control.yaw_speed_current;

            // 处理挡位开关通道 
			sbus_rx_data.jump_flag =  map_to_2levels(sbus_raw.ch[11]);
            sbus_rx_data.status_flag = map_to_3levels(sbus_raw.ch[4]);
			sbus_rx_data.reset_flag = map_to_2levels(sbus_raw.ch[9]);
        }
    }
}




/* Private Function Implementations ------------------------------------------*/

/**
 * @brief 将SBUS原始值映射到指定的输出范围，包含死区处理。
 */
static float map_sbus_to_range(uint16_t sbus_value, float min_out, float max_out)
{
    if (sbus_value < SBUS_MIN) sbus_value = SBUS_MIN;
    if (sbus_value > SBUS_MAX) sbus_value = SBUS_MAX;
    
    if (abs(sbus_value - SBUS_MID) <= DEADZONE_THRESHOLD)
    {
        return 0.0f;
    }

    if (sbus_value > SBUS_MID) 
    {
        uint16_t effective_min = SBUS_MID + DEADZONE_THRESHOLD;
        return (float)(sbus_value - effective_min) / (float)(SBUS_MAX - effective_min) * max_out;
    }
    else
    {
        uint16_t effective_max = SBUS_MID - DEADZONE_THRESHOLD;
        // 注意分母是 (SBUS_MIN - effective_max)，这是一个负数，与min_out的符号抵消
        return (float)(sbus_value - effective_max) / (float)(SBUS_MIN - effective_max) * min_out;
    }
}

/**
 * @brief 斜坡函数，使当前值平滑地趋近于目标值。
 */
static float Ramp_Handle(float current, float target, float ramp_rate, float dt)
{
    float max_change = ramp_rate * dt;
    float error = target - current;

    if (error > max_change)
    {
        return current + max_change;
    }
    else if (error < -max_change)
    {
        return current - max_change;
    }
    else
    {
        return target;
    }
}


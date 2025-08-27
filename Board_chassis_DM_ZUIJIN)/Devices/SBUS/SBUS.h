/**
 * @file SBUS.h（ET16S遥控器）
 * @author ZHY
 * @brief 
 * @version 0.1
 * @date 2025-8-26
 * 
 * @copyright 
 * 
 */

#ifndef __SBUS_H
#define __SBUS_H


#include "main.h"
#include "usart.h"
#include "string.h"
#include "arm_math.h"
#include <stdlib.h> 

#define SBUS_DATA_SIZE      25      // 25字节
#define DEADZONE_THRESHOLD 20.0f  // SBUS值的死区阈值
#define SPEED_MAX 2.0f
#define W_SPEED_MAX 4.0f
#define MID_LEN 0.3f  //腿长中值
#define MAX_LEN 0.1f  //腿长最大变化量
#define YAW_SPEED_MAX 4.0f
 // 斜坡处理参数配置
#define RAMP_RATE_SPEED    2.0f    // 速度斜坡率 (单位/秒)
#define RAMP_RATE_W_SPEED  3.0f    // 角速度斜坡率 (单位/秒)
#define RAMP_RATE_YAW      4.0f    // 偏航速度斜坡率 (单位/秒)
#define RAMP_RATE_LEN      1.0f    // 长度斜坡率 (单位/秒)


struct SBUS_t{
    uint16_t ch[16];                // 16个字节数据
}; 
//接收数据包
struct SBUSRevPack {
 float speed;
float w_speed;
float len;
float yaw_speed;
float jump_flag ;
float stop_flag;
};
//斜坡处理数据包
typedef struct {
    float speed_target;      // 斜坡处理后的目标速度
    float w_speed_target;    // 斜坡处理后的目标角速度
    float yaw_speed_target;  // 斜坡处理后的目标偏航速度
    float len_target;        // 斜坡处理后的目标长度
    
    // 斜坡处理相关变量
    float speed_current;
    float w_speed_current;
    float yaw_speed_current;
    float len_current;
} RampControl_t;

void SBUS_IT_Open(void);
void SBUS_Handle();
float map_sbus_to_range(uint16_t sbus_value, float min_out, float max_out);
float Ramp_Handle(float current, float target, float ramp_rate, float dt);
extern uint8_t sbus_rx_sta;
extern uint8_t sbus_rx_buf[SBUS_DATA_SIZE];
extern struct SBUSRevPack  sbusrev;
extern struct SBUS_t sbus;
extern RampControl_t ramp_control;
#endif
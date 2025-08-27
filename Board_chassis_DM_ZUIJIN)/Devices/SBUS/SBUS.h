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
 

extern uint8_t sbus_rx_sta;
extern uint8_t sbus_rx_buf[SBUS_DATA_SIZE];
extern struct SBUSRevPack  sbusrev;
extern struct SBUS_t sbus;
struct SBUS_t{
    uint16_t ch[16];                // 16个字节数据
}; 
struct SBUSRevPack {
 float speed;
float w_speed;
float len;
float yaw_speed;
float jump_flag ;
float stop_flag;
};


void SBUS_IT_Open(void);
void SBUS_Handle();
float map_sbus_to_range(uint16_t sbus_value, float min_out, float max_out);

#endif
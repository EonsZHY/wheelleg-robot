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


#define SBUS_DATA_SIZE      25      // 25字节

extern uint8_t sbus_rx_sta;
extern uint8_t sbus_rx_buf[SBUS_DATA_SIZE];

struct SBUS_t{
    uint16_t ch[16];                // 16个字节数据
};



void SBUS_IT_Open(void);
void SBUS_Handle();

#endif
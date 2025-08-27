/**
 * @file SBUS.c
 * @author zhy
 * @brief 
 * @version 1.0
 * @date 2025-8-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "SBUS.h"

struct SBUS_t sbus;                     // SBUS 结构体实例化
struct SBUSRevPack  sbusrev;
uint8_t sbus_rx_sta = 0;                // sbus 接收状态，0：未完成，1：已完成一帧接收
uint8_t sbus_rx_buf[SBUS_DATA_SIZE];    // 接收sbus数据缓冲区
// SBUS原始范围：321-1663，中值992
const uint16_t SBUS_MIN = 321;
const uint16_t SBUS_MAX = 1663;
const uint16_t SBUS_MID = 992;
void SBUS_IT_Open(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, (uint8_t *)sbus_rx_buf, SBUS_DATA_SIZE);
}

void SBUS_Handle()
{

		if ((sbus_rx_buf[0] == 0x0F))
		{
			sbus_rx_sta = 1;
		}
		else
		{
			sbus_rx_sta=0;
		}

		if(sbus_rx_sta==1)
        {

            sbus.ch[0] =((sbus_rx_buf[2]<<8)   + (sbus_rx_buf[1])) & 0x07ff;  //右手左右  左：321 中：992  右：1663
            sbus.ch[1] =((sbus_rx_buf[3]<<5)   + (sbus_rx_buf[2]>>3)) & 0x07ff;  //右手上下  上：321  中：992 下：1663
            sbus.ch[2] =((sbus_rx_buf[5]<<10)  + (sbus_rx_buf[4]<<2) + (sbus_rx_buf[3]>>6)) & 0x07ff;  //左手上下  上：1661 中：992 下：321
            sbus.ch[3] =((sbus_rx_buf[6]<<7)   + (sbus_rx_buf[5]>>1)) & 0x07ff;  //左手左右  左：321 中：992 右：1663
            sbus.ch[4] =((sbus_rx_buf[7]<<4)   + (sbus_rx_buf[6]>>4)) & 0x07ff;  //SH  上：1663  下：321
            sbus.ch[5] =((sbus_rx_buf[9]<<9)   + (sbus_rx_buf[8]<<1) + (sbus_rx_buf[7]>>7)) & 0x07ff;  //SD  上：321 中：992  下：1663
            // sbus.ch[6] =((sbus_rx_buf[10]<<6)  + (sbus_rx_buf[9]>>2)) & 0x07ff;  //无法控制  恒为992
            sbus.ch[7] =((sbus_rx_buf[11]<<3)  + (sbus_rx_buf[10]>>5)) & 0x07ff;  //SB 上：321 中：992  下：1663
            sbus.ch[8] =((sbus_rx_buf[13]<<8)  + (sbus_rx_buf[12])) & 0x07ff; //SC 上：65  中：224  下：127

            //未使用通道
            // sbus.ch[9] =((sbus_rx_buf[14]<<5)  + (sbus_rx_buf[13]>>3)) & 0x07ff;
            // sbus.ch[10]=((sbus_rx_buf[16]<<10) + (sbus_rx_buf[15]<<2) + (sbus_rx_buf[14]>>6)) & 0x07ff;
            // sbus.ch[11]=((sbus_rx_buf[17]<<7)  + (sbus_rx_buf[16]>>1)) & 0x07ff;
            // sbus.ch[12]=((sbus_rx_buf[18]<<4)  + (sbus_rx_buf[17]>>4)) & 0x07ff;
            // sbus.ch[13]=((sbus_rx_buf[20]<<9)  + (sbus_rx_buf[19]<<1) + (sbus_rx_buf[18]>>7)) & 0x07ff;
            // sbus.ch[14]=((sbus_rx_buf[21]<<6)  + (sbus_rx_buf[20]>>2)) & 0x07ff;
            // sbus.ch[15]=((sbus_rx_buf[22]<<3)  + (sbus_rx_buf[21]>>5)) & 0x07ff;
        sbusrev.speed = map_sbus_to_range(sbus.ch[1], -SPEED_MAX, SPEED_MAX);
        sbusrev.w_speed = map_sbus_to_range(sbus.ch[0], -W_SPEED_MAX, W_SPEED_MAX);
        sbusrev.yaw_speed = map_sbus_to_range(sbus.ch[3], -YAW_SPEED_MAX, YAW_SPEED_MAX);
        sbusrev.len = map_sbus_to_range(sbus.ch[2], -MAX_LEN, MAX_LEN) + MID_LEN;
        sbusrev.jump_flag = (sbus.ch[4] > 992) ? 1.0f : 0.0f;  // SH通道：大于中值为1，否则为0
        sbusrev.stop_flag = (sbus.ch[5] > 992) ? 1.0f : 0.0f;  // SD通道：大于中值为1，否则为0
            sbus_rx_sta = 0;                        
        }
        memset(sbus_rx_buf, 0, SBUS_DATA_SIZE);
        
}
float map_sbus_to_range(uint16_t sbus_value, float min_out, float max_out)
{
      // 确保输入值在有效范围内
    if (sbus_value < SBUS_MIN) sbus_value = SBUS_MIN;
    if (sbus_value > SBUS_MAX) sbus_value = SBUS_MAX;
    
    // 死区处理
    if (abs(sbus_value - SBUS_MID) <= DEADZONE_THRESHOLD) {
        return 0.0f;  // 在死区内，返回零值
    }
     if (sbus_value > SBUS_MID) 
     {
            // 正半轴映射（调整死区边界）
            uint16_t effective_min = SBUS_MID + DEADZONE_THRESHOLD;
            return (float)(sbus_value - effective_min) / (SBUS_MAX - effective_min) * max_out;
    }
    else {
            // 负半轴映射（调整死区边界）
            uint16_t effective_max = SBUS_MID - DEADZONE_THRESHOLD;
            return (float)(sbus_value - effective_max) / (SBUS_MIN - effective_max) * min_out;
        }
}
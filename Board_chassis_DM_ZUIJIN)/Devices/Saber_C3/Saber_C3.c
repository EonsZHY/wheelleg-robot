/**
 * @file Saber_C3.c
 * @author lxr(784457420@qq.com)
 * @brief 
 * @version 1.0
 * @date 2023-8-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */
 
#include "Saber_C3.h"
#include "string.h"
 
uint8_t Saber_Rxbuffer[Saber_Data_Length];
uint8_t Saber_Montage[Saber_Data_Buffer];
Saber_Angle_t Saber_Angle;
 
 /**
  * @brief 十六进制转float
  */
static float R4(uint8_t *p) {float r; memcpy(&r, p, 4); return r;}
 /**
  * @brief          IMU初始化任务，打开串口
  * @retval         none
  */
 void Saber_Init(void)
 {
	 HAL_UARTEx_ReceiveToIdle_DMA(&huart7,Saber_Rxbuffer,sizeof(Saber_Rxbuffer));
 }
 
/**
  * @brief          帧头帧尾判断
  * @parameter[in]  接收数据首地址
  * @retval         检查符合协议返回1，否则返回0。
  */
 uint8_t Saber_Cheak(uint8_t *p)
{
	/* Header用于遍历整个数组，Last用于标记遍历的开始以及结束位置 */
	static uint8_t Header_Position = 0;
	static uint8_t Last_Position;
	Last_Position = Header_Position;
	do
	{
		if(p[Header_Position] == 0x41 && p[(Header_Position+1) % Saber_Data_Buffer] == 0x78 \
			&& p[(Header_Position+2) % Saber_Data_Buffer] == 0xFF && p[(Header_Position+3) \
			% Saber_Data_Buffer]== 0x06 && p[(Header_Position + Frame_End) % Saber_Data_Buffer] == 0X6D)
		{
			return Header_Position;
		}
		Header_Position++;
		if(Header_Position >= Saber_Data_Buffer)
			Header_Position = 0;
	}while(Header_Position!=Last_Position);
	
	return Saber_Data_Buffer; //判读时就看返回值是不是小于Saber_Data_Buffer就好
}
 
 /**
  * @brief          IMU欧拉角读取任务
  * @retval         none
  */
void Saber_Read(void)
{
	static uint8_t Header_Position;
	Header_Position = Saber_Cheak(Saber_Montage);
	if(Header_Position < Saber_Data_Buffer)             //排除异常值
	{
			Saber_Angle.Acc_x = R4(&Saber_Rxbuffer[0 + 9]);
		Saber_Angle.Acc_y = R4(&Saber_Rxbuffer[0 + 13]);
		Saber_Angle.Acc_z = R4(&Saber_Rxbuffer[0 + 17]);
		
		Saber_Angle.Gyro_x = R4(&Saber_Rxbuffer[0 + 24]);
		Saber_Angle.Gyro_y = R4(&Saber_Rxbuffer[0 + 28]);
		Saber_Angle.Gyro_z = R4(&Saber_Rxbuffer[0 + 32]);
		
		Saber_Angle.q1 = R4(&Saber_Rxbuffer[0 + 39]);
		Saber_Angle.q2= R4(&Saber_Rxbuffer[0 + 43]);
		Saber_Angle.q3 = R4(&Saber_Rxbuffer[0 + 47]);
		Saber_Angle.q4 = R4(&Saber_Rxbuffer[0 + 51]);
		
		Saber_Angle.RoLL = R4(&Saber_Rxbuffer[0 + 62]);
		Saber_Angle.Pitch = R4(&Saber_Rxbuffer[0 + 58]);
		Saber_Angle.Yaw  = R4(&Saber_Rxbuffer[0 + 66]);
	//	memset(Saber_Rxbuffer,0,sizeof(Saber_Rxbuffer));
	}

}

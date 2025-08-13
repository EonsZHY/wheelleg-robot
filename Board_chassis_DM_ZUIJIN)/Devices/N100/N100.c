/**
 * @file N100.c
 * @author zhy
 * @brief 
 * @version 1.0
 * @date 2025-8-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "N100.h"
#include "string.h"


IMUData_Packet_t IMUData_Packet;
AHRSData_Packet_t AHRSData_Packet;

//串口接收标志
int Flag_Ahrs = 0;
int Flag_Imu = 0;

//数据处理标志
int Handle_Ahrs = 0;
int Handle_Imu = 0;

uint8_t N100_Rxbuffer;
uint8_t N100_ReImu[IMU_RS];
uint8_t N100_ReAhrs[AHRS_RS];
uint8_t N100_tmpData[IMU_RS];
uint8_t Count = 0;
uint8_t last_rsnum = 0;
/**
  * @brief          IMU初始化任务，打开串口
  * @retval         none
  */
void N100_Init()
{
    
    // 清零缓冲区
    memset(N100_ReImu, 0, sizeof(N100_ReImu));
    memset(N100_ReAhrs, 0, sizeof(N100_ReAhrs));
	memset(N100_tmpData, 0, sizeof(N100_tmpData));
    
    // 启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, &N100_Rxbuffer, sizeof(N100_Rxbuffer));
    
}


 /**
  * @brief          IMU欧拉角读取任务
  * @retval         none
  */
void N100_Read()
{
    if(Handle_Ahrs==1)
	{
		if(N100_ReAhrs[1]==TYPE_AHRS&&N100_ReAhrs[2]==AHRS_LEN)
		{	
		AHRSData_Packet.RollSpeed=DATA_Trans(N100_ReAhrs[7],N100_ReAhrs[8],N100_ReAhrs[9],N100_ReAhrs[10]);       //横滚角速度
		AHRSData_Packet.PitchSpeed=DATA_Trans(N100_ReAhrs[11],N100_ReAhrs[12],N100_ReAhrs[13],N100_ReAhrs[14]);   //俯仰角速度
		AHRSData_Packet.HeadingSpeed=DATA_Trans(N100_ReAhrs[15],N100_ReAhrs[16],N100_ReAhrs[17],N100_ReAhrs[18]); //偏航角速度
			
        AHRSData_Packet.Roll=DATA_Trans(N100_ReAhrs[19],N100_ReAhrs[20],N100_ReAhrs[21],N100_ReAhrs[22]);      //横滚角
		AHRSData_Packet.Pitch=DATA_Trans(N100_ReAhrs[23],N100_ReAhrs[24],N100_ReAhrs[25],N100_ReAhrs[26]);     //俯仰角
		AHRSData_Packet.Heading=DATA_Trans(N100_ReAhrs[27],N100_ReAhrs[28],N100_ReAhrs[29],N100_ReAhrs[30]);;	 //偏航角
			
		AHRSData_Packet.Qw=DATA_Trans(N100_ReAhrs[31],N100_ReAhrs[32],N100_ReAhrs[33],N100_ReAhrs[34]);;  //四元数
		AHRSData_Packet.Qx=DATA_Trans(N100_ReAhrs[35],N100_ReAhrs[36],N100_ReAhrs[37],N100_ReAhrs[38]);;
		AHRSData_Packet.Qy=DATA_Trans(N100_ReAhrs[39],N100_ReAhrs[40],N100_ReAhrs[41],N100_ReAhrs[42]);;
		AHRSData_Packet.Qz=DATA_Trans(N100_ReAhrs[43],N100_ReAhrs[44],N100_ReAhrs[45],N100_ReAhrs[46]);;
		AHRSData_Packet.Timestamp=timestamp(N100_ReAhrs[47],N100_ReAhrs[48],N100_ReAhrs[49],N100_ReAhrs[50],N100_ReAhrs[51],N100_ReAhrs[52],N100_ReAhrs[53],N100_ReAhrs[54]);   //时间戳
		}
	Handle_Ahrs=0;
 }
	if(Handle_Imu==1)
	{
		if(N100_ReImu[1]==TYPE_IMU&&N100_ReImu[2]==IMU_LEN)
		{
		IMUData_Packet.gyroscope_x=DATA_Trans(N100_ReImu[7],N100_ReImu[8],N100_ReImu[9],N100_ReImu[10]);  //角速度
		IMUData_Packet.gyroscope_y=DATA_Trans(N100_ReImu[11],N100_ReImu[12],N100_ReImu[13],N100_ReImu[14]);
		IMUData_Packet.gyroscope_z=DATA_Trans(N100_ReImu[15],N100_ReImu[16],N100_ReImu[17],N100_ReImu[18]);
			
		IMUData_Packet.accelerometer_x=DATA_Trans(N100_ReImu[19],N100_ReImu[20],N100_ReImu[21],N100_ReImu[22]);  //线加速度
		IMUData_Packet.accelerometer_y=DATA_Trans(N100_ReImu[23],N100_ReImu[24],N100_ReImu[25],N100_ReImu[26]);
		IMUData_Packet.accelerometer_z=DATA_Trans(N100_ReImu[27],N100_ReImu[28],N100_ReImu[29],N100_ReImu[30]);

		IMUData_Packet.magnetometer_x=DATA_Trans(N100_ReImu[31],N100_ReImu[32],N100_ReImu[33],N100_ReImu[34]);  //磁力计数据
		IMUData_Packet.magnetometer_y=DATA_Trans(N100_ReImu[35],N100_ReImu[36],N100_ReImu[37],N100_ReImu[38]);
		IMUData_Packet.magnetometer_z=DATA_Trans(N100_ReImu[39],N100_ReImu[40],N100_ReImu[41],N100_ReImu[42]);
			
		IMUData_Packet.Timestamp=timestamp(N100_ReImu[55],N100_ReImu[56],N100_ReImu[57],N100_ReImu[58],N100_ReImu[59],N100_ReImu[60],N100_ReImu[61],N100_ReImu[62]);   //时间戳
		}
		Handle_Imu=0;
 }
 //HAL_UARTEx_ReceiveToIdle_DMA(&huart10, &N100_Rxbuffer, sizeof(N100_Rxbuffer));
}

/**
  * @brief       实现16进制的数据转化为float   
  * @retval         
  */
float DATA_Trans(uint8_t Data_1,uint8_t Data_2,uint8_t Data_3,uint8_t Data_4)
{
  uint32_t transition_32;
	float tmp=0;
	int sign=0;
	int exponent=0;
	float mantissa=0;
  transition_32 = 0;
  transition_32 |=  Data_4<<24;   
  transition_32 |=  Data_3<<16; 
	transition_32 |=  Data_2<<8;
	transition_32 |=  Data_1;
  sign = (transition_32 & 0x80000000) ? -1 : 1;//符号位
	//先右移操作，再按位与计算，出来结果是30到23位对应的e
	exponent = ((transition_32 >> 23) & 0xff) - 127;
	//将22~0转化为10进制，得到对应的x系数 
	mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
	tmp=sign * mantissa * pow(2, exponent);
	return tmp;
}


long long timestamp(uint8_t Data_1,uint8_t Data_2,uint8_t Data_3,uint8_t Data_4,uint8_t Data_5,uint8_t Data_6,uint8_t Data_7,uint8_t Data_8)
{
  long long transition_64;
  transition_64 = 0;
  transition_64 |=  Data_8<<56;   
  transition_64 |=  Data_7<<48; 
	transition_64 |=  Data_6<<40;
	transition_64 |=  Data_5<<32;	
  transition_64 |=  Data_4<<24;   
  transition_64 |=  Data_3<<16; 
	transition_64 |=  Data_2<<8;
	transition_64 |=  Data_1;
	return transition_64;
}
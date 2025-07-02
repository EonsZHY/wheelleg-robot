/**
 * @file M1505B_Motor.c
 * @author ZHY
 * @brief
 * @version 0.1
 * @date 2025-5-20
 *
 * @copyright
 *
 */

#include "M3508.h"
#include <stdio.h>

static int16_t FloatDeadband(int16_t _value,int16_t _min_value, int16_t _max_value)
{
    if (_value < _max_value && _value > _min_value) {
        _value = 0;
    }
    return _value;
}

M3508_t M3508_Array[TotalNum] = {
    [Wheel_L] 		  = {0},
    [Wheel_R] 		  = {0},
};

/*接收数据数组*/
uint8_t M3508_Data_Rx[8];
/*发送数据数组*/
uint8_t M3508_Data_Tx[8];


//void M3508_Enable(void);
//void M3508_Disable(void);
//void M3508_SetIDL(void);
//void M3508_SetIDR(void);
void M3508_setCurrent(void);
//void M3508_setSeq(void);
void M3508_getInfo(FDCan_Export_Data_t RxMessage);
void M3508_SetTor(void);
void M3508_Init(void);

M3508_FUN_t M3508_FUN = M3508_FunGroundInit;
#undef M3508_FunGroundInit

/**
  * @brief  设置M1505B电机电流值（id号为1~4）
  * @param  CtrlDatax (x:1~4) 对应id号电机的电流值，范围-16383~0~16383
  * @retval None
  */
 void M3508_setCurrent()
 {
    M3508_Data_Tx[0] = M3508_Array[0].sendCurrent >> 8;
    M3508_Data_Tx[1] = M3508_Array[0].sendCurrent;
    M3508_Data_Tx[2] = M3508_Array[1].sendCurrent >> 8;
    M3508_Data_Tx[3] = M3508_Array[1].sendCurrent ;
    Can_Fun.fdcanx_send_data(&hfdcan3, M3508_SENDID_Chassis,M3508_Data_Tx, 8);
 }

/**
  * @brief  从CAN报文中获取M1505B电机信息
  * @param  RxMessage 	CAN报文接收结构体
  * @retval None
  */
void M3508_getInfo(FDCan_Export_Data_t RxMessage)
{
    uint32_t StdId;
    StdId = (int32_t)(RxMessage.fdcan_RxHeader.Identifier- M3508_READID_START);
    //解包数据，数据格式见手册
    M3508_Array[StdId].circleAngle= (int16_t)((RxMessage.FDCANx_Export_RxMessage[0] << 8 | RxMessage.FDCANx_Export_RxMessage[1]));
    M3508_Array[StdId].receiveSpeed = (int16_t)(RxMessage.FDCANx_Export_RxMessage[2] << 8 | RxMessage.FDCANx_Export_RxMessage[3]);
    M3508_Array[StdId].receiveTorqueI = (int16_t)(RxMessage.FDCANx_Export_RxMessage[4] << 8 | RxMessage.FDCANx_Export_RxMessage[5]);
    M3508_Array[StdId].temperture = RxMessage.FDCANx_Export_RxMessage[6];
    M3508_Array[StdId].realTorqueI =  (float)M3508_Array[StdId].receiveTorqueI/M3508_CURRENT_RAW_MAX* M3508_CURRENT_MAX_A;
    M3508_Array[StdId].realSpeed = (float)M3508_Array[StdId].receiveSpeed * 2.0f * 3.1415926f / (60.0f * gear_ratio);
    if (M3508_Array[StdId].circleAngle - M3508_Array[StdId].lastAngle < -6000)
    {
        M3508_Array[StdId].turnCount++;
    }
    else if (M3508_Array[StdId].lastAngle - M3508_Array[StdId].circleAngle < -6000)
    {
        M3508_Array[StdId].turnCount--;
    }
    M3508_Array[StdId].totalAngle = M3508_Array[StdId].circleAngle + (8192 * M3508_Array[StdId].turnCount);
    M3508_Array[StdId].lastAngle = M3508_Array[StdId].circleAngle;

    //帧率统计，数据更新标志位
    M3508_Array[StdId].InfoUpdateFrame++;
    M3508_Array[StdId].InfoUpdateFlag = 1;
}
/**
* @brief  对电机参数进行初始化
  * @param  None
  * @retval None
  * @attention 当前只针对其初始输出电流进行了清零，以后有更多需求需改写
  */
void M3508_Init(void)
{
    for(int i = 0 ; i <= 1 ; i++ )
    {
        M3508_Array[i].sendCurrent = 0;
    }
}
void M3508_SetTor(void)
{
	uint32_t StdId;
	for(StdId=0; StdId<=1; StdId++)
	{
		M3508_Array[StdId].targetTorqueI=(float)(M3508_Array[StdId].targetTorque/(KT*gear_ratio));
		M3508_Array[StdId].targetCurrent=	M3508_Array[StdId].targetTorqueI+T_Update(&M3508_Array[StdId].TPID, M3508_Array[StdId].realTorqueI, M3508_Array[StdId].targetTorqueI);
		M3508_Array[StdId].sendCurrent= (int16_t)(M3508_Array[StdId].targetCurrent/ M3508_CURRENT_MAX_A*M3508_CURRENT_RAW_MAX);	
        if (M3508_Array[StdId].sendCurrent > M3508_CURRENT_RAW_MAX) M3508_Array[StdId].sendCurrent = M3508_CURRENT_RAW_MAX;
        if (M3508_Array[StdId].sendCurrent < -M3508_CURRENT_RAW_MAX) M3508_Array[StdId].sendCurrent = -M3508_CURRENT_RAW_MAX;
	
	}
}

/**
 * @file M2006.cpp
 * @author ZHY
 * @brief 大疆2006电机配置与操作
 * @version 
 * @date 2025-8-30
 *
 * @copyright 
 *
 */


#include "M2006.h"
#include <stdio.h>


FDCAN_HandleTypeDef *Class_M2006::Can_Motor = &hfdcan3; //绑定CAN总线

/**
 * @brief 2006电机初始化
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @param __Control_Method 电机控制方式, 默认速度
 * @param __Gearbox_Rate 减速箱减速比, 默认为原装减速箱, 如拆去减速箱则该值设为1
 * @param __Torque_Max 最大扭矩, 需根据不同负载测量后赋值
 */
void Class_M2006::Init(Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method, float __Gearbox_Rate, float __Current_Max)
{
    CAN_ID = __CAN_ID;
    Control_Method = __Control_Method;
    Gearbox_Rate = __Gearbox_Rate;
    Current_Max = __Current_Max;
    CAN_Tx_Data = allocate_tx_data(Can_Motor, __CAN_ID);
}
/**
  * @brief  设置M2006电机电流值（id号为7）M2006与6623共用发送函数
  * @param  iqx (x:5) 对应id号电机的电流值，范围-10000~0~10000
  * @retval None
  */
 void Class_M2006::SetCurrent(void)
 {
    Can_Fun.fdcanx_send_data(Can_Motor,M2006_SENDID,CAN3_0x200_Tx_Data, 8);
 }
 /**
 * @brief 电机数据输出到CAN总线发送缓冲区
 *
 */
void Class_M2006::Output()
{
    CAN_Tx_Data[0] = (int16_t)outCurrent >> 8;
    CAN_Tx_Data[1] = (int16_t)outCurrent;
}
/**
 * @brief 设定输出量
 *
 * @param __Output_Voltage 输出量
 */
void Class_M2006::Set_Out(int16_t __Out)
{
   outCurrent = __Out;
}
/**
  * @brief  从CAN报文中获取M2006电机信息
  * @param  RxMessage 	CAN报文接收结构体
  * @retval None
  */

void Class_M2006::FDCAN_RxCpltCallback(FDCan_Export_Data_t RxMessage)
{

    lastAngle = realAngle;
    realAngle = (uint16_t)(RxMessage.FDCANx_Export_RxMessage[0] << 8 | RxMessage.FDCANx_Export_RxMessage[1]);
    realSpeed = (int16_t)(RxMessage.FDCANx_Export_RxMessage[2] << 8 | RxMessage.FDCANx_Export_RxMessage[3]);
    realTorque = (int16_t)(RxMessage.FDCANx_Export_RxMessage[4] << 8 | RxMessage.FDCANx_Export_RxMessage[5]);
    temperture = RxMessage.FDCANx_Export_RxMessage[6];

    if (realAngle - lastAngle < -6000)
    {
        turnCount++;
    }

    if (lastAngle - realAngle < -6000)
    {
        turnCount--;
    }

    totalAngle = realAngle + (8192 * turnCount);
    lastAngle = realAngle;
    //帧率统计，数据更新标志位
    InfoUpdateFrame++;
    InfoUpdateFlag = 1;

}


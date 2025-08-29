/**
 * @file GM6020.cpp
 * @author ZHY
 * @brief 大疆6020电机配置与操作
 * @version 
 * @date 2025-8-18
 *
 * @copyright 
 *
 */

#include "dji_motor.h"
#include "GM6020.h"
#include <stdio.h>


FDCAN_HandleTypeDef *Class_GM6020::Can_Motor = &hfdcan3; //绑定CAN总线
/**
 * @brief 6020电机初始化
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @param __Control_Method 电机控制方式, 默认速度
 * @param __Gearbox_Rate 减速箱减速比, 默认为原装减速箱, 如拆去减速箱则该值设为1
 * @param __Torque_Max 最大扭矩, 需根据不同负载测量后赋值
 */
void Class_GM6020::Init(Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method, float __Gearbox_Rate, float __Voltage_Max)
{
    CAN_ID = __CAN_ID;
    Control_Method = __Control_Method;
    Gearbox_Rate = __Gearbox_Rate;
    Voltage_Max = __Voltage_Max;
    CAN_Tx_Data = allocate_tx_data(Can_Motor, __CAN_ID);
}

///**
// * @brief 6020电机PID初始化
// *
// * @param hcan CAN编号
// */
//void Class_GM6020::PID_Init()
//{
//    Position_PIDInit(&M6020s_Yaw.Aim_position_PID, 1300.0f, 70.0f, 900.0f, 300.0f, 30000, 10000);
//	Position_PIDInit(&M6020s_Yaw.position_PID, 5.0f, 0.0f, 3.0f, 0.0f, 30000, 10000);
//	Position_PIDInit(&M6020s_Yaw.velocity_PID, 50.0f, 0.0f, 1.5f, 0.0f, 30000, 10000);
//}
/**
 * @brief 6020电机PID初始化
 *
 * @param hcan CAN编号
 */
/**
  * @brief  设置M6020电机电压（id号为1~4）
  * @param  电压值范围 -30000~0~30000
  * @retval None
  */
 void Class_GM6020::SetVoltage(void)
 {
    Can_Fun.fdcanx_send_data(Can_Motor,M6020_SENDID,CAN3_0x200_Tx_Data, 8);
 }

 /**
 * @brief 电机数据输出到CAN总线发送缓冲区
 *
 */
void Class_GM6020::Output()
{
    CAN_Tx_Data[0] = (int16_t)Out >> 8;
    CAN_Tx_Data[1] = (int16_t)Out;
}

/**
 * @brief 设定输出量
 *
 * @param __Output_Voltage 输出量
 */
void Class_GM6020::Set_Out(int16_t __Out)
{
   outCurrent = __Out;
}

/**
 * @brief 设定目标角度
 *
 * @param 
 */
void Class_GM6020::SetTargetAngle(int32_t angle)
{
    targetAngle = angle;
}

/**
  * @brief  M6020_Reset
  * @param  
  * @retval None
  * 说明：调运此函数以解决totalAngle 等溢出的问题。
  */
void Class_GM6020::Reset(void)
{
    lastAngle = realAngle;
    totalAngle = realAngle;
    turnCount = 0;
}

/**
  * @brief  从CAN报文中获取M6020电机信息
  * @param  RxMessage 	CAN报文接收结构体
  * @retval None
  */

void Class_GM6020::FDCAN_RxCpltCallback(FDCan_Export_Data_t RxMessage)
{

    lastAngle = realAngle;
    realAngle = (uint16_t)(RxMessage.FDCANx_Export_RxMessage[0] << 8 | RxMessage.FDCANx_Export_RxMessage[1]);
    realSpeed = (int16_t)(RxMessage.FDCANx_Export_RxMessage[2] << 8 | RxMessage.FDCANx_Export_RxMessage[3]);
    realCurrent = (int16_t)(RxMessage.FDCANx_Export_RxMessage[4] << 8 | RxMessage.FDCANx_Export_RxMessage[5]);
    temperture = RxMessage.FDCANx_Export_RxMessage[6];

    if (realAngle - lastAngle < -6500)
    {
        turnCount++;
    }

    if (lastAngle - realAngle < -6500)
    {
        turnCount--;
    }

    totalAngle = realAngle + (8192 * turnCount);
    //帧率统计，数据更新标志位
    InfoUpdateFrame++;
    InfoUpdateFlag = 1;

}
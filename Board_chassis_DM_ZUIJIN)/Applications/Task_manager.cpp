#include "Task_manager.h"
#include "Chassis.h"
#include "bsp_dwt.h"
#include "Saber_C3.h"
#include "unitree.h"
#include "ins.h"
#include "board_comm.h"
#include "M3508.h"
#include "vofa.h"
/**DEBUG**/
float joint_pos;
/**DEBUG**/

/**
 * @brief 模组初始化
 * @param 
 * @param 
 * @param
 */
void motor_init()
{
	chassis.MotorInit();
	chassis.PidInit();
chassis.SpeedEstInit();
}

// /**
//  * @brief 宇树电机控制指令下发
//  * @param 
//  * @param 
//  * @param
//  */
// void UnitreeMotorTask()
// {
// 	chassis.lf_joint_.DM_8009P_Ctrl();
// 	chassis.rf_joint_.DM_8009P_Ctrl();
// 	DWT_Delay(0.002f);
// //	
// 	chassis.lb_joint_.DM_8009P_Ctrl();
// chassis.rb_joint_.DM_8009P_Ctrl();

// }

/**
 * @brief 达妙电机控制指令下发
 * @param 
 * @param 
 * @param
 */
void DM_MotorTask()
{

	chassis.lf_joint_.DM_8009P_Ctrl();
	DWT_Delay(0.001f);
	chassis.rf_joint_.DM_8009P_Ctrl();
	DWT_Delay(0.001f);
	chassis.lb_joint_.DM_8009P_Ctrl();
	DWT_Delay(0.001f);
	chassis.rb_joint_.DM_8009P_Ctrl();
	DWT_Delay(0.001f);

}







/**
 * @brief 轮毂电机控制指令下发
 * @param 
 * @param 
 * @param
 */
void WheelMotorTask() {
	M3508_FUN.M3508_setCurrent();
}

void YawMotorTask()
{
	M6020_Fun.M6020_setVoltage();
}

/**
 * @brief 主控制计算循环
 * @param 
 * @param 
 * @param
 */
void ChassisCalcTask() {
//	if(INS.Yaw!=0)
//	{
//		chassis.Controller();
//	}
	chassis.Controller();
}

/**
 * @brief 板间通讯
 * @param 
 * @param 
 * @param
 */
void boardCommunicateTask()
{
	board_comm.send_.yaw_real_position=(int16_t)((M6020s_Yaw.realAngle+Saber_Angle.Yaw/360*8192)/2-2048) ;
	board_comm.send_.bullet_speed=25;
	BoardCommSend(&board_comm);
}
/**
* @brief 上位机查看数据
 * @param 
 * @param 
 * @param
 */
void vofaTask()
{
	 Vofa.data[0] = INS.Roll;
     Vofa.data[1] = INS.Pitch;
	 Vofa.data[2] = INS.Yaw;
	
}


///**
// * @brief 遥控器指令接收
// * @param 
// * @param 
// * @param
// */
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//	
//		static uint8_t Saber_Montage_Flag = 0;            //表示陀螺仪数据的拼接起点
//	
//	//如果数据来自USART1,即为IMU数据
//	if(huart->Instance == UART7)
//	{
//		if(Saber_Montage_Flag)
//		{
//			memcpy(Saber_Montage + Saber_Data_Length, Saber_Rxbuffer, Saber_Data_Length);
//			Saber_Montage_Flag = 0;
//		}
//		else
//		{
//			memcpy(Saber_Montage, Saber_Rxbuffer, Saber_Data_Length);
//			Saber_Montage_Flag = 1;
//		}

//		HAL_UARTEx_ReceiveToIdle_DMA(&huart7,Saber_Rxbuffer,sizeof(Saber_Rxbuffer));
//	}
	
//	if(huart->Instance == USART2)
//	{
//	
//		if(temprecvl.head.motorID==0x00)
//		{
//			if (temprecvl.head.start[0] == 0xFE &&(uint8_t)temprecvl.head.start[1] == 0xEE)
//			{
//			
//    chassis.lf_joint_.motor_recv_.motor_id = temprecvl.head.motorID;
//    chassis.lf_joint_.motor_recv_.mode = temprecvl.Mdata.mode;
//    chassis.lf_joint_.motor_recv_.Temp = temprecvl.Mdata.Temp;
//    chassis.lf_joint_.motor_recv_.MError = temprecvl.Mdata.MError;
//    chassis.lf_joint_.motor_recv_.W = ((float)temprecvl.Mdata.W / 128) * 0.95f +
//                    chassis.lf_joint_.motor_recv_.W * 0.05f;
//    chassis.lf_joint_.motor_recv_.T = (float)temprecvl.Mdata.T / 256;
//    chassis.lf_joint_.motor_recv_.Pos =
//        6.2832f * ((float)temprecvl.Mdata.Pos) / 16384;
//  HAL_UARTEx_ReceiveToIdle_DMA(&huart2,(uint8_t*)&temprecvl,sizeof(temprecvl));
//			}

//  }
//		
//		 if(temprecvl.head.motorID==0x02)
//		{
//	if (temprecvl.head.start[0] == 0xFE &&temprecvl.head.start[1] == 0xEE)
//			{
//			
//    chassis.lb_joint_.motor_recv_.motor_id = temprecvl.head.motorID;
//    chassis.lb_joint_.motor_recv_.mode = temprecvl.Mdata.mode;
//    chassis.lb_joint_.motor_recv_.Temp = temprecvl.Mdata.Temp;
//    chassis.lb_joint_.motor_recv_.MError = temprecvl.Mdata.MError;
//    chassis.lb_joint_.motor_recv_.W = ((float)temprecvl.Mdata.W / 128) * 0.95f +
//                    chassis.lb_joint_.motor_recv_.W * 0.05f;
//    chassis.lb_joint_.motor_recv_.T = (float)temprecvl.Mdata.T / 256;
//    chassis.lb_joint_.motor_recv_.Pos =
//        6.2832f * ((float)temprecvl.Mdata.Pos) / 16384;
//  HAL_UARTEx_ReceiveToIdle_DMA(&huart2,(uint8_t*)&temprecvl,sizeof(temprecvl));
//			}
//		}
//		
//	}
//	if(huart->Instance == USART3)
//	{
//	
//		if(temprecvr.head.motorID==0x00)
//		{
//			if (temprecvr.head.start[0] == 0xFE &&temprecvr.head.start[1] == 0xEE)
//			{
//				
//    chassis.rf_joint_.motor_recv_.motor_id = temprecvr.head.motorID;
//    chassis.rf_joint_.motor_recv_.mode = temprecvr.Mdata.mode;
//    chassis.rf_joint_.motor_recv_.Temp = temprecvr.Mdata.Temp;
//    chassis.rf_joint_.motor_recv_.MError = temprecvr.Mdata.MError;
//    chassis.rf_joint_.motor_recv_.W = ((float)temprecvr.Mdata.W / 128) * 0.95f +
//                    chassis.rf_joint_.motor_recv_.W * 0.05f;
//    chassis.rf_joint_.motor_recv_.T = (float)temprecvr.Mdata.T / 256;
//    chassis.rf_joint_.motor_recv_.Pos =
//        6.2832f * ((float)temprecvr.Mdata.Pos) / 16384;
//  
//			}
//			HAL_UARTEx_ReceiveToIdle_DMA(&huart3,(uint8_t*)&temprecvr,sizeof(temprecvr));
//  }
//		
//		if(temprecvr.head.motorID==0x02)
//		{
//	if (temprecvr.head.start[0] == 0xFE &&temprecvr.head.start[1] == 0xEE)
//			{
//				
//    chassis.rb_joint_.motor_recv_.motor_id = temprecvr.head.motorID;
//    chassis.rb_joint_.motor_recv_.mode = temprecvr.Mdata.mode;
//    chassis.rb_joint_.motor_recv_.Temp = temprecvr.Mdata.Temp;
//    chassis.rb_joint_.motor_recv_.MError = temprecvr.Mdata.MError;
//    chassis.rb_joint_.motor_recv_.W = ((float)temprecvr.Mdata.W / 128) * 0.95f +
//                    chassis.rb_joint_.motor_recv_.W * 0.05f;
//    chassis.rb_joint_.motor_recv_.T = (float)temprecvr.Mdata.T / 256;
//    chassis.rb_joint_.motor_recv_.Pos =
//        6.2832f * ((float)temprecvr.Mdata.Pos) / 16384;
//  }
//			HAL_UARTEx_ReceiveToIdle_DMA(&huart3,(uint8_t*)&temprecvr,sizeof(temprecvr));
//		}
//		
//	}

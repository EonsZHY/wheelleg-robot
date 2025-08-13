/* Includes ------------------------------------------------------------------*/
#include "bsp_uart.h"
#include <string.h>
#include "stdlib.h"
#include "Saber_C3.h"
#include "N100.h"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t idx;
static UartInstance *uart_instance[5] = {NULL};
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief Registers a UART instance with the specified initialization configuration.
 *
 * This function is used to register a UART instance with the provided initialization configuration.
 *
 * @param _config The initialization configuration for the UART instance.
 * @return A pointer to the registered UART instance.
 */
UartInstance *pUartRegister(UartInitConfig *_pconfig)
{
    if (idx >= 5) {
        while (1) {
        }
    }
    UartInstance *instance = (UartInstance *)malloc(sizeof(UartInstance));
    memset(instance, 0, sizeof(UartInstance));

    instance->huart = _pconfig->huart;
    instance->rx_buffer_size = _pconfig->rx_buffer_size;
    instance->callback_function = _pconfig->callback_function;

    uart_instance[idx++] = instance;
    UartInit(instance);
    return instance;
}

/**
 * @brief Initializes the UART instance.
 *
 * This function initializes the UART instance specified by `_instance`.
 *
 * @param _instance The UART instance to be initialized.
 */
void UartInit(UartInstance *_pinstance)
{
    HAL_UARTEx_ReceiveToIdle_DMA(_pinstance->huart, _pinstance->rx_buffer, _pinstance->rx_buffer_size);
}

/**
 * @brief Sends data over UART.
 *
 * This function sends data over UART using the specified UART handle, send buffer, send size, and transmit mode.
 *
 * @param _huart The UART handle.
 * @param _send_buf The send buffer containing the data to be sent.
 * @param _send_size The size of the data to be sent.
 * @param _mode The transmit mode.
 */
void UartSendData(UART_HandleTypeDef *_phuart, uint8_t *_psend_buf, uint16_t _send_size, UART_TRANSMIT_MODE _mode)
{
    switch (_mode) {
        case UART_TRAMSMIT_BLOCKING:
            HAL_UART_Transmit(_phuart, _psend_buf, _send_size, 10);
            break;
        case UART_TRANSMIT_IT:
            HAL_UART_Transmit_IT(_phuart, _psend_buf, _send_size);
            break;
        case UART_TRAMSMIT_DMA:
            HAL_UART_Transmit_DMA(_phuart, _psend_buf, _send_size);
            break;
        default:
            break;
    }
}

/**
 * @brief Callback function for UART receive event.
 *
 * This function is called when a UART receive event occurs.
 *
 * @param huart Pointer to the UART handle.
 * @param Size Number of bytes received.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	
		static uint8_t Saber_Montage_Flag = 0;            //表示陀螺仪数据的拼接起点
	
	//如果数据来自USART1,即为IMU数据
	if(huart->Instance == UART7)
	{
	// 	if(Saber_Montage_Flag)
	// 	{
	// 		memcpy(Saber_Montage + Saber_Data_Length, Saber_Rxbuffer, Saber_Data_Length);
	// 		Saber_Montage_Flag = 0;
	// 	}
	// 	else
	// 	{
	// 		memcpy(Saber_Montage, Saber_Rxbuffer, Saber_Data_Length);
	// 		Saber_Montage_Flag = 1;
	// 	}
       
        N100_tmpData[Count] = N100_Rxbuffer;  // 将串口数据填入数组
        if (((last_rsnum == FRAME_TAIL) && (N100_Rxbuffer == FRAME_HEAD)) || Count > 0) {
            Count++;
            if ((N100_tmpData[1] == TYPE_IMU) && (N100_tmpData[2] == IMU_LEN)) {
                 Flag_Imu= 1;
            }
            if ((N100_tmpData[1] == TYPE_AHRS) && (N100_tmpData[2] == AHRS_LEN)) {
                Flag_Ahrs = 1;
            }
        } else {
            Count = 0;
        }
        last_rsnum = N100_Rxbuffer;

        if (Flag_Imu == 1 && Count == IMU_RS) {  // 保存 IMU 数据
            Count = 0;
            Flag_Imu = 0;
            Handle_Imu = 1;
            if (N100_tmpData[IMU_RS - 1] == FRAME_TAIL) {  // 帧尾校验
                memcpy(N100_ReImu, N100_tmpData, sizeof(N100_tmpData));
            }
        }

        if (Flag_Ahrs == 1 && Count == AHRS_RS) {  // 保存 AHRS 数据
            Count = 0;
            Flag_Ahrs = 0;
            Handle_Ahrs = 1;
            if (N100_tmpData[AHRS_RS - 1] == FRAME_TAIL) {
                memcpy(N100_ReAhrs, N100_tmpData, sizeof(N100_tmpData));
            }
            for (int i = 0; i < sizeof(N100_tmpData); i++) N100_tmpData[i] = 0;  // 清空数据数组
        }
		HAL_UARTEx_ReceiveToIdle_DMA(&huart7, &N100_Rxbuffer, sizeof(N100_Rxbuffer));
 
    }
	
	else{
    for (uint8_t i = 0; i < idx; ++i) {
        if (huart == uart_instance[i]->huart) {
            if (uart_instance[i]->callback_function != NULL) {
                uart_instance[i]->callback_function();
                memset(uart_instance[i]->rx_buffer, 0, Size);  // 接收结束后清空buffer,对于变长数据是必要的
            }
            HAL_UARTEx_ReceiveToIdle_DMA(uart_instance[i]->huart, uart_instance[i]->rx_buffer, uart_instance[i]->rx_buffer_size);
            return;
        }
    }
	}
	
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    for (uint8_t i = 0; i < idx; ++i) {
        if (huart == uart_instance[i]->huart) {
            HAL_UARTEx_ReceiveToIdle_DMA(uart_instance[i]->huart, uart_instance[i]->rx_buffer, uart_instance[i]->rx_buffer_size);
            memset(uart_instance[i]->rx_buffer, 0, uart_instance[i]->rx_buffer_size);  // 接收结束后清空buffer,对于变长数据是必要的
            return;
        }
    }
}




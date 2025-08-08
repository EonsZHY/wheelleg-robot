/* Includes ------------------------------------------------------------------*/
#include "remote.h"
Remote_t remote; //Applicating
/**
 * @brief Converts Sbus data to RC data.
 *
 * This function takes a pointer to Sbus data and converts it to RC data.
 *
 * @param _pdata Pointer to the Sbus data.
 */
void Remote_t::SbusToRc(uint8_t *rx_buffer)
{
	uint16_t channel_buffer[IBUS_MAX_CHANNLES] = {0};
	
	if(rx_buffer[0] == IBUS_LENGTH && rx_buffer[1] == IBUS_COMMAND40)
		{
			checksum_cal = 0xffff - rx_buffer[0] - rx_buffer[1];
			for(int i = 0; i < IBUS_MAX_CHANNLES; i++)
			{
				channel_buffer[i] = (uint16_t)(rx_buffer[i * 2 + 3] << 8 | rx_buffer[i * 2 + 2]);
				checksum_cal = checksum_cal - rx_buffer[i * 2 + 3] - rx_buffer[i * 2 + 2];
			}
			checksum_ibus = rx_buffer[31] << 8 | rx_buffer[30];

			if(checksum_cal == checksum_ibus)
			{
				for(int j = 0; j < IBUS_USER_CHANNELS; j++)
				{
					channel[j] = channel_buffer[j];
				}
			}
		}
}

/**
 * @brief This function is the callback for remote control events.
 *
 * It is called when a remote control event occurs.
 *
 * @return void
 */
static void RemoteControlCallBack()
{
    remote.SbusToRc(remote.premote_instance->rx_buffer);
}

/**
 * @brief Initializes the remote control module.
 *
 * This function initializes the remote control module by configuring the UART peripheral.
 *
 * @param _phuart Pointer to the UART handle structure.
 */
void Remote_t::Init(UART_HandleTypeDef *_phuart)
{
    UartInitConfig conf;
    conf.huart = _phuart;
    conf.rx_buffer_size = 32;
    conf.callback_function = RemoteControlCallBack;
    remote.premote_instance = pUartRegister(&conf);
    return;
}

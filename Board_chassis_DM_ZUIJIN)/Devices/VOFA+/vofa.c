#include "vofa.h"
uint8_t vofaTxBuffer[100];
Vofa_t Vofa;
// 按printf格式写，最后必须加\r\n
void Vofa_FireWater(const char *format, ...)
{
    uint8_t txBuffer[100];
    uint32_t n;
    va_list args;
    va_start(args, format);
    n = vsnprintf((char *)txBuffer, 100, format, args);

    //....在此替换你的串口发送函数...........
    HAL_UART_Transmit_DMA(&huart10, (uint8_t *)txBuffer, n);
    //......................................

    va_end(args);
}

// 输入个数和数组地址s
void Vofa_JustFloat(float *_data, uint8_t _num) {
    memcpy(vofaTxBuffer, _data, _num * 4);
    uint8_t temp_end[4] = {0x00, 0x00, 0x80, 0x7F};
    memcpy(vofaTxBuffer + (_num * 4), temp_end, 4);
    
    HAL_UART_Transmit_DMA(&huart10, vofaTxBuffer, (_num * 4) + 4);
}

/*...........示例..............
    float f1=0.5,f2=114.514;
    Vofa_FireWater("%f,%f\r\n", f1, f2);

    float f3[3]={88.77,0.66,55.44};
    Vofa_JustFloat(f3, 3);
*/

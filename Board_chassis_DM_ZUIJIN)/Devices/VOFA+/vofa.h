#ifndef _VOFA_H
#define _VOFA_H

//....在此替换你的串口函数路径........
#include "usart.h"
//...................................
#include <stdio.h>
#include "stdint.h"
#include <string.h>
#include <stdarg.h>


typedef struct
{
	float roll;
	float yaw;
	float pitch;
	float data[3];
}Vofa_t;
void Vofa_FireWater(const char *format, ...);
void Vofa_JustFloat(float *_data, uint8_t _num);
extern Vofa_t Vofa;
#endif

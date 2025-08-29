/**
 * @file FuzzyPID.c
 * @author Why
 * @brief 
 * @version 0.1
 * @date 2023-09-28
 * 
 */
#ifndef __FUZZYPID_H
#define __FUZZYPID_H

//初始化结构体参数
#define FUZZYPID_Yaw_GroupInit           \
    {                                \
		0,                           \
		0,                           \
		0,                           \
		50,                       \
		-50,                      \
		15.f,                        \
		0.2f,                     \
		3.0,                        \
    }
#define FUZZYPID_AimYaw_GroupInit           \
    {                                \
		0,                           \
		0,                           \
		0,                           \
		50,                       \
		-50,                      \
		5.f,                        \
		0,                     \
		3.0,                        \
    }

//定义模糊PID结构体
typedef struct
{
	float deta_kp; //比例值增量
	float date_ki;  //积分值增量
	float date_kd;  //微分值增量

	float maximum; //输出值的上限
	float minimum;  //输出值的下限

	float qKp;    //kp增量的修正系数
	float qKi;      //ki增量的修正系数
	float qKd;    //kd增量的修正系数
}FUZZYPID_Data_t;



extern FUZZYPID_Data_t FuzzyPID_Yaw;
extern FUZZYPID_Data_t FuzzyPID_AimYaw;
#ifdef __cplusplus
extern "C" {
#endif
void FuzzyComputation (FUZZYPID_Data_t *vPID, float thisError, float lastError);
#ifdef __cplusplus
}
#endif

#endif

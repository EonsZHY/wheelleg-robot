/**
 * @file FuzzyPID.c
 * @author Why
 * @brief 这里的函数设计想法是写到PID的计算内部去，实现Fuzzy在线调整PID参数
 *		  需要调的数只有PID改变量的比例
 * @version 0.1
 * @date 2023-09-28
 *
 */
#include "FuzzyPID.h"

#define NB   -3
#define NM	 -2
#define NS	 -1
#define ZO	 0
#define PS	 1
#define PM	 2
#define PB	 3

/* 规则库 */
static const float ruleKp[7][7] = {
    PB,	PB,	PM,	PM,	PS,	ZO,	ZO,
    PB,	PB,	PM,	PS,	PS,	ZO,	NS,
    PM,	PM,	PM,	PS,	ZO,	NS,	NS,
    PM,	PS,	PS,	ZO,	NS,	NM,	NM,
    PS,	PS,	ZO,	NS,	NS,	NM,	NM,
    PS,	ZO,	NS,	NM,	NM,	NM,	NB,
    ZO,	ZO,	NM,	NM,	NM,	NB,	NB
};

static const float ruleKi[7][7] = {
    NB,	NB,	NM,	NM,	NS,	ZO,	ZO,
    NB,	NB,	NM,	NS,	NS,	ZO,	ZO,
    NB,	NM,	NS,	NS,	ZO,	PS,	PS,
    NM,	NS,	NS,	ZO,	PS,	PM,	PM,
    NS,	NS,	ZO,	PS,	PS,	PM,	PB,
    ZO,	ZO,	PS,	PS,	PM,	PB,	PB,
    ZO,	ZO,	PS,	PM,	PM,	PB,	PB
};

static const float ruleKd[7][7] = {
    PS,	NS,	NB,	NB,	NB,	NM,	PS,
    PS,	NS,	NB,	NM,	NM,	NS,	ZO,
    ZO,	NS,	NM,	NM,	NS,	NS,	ZO,
    ZO,	NS,	NS,	NS,	NS,	NS,	ZO,
    ZO,	ZO,	ZO,	ZO,	ZO,	ZO,	ZO,
    PB,	NS,	PS,	PS,	PS,	PS,	PB,
    PB,	PM,	PM,	PM,	PS,	PS,	PB
};

FUZZYPID_Data_t FuzzyPID_Yaw = FUZZYPID_Yaw_GroupInit;
FUZZYPID_Data_t FuzzyPID_AimYaw = FUZZYPID_AimYaw_GroupInit;
#undef FUZZYPID_Yaw_GroupInit
#undef FUZZYPID_AimYaw_GroupInit

/**
  * @brief  把误差和误差的变化量转化到-3到3的论域上，先算e再算ec
  * @param  FUZZYPID_Data_t *vPID --对应的FuzzyPID结构体指针
			float thisError       --这次的误差
			float lastError       --上一次的误差
			float *qValue         --存E和EC量化的数组首地址指针
  * @retval None
  */
static void Linear_Quantization(FUZZYPID_Data_t *vPID, float thisError, float lastError, float *qValue)
{
    float deltaError;
    deltaError = thisError - lastError;   //计算偏差增量

    //E和EC的量化
    qValue[0] = 3.0f * thisError / (vPID->maximum - vPID->minimum);
    qValue[1] = 3.0f * deltaError / (vPID->maximum - vPID->minimum);

}

/**
  * @brief  隶属度计算函数，使用纯线性的折线
  * @param  float *ms 		--对应模糊子集的隶属度
			float qv      	--量化后的模糊量
			int *index      --根据大小存规则表下标的数组首地址
  * @retval None
  */
static void Membership_Calc(float *ms, float qv, int *index)
{
    if((qv >= NB) && (qv < NM))
    {
        index[0] = 0;
        index[1] = 1;
        ms[0] = -1 * qv - 2.0f; 	//y=-1x-2.0
        ms[1] = qv + 3.0f;  	   	//y=0.5x+3.0
    }
    else if((qv >= NM) && (qv < NS))
    {
        index[0] = 1;
        index[1] = 2;
        ms[0] = -1 * qv - 1.0f; 	//y=-1x-1.0
        ms[1] = qv + 2.0f;  	   	//y=0.5x+2.0
    }
    else if((qv >= NS) && (qv < ZO))
    {
        index[0] = 2;
        index[1] = 3;
        ms[0] = -1 * qv;  	   		//y=-1x
        ms[1] = qv + 1.0f; 	   		//y=0.5x+1.0
    }
    else if((qv >= ZO) && (qv < PS))
    {
        index[0] = 3;
        index[1] = 4;
        ms[0] = -1 * qv + 1.0f; 	//y=-1x+1.0
        ms[1] = qv;   	 	   		//y=0.5x
    }
    else if((qv >= PS) && (qv < PM))
    {
        index[0] = 4;
        index[1] = 5;
        ms[0] = -1 * qv + 2.0f; 	//y=-1x+2.0
        ms[1] = qv - 1.0f; 	 	  	//y=0.5x-1.0
    }
    else if((qv >= PM) && (qv <= PB))
    {
        index[0] = 5;
        index[1] = 6;
        ms[0] = -1 * qv + 3.0f; 	//y=-1x+3.0
        ms[1] = qv - 2.0f;  	   	//y=0.5x-2.0
    }
}

/**
  * @brief  更新FuzzyPID输出的PID参数的改变值
  * @param  FUZZYPID_Data_t *vPID --对应的模糊PID结构体
			float thisError       --这次的误差
			float lastError       --上次的误差
  * @retval None
  */
void FuzzyComputation (FUZZYPID_Data_t *vPID, float thisError, float lastError)
{
    /* 量化值 */
    float qValue[2] = {0, 0};

    int indexE[2] = {0, 0};  //e在规则库中的索引
    float msE[2] = {0, 0};   //e的隶属度

    int indexEC[2] = {0, 0}; //ec在规则库中的索引
    float msEC[2] = {0, 0};  //ec的隶属度

    /* pid增量值 */
    float pidvalue[3];

    /* 量化 */
    Linear_Quantization(vPID, thisError, lastError, qValue);
    //计算e的隶属度和索引 */
    Membership_Calc(msE, qValue[0], indexE);
    /* 计算ec的隶属度和索引 */
    Membership_Calc(msEC, qValue[1], indexEC);

    /* 采用重心法计算pid增量值 */
    pidvalue[0] = msE[0] * (msEC[0] * ruleKp[indexE[0]][indexEC[0]] + msEC[1] * ruleKp[indexE[0]][indexEC[1]])\
                  + msE[1] * (msEC[0] * ruleKp[indexE[1]][indexEC[0]] + msEC[1] * ruleKp[indexE[1]][indexEC[1]]);
    pidvalue[1] = msE[0] * (msEC[0] * ruleKi[indexE[0]][indexEC[0]] + msEC[1] * ruleKi[indexE[0]][indexEC[1]])\
                  + msE[1] * (msEC[0] * ruleKi[indexE[1]][indexEC[0]] + msEC[1] * ruleKi[indexE[1]][indexEC[1]]);
    pidvalue[2] = msE[0] * (msEC[0] * ruleKd[indexE[0]][indexEC[0]] + msEC[1] * ruleKd[indexE[0]][indexEC[1]])\
                  + msE[1] * (msEC[0] * ruleKd[indexE[1]][indexEC[0]] + msEC[1] * ruleKd[indexE[1]][indexEC[1]]);

    /* pid增量修正 */
    vPID->deta_kp = vPID->qKp * pidvalue[0]; 
    vPID->date_ki = vPID->qKi * pidvalue[1];
    vPID->date_kd = vPID->qKd * pidvalue[2];
}

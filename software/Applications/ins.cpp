/* Includes ------------------------------------------------------------------*/
#include "ins.h"
#include "bsp_dwt.h"
#include "pid.h"
#include "tim.h"
#include "user_lib.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
INS_t INS;
/* Private function prototypes -----------------------------------------------*/
#define X 0
#define Y 1
#define Z 2
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};


uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;

static void IMU_Param_Correction(IMU_Param_t* param, float gyro[3],
                                 float accel[3]);



void INS_Init(void) {
 Saber_Init();
}

void INS_Task(void) {

const float gravity[3] = {0, 0, 9.805f};
dt = DWT_GetDeltaT(&INS_DWT_Count);
t += dt;
Saber_Read();
INS.Accel[X] = Saber_Angle.Acc_x*9.8;//单位m/s2
INS.Accel[Y] = Saber_Angle.Acc_y*9.8 ;
INS.Accel[Z] = Saber_Angle.Acc_z*9.8;

INS.Gyro[X] =Saber_Angle.Gyro_x*DEGREE_2_RAD;
INS.Gyro[Y] = Saber_Angle.Gyro_y*DEGREE_2_RAD;
INS.Gyro[Z] =Saber_Angle.Gyro_z*DEGREE_2_RAD;
INS.q[0]=Saber_Angle.q1;
INS.q[1]=Saber_Angle.q2;
INS.q[2]=Saber_Angle.q3;
INS.q[3]=Saber_Angle.q4;
BodyFrameToEarthFrame(xb, INS.xn, INS.q);
BodyFrameToEarthFrame(yb, INS.yn, INS.q);
BodyFrameToEarthFrame(zb, INS.zn, INS.q);

    // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
 float gravity_b[3];
 EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
 for (uint8_t i = 0; i < 3; i++)  // 同样过一个低通滤波
    {
      INS.MotionAccel_b[i] =(INS.Accel[i] - gravity_b[i]) *0.9 +INS.MotionAccel_b[i] * 0.1;
 }
BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n,INS.q);  // 转换回导航系n
INS.Roll=Saber_Angle.Pitch;
INS.Pitch=Saber_Angle.RoLL;
INS.Yaw=Saber_Angle.Yaw;
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float* vecBF, float* vecEF, float* q) {
  vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                     (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                     (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

  vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                     (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                     (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

  vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                     (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                     (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float* vecEF, float* vecBF, float* q) {
  vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                     (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                     (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

  vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                     (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                     (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

  vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                     (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                     (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief
 * reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t* param, float gyro[3],
                                 float accel[3]) {
  static float lastYawOffset, lastPitchOffset, lastRollOffset;
  static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
  float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

  if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
      fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
      fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag) {
    cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
    cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
    cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
    sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
    sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
    sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

    // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
    c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
    c_12 = cosPitch * sinYaw;
    c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
    c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
    c_22 = cosYaw * cosPitch;
    c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
    c_31 = -cosPitch * sinRoll;
    c_32 = sinPitch;
    c_33 = cosPitch * cosRoll;
    param->flag = 0;
  }
  float gyro_temp[3];
  for (uint8_t i = 0; i < 3; i++)
    gyro_temp[i] = gyro[i] * param->scale[i];

  gyro[X] = c_11 * gyro_temp[X] + c_12 * gyro_temp[Y] + c_13 * gyro_temp[Z];
  gyro[Y] = c_21 * gyro_temp[X] + c_22 * gyro_temp[Y] + c_23 * gyro_temp[Z];
  gyro[Z] = c_31 * gyro_temp[X] + c_32 * gyro_temp[Y] + c_33 * gyro_temp[Z];

  float accel_temp[3];
  for (uint8_t i = 0; i < 3; i++)
    accel_temp[i] = accel[i];

  accel[X] = c_11 * accel_temp[X] + c_12 * accel_temp[Y] + c_13 * accel_temp[Z];
  accel[Y] = c_21 * accel_temp[X] + c_22 * accel_temp[Y] + c_23 * accel_temp[Z];
  accel[Z] = c_31 * accel_temp[X] + c_32 * accel_temp[Y] + c_33 * accel_temp[Z];

  lastYawOffset = param->Yaw;
  lastPitchOffset = param->Pitch;
  lastRollOffset = param->Roll;
}
/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float* q, float* Yaw, float* Pitch, float* Roll) {
  *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) *
         57.295779513f;
  *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                  2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) *
           57.295779513f;
  *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float* q) {
  float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
  Yaw /= 57.295779513f;
  Pitch /= 57.295779513f;
  Roll /= 57.295779513f;
  cosPitch = arm_cos_f32(Pitch / 2);
  cosYaw = arm_cos_f32(Yaw / 2);
  cosRoll = arm_cos_f32(Roll / 2);
  sinPitch = arm_sin_f32(Pitch / 2);
  sinYaw = arm_sin_f32(Yaw / 2);
  sinRoll = arm_sin_f32(Roll / 2);
  q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
  q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
  q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
  q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}

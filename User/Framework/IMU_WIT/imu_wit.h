//
// Created by DELL on 2023/9/12.
//

#ifndef CODE_IMU_H
#define CODE_IMU_H

#define IMU_DATASIZE 33u
#include "stm32f4xx_hal.h"

typedef struct
{
	float w;
	float x;
	float y;
	float z;
} GetQuaternion; // 0-8191 对应0-360°

typedef struct
{
  float roll;
  float pitch;
  float yaw;
} GetNaiveAngle; // 0-8191 对应0-360°

void IMU_UartInit(void);
void IMU_Data_Handler(void);
float IMU_Angle_Wit(int8_t Witch_angle);
float IMU_Speed_Wit(int8_t Witch_angle);
float IMU_AngleIncreLoop(float angle_now);
float IMU_AngleIncreLoop1(float angle_now);
GetQuaternion IMU_Quaternion(void);
GetNaiveAngle IMU_NaiveAngle(void);

#endif //CODE_IMU_H

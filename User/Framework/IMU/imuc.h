//
// Created by DELL on 2023/11/20.
//

#ifndef CMC_GIMBAL_2024_IMUC_H
#define CMC_GIMBAL_2024_IMUC_H
#include "stm32f4xx_hal.h"
#include "INS_task.h"
#define PI 3.1415926535

#define rad2degree 180.0f/PI    //弧度转换为度
#define rad2rpm     30.0f/PI    //rad/s转换为rpm

float IMU_AngleIncreLoop(float now_angle);
float IMU_Angle(int8_t Witch_angle);
float IMU_Speed(int8_t Witch_angle);

#endif //CMC_GIMBAL_2024_IMUC_H

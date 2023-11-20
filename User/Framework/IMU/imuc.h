//
// Created by DELL on 2023/11/20.
//

#ifndef CMC_GIMBAL_2024_IMUC_H
#define CMC_GIMBAL_2024_IMUC_H
#include "stm32f4xx_hal.h"
#include "INS_task.h"

float IMU_AngleIncreLoop(float now_angle);
float IMU_Angle(int8_t Witch_angle);
float IMU_Speed(int8_t Witch_angle);

#endif //CMC_GIMBAL_2024_IMUC_H

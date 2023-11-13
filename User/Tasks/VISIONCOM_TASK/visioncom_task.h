//
// Created by DELL on 2023/10/14.
//

#ifndef HERO_TEST_VISIONCOM_TASK_H
#define HERO_TEST_VISIONCOM_TASK_H
#include "stm32f4xx_hal.h"
//直接在freertos里面include该文件，任务定位不到真函数，只有弄到main.h才行，这个是什么原理？
//调试运行只往error_handler里面跑 -要reset一下再运行
void Vision_JudgeUpdate(float shoot_speed, int8_t color, int8_t type);
void VisionChattingLoop(uint8_t mode);
#endif //HERO_TEST_VISIONCOM_TASK_H

//
// Created by DELL on 2023/9/23.
//

#include "laser.h"
#include "gpio.h"

//激光模块
void Laser_On(void)
{
    HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_SET);
}

void Laser_Off(void)
{
    HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_RESET);
}

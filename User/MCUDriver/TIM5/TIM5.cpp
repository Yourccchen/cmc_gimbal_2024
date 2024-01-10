//
// Created by DELL on 2023/9/26.
//

#include "TIM5.h"
#include "bsp_can.h"
#include "debugc.h"
#include "remotec.h"
#include "gimbalc.h"
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim1;

/**
  *@brief   中断回调函数，也是主函数入口（通过中断进入，5ms一次）
  *@param   none
  *@retval  none
  */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
//{
//    if(htim == &htim5)
//    {
////        gimbal.GimbalControlLoop();
//  }
//}

/**
  *@brief   开启TIM5的中断
  *@param   none
  *@retval  none
  */
void TIM5_IT_Init(void)
{
    HAL_TIM_Base_Start_IT(&htim5);
}


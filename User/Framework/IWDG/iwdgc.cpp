//
// Created by DELL on 2023/9/12.
//

#include "iwdgc.h"

extern IWDG_HandleTypeDef hiwdg;

/**
	* @brief  喂狗函数，防止程序卡死
	* @param  None
	* @retval None
*/
void FeedDog(void)
{
	HAL_IWDG_Refresh(&hiwdg); //喂狗
}

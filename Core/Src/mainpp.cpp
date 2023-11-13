//
// Created by DELL on 2023/11/13.
//
#include "BSP.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
void BSP_Init(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_USART2_UART_Init();
    MX_USART1_UART_Init();
//    MX_IWDG_Init();  //看门狗初始化
    MX_TIM4_Init();
    MX_FREERTOS_Init();  //FreeRTOS初始化
}

void User_Init()
{
    DEBUGC_UartInit();         //使能串口UART8中断
    Power_on();                //电源打开
    REMOTEC_Init();            //遥控器初始化
    CAN_All_Init();            //CAN通信相关配置初始化（设置过滤条件、打开CAN中断等）
//    IMU_UartInit();            //IMU串口中断及DMA初始化
    gimbal.Gimbal_KalmanInit();//云台的卡尔曼算法初始函数
    Laser_On();
//    TIM5_IT_Init();          //TIM5中断打开
    osKernelStart();           //FreeRTOS内核初始化，在该函数调用前，切勿使用osDelay()来延时
}

int main()
{
    BSP_Init();
    User_Init();
    usart_printf("Program Success!\r\n");
    while(1)
    {
        HAL_Delay(100);
    }
}
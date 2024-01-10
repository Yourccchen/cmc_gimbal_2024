//
// Created by DELL on 2023/11/13.
//
#include "BSP.h"

void BSP_Init(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();

//    MX_IWDG_Init();

    MX_TIM5_Init();
    MX_TIM10_Init();

    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_USART6_UART_Init();

    MX_SPI1_Init();
    MX_I2C3_Init();

//    delay_init();

}

void User_Init()
{
    GIMBAL=OLD_HERO;           //切换新旧英雄
    DEBUGC_UartInit();         //使能串口UART1中断
    Power_on();              //电源打开
    REMOTEC_Init();            //遥控器初始化
    CAN_All_Init();            //CAN通信相关配置初始化（设置过滤条件、打开CAN中断等）

    gimbal.Gimbal_KalmanInit();//云台的卡尔曼算法初始函数
    Laser_On();
    IMU_UartInit();
    HAL_Delay(1000);
    init_cybergear(&mi_motor[0],0x7F,Motion_mode);          //小米电机初始化

    MX_FREERTOS_Init();        //FreeRTOS初始化
    osKernelStart();           //FreeRTOS内核初始化，在该函数调用前，切勿使用osDelay()来延时
}

int main()
{

    BSP_Init();
    User_Init();
    while(1)
    {
        HAL_Delay(100);
    }
}
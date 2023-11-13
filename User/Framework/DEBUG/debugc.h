//
// Created by DELL on 2023/9/23.
//
/**
attention: while using this file,please add "DEBUGC_UartIrqHandler(&huartX);" in "stm32fxxx_it.c\void USARTX_IRQHandler(void)"
and add "void DEBUGC_UartIrqHandler(UART_HandleTypeDef* huart); void DEBUGC_UartIdleCallback(UART_HandleTypeDef *huart);" in "usart.h"
and add " #include "usart.h" " in "stm32f4xx_it.c"
 **/

#ifndef HERO_TEST_DEBUGC_H
#define HERO_TEST_DEBUGC_H



#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdint.h"

#define DEBUG_RVSIZE 255
typedef struct {
    float speed_kp;  //50
    float speed_ki;
    float speed_kd;
    int32_t speed_maxOutput;
    int32_t speed_maxIntegral;
    int32_t speed_rampTargetValue;
    int32_t speed_rampTargetTime;
    int32_t speed_rampTargetStep;

    float pos_kp;
    float pos_ki;
    float pos_kd;
    int32_t pos_maxOutput;
    int32_t pos_maxIntegral;  //800
    int32_t pos_maxOutStep;    //改大了速度跟不上   就是这个问题 导致速度跟不上，到达目标位置附近小幅度震荡（正负3左右）
    int32_t pos_targetAngle;
} DebugParam;

#define STARTPID  0x70         //start_pid
#define STARTLQR 0x6C          //start_lqr
#define START 0x31
#define STOP  0x30
#define MAOHAO  0x3A

//第一位
#define SPEED_LOOP 0x73
//第四位
#define SPEED_KP 0x70            //s_kp
#define SPEED_KI 0x69            //s_ki
#define SPEED_KD 0x64            //s_kd
#define SPEED_MAXOUT 0x6F        //s_mo
#define SPEED_MAXINTEGRAL 0x61   //s_ma
#define SPEED_TARVALUE 0x76      //s_tv
#define SPEED_TARTIME 0x74       //s_tt
#define SPEED_TARSTEP 0x73       //s_ts

//第一位
#define POS_LOOP 0x70
//第四位
#define POS_KP 0x70            //p_kp
#define POS_KI 0x69            //p_ki
#define POS_KD 0x64            //p_kd
#define POS_MAXOUT 0x6F        //p_mo
#define POS_MAXINTEGRAL 0x61   //p_ma
#define POS_MAXSTEP 0x73       //p_ms
#define POS_TARVALUE 0x76      //p_tv

//class debugc {
//
//};
DebugParam Debug_Param();
void usart_printf(const char *format, ...);

#endif //HERO_TEST_DEBUGC_H

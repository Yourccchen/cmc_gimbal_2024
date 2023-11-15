//
// Created by DELL on 2023/9/23.
//

#include "debugc.h"
#include "debugc.h"
#include <cstring>
#include <cstdlib>
#include "usart.h"

extern UART_HandleTypeDef huart6;
//UART2 printf设置
#define TX_BUF_SIZE 512
uint8_t send_buf[TX_BUF_SIZE];

DebugParam debugParam;
char debugRvBuff[DEBUG_RVSIZE] = { 0 };  //存放串口2（调试用）接收的第一手数据
char debugBuff[DEBUG_RVSIZE] = { 0 };    //进行一定变换
char *pEnd;
int16_t start_flag = 0;

void DEBUGC_UartInit(void)
{
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart6, (uint8_t*)debugRvBuff, DEBUG_RVSIZE);
}

void usart_printf(const char* format, ...)
{
    va_list args;
    uint32_t length;
    va_start(args, format);
    length = vsnprintf((char*)send_buf, TX_BUF_SIZE, (const char*)format, args);
    va_end(args);
    HAL_UART_Transmit_DMA(&huart6, (uint8_t*)send_buf, length);
}

void DEBUGC_UartIrqHandler(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART6)                                 //判断是否是串口1
    {
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)!= RESET)   //判断是否是空闲中断
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);                     //清楚空闲中断标志（否则会一直不断进入中断）
            DEBUGC_UartIdleCallback(huart);                       //调用中断处理函数
        }
    }
}

void DEBUGC_UartIdleCallback(UART_HandleTypeDef* huart)
{
    HAL_UART_DMAStop(huart);                                                     //停止本次DMA传输
    //解包预处理
    //将串口收到的数据进行处理，新的数组以数字开头，便于之后字符转浮点数的运算
    memcpy(debugBuff, &debugRvBuff[5], 10);
    uint8_t data_length = DEBUG_RVSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);   //计算接收到的数据长度
    //电机启动
    if (debugRvBuff[5] == MAOHAO && debugRvBuff[6] == START)
    {
        start_flag = 1;
    }
    else if (debugRvBuff[5] == MAOHAO && debugRvBuff[6] == STOP) start_flag = 0;

    switch (debugRvBuff[0])
    {
        case SPEED_LOOP:
        {
            switch (debugRvBuff[3])
            {
                case SPEED_KP:
                    debugParam.speed_kp = strtof(debugBuff, &pEnd);
                    break;
                case SPEED_KI:
                    debugParam.speed_ki = strtof(debugBuff, &pEnd);
                    break;
                case SPEED_KD:
                    debugParam.speed_kd = strtof(debugBuff, &pEnd);
                    break;
                case SPEED_MAXOUT:
                    debugParam.speed_maxOutput = strtof(debugBuff, &pEnd);
                    break;
                case SPEED_MAXINTEGRAL:
                    debugParam.speed_maxIntegral = strtof(debugBuff, &pEnd);
                    break;
                case SPEED_TARVALUE:
                    debugParam.speed_rampTargetValue = strtof(debugBuff, &pEnd);
                    break;
                case SPEED_TARTIME:
                    debugParam.speed_rampTargetTime = strtof(debugBuff, &pEnd);
                    break;
                case SPEED_TARSTEP:
                    debugParam.speed_rampTargetStep = strtof(debugBuff, &pEnd);
                    break;
            }
        }
        case POS_LOOP:
        {
            switch (debugRvBuff[3])
            {
                case POS_KP:
                    debugParam.pos_kp = strtof(debugBuff, &pEnd);
                    break;
                case POS_KI:
                    debugParam.pos_ki = strtof(debugBuff, &pEnd);
                    break;
                case POS_KD:
                    debugParam.pos_kd = strtof(debugBuff, &pEnd);
                    break;
                case POS_MAXOUT:
                    debugParam.pos_maxOutput = strtof(debugBuff, &pEnd);
                    break;
                case POS_MAXINTEGRAL:
                    debugParam.pos_maxIntegral = strtof(debugBuff, &pEnd);
                    break;
                case POS_MAXSTEP:
                    debugParam.pos_maxOutStep = strtof(debugBuff, &pEnd);
                    break;
                case POS_TARVALUE:
                    debugParam.pos_targetAngle = strtof(debugBuff, &pEnd);
                    break;
            }
        }
    }
    memset(debugRvBuff, 0, data_length);//清零接收缓冲区
    data_length = 0;
    HAL_UART_Receive_DMA(huart, (uint8_t*)debugRvBuff, DEBUG_RVSIZE);
}

DebugParam Debug_Param()
{
    return debugParam;
}


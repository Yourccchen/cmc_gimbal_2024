//
// Created by ShiF on 2023/9/12.
//

#ifndef KOSANN_UAVGIMBAL_LEDC_H
#define KOSANN_UAVGIMBAL_LEDC_H

/* Includes ------------------------------------------------------------------*/
#include "cstdint"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
/* Exported defines -----------------------------------------------------------*/
#define LED_Port_IO         GPIO_TypeDef*
#define LED_Port_Pin        uint16_t

/* Exported types ------------------------------------------------------------*/
typedef enum {
    LED_OFF = 0,
    LED_ON,
    LED_TOGGLE
} LED_State_e;

/* Exported functions prototypes ---------------------------------------------*/
class LedC {
public:
    LED_State_e state = LED_OFF;
    uint8_t on_level; //点亮电平

    LED_Port_IO led_port;
    LED_Port_Pin led_pin;

    LedC(LED_Port_IO led_Io, LED_Port_Pin led_Pin, uint8_t on_Level = 0) :
            led_port(led_Io), led_pin(led_Pin), on_level(on_Level) {};

    void setStatus(LED_State_e status);
    void setStatus(int status);

private:
};
/* Exported functions  --------------------------------------------------------*/
// 指向外部一个操作底层的函数
extern void (*setLedLevel)(LedC led, uint8_t level);

void LED_Init(void);
void LED_UI(int Led1, bool Led2, bool Led3, bool Led4, bool Led5);

void UI_Init();
void LED_UI(int Led1, bool Led2, bool Led3, bool Led4, bool Led5);
void UI_Fric(LED_State_e status);  ///是否开启摩擦轮，亮是开启
void UI_TUOLUO(LED_State_e status);  ///是否小陀螺，亮是小陀螺
void UI_ZIMIAO(LED_State_e status);  ///是否开启自瞄，亮是开启
void UI_PROTECT(LED_State_e status);
void UI_POWER(LED_State_e status);
void UI_gyroscope(int Led1);  ///小陀螺， 0是关闭
void UI_MIAOZHUN(LED_State_e status);
#endif //KOSANN_UAVGIMBAL_LEDC_H

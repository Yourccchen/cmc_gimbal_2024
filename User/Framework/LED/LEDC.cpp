//
// Created by ShiF on 2023/9/12.
//

#include "LEDC.h"
/* Private includes ----------------------------------------------------------*/
#include "ledio.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
LedC led1(GPIOF, GPIO_PIN_0, LED_ON);   //LED_1 PF0
LedC led2(GPIOF, GPIO_PIN_1, LED_ON);   //LED_2 PF1
LedC led3(GPIOB, GPIO_PIN_14, LED_ON);   //LED_3 PB12
LedC led4(GPIOB, GPIO_PIN_15, LED_ON);   //LED_4 PB13
LedC led5(GPIOB, GPIO_PIN_13, LED_ON);   //LED_5 PB14
LedC led6(GPIOB, GPIO_PIN_12, LED_ON);   //LED_6 PB15

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

void (*setLedLevel)(LedC led, uint8_t level);

/* Private user code ---------------------------------------------------------*/
void LED_SetLedOn(LedC &led) {
    led.state = LED_ON;
    setLedLevel(led, led.on_level);
}

void LED_SetLedOff(LedC &led) {
    led.state = LED_OFF;
    setLedLevel(led, !led.on_level);
}

void LED_SetLedToggle(LedC &led) {
    led.state = LED_TOGGLE;
    setLedLevel(led, 3);
}

/**
  * @brief  设置led的状态
  * @param  status: LED_OFF, LED_ON, LED_TOGGLE
  * @retval None
  */
void LedC::setStatus(LED_State_e status) {
    this->state = status;
    switch (status) {
        case LED_ON:
            LED_SetLedOn(*this);
            break;
        case LED_OFF:
            LED_SetLedOff(*this);
            break;
        case LED_TOGGLE:
            LED_SetLedToggle(*this);
            break;
    }
}
void LedC::setStatus(int status) {
    switch (status) {
        case 0:
            LED_SetLedOn(*this);
            break;
        case 1:
            LED_SetLedOff(*this);
            break;
        case 2:
            LED_SetLedToggle(*this);
            break;
    }
}

void LED_Init(void) {
    LEDIO_ConfigInit();
    LedC led(GPIOE, GPIO_PIN_11, LED_ON);   //LED_R PE11
    led.setStatus(LED_TOGGLE);
}

///--------物理ui接口--------///

///Led_Ui 24译码器灯在右边 为Led1.....

void UI_Init()
{
    LED_UI(3,1,0,0,0);
}

void LED_UI(int Led1, bool Led2, bool Led3, bool Led4, bool Led5) //Led_Ui 24译码器灯为Led1.....0为亮，1为灭
{
    LEDIO_ConfigInit();
    int Led1x[2];
    switch (Led1)
    {
        case 0:
            Led1x[0] = 0;
            Led1x[1] = 0;
            break;
        case 1:
            Led1x[0] = 0;
            Led1x[1] = 1;
            break;
        case 2:
            Led1x[0] = 1;
            Led1x[1] = 0;
            break;
        case 3:
            Led1x[0] = 1;
            Led1x[1] = 1;
            break;
    }
    led1.setStatus(Led1x[0]);
    led2.setStatus(Led1x[1]);
    led3.setStatus(Led2);
    led4.setStatus(Led3);
    led5.setStatus(Led4);
    led6.setStatus(Led5);
}

void UI_Fric(LED_State_e status)  ///摩擦轮，最左边的LED灯
{
    led6.setStatus(status);
}

void UI_TUOLUO(LED_State_e status)  ///小陀螺，左边第二的LED灯
{
    led5.setStatus(status);
}

void UI_ZIMIAO(LED_State_e status)  ///自瞄，左边第三的LED灯
{
    led4.setStatus(status);
}

void UI_PROTECT(LED_State_e status)  ///保护，左边第四的LED灯
{
    led2.setStatus(status);
}

void UI_wait1(LED_State_e status)  ///暂定，左边第五的LED灯
{
    led3.setStatus(status);
}
void UI_wait2(LED_State_e status)  ///暂定，左边第六的LED灯
{
    led1.setStatus(status);
}



void UI_gyroscope(int Led1)  ///小陀螺， 3是关闭
{
    int Led1x[2];
    switch (Led1) {
        case 0:
            Led1x[0] = 0;
            Led1x[1] = 0;
            break;
        case 1:
            Led1x[0] = 0;
            Led1x[1] = 1;
            break;
        case 2:
            Led1x[0] = 1;
            Led1x[1] = 0;
            break;
        case 3:
            Led1x[0] = 1;
            Led1x[1] = 1;
            break;
    }
    led1.setStatus(Led1x[0]);
    led2.setStatus(Led1x[1]);
}
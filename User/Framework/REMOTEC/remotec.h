//
// Created by DELL on 2023/9/25.
//

#ifndef HERO_TEST_REMOTEC_H
#define HERO_TEST_REMOTEC_H

#include <cctype>
#include "remoteio.h"
#include "stm32f4xx_hal.h"

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */

typedef __packed struct
{
    uint8_t Last_State: 1;
    uint8_t Now_State: 1;
    uint8_t Is_Click_Once: 1;
} Key_State;

typedef __packed struct
{
    __packed struct
    {
        int16_t ch[5];
        char s[2];
    } rc;
    __packed struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
//以下为使用到点按键值的键
        Key_State press_l;
        Key_State press_r;
    } mouse;
    __packed struct
    {
        uint16_t value;
        Key_State W;
        Key_State S;
        Key_State A;
        Key_State D;
        //以下为使用到点按键值的键
        Key_State SHIFT;
        Key_State CONTRL;
        Key_State Q;
        Key_State E;
        Key_State R;
        Key_State F;
        Key_State G;
        Key_State Z;
        Key_State X;
        Key_State C;
        Key_State V;
        Key_State B;
    } key;
} RC_ctrl_t;


/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t* sbus_buf, RC_ctrl_t* rc_ctrl);
void REMOTEC_Init(void);
const RC_ctrl_t* get_remote_control_point(void);

void RC_DataHandle(RC_ctrl_t* rc_ctrl);
RC_ctrl_t RC_GetDatas(void);//返回按键结构体函数
void portHandle(Key_State* port);
int16_t RC_UpdateData();

float portSetVx(void);
float portSetVy(void);
float portSetYawSpeed(void);
float portSetPihSpeed(void);
uint8_t portIsZimiao(void);
int8_t portSetCarMode(void);
int8_t portSetShootMode(void);
int8_t portSetControlMode(void);
void portSetScope();
void portSetRammer(void);
void portSetTurn(void);
#endif//HERO_TEST_REMOTEC_H

//
// Created by DELL on 2023/9/24.
//

#ifndef HERO_TEST_SHOOTC_H
#define HERO_TEST_SHOOTC_H

#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdint.h"

class cShoot
{
public:
    float SHOOT_SPEED; //发射速度
    float ShootLOUT_ADRC;
    float ShootROUT_ADRC;
    float ShootUOUT_ADRC;
    ///CAN_JUDGE_BARREL_ID参数
    uint16_t heat_limit; //热量限制
    uint16_t cool_spd;   //冷却速度
    uint16_t heat_now;   //当前热量

    ///CAN_JUDGE_PARAM_ID参数
    float shoot_spd_now=0;  //当前射击速度
    uint16_t shoot_spd_max; //速度限制
    uint8_t  color;         //队伍颜色
    uint8_t  smallORbig;    //能量机关状态

    ///摩擦轮、拨弹轮相关参数
    int16_t fric_flag=0;     //摩擦轮标志位:0为关摩擦轮，1为开摩擦轮
    int16_t fric_count=0;    //摩擦轮收到遥控器发送指令的的次数
    int16_t rammer_flag=0;   //0为不转，1为转一次。无其他数值可能
    int16_t rammer_count=0;  //拨弹收到遥控器发送指令的次数，大于某个值就让rammer_flag加一
    int16_t rammer_current=0;//拨弹轮当前的电流

    void Shoot_ControlLoop();//发弹主循环，但不包含最重要的位置环和速度环
    void Shoot_PosC();
    void Shoot_SpeedC();
    void Shoot_SendCurrent(float LOut,float ROut,float UOut,float RamOut);
    void Stuck_Check();
    void Heat_Protect();
    void ShootSpeedClean();
    void Shoot_SpdChoose();
    void Shoot_ParamChoose();

private:
    int32_t stuck_time=0;//拨弹轮堵转时间
    int32_t reverse_time=0;//拨弹轮反转时间
};

#endif //HERO_TEST_SHOOTC_H

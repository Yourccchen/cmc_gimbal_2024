//
// Created by DELL on 2023/9/25.
//

#ifndef HERO_TEST_MOTORC_H
#define HERO_TEST_MOTORC_H
#include "stm32f4xx_hal.h"

#define M3508_RATION  (3591.0f / 187.0f)  //电机减速比19左右
#define M2006_RATION  (36 / 1.0f)
#define ENCODER_TO_ANGLE   (360.0f / 8192.0f)   //编码器脉冲数 -> 转动角度(度)
#define ANGLE_TO_ENCODER  (8192.0f / 360.0f)

#define M3508             20
#define M3508_OffReducer  21
#define M2006             22
#define GM6020            23

class cMotor
{
public:

    cMotor() = default;

    cMotor(int MotorType,int Which_Mode,int Which_Angle,int Algorithm):
    MotorType(MotorType),Which_Mode(Which_Mode),Which_Angle(Which_Angle),Algorithm(Algorithm)
    {}


    /**从电机获得的原始数据(会实时被CAN反馈改变，除ECD_MODE外不要用此处的变量进行计算）**/
    uint16_t RawAngle;          //电机总角度
    int16_t  RawSpeed;          //电机速度
    int16_t  RawTorqueCurrent;  //电机转矩电流
    uint8_t  RawTemperature;    //电机温度
    uint8_t  Null;
    uint16_t Connected;

    /**经过计算后得到的实际数据**/
    int64_t AllAngle;            //总原始角度
    uint16_t NowAngle;           //当前的原始角度
    uint16_t LastAngle;          //上次的原始角度
    int16_t IncreAngle;          //经角度连续化处理后得到的增量角度

    double RealAngle_Ecd;
    double RealAngle_Imu;
    float RealSpeed;

    int8_t Which_Angle;           //哪个角度，Yaw or Pitch
    int8_t Which_Mode;            //电机反馈模式
    int8_t Algorithm;            //算法选择
    int MotorType;  //电机类型

    void UpdateMotorInfo();       //处理电机数据
    int16_t GetEncoderContinueAngel(int16_t Last, int16_t Now);

private:

};

#endif //HERO_TEST_MOTORC_H

//
// Created by DELL on 2023/9/25.
//

#include "motorc.h"
#include "bsp_can.h"
#include "debugc.h"
#include "gimbalc.h"
#include "imuc.h"
/**
  *@brief   M3508角度连续化,并判断方向
  *@param   编码值读到的上次角度Last和当前角度Now
  *@retval  Incre
  */
int16_t cMotor::GetEncoderContinueAngel(int16_t Last, int16_t Now)
{
    const int16_t ContinueAngelMax = 4096; //角度连续化比较角度
    int16_t Incre = 0;
    Incre = Now - Last;
    if (Incre >= 0 && Incre < ContinueAngelMax)
    { //正转
        Incre += 0;
    }
    if (Incre < -ContinueAngelMax)
    { //正转，并且不连续
        Incre += 8192;
    }
    if (Incre <= 0 && Incre > -ContinueAngelMax)
    { //反转
        Incre += 0;
    }
    if (Incre > ContinueAngelMax)
    { //反转，并且不连续
        Incre -= 8192;
    }
    return Incre;
}


/**
  *@brief   处理电机数据，计算出电机输出端对外的速度（rpm）和角度（°）
  *@param   none
  *@retval  none
  */
void cMotor::UpdateMotorInfo()
{
    switch(Which_Mode)
    {
        case ECD_MODE:
            if (MotorType == M3508 )
            {
                RealSpeed = (float) (RawSpeed / M3508_RATION); //实际速度，用于计算
                LastAngle = NowAngle;
                NowAngle = RawAngle;
                IncreAngle = GetEncoderContinueAngel(LastAngle, NowAngle);
                AllAngle += IncreAngle; //转子角度

                RealAngle_Ecd = ((float) AllAngle * ENCODER_TO_ANGLE) / M3508_RATION; //实际角度，用于计算
            }
            else if (MotorType == GM6020|| MotorType == M3508_OffReducer )
            {
                RealSpeed = (float) RawSpeed;       //实际速度，用于计算
                LastAngle = NowAngle;
                NowAngle = RawAngle;
                IncreAngle = GetEncoderContinueAngel(LastAngle, NowAngle);
                AllAngle += IncreAngle; //转子角度

                RealAngle_Ecd = ((float) AllAngle * ENCODER_TO_ANGLE); //实际角度，用于计算
            }
            break;
        case IMU_MODE:
            if(Which_Angle==PIH_ANGLE)
            {
                ///英雄的Pih反向!!!!!!!!!///
                RealAngle_Imu=-IMU_Angle(PIH_ANGLE); //IMU读取到的角度，直接进行计算

                RealSpeed=-IMU_Speed(PIH_ANGLE);     //IMU读取到的速度，直接进行计算
            }
            else if(Which_Angle==YAW_ANGLE)
            {
                RealAngle_Imu=IMU_Angle(YAW_ANGLE);
                RealAngle_Ecd=((float) RawAngle * ENCODER_TO_ANGLE);
                RealSpeed=IMU_Speed(YAW_ANGLE);
            }
    }
}

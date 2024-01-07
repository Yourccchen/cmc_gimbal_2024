//
// Created by DELL on 2023/9/24.
//

#include "shootc.h"
#include "gimbalc.h"
#include "bsp_can.h"

/**
  *@brief   射击主循环，主要包括堵转检测、热量检测、速度设置、参数选择等
  */
void cShoot::Shoot_ControlLoop()
{
    Shoot_SpdChoose();
    Shoot_ParamChoose();
//    Stuck_Check();
}

/**
  *@brief   拨弹轮的位置环
  */
void cShoot::Shoot_PosC()
{
    portSetRammer();//拨弹轮设置目标值

    if(abs(gimbal.motors_pid[RamPos].PID_Target-gimbal.motors[RamMotor].RealAngle_Ecd)>100)
        gimbal.motors_pid[RamPos].PID_Target=gimbal.motors[RamMotor].RealAngle_Ecd;

    if(rammer_flag)
    {
        switch(GIMBAL)
        {
            case OLD_HERO:
            {
                gimbal.setMotorPos(RamPos, gimbal.motors_pid[RamPos].PID_Target + 72);
                break;
            }
            case NEW_HERO:
            {
                gimbal.setMotorPos(RamPos, gimbal.motors_pid[RamPos].PID_Target + 45.0* 28.0/100.0);
                break;
            }
        }
    }

    rammer_flag=0;

    float RamOut=gimbal.motors_pid[RamPos].PID_GetPositionPID(gimbal.motors[RamMotor].RealAngle_Ecd);

    //串级PID，位置环的输出是速度环的目标值
    gimbal.setMotorSpeed(RamSpd,RamOut);

    Shoot_SpeedC();
}

/**
  *@brief   摩擦轮和拨弹轮的速度环
  */
void cShoot::Shoot_SpeedC()
{
    if(gimbal.CarMode!=PROTECT)
    {
        gimbal.setMotorSpeed(ShootSpdL,-SHOOT_SPEED);
        gimbal.setMotorSpeed(ShootSpdR,SHOOT_SPEED);
        gimbal.setMotorSpeed(ShootSpdU,1.01*SHOOT_SPEED);
    }
    //ADRC摩擦轮计算
    ShootLOUT_ADRC=gimbal.adrc.ADRC_Calc(SHOOT_SPEED,gimbal.motors[ShootLMotor].RealSpeed);
    ShootROUT_ADRC=gimbal.adrc.ADRC_Calc(-SHOOT_SPEED,gimbal.motors[ShootRMotor].RealSpeed);
    ShootUOUT_ADRC=gimbal.adrc.ADRC_Calc(SHOOT_SPEED,gimbal.motors[ShootUMotor].RealSpeed);

    //普通PID的摩擦轮、拨弹轮计算
    gimbal.motors_pid[ShootSpdL].PID_GetPositionPID(gimbal.motors[ShootLMotor].RealSpeed);
    gimbal.motors_pid[ShootSpdR].PID_GetPositionPID(gimbal.motors[ShootRMotor].RealSpeed);
    gimbal.motors_pid[ShootSpdU].PID_GetPositionPID(gimbal.motors[ShootUMotor].RealSpeed);

    gimbal.motors_pid[RamSpd].PID_GetPositionPID(gimbal.motors[RamMotor].RealSpeed);
}
/**
  *@brief   摩擦轮与拨弹轮的电流发送
  */
void cShoot::Shoot_SendCurrent(float LOut,float ROut,float UOut,float RamOut)
{
    CAN_ShootSendCurrent(LOut,ROut,UOut,RamOut);
}

/**
  *@brief   摩擦轮拨弹轮速度清值清零
  */
void cShoot::ShootSpeedClean()
{
    gimbal.motors_pid[ShootSpdL].PID_Target=0;
    gimbal.motors_pid[ShootSpdR].PID_Target=0;
    gimbal.motors_pid[ShootSpdU].PID_Target=0;
    gimbal.motors_pid[RamSpd].PID_Target=0;

    //清除漏电流，防止关闭摩擦轮后电机仍以小速度旋转
    if(abs(gimbal.motors[ShootLMotor].RealSpeed)<100
     ||abs(gimbal.motors[ShootRMotor].RealSpeed)<100
     ||abs(gimbal.motors[ShootUMotor].RealSpeed)<100)
    {
        gimbal.motors_pid[ShootSpdL].PID_Out=0;
        gimbal.motors_pid[ShootSpdR].PID_Out=0;
        gimbal.motors_pid[ShootSpdU].PID_Out=0;
    }
    gimbal.motors_pid[RamSpd].PID_Out=0;

    //摩擦轮关闭情况下，让目标值始终等于当前编码值，防止开启摩擦轮时偏差过大导致疯转
    gimbal.motors_pid[RamPos].PID_Target=gimbal.motors[RamMotor].RealAngle_Ecd;

    ShootLOUT_ADRC=0;
    ShootROUT_ADRC=0;
    ShootUOUT_ADRC=0;
}

/**
  *@brief   拨弹轮堵转检测
  */
void cShoot::Stuck_Check()
{
    rammer_current=gimbal.motors[RamMotor].RawTorqueCurrent;//读取拨弹轮电机当前的电流值
    if(rammer_current>15000)
    {
        stuck_time++;
    }
    if(stuck_time>160)//堵转超过800ms
    {
        gimbal.motors_pid[RamSpd].PID_ErrAll=0;
        gimbal.motors_pid[RamPos].PID_ErrAll=0;

        gimbal.motors[RamMotor].RawAngle=0;
        gimbal.motors[RamMotor].AllAngle=0;
        gimbal.motors[RamMotor].RealAngle_Ecd=0;

        gimbal.setMotorSpeed(RamSpd,-100);
        rammer_flag=0;
        reverse_time++;
    }
    if(reverse_time>200)//反转1s
    {
        reverse_time=0;
        stuck_time=0;
    }
}
/**
  *@brief   摩擦轮的速度选择
  */
void cShoot::Shoot_SpdChoose()
{
    SHOOT_SPEED=6000;
}

/**
  *@brief  摩擦轮和拨弹轮的参数选择
  */
void cShoot::Shoot_ParamChoose()
{
    //拨弹轮PID设置
    gimbal.motors_pid[RamPos].SetKpid(3,0,0.1); //空转时，3稳定;负载时，7稳定
    gimbal.motors_pid[RamPos].PID_OutMax=500;

    gimbal.motors_pid[RamSpd].SetKpid(50,2,0);
    gimbal.motors_pid[RamSpd].PID_OutMax=16000;

    //摩擦轮PID设置
    gimbal.motors_pid[ShootSpdL].SetKpid(5,0.1,0);//位置式PID参数
    gimbal.motors_pid[ShootSpdR].SetKpid(5,0.1,0);
    gimbal.motors_pid[ShootSpdU].SetKpid(5,0.1,0);
}

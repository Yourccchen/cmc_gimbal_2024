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
    Shoot_SpdChoose();//速度选择
    Shoot_ParamChoose();//参数设置
    Stuck_Check();//堵转检测
    Heat_Protect();//热量保护
}

/**
  *@brief   拨弹轮的位置环
  */
void cShoot::Shoot_PosC()
{
    portSetRammer();//拨弹轮设置目标值

    if(abs(gimbal.motors_pid[RamPos].PID_Target-gimbal.motors[RamMotor].RealAngle_Ecd)>360)
        gimbal.motors_pid[RamPos].PID_Target=gimbal.motors[RamMotor].RealAngle_Ecd;

    if (shoot_permit==SHOOT_PERMIT)
    {
        if(rammer_flag==1)//0为不转，1为转一次，-1为翻转一次。无其他数值可能
        {
            gimbal.setMotorPos(RamPos, gimbal.motors_pid[RamPos].PID_Target +  360.0/9.0/31.0*110.0);
        }
        else if(rammer_flag==-1)
        {
            gimbal.setMotorPos(RamPos, gimbal.motors_pid[RamPos].PID_Target -  360.0/9.0/31.0*110.0);
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
    }
    ///ADRC摩擦轮计算
    ShootLOUT_ADRC=gimbal.adrc[0].ADRC_Calc(-SHOOT_SPEED,gimbal.motors[ShootLMotor].RealSpeed);
    ShootROUT_ADRC=gimbal.adrc[1].ADRC_Calc(SHOOT_SPEED,gimbal.motors[ShootRMotor].RealSpeed);

    ///PID摩擦轮计算
    gimbal.motors_pid[ShootSpdL].PID_GetPositionPID(gimbal.motors[ShootLMotor].RealSpeed);
    gimbal.motors_pid[ShootSpdR].PID_GetPositionPID(gimbal.motors[ShootRMotor].RealSpeed);

    ///拨弹轮计算
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
  *@brief   摩擦轮、拨弹轮速度清值清零
  */
void cShoot::ShootSpeedClean()
{
    ///PID目标值清零
    gimbal.motors_pid[ShootSpdL].PID_Target=0;
    gimbal.motors_pid[ShootSpdR].PID_Target=0;

    gimbal.motors_pid[RamSpd].PID_Target=0;

    ///ADRC目标值清零
    gimbal.adrc[0].Target=0;
    gimbal.adrc[1].Target=0;
    //清除漏电流，防止关闭摩擦轮后电机仍以小速度旋转
    if(abs(gimbal.motors[ShootLMotor].RealSpeed)<100
     ||abs(gimbal.motors[ShootRMotor].RealSpeed)<100)
    {
        ///PID相关
        gimbal.motors_pid[ShootSpdL].PID_Out=0;
        gimbal.motors_pid[ShootSpdR].PID_Out=0;

        ///ADRC相关
        ShootLOUT_ADRC=0;
        ShootROUT_ADRC=0;
    }

    gimbal.motors_pid[RamSpd].PID_Out=0;

    //摩擦轮关闭情况下，让目标值始终等于当前编码值，防止开启摩擦轮时偏差过大导致疯转
    gimbal.motors_pid[RamPos].PID_Target=gimbal.motors[RamMotor].RealAngle_Ecd;
}

/**
  *@brief   拨弹轮堵转检测
  */
void cShoot::Stuck_Check()
{
    rammer_current=gimbal.motors[RamMotor].RawTorqueCurrent;//读取拨弹轮电机当前的电流值
    if(rammer_current>14000)
    {
        stuck_time++;
    }
    if(stuck_time>50)//堵转超过250ms
    {
        gimbal.motors_pid[RamSpd].PID_ErrAll=0;
        gimbal.motors_pid[RamPos].PID_ErrAll=0;

        gimbal.motors_pid[RamPos].PID_Target=gimbal.motors[RamMotor].RealAngle_Ecd;
        Shoot_SendCurrent(0,0,0,-3000);
        rammer_flag=0;
        reverse_time++;
    }
    if(reverse_time>100)//反转0.5s
    {
        reverse_time=0;
        stuck_time=0;
    }
}
/**
  *@brief   自行计算的当前热量（与裁判系统传回的热量同时比较，取大的作为标准）
  */
int cShoot::Heat_Cal()
{
    if(fric_flag==OPENFRIC && abs(SHOOT_SPEED - gimbal.motors[ShootSpdL].RealSpeed) < 200 && shootspd_reach == 0)
    {//如果现在摩擦轮打开，并且速度与目标值只差200
        shootspd_reach=1;
    }
    if(fric_flag==CLOSEFRIC)
    {
        shootspd_reach=0;
        shootspd_drop=0;
    }
    if(shootspd_reach == 1 && (abs(gimbal.motors[ShootSpdL].RealSpeed) < SHOOT_SPEED*0.9) && shootspd_drop == 0)
    {//开摩擦轮检测到掉速
        shootspd_reach = 0;
        shootspd_drop = 1;//掉速标志位置1
    }
    if(shootspd_drop == 1)
    {
        heat_now_user += 100;  //100是一发大弹丸热量
        shootspd_drop = 0;
    }
    heat_now_user-=(float)cool_spd/200.0f; //周期是5ms
    if(heat_now_user<0)
    {
        heat_now_user=0;
    }
//    usart_printf("%d,%d\r\n",heat_now_user,heat_now);

    return (heat_now_user>heat_now) ? heat_now_user : heat_now ;//返回较大的值作为当前热量标准
}

/**
  *@brief   热量保护
  */
void  cShoot::Heat_Protect()
{
    //如果热量限制减去当前热量大于等于100，允许发弹，其余情况均不允许发弹
   if(heat_limit-Heat_Cal()>=100)
   {
       shoot_permit=SHOOT_PERMIT;
   }
   else
   {
       shoot_permit=SHOOT_FORBID;
   }

}
/**
  *@brief  返回摩擦轮当前状态
  *@retval 如果摩擦轮正在转动，返回FRIC_ON，即1；否则返回FRIC_OFF，即0
  */
int8_t cShoot::GetFricStatus(void)
{
    if (abs(gimbal.motors[ShootSpdL].RealSpeed) > 2000 && abs(gimbal.motors[ShootSpdR].RealSpeed) > 2000)
    {
        return FRIC_ON;
    }
    else
        return FRIC_OFF;
}
/**
  *@brief   摩擦轮的速度选择
  */
void cShoot::Shoot_SpdChoose()
{
    SHOOT_SPEED=6200;
}

/**
  *@brief  摩擦轮和拨弹轮的参数选择
  */
void cShoot::Shoot_ParamChoose()
{
    //拨弹轮PID设置
    gimbal.motors_pid[RamPos].SetKpid(1.5,0,0.1); //目前满载时2稳定
    gimbal.motors_pid[RamPos].PID_OutMax=200;
    gimbal.motors_pid[RamPos].PID_RampStep=5;

    gimbal.motors_pid[RamSpd].SetKpid(75,5,0);
    gimbal.motors_pid[RamSpd].PID_OutMax=15000;

    //摩擦轮PID设置
    gimbal.motors_pid[ShootSpdL].SetKpid(5,0.1,0);//位置式PID参数
    gimbal.motors_pid[ShootSpdR].SetKpid(5,0.1,0);
    gimbal.motors_pid[ShootSpdU].SetKpid(5,0.1,0);
}

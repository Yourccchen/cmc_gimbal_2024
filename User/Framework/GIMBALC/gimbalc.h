//
// Created by DELL on 2023/9/23.
//
#ifndef HERO_TEST_GIMBALC_H
#define HERO_TEST_GIMBALC_H
#include "stm32f4xx_hal.h"
#include "PIDC.h"
#include "pid.h"
#include "motorc.h"
#include "bsp_can.h"
#include "remotec.h"
#include "debugc.h"
#include "motorc.h"
#include "filters.h"
#include "kalman.h"
#include "shootc.h"
#include "power.h"
#include "usbd_cdc_if.h"
#include "packet.hpp"
#include "ADRC.h"
#include "filters.h"
#include "iwdgc.h"
#include "CH100.h"
#include "CyberGear.h"
#include "dm4310_ctrl.h"
#define RAMPSTEP 100
#define SHOOT_RAMPSTEP 20
//反馈模式选择
#define ECD_MODE 0     //编码器反馈
#define IMU_MODE 1     //陀螺仪反馈

//选择是哪个角
#define PIH_ANGLE 1  //Pitch轴角度
#define YAW_ANGLE 2  //Yaw轴角度
#define ROLL_ANGLE 3 //倍镜电机，Roll轴
#define RAM_ANGLE 4  //拨弹轮角度
#define NO_ANGLE  5  //只用到速度环的电机，如摩擦轮

//算法选择
#define NORMAL 1    //普通PID
#define MATLAB 2    //MATLAB算法，自带积分抗饱和(基本上所有兵种的Yaw轴可以用这个算法，Pitch轴需要根据实际情况调整)
#define ADRC 3      //ADRC算法
#define CYBERGEAR 4 //小米电机自带的运控模式
#define DAMIAO 5    //达妙电机的速度模式
//车体模式赋值(右侧拨杆)
#define TUOLUO   1     //小陀螺模式
#define SUIDONG  3     //随动模式
#define ZIMIAO   2     //自瞄
#define PROTECT  5     //保护模式
//控制模式赋值(左侧拨杆)
#define OPENFRIC  1    //开启摩擦轮
#define CLOSEFRIC 3    //关闭摩擦轮
#define KEY_MODE  2    //键盘操作

#define OPENZIMIAO 1
#define CLOSEZIMIAO 0
//射击模式赋值

//在线状态赋值(遥控器是否断电有关)
#define OFFLINE 0
#define ONLINE 1

//是否允许发弹(热量限制有关)
#define SHOOT_FORBID 0
#define SHOOT_PERMIT 1

//摩擦轮当前状态(热量限制有关)
#define FRIC_ON 1
#define FRIC_OFF 0

//切换新旧英雄判断
extern bool GIMBAL;

#define OLD_HERO 0
#define NEW_HERO 1

extern ExtU rtU;
extern ExtY rtY;
#define Pid_In rtU
#define Pid_Out rtY //matlab生成的PID

///电机PID计算枚举类型
typedef enum
{
    PihSpd=0,
    PihPos=1,
    YawSpd=2,
    YawPos=3,
    RamSpd=4,
    RamPos=5,
    ShootSpdL=6,
    ShootSpdR=7,
    ShootSpdU=8,
    ChassisYaw=9,
    ScopeUSpd=10,
    ScopeUPos=11
}eMotorsPid;

///电机枚举类型
typedef enum
{
    PihMotor=0,
    YawMotor=1,
    RamMotor=2,
    ShootLMotor=3,
    ShootRMotor=4,
    ShootUMotor=5,
    ScopeUMotor=6
}eMotors;

class cGimbal
{
public:
    cPID motors_pid[12];

    cMotor motors[7];

    cShoot shoot;

    cADRC adrc[3];

    cLowPassFilter lowfilter;

    cGimbal():
        motors_pid
                {
/*PihSpeedPid*/     {6,0,0       ,PID_DEFAULT_ERRALL_MAX,PID_DEFAULT_OUTPUT_MAX,PID_DEFAULT_OUTPUT_STEP_MAX,RAMPSTEP,Ramp_e,PositionPID_e},
/*PihPosPid*/       {0.5,0,0     ,PID_DEFAULT_ERRALL_MAX,PID_DEFAULT_OUTPUT_MAX,PID_DEFAULT_OUTPUT_STEP_MAX,RAMPSTEP,Ramp_e,PositionPID_e},
/*YawSpeedPid*/     {40,40,0     ,PID_DEFAULT_ERRALL_MAX,PID_DEFAULT_OUTPUT_MAX,PID_DEFAULT_OUTPUT_STEP_MAX,RAMPSTEP,Ramp_e,PositionPID_e},
/*YawPosPid*/       {1,0,0       ,PID_DEFAULT_ERRALL_MAX,PID_DEFAULT_OUTPUT_MAX,PID_DEFAULT_OUTPUT_STEP_MAX,RAMPSTEP,Ramp_e,PositionPID_e},
/*RamSpeedPid*/     {20,0,0      ,PID_DEFAULT_ERRALL_MAX,PID_DEFAULT_OUTPUT_MAX,PID_DEFAULT_OUTPUT_STEP_MAX,SHOOT_RAMPSTEP,Normal_e,PositionPID_e},
/*RamPosPid*/       {3,0,0       ,PID_DEFAULT_ERRALL_MAX,PID_DEFAULT_OUTPUT_MAX,PID_DEFAULT_OUTPUT_STEP_MAX,SHOOT_RAMPSTEP,Ramp_e,PositionPID_e},
/*ShootSpeedPidL*/  {7,0,0       ,PID_DEFAULT_ERRALL_MAX,PID_DEFAULT_OUTPUT_MAX,PID_DEFAULT_OUTPUT_STEP_MAX,SHOOT_RAMPSTEP,Ramp_e,PositionPID_e},
/*ShootSpeedPidR*/  {7,0,0       ,PID_DEFAULT_ERRALL_MAX,PID_DEFAULT_OUTPUT_MAX,PID_DEFAULT_OUTPUT_STEP_MAX,SHOOT_RAMPSTEP,Ramp_e,PositionPID_e},
/*ShootSpeedPidU*/  {7,0,0       ,PID_DEFAULT_ERRALL_MAX,PID_DEFAULT_OUTPUT_MAX,PID_DEFAULT_OUTPUT_STEP_MAX,SHOOT_RAMPSTEP,Ramp_e,PositionPID_e},
/*ChassisYawPid*/   {1.5,0,5       ,PID_DEFAULT_ERRALL_MAX,100,PID_DEFAULT_OUTPUT_STEP_MAX,RAMPSTEP,Normal_e,PositionPID_e},
/*ScopeUSpeedPid*/  {3,0.2,0      ,PID_DEFAULT_ERRALL_MAX,PID_DEFAULT_OUTPUT_MAX,PID_DEFAULT_OUTPUT_STEP_MAX,RAMPSTEP,Ramp_e,PositionPID_e},
/*ScopeUPosPid*/    {3,0,0.2      ,PID_DEFAULT_ERRALL_MAX,PID_DEFAULT_OUTPUT_MAX,PID_DEFAULT_OUTPUT_STEP_MAX,RAMPSTEP,Ramp_e,PositionPID_e},
                },
         motors
                {
/*PihMotor*/        {GM6020,CYBERGEAR,PIH_ANGLE,CYBERGEAR},
/*YawMotor*/        {GM6020,IMU_MODE,YAW_ANGLE,DAMIAO},
/*RamMotor*/        {M3508,ECD_MODE,RAM_ANGLE,NORMAL},
/*ShootLMotor*/     {M3508_OffReducer,ECD_MODE,NO_ANGLE,NORMAL},//纯速度环，无角度控制
/*ShootRMotor*/     {M3508_OffReducer,ECD_MODE,NO_ANGLE,NORMAL},
/*ShootUMotor*/     {M3508_OffReducer,ECD_MODE,NO_ANGLE,NORMAL},
/*ScopeUMotor*/     {M2006,ECD_MODE,ROLL_ANGLE,NORMAL},
                }
                {}

    void Gimbal_ControlLoop();          //云台控制主循环
    void Gimbal_ControlWithRC(void);    //遥控器操控云台

    void Gimbal_ShootMode(int8_t shoot_mode);
    void Gimbal_ControlMode(int8_t control_mode);
    void Gimbal_CarMode(int8_t car_mode);
    void Chassis_ComLoop(float vx,float vy,float vz,int8_t car_mode,int8_t aimbot_mode);

    void Gimbal_PosC();
    void Gimbal_SpeedC();

    void Gimbal_ParamChoose(int8_t mode);
    void setMotorSpeed(int WhichMotorPid,float spd);
    void setMotorPos(int WhichMotorPid,float angle);

    void Pitch_EcdLimit(float & Target);
    void Pitch_ImuLimit(float& Target);
    void Pitch_MILimit(float& Target);
    void Gimbal_KalmanInit(void);
    void Online_Check();
    void Printf_Test();

    ///车体模式变量///
    int8_t ControlMode;             //控制模式，左侧拨杆控制
    int8_t Last_ControlMode;                //上次的控制模式

    int8_t CarMode=SUIDONG;                 //车体模式，右侧拨杆控制
    int8_t Last_CarMode=PROTECT;            //上次的车体模式

    int8_t ProtectFlag=ONLINE;              //保护状态标志
    int8_t Last_ProtectFlag=ONLINE;         //上次的保护状态标志

    int8_t ShootMode=CLOSEFRIC;             //射击模式
    int8_t Last_ShootMode;                  //上次的射击模式

    int8_t ZimiaoFlag=0; //0代表关闭，1代表开启
    ///遥控器控制变量///
    float MousePih,MouseYaw,RCPih,RCYaw;
    float ChassisYawTarget=275;//随动模式下正方向的角度
    float vx, vy, vz, PihTarget=-66, YawTarget;//与遥控器交互用到的  车体运动参数与云台运动参数
    float ScopeUTarget;
    extKalman_t Gimbal_YawAngle, Gimbal_PihAngle, Gimbal_MouseX, Gimbal_MouseY,ZIMIAO_Yaw,ZIMIAO_Pih;//定义一个卡尔曼滤波器结构体

    ///射击控制变量//
    int8_t RammerStatus;
    int8_t FricStatus;
    int8_t RedrawStatus;
    int8_t AimbotMode=0;     //自瞄模式
    uint32_t Last_ID;        //自瞄的上次ID

    int8_t GimbalPower;
    ///分时发送控制变量///
    int32_t count_time_send=0;
private:
    //Pitch轴限幅
    float _Pitch_EcdUpLimit=-119;
    float _Pitch_EcdLowLimit=-163;

    float _Pitch_ImuUpLimit = 15;
    float _Pitch_ImuLowLimit=-20;

    float _Pitch_MIUpLimit=-28;
    float _Pitch_MILowLimit=-70;

    //遥控器接收到的上次数据
    int16_t RC_GetLastData=0;
    int16_t RC_CheckTimes=20;
};

extern cGimbal gimbal;



#endif //HERO_TEST_GIMBALC_H
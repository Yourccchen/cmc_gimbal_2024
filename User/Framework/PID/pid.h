//
// Created by DELL on 2023/9/23.
//

#ifndef HERO_TEST_PID_H
#define HERO_TEST_PID_H
#include "stm32f4xx_hal.h"

#define PID_DEFAULT_PRECISION 0.0f		 //控制精度，当目标速度与实际速度小于此值时，认为没有误差，使PID更稳定
#define PID_DEFAULT_ERRALL_MAX 3000		 //控制ERR_ALL最大值，否则ERR_ALL最大值过大，会使PID反应慢，不稳定，积分限幅
#define PID_DEFAULT_OUTPUT_MAX 10192	 //输出限幅
#define PID_DEFAULT_OUTPUT_STEP_MAX 3192 //输出微分限幅

typedef enum
{
    IncrementPID_e=0,  //增量式PID
    PositionPID_e=1,   //位置式PID
}PID_Type_e;

typedef enum
{
    Normal_e = 0,			// PID工作在正常状态
    Ramp_e = 1,				// PID工作在斜坡函数状态
} PID_RampState_e;

class cPID
{
public:
    cPID()=default;

    cPID(float kp, float ki, float kd,
         float PID_ErrAllMax, float PID_OutMax, float PID_OutStepMax, float PID_RampStep,
         bool PID_Mode ,bool PID_WorkType) :
        Kp(kp) ,Ki(ki) ,Kd(kd),
        PID_ErrAllMax(PID_ErrAllMax), PID_OutMax(PID_OutMax),PID_OutStepMax(PID_OutStepMax), PID_RampStep(PID_RampStep),
        PID_Mode(PID_Mode), PID_WorkType(PID_WorkType){}
///后续再完善类初始化的相关部分

    void PID_UpdateTarget();
    void PID_SetTarget(float target);
    float PID_GetPositionPID(float input);
    float PID_GetIncrementalPID(float input);
    void SetKpid(float kp,float ki,float kd);
    void SetMax(float errallmax,float outmax,float outstepmax);
    void PID_SpeedParamInit();
    void PID_PosParamInit();
    void PID_Clear();

    float PID_Target;       //当前目标值
    float PID_LastTarget;   //上次目标值
    float PID_Input;        //当前输入
    float PID_Out;          //当前输出
    float PID_RampStep;     //斜坡函数步幅（加速度）

    float Kp{};             //比例系数
    float Ki{};             //积分系数
    float Kd{};             //微分系数

    float PID_ErrNow;          //当前误差
    float PID_ErrLast;         //上次误差
    float PID_ErrLastLast;     //上上误差
    float PID_ErrAll;          //累计误差
    float PID_ErrAllMax;       //累计误差最大值

    float PID_RampTarget;   //斜坡目标值

    float PID_LastOut;      //上次输出
    float PID_OutMax;       //最大输出值
    float PID_OutStepMax;   //最大步进输出值
    float PID_Precision;    //PID最小精度

    bool PID_Mode;          //PID模式（斜坡计算或者普通计算）
    bool PID_WorkType;      //PID工作类型（位置式或者增量式）

private:

};

#endif //HERO_TEST_PID_H

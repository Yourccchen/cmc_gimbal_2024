//
// Created by DELL on 2023/9/23.
//

#include "pid.h"

void cPID::PID_SetTarget(float target)
{
    if(PID_Mode==Ramp_e)
        PID_RampTarget=target;
    else
        PID_Target=target;
}

/**
	* @name   PID_Update
	* @brief  更新PID的数据，即更新PID的输入值和误差值
*/
void cPID::PID_UpdateTarget()
{
    if(PID_Mode==Ramp_e)
    {
        if(PID_Target<PID_RampTarget)
        {
            PID_Target+=PID_RampStep;

            if(PID_Target>=PID_RampTarget)
                PID_Target=PID_RampTarget;
        }

        else if(PID_Target>PID_RampTarget)
        {
            PID_Target-=PID_RampStep;
            if(PID_Target<=PID_RampTarget)
                PID_Target=PID_RampTarget;
        }
    }
    else
        PID_Mode=Normal_e;

}
/**
	* @name   PID_GetPositionPID
	* @brief  位置式PID
    * @retval PID_Out输出值
*/
float cPID::PID_GetPositionPID(float input)
{

    PID_UpdateTarget();
    //传递当前输入并在后面计算误差值
    PID_Input=input;

    //位置式PID核心计算
    PID_Out =
            Kp * PID_ErrNow + Kd * (PID_ErrNow - PID_ErrLast);
    PID_Out += ( Ki * PID_ErrAll);

    //总输出限幅
    if (PID_Out >= PID_OutMax)
        PID_Out = PID_OutMax;
    if (PID_Out <= -PID_OutMax)
        PID_Out = -PID_OutMax;

    //传递并计算误差值
    PID_ErrLastLast = PID_ErrLast;
    PID_ErrLast = PID_ErrNow;
    PID_ErrNow = PID_Target - PID_Input;

    //在精准度范围内，可认为已到达目标值
    if (PID_ErrNow< PID_Precision && PID_ErrNow > -PID_Precision)
        PID_ErrNow = 0;

    //积分限幅
    PID_ErrAll += PID_ErrNow;
    if (PID_ErrAll > PID_ErrAllMax)
    {
        PID_ErrAll =PID_ErrAllMax;
    }
    else if (PID_ErrAll < -PID_ErrAllMax)
    {
        PID_ErrAll = -PID_ErrAllMax;
    }

    //微分限幅
    if (PID_Out > PID_LastOut + PID_OutStepMax)
    {
        PID_Out = PID_LastOut + PID_OutStepMax;
    }
    if (PID_Out < PID_LastOut - PID_OutStepMax)
    {
        PID_Out = PID_LastOut - PID_OutStepMax;
    }

    PID_LastOut = PID_Out;

    return PID_Out;
}

/**
	* @name   PID_GetIncrementalPID
	* @brief  增量式PID
	* @retval PID_Out输出值
*/
float cPID::PID_GetIncrementalPID(float input)
{
    PID_UpdateTarget();
    //传递当前输入并在后面计算误差值
    PID_Input=input;

    //增量式PID核心计算
    PID_Out +=
            Ki * PID_ErrNow + Kp * (PID_ErrNow - PID_ErrLast);
    PID_Out +=
            Kd * (PID_ErrNow - 2 * PID_ErrLast + PID_ErrLastLast);

    //总输出限幅
    if (PID_Out >= PID_OutMax)
        PID_Out = PID_OutMax;
    if (PID_Out <= -PID_OutMax)
        PID_Out = -PID_OutMax;

    //传递并计算误差值
    PID_ErrLastLast = PID_ErrLast;
    PID_ErrLast = PID_ErrNow;
    PID_ErrNow = PID_Target - PID_Input;

    //在精准度范围内，可认为已到达目标值
    if (PID_ErrNow< PID_Precision && PID_ErrNow > -PID_Precision)
        PID_ErrNow = 0;

    //积分限幅
    PID_ErrAll += PID_ErrNow;
    if (PID_ErrAll > PID_ErrAllMax)
    {
        PID_ErrAll =PID_ErrAllMax;
    }
    else if (PID_ErrAll < -PID_ErrAllMax)
    {
        PID_ErrAll = -PID_ErrAllMax;
    }

    //微分限幅
    if (PID_Out > PID_LastOut + PID_OutStepMax)
    {
        PID_Out = PID_LastOut + PID_OutStepMax;
    }
    if (PID_Out < PID_LastOut - PID_OutStepMax)
    {
        PID_Out = PID_LastOut - PID_OutStepMax;
    }

    PID_LastOut = PID_Out;

    return PID_Out;
}

/**
	* @name   SetKpid
	* @brief  设置PID的三个参数kp、ki、kd
	* @retval None
*/
void cPID::SetKpid(float kp,float ki,float kd)
{
    Kp=kp;
    Ki=ki;
    Kd=kd;
}
/**
	* @name   SetMax
	* @brief  设置PID的积分限幅、最大输出值、最大步进输出（微分）
	* @retval None
*/
void cPID::SetMax(float errallmax,float outmax,float outstepmax)
{
    PID_ErrAllMax=errallmax;
    PID_OutMax=outmax;
    PID_OutStepMax=outstepmax;
}
/**
	* @name   PID_SpeedParamInit
	* @brief  初始化PID速度环的参数
	* @retval None
*/
void cPID::PID_SpeedParamInit()
{ //初始化PID的默认参数
    Kp = 20.0;
    Ki = 1;
    Kd = 0;
    PID_ErrNow = 0.0;
    PID_ErrLast = 0.0;
    PID_ErrLastLast = 0.0;
    PID_ErrAll = 0.0;
    PID_Out = 0.0;
    PID_LastOut = 0.0;
    PID_Target = 0.0;
    PID_Input = 0.0;
    PID_RampTarget = 0.0;
    PID_RampStep=100.0;
    PID_Mode= Normal_e;
    PID_WorkType = PositionPID_e;
    PID_Precision = PID_DEFAULT_PRECISION;
    PID_ErrAllMax = PID_DEFAULT_ERRALL_MAX;
    PID_OutMax = PID_DEFAULT_OUTPUT_MAX;
    PID_OutStepMax = PID_DEFAULT_OUTPUT_STEP_MAX;
}

/**
	* @name   PID_PosParamInit
	* @brief  初始化PID位置环的参数
	* @retval None
*/
void cPID::PID_PosParamInit()
{ //初始化PID的默认参数
    Kp = 0.3;
    Ki = 0;
    Kd = 0;
    PID_ErrNow = 0.0;
    PID_ErrLast = 0.0;
    PID_ErrLastLast = 0.0;
    PID_ErrAll = 0.0;
    PID_Out = 0.0;
    PID_LastOut = 0.0;
    PID_Target = 0.0;
    PID_Input = 0.0;
    PID_RampTarget = 0.0;
    PID_RampStep=100.0;
    PID_Mode= Normal_e;
    PID_WorkType = PositionPID_e;
    PID_Precision = PID_DEFAULT_PRECISION;
    PID_ErrAllMax = PID_DEFAULT_ERRALL_MAX;
    PID_OutMax = PID_DEFAULT_OUTPUT_MAX;
    PID_OutStepMax = PID_DEFAULT_OUTPUT_STEP_MAX;
}

/**
	* @name   PID_Clear
	* @brief  PID参数清零
	* @retval None
*/
void cPID::PID_Clear()
{
    PID_ErrNow = 0.0;
    PID_ErrLast = 0.0;
    PID_ErrLastLast = 0.0;
    PID_ErrAll = 0.0;
    PID_Out = 0.0;
    PID_LastOut = 0.0;
}
//
// Created by Lenovo on 2024/3/15.
//

#include "Sliding.hpp"

void cSMC::Init()
{
    smc.J = 0;
    smc.K = 0;
    smc.c = 0;
    smc.epsilon = 0;
//    smc.flag = SAT;
    smc.u_max = 0;
    smc.limit = 0;

    smc.error.tar_now = 0;
    smc.error.tar_last = 0;
    smc.error.tar_differential = 0;

    smc.error.p_error = 0;
    smc.error.v_error = 0;
    smc.error.pos_error_eps = 0.1;
    smc.error.vol_error_eps = 0;
    smc.error.pos_get = 0;
    smc.error.vol_get = 0;
}

void cSMC::SetParam(float J, float K, float c, float epsilon, float limit, float u_max) {
    smc.J = J;
    smc.K = K;
    smc.c = c;
    smc.epsilon = epsilon;
    smc.u_max = u_max;
    smc.limit = limit;
}

void cSMC::ErrorUpdate(float target, float pos_now, float vel_now) //误差更新
{
    smc.error.tar_now = target;
    smc.error.tar_differential = (float)((smc.error.tar_now - smc.error.tar_last)/0.005);//0.005为控制周期
    smc.error.p_error = pos_now - target;
    smc.error.v_error = vel_now - smc.error.tar_differential;
    smc.error.tar_last = smc.error.tar_now;
}

float cSMC::SmcCalculate()
{
    float s,u,fun;

    if (std::abs(smc.error.p_error) - smc.error.pos_error_eps < 0)
    {
        smc.u = 0;
        return 0;
    }

    s = smc.c * smc.error.p_error + smc.error.v_error; //滑模面

    fun =  Sat(s);//饱和函数消除抖动

    u =  smc.J * ( (-smc.c * smc.error.tar_differential) - (smc.K * s + smc.epsilon * fun)); //控制器计算

    smc.error.error_last = smc.error.p_error; //更新上一步的误差
    //控制量限幅
    if (u > smc.u_max)
        u = smc.u_max;
    if (u < -smc.u_max)
        u = -smc.u_max;

    smc.u = u;
    return u;
}

// 符号函数
float cSMC::Signal(float s)
{
    if (s > 0)
        return 1;
    else if (s == 0)
        return 0;
    else
        return -1;
}

//饱和函数
float cSMC::Sat(float s)
{
    float y;
    y = s / smc.epsilon;
    if (std::abs(y) <= smc.limit)
        return y;
    else
        return Signal(y);
}

const Sliding &cSMC::getSmc() const {
    return smc;
}

float cSMC::Out() {
    return smc.u;
}




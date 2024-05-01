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
    smc.flag = EXPONENT;
    smc.u_max = 0;
    smc.limit = 0;

    smc.error.tar_now = 0;
    smc.error.tar_last = 0;
    smc.error.tar_differential = 0;

    smc.error.p_error = 0;
    smc.error.v_error = 0;
    smc.error.v_error_integral = 0;
    smc.error.pos_error_eps = 0.1f;
    smc.error.vol_error_eps = 0;
    smc.error.pos_get = 0;
    smc.error.vol_get = 0;
}

void cSMC::SetParam(float J, float K, float c, float epsilon, float limit, float u_max, Rmode flag) { ///EXPONENT,POWER,VELSMC 参数设定
    smc.J = J;
    smc.K = K;
    smc.c = c;
    smc.flag = flag;
    smc.epsilon = epsilon;
    smc.u_max = u_max;
    smc.limit = limit;
}
void cSMC::SetParam(float J, float K, float c, float p, float q, float beta, float epsilon, float limit, float u_max, Rmode flag) { ///TFSMC 参数设定
    smc.J = J;
    smc.K = K;
    smc.c = c;
    smc.p = p;
    smc.q = q;
    smc.beta = beta;
    smc.flag = flag;
    smc.epsilon = epsilon;
    smc.u_max = u_max;
    smc.limit = limit;
}

void cSMC::ErrorUpdate(float target, float pos_now, float vel_now) //位置环误差更新
{
    smc.error.tar_now = target;
    smc.error.tar_differential = (float)((smc.error.tar_now - smc.error.tar_last)/SAMPLE_PERIOD);

    smc.error.tar_differential_second = (float)((smc.error.tar_differential- smc.error.tar_differential_last)/SAMPLE_PERIOD); ///二阶导

    smc.error.p_error = pos_now - target;
    smc.error.v_error = vel_now - smc.error.tar_differential;
    smc.error.tar_last = smc.error.tar_now;

    smc.error.tar_differential_last = smc.error.tar_differential; ///二阶导更新

}

void cSMC::ErrorUpdate(float target,float vel_now) //速度环误差更新
{
    smc.error.tar_now = target;
    smc.error.tar_differential = (float)((smc.error.tar_now - smc.error.tar_last)/SAMPLE_PERIOD);
    smc.error.v_error = vel_now - smc.error.tar_now;
    smc.error.v_error_integral += (float)(smc.error.v_error * SAMPLE_PERIOD); ///速度误差积分项
    smc.error.tar_last = smc.error.tar_now;


}

float cSMC::SmcCalculate()
{
    float s,u,fun;
    float pos_pow,vel_pow;//ftsmc位置和速度的幂

    switch (smc.flag) {
        case EXPONENT:///线性滑模面，指数趋近率

            if (std::abs(smc.error.p_error) - smc.error.pos_error_eps < 0)
            {
                smc.u = 0;
                return 0;
            }

            s = smc.c * smc.error.p_error + smc.error.v_error; //滑模面
            fun = Sat(s);//饱和函数消除抖动
            u =  smc.J * ( (-smc.c * smc.error.tar_differential) - smc.K * s - smc.epsilon * fun + smc.error.tar_differential_second); //控制器计算,指数趋近率
            break;
        case POWER:///线性滑模面，幂次趋近率

            if (std::abs(smc.error.p_error) - smc.error.pos_error_eps < 0)
            {
                smc.u = 0;
                return 0;
            }

            s = smc.c * smc.error.p_error + smc.error.v_error; //滑模面
            fun = Sat(s);//饱和函数消除抖动
            u =  smc.J * ( (-smc.c * smc.error.tar_differential) - smc.K * s - smc.K* (std::pow(std::abs(s),smc.epsilon)) * fun + smc.error.tar_differential_second); //控制器计算,幂次趋近率
            break;
        case TSMC:///TSMC

            if (std::abs(smc.error.p_error) - smc.error.pos_error_eps < 0)
            {
                smc.u = 0;
                return 0;
            }

            pos_pow = std::pow(std::abs(smc.error.p_error),smc.q/smc.p);
            if(smc.error.p_error<=0) pos_pow = -pos_pow;

            vel_pow = std::pow(std::abs(smc.error.v_error),smc.q/smc.p);
            if(smc.error.v_error<=0) vel_pow = -vel_pow;

            s = smc.beta * pos_pow + smc.error.v_error; //滑模面

            fun = Sat(s);//饱和函数消除抖动

            if(smc.error.p_error!=0)
            {
                u = smc.J * (smc.error.tar_differential_second//目标值的二阶导
                             -smc.K * s //s*K
                             -smc.epsilon * fun  //epsilon*SAT(S)
                             -smc.beta * vel_pow
                             -((smc.q * smc.beta) * pos_pow) / (smc.p * smc.error.p_error)); //控制器计算
            }
            else u = 0;
            break;
        case VELSMC:///积分滑模面，指数趋近律，速度控制
            s = smc.error.v_error + smc.c * smc.error.v_error_integral; //滑模面
            fun = Sat(s);//饱和函数消除抖动
            u =  smc.J * (smc.error.tar_differential - (smc.c * smc.error.v_error) - smc.K * s - smc.epsilon * fun); //控制器计算，速度控制
            break;
    }
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




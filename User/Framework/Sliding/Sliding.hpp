//
// Created by Lenovo on 2024/3/15.
//

#ifndef KOSANN_UAVGIMBAL_SLIDING_HPP
#define KOSANN_UAVGIMBAL_SLIDING_HPP

#include <cmath>

typedef struct {

    float tar_now;
    float tar_last;
    float tar_differential;

    float pos_get;
    float vol_get;

    float p_error;
    float v_error;

    float pos_error_eps;   //误差精度
    float vol_error_eps;   //误差精度
    float error_last;
}RError;

typedef struct {
    float J;
    float K;
    float c;
    float u;
    float epsilon; //ε噪声上限
    RError error;
    float u_max;
    int flag; //符号和饱和切换，未用到
    float limit; //饱和函数上下限
}Sliding;

class cSMC
{
public:
    void Init();

    void SetParam(float J, float K, float c, float epsilon, float limit, float u_max); //滑膜参数设定
    void ErrorUpdate(float target, float pos_now, float vol_now); //滑膜误差更新
    float SmcCalculate(); //滑模控制器计算函数
    float Out();
    const Sliding &getSmc() const; //参量计算

private:
    Sliding smc;
    float Signal(float s); //符号函数
    float Sat(float s); //饱和函数
};
#endif //KOSANN_UAVGIMBAL_SLIDING_HPP

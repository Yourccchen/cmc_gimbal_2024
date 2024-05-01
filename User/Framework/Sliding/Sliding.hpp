//
// Created by Lenovo on 2024/3/15.
//

#ifndef KOSANN_UAVGIMBAL_SLIDING_HPP
#define KOSANN_UAVGIMBAL_SLIDING_HPP

#include <cmath>
#include "DebugC.h"

#define SAMPLE_PERIOD 0.005

typedef enum {
    EXPONENT,
    POWER,
    TSMC,
    VELSMC
} Rmode;

typedef struct {

    float tar_now;
    float tar_last;
    float tar_differential;
    float tar_differential_last;
    float tar_differential_second;

    float pos_get;
    float vol_get;

    float p_error;
    float v_error;

    float v_error_integral;

    float pos_error_eps;   //����
    float vol_error_eps;   //����
    float error_last;
}RError;

typedef struct {
    float J;
    float K;
    float c;
    float u;
    float p;    //������ p>q
    float q;    //������
    float beta; //����
    float epsilon; //����������
    RError error;
    float u_max;
    Rmode flag; //���źͱ����л���δ�õ�
    float limit; //���ͺ���������
}Sliding;

class cSMC
{
public:
    void Init();

    void SetParam(float J, float K, float c, float epsilon, float limit, float u_max, Rmode flag); //��Ĥ�����趨 EXPONENT,POWER,VOLSMC ����
    void SetParam(float J, float K, float c, float p, float q, float beta, float epsilon, float limit, float u_max, Rmode flag);//��Ĥ�����趨 EXPONENT,POWER,VOLSMC ����
    void ErrorUpdate(float target, float pos_now, float vol_now); //��Ĥλ��������
    void ErrorUpdate(float target,float vol_now); //��ģ�ٶ�������

    float SmcCalculate(); //��ģ���������㺯��
    float Out();
    const Sliding &getSmc() const; //��������

private:
    Sliding smc;
    float Signal(float s); //���ź���
    float Sat(float s); //���ͺ���
};
#endif //KOSANN_UAVGIMBAL_SLIDING_HPP

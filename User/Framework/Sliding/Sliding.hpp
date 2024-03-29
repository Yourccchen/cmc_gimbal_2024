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

    float pos_error_eps;   //����
    float vol_error_eps;   //����
    float error_last;
}RError;

typedef struct {
    float J;
    float K;
    float c;
    float u;
    float epsilon; //����������
    RError error;
    float u_max;
    int flag; //���źͱ����л���δ�õ�
    float limit; //���ͺ���������
}Sliding;

class cSMC
{
public:
    void Init();

    void SetParam(float J, float K, float c, float epsilon, float limit, float u_max); //��Ĥ�����趨
    void ErrorUpdate(float target, float pos_now, float vol_now); //��Ĥ������
    float SmcCalculate(); //��ģ���������㺯��
    float Out();
    const Sliding &getSmc() const; //��������

private:
    Sliding smc;
    float Signal(float s); //���ź���
    float Sat(float s); //���ͺ���
};
#endif //KOSANN_UAVGIMBAL_SLIDING_HPP

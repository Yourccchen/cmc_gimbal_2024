#ifndef HERO_TEST_ADRC_H
#define HERO_TEST_ADRC_H
#include "math.h"

class cADRC
{
public:
///类初始化
//    cADRC(float r, float h,float b,float delta,
//          float beta01, float beta02, float beta03,
//          float alpha1 ,float alpha2,float betac1,float betac2) :
//            r(r) ,h(h) ,b(b), delta(delta),
//            beta01(beta01),beta02(beta02), beta03(beta03),
//            alpha1(alpha1), alpha2(alpha2), betac1(betac1),betac2(betac2)
//    {}
    //TD的两个输出
    float x1;
    float x2;
    //ESO的三个输出
    float z1;
    float z2;
    float z3;

    float Target;   //目标值
    float Feedback; //反馈

    float delta;//一般不影响输出，在0.01~0.1之间选取，过大会产生震荡

    float u0;//非线性组合输出值

    float u; //输出给被控对象的最终值

    //TD
    float r; //r与跟踪速度呈正相关，然而，随之带来的是噪声放大的副作用
    float h; //h与滤波效果呈正相关，但当h增大时，跟踪信号的相位损失也会随之增加。1ms调用一次则是h=0.001

    //ESO
    float epsilon;
    float b; //补偿系数，ADRC的灵魂
    float beta01;
    float beta02;
    float beta03;

    //非线性组合
    float alpha1;// 0<alpha1<1<alpha2
    float alpha2;
    float betac1; //Kp
    float betac2; //Kd

    float sgn(float x);
    float fal(float e, float alpha, float delta);
    float fsg(float x, float y);
    float fhan(float x1, float x2, float r, float h);

    void  ADRC_TD();
    void  ADRC_ESO();
    void  ADRC_NonLinerFeedBack();

    void ADRCParamInit(float _r,float _h,float _b,float _delta,
                              float _beta01,float _betal02,float _betal03,
                              float _alpha1,float _alpha2,float _betac1,float _betac2);

    float ADRC_Calc(float target, float feedback);


};

#endif //HERO_TEST_ADRC_H

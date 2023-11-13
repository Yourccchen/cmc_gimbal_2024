#include "ADRC.h"
#include "debugc.h"
/**ADRC算法参数记录
 * ESO共有b、delta、belta01、belta02、belta03共5个参数，其中delta取值范围在5h<=delta<=10h,h为ADRC控制周期。
 * 参数整定可以先将b定下来，比如取1或者2（最好还是能够知道你的二阶系统系数）
 * 然后先后调整belta01、belta02、belta03
 * 观测z1能不能够很好的跟随反馈y，如果是，那么大概参数就调好了；
 * 如果不是，可以改动一下b，还是不行的话就得认认真真的检测一下反馈y是不是出了什么问题，
 * 比如变量数据类型转换有没有做好。如果懂得自己在输出中加入随机数（白噪声），注意幅值不能过大，
 * 观测一下z3是不是能够很好的观测到随机扰动。若以上两个条件都成立，那么ADRC就几乎被整定好了。
 * NLSEF参数有alpha1，alpha2，belta1，belta2四个，其中0<alpha1<1<alpha2。
 * belta1和belta2则视效果而定，通常ESO和NLSEF一起调，在整定ESO参数时，可以先把delta1和delta2定为1，
 * 再调ESO，待ESO有一定效果后，反复调整ESO参数无果，可以加入NLSEF参数整定，取得更好的效果。

 * 调参顺序:TD -> NONLINER FEEDBACK <--> ESO（ ESO 和 NONLINER FEEDBACK 联调）
 *      ADRC_motor.r = 200;
        ADRC_motor.h = 0.05;
        ADRC_motor.delta = 0.5;
        ADRC_motor.b = 5;
        ADRC_motor.beta01 = 1;
        ADRC_motor.beta02 = 300;
        ADRC_motor.beta03 = 5;
        ADRC_motor.alpha1 = 0.5;
        ADRC_motor.alpha2 = 1.25;
        ADRC_motor.betac1 = 80;
        ADRC_motor.betac2 = 1;
 */

/**
 * sgn 函数
 * @param x
 * @return
 */
float cADRC::sgn(float x)
{
    float sgn_out;
    if (x > 0) sgn_out = 1;
    else if (x < 0) sgn_out = -1;
    else sgn_out = 0;

    return sgn_out;
}

/**
 * fal 函数
 * @param e
 * @param alpha
 * @param delta
 * @return
 */
float cADRC::fal(float e, float alpha, float delta)
{
    float fal_out;
    if (fabsf(e) > delta) fal_out = powf(fabsf(e), alpha) * sgn(e);
    else fal_out = e / (powf(delta, 1 - alpha));
    return fal_out;
}

/**
 * fsg 函数
 * @param x
 * @param y
 * @return
 */
float cADRC::fsg(float x, float y)
{
    float fsg_out;
    fsg_out = (sgn(x + y) - sgn(x - y)) / 2;
    return fsg_out;
}

/**
 * fhan 函数
 * @param x1
 * @param x2
 * @param r
 * @param h
 * @return
 */
float cADRC::fhan(float x1, float x2, float r, float h)
{
    float d, a0, a1, a2, a, y, fhan_out;
    d = powf(h, 2) * r;
    a0 = h * x2;
    y = x1 + a0;
    a1 = sqrtf(d * (d + 8 * fabsf(y)));
    a2 = a0 + sgn(y) * (a1 - d) * 0.5;
    a = (a0 + y) * fsg(y, d) + a2 * (1 - fsg(y, d));
    fhan_out = -r * (a / d) * fsg(y, d) - r * sgn(a) * (1 - fsg(a, d));
    return fhan_out;
}

/**
 * TD微分跟踪器
 * @param which
 */
void cADRC::ADRC_TD()
{
    float fh;
    fh = fhan(x1 - Target, x2, r, h);
    x1 += h * x2;
    x2 += h * fh;
}

/**
 * ESO自干扰观测器
 * @param which
 */
void cADRC::ADRC_ESO()
{
    float fe, fe1;
    epsilon = z1 - Feedback;
    fe = fal(epsilon, 0.5, delta);
    fe1 = fal(epsilon, 0.25, delta);
    z1 += h * (z2 - beta01 * epsilon);
    z2 += h * (z3 - beta02 * fe + u * b);
    z3 += h * (fe1 * beta03 * (-1));
/******************限幅，ADRC正常的话不会达到限幅条件********************/
    if(z1>=30000) z1=30000;
    if(z1<=-30000) z1 = -30000;
    if(z2>=30000) z2=30000;
    if(z2<=-30000) z2 = -30000;
    if(z3>=30000) z3=30000;
    if(z3<=-30000) z3 = -30000;
}

/**
 * 非线性反馈 PD组合
 * @param which
 */
void cADRC::ADRC_NonLinerFeedBack()
{
    float e1, e2;
    e1 = x1 - z1;
    e2 = x2 - z2;
    u0 =   betac1 * fal(e1, alpha1, delta)
         + betac2 * fal(e2, alpha2, delta);
    u = u0 - z3 / b;
}

/**
 * ADRC参数初始化
 * @param which
 */
void cADRC::ADRCParamInit()
{
    x1 = 0;//跟踪输入
    x2 = 0;//跟踪输入的微分
    z1 = 0;//跟踪反馈值
    z2 = 0;//跟踪反馈值的微分
    z3 = 0;//跟踪系统的扰动（总扰动）
    epsilon = 0;//误差值

    Target = 0;//目标值
    Feedback = 0;//反馈值

    u0 = 0;//非线性组合输出值
    u = 0;//最终输出值

    //TD
    r = 1500;//r与跟踪速度呈正相关，然而，随之带来的是噪声放大的副作用。
    h = 0.05;//h与滤波效果呈正相关，但当h增大时，跟踪信号的相位损失也会随之增加。

    //ESO
    b = 5;
    beta01 = 1;
    beta02 = 1000;
    beta03 = 10;
    delta = 0.05;// 5*h<=delta<=10*h

    //NLSEF
    alpha1 = 0.5;// 0<alpha1<1<alpha2
    alpha2 = 1.25;
    betac1 = 80;
    betac2 = 1;
}

/**
 * ADRC执行过程
 * @param which
 * @param target    目标值
 * @param feedback  反馈值
 * @return 计算出来的输出值
 */
float cADRC::ADRC_Calc( float target, float feedback)
{
    Target = target;
    Feedback = feedback;
    ADRC_TD();
    ADRC_ESO();
    ADRC_NonLinerFeedBack();
    return u;
}




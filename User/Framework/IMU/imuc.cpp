//
// Created by DELL on 2023/11/20.
//

#include "imuc.h"
static float Pih_Angle,Yaw_Angle,Roll_Angle;
static float Pih_Speed,Yaw_Speed,Roll_Speed;

static int32_t rotate_times;//圈数
static float last_angle;//上次角度

/**
	* @name   IMU_AngleIncreLoop
	* @brief  IMU角度连续化处理
	* @param  angle_now 当前IMU反馈角度
	* @retval angle_now 连续化后的IMU反馈角度
*/
float IMU_AngleIncreLoop(float now_angle)
{
    float this_angle;
    this_angle=now_angle;
    if((this_angle-last_angle)>300)
        rotate_times--;
    if((this_angle-last_angle)<-300)
        rotate_times++;
    now_angle=this_angle+rotate_times*360.0f;
    last_angle=this_angle;
    return now_angle;
}
/**
	* @name   IMU_Angle
	* @brief  返回IMU_Angle,1为PIH轴角度，2为YAW轴角度,3为ROLL轴角度
	* @param  Witch_angle
	* @retval 角度
*/
float IMU_Angle(int8_t Witch_angle)
{
    const float *imuAngle = get_INS_angle_point();
    Pih_Angle=imuAngle[INS_PITCH_ADDRESS_OFFSET] * rad2degree;

    Yaw_Angle=IMU_AngleIncreLoop(imuAngle[INS_YAW_ADDRESS_OFFSET] * rad2degree);

    Roll_Angle=imuAngle[INS_ROLL_ADDRESS_OFFSET] * rad2degree;
    switch (Witch_angle)
    {
        case 1:
            return Pih_Angle;
        case 2:
            return Yaw_Angle;
        case 3:
            return Roll_Angle;
    }
}
/**
	* @name   IMU_Speed
	* @brief  返回IMU_Speed，1为x轴速度，2为y轴速度，3为ROLL轴速度
	* @param  Witch_speed
	* @retval 速度
*/
float IMU_Speed(int8_t Witch_angle)
{
    const float *imuGyro = get_gyro_data_point();
    switch (Witch_angle)
    {
        case 1:
            return imuGyro[INS_GYRO_X_ADDRESS_OFFSET] * rad2rpm;
        case 2:
            return imuGyro[INS_GYRO_Y_ADDRESS_OFFSET] * rad2rpm;
        case 3:
            return imuGyro[INS_GYRO_Z_ADDRESS_OFFSET] * rad2rpm;
    }
}
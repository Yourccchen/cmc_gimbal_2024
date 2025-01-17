#include "dm4310_drv.h"
#include "dm4310_ctrl.h"
#include "can_driver.h"
#include "string.h"

///Motor1是YAW轴，Motor2是PITCH轴
motor_t motor[num];
int lcd_flag;
int8_t motor_id = 1;

/**
************************************************************************
* @brief:      	dm4310_motor_init: DM4310电机初始化函数
* @param:      	void
* @retval:     	void
* @details:    	初始化三个DM4310型号的电机，设置默认参数和控制模式。
*               分别初始化Motor1和Motor2，设置ID、控制模式和命令模式等信息。
************************************************************************
**/
void dm4310_motor_init(void)
{
	// 初始化Motor1和Motor2的电机结构
	memset(&motor[Motor1], 0, sizeof(motor[Motor1]));
	memset(&motor[Motor2], 0, sizeof(motor[Motor2]));
//	memset(&motor[Motor3], 0, sizeof(motor[Motor2]));

	// 设置Motor1的电机信息
	motor[Motor1].id = 0x01;
	motor[Motor1].ctrl.mode = 0;		// 0: MIT模式   1: 位置速度模式   2: 速度模式
	motor[Motor1].cmd.mode = 0;

	// 设置Motor2的电机信息
	motor[Motor2].id = 0x02;
	motor[Motor2].ctrl.mode = 1;
	motor[Motor2].cmd.mode = 1;

	// 设置Motor3的电机信息
//	motor[Motor3].id = 2;
//	motor[Motor3].ctrl.mode = 2;
//	motor[Motor3].cmd.mode = 2;
}
/**
************************************************************************
* @brief:      	motor_para_add: 修改电机参数函数
* @param[in]:   motor:   指向motor_t结构的指针，包含电机参数信息
* @retval:     	void
* @details:    	根据当前LCD标志位（lcd_flag），修改指定电机的参数信息。
*               根据lcd_flag的不同，可修改电机ID、控制模式、位置设定、速度设定、
*               扭矩设定、比例增益和微分增益等参数。
************************************************************************
**/
void motor_para_add(motor_t *motor)
{
	switch(lcd_flag)
	{
		case 0:
			motor_id += 1;
			if (motor_id > num)
				motor_id = 1;
			break;
		case 1:
			motor->ctrl.mode += 1;
			if (motor->ctrl.mode > 2)
				motor->ctrl.mode = 0;
			break;
		case 2:
			motor->cmd.pos_set += 1.0f;
			break;
		case 3:
			motor->cmd.vel_set += 1.0f;
			break;
		case 4:
			motor->cmd.tor_set += 1.0f;
			break;
		case 5:
			motor->cmd.kp_set += 1.0f;
			break;
		case 6:
			motor->cmd.kd_set += 0.5f;
			break;
	}
}
/**
************************************************************************
* @brief:      	motor_para_minus: 减小电机参数函数
* @param[in]:   motor:   指向motor_t结构的指针，包含电机参数信息
* @retval:     	void
* @details:    	根据当前LCD标志位（lcd_flag），减小指定电机的参数信息。
*               根据lcd_flag的不同，可减小电机ID、控制模式、位置设定、速度设定、
*               扭矩设定、比例增益和微分增益等参数。
************************************************************************
**/
void motor_para_minus(motor_t *motor)
{

	switch(lcd_flag)
	{
		case 0:
			motor_id -= 1;
			if (motor_id < 1)
				motor_id = num;
			break;
		case 1:
			motor->ctrl.mode -= 1;
			if (motor->ctrl.mode < 0)
				motor->ctrl.mode = 2;
			break;
		case 2:
			motor->cmd.pos_set -= 1.0f;
			break;
		case 3:
			motor->cmd.vel_set -= 1.0f;
			break;
		case 4:
			motor->cmd.tor_set -= 1.0f;
			break;
		case 5:
			motor->cmd.kp_set -= 1.0f;
			break;
		case 6:
			motor->cmd.kd_set -= 0.5f;
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_enable: 启用电机控制函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），启用对应的电机控制。
*               设置指定电机的启动标志，并调用dm4310_enable函数启用电机。
************************************************************************
**/
void ctrl_enable_yaw(void)
{
    // 启用Motor1、Motor2的电机控制
    motor[Motor1].start_flag = 1;
    dm4310_enable(&hcan2, &motor[Motor1]);//Yaw轴电机

}
void ctrl_enable_pitch(void)
{
    // 启用Motor1、Motor2的电机控制
    motor[Motor2].start_flag = 1;
    dm4310_enable(&hcan1, &motor[Motor2]);//Pitch轴电机
}
/**
************************************************************************
* @brief:      	ctrl_disable: 禁用电机控制函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），禁用对应的电机控制。
*               设置指定电机的启动标志为0，并调用dm4310_disable函数禁用电机。
************************************************************************
**/
void ctrl_disable_yaw(void)
{
    // 禁用Motor1、Motor2的电机控制
    motor[Motor1].start_flag = 1;
    dm4310_disable(&hcan2, &motor[Motor1]);//Yaw轴电机
}

void ctrl_disable_pitch(void)
{
    motor[Motor2].start_flag = 1;
    dm4310_disable(&hcan1, &motor[Motor2]);//Pitch轴电机
}
/**
************************************************************************
* @brief:      	ctrl_set: 设置电机参数函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），设置对应电机的参数。
*               调用dm4310_set函数设置指定电机的参数，以响应外部命令。
************************************************************************
**/
void ctrl_set(void)
{
//	switch(motor_id)
//	{
//		case 1:
			// 设置Motor1的电机参数
			dm4310_set(&motor[Motor1]);
//			break;
//	}
}
/**
************************************************************************
* @brief:      	ctrl_posvelset: 设置电机位置速度参数设置函数
* @param:       PosSet,VelSet
* @retval:     	void
* @details:    	设置电机位置、最大速度
************************************************************************
**/
void ctrl_posvelset(float PosSet,float VelSet)
{
    motor[Motor2].ctrl.pos_set=PosSet*deg2rad;
    motor[Motor2].ctrl.vel_set=VelSet;
}
/**
************************************************************************
* @brief:      	ctrl_velset: 设置电机力矩函数
* @param:      	TorSet 要设置的电机转动力矩
* @retval:     	void
* @details:    	设置电机力矩
************************************************************************
**/
void ctrl_torset(float TorSet)
{
    motor[Motor1].ctrl.tor_set=TorSet;
}
/**
************************************************************************
* @brief:      	ctrl_clear_para: 清除电机参数函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），清除对应电机的参数。
*               调用dm4310_clear函数清除指定电机的参数，以响应外部命令。
************************************************************************
**/
void ctrl_clear_para(void)
{
	switch(motor_id)
	{
		case 1:
			// 清除Motor1的电机参数
			dm4310_clear_para(&motor[Motor1]);
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_clear_err: 清除电机错误信息
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），清除对应电机的参数。
*               调用dm4310_clear函数清除指定电机的参数，以响应外部命令。
************************************************************************
**/
void ctrl_clear_err(void)
{
	switch(motor_id)
	{
		case 1:
			// 清除Motor1的电机错误参数
			dm4310_clear_err(&hcan1, &motor[Motor1]);
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_add: 增加电机参数函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），增加对应电机的参数。
*               调用motor_para_add函数增加指定电机的参数，以响应外部命令。
************************************************************************
**/
void ctrl_add(void)
{
	switch(motor_id)
	{
		case 1:
			// 增加Motor1的电机参数
			motor_para_add(&motor[Motor1]);
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_minus: 减少电机参数函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），减少对应电机的参数。
*               调用motor_para_minus函数减少指定电机的参数，以响应外部命令。
************************************************************************
**/
void ctrl_minus(void)
{
	switch(motor_id)
	{
		case 1:
//			 减少Motor1的电机参数
			motor_para_minus(&motor[Motor1]);
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_send: 发送电机控制命令函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），向对应电机发送控制命令。
*               调用dm4310_ctrl_send函数向指定电机发送控制命令，以响应外部命令。
************************************************************************
**/
void ctrl_send(void)
{
//	switch(motor_id)
//	{
//		case 1:
			 // 向Motor1发送控制命令
            dm4310_ctrl_send(&hcan2, &motor[Motor1]);
            dm4310_ctrl_send(&hcan1, &motor[Motor2]);
//			break;
//	}
}
/**
************************************************************************
* @brief:      	can1_rx_callback: CAN2接收回调函数
* @param:      	void
* @retval:     	void
* @details:    	处理CAN2接收中断回调，根据接收到的ID和数据，执行相应的处理。
*               当接收到ID为0时，调用dm4310_fbdata函数更新Motor的反馈数据。
************************************************************************
**/
void can2_rx_callback(void)
{
	uint16_t rec_id;
	uint8_t rx_data[8] = {0};
	canx_receive_data(&hcan2, &rec_id, rx_data);
    switch(rec_id)
    {
        case 0xB:dm4310_fbdata(&motor[Motor1], rx_data);break;
    }
}

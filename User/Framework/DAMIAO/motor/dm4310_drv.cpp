#include "dm4310_drv.h"
#include "can_driver.h"
#include "can.h"
/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int Float_To_Uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float Uint_To_Float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
************************************************************************
* @brief:      	dm4310_enable: 启用DM4310电机控制模式函数
* @param[in]:   hcan:    指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
* @retval:     	void
* @details:    	根据电机控制模式启用相应的模式，通过CAN总线发送启用命令
*               支持的控制模式包括位置模式、位置速度控制模式和速度控制模式
************************************************************************
**/
void dm4310_enable(hcan_t* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case 0:
			enable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
		case 1:
			enable_motor_mode(hcan, motor->id, POS_MODE);
			break;
		case 2:
			enable_motor_mode(hcan, motor->id, SPEED_MODE);
			break;
	}	
}
/**
************************************************************************
* @brief:      	dm4310_disable: 禁用DM4310电机控制模式函数
* @param[in]:   hcan:    指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
* @retval:     	void
* @details:    	根据电机控制模式禁用相应的模式，通过CAN总线发送禁用命令
*               支持的控制模式包括位置模式、位置速度控制模式和速度控制模式
************************************************************************
**/
void dm4310_disable(hcan_t* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case 0:
			disable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
		case 1:
			disable_motor_mode(hcan, motor->id, POS_MODE);
			break;
		case 2:
			disable_motor_mode(hcan, motor->id, SPEED_MODE);
			break;
	}	
	dm4310_clear_para(motor);
}
/**
************************************************************************
* @brief:      	dm4310_ctrl_send: 发送DM4310电机控制命令函数
* @param[in]:   hcan:    指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
* @retval:     	void
* @details:    	根据电机控制模式发送相应的命令到DM4310电机
*               支持的控制模式包括位置模式、位置速度控制模式和速度控制模式
************************************************************************
**/
void dm4310_ctrl_send(hcan_t* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case 0:
			mit_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set);
			break;
		case 1:
			pos_speed_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
			break;
		case 2:
			speed_ctrl(hcan, motor->id, motor->ctrl.vel_set);
			break;
	}	
}
/**
************************************************************************
* @brief:      	dm4310_set: 设置DM4310电机控制参数函数
* @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
* @retval:     	void
* @details:    	根据命令参数设置DM4310电机的控制参数，包括位置、速度、
*               比例增益(KP)、微分增益(KD)和扭矩
************************************************************************
**/
void dm4310_set(motor_t *motor)
{
    motor->ctrl.kp_set	= 0;
	motor->ctrl.kd_set 	= 0;
	motor->ctrl.pos_set	= 0;
	motor->ctrl.vel_set	= 0;
	motor->ctrl.tor_set	= 0;

}
/**
************************************************************************
* @brief:      	dm4310_clear: 清除DM4310电机控制参数函数
* @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
* @retval:     	void
* @details:    	将DM4310电机的命令参数和控制参数清零，包括位置、速度、
*               比例增益(KP)、微分增益(KD)和扭矩
************************************************************************
**/
void dm4310_clear_para(motor_t *motor)
{
	motor->cmd.kd_set 	= 0;
	motor->cmd.kp_set	= 0;
	motor->cmd.pos_set 	= 0;
	motor->cmd.vel_set 	= 0;
	motor->cmd.tor_set 	= 0;
	
	motor->ctrl.kd_set 	= 0;
	motor->ctrl.kp_set	= 0;
	motor->ctrl.pos_set = 0;
	motor->ctrl.vel_set = 0;
	motor->ctrl.tor_set = 0;
}
/**
************************************************************************
* @brief:      	dm4310_clear_err: 清除DM4310电机错误函数
* @param[in]:   hcan: 	 指向CAN控制结构体的指针
* @param[in]:  	motor:   指向电机结构体的指针
* @retval:     	void
* @details:    	根据电机的控制模式，调用对应模式的清除错误函数
************************************************************************
**/
void dm4310_clear_err(hcan_t* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case 0:
			clear_err(hcan, motor->id, MIT_MODE);
			break;
		case 1:
			clear_err(hcan, motor->id, POS_MODE);
			break;
		case 2:
			clear_err(hcan, motor->id, SPEED_MODE);
			break;
	}	
}

static float IMU_AngleIncreLoop(float angle_now)
{
    static float last_angle;
    static int32_t rotate_times;

    float this_angle;
    this_angle = angle_now;
    if ((this_angle - last_angle) > 300)
        rotate_times--;
    if ((this_angle - last_angle) < -300)
        rotate_times++;
    angle_now = this_angle + rotate_times * 360.0f;
    last_angle = this_angle;
    return angle_now;
}
/**
************************************************************************
* @brief:      	dm4310_fbdata: 获取DM4310电机反馈数据函数
* @param[in]:   motor:    指向motor_t结构的指针，包含电机相关信息和反馈数据
* @param[in]:   rx_data:  指向包含反馈数据的数组指针
* @retval:     	void
* @details:    	从接收到的数据中提取DM4310电机的反馈信息，包括电机ID、
*               状态、位置、速度、扭矩以及相关温度参数
************************************************************************
**/
void dm4310_fbdata(motor_t *motor, uint8_t *rx_data)
{
	motor->para.id = (rx_data[0])&0x0F;
	motor->para.state = (rx_data[0])>>4;
	motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	motor->para.pos = Uint_To_Float(motor->para.p_int, P_MIN, P_MAX, 16);
	motor->para.vel = Uint_To_Float(motor->para.v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
	motor->para.tor = Uint_To_Float(motor->para.t_int, T_MIN, T_MAX, 12);  // (-18.0,18.0)
	motor->para.Tmos = (float)(rx_data[6]);
	motor->para.Tcoil = (float)(rx_data[7]);

    motor->para.angle=motor->para.pos*rad2deg;
    motor->para.speed=motor->para.vel*rad2deg;
}


/**
************************************************************************
* @brief:      	enable_motor_mode: 启用电机模式函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要开启的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送启用特定模式的命令
************************************************************************
**/
int count_number=0;
void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;

//    if(count_number % 100 ==0)
//    {
//        HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
//        count_number = 0;
//    }
//    count_number++;
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	disable_motor_mode: 禁用电机模式函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要禁用的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送禁用特定模式的命令
************************************************************************
**/
void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	save_pos_zero: 保存位置零点函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要保存位置零点的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送保存位置零点的命令
************************************************************************
**/
void save_pos_zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFE;
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	clear_err: 清除电机错误函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要清除错误的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送清除错误的命令。
************************************************************************
**/
void clear_err(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFB;
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	mit_ctrl: MIT模式下的电机控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = Float_To_Uint(pos,  P_MIN,  P_MAX,  16);
	vel_tmp = Float_To_Uint(vel,  V_MIN,  V_MAX,  12);
	kp_tmp  = Float_To_Uint(kp,   KP_MIN, KP_MAX, 12);
	kd_tmp  = Float_To_Uint(kd,   KD_MIN, KD_MAX, 12);
	tor_tmp = Float_To_Uint(torq, T_MIN,  T_MAX,  12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	pos_speed_ctrl: 位置速度控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	    电机ID，指定目标电机
* @param[in]:   vel:			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
void pos_speed_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	
	id = motor_id + POS_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	data[6] = *(vbuf+2);
	data[7] = *(vbuf+3);
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	speed_ctrl: 速度控制函数
* @param[in]:   hcan: 		指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   vel: 			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送速度控制命令
************************************************************************
**/
void speed_ctrl(hcan_t* hcan,uint16_t motor_id, float vel)
{
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];
	
	id = motor_id + SPEED_MODE;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *vbuf;
	data[1] = *(vbuf+1);
	data[2] = *(vbuf+2);
	data[3] = *(vbuf+3);
	
	canx_send_data(hcan, id, data, 4);
}

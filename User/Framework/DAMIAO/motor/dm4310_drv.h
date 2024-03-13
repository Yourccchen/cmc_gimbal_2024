#ifndef __DM4310_DRV_H__
#define __DM4310_DRV_H__
#include "main.h"
#include "can.h"
#include "bsp_can.h"

#define PI 3.14159265f
#define rad2deg 180.0f/PI
#define deg2rad PI/180.0f
#define rad2rpm 30.0f/PI
#define rpm2rad PI/30.0f
#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPEED_MODE		0x200

#define P_MIN -3.14159265f
#define P_MAX 3.14159265f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -30000.0f
#define T_MAX 30000.0f

// 电机回传信息结构体
typedef struct 
{
	int id;
	int state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
    float angle; //自定义，连续化后，单位为°
	float vel;
    float speed; //自定义，单位为°/s
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
}motor_fbpara_t;

// 电机参数设置结构体
typedef struct 
{
	int8_t mode;
	float pos_set;
	float vel_set;
	float tor_set;
	float kp_set;
	float kd_set;
}motor_ctrl_t;

typedef struct
{
	int8_t id;
	uint8_t start_flag;
	motor_fbpara_t para;
	motor_ctrl_t ctrl;
	motor_ctrl_t cmd;
}motor_t;

void dm4310_ctrl_send(hcan_t* hcan, motor_t *motor);
void dm4310_enable(hcan_t* hcan, motor_t *motor);
void dm4310_disable(hcan_t* hcan, motor_t *motor);
void dm4310_set(motor_t *motor);
void dm4310_clear_para(motor_t *motor);
void dm4310_clear_err(hcan_t* hcan, motor_t *motor);
void dm4310_fbdata(motor_t *motor, uint8_t *rx_data);
float Uint_To_Float(int x_int, float x_min, float x_max, int bits);

void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
void pos_speed_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel);
void speed_ctrl(hcan_t* hcan,uint16_t motor_id, float _vel);
void save_pos_zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
void clear_err(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);

#endif /* __DM4310_DRV_H__ */


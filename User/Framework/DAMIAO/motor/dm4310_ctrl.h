#ifndef __DM4310_CTRL_H__
#define __DM4310_CTRL_H__
#include "main.h"
#include "dm4310_drv.h"

extern int8_t motor_id;

typedef enum
{
	Motor1,
	Motor2,
	Motor3,
	num
} motor_num;

extern motor_t motor[num];

void dm4310_motor_init(void);
void ctrl_enable_yaw(void);
void ctrl_enable_pitch(void);
void ctrl_disable_yaw(void);
void ctrl_disable_pitch(void);
void ctrl_set(void);
void ctrl_clear_para(void);
void ctrl_clear_err(void);
void ctrl_add(void);
void ctrl_minus(void);
void ctrl_send(void);
void ctrl_posvelset(float PosSet,float VelSet);
void ctrl_torset(float TorSet);
#endif /* __DM4310_CTRL_H__ */


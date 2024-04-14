//
// Created by DELL on 2023/9/22.
//

#ifndef HERO_TEST_BSP_CAN_H
#define HERO_TEST_BSP_CAN_H
#include "stm32f4xx_hal.h"
#include "motorc.h"

#ifdef __cplusplus
extern "C"
{
#endif
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
#ifdef __cplusplus
}
#endif

//Pitch CAN1
//Yaw CAN2
typedef enum
{
    CAN_YAW_RCV_ID = 0x209,
    CAN_YAW_SEND_ID = 0x2FF,

    CAN_PIH_RCV_ID = 0x20A,
    CAN_PIH_SEND_ID = 0x2FF, //0x20A

    //注释分别对应DATA[i]&[i+1]中的内容
    CAN_CHASSIS_VAL_ID = 0x401, //x轴速度 y轴速度 z轴速度 模式
    CAN_CHASSIS_YAW_ID = 0x402, //yaw轴角度 pitch轴角度 状态标志位 车间标志位

    CAN_SHOOT_LEFT_ID = 0x201,  //英雄左摩擦轮,ID为1
    CAN_SHOOT_RIGHT_ID = 0x202, //英雄右摩擦轮,ID为2
    CAN_RAMC_ID = 0X203,        //英雄拨弹轮,ID为3
    CAN_SHOOT_UP_ID = 0X204,    //英雄上摩擦轮,ID为4
    CAN_SCOPE_UP_ID=0x206,
    CAN_SCOPE_DOWN_ID=0x207,

    CAN_SHOOT_SEND_ID = 0x200,
    CAN_SCOPE_SEND_ID = 0x1FF,

    CAN_JUDGE_BARREL_ID = 0x405, //枪管热量限制 枪管冷却速度 枪管当前热量 机动枪管当前热量（双枪步兵用）
    CAN_JUDGE_PARAM_ID = 0x407   //当前弹速 弹速限制 队伍颜色 能量机关状态
}CAN_Msg_enum;

void CAN_All_Init(void);
void CAN_Filter_Init(CAN_HandleTypeDef* hcan);
int CAN_TxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[]);
void CAN_YawSendCurrent(int16_t current);
void CAN_PitchSendCurrent(int16_t current);
void CAN_ChasisSendSpd(int16_t vx, int16_t vy, int16_t vz, int8_t car_mode, int8_t is_aimbot);
void CAN_ChasisSendMsg(int16_t yaw, int16_t pitch, int8_t servo_status, int8_t fric_status, int8_t rammer_status,
                       int8_t redraw_status);
void CAN_ShootSendCurrent(int16_t friLc, int16_t friRc, int16_t friUc);
void CAN_RamSendCurrent(int16_t Ramc);
void CAN_ScopeSendCurrent(int16_t scopeu);

///以下为达妙电机相关CAN函数
typedef CAN_HandleTypeDef hcan_t;
uint8_t canx_bsp_send_data(hcan_t *hcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t canx_bsp_receive(hcan_t *hcan, uint16_t *recid, uint8_t *buf);
void can2_rx_callback(void);
#endif //HERO_TEST_BSP_CAN_H

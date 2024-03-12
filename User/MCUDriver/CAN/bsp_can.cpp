//
// Created by DELL on 2023/9/22.
//

#include "bsp_can.h"
#include <cstring>
#include "debugc.h"
#include "gimbalc.h"
#include "shootc.h"
#include "visioncom_task.h"
#include "CyberGear.h"
///掉线检测
uint16_t isRecvShoot; //摩擦轮掉线检测
uint16_t isRecvYaw;   //Yaw掉线检测
uint16_t isRecvPih = 0;   //Pih掉线检测


/**
*@brief   CAN全部配置初始化
*@param   none
*@retval  none
*@Attention 只需调用该函数即可初始化
*/
void CAN_All_Init(void)
{
    CAN_Filter_Init(&hcan1);
    HAL_CAN_Start(&hcan1);
    __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    CAN_Filter_Init(&hcan2);
    HAL_CAN_Start(&hcan2);
    __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  *@brief   CAN过滤函数
  *@param   hcan句柄
  *@retval  None
  */
void CAN_Filter_Init(CAN_HandleTypeDef* hcan)
{
    CAN_FilterTypeDef sFilterConfig;
    HAL_StatusTypeDef HAL_Status;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; //工作在32位掩码模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;//滤波器位宽为单个32位
    sFilterConfig.FilterIdHigh = 0X0000;
    sFilterConfig.FilterIdLow = 0X0000;
    //过滤屏蔽码
    sFilterConfig.FilterMaskIdHigh = 0X0000;
    sFilterConfig.FilterMaskIdLow = 0X0000;
    sFilterConfig.SlaveStartFilterBank=14;

    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    if (hcan->Instance == CAN1)
    {
        sFilterConfig.FilterBank = 0;
    }
    else if (hcan->Instance == CAN2)
    {
        sFilterConfig.FilterBank = 14;
    }
    sFilterConfig.FilterActivation = ENABLE;
    HAL_Status = HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
    if (HAL_Status != HAL_OK)
    {
//        usart_printf("NO CAN\r\n");
    }
}

/**
  *@brief   CAN自动选择邮箱发送
  *@param   句柄hcan,发送CAN消息结构体pHeader,要发送的数据aData
  *@retval  邮箱号mailbox
  */
int CAN_TxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[])
{
    uint8_t mailbox = 0;
    uint32_t pTxMailbox[mailbox];
    if (hcan->Instance->TSR&(1<<26)) mailbox = 0;       //邮箱0为空
    else if (hcan->Instance->TSR&(1<<27)) mailbox = 1;  //邮箱1为空
    else if (hcan->Instance->TSR&(1<<28)) mailbox = 2;  //邮箱2为空
    else return 0xFF;                                   //无空邮箱,无法发送
    hcan->Instance->sTxMailBox[mailbox].TIR = 0;        //清除之前的设置
    HAL_CAN_AddTxMessage(hcan, pHeader, aData, pTxMailbox);
    return mailbox;
}

/**
  *@brief   CAN接收回调函数
  *@param   hcan句柄
  *@retval  None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)  //接收回调函数
{
    uint8_t recvData[8]; //接受数组
    HAL_StatusTypeDef HAL_Status;
    CAN_RxHeaderTypeDef RxMeg; //CAN接受指针结构体
    HAL_Status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMeg, recvData);

    if (hcan->Instance == CAN1)
    {
        if (HAL_Status == HAL_OK)                                                    //在这里接收数据
        {
            //回调函数
            ///小米电机部分
            Motor_Can_ID=Get_Motor_ID(RxMeg.ExtId);//首先获取回传电机ID信息
            switch(Motor_Can_ID)                          //将对应ID电机信息提取至对应结构体
            {
                case 0x7F:
                    if(rxMsg.ExtId>>24!= 0)               //检查是否为广播模式
                        Motor_Data_Handler(&mi_motor[0],recvData,RxMeg.ExtId);
                    else
                        mi_motor[0].MCU_ID = recvData[0];
                    break;
                default:
                    break;
            }
        }
    }
    if (hcan->Instance == CAN2)
    {
        if (HAL_Status == HAL_OK)                                                    //在这里接收数据
        {
            if (RxMeg.StdId == 0xB)
            {
                dm4310_fbdata(&motor[Motor1], recvData);
//                can2_rx_callback();
            }
            if (RxMeg.StdId == CAN_SHOOT_LEFT_ID)
            {//左摩擦轮
                gimbal.motors[ShootLMotor].Connected = 1;
                gimbal.motors[ShootLMotor].RawAngle = (int16_t)(recvData[0] << 8 | recvData[1]);     // 0~8191
                gimbal.motors[ShootLMotor].RawSpeed = (int16_t)(recvData[2] << 8 | recvData[3]);     // rpm
                gimbal.motors[ShootLMotor].RawTorqueCurrent = (int16_t)(recvData[4] << 8 | recvData[5]);    //转矩
                gimbal.motors[ShootLMotor].RawTemperature = (int16_t)(recvData[6]);                  //温度
                gimbal.motors[ShootLMotor].Null = (int16_t)(recvData[7]);
            }
            if (RxMeg.StdId == CAN_SHOOT_RIGHT_ID)
            {//右摩擦轮
                gimbal.motors[ShootRMotor].Connected = 1;
                gimbal.motors[ShootRMotor].RawAngle = (int16_t)(recvData[0] << 8 | recvData[1]);     //0~8191
                gimbal.motors[ShootRMotor].RawSpeed = (int16_t)(recvData[2] << 8 | recvData[3]);     //rpm
                gimbal.motors[ShootRMotor].RawTorqueCurrent = (int16_t)(recvData[4] << 8 | recvData[5]);    //转矩
                gimbal.motors[ShootRMotor].RawTemperature = (int16_t)(recvData[6]);                  //温度
                gimbal.motors[ShootRMotor].Null = (int16_t)(recvData[7]);
            }
            if (RxMeg.StdId == CAN_RAMC_ID)
            {//拨弹轮
                gimbal.motors[RamMotor].Connected = 1;
                gimbal.motors[RamMotor].RawAngle = (int16_t)(recvData[0] << 8 | recvData[1]);     //0~8191
                gimbal.motors[RamMotor].RawSpeed = (int16_t)(recvData[2] << 8 | recvData[3]);     //rpm
                gimbal.motors[RamMotor].RawTorqueCurrent = (int16_t)(recvData[4] << 8 | recvData[5]);    //转矩(电流)
                gimbal.motors[RamMotor].RawTemperature = (int16_t)(recvData[6]);                  //温度
                gimbal.motors[RamMotor].Null = (int16_t)(recvData[7]);
            }
            if (RxMeg.StdId == CAN_SHOOT_UP_ID)
            {//上摩擦轮
                gimbal.motors[ShootUMotor].Connected = 1;
                gimbal.motors[ShootUMotor].RawAngle = (int16_t)(recvData[0] << 8 | recvData[1]);     //0~8191
                gimbal.motors[ShootUMotor].RawSpeed = (int16_t)(recvData[2] << 8 | recvData[3]);     //rpm
                gimbal.motors[ShootUMotor].RawTorqueCurrent = (int16_t)(recvData[4] << 8 | recvData[5]);    //转矩
                gimbal.motors[ShootUMotor].RawTemperature = (int16_t)(recvData[6]);                  //温度
                gimbal.motors[ShootUMotor].Null = (int16_t)(recvData[7]);
            }

            if (RxMeg.StdId == CAN_YAW_RCV_ID)
            {//Yaw轴
                gimbal.motors[YawMotor].Connected = 1;
                gimbal.motors[YawMotor].RawAngle = (int16_t)(recvData[0] << 8 | recvData[1]);     //0~8191
                gimbal.motors[YawMotor].RawSpeed = (int16_t)(recvData[2] << 8 | recvData[3]);     //rpm
                gimbal.motors[YawMotor].RawTorqueCurrent = (int16_t)(recvData[4] << 8 | recvData[5]);    //转矩
                gimbal.motors[YawMotor].RawTemperature = (int16_t)(recvData[6]);                  //温度
                gimbal.motors[YawMotor].Null = (int16_t)(recvData[7]);
            }
            if (RxMeg.StdId == CAN_SCOPE_UP_ID)
            {//开镜上电机
                gimbal.motors[ScopeUMotor].Connected = 1;
                gimbal.motors[ScopeUMotor].RawAngle = (int16_t)(recvData[0] << 8 | recvData[1]);     //0~8191
                gimbal.motors[ScopeUMotor].RawSpeed = (int16_t)(recvData[2] << 8 | recvData[3]);     //rpm
                gimbal.motors[ScopeUMotor].RawTorqueCurrent = (int16_t)(recvData[4] << 8 | recvData[5]);    //转矩
                gimbal.motors[ScopeUMotor].RawTemperature = (int16_t)(recvData[6]);                  //温度
                gimbal.motors[ScopeUMotor].Null = (int16_t)(recvData[7]);
            }
            if (RxMeg.StdId == CAN_JUDGE_BARREL_ID)
            {//枪管信息
                gimbal.shoot.heat_limit = (uint16_t)(recvData[0] << 8 | recvData[1]);
                gimbal.shoot.cool_spd = (uint16_t)(recvData[2] << 8 | recvData[3]);
                gimbal.shoot.heat_now = (uint16_t)(recvData[4] << 8 | recvData[5]);
            }
            if (RxMeg.StdId == CAN_JUDGE_PARAM_ID)   //裁判系统信息的补充
            {//判断信息
                memcpy(&(gimbal.shoot.shoot_spd_now), recvData, 4);
                gimbal.shoot.shoot_spd_max = (uint16_t)((recvData)[4] << 8 | recvData[5]);
                gimbal.shoot.color = (uint8_t)recvData[6];
                gimbal.shoot.smallORbig = (uint8_t)recvData[7];
            }
            Vision_JudgeUpdate(gimbal.shoot.shoot_spd_now,  gimbal.shoot.color, gimbal.shoot.smallORbig);
        }
    }

}

/**
  *@brief   CAN发送电流给Yaw轴
  *@param   电流
  *@retval  None
  */
void CAN_YawSendCurrent(int16_t current)
{
    CAN_TxHeaderTypeDef tx_msg;
    uint32_t send_mail_box = 0;
    uint8_t send_data[8];
    tx_msg.StdId = CAN_YAW_SEND_ID;
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 0x08;
    send_data[0] = (current >> 8);
    send_data[1] = current & 0xff;

    HAL_CAN_AddTxMessage(&hcan2,&tx_msg,send_data,&send_mail_box);
}

/**
  *@brief   CAN发送电流给Pitch轴
  *@param   电流
  *@retval  None
  */
void CAN_PitchSendCurrent(int16_t current)
{
    CAN_TxHeaderTypeDef tx_msg;
    uint32_t send_mail_box = 0;
    uint8_t send_data[8];
    tx_msg.StdId = CAN_PIH_SEND_ID;
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 0x04;
    send_data[0] = 0;
    send_data[1] = 0; //预留给双枪管

    send_data[2] = (current >> 8);
    send_data[3] = current & 0xff;

    HAL_CAN_AddTxMessage(&hcan1,&tx_msg,send_data,&send_mail_box);
}
/**
  *@breif   CAN发送电流给底盘水平运动
  *@param   vx、vy、vz、车体模式、自瞄模式
  *@retval  None
  */
void CAN_ChasisSendSpd(int16_t vx, int16_t vy, int16_t vz, int8_t car_mode, int8_t is_aimbot)
{
    CAN_TxHeaderTypeDef tx_msg;
    uint32_t send_mail_box = 1;
    uint8_t send_data[8];
    tx_msg.StdId = CAN_CHASSIS_VAL_ID;
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 0x08;
    send_data[0] = (vx >> 8);
    send_data[1] = vx & 0xff;

    send_data[2] = (vy >> 8);
    send_data[3] = vy & 0xff;

    send_data[4] = (vz >> 8);
    send_data[5] = vz & 0xff;

    send_data[6] = car_mode;  //车的模式
    send_data[7] = is_aimbot; //自瞄状态

    HAL_CAN_AddTxMessage(&hcan2,&tx_msg,send_data,&send_mail_box);
}
/**
  *@breif   CAN发送云台的信息给底盘
  *@param
  *@retval  None
  */
void CAN_ChasisSendMsg(int16_t yaw, int16_t pitch, int8_t servo_status, int8_t fric_status, int8_t rammer_status,
                       int8_t redraw_status)
{
    CAN_TxHeaderTypeDef tx_msg;
    uint32_t send_mail_box = 2;
    uint8_t send_data[8];
    tx_msg.StdId = CAN_CHASSIS_YAW_ID;
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 0x08;
    send_data[0] = (yaw >> 8);
    send_data[1] = yaw & 0xff;

    send_data[2] = (pitch >> 8);
    send_data[3] = pitch & 0xff;

    send_data[4] = servo_status;
    send_data[5] = fric_status;

    send_data[6] = rammer_status;
    send_data[7] = redraw_status;

    HAL_CAN_AddTxMessage(&hcan2,&tx_msg,send_data,&send_mail_box);
}

/**
  *@breif   CAN发送电流给摩擦轮两电机与拨弹轮
  *@param   none
  *@retval  none
  */
void CAN_ShootSendCurrent(int16_t friLc, int16_t friRc, int16_t friUc , int16_t ramc)
{
    CAN_TxHeaderTypeDef tx_msg;
    uint32_t send_mail_box = 0;
    uint8_t send_data[8];
    tx_msg.StdId = CAN_SHOOT_SEND_ID;
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 0x08;
    send_data[0] = (friLc >> 8);
    send_data[1] = friLc;

    send_data[2] = (friRc >> 8);
    send_data[3] = friRc;

    send_data[4] = (ramc >> 8);
    send_data[5] = ramc;

    send_data[6] = (friUc >> 8);
    send_data[7] = friUc;

    HAL_CAN_AddTxMessage(&hcan2,&tx_msg,send_data,&send_mail_box);
}
/**
  *@breif   CAN发送电流给开镜电机
  *@param   none
  *@retval  none
  */
void CAN_ScopeSendCurrent(int16_t scopeu)
{
    CAN_TxHeaderTypeDef tx_msg;
    uint32_t send_mail_box = 0;
    uint8_t send_data[8];
    tx_msg.StdId = CAN_SCOPE_SEND_ID;
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 0x08;

    send_data[2] = (scopeu >> 8);
    send_data[3] = scopeu;

    HAL_CAN_AddTxMessage(&hcan2,&tx_msg,send_data,&send_mail_box);
}

/**
************************************************************************
* @brief:      	canx_bsp_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hcan: CAN句柄
* @param:       id: 	CAN设备ID
* @param:       data: 发送的数据
* @param:       len:  发送的数据长度
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/
uint8_t canx_bsp_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
    CAN_TxHeaderTypeDef	tx_header;
    uint32_t send_mail_box = 0;
    tx_header.StdId = id;
    tx_header.ExtId = 0;
    tx_header.IDE   = 0;
    tx_header.RTR   = 0;
    tx_header.DLC   = len;
    /*找到空的发送邮箱，把数据发送出去*/
    HAL_CAN_AddTxMessage(hcan, &tx_header, data, &send_mail_box);
    return 0;
}
/**
************************************************************************
* @brief:      	canx_bsp_receive(CAN_HandleTypeDef *hcan, uint8_t *buf)
* @param:       hcan: CAN句柄
* @param[out]:  rec_id: 	接收到数据的CAN设备ID
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
uint8_t canx_bsp_receive(hcan_t *hcan, uint16_t *rec_id, uint8_t *buf)
{
    CAN_RxHeaderTypeDef rx_header;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, buf) == HAL_OK)
    {
        *rec_id = rx_header.StdId;
        return rx_header.DLC; //接收数据长度
    }
    else
        return 0;
}


/**
************************************************************************
* @brief:      	can1_rx_callback(void)
* @param:       void
* @retval:     	void
* @details:    	供用户调用的接收弱函数
************************************************************************
**/
__weak void can2_rx_callback(void)
{

}
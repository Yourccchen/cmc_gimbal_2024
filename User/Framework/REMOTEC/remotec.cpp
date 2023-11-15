//
// Created by DELL on 2023/9/25.
//
#include "remotec.h"
#include "usart.h"
#include "cstring"
#include "cstdio"
#include "stdlib.h"
#include "debugc.h"
#include "gimbalc.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

//遥控器控制变量
RC_ctrl_t rc_ctrl;

//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         nonehh
  */
void REMOTEC_Init(void)
{
    REMOTEIO_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t* get_remote_control_point(void)
{
    return &rc_ctrl;
}

/**
  * @brief     huart1中断服务程序
  * @param     none
  * @retval    none
  */
void REMOTEC_UartIrqHandler(void)
{
    if (huart1.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if (USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA2_Stream2->CR &= ~(DMA_SxCR_CT);

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
        RC_DataHandle(&rc_ctrl);
    }
}

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指针
  * @retval         none
  */
int16_t RC_GetNewData = 0;//检测键值是否在发送/更新
static void sbus_to_rc(volatile const uint8_t* sbus_buf, RC_ctrl_t* rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }
    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) & 0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                       //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis 滚轮
    rc_ctrl->mouse.press_l.Now_State = sbus_buf[12];                        //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r.Now_State = sbus_buf[13];                        //!< Mouse Right Is Press ?
    rc_ctrl->key.value =
            sbus_buf[14] | (sbus_buf[15] << 8);                             //!< KeyBoard value  W S A D Q E SHIFT CTRL
    rc_ctrl->key.W.Now_State= (rc_ctrl->key.value & 0x01);
    rc_ctrl->key.S.Now_State= (rc_ctrl->key.value & 0x02) >> 1;             //按下为1，未按下时为0
    rc_ctrl->key.A.Now_State= (rc_ctrl->key.value & 0x04) >> 2;
    rc_ctrl->key.D.Now_State= (rc_ctrl->key.value & 0x08) >> 3;
    rc_ctrl->key.SHIFT.Now_State = (rc_ctrl->key.value & 0x10) >> 4;
    rc_ctrl->key.CONTRL.Now_State = (rc_ctrl->key.value & 0x20) >> 5;
    rc_ctrl->key.Q.Now_State = (rc_ctrl->key.value & 0x40) >> 6;
    rc_ctrl->key.E.Now_State = (rc_ctrl->key.value & 0x80) >> 7;
    rc_ctrl->key.R.Now_State = (rc_ctrl->key.value & 0x100) >> 8;
    rc_ctrl->key.F.Now_State = (rc_ctrl->key.value & 0x200) >> 9;
    rc_ctrl->key.G.Now_State = (rc_ctrl->key.value & 0x400) >> 10;
    rc_ctrl->key.Z.Now_State = (rc_ctrl->key.value & 0x800) >> 11;
    rc_ctrl->key.X.Now_State = (rc_ctrl->key.value & 0x1000) >> 12;
    rc_ctrl->key.C.Now_State = (rc_ctrl->key.value & 0x2000) >> 13;
    rc_ctrl->key.V.Now_State = (rc_ctrl->key.value & 0x4000) >> 14;
    rc_ctrl->key.B.Now_State = (rc_ctrl->key.value & 0x8000) >> 15;

    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //拨盘 值范围:0~660

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
    RC_GetNewData++;
    if (RC_GetNewData > 10000)
    {
        RC_GetNewData = 0;
    }
}
/**
  * @brief      检测遥控器是否在更新数据并发送
  * @retval     检测键值是否在发送/更新
  */
int16_t RC_UpdateData()
{
    return RC_GetNewData;
}
/**
  * @brief      处理异常值
  * @param      遥控器控制变量rc_ctrl
  * @retval     none
  */
void RC_DataHandle(RC_ctrl_t* rc_ctrl)
{
    if (abs(rc_ctrl->rc.ch[0]) < 5)rc_ctrl->rc.ch[0] = 0;
    if (abs(rc_ctrl->rc.ch[1]) < 5)rc_ctrl->rc.ch[1] = 0;
    if (abs(rc_ctrl->rc.ch[2]) < 5)rc_ctrl->rc.ch[2] = 0;
    if (abs(rc_ctrl->rc.ch[3]) < 5)rc_ctrl->rc.ch[3] = 0;
    if (abs(rc_ctrl->rc.ch[4]) < 5)rc_ctrl->rc.ch[4] = 0;
    if (abs(rc_ctrl->rc.ch[0]) > 670 || abs(rc_ctrl->rc.ch[3]) > 670)
    {
        memset(rc_ctrl, 0, sizeof(RC_ctrl_t)); //异常值
    }
}

/**
  * @brief      将结构体调用改为函数调用
  * @param      none
  * @retval     RC_ctrl_t结构体
  */
RC_ctrl_t RC_GetDatas(void)
{
    return rc_ctrl;
}


/**
	* @name   portHandle
	* @brief  非连续键值处理 即上一次是0，本次是1，判断为按了一次，用于处理状态切换等不适用于连续用手按的按键
	* @param  port 键值状态结构体
  	* @retval None
*/
void portHandle(Key_State* port)
{
    if (port->Now_State == 1 && port->Last_State == 0)
        port->Is_Click_Once = 1;    //之前没按下，按下后，视为按了一次。Is_Click_Once赋值为1
    else
        port->Is_Click_Once = 0;

    port->Last_State = port->Now_State;
}

/**
  *@breif   设置Yaw轴移动值
  *@param   目标值
  *@retval  Yaw轴移动值yaw_speed
  */
float portSetYawSpeed(void)
{
    float yaw_tarpos;
    switch (gimbal.ControlMode)
    {
        case KEY_MODE:
        {
            yaw_tarpos = -rc_ctrl.mouse.x * 2; //这儿后期加等级分档位
            break;
        }
        case RC_MODE:
        {
            yaw_tarpos = -RC_GetDatas().rc.ch[0] * 360 / 660.0f;
            break;
        }
    }
    return yaw_tarpos;
}

/**
  *@breif   设置Pitch轴移动值
  *@param   目标值
  *@retval  Pitch轴移动值pih_speed
  */
float portSetPihSpeed(void)
{
    float pih_tarpos;
    switch (gimbal.ControlMode)
    {
        case KEY_MODE:
        {
            pih_tarpos = -rc_ctrl.mouse.y * 1.5; //这儿后期加等级分档位
            break;
        }
        case RC_MODE:
        {
            pih_tarpos = RC_GetDatas().rc.ch[1] * 360 / 660.0f;
            break;
        }
    }
    return pih_tarpos;
}

/**
  *@breif   设置水平移动值
  *@param   目标值
  *@retval  水平移动值vx
  */
float portSetVx(void)
{
    float vx;
    switch (gimbal.ControlMode)
    {
        case KEY_MODE:
        {
            vx = (rc_ctrl.key.D.Now_State - rc_ctrl.key.A.Now_State) * 80.0; //这儿后期加等级分档位
            if (rc_ctrl.key.SHIFT.Now_State == 1)
                vx *= 2.4; //按住shift加速
            break;
        }
        case RC_MODE:
        {
            vx = RC_GetDatas().rc.ch[2] * 200.0f / 660.0f;
            break;
        }
        case ZIMIAO:
        {
            vx = RC_GetDatas().rc.ch[2] * 200.0f / 660.0f;
            break;
        }
    }
    return vx;
}
/**
  *@breif   设置垂直移动值
  *@param   目标值
  *@retval  垂直移动值vy
  */
float portSetVy(void)
{
    float vy;
    switch (gimbal.ControlMode)
    {
        case KEY_MODE:
        {
            vy = (rc_ctrl.key.W.Now_State - rc_ctrl.key.S.Now_State) * 80.0;
            if (rc_ctrl.key.SHIFT.Now_State == 1)
                vy *= 2.4; //按住shift加速
            break;
        }
        case RC_MODE:
        {
            vy = RC_GetDatas().rc.ch[3] * 200.0f / 660.0f;
            break;
        }
        case ZIMIAO:
        {
            vy = RC_GetDatas().rc.ch[3] * 200.0f / 660.0f;
            break;
        }
    }
    return vy;
}
/**
  * @brief     开启自瞄模式（鼠标右键开启带数字的自瞄，E键开启大符的判断）
  * @param     none
  * @retval    none
  */
uint8_t portIsZimiao(void)
{
    if (rc_ctrl.mouse.press_r.Now_State)
        return 0x01;  //带数字的自瞄
    else if (rc_ctrl.key.E.Now_State) //打幅，默认小幅，大符会在外面判断
        return 0xbb;
    else
        return 0;
}
/**
  *@breif   车体模式切换：保护、随动、小陀螺
  *@param   none
  *@retval  当前的车体模式
  */
int8_t portSetCarMode(void)
{
    //非连续键值处理 即上一次是0，本次是1，判断为按了一次
    portHandle(&rc_ctrl.key.X);
    portHandle(&rc_ctrl.key.G);
    switch (gimbal.ControlMode)
    {
        //Z小陀螺 X随动 G保护
        case KEY_MODE:
        {
            if (rc_ctrl.key.X.Is_Click_Once && gimbal.CarMode==SUIDONG)
            {
                gimbal.CarMode = TUOLUO;
            }
            else if (rc_ctrl.key.X.Is_Click_Once && (gimbal.CarMode==TUOLUO|| gimbal.CarMode==PROTECT))
            {
                gimbal.CarMode = SUIDONG;
            }
            else if (rc_ctrl.key.G.Is_Click_Once)
            {
                gimbal.CarMode = PROTECT;
            }
            else
                gimbal.CarMode = gimbal.Last_CarMode;
            break;
        }
        case RC_MODE:
        {
            gimbal.CarMode = rc_ctrl.rc.s[0]; //右侧拨杆
            break;
        }
    }

    gimbal.Last_CarMode = gimbal.CarMode;
    return gimbal.CarMode;
}

/**
  *@breif   射击模式切换
  *@param   none
  *@retval  ShootMode射击模式的参数(0为关摩擦轮，1为开摩擦轮）
  */
int8_t portSetShootMode(void)
{
    portHandle(&rc_ctrl.key.Q);//非连续键值处理 即上一次是0，本次是1，判断为按了一次
    switch(gimbal.ControlMode)
    {
        case KEY_MODE:
        {
            if(rc_ctrl.key.Q.Is_Click_Once && gimbal.shoot.fric_flag== CLOSEFRIC)
            {
                gimbal.shoot.fric_flag=OPENFRIC;
            }
            else if(rc_ctrl.key.Q.Is_Click_Once && gimbal.shoot.fric_flag==OPENFRIC)
            {
                gimbal.shoot.fric_flag=CLOSEFRIC;
            }
            break;
        }
        case RC_MODE:
        {
            if(RC_GetDatas().rc.ch[4]==-660)
            {
                gimbal.shoot.fric_count++;
                if( ((gimbal.shoot.fric_count>50) && (gimbal.shoot.fric_flag==CLOSEFRIC)) )
                {
                    gimbal.shoot.fric_count=0;
                    gimbal.shoot.fric_flag=OPENFRIC;
                }
                if(((gimbal.shoot.fric_count>50) && (gimbal.shoot.fric_flag==OPENFRIC)) )
                {
                    gimbal.shoot.fric_count=0;
                    gimbal.shoot.fric_flag=CLOSEFRIC;
                }
            }
            break;
        }
    }
    return gimbal.shoot.fric_flag;
}

/**
  *@breif   定义拨弹轮模式
  */
void portSetRammer(void)
{
    portHandle(&rc_ctrl.mouse.press_l);//非连续化处理
    switch(gimbal.ControlMode)
    {
        case RC_MODE:
        {
            if(RC_GetDatas().rc.ch[4]==660)
            {
                gimbal.shoot.rammer_count++;
                if(gimbal.shoot.rammer_count>100)
                {
                    gimbal.shoot.rammer_count=0;
                    gimbal.shoot.rammer_flag++;
                }
            }
            break;
        }
        case KEY_MODE:
        {
            if(rc_ctrl.mouse.press_l.Is_Click_Once)
            {
                gimbal.shoot.rammer_flag++;
            }
            break;
        }
    }
}

/**
  *@breif   控制模式切换
  *@retval  ControlMode模式的参数（有KEY_MODE,RC_MODE,ZIMIAO三种）
  */
int8_t portSetControlMode(void)
{
    return RC_GetDatas().rc.s[1];//左侧拨杆
}


/**
  * @brief     按住ctrl则以当前姿态为目标角度
  * @retval    ctrl的当前状态
  */
uint8_t portSetGimbal(void)
{
    return rc_ctrl.key.CONTRL.Now_State;
}
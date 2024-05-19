//
// Created by DELL on 2023/10/14.
//
#include "visioncom_task.h"
#include "packet.hpp"
#include "crc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "remotec.h"
#include "bsp_can.h"
#include "debugc.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
#include "gimbalc.h"
//#include "imu_wit.h"
#include "CH100.h"
float Shoot_SpeedNow = 0;
int8_t My_Color, Fu_Type;
int32_t id = 0;
int InitPermit=0;

void Vision_JudgeUpdate(float shoot_speed, int8_t color, int8_t type)
{
    Shoot_SpeedNow = shoot_speed;
    My_Color = color;
    Fu_Type = type;
}

/**
	* @name   VisionChattingLoop
	* @brief  视觉通信循环
	* @param  mode 自瞄模式
	* @retval None
*/
SendPacket send_packet;
uint8_t Buf[sizeof(SendPacket)];

void VisionChattingLoop(uint8_t mode)
{
//	float w = IMU_Quaternion().w;
//	float x = IMU_Quaternion().x;
//	float y = IMU_Quaternion().y;
//	float z = IMU_Quaternion().z;
//    float roll = IMU_NavigationAngle().roll;
//    float pitch = IMU_NavigationAngle().pitch; //用原始角好还是连续化之后的好？
//    float yaw = IMU_NavigationAngle().yaw;
//	if (Color_now == 1)	send_packet.color =0;
//	if (Color_now == 0)	send_packet.color =1;
    send_packet.header = 0x5A;
    send_packet.shoot_spd =  Shoot_SpeedNow;
//    send_packet.shoot_spd =15.3;
    send_packet.pitch = -IMU_Angle_CH100(PIH_ANGLE);
    send_packet.yaw = -IMU_Angle_CH100(YAW_ANGLE);
//    send_packet.roll = roll;

    if(My_Color == 0)
        send_packet.enemy_color = 'B';
    else
        send_packet.enemy_color = 'R';

//    send_packet.enemy_color = 'B';


//	send_packet.packet_id = id;

    std::copy(reinterpret_cast<const uint8_t*>(&send_packet),
              reinterpret_cast<const uint8_t*>(&send_packet) + sizeof (SendPacket),Buf);
    Append_CRC16_Check_Sum(Buf, sizeof(SendPacket));
    CDC_Transmit_FS(Buf, sizeof(SendPacket));
    id++;
}

/**
	* @name   VisionComTask
	* @brief  视觉通信任务,1ms1次
	* @param  None
	* @retval None
*/
void VisionComTask(void const* argument)
{
    /* USER CODE BEGIN VisionComTask */
    portTickType CurrentTime;
    /* Infinite loop */
    for (;;)
    {
        CurrentTime = xTaskGetTickCount();
        int8_t Zimiao = portIsZimiao();

        VisionChattingLoop(Zimiao);
        vTaskDelayUntil(&CurrentTime, 5 / portTICK_RATE_MS);
    }
    /* USER CODE END VisionComTask */
}


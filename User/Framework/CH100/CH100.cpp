//
// Created by DELL on 2024/2/22.
//
/**
 * ch100 \
 */
#include "CH100.h"
#include "usart.h"

#define IMU_RVSIZE 512
#define IMU_DATASIZE 82U
uint8_t IMU_RxBuf[IMU_RVSIZE] = {0};
uint8_t IMU_Buf[IMU_RVSIZE] = {0};

static float angle[3] = {};
static float Yaw_Angle, Yaw_Speed, Pih_Angle, Pih_Speed, Roll_Angle, Roll_Speed;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

void IMU_UartInit(void)
{
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, IMU_RxBuf, IMU_RVSIZE);
}

static float R4(uint8_t *p)
{
    float r;
    memcpy(&r, p, 4);
    return r;
}

static void IMU_Data_Handler()
{
    memcpy(IMU_Buf, &IMU_RxBuf[0], IMU_DATASIZE);
    uint8_t data_length = IMU_RVSIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    if (IMU_Buf[0] == 0x5A && IMU_Buf[1] == 0xA5 && IMU_Buf[6] == 0x91)
    {
        Roll_Speed = R4(IMU_Buf + 6 + 24);
        Pih_Speed = R4(IMU_Buf + 6 + 28);
        Yaw_Speed = R4(IMU_Buf + 6 + 32);
        angle[0] = R4(IMU_Buf + 6 + 52);//roll
        angle[1] = R4(IMU_Buf + 6 + 48);//pitch
        angle[2] = R4(IMU_Buf + 6 + 56);//yaw
    }
}

void CH100_IMU_Receive_Data(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        HAL_UART_DMAStop(&huart1);
        IMU_Data_Handler();
        uint8_t rx_len = IMU_DATASIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, IMU_DATASIZE);
        memset(IMU_RxBuf, 0, rx_len);
        HAL_UART_Receive_DMA(&huart1, IMU_RxBuf, IMU_DATASIZE);
    }
}

static void crc16_update(uint16_t *currect_crc, const uint8_t *src, uint32_t len)
{
    uint32_t crc = *currect_crc;
    uint32_t j;
    for (j = 0; j < len; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    }
    *currect_crc = crc;
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

static float IMU_AngleIncreLoop1(float angle_now)
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

float IMU_Angle_CH100(uint8_t which)
{
    Yaw_Angle=IMU_AngleIncreLoop(angle[2]);
    Pih_Angle=IMU_AngleIncreLoop1(angle[1]);
    Roll_Angle=angle[0];
    switch (which)
    {
        case 1:
            return Pih_Angle;
        case 2:
            return Yaw_Angle;
        case 3:
            return Roll_Angle;
    }
    return 0;
}

float IMU_Speed_CH100(uint8_t which){
    switch (which)
    {
        case 1:
            return Pih_Speed;
        case 2:
            return Yaw_Speed;
        case 3:
            return Roll_Speed;
    }
}
//
// Created by DELL on 2023/9/23.
//

#include "main.h"
#include "gimbal_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "gimbalc.h"
#include "debugc.h"

void GimbalControlTask(void const * argument)
{
    /* USER CODE BEGIN GimbalControlTask */
    portTickType CurrentTime;
    /* Infinite loop */
    for (;;)
    {
        if(gimbal.ProtectFlag==ONLINE && motor[Motor1].para.state==0)
        {
            ctrl_enable_yaw();
        }
        else if(gimbal.ProtectFlag==OFFLINE && motor[Motor1].para.state==1)
        {
            ctrl_disable_yaw();
        }

        if(gimbal.ProtectFlag==OFFLINE)
        {
            ctrl_disable_pitch();
        }
        else
        {
            ctrl_enable_pitch();
        }

        gimbal.count_time_send++;
        if(gimbal.count_time_send > 4)
            gimbal.count_time_send = 0;
        CurrentTime = xTaskGetTickCount();
        gimbal.Gimbal_ControlLoop();
        gimbal.Printf_Test();

        vTaskDelayUntil(&CurrentTime, 5/portTICK_RATE_MS);
    }
    /* USER CODE END GimbalControlTask */
}

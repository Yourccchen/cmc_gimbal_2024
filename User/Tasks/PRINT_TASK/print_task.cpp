//
// Created by DELL on 2023/9/23.
//

#include "print_task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "gimbalc.h"
//#include "INS_task.h"

void PrintControlTask(void const* argument)
{
    /* USER CODE BEGIN GimbalControlTask */
    portTickType CurrentTime;
    uint64_t count_number = 0;
    /* Infinite loop */
    for (;;)
    {
        ctrl_enable();


        CurrentTime = xTaskGetTickCount();
        gimbal.Printf_Test();

        if(count_number % 100 ==0)
        {
            HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
            count_number = 0;
        }
        count_number++;
        vTaskDelayUntil(&CurrentTime, 5 / portTICK_RATE_MS);
    }
    /* USER CODE END GimbalControlTask */
}
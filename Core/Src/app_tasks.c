#include "app_tasks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "pid.h"

#define CTRL_PERIOD_MS   10    
#define SIN_PERIOD_MS    10     
#define SIN_AMP          30.0f    /* counts per 10 ms, 30 -> about 136 rpm */
#define SIN_FREQ         0.3f    
/* Encoder counts per output revolution = lines * 4 * gear ratio
   e.g. 11 lines, 4x quadrature, 1:30 gearbox -> 11*4*30 = 1320
   Change this number to match your motor, then the rpm values are correct */
#define ENC_CPR          1320.0f
/* counts in one sampling window -> rev / minute */
#define CNT_TO_RPM(n)    ((n) * 60000.0f / (ENC_CPR * (float)CTRL_PERIOD_MS))
volatile float g_sin_target = 0.0f;
PID_t SpeedPID;
static void Motor_SetPWM(int16_t pwm)
{
    if (pwm >= 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    } else {
        pwm = -pwm;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    }
    if (pwm > 100) pwm = 100;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint16_t)pwm);
}
static void Sin_Generate_Task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t t_ms = 0;
    (void)pvParameters;
    for (;;)
    {
        t_ms += SIN_PERIOD_MS;
        g_sin_target = SIN_AMP * sinf(2.0f * 3.14159265f * SIN_FREQ * t_ms / 1000.0f);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SIN_PERIOD_MS));
    }
}
static void Motor_Task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint16_t last_cnt = __HAL_TIM_GET_COUNTER(&htim3);
    uint8_t  print_div = 0;
    char buf[64];
    (void)pvParameters;

    for (;;)
    {
     
        uint16_t cnt = __HAL_TIM_GET_COUNTER(&htim3);
        int16_t  speed = (int16_t)(cnt - last_cnt);
        last_cnt = cnt;

        SpeedPID.Actual = (float)speed;
        SpeedPID.Target = g_sin_target;
        PID_Update(&SpeedPID);
        Motor_SetPWM((int16_t)SpeedPID.Out);

 
        if (++print_div >= 5)
        {
            print_div = 0;
            int len = snprintf(buf, sizeof(buf), "%.1f,%.1f,%.1f\n",
                               CNT_TO_RPM(SpeedPID.Target),
                               CNT_TO_RPM(SpeedPID.Actual),
                               SpeedPID.Out);
            HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 10);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }
}


void App_Tasks_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);        
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);


    PID_Init(&SpeedPID, 0.49f, 0.17f, 0.0f, 50.0f, 100.0f);

    xTaskCreate(Sin_Generate_Task, "sin",   256, NULL, 2, NULL);
    xTaskCreate(Motor_Task,        "motor", 256, NULL, 3, NULL);
}

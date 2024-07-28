/**
 * Copyright 2021 by MooreSilicon.All rights reserved
 * @file  test_freertos_timer.c
 * @brief
 * @author weiquan.ou
 * @date 2022-3-29
 * @version 1.0
 * @Revision
 */

/* Kernel includes. */
#include "FreeRTOS.h" /* Must come first. */
#include "queue.h"    /* RTOS queue related API prototypes. */
#include "semphr.h"   /* Semaphore related API prototypes. */
#include "task.h"     /* RTOS task related API prototypes. */
#include "timers.h"   /* Software timer related API prototypes. */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "log.h"
#include "unity_test_runner.h"

static TaskHandle_t task1_handler;
static char *task1_name = "timer_task";
static TimerHandle_t timer1;
static TimerHandle_t timer2;
static char *timer1_name = "reload_timer";
static char *timer2_name = "oneshort_timer";
static SemaphoreHandle_t test_freertos_timer_block;

#define TASK_STK_SIZE 256

static void task1(void *pst)
{
    int i;
    int ret;

    for (i = 0; i < 3; i++) {
        switch (i) {
            case 0:
                xTimerStart(timer1, 0);
                MS_LOGI(MS_FREERTOS, "%s start !!\r\n", timer1_name);
                break;
            case 1:
                xTimerStart(timer2, 0);
                MS_LOGI(MS_FREERTOS, "%s start !!\r\n", timer2_name);
                break;
            case 2:
                xTimerStop(timer1, 0);
                xTimerStop(timer2, 0);
                MS_LOGI(MS_FREERTOS, "%s %s stop !!\r\n", timer1_name, timer2_name);
                break;
            default:
                break;
        }
        vTaskDelay(300);
    }

    ret = xSemaphoreGive(test_freertos_timer_block);
    if (ret != pdTRUE) {
        MS_LOGI(MS_FREERTOS, "%s release binary semaphore ERROR !\r\n", task1_name);
    }

    vTaskDelete(NULL);
}

void timer1_func(TimerHandle_t timer)
{
    MS_LOGI(MS_FREERTOS, "%s running !!\r\n", timer1_name);
    vTaskDelay(100);
    MS_LOGI(MS_FREERTOS, "%s end !!\r\n", timer1_name);
}

void timer2_func(TimerHandle_t timer)
{
    MS_LOGI(MS_FREERTOS, "%s running !!\r\n", timer2_name);
    vTaskDelay(100);
    MS_LOGI(MS_FREERTOS, "%s end !!\r\n", timer2_name);
}

extern void task_list_show(void);

TEST_CASE("freertos","test_freertos_timer", "[FreeRTOS/timer]")
{
    int ret;
#if 1
    taskENTER_CRITICAL();

    timer1 = xTimerCreate(timer1_name, 100, pdTRUE, 0, timer1_func);
    if (NULL == timer1) {
        taskEXIT_CRITICAL();
        MS_LOGE(MS_FREERTOS, "create %s ERROR!!\r\n", timer1_name);
        return;
    }

    timer2 = xTimerCreate(timer2_name, 100, pdFALSE, 0, timer2_func);
    if (NULL == timer2) {
        xTimerDelete(timer1, 0);
        MS_LOGE(MS_FREERTOS, "create %s ERROR!!\r\n", timer2_name);
        taskEXIT_CRITICAL();
        return;
    }

    test_freertos_timer_block = xSemaphoreCreateBinary();
    if (NULL == test_freertos_timer_block) {
        xTimerDelete(timer1, 0);
        xTimerDelete(timer2, 0);
        taskEXIT_CRITICAL();
        MS_LOGE(MS_FREERTOS, "create binary semaphore ERROR!!\r\n");
        return;
    }

    ret = xTaskCreate(task1,
                      task1_name,
                      (configSTACK_DEPTH_TYPE)TASK_STK_SIZE,
                      NULL,
                      2,
                      &task1_handler);
    if (ret != pdPASS) {
        xTimerDelete(timer1, 0);
        xTimerDelete(timer2, 0);
        vSemaphoreDelete(test_freertos_timer_block);
        taskEXIT_CRITICAL();
        MS_LOGE(MS_FREERTOS, "%s create ERROR!!\r\n", task1_name);
        return;
    }

    taskEXIT_CRITICAL();

    xSemaphoreTake(test_freertos_timer_block, portMAX_DELAY);
    vSemaphoreDelete(test_freertos_timer_block);
    xTimerDelete(timer1, 0);
    xTimerDelete(timer2, 0);
#endif
    task_list_show();
}


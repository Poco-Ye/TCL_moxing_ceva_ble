/**
 * Copyright 2021 by MooreSilicon.All rights reserved
 * @file  test_freertos_task_suspend.c
 * @brief
 * @author weiquan.ou
 * @date 2022-3-28
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

#include "log.h"
#include "unity_test_runner.h"

static TaskHandle_t task1_handler;
static TaskHandle_t task2_handler;
static TaskHandle_t mgmt_handler;
static char *task1_name = "suspend_task1";
static char *task2_name = "suspend_task2";
static char *mgmt_name = "suspend_mgmt";
static SemaphoreHandle_t binary;


#define TASK_STK_SIZE 256
#define TASK_PRI 2
#define TASK_END_NUM 5

static void task1(void *pst)
{
    int num = 0;

    while(1) {
        num++;
        MS_LOGI(MS_FREERTOS, "%s has already run %d times\r\n", task1_name, num);
        vTaskDelay(1000);
    }
}

static void task2(void *pst)
{
    int num = 0;

    while(1) {
        num++;
        MS_LOGI(MS_FREERTOS, "%s has already run %d times\r\n", task2_name, num);
        vTaskDelay(1000);
    }
}

static void mgmt(void *pst)
{
    int num = 0;
    int ret;

    while(num < TASK_END_NUM) {
        vTaskDelay(1000);
        switch(num) {
            case 0:
                vTaskSuspend(task1_handler);
                MS_LOGI(MS_FREERTOS, "%s suspend.\r\n", task1_name);
                break;
            case 1:
                vTaskSuspend(task2_handler);
                MS_LOGI(MS_FREERTOS, "%s suspend.\r\n", task2_name);
                break;
            case 2:
                vTaskResume(task1_handler);
                MS_LOGI(MS_FREERTOS, "%s resume.\r\n", task1_name);
                break;
            case 3:
                vTaskResume(task2_handler);
                MS_LOGI(MS_FREERTOS, "%s resume.\r\n", task1_name);
                break;
            case 4:
                vTaskDelete(task1_handler);
                vTaskDelete(task2_handler);
                MS_LOGI(MS_FREERTOS, "delete %s %s.\r\n", task1_name, task2_name);
                break;
            default:
                break;
        }

        num++;
    }

    ret = xSemaphoreGive(binary);
    if (ret != pdTRUE) {
        MS_LOGI(MS_FREERTOS, "release binary semaphore ERROR !\r\n");
    }

    mgmt_handler = NULL;
    vTaskDelete(NULL);
}


TEST_CASE("freertos","test_freertos_task_suspend", "[FreeRTOS/task_suspend]")
{
    int ret;

    taskENTER_CRITICAL();

    binary = xSemaphoreCreateBinary();
    if (NULL == binary) {
        taskEXIT_CRITICAL();
        MS_LOGE(MS_FREERTOS, "create binary semaphore ERROR!!\r\n");
        return;
    }

    ret = xTaskCreate(task1,
                      task1_name,
                      (configSTACK_DEPTH_TYPE)TASK_STK_SIZE,
                      NULL,
                      TASK_PRI,
                      &task1_handler);
    if (ret != pdPASS) {
        vSemaphoreDelete(binary);
        MS_LOGE(MS_FREERTOS, "%s create ERROR!!\r\n", task1_name);
        taskEXIT_CRITICAL();
        return;
    }

    ret = xTaskCreate(task2,
                      task2_name,
                      (configSTACK_DEPTH_TYPE)TASK_STK_SIZE,
                      NULL,
                      TASK_PRI,
                      &task2_handler);
    if (ret != pdPASS) {
        vSemaphoreDelete(binary);
        MS_LOGE(MS_FREERTOS, "%s create ERROR!!\r\n", task2_name);
        vTaskDelete(task1_handler);
        taskEXIT_CRITICAL();
        return;
    }

    ret = xTaskCreate(mgmt,
                      mgmt_name,
                      (configSTACK_DEPTH_TYPE)TASK_STK_SIZE,
                      NULL,
                      TASK_PRI,
                      &mgmt_handler);
    if (ret != pdPASS) {
        vSemaphoreDelete(binary);
        MS_LOGE(MS_FREERTOS, "%s create ERROR!!\r\n", mgmt_name);
        vTaskDelete(task1_handler);
        vTaskDelete(task2_handler);
        taskEXIT_CRITICAL();
        return;
    }

    taskEXIT_CRITICAL();

    xSemaphoreTake(binary, portMAX_DELAY);
    vSemaphoreDelete(binary);
}




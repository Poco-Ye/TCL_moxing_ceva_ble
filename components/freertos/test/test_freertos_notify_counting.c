/**
 * Copyright 2021 by MooreSilicon.All rights reserved
 * @file  test_freertos_notify_counting.c
 * @brief
 * @author weiquan.ou
 * @date 2022-3-31
 * @version 1.0
 * @Revision
 */

/* Kernel includes. */
#include "FreeRTOS.h" /* Must come first. */
#include "queue.h"    /* RTOS queue related API prototypes. */
#include "semphr.h"   /* Semaphore related API prototypes. */
#include "task.h"     /* RTOS task related API prototypes. */
#include "timers.h"   /* Software timer related API prototypes. */
#include "event_groups.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "log.h"
#include "unity_test_runner.h"

static TaskHandle_t task1_handler;
static TaskHandle_t task2_handler;
static char *task1_name = "notify_take_task";
static char *task2_name = "notify_give_task";
static SemaphoreHandle_t test_block;

#define TASK_STK_SIZE 256
#define TASK_PRI 2

static void task1(void *pst)
{
    int notify;

    while (1) {
        notify = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        MS_LOGI(MS_FREERTOS, "%s has notify %d !\r\n", task1_name, notify);
        vTaskDelay(10);
    }
}

static void task2(void *pst)
{
    int ret;
    int i;

    for (i = 0; i < 10; i++) {
        if (i % 3 == 0)
            vTaskDelay(1000);
        xTaskNotifyGive(task1_handler);
        MS_LOGI(MS_FREERTOS, "%s give notify %d !\r\n", task2_name, i);
    }

    ret = xSemaphoreGive(test_block);
    if (ret != pdTRUE) {
        MS_LOGE(MS_FREERTOS, "%s release binary semaphore ERROR !\r\n", task1_name);
    }

    vTaskDelete(NULL);
}

TEST_CASE("freertos","test_freertos_notify_counting", "[FreeRTOS/notify_counting]")
{
    int ret;

    taskENTER_CRITICAL();

    test_block = xSemaphoreCreateBinary();
    if (NULL == test_block) {
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
        vSemaphoreDelete(test_block);
        taskEXIT_CRITICAL();
        MS_LOGE(MS_FREERTOS, "%s create ERROR!!\r\n", task1_name);
        return;
    }

    ret = xTaskCreate(task2,
                      task2_name,
                      (configSTACK_DEPTH_TYPE)TASK_STK_SIZE,
                      NULL,
                      TASK_PRI,
                      &task2_handler);
    if (ret != pdPASS) {
        vSemaphoreDelete(test_block);
        vTaskDelete(task1_handler);
        taskEXIT_CRITICAL();
        MS_LOGE(MS_FREERTOS, "%s create ERROR!!\r\n", task2_name);
        return;
    }

    taskEXIT_CRITICAL();

    xSemaphoreTake(test_block, portMAX_DELAY);
    vSemaphoreDelete(test_block);
    vTaskDelete(task1_handler);
}





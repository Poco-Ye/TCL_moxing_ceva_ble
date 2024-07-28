/**
 * Copyright 2021 by MooreSilicon.All rights reserved
 * @file  test_freertos_semaphore_mutex.c
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
static TaskHandle_t task2_handler;
static TaskHandle_t task3_handler;
static char *task1_name = "mutex_high_task1";
static char *task2_name = "mutex_low_task2";
static char *task3_name = "mutex_middle_task3";

static SemaphoreHandle_t mutex;
static SemaphoreHandle_t test_block;

#define TASK_STK_SIZE 256

static void task1(void *pst)
{
    vTaskDelay(300);
    xSemaphoreTake(mutex, portMAX_DELAY);
    MS_LOGI(MS_FREERTOS, "%s running !!\r\n", task1_name);
    xSemaphoreGive(mutex);
    xSemaphoreGive(test_block);
    vTaskDelete(NULL);
}

static void task2(void *pst)
{
    int i;

    xSemaphoreTake(mutex, portMAX_DELAY);

    for (i = 0; i < 10; i++) {
        MS_LOGI(MS_FREERTOS, "%s running time[%d] priority[%d].\r\n", task2_name, i, uxTaskPriorityGet(task2_handler));
        vTaskDelay(100);
        taskYIELD();
    }

    xSemaphoreGive(mutex);

    vTaskDelete(NULL);
}

static void task3(void *pst)
{
    int i;

    for (i = 0; i < 10; i++) {
        MS_LOGI(MS_FREERTOS, "%s running time[%d].\r\n", task3_name, i);
        vTaskDelay(100);
        taskYIELD();
    }

    vTaskDelete(NULL);
}

TEST_CASE("freertos","test_freertos_mutex", "[FreeRTOS/mutex]")
{
    int ret;

    taskENTER_CRITICAL();

    mutex = xSemaphoreCreateMutex();
    if (NULL == mutex) {
        MS_LOGE(MS_FREERTOS, "create counting semaphore ERROR!!\r\n");
        taskENTER_CRITICAL();
        return;
    }

    test_block = xSemaphoreCreateBinary();
    if (NULL == test_block) {
        vSemaphoreDelete(mutex);
        MS_LOGE(MS_FREERTOS, "create Binary semaphore ERROR!!\r\r\n");
        taskEXIT_CRITICAL();
        return;
    }

    ret = xTaskCreate(task1,
                      task1_name,
                      (configSTACK_DEPTH_TYPE)TASK_STK_SIZE,
                      NULL,
                      6,
                      &task1_handler);
    if (ret != pdPASS) {
        MS_LOGE(MS_FREERTOS, "%s create ERROR!!\r\n", task1_name);
        vSemaphoreDelete(mutex);
        vSemaphoreDelete(test_block);
        taskEXIT_CRITICAL();
        return;
    }

    ret = xTaskCreate(task2,
                      task2_name,
                      (configSTACK_DEPTH_TYPE)TASK_STK_SIZE,
                      NULL,
                      2,
                      &task2_handler);
    if (ret != pdPASS) {
        MS_LOGE(MS_FREERTOS, "%s create ERROR!!\r\n", task2_name);
        vTaskDelete(task1_handler);
        vSemaphoreDelete(mutex);
        vSemaphoreDelete(test_block);
        taskEXIT_CRITICAL();
        return;
    }

    ret = xTaskCreate(task3,
                      task3_name,
                      (configSTACK_DEPTH_TYPE)TASK_STK_SIZE,
                      NULL,
                      4,
                      &task3_handler);
    if (ret != pdPASS) {
        MS_LOGE(MS_FREERTOS, "%s create ERROR!!\r\n", task3_name);
        vTaskDelete(task2_handler);
        vTaskDelete(task1_handler);
        vSemaphoreDelete(test_block);
        vSemaphoreDelete(mutex);
    }

    taskEXIT_CRITICAL();

    xSemaphoreTake(test_block, portMAX_DELAY);
    vSemaphoreDelete(test_block);
    vSemaphoreDelete(mutex);
}









/**
 * Copyright 2021 by MooreSilicon.All rights reserved
 * @file  test_freertos_semaphore_counting.c
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
static char *task1_name = "counting_seamphore_task1";
static char *task2_name = "counting_seamphore_task2";
static SemaphoreHandle_t counting_seamphore;
static SemaphoreHandle_t test_block;

#define TASK_STK_SIZE 256
#define TASK_PRI 2
#define SEMAPHORE_COUNTING_NUM 5

static void task1(void *pst)
{
    int ret;
    int i;

    for (i = 0; i < SEMAPHORE_COUNTING_NUM; i++) {
        ret = xSemaphoreTake(counting_seamphore, 1000);
        if (ret != pdTRUE) {
            MS_LOGE(MS_FREERTOS, "%s take counting semaphore[%d] ERROR!!\r\n", task1_name, i);
            vTaskDelete(NULL);
            return;
        }

        MS_LOGI(MS_FREERTOS, "%s take counting semaphore[%d] success !!\r\n", task1_name, i);
    }

    ret = xSemaphoreGive(test_block);
    if (ret != pdTRUE) {
        MS_LOGI(MS_FREERTOS, "%s release test block ERROR !\r\n", task1_name);
    }

    vTaskDelete(NULL);
}

static void task2(void *pst)
{
    int ret;
    int i;

    MS_LOGI(MS_FREERTOS, "%s running.\r\n", task2_name);

    for (i = 0; i < SEMAPHORE_COUNTING_NUM; i++) {
        vTaskDelay(80);

        ret = xSemaphoreGive(counting_seamphore);
        if (ret != pdTRUE) {
            MS_LOGI(MS_FREERTOS, "%s release counting semaphore[%d] ERROR !\r\n", task2_name, i);
            vTaskDelete(NULL);
            return;
        }

        MS_LOGI(MS_FREERTOS, "%s release counting semaphore[%d] success !\r\n", task2_name, i);
    }

    vTaskDelete(NULL);
}

TEST_CASE("freertos","test_freertos_semaphore_counting", "[FreeRTOS/semaphore_counting]")
{
    int ret;

    taskENTER_CRITICAL();

    counting_seamphore = xSemaphoreCreateCounting(SEMAPHORE_COUNTING_NUM, 0);
    if (NULL == counting_seamphore) {
        MS_LOGE(MS_FREERTOS, "create counting semaphore ERROR!!\r\n");
        taskEXIT_CRITICAL();
        return;
    }

    test_block = xSemaphoreCreateBinary();
    if (NULL == test_block) {
        vSemaphoreDelete(counting_seamphore);
        MS_LOGE(MS_FREERTOS, "create Binary semaphore ERROR!!\r\n");
        taskEXIT_CRITICAL();
        return;
    }

    ret = xTaskCreate(task1,
                      task1_name,
                      (configSTACK_DEPTH_TYPE)TASK_STK_SIZE,
                      NULL,
                      TASK_PRI,
                      &task1_handler);
    if (ret != pdPASS) {
        MS_LOGE(MS_FREERTOS, "%s create ERROR!!\r\n", task1_name);
        vSemaphoreDelete(counting_seamphore);
        vSemaphoreDelete(test_block);
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
        MS_LOGE(MS_FREERTOS, "%s create ERROR!!\r\n", task2_name);
        vTaskDelete(task1_handler);
        vSemaphoreDelete(counting_seamphore);
        vSemaphoreDelete(test_block);
        taskEXIT_CRITICAL();
        return;
    }

    taskEXIT_CRITICAL();

    xSemaphoreTake(test_block, portMAX_DELAY);
    vSemaphoreDelete(test_block);
    vSemaphoreDelete(counting_seamphore);
}








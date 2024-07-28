/**
 * Copyright 2021 by MooreSilicon.All rights reserved
 * @file  test_freertos_semaphore.c
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
static char *task1_name = "bin_seamphore_task1";
static char *task2_name = "bin_seamphore_task2";
static SemaphoreHandle_t bin_seamphore;
static SemaphoreHandle_t test_block;

#define TASK_STK_SIZE 256
#define TASK_PRI 2

static void task1(void *pst)
{
    int ret;

    ret = xSemaphoreTake(bin_seamphore, 1000);
    if (ret != pdTRUE) {
        MS_LOGE(MS_FREERTOS, "%s take binary semaphore ERROR!!\r\n", task1_name);
        vTaskDelete(NULL);
        return;
    }

    MS_LOGI(MS_FREERTOS, "%s take binary semaphore success !!\r\n", task1_name);

    ret = xSemaphoreGive(test_block);
    if (ret != pdTRUE) {
        MS_LOGI(MS_FREERTOS, "%s release binary semaphore ERROR !\r\n", task1_name);
    }

    vTaskDelete(NULL);
}

static void task2(void *pst)
{
    int ret;

    MS_LOGI(MS_FREERTOS, "%s running.\r\n", task2_name);
    vTaskDelay(80);

    ret = xSemaphoreGive(bin_seamphore);
    if (ret != pdTRUE) {
        MS_LOGI(MS_FREERTOS, "%s release binary semaphore ERROR !\r\n", task2_name);
        vTaskDelete(NULL);
        return;
    }

    MS_LOGI(MS_FREERTOS, "%s release binary semaphore success !\r\n", task2_name);

    vTaskDelete(NULL);
}

TEST_CASE("freertos","test_freertos_semaphore", "[FreeRTOS/semaphore]")
{
    int ret;

    taskENTER_CRITICAL();

    bin_seamphore = xSemaphoreCreateBinary();
    if (NULL == bin_seamphore) {
        MS_LOGE(MS_FREERTOS, "create binary semaphore ERROR!!\r\n");
        taskEXIT_CRITICAL();
        return;
    }

    test_block = xSemaphoreCreateBinary();
    if (NULL == test_block) {
        vSemaphoreDelete(bin_seamphore);
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
        MS_LOGE(MS_FREERTOS, "%s create ERROR!!\r\n", task1_name);
        vSemaphoreDelete(bin_seamphore);
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
        vSemaphoreDelete(bin_seamphore);
        vSemaphoreDelete(test_block);
        taskEXIT_CRITICAL();
        return;
    }

    taskEXIT_CRITICAL();
    xSemaphoreTake(test_block, portMAX_DELAY);
    vSemaphoreDelete(test_block);
    vSemaphoreDelete(bin_seamphore);
}







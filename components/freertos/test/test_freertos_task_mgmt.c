/**
 * Copyright 2021 by MooreSilicon.All rights reserved
 * @file  test_freertos_task_mgnt.c
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

#define TASK1_NAME "mgmt_task1"
#define TASK2_NAME "mgmt_task2"

#define TASK_STK_SIZE 256
#define TASK_PRI 2
#define TASK_END_NUM 5

struct test_freertos_task_mgmt_s {
    TaskHandle_t task1_handler;
    TaskHandle_t task2_handler;
    SemaphoreHandle_t binary;
};

static void task1(void *pst)
{
    struct test_freertos_task_mgmt_s *tcb = (struct test_freertos_task_mgmt_s *)pst;
    int num = 0;
    int ret;

    while(num < TASK_END_NUM) {
        num++;
        MS_LOGI(MS_FREERTOS, "%s has already run %d times\r\n", pcTaskGetName(tcb->task1_handler), num);
        vTaskDelay(1000);
    }

    MS_LOGI(MS_FREERTOS, "%s delete %s\r\n", pcTaskGetName(tcb->task1_handler), pcTaskGetName(tcb->task2_handler));
    vTaskDelete(tcb->task2_handler);
    tcb->task2_handler = NULL;

    ret = xSemaphoreGive(tcb->binary);
    if (ret != pdTRUE) {
        MS_LOGI(MS_FREERTOS, "release binary semaphore ERROR !\r\n");
    }

    tcb->task1_handler = NULL;
    vTaskDelete(NULL);
}

static void task2(void *pst)
{
    struct test_freertos_task_mgmt_s *tcb = (struct test_freertos_task_mgmt_s *)pst;
    int num = 0;
    int ret;

    while(num < TASK_END_NUM) {
        num++;
        MS_LOGI(MS_FREERTOS, "%s has already run %d times\r\n", pcTaskGetName(tcb->task2_handler), num);
        vTaskDelay(1000);
    }

    MS_LOGI(MS_FREERTOS, "%s delete %s\r\n", pcTaskGetName(tcb->task2_handler), pcTaskGetName(tcb->task1_handler));
    vTaskDelete(tcb->task1_handler);
    tcb->task1_handler = NULL;

    ret = xSemaphoreGive(tcb->binary);
    if (ret != pdTRUE) {
        MS_LOGI(MS_FREERTOS, "release binary semaphore ERROR !\r\n");
    }

    tcb->task2_handler = NULL;
    vTaskDelete(NULL);
}


TEST_CASE("freertos","test_freertos_task_mgmt", "[FreeRTOS/task_create_mgmt]")
{
    struct test_freertos_task_mgmt_s *tcb;
    int ret;

    tcb = pvPortMalloc(sizeof(struct test_freertos_task_mgmt_s));
    if (tcb == NULL) {
        MS_LOGE(MS_FREERTOS, "malloc tcb ERROR!!\n");
        return;
    }

    taskENTER_CRITICAL();

    tcb->binary = xSemaphoreCreateBinary();
    if (NULL == tcb->binary) {
        vPortFree(tcb);
        taskEXIT_CRITICAL();
        MS_LOGE(MS_FREERTOS, "create binary semaphore ERROR!!\n");
        return;
    }

    ret = xTaskCreate(task1,
                      TASK1_NAME,
                      (configSTACK_DEPTH_TYPE)TASK_STK_SIZE,
                      tcb,
                      TASK_PRI,
                      &tcb->task1_handler);
    if (ret != pdPASS) {
        vSemaphoreDelete(tcb->binary);
        vPortFree(tcb);
        taskEXIT_CRITICAL();
        MS_LOGE(MS_FREERTOS, "%s create ERROR!!\n", TASK1_NAME);
        return;
    }

    ret = xTaskCreate(task2,
                      TASK2_NAME,
                      (configSTACK_DEPTH_TYPE)TASK_STK_SIZE,
                      tcb,
                      TASK_PRI,
                      &tcb->task2_handler);
    if (ret != pdPASS) {
        vTaskDelete(tcb->task1_handler);
        vSemaphoreDelete(tcb->binary);
        vPortFree(tcb);
        taskEXIT_CRITICAL();
        MS_LOGE(MS_FREERTOS, "%s create ERROR!!\n", TASK2_NAME);
        return;
    }

    taskEXIT_CRITICAL();

    xSemaphoreTake(tcb->binary, portMAX_DELAY);
    vSemaphoreDelete(tcb->binary);
    vPortFree(tcb);
}



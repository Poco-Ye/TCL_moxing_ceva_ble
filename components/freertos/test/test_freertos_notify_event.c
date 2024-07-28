/**
 * Copyright 2021 by MooreSilicon.All rights reserved
 * @file  test_freertos_notify_event.c
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
#define TEST_FREEROTS_NOTIFY_EVENTBIT0 (1 << 0)
#define TEST_FREEROTS_NOTIFY_EVENTBIT1 (1 << 1)
#define TEST_FREEROTS_NOTIFY_EVENTBIT2 (1 << 2)
#define TEST_FREEROTS_NOTIFY_EVENTBIT_ALL (TEST_FREEROTS_NOTIFY_EVENTBIT0 | TEST_FREEROTS_NOTIFY_EVENTBIT1 | TEST_FREEROTS_NOTIFY_EVENTBIT2)

static void task1(void *pst)
{
    unsigned long notify;
    int ret;

    while (1) {
        ret = xTaskNotifyWait(0x0, 0xffffffff, &notify, portMAX_DELAY);
        if (ret == pdTRUE)
            MS_LOGI(MS_FREERTOS, "%s has notify %d !\r\n", task1_name, notify);
    }
}

static void task2(void *pst)
{
    int ret;
    int i;
    int notify;

    for (i = 0; i < 9; i++) {
        vTaskDelay(1000);

        switch (i) {
            case 0:
            case 3:
            case 6:
                notify = TEST_FREEROTS_NOTIFY_EVENTBIT0;
                break;
            case 1:
            case 4:
            case 7:
                notify = TEST_FREEROTS_NOTIFY_EVENTBIT1;
                break;
            case 2:
            case 5:
            case 8:
                notify = TEST_FREEROTS_NOTIFY_EVENTBIT2;
                break;
            default:
                notify = 0;
                break;
        }

        ret = xTaskNotify(task1_handler, notify, eSetBits);
        if (ret == pdPASS)
            MS_LOGI(MS_FREERTOS, "%s give notify[%d] !\r\n", task2_name, notify);
    }

    ret = xSemaphoreGive(test_block);
    if (ret != pdTRUE)
        MS_LOGE(MS_FREERTOS, "%s release binary semaphore ERROR !\r\n", task2_name);

    vTaskDelete(NULL);
}

TEST_CASE("freertos","test_freertos_notify_event", "[FreeRTOS/notify_event]")
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






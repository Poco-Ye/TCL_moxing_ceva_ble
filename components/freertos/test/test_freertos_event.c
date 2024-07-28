/**
 * Copyright 2021 by MooreSilicon.All rights reserved
 * @file  test_freertos_event.c
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
#include "event_groups.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "log.h"
#include "unity_test_runner.h"

static TaskHandle_t task1_handler;
static TaskHandle_t task2_handler;
static TaskHandle_t task3_handler;
static char *task1_name = "event_gen_task";
static char *task2_name = "event_wait_task";
static char *task3_name = "event_get_task";
static EventGroupHandle_t event_handler;
static SemaphoreHandle_t test_block;

#define TASK_STK_SIZE 256
#define TASK_PRI 2
#define TEST_FREEROTS_EVENTBIT0 (1 << 0)
#define TEST_FREEROTS_EVENTBIT1 (1 << 1)
#define TEST_FREEROTS_EVENTBIT2 (1 << 2)
#define TEST_FREEROTS_EVENTBIT_ALL (TEST_FREEROTS_EVENTBIT0 | TEST_FREEROTS_EVENTBIT1 | TEST_FREEROTS_EVENTBIT2)

static void task1(void *pst)
{
    int i;
    int ret;

    for (i = 0; i < 6; i++) {
        switch (i) {
            case 0:
            case 3:
                xEventGroupSetBits(event_handler, TEST_FREEROTS_EVENTBIT0);
                MS_LOGI(MS_FREERTOS, "event[%d] set\r\n", TEST_FREEROTS_EVENTBIT0);
                break;
            case 1:
            case 4:
                xEventGroupSetBits(event_handler, TEST_FREEROTS_EVENTBIT1);
                MS_LOGI(MS_FREERTOS, "event[%d] set\r\n", TEST_FREEROTS_EVENTBIT1);
                break;
            case 2:
            case 5:
                xEventGroupSetBits(event_handler, TEST_FREEROTS_EVENTBIT2);
                MS_LOGI(MS_FREERTOS, "event[%d] set\r\n", TEST_FREEROTS_EVENTBIT2);
                break;
            default:
                break;
        }
        vTaskDelay(300);
    }

    ret = xSemaphoreGive(test_block);
    if (ret != pdTRUE) {
        MS_LOGE(MS_FREERTOS, "%s release binary semaphore ERROR !\r\n", task1_name);
    }

    vTaskDelete(NULL);
}

static void task2(void *pst)
{
    int event;

    while (1) {
        event = xEventGroupWaitBits(event_handler, TEST_FREEROTS_EVENTBIT_ALL, pdTRUE, pdTRUE, portMAX_DELAY);
        MS_LOGI(MS_FREERTOS, "%s has event[0x%x] \r\n", task2_name, event);
    }
}

static void task3(void *pst)
{
    int event;
    int last = 0;

    while (1) {
        event = xEventGroupGetBits(event_handler);
        if (event != last) {
            last = event;
            MS_LOGI(MS_FREERTOS, "%s has event[0x%x] \r\n", task3_name, event);
        }
        vTaskDelay(300);
    }
}


TEST_CASE("freertos","test_freertos_event", "[FreeRTOS/event]")
{
    int ret;

    taskENTER_CRITICAL();

    event_handler = xEventGroupCreate();
    if (NULL == event_handler) {
        taskEXIT_CRITICAL();
        MS_LOGE(MS_FREERTOS, "create event group ERROR!!\r\n");
        return;
    }

    test_block = xSemaphoreCreateBinary();
    if (NULL == test_block) {
        vEventGroupDelete(event_handler);
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
        vEventGroupDelete(event_handler);
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
        vEventGroupDelete(event_handler);
        vSemaphoreDelete(test_block);
        vTaskDelete(task1_handler);
        taskEXIT_CRITICAL();
        MS_LOGE(MS_FREERTOS, "%s create ERROR!!\r\n", task2_name);
        return;
    }

    ret = xTaskCreate(task3,
                      task3_name,
                      (configSTACK_DEPTH_TYPE)TASK_STK_SIZE,
                      NULL,
                      TASK_PRI,
                      &task3_handler);
    if (ret != pdPASS) {
        vEventGroupDelete(event_handler);
        vSemaphoreDelete(test_block);
        vTaskDelete(task1_handler);
        vTaskDelete(task2_handler);
        taskEXIT_CRITICAL();
        MS_LOGE(MS_FREERTOS, "%s create ERROR!!\r\n", task3_name);
        return;
    }

    taskEXIT_CRITICAL();

    xSemaphoreTake(test_block, portMAX_DELAY);
    vSemaphoreDelete(test_block);
    vEventGroupDelete(event_handler);
    vTaskDelete(task2_handler);
    vTaskDelete(task3_handler);
}



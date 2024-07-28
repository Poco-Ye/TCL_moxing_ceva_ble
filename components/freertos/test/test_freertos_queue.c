/**
 * Copyright 2021 by MooreSilicon.All rights reserved
 * @file  test_freertos_queue.c
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
static char *task1_name = "queue_task1";
static char *task2_name = "queue_task2";
static QueueHandle_t queue;
static SemaphoreHandle_t test_block;

struct msg_s {
    int type;
    int state;
    int value;
};

#define TASK_STK_SIZE 256
#define TASK_PRI 2
#define TEST_QUEUE_LEN 10

static void queue_show(void)
{
    int remain;
    int total;

    remain = uxQueueSpacesAvailable(queue);
    total = remain + uxQueueMessagesWaiting(queue);
    MS_LOGI(MS_FREERTOS, "total size: %d, remain size: %d\r\n", total, remain);
}

static void task1(void *pst)
{
    struct msg_s msg;
    int i;
    int ret;

    for (i = 0; i < TEST_QUEUE_LEN; i++) {
        memset(&msg, 0, sizeof(msg));
        msg.value = i;
        ret = xQueueSend(queue, &msg, 10);
        if (ret != pdTRUE) {
            MS_LOGE(MS_FREERTOS, "%s queue send ERROR!!\r\n", task1_name);
            return;
        }

        MS_LOGI(MS_FREERTOS, "%s SEND msg[%d].type = %d state = %d value = %d\r\n",
            task1_name, i, msg.type, msg.state, msg.value);
        queue_show();
        vTaskDelay(100);
    }

    vTaskDelete(NULL);
}

static void task2(void *pst)
{
    struct msg_s msg;
    int i;
    int ret;

    for (i = 0; i < TEST_QUEUE_LEN; i++) {
        memset(&msg, 0, sizeof(msg));
        ret = xQueueReceive(queue, &msg, 100);
        if (ret != pdTRUE) {
            MS_LOGE(MS_FREERTOS, "%s queue receive ERROR!!\r\n", task2_name);
            return;
        }

        MS_LOGI(MS_FREERTOS, "%s RCV msg[%d].type = %d state = %d value = %d\r\n",
            task2_name, i, msg.type, msg.state, msg.value);
        vTaskDelay(300);
    }

    ret = xSemaphoreGive(test_block);
    if (ret != pdTRUE) {
        MS_LOGI(MS_FREERTOS, "%s release binary semaphore ERROR !\r\n", task2_name);
    }

    vTaskDelete(NULL);
}

TEST_CASE("freertos","test_freertos_queue", "[FreeRTOS/queue]")
{
    int ret;

    taskENTER_CRITICAL();

    queue = xQueueCreate(TEST_QUEUE_LEN, sizeof(struct msg_s));
    if (NULL == queue) {
        taskEXIT_CRITICAL();
        MS_LOGE(MS_FREERTOS, "create queue ERROR!!\r\n");
        return;
    }

    test_block = xSemaphoreCreateBinary();
    if (NULL == test_block) {
        vQueueDelete(queue);
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
        vQueueDelete(queue);
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
        vQueueDelete(queue);
        vSemaphoreDelete(test_block);
        taskEXIT_CRITICAL();
        return;
    }

    taskEXIT_CRITICAL();

    xSemaphoreTake(test_block, portMAX_DELAY);
    vSemaphoreDelete(test_block);
    vQueueDelete(queue);
}






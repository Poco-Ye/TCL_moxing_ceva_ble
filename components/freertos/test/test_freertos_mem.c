/**
 * Copyright 2021 by MooreSilicon.All rights reserved
 * @file  test_freertos_mem.c
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
static char *task1_name = "mem_task";
static SemaphoreHandle_t test_block;
static int g_size[] = {32, 33, 64, 65, 127, 128, 255, 256, 511, 512, 1023, 1024, 2048};

#define TASK_STK_SIZE 256
#define TASK_PRI 2

static void task1(void *pst)
{
    int ret;
    int i;
    int size;
    char *buf;
    int idx;

    MS_LOGI(MS_FREERTOS, "start free memory %d !\r\n", xPortGetFreeHeapSize());

    for (i = 0; i < 1000; i++) {
        idx = i % (sizeof(g_size) / 4);
        size = g_size[idx];
        buf = pvPortMalloc(size);
        if (buf == NULL)
            MS_LOGE(MS_FREERTOS, "size[%d] malloc ERROR !\r\n", size);
        memset(buf, 0x5a, size);
        vTaskDelay(10);
        vPortFree(buf);

        vTaskDelay(10);
    }

    MS_LOGI(MS_FREERTOS, "end free memory %d !\r\n", xPortGetFreeHeapSize());

    ret = xSemaphoreGive(test_block);
    if (ret != pdTRUE)
        MS_LOGE(MS_FREERTOS, "%s release binary semaphore ERROR !\r\n", task1_name);

    vTaskDelete(NULL);
}

TEST_CASE("freertos","test_freertos_mem", "[FreeRTOS/mem]")
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

    taskEXIT_CRITICAL();

    xSemaphoreTake(test_block, portMAX_DELAY);
    vSemaphoreDelete(test_block);
}






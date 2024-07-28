/**
 * Copyright 2021 by MooreSilicon.All rights reserved
 * @file  test_freertos_task_stat.c
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
#include <string.h>

#include "log.h"
#include "unity_test_runner.h"

#define TASK1_NAME "stat_task1"
#define TASK2_NAME "stat_task2"
#define MGMT_NAME "stat_mgmt"

#define TASK_STK_SIZE 256
#define TASK_PRI 2
#define TASK_END_NUM 5

struct test_freertos_task_stat_tcb {
    TaskHandle_t task1_handler;
    TaskHandle_t task2_handler;
    TaskHandle_t mgmt_handler;
    SemaphoreHandle_t binary;
};

static void task_state_buf(eTaskState state, char *buf)
{
    switch (state) {
        case eRunning:
            sprintf(buf, "running");
            break;
        case eReady:
            sprintf(buf, "ready");
            break;
        case eBlocked:
            sprintf(buf, "block");
            break;
        case eSuspended:
            sprintf(buf, "suspend");
            break;
        case eDeleted:
            sprintf(buf, "delete");
            break;
        case eInvalid:
            sprintf(buf, "invalid");
            break;
        default:
            break;
    }
}

static void task_info_show(TaskHandle_t handler)
{
    TaskStatus_t *status;

    status = pvPortMalloc(sizeof(TaskStatus_t));
    if (NULL == status) {
        MS_LOGE(MS_FREERTOS, "malloc ERROR!!\r\n");
        return;
    }

    vTaskGetInfo(handler, status, pdTRUE, eInvalid);
    MS_LOGI(MS_FREERTOS, "task name: \t\t%s\r\n", status->pcTaskName);
    MS_LOGI(MS_FREERTOS, "task number: \t\t%d\r\n", status->xTaskNumber);
    MS_LOGI(MS_FREERTOS, "current priority: \t%d\r\n", status->uxCurrentPriority);
    MS_LOGI(MS_FREERTOS, "base priority: \t\t%d\r\n", status->uxBasePriority);
    MS_LOGI(MS_FREERTOS, "stack base addr: \t0x%x\r\n", status->pxStackBase);
    MS_LOGI(MS_FREERTOS, "stack overload: \t%d\r\n", status->usStackHighWaterMark);
    MS_LOGI(MS_FREERTOS, "state: \t\t\t%d\r\n", status->eCurrentState);

    vPortFree(status);
}

static void task_state_show(TaskHandle_t handler)
{
    eTaskState state;
    char *buf;

    buf = pvPortMalloc(32);
    if (NULL == buf) {
        MS_LOGE(MS_FREERTOS, "malloc ERROR!!\r\n");
        return;
    }

    state = eTaskGetState(handler);
    memset(buf, 0, 32);
    task_state_buf(state, buf);
    MS_LOGI(MS_FREERTOS, "%s task state: %d %s\r\n", pcTaskGetName(handler), state, buf);

    vPortFree(buf);
}

void task_list_show(void)
{
    char *buf;

    MS_LOGI(MS_FREERTOS, "\r\n********** vTaskList test **********\r\n");
    buf = pvPortMalloc(1024);
    if (NULL == buf) {
        MS_LOGE(MS_FREERTOS, "malloc ERROR!!\r\n");
        return;
    }

    vTaskList(buf);
    MS_LOGI(MS_FREERTOS, "%s\r\n", buf);
    vPortFree(buf);
}

static void task_stat_get(struct test_freertos_task_stat_tcb *tcb)
{
    TaskStatus_t *status;
    eTaskState state;
    unsigned long run_time;
    char *buf;
    int size;
    int i;

    MS_LOGI(MS_FREERTOS, "\r\n********** uxTaskGetSystemState test **********\r\n");
    size = uxTaskGetNumberOfTasks();
    size = size * sizeof(TaskStatus_t);
    status = pvPortMalloc(size);
    if (NULL == status) {
        MS_LOGE(MS_FREERTOS, "malloc ERROR!!\r\n");
        return;
    }

    size = uxTaskGetSystemState(status, size, &run_time);

    MS_LOGI(MS_FREERTOS, "task name\t\tpriority\ttask number\r\n");
    for (i = 0; i < size; i++) {
        MS_LOGI(MS_FREERTOS, "%s\t\t%d\t\t%d\t\t\r\n",
            status[i].pcTaskName, status[i].uxCurrentPriority, status[i].xTaskNumber);
    }

    vPortFree(status);

    MS_LOGI(MS_FREERTOS, "\r\n********** vTaskGetInfo test **********\r\n");
    if (tcb->task1_handler != NULL)
        task_info_show(tcb->task1_handler);
    if (tcb->task2_handler != NULL)
        task_info_show(tcb->task2_handler);

    MS_LOGI(MS_FREERTOS, "\r\n********** eTaskGetState test **********\r\n");
    if (tcb->task1_handler != NULL)
        task_state_show(tcb->task1_handler);
    if (tcb->task2_handler != NULL)
        task_state_show(tcb->task2_handler);

    task_list_show();
}

static void task1(void *pst)
{
    int num = 0;

    while(1) {
        num++;
        MS_LOGI(MS_FREERTOS, "%s has already run %d times\r\n",
            pcTaskGetName(xTaskGetCurrentTaskHandle()), num);
        vTaskDelay(1000);
    }
}

static void task2(void *pst)
{
    int num = 0;

    while(1) {
        num++;
        MS_LOGI(MS_FREERTOS, "%s has already run %d times\r\n",
            pcTaskGetName(xTaskGetCurrentTaskHandle()), num);
        vTaskDelay(1000);
    }
}

static void mgmt(void *pst)
{
    struct test_freertos_task_stat_tcb *tcb = (struct test_freertos_task_stat_tcb *)pst;
    int num = 0;
    int ret;

    while(num < TASK_END_NUM) {
        vTaskDelay(1000);
        switch(num) {
            case 0:
                if (tcb->task1_handler != NULL) {
                    vTaskSuspend(tcb->task1_handler);
                    MS_LOGI(MS_FREERTOS, "\r\n === %s suspend. ===\r\n", pcTaskGetName(tcb->task1_handler));
                }
                break;
            case 1:
                if (tcb->task2_handler != NULL) {
                    vTaskSuspend(tcb->task2_handler);
                    MS_LOGI(MS_FREERTOS, "\r\n === %s suspend. ===\r\n", pcTaskGetName(tcb->task2_handler));
                }
                break;
            case 2:
                if (tcb->task1_handler != NULL) {
                    vTaskResume(tcb->task1_handler);
                    MS_LOGI(MS_FREERTOS, "\r\n === %s resume. ===\r\n", pcTaskGetName(tcb->task1_handler));
                }
                break;
            case 3:
                if (tcb->task2_handler != NULL) {
                    vTaskResume(tcb->task2_handler);
                    MS_LOGI(MS_FREERTOS, "\r\n === %s resume. ===\r\n", pcTaskGetName(tcb->task2_handler));
                }
                break;
            case 4:
                if (tcb->task1_handler != NULL && tcb->task2_handler != NULL) {
                    MS_LOGI(MS_FREERTOS, "\r\n === delete %s %s. ===\r\n", pcTaskGetName(tcb->task1_handler), pcTaskGetName(tcb->task2_handler));
                    vTaskDelete(tcb->task1_handler);
                    vTaskDelete(tcb->task2_handler);
                    tcb->task1_handler = NULL;
                    tcb->task2_handler = NULL;
                }
                break;
            default:
                break;
        }

        task_stat_get(tcb);
        num++;
    }

    ret = xSemaphoreGive(tcb->binary);
    if (ret != pdTRUE) {
        MS_LOGI(MS_FREERTOS, "release binary semaphore ERROR !\r\n");
    }

    tcb->mgmt_handler = NULL;
    vTaskDelete(NULL);
}


TEST_CASE("freertos","test_freertos_task_stat", "[FreeRTOS/task_stat]")
{
    struct test_freertos_task_stat_tcb *tcb;
    int ret;

    tcb = pvPortMalloc(sizeof(struct test_freertos_task_stat_tcb));
    if (NULL == tcb) {
        MS_LOGE(MS_FREERTOS, "create tcb mem ERROR!!\r\n");
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
                      NULL,
                      TASK_PRI,
                      &tcb->task1_handler);
    if (ret != pdPASS) {
        vSemaphoreDelete(tcb->binary);
        vPortFree(tcb);
        MS_LOGE(MS_FREERTOS, "%s create ERROR!!\r\n", TASK1_NAME);
        taskEXIT_CRITICAL();
        return;
    }

    ret = xTaskCreate(task2,
                      TASK2_NAME,
                      (configSTACK_DEPTH_TYPE)TASK_STK_SIZE,
                      NULL,
                      TASK_PRI,
                      &tcb->task2_handler);
    if (ret != pdPASS) {
        vTaskDelete(tcb->task1_handler);
        vSemaphoreDelete(tcb->binary);
        vPortFree(tcb);
        MS_LOGE(MS_FREERTOS, "%s create ERROR!!\r\n", TASK2_NAME);
        taskEXIT_CRITICAL();
        return;
    }

    ret = xTaskCreate(mgmt,
                      MGMT_NAME,
                      (configSTACK_DEPTH_TYPE)TASK_STK_SIZE,
                      tcb,
                      TASK_PRI,
                      &tcb->mgmt_handler);
    if (ret != pdPASS) {
        vTaskDelete(tcb->task1_handler);
        vTaskDelete(tcb->task2_handler);
        vSemaphoreDelete(tcb->binary);
        vPortFree(tcb);
        MS_LOGE(MS_FREERTOS, "%s create ERROR!!\r\n", MGMT_NAME);
        taskEXIT_CRITICAL();
        return;
    }

    taskEXIT_CRITICAL();

    xSemaphoreTake(tcb->binary, portMAX_DELAY);
    if (tcb->mgmt_handler != NULL)
        vTaskDelete(tcb->mgmt_handler);
    vSemaphoreDelete(tcb->binary);
    vPortFree(tcb);

    task_list_show();
}





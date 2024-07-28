/**
 * Copyright 2021 by MooreSilicon.All rights reserved
 * @file  test_freertos_list.c
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

static TaskHandle_t list_task_handler;
static char *list_task_name = "list_task";
static SemaphoreHandle_t test_block;

#define TASK_STK_SIZE 256
#define TASK_PRI 2

struct node_s {
    int value;
    ListItem_t item;
};

static void list_show(List_t *list)
{
    ListItem_t *item;
    struct node_s *node;
    int i;

    if (listLIST_IS_EMPTY(list)) {
        MS_LOGI(MS_FREERTOS, "list empty\r\n");
        return;
    }

    MS_LOGI(MS_FREERTOS, "list show start\r\n");
    for (item = listGET_HEAD_ENTRY(list), i = 0;
         item != listGET_END_MARKER(list);
         item = listGET_NEXT(item), i++) {
        node = item->pvOwner;
        MS_LOGI(MS_FREERTOS, "item[%d] : %d\r\n", i, node->value);
    }
    MS_LOGI(MS_FREERTOS, "list show end\r\n");
}

static void list_clean(List_t *list)
{
    ListItem_t *item;
    struct node_s *node;

    while (!listLIST_IS_EMPTY(list)) {
        item = listGET_HEAD_ENTRY(list);
        listREMOVE_ITEM(item);
        node = item->pvOwner;
        vPortFree(node);
    }
}

static void list_task(void *pst)
{
    int i;
    int ret;
    List_t test_list;
    struct node_s *node;

    vListInitialise(&test_list);

    for (i = 0; i < 8; i++) {
        node = pvPortMalloc(sizeof(struct node_s));
        if (NULL == node) {
            MS_LOGE(MS_FREERTOS, "malloc ERROR!!\r\n");
            list_clean(&test_list);
            vTaskDelete(NULL);
            return;
        }

        vListInitialiseItem(&node->item);
        node->item.pvOwner = node;
        node->value = i;
        vListInsert(&test_list, &node->item);
        list_show(&test_list);

        vTaskDelay(1000);
    }

    list_clean(&test_list);
    list_show(&test_list);

    ret = xSemaphoreGive(test_block);
    if (ret != pdTRUE) {
        MS_LOGI(MS_FREERTOS, "release binary semaphore ERROR !\r\n");
    }

    vTaskDelete(NULL);
}


TEST_CASE("freertos","test_freertos_list", "[FreeRTOS/list]")
{
    int ret;

    taskENTER_CRITICAL();

    test_block = xSemaphoreCreateBinary();
    if (NULL == test_block) {
        taskEXIT_CRITICAL();
        MS_LOGE(MS_FREERTOS, "create binary semaphore ERROR!!\r\n");
        return;
    }

    ret = xTaskCreate(list_task,
                      list_task_name,
                      (configSTACK_DEPTH_TYPE)TASK_STK_SIZE,
                      NULL,
                      TASK_PRI,
                      &list_task_handler);
    if (ret != pdPASS) {
        MS_LOGE(MS_FREERTOS, "%s create ERROR!!\r\n", list_task_name);
        vSemaphoreDelete(test_block);
        taskEXIT_CRITICAL();
        return;
    }

    taskEXIT_CRITICAL();
    xSemaphoreTake(test_block, portMAX_DELAY);
    vSemaphoreDelete(test_block);
}





/**
 * Copyright 2021 by MooreSilicon.All rights reserved
 * @file  key_mgmt_connection_native.c
 * @brief
 * @author weiquan.ou
 * @date 2022-4-27
 * @version 1.0
 * @Revision
 */

static SemaphoreHandle_t g_key_mgmt_connect_native_binary = NULL;
static TaskHandle_t g_key_mgmt_connect_native_task_handler;
static unsigned long g_key_mgmt_connect_native_timeout = 0;

static void key_mgmt_connect_native_timer(TimerHandle_t timer)
{
    /* 设置超时标记 */
    g_key_mgmt_connect_native_timeout = 1;
}

static void key_mgmt_connect_native_task(void *pst)
{
    TimerHandle_t timer;
    unsigned long timeout;
    int err;

    /* 按键超过1s才执行连接处理 */
    if (xSemaphoreTake(g_key_mgmt_connect_native_binary, 1000) == pdTRUE) {

        /* 1s内收到信号量表明按键松开了 */
        vSemaphoreDelete(g_key_mgmt_connect_native_binary);
        g_key_mgmt_connect_native_binary = NULL;
        vTaskDelete(NULL);
        return;
    }

    vSemaphoreDelete(g_key_mgmt_connect_native_binary);
    g_key_mgmt_connect_native_binary = NULL;

    /* 发送三次红外键值0x00A0 */

    key_mgmt_ble_connect_timeout_get(&timeout);

    /* 清空超时标记 */
    g_key_mgmt_connect_native_timeout = 0;

    timer = xTimerCreate("key_mgmt_ble_connect_timer", timeout, pdFALSE, 0, key_mgmt_connect_native_timer);

    key_mgmt_ble_adv_start();

    do {
        vTaskDelay(1);
    } while (!key_mgmt_ble_is_connected() && g_key_mgmt_connect_native_timeout == 0);

    xTimerDelete(timer, 0);

    key_mgmt_ble_adv_stop();

    /* 不再发红外 */

    /* 任务结束 */
    vTaskDelete(NULL);
}

/*
 * 启动对码流程
 * 国内模式
 */
int key_mgmt_connect_native_start()
{
    g_key_mgmt_connect_native_binary = xSemaphoreCreateBinary();

    xTaskCreate((TaskFunction_t)key_mgmt_connect_native_task, (const char*)"key_mgmt_connect_native_task",
                (uint16_t)KEY_MGMT_TASK_STACK_DEPTH, (void*)NULL, (UBaseType_t)KEY_MGMT_TASK_PRI,
                (TaskHandle_t*)&g_key_mgmt_connect_native_task_handler);
}

/*
 * 结束对码流程
 * 国内模式
 */
int key_mgmt_connect_native_end()
{
    if (g_key_mgmt_connect_native_binary != NULL)
        xSemaphoreGive(g_key_mgmt_connect_native_binary);
}



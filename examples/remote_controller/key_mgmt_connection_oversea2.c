/**
 * Copyright 2021 by MooreSilicon.All rights reserved
 * @file  key_mgmt_connection_native.c
 * @brief
 * @author weiquan.ou
 * @date 2022-4-27
 * @version 1.0
 * @Revision
 */

static SemaphoreHandle_t g_key_mgmt_connect_oversea2_binary;
static TaskHandle_t g_key_mgmt_connect_oversea2_task_handler;
static unsigned long g_key_mgmt_connect_oversea2_timeout = 0;
static unsigned long g_key_mgmt_connect_ir_stop = 0;

static void key_mgmt_connect_oversea2_timer(TimerHandle_t timer)
{
    /* ���ó�ʱ��� */
    g_key_mgmt_connect_oversea2_timeout = 1;
}

static void key_mgmt_connect_oversea2_task(void *pst)
{
    TimerHandle_t timer;
    unsigned long timeout;
    int err;

    /* ��������1s��ִ�����Ӵ��� */
    if (xSemaphoreTake(g_key_mgmt_connect_oversea2_binary, 1000) == pdTRUE) {

        /* 1s���յ��ź������������ɿ��� */
        vSemaphoreDelete(g_key_mgmt_connect_oversea2_binary);
        g_key_mgmt_connect_oversea2_binary = NULL;
        vTaskDelete(NULL);
        return;
    }

    vSemaphoreDelete(g_key_mgmt_connect_oversea2_binary);
    g_key_mgmt_connect_oversea2_binary = NULL;

    /* �������κ����ֵ0x00A0 */

    key_mgmt_ble_dis_connect();
    key_mgmt_ble_dev_del();

    key_mgmt_ble_connect_timeout_get(&timeout);

    /* ��ճ�ʱ��� */
    g_key_mgmt_connect_oversea2_timeout = 0;

    timer = xTimerCreate("key_mgmt_ble_connect_timer", timeout, pdFALSE, 0, key_mgmt_connect_oversea2_timer);

    key_mgmt_ble_adv_start();

    do {
        vTaskDelay(1);
    } while (!key_mgmt_ble_is_connected() && g_key_mgmt_connect_oversea2_timeout == 0);

    xTimerDelete(timer, 0);

    key_mgmt_ble_adv_stop();

    /* ����������ֱ������̧�� */
    do {
        /* �������ֵ0x00AE */
    } while(!g_key_mgmt_connect_ir_stop);

    /* ������� */
    vTaskDelete(NULL);
}

/*
 * ������������
 * ����ģʽ
 */
int key_mgmt_connect_oversea2_start()
{
    g_key_mgmt_connect_oversea2_binary = xSemaphoreCreateBinary();
    g_key_mgmt_connect_ir_stop = 0;

    xTaskCreate((TaskFunction_t)key_mgmt_connect_oversea2_task, (const char*)"key_mgmt_connect_oversea2_task",
                (uint16_t)KEY_MGMT_TASK_STACK_DEPTH, (void*)NULL, (UBaseType_t)KEY_MGMT_TASK_PRI,
                (TaskHandle_t*)&g_key_mgmt_connect_oversea2_task_handler);
}

/*
 * ������������
 * ����ģʽ
 */
int key_mgmt_connect_oversea2_end()
{
    if (g_key_mgmt_connect_oversea2_binary != NULL)
        xSemaphoreGive(g_key_mgmt_connect_oversea2_binary);

    g_key_mgmt_connect_ir_stop = 1;
}

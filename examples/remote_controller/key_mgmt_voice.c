/**
 * Copyright 2021 by MooreSilicon.All rights reserved
 * @file  key_mgmt_voice.c
 * @brief
 * @author weiquan.ou
 * @date 2022-4-26
 * @version 1.0
 * @Revision
 */

static enum key_mgmt_voice_mode g_google_voice_mode = KEY_MGMT_VOICE_ON_REQUEST;
static TaskHandle_t g_key_mgmt_voice_task_handler = NULL;
static unsigned long g_key_mgmt_voice_end = 0;
static unsigned long g_key_mgmt_voice_timeout = 0;

/*
 * 启动BLE传输音频
 */
static int key_mgmt_voice_ble_capture_start()
{
    return 0;
}

/*
 * 停止BLE传输音频
 */
static int key_mgmt_voice_ble_capture_end()
{
    return 0;
}

static int key_mgmt_voice_ble_timeout_get(unsigned long *timeout)
{
    return 0;
}

/*
 * 等待BLE MIC_OPEN消息
 */
static int key_mgmt_voice_ble_mic_open()
{
    return 0;
}

/*
 * 等待BLE MIC_CLOSED消息
 */
static int key_mgmt_voice_ble_mic_closed()
{
    return 0;
}

static void key_mgmt_voice_timer(TimerHandle_t timer)
{
    /* 设置超时标记 */
    g_key_mgmt_voice_timeout = 1;
}

int key_mgmt_voice_on_request()
{
    TimerHandle_t timer;
    unsigned long *timeout;

    key_mgmt_voice_ble_capture_start();

    /* 等待MIC_OPEN消息 */
    key_mgmt_voice_ble_mic_open();

    key_mgmt_voice_ble_timeout_get(&timeout);
    g_key_mgmt_voice_timeout = 0;
    timer = xTimerCreate("key_mgmt_ble_connect_timer", timeout, pdFALSE, 0, key_mgmt_voice_timer);

    /* 启动ADC驱动数据采集 */

    do {
        /* 读取ADC驱动数据 */
        /* BLE发送语音数据 */
    } while (!key_mgmt_voice_ble_mic_closed() && !g_key_mgmt_voice_end && !g_key_mgmt_voice_timeout);

    /* 停止ADC驱动数据采集 */

    key_mgmt_voice_ble_capture_end();

    vTaskDelete(NULL);
}

int key_mgmt_voice_ptt()
{
    TimerHandle_t timer;
    unsigned long *timeout;

    key_mgmt_voice_ble_capture_start();

    key_mgmt_voice_ble_timeout_get(&timeout);
    g_key_mgmt_voice_timeout = 0;
    timer = xTimerCreate("key_mgmt_ble_connect_timer", timeout, pdFALSE, 0, key_mgmt_voice_timer);

    /* 启动ADC驱动数据采集 */

    do {
        /* 读取ADC驱动数据 */
        /* BLE发送语音数据 */
    } while (!key_mgmt_voice_ble_mic_closed() && !g_key_mgmt_voice_end && !g_key_mgmt_voice_timeout);

    /* 停止ADC驱动数据采集 */

    key_mgmt_voice_ble_capture_end();

    vTaskDelete(NULL);
}

int key_mgmt_voice_htt()
{
    TimerHandle_t timer;
    unsigned long *timeout;

    key_mgmt_voice_ble_capture_start();

    key_mgmt_voice_ble_timeout_get(&timeout);
    g_key_mgmt_voice_timeout = 0;
    timer = xTimerCreate("key_mgmt_ble_connect_timer", timeout, pdFALSE, 0, key_mgmt_voice_timer);

    /* 启动ADC驱动数据采集 */

    do {
        /* 读取ADC驱动数据 */
        /* BLE发送语音数据 */
    } while (!g_key_mgmt_voice_end && !g_key_mgmt_voice_timeout);

    /* 停止ADC驱动数据采集 */

    key_mgmt_voice_ble_capture_end();

    vTaskDelete(NULL);
}

int key_mgmt_voice_start()
{
    TaskFunction_t task;
    int err;

    g_key_mgmt_voice_end = 0;

    switch (g_google_voice_mode) {
        case KEY_MGMT_VOICE_ON_REQUEST:
            task = key_mgmt_voice_on_request;
            break;

        case KEY_MGMT_VOICE_PTT:
            task = key_mgmt_voice_ptt;
            break;

        case KEY_MGMT_VOICE_HTT:
            task = key_mgmt_voice_htt;
            break;

        default:
            task = key_mgmt_voice_on_request;
            break;
    }

    xTaskCreate((TaskFunction_t)task, (const char*)"key_mgmt_voice_start",
                (uint16_t)KEY_MGMT_TASK_STACK_DEPTH, (void*)NULL, (UBaseType_t)KEY_MGMT_TASK_PRI,
                (TaskHandle_t*)&g_key_mgmt_voice_task_handler);

    return err;
}

int key_mgmt_voice_end()
{
    g_key_mgmt_voice_end = 1;
    return 0;
}

int key_mgmt_voice_init(struct key_mgmt_voice_cap *cap)
{
    /* 配置BLE   google voice 能力 */

    g_google_voice_mode = cap->assistant_interaction_model;

    return 0;
}

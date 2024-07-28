/**
 * Copyright 2021 by MooreSilicon.All rights reserved
 * @file  key_mgmt.c
 * @brief
 * @author weiquan.ou
 * @date 2022-4-26
 * @version 1.0
 * @Revision
 */

#include "FreeRTOS.h"
#include "task.h"

#include "key_mgmt_state.h"

#define KEY_MGMT_TASK_STACK_DEPTH 256
#define KEY_MGMT_TASK_PRI 2

#define KEY_MGMT_BLE_DRIC_CONNECT_TRY_TIMES 3

static TaskHandle_t g_key_mgmt_task_handler;

/*
 * 单键按下或抬起的处理
 * struct key_value *event 当前事件的按键
 */
int key_mgmt_one_key_proc(struct key_value *event)
{
    int err;
    int i;

    switch (event->state) {

        case KEY_RELEASE:

            if (key_mgmt_ble_is_connected()) {

                if (key_mgmt_is_voice_key(event->key)) {

                    /* 发送语音HID键值抬起 */

                    /* 结束语音发送 */
                    key_mgmt_voice_end();
                } else {
                    /* 发送HID键值抬起 */
                }

            } else {

                /* 结束发送红外IR键值 */

            }

            break;

        case KEY_PRESS:

            if (key_mgmt_ble_is_connected()) {

                if (key_mgmt_is_voice_key(event->key)) {
                    /* 发送语音HID键值 */

                    /* 发语音处理 */
                    key_mgmt_voice_start();

                } else {
                    /* 发送HID键值 */
                }

            } else {

                if (key_mgmt_ble_has_dev()) {

                    if (key_mgmt_is_voice_key(event->key)) {

                        /* 尝试回连 */
                        for (i = 0; i < KEY_MGMT_BLE_DRIC_CONNECT_TRY_TIMES; i++) {
                            err = key_mgmt_ble_dirc_adv_sync();
                            if (!err) {
                                break;
                            }
                        }

                        if (err) {
                            /* 发送配对异常IR信号0xA3 3次 */
                        } else {
                            /* 发语音处理 */
                            key_mgmt_voice_start();
                        }

                    } else {

                        key_mgmt_ble_dirc_adv_start();

                        /* 持续发送IR键值 */
                    }

                } else {

                    if (key_mgmt_is_voice_key(event->key)) {
                        /* 发送配对异常IR信号0xA3 3次 */
                    } else {
                        /* 持续发送IR键值 */
                    }
                }
            }
            break;

        default:
            break;
    }

    return 0;
}

/*
 * 双键按下或抬起的处理
 * struct key_value *event 当前事件的按键
 * unsigned long key 历史仍在按下的键
 */
int key_mgmt_two_key_proc(struct key_value *event, unsigned long key)
{
    struct key_value one_key;

    switch (event->state) {

        case KEY_RELEASE:

            if (key_mgmt_is_conntion_key(event->key, key)) {
                key_mgmt_connect_end();
            } else {
                /* 一个键按下的处理 */
                one_key.key = key;
                one_key.state = KEY_PRESS;
                key_mgmt_one_key_proc(&one_key);
            }

            break;

        case KEY_PRESS:

            if (!key_mgmt_is_conntion_key(event->key, key))
                break;

            key_mgmt_connect_start();

            break;

        default:
            break;
    }

    return 0;
}

/*
 * 三键按下或抬起的处理
 * struct key_value *event 当前事件的按键
 * unsigned long key1 历史仍在按下的键1
 * unsigned long key2 历史仍在按下的键2
 */
int key_mgmt_three_key_proc(struct key_value *event, unsigned long key1, unsigned long key2)
{
    struct key_value one_key;

    switch (event->state) {

        case KEY_RELEASE:

            if (key_mgmt_is_factory_key(event->key, key1, key2)) {
                /* 结束产测模式 */
            } else {
                /* 双键按下的处理 */
                one_key.key = key1;
                one_key.state = KEY_PRESS;
                key_mgmt_two_key_proc(&one_key, key2);
            }

            break;
        case KEY_PRESS:

            if (key_mgmt_is_conntion_key(key1, key2)) {
                key_mgmt_connect_end();
            }

            if (!key_mgmt_is_factory_key(event->key, key1, key2))
                break;

            /* 进入产测处理 */
            break;

        default:
            break;
    }

    return 0;
}

void key_mgmt_main(void *param)
{
    struct key_value event;
    struct key_mgmt_voice_cap cap;

    /* 读配置管理表项 */

    /* 配置BLE设备信息 */

    /* 配置Google Voice 语音模式和采样模式 */
    key_mgmt_voice_init(&cap);

    /* 配置keyscan  驱动键值表 */

    /* 配置遥控器键值表和HID      report map */

    while (1) {

        /* 读取键值 */

        /* 确认是有效键值 */

        /* 键值输入状态机处理 */
        key_mgmt_state_proc(&event);
    }
}

int main(void)
{
    xTaskCreate((TaskFunction_t)key_mgmt_main, (const char*)"key_mgmt",
                (uint16_t)KEY_MGMT_TASK_STACK_DEPTH, (void*)NULL, (UBaseType_t)KEY_MGMT_TASK_PRI,
                (TaskHandle_t*)&g_key_mgmt_task_handler);

    vTaskStartScheduler();

    while(1);
    return 0;
}


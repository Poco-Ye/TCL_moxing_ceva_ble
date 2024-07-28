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
 * �������»�̧��Ĵ���
 * struct key_value *event ��ǰ�¼��İ���
 */
int key_mgmt_one_key_proc(struct key_value *event)
{
    int err;
    int i;

    switch (event->state) {

        case KEY_RELEASE:

            if (key_mgmt_ble_is_connected()) {

                if (key_mgmt_is_voice_key(event->key)) {

                    /* ��������HID��ֵ̧�� */

                    /* ������������ */
                    key_mgmt_voice_end();
                } else {
                    /* ����HID��ֵ̧�� */
                }

            } else {

                /* �������ͺ���IR��ֵ */

            }

            break;

        case KEY_PRESS:

            if (key_mgmt_ble_is_connected()) {

                if (key_mgmt_is_voice_key(event->key)) {
                    /* ��������HID��ֵ */

                    /* ���������� */
                    key_mgmt_voice_start();

                } else {
                    /* ����HID��ֵ */
                }

            } else {

                if (key_mgmt_ble_has_dev()) {

                    if (key_mgmt_is_voice_key(event->key)) {

                        /* ���Ի��� */
                        for (i = 0; i < KEY_MGMT_BLE_DRIC_CONNECT_TRY_TIMES; i++) {
                            err = key_mgmt_ble_dirc_adv_sync();
                            if (!err) {
                                break;
                            }
                        }

                        if (err) {
                            /* ��������쳣IR�ź�0xA3 3�� */
                        } else {
                            /* ���������� */
                            key_mgmt_voice_start();
                        }

                    } else {

                        key_mgmt_ble_dirc_adv_start();

                        /* ��������IR��ֵ */
                    }

                } else {

                    if (key_mgmt_is_voice_key(event->key)) {
                        /* ��������쳣IR�ź�0xA3 3�� */
                    } else {
                        /* ��������IR��ֵ */
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
 * ˫�����»�̧��Ĵ���
 * struct key_value *event ��ǰ�¼��İ���
 * unsigned long key ��ʷ���ڰ��µļ�
 */
int key_mgmt_two_key_proc(struct key_value *event, unsigned long key)
{
    struct key_value one_key;

    switch (event->state) {

        case KEY_RELEASE:

            if (key_mgmt_is_conntion_key(event->key, key)) {
                key_mgmt_connect_end();
            } else {
                /* һ�������µĴ��� */
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
 * �������»�̧��Ĵ���
 * struct key_value *event ��ǰ�¼��İ���
 * unsigned long key1 ��ʷ���ڰ��µļ�1
 * unsigned long key2 ��ʷ���ڰ��µļ�2
 */
int key_mgmt_three_key_proc(struct key_value *event, unsigned long key1, unsigned long key2)
{
    struct key_value one_key;

    switch (event->state) {

        case KEY_RELEASE:

            if (key_mgmt_is_factory_key(event->key, key1, key2)) {
                /* ��������ģʽ */
            } else {
                /* ˫�����µĴ��� */
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

            /* ������⴦�� */
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

    /* �����ù������ */

    /* ����BLE�豸��Ϣ */

    /* ����Google Voice ����ģʽ�Ͳ���ģʽ */
    key_mgmt_voice_init(&cap);

    /* ����keyscan  ������ֵ�� */

    /* ����ң������ֵ���HID      report map */

    while (1) {

        /* ��ȡ��ֵ */

        /* ȷ������Ч��ֵ */

        /* ��ֵ����״̬������ */
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


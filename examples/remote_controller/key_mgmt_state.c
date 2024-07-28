/**
 * Copyright 2021 by MooreSilicon.All rights reserved
 * @file  key_mgmt_state.c
 * @brief
 * @author weiquan.ou
 * @date 2022-4-26
 * @version 1.0
 * @Revision
 */

typedef int (* key_mgmt_state_func)(struct key_value *);

enum key_mgmt_state {

    /* 无键按下状态 */
    KEY_MGMT_IDLE = 0,

    /* 一键按下状态 */
    KEY_MGMT_ONE_KEY,

    /* 两键按下状态 */
    KEY_MGMT_TWO_KEY,

    /* 三键按下状态 */
    KEY_MGMT_THREE_KEY,

    /* 状态总数 */
    KEY_MGMT_STATE_MAX
};

static int key_mgmt_state_idle_proc(struct key_value *event);
static int key_mgmt_state_one_key_proc(struct key_value *event);
static int key_mgmt_state_two_key_proc(struct key_value *event);
static int key_mgmt_state_three_key_proc(struct key_value *event);

static key_mgmt_state_func g_key_mgmt_state_proc[KEY_MGMT_STATE_MAX] = {
    key_mgmt_state_idle_proc,
    key_mgmt_state_one_key_proc,
    key_mgmt_state_two_key_proc,
    key_mgmt_state_three_key_proc
};

static enum key_mgmt_state g_key_mgmt_state = KEY_MGMT_IDLE;

static unsigned long g_key[KEY_MGMT_MAX_PRESS_NUM] = {0, 0, 0};

int key_mgmt_state_proc(struct key_value *event)
{
    return g_key_mgmt_state_proc[g_key_mgmt_state](event);
}

static int key_mgmt_state_key_match(unsigned long key)
{
    int i;

    for (i = 0; i < KEY_MGMT_MAX_PRESS_NUM; i++) {
        if (g_key[i] == key) {
            g_key[i] = 0;
            return i;
        }
    }

    return -1;
}

static int key_mgmt_state_key_save(unsigned long key)
{
    int i;

    for (i = 0; i < KEY_MGMT_MAX_PRESS_NUM; i++) {
        if (g_key[i] == 0) {
            g_key[i] = key;
            return i;
        }
    }

    return -1;
}

static int key_mgmt_state_find_two_key(unsigned long *key1, unsigned long *key2)
{
    int i, j;
    int start = 0;
    int err = -1;
    unsigned long *key_output[KEY_MGMT_CONN_KEY_NUM];

    key_output[0] = key1;
    key_output[1] = key2;

    for (j = 0; j < KEY_MGMT_CONN_KEY_NUM; j++) {

        for (i = start; i < KEY_MGMT_MAX_PRESS_NUM; i++) {
            if (g_key[i]) {
                *key_output[j] = g_key[i];
                err = 0;
                break;
            }
        }

        start = i + 1;
    }

    return err;
}

static int key_mgmt_state_find_one_key(unsigned long *key1)
{
    int i;
    int err = -1;

    for (i = 0; i < KEY_MGMT_MAX_PRESS_NUM; i++) {
        if (g_key[i]) {
            *key1 = g_key[i];
            err = 0;
            break;
        }
    }

    return err;
}


static int key_mgmt_state_idle_proc(struct key_value *event)
{
    switch (event->state) {

        case KEY_RELEASE:
            break;

        case KEY_PRESS:
            key_mgmt_state_key_save(event->key);

            /* 单键按键逻辑处理 */
            key_mgmt_one_key_proc(event);

            g_key_mgmt_state = KEY_MGMT_ONE_KEY;
            break;

        default:
            break;
    }

    return 0;
}

static int key_mgmt_state_one_key_proc(struct key_value *event)
{
    unsigned long key;

    switch (event->state) {

        case KEY_RELEASE:
            if (key_mgmt_state_key_match(event->key) < 0)
                break;

            /* 抬键逻辑处理 */
            key_mgmt_one_key_proc(event);

            g_key_mgmt_state = KEY_MGMT_IDLE;
            break;

        case KEY_PRESS:
            key_mgmt_state_find_one_key(&key);

            /* 双键按键处理 */
            key_mgmt_two_key_proc(event, key);

            key_mgmt_state_key_save(event->key);

            g_key_mgmt_state = KEY_MGMT_TWO_KEY;
            break;

        default:
            break;
    }

    return 0;
}

static int key_mgmt_state_two_key_proc(struct key_value *event)
{
    unsigned long key1, key2;

    switch (event->state) {

        case KEY_RELEASE:
            if (key_mgmt_state_key_match(event->key) < 0)
                break;

            key_mgmt_state_find_one_key(&key1);

            /* 有一个键抬起处理 */
            key_mgmt_two_key_proc(event, key1);

            g_key_mgmt_state = KEY_MGMT_ONE_KEY;
            break;

        case KEY_PRESS:
            key_mgmt_state_find_two_key(&key1, &key2);

            /* 三键按键处理 */
            key_mgmt_three_key_proc(event, key1, key2);

            key_mgmt_state_key_save(event->key);

            g_key_mgmt_state = KEY_MGMT_THREE_KEY;
            break;

        default:
            break;
    }

    return 0;
}

static int key_mgmt_state_three_key_proc(struct key_value *event)
{
    unsigned long key1, key2;

    switch (event->state) {

        case KEY_RELEASE:
            if (key_mgmt_state_key_match(event->key) < 0)
                break;

            key_mgmt_state_find_two_key(&key1, &key2);

            /* 有一个键抬起处理 */
            key_mgmt_three_key_proc(event, key1, key2);

            g_key_mgmt_state = KEY_MGMT_TWO_KEY;
            break;

        case KEY_PRESS:

            /* 不处理再按键的 */
            break;

        default:
            break;
    }

    return 0;
}




/**
 * Copyright 2021 by MooreSilicon.All rights reserved
 * @file  key_mgmt_table.c
 * @brief
 * @author weiquan.ou
 * @date 2022-4-27
 * @version 1.0
 * @Revision
 */

int key_mgmt_is_voice_key(unsigned long key)
{
    if (/* 是语音键 */)
        return 0;
    else
        return -1;
}

/*
 * 是连接组合键
 */
int key_mgmt_is_conntion_key(unsigned long key1, unsigned long key2)
{
#ifdef MS_KEY_MGMT_CONNECTION_NATIVE
    if (/* ok + 返回 */)
#elif MS_KEY_MGMT_CONNECTION_OVERSEA1
    if (/* ok + 主页 */)
#elif MS_KEY_MGMT_CONNECTION_OVERSEA2
    if (/* ok + 返回 */)
#endif
        return 0;
    else
        return -1;
}

int key_mgmt_is_factory_key(unsigned long key1, unsigned long key2, unsigned long key3)
{
    if (/* 是工厂组合键 */)
        return 0;
    else
        return -1;
}

int key_mgmt_tab_init()
{
    /* 初始化按键驱动键值表 */

    /* 初始化遥控器键值表 */

    /* 初始化HID表 */

    return 0;
}



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
    if (/* �������� */)
        return 0;
    else
        return -1;
}

/*
 * ��������ϼ�
 */
int key_mgmt_is_conntion_key(unsigned long key1, unsigned long key2)
{
#ifdef MS_KEY_MGMT_CONNECTION_NATIVE
    if (/* ok + ���� */)
#elif MS_KEY_MGMT_CONNECTION_OVERSEA1
    if (/* ok + ��ҳ */)
#elif MS_KEY_MGMT_CONNECTION_OVERSEA2
    if (/* ok + ���� */)
#endif
        return 0;
    else
        return -1;
}

int key_mgmt_is_factory_key(unsigned long key1, unsigned long key2, unsigned long key3)
{
    if (/* �ǹ�����ϼ� */)
        return 0;
    else
        return -1;
}

int key_mgmt_tab_init()
{
    /* ��ʼ������������ֵ�� */

    /* ��ʼ��ң������ֵ�� */

    /* ��ʼ��HID�� */

    return 0;
}



/**
 * Copyright 2021 by MooreSilicon.All rights reserved
 * @file  key_mgmt_connection.c
 * @brief
 * @author weiquan.ou
 * @date 2022-4-27
 * @version 1.0
 * @Revision
 */

/*
 * ָʾBLE�������
 */
int key_mgmt_ble_dirc_adv_start()
{
    /* BLE���Ͷ���㲥��Ϣ */
    return 0;
}

/*
 * ָʾBLE�������, �ȴ����ָʾ
 */
int key_mgmt_ble_dirc_adv_sync()
{
    /* ������� */
    key_mgmt_ble_dirc_adv_start();

    /* �ȴ�������� */

    if (/* �ɹ� */) {
        return 0;
    } else {
        return -1;
    }
}

/*
 * BLE�Ƿ�����
 */
int key_mgmt_ble_is_connected()
{
    if (/* ���� */) {
        return 0;
    } else {
        return -1;
    }
}

/*
 * BLE�Ͽ�����
 */
int key_mgmt_ble_dis_connect()
{
    return 0;
}

/*
 * BLE�Ƿ��а��豸
 */
int key_mgmt_ble_has_dev()
{
    if (/* has device */) {
        return 0;
    } else {
        return -1;
    }
}

/*
 * BLEɾ�����豸
 */
int key_mgmt_ble_dev_del()
{
    return 0;
}

/*
 * BLE �Ƕ���㲥
 */
int key_mgmt_ble_adv_start()
{

}

/*
 * ֹͣBLE �Ƕ���㲥
 */
int key_mgmt_ble_adv_stop()
{

}


int key_mgmt_ble_connect_timeout_get(unsigned long *timeout)
{
    return 0;
}


/*
 * ������������
 * ����ѡ�����/����ģʽ
 */
int key_mgmt_connect_start()
{
#ifdef MS_KEY_MGMT_CONNECTION_NATIVE
    return key_mgmt_connect_native_start();
#elif MS_KEY_MGMT_CONNECTION_OVERSEA1
    return key_mgmt_connect_oversea1_start();
#elif MS_KEY_MGMT_CONNECTION_OVERSEA2
    return key_mgmt_connect_oversea2_start();
#endif
}

/*
 * ������������
 * ����ѡ�����/����ģʽ
 */
int key_mgmt_connect_end()
{
#ifdef MS_KEY_MGMT_CONNECTION_NATIVE
    return key_mgmt_connect_native_end();
#elif MS_KEY_MGMT_CONNECTION_OVERSEA1
    return key_mgmt_connect_oversea1_end();
#elif MS_KEY_MGMT_CONNECTION_OVERSEA2
    return key_mgmt_connect_oversea2_end();
#endif
}



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
 * 指示BLE定向回连
 */
int key_mgmt_ble_dirc_adv_start()
{
    /* BLE发送定向广播消息 */
    return 0;
}

/*
 * 指示BLE定向回连, 等待结果指示
 */
int key_mgmt_ble_dirc_adv_sync()
{
    /* 定向回连 */
    key_mgmt_ble_dirc_adv_start();

    /* 等待回连结果 */

    if (/* 成功 */) {
        return 0;
    } else {
        return -1;
    }
}

/*
 * BLE是否连接
 */
int key_mgmt_ble_is_connected()
{
    if (/* 连接 */) {
        return 0;
    } else {
        return -1;
    }
}

/*
 * BLE断开连接
 */
int key_mgmt_ble_dis_connect()
{
    return 0;
}

/*
 * BLE是否有绑定设备
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
 * BLE删除绑定设备
 */
int key_mgmt_ble_dev_del()
{
    return 0;
}

/*
 * BLE 非定向广播
 */
int key_mgmt_ble_adv_start()
{

}

/*
 * 停止BLE 非定向广播
 */
int key_mgmt_ble_adv_stop()
{

}


int key_mgmt_ble_connect_timeout_get(unsigned long *timeout)
{
    return 0;
}


/*
 * 启动对码流程
 * 编译选择国内/国外模式
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
 * 结束对码流程
 * 编译选择国内/国外模式
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



/**
 * Copyright 2021 by MooreSilicon.All rights reserved
 * @file  key_mgmt_state.h
 * @brief
 * @author weiquan.ou
 * @date 2022-4-26
 * @version 1.0
 * @Revision
 */

#define KEY_MGMT_MAX_PRESS_NUM 3
#define KEY_MGMT_CONN_KEY_NUM  2

enum key_state {
	KEY_RELEASE = 0,
	KEY_PRESS = 1
};

struct key_value {
	enum key_state state;
	unsigned long key;
};

int key_mgmt_state_proc(struct key_value *event);


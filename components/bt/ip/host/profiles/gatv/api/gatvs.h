/**
 ****************************************************************************************
 *
 * @file gatvs.h
 *
 * @brief Header file - Gatv Service Server Role - Native API.
 *
 * Copyright (C) MooreSilicon 2021-2031
 *
 ****************************************************************************************
 */


#ifndef _HIDS_H_
#define _HIDS_H_

enum gatvs_features
{
	/// hid Level Characteristic doesn't support notifications
	GATVS_REPORT_NTF_NOT_SUP	= 0,
	/// hid Level Characteristic support notifications
	GATVS_REPORT_LVL_NTF_SUP = 1,
};

typedef uint16_t (*cb_gatvs_att_read_get)(uint16_t att_idx, co_buf_t** pp_buf);
typedef uint16_t (*cb_gatvs_att_set)(uint8_t conidx,uint16_t att_idx, co_buf_t* p_buf);

struct gatvs_db_cfg
{
	uint8_t att_table_size;
	const gatt_att_desc_t* att_table;
	cb_gatvs_att_read_get cb_get;
	cb_gatvs_att_set cb_set;
};

uint16_t gatvs_send_report(uint8_t conidx,uint16_t att_idx, co_buf_t* p_buf);


#endif /* _HIDS_H_ */

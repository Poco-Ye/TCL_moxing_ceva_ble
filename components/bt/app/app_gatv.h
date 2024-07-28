/**
 ****************************************************************************************
 *
 * @file app_hid.h
 *
 * @brief  Application GATV Module entry point
 *
 * Copyright (C) MX 2022-04
 *
 *
 ****************************************************************************************
 */

#ifndef APP_GATV_H_
#define APP_GATV_H_

/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup RICOW
 *
 * @brief HID Application Module entry point
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (BLE_APP_GATV)

#include <stdint.h>          // Standard Integer Definition
#include "ke_task.h"         // Kernel Task Definition

 /// Table of message handlers
extern const struct app_subtask_handlers app_gatv_handlers;

enum gatvs_atv2remote_msg{
    ATV_GET_CAPS = 0x0A,
    ATV_MIC_OPEN = 0x0C,
    ATV_MIC_CLOSE = 0x0D,
    ATV_MIC_EXTEND = 0x0E,
};

enum gatvs_remote2atv_msg{
    REMOTE_AUDIO_STOP = 0x00,
    REMOTE_AUDIO_START = 0x04,
    REMOTE_START_SEARCH = 0x08,
    REMOTE_AUDIO_SYNC = 0x0A,
    REMOTE_GET_CAPS_RESP = 0x0B,
    REMOTE_MIC_OPEN_ERROR = 0x0C,
    //REMOTE_SEND_VOICE = 0xFE,
};

enum gatvs_audio_start_reason{
    START_BY_MIC_OPEN = 0x00,
    START_BY_PTT_ASSISTANT = 0x01,
    START_BY_HTT_ASSISTANT = 0x02,

};

enum gatvs_audio_stop_reason{
    STOP_BY_MIC_CLOSE = 0x00,
    STOP_BY_HTT_ASSISTANT = 0x02,
    STOP_BY_AUDIO_START = 0x04,
    STOP_BY_TIME_OUT = 0x08,
	 STOP_BY_DISABLE_NOTIFY = 0x10,
	 STOP_BY_OTHER_REASON = 0x80,
};


typedef enum gatvs_audio_start_state_t{
    AUDIO_NONE = 0,
    AUDIO_MIC_PRESS = 1,
    AUDIO_MIC_RELEASE = 2,
    AUDIO_SEARCH = 3,
}audio_state;

typedef struct __gatv_audio_packet_header
{
    uint16_t seq_num;
    uint8_t  rc_id;
    uint16_t prev_pred;
    uint8_t index;
}gatv_aph;


/**
 ****************************************************************************************
 * @brief Initialize HID Application Module
 ****************************************************************************************
 */
void app_gatv_init(void);

/**
 ****************************************************************************************
 * @brief Add a HID Service instance in the DB
 ****************************************************************************************
 */
void app_gatv_add_gatvs(void);

/**
 ****************************************************************************************
 * @brief Enable the HID Over GATT Profile device role
 *
 * @param[in]:  conhdl - Connection handle for the connection
 ****************************************************************************************
 */
void app_gatv_enable_prf(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Send a mouse report to the peer device
 *
 * @param[in]:  report - Mouse report sent by the PS2 driver
 ****************************************************************************************
 */
void app_hid_send_report(int32_t key);

void app_hid_send_data_report(const void* p_data, int32_t cb_data);

void gatvs_rc_start_search_handler(void);

void gatvs_send_audio_start(uint8_t reason);

void gatvs_send_data_via_rx_ntf(uint16_t data_len, uint8_t *p_data);

#endif //(BLE_APP_HID)

/// @} APP

#endif // APP_HID_H_

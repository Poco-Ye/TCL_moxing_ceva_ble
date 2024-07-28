/**
 ****************************************************************************************
 *
 * @file gapm_msg.c
 *
 * @brief Handler of message API
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#if (HOST_MSG_API)
#include "arch.h"
#include "rwip.h"
#include "gapm.h"
#include "gapm_int.h"
#if(BLE_HOST_PRESENT)
#include "gapm_le_msg.h"
#endif // (BLE_HOST_PRESENT)
#if(BT_HOST_PRESENT)
#include "gapm_bt_msg.h"
#if (HOST_TEST_MODE)
#include "gapm_bt_test.h"
#endif // (HOST_TEST_MODE)
#endif // (BT_HOST_PRESENT)

#if (GAPC_PRESENT)
#include "gapc_msg.h"
#endif // (GAPC_PRESENT)
#include "ke_task.h"
#include "ke_msg.h"
#include "ke_mem.h"
#include "ke.h"
#include "string.h"
#include <string.h>

#if (HOST_PROFILES)
#include "prf.h"
#include "../../inc/prf_hl_api.h"
#endif // (HOST_PROFILES)

#if (AHI_TL_SUPPORT)
#include "ahi.h"
#endif // (AHI_TL_SUPPORT)

/*
 * MACROS
 ****************************************************************************************
 */
#define DEST_BF(operation, dest_id)  ((((uint32_t) operation) << 16) | ((uint32_t) dest_id))
#define OPERATION_GET(dest_bf)       ((uint8_t) (((dest_bf) >> 16) & 0xFF))
#define DEST_ID_GET(dest_bf)         ((uint16_t) ((dest_bf) & 0xFFFF))


/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Message environment
typedef struct gapm_msg_env_
{
    /// Pointer to device name
    uint8_t* p_name;
    /// Pointer to new device name
    uint8_t* p_new_name;
    /// Temporary parameters used during a command execution
    void* p_temp_param;
} gapm_msg_env_t;


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
#if (GAPC_PRESENT)
const gapm_callbacks_t* gapc_msg_get_callback_itf(void);
void gapc_msg_set_default_callbacks(void);
#endif // (GAPC_PRESENT)

/// Environment
__STATIC gapm_msg_env_t gapm_msg_env;

#if (BLE_HOST_PRESENT)
__STATIC void gapm_msg_adv_proc_cmp(uint32_t dest_id, uint8_t proc_id, uint8_t actv_idx, uint16_t status);
__STATIC void gapm_msg_adv_actv_stopped(uint32_t dest_id, uint8_t actv_idx, uint16_t reason);
__STATIC void gapm_msg_adv_created(uint32_t dest_id, uint8_t actv_idx, int8_t tx_pwr);
__STATIC void gapm_msg_adv_scan_req_received(uint32_t dest_id, uint8_t actv_idx, const gap_bdaddr_t* p_addr);

__STATIC void gapm_msg_scan_proc_cmp(uint32_t dest_id, uint8_t proc_id, uint8_t actv_idx, uint16_t status);
__STATIC void gapm_msg_actv_report_received(uint32_t dest_id, uint8_t actv_idx, const gapm_adv_report_info_t* p_info, co_buf_t* p_report);

__STATIC void gapm_msg_per_sync_proc_cmp(uint32_t dest_id, uint8_t proc_id, uint8_t actv_idx, uint16_t status);
__STATIC void gapm_msg_per_sync_established(uint32_t dest_id, uint8_t actv_idx, const gapm_per_sync_info_t* p_info);
__STATIC void gapm_msg_per_sync_iq_report_received(uint32_t dest_id, uint8_t actv_idx, const gapm_iq_report_info_t* p_info, uint8_t nb_sample, const gap_iq_sample_t* p_samples);
__STATIC void gapm_msg_per_sync_big_info_report_received(uint32_t dest_id, uint8_t actv_idx, const gap_big_info_t* p_report);

__STATIC void gapm_msg_init_proc_cmp(uint32_t dest_id, uint8_t proc_id, uint8_t actv_idx, uint16_t status);
__STATIC void gapm_msg_init_peer_name(uint32_t dest_id, uint8_t actv_idx, const gap_bdaddr_t* p_addr, uint16_t name_len, const uint8_t* p_name);

#if (HOST_TEST_MODE)
__STATIC void gapm_msg_test_tx_proc_cmp(uint32_t dest_id, uint8_t proc_id, uint8_t actv_idx, uint16_t status);
__STATIC void gapm_msg_test_rx_proc_cmp(uint32_t dest_id, uint8_t proc_id, uint8_t actv_idx, uint16_t status);
__STATIC void gapm_msg_test_stopped(uint32_t dest_id, uint8_t actv_idx, uint16_t reason);
__STATIC void gapm_msg_test_rx_nb_packet_received(uint32_t dest_id, uint8_t actv_idx, uint16_t nb_packet);
__STATIC void gapm_msg_test_rx_iq_report_received(uint32_t dest_id, uint8_t actv_idx, const gapm_iq_report_info_t* p_info, uint8_t nb_sample, const gap_iq_sample_t* p_samples);
#endif // (HOST_TEST_MODE)

__STATIC void gapm_msg_actv_addr_updated(uint32_t dest_id, uint8_t actv_idx, const gap_addr_t* p_addr);
#endif // (BLE_HOST_PRESENT)
#if(BT_HOST_PRESENT)
__STATIC void gapm_msg_inquiry_proc_cmp(uint32_t dest_id, uint8_t proc_id, uint8_t actv_idx, uint16_t status);
__STATIC void gapm_msg_inquiry_report_received(uint32_t dest_id, uint8_t actv_idx, const gapm_inquiry_report_t* p_report,
                                           co_buf_t* p_eir_data);
__STATIC void gapm_msg_inquiry_scan_proc_cmp(uint32_t dest_id, uint8_t proc_id, uint8_t actv_idx, uint16_t status);
__STATIC void gapm_msg_page_scan_proc_cmp(uint32_t dest_id, uint8_t proc_id, uint8_t actv_idx, uint16_t status);
__STATIC void gapm_msg_page_proc_cmp(uint32_t dest_id, uint8_t proc_id, uint8_t actv_idx, uint16_t status);
#endif // (BT_HOST_PRESENT)

__STATIC void gapm_msg_actv_stopped(uint32_t dest_id, uint8_t actv_idx, uint16_t reason);

#if (BLE_HOST_PRESENT)
/// Callback structure required to create an advertising activity
__STATIC const gapm_adv_actv_cb_t gapm_msg_adv_actv_cb =
{
    .hdr.actv.stopped      = gapm_msg_adv_actv_stopped,
    .hdr.actv.proc_cmp     = gapm_msg_adv_proc_cmp,
    .hdr.addr_updated      = gapm_msg_actv_addr_updated,
    .created               = gapm_msg_adv_created,
    .scan_req_received     = gapm_msg_adv_scan_req_received,
    .ext_adv_stopped       = gapm_msg_actv_stopped,
};


/// Callback structure required to create a scan activity
__STATIC const gapm_scan_actv_cb_t gapm_msg_scan_actv_cb =
{
	.le_actv.actv.stopped      = gapm_msg_actv_stopped,
    .le_actv.actv.proc_cmp     = gapm_msg_scan_proc_cmp,
    .le_actv.addr_updated      = gapm_msg_actv_addr_updated,
    .report_received           = gapm_msg_actv_report_received,
};


/// Callback structure required to create a periodic sync activity
__STATIC const gapm_per_sync_actv_cb_t gapm_msg_per_sync_actv_cb =
{
	.actv.stopped             = gapm_msg_actv_stopped,
    .actv.proc_cmp            = gapm_msg_per_sync_proc_cmp,
    .report_received          = gapm_msg_actv_report_received,
    .established              = gapm_msg_per_sync_established,
    .iq_report_received       = gapm_msg_per_sync_iq_report_received,
    .big_info_report_received = gapm_msg_per_sync_big_info_report_received,
};

/// Callback structure required to create an initiating activity
__STATIC const gapm_init_actv_cb_t gapm_msg_init_actv_cb =
{
    .hdr.actv.stopped  = gapm_msg_actv_stopped,
    .hdr.actv.proc_cmp = gapm_msg_init_proc_cmp,
    .hdr.addr_updated  = gapm_msg_actv_addr_updated,
    .peer_name         = gapm_msg_init_peer_name,
};

#if (HOST_TEST_MODE)
/// Callback structure required to create a TX test activity
__STATIC const gapm_actv_cb_t gapm_msg_test_tx_actv_cb =
{
    .proc_cmp     = gapm_msg_test_tx_proc_cmp,
    .stopped      = gapm_msg_test_stopped,
};

/// Callback structure required to create a TX test activity
__STATIC const gapm_le_test_rx_actv_cb_t gapm_msg_test_rx_actv_cb =
{
	.actv.stopped       = gapm_msg_test_stopped,
    .actv.proc_cmp      = gapm_msg_test_rx_proc_cmp,
    .nb_packet_received = gapm_msg_test_rx_nb_packet_received,
    .iq_report_received = gapm_msg_test_rx_iq_report_received,
};
#endif // (HOST_TEST_MODE)
#endif // (BLE_HOST_PRESENT)

#if(BT_HOST_PRESENT)

/// Callback structure required to create an inquiry activity
__STATIC const gapm_inquiry_actv_cb_t gapm_msg_inquiry_actv_cb =
{
	.actv.stopped      = gapm_msg_actv_stopped,
    .actv.proc_cmp     = gapm_msg_inquiry_proc_cmp,
    .report_received   = gapm_msg_inquiry_report_received,
};

/// Callback structure required to create an inquiry scan activity
__STATIC const gapm_inquiry_scan_actv_cb_t gapm_msg_inquiry_scan_actv_cb =
{
    .proc_cmp     = gapm_msg_inquiry_scan_proc_cmp,
    .stopped      = gapm_msg_actv_stopped,
};

/// Callback structure required to create an page activity
__STATIC const gapm_page_actv_cb_t gapm_msg_page_actv_cb =
{
    .proc_cmp     = gapm_msg_page_proc_cmp,
    .stopped      = gapm_msg_actv_stopped,
};

/// Callback structure required to create an page scan activity
__STATIC const gapm_page_scan_actv_cb_t gapm_msg_page_scan_actv_cb =
{
    .hdr.stopped          = gapm_msg_actv_stopped,
    .hdr.proc_cmp         = gapm_msg_page_scan_proc_cmp,
    .connection_authorize = NULL,
};
#endif // (BT_HOST_PRESENT)

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */



/**
 ****************************************************************************************
 * @brief Send Command Complete Event of requested operation
 *
 * @param[in] operation Operation code see enum #gapm_operation
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] actv_idx  Activity index
 * @param[in] status    Operation execution status
 ****************************************************************************************
*/
__STATIC void gapm_msg_send_cmp_evt(uint8_t operation, uint16_t dest_id, uint8_t actv_idx, uint16_t status)
{
    // prepare command completed event with error status
    struct gapm_cmp_evt* p_cmp_evt = KE_MSG_ALLOC(GAPM_CMP_EVT, dest_id, TASK_GAPM, gapm_cmp_evt);
    if(p_cmp_evt != NULL)
    {
        p_cmp_evt->operation = operation;
        p_cmp_evt->status    = status;
        p_cmp_evt->actv_idx  = actv_idx;
        // send event
        ke_msg_send(p_cmp_evt);
    }
}

/// Default Procedure complete handler
__STATIC void gapm_msg_default_cmp_evt_handler(uint32_t dest_bf, uint16_t status)
{
    gapm_msg_send_cmp_evt(OPERATION_GET(dest_bf), DEST_ID_GET(dest_bf), GAP_INVALID_ACTV_IDX, status);
}


/// Callback executed when device configuration procedure is completed
__STATIC void gapm_msg_set_dev_config_cmp_handler(uint32_t dest_bf, uint16_t status)
{
    #if(GAPC_PRESENT)
    if(status == GAP_ERR_NO_ERROR)
    {
        // Force default message callbacks
        gapc_msg_set_default_callbacks();
    }
    #endif // (GAPC_PRESENT)

    gapm_msg_default_cmp_evt_handler(dest_bf, status);
}

/// Callback executed when set name procedure is completed
__STATIC void gapm_msg_set_name_cmp_handler(uint32_t dest_bf, uint16_t status)
{
    if(status != GAP_ERR_NO_ERROR)
    {
        ke_free(gapm_msg_env.p_new_name);
    }
    else
    {
        if(gapm_msg_env.p_name != NULL)
        {
            ke_free(gapm_msg_env.p_name);
        }

        gapm_msg_env.p_name     = gapm_msg_env.p_new_name;
        gapm_msg_env.p_new_name = NULL;
    }

    gapm_msg_default_cmp_evt_handler(dest_bf, status);
}

/// Callback executed when get host version procedure is completed
__STATIC void gapm_msg_version_handler(uint32_t dest_bf, uint16_t status, const gapm_version_t* p_version)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapm_dev_version_ind * p_ind = KE_MSG_ALLOC(GAPM_DEV_VERSION_IND, DEST_ID_GET(dest_bf), TASK_GAPM, gapm_dev_version_ind);
        if(p_ind)
        {
            p_ind->hci_subver  = p_version->hci_subver;
            p_ind->lmp_subver  = p_version->lmp_subver;
            p_ind->lmp_ver     = p_version->lmp_ver;
            p_ind->manuf_name  = p_version->manuf_name;
            p_ind->host_ver    = p_version->host_ver;
            p_ind->host_subver = p_version->host_subver;
            ke_msg_send(p_ind);
        }
        else
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }

    gapm_msg_default_cmp_evt_handler(dest_bf, status);
}

#if (BLE_HOST_PRESENT)
/// Callback executed when get Tx Power level procedure is completed
__STATIC void gapm_msg_adv_tx_power_handler(uint32_t dest_bf, uint16_t status, int8_t power_lvl)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapm_dev_adv_tx_power_ind *p_ind = KE_MSG_ALLOC(GAPM_DEV_ADV_TX_POWER_IND, DEST_ID_GET(dest_bf),
                                                               TASK_GAPM, gapm_dev_adv_tx_power_ind);
        if(p_ind)
        {
            p_ind->power_lvl = power_lvl;
            ke_msg_send(p_ind);
        }
        else
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }

    gapm_msg_default_cmp_evt_handler(dest_bf, status);
}

/// Callback executed when get Suggested Default Data Length procedure is completed
__STATIC void gapm_msg_sugg_dflt_data_len_handler(uint32_t dest_bf, uint16_t status, const gapm_sugg_dflt_data_len_t *p_info)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapm_sugg_dflt_data_len_ind *p_ind = KE_MSG_ALLOC(GAPM_SUGG_DFLT_DATA_LEN_IND, DEST_ID_GET(dest_bf),
                                                                 TASK_GAPM, gapm_sugg_dflt_data_len_ind);
        if(p_ind)
        {
            p_ind->suggted_max_tx_octets = p_info->suggted_max_tx_octets;
            p_ind->suggted_max_tx_time   = p_info->suggted_max_tx_time;
            ke_msg_send(p_ind);
        }
        else
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }

    gapm_msg_default_cmp_evt_handler(dest_bf, status);
}

/// Callback executed when Get Maximum LE Data Length procedure is completed
__STATIC void gapm_msg_max_le_data_len_handler(uint32_t dest_bf, uint16_t status, const gapm_max_le_data_len_t *p_info)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        // Send indication to the APP
        struct gapm_max_le_data_len_ind *p_ind = KE_MSG_ALLOC(GAPM_MAX_LE_DATA_LEN_IND, DEST_ID_GET(dest_bf), TASK_GAPM,
                                                              gapm_max_le_data_len_ind);

        if(p_ind)
        {
            p_ind->suppted_max_rx_octets = p_info->suppted_max_rx_octets;
            p_ind->suppted_max_rx_time   = p_info->suppted_max_rx_time;
            p_ind->suppted_max_tx_octets = p_info->suppted_max_tx_octets;
            p_ind->suppted_max_tx_time   = p_info->suppted_max_tx_time;
            ke_msg_send(p_ind);
        }
        else
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }

    gapm_msg_default_cmp_evt_handler(dest_bf, status);
}

/// Function executed when Get List size procedure is completed
__STATIC void gapm_msg_list_size_handler(uint32_t dest_bf, uint16_t status, uint8_t size)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapm_list_size_ind *p_ind = KE_MSG_ALLOC(GAPM_LIST_SIZE_IND, DEST_ID_GET(dest_bf), TASK_GAPM, gapm_list_size_ind);
        if(p_ind)
        {
            p_ind->operation = OPERATION_GET(dest_bf);
            p_ind->size      = size;
            ke_msg_send(p_ind);
        }
        else
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }

    gapm_msg_default_cmp_evt_handler(dest_bf, status);
}

/// Callback executed when Get available number of advertising sets procedure is completed
__STATIC void gapm_msg_nb_adv_set_handler(uint32_t dest_bf, uint16_t status, uint8_t nb_adv_set)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapm_nb_adv_sets_ind *p_ind = KE_MSG_ALLOC(GAPM_NB_ADV_SETS_IND, DEST_ID_GET(dest_bf), TASK_GAPM, gapm_nb_adv_sets_ind);
        if(p_ind)
        {
            p_ind->nb_adv_sets = nb_adv_set;
            ke_msg_send(p_ind);
        }
        else
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }

    gapm_msg_default_cmp_evt_handler(dest_bf, status);
}

/// Callback executed when Get maximum advertising data length supported by controller procedure is completed
__STATIC void gapm_msg_max_adv_len_handler(uint32_t dest_bf, uint16_t status, uint16_t max_adv_len)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapm_max_adv_data_len_ind *p_ind = KE_MSG_ALLOC(GAPM_MAX_ADV_DATA_LEN_IND, DEST_ID_GET(dest_bf),
                                                               TASK_GAPM, gapm_max_adv_data_len_ind);
        if(p_ind)
        {
            p_ind->length = max_adv_len;
            ke_msg_send(p_ind);
        }
        else
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }

    gapm_msg_default_cmp_evt_handler(dest_bf, status);
}

/// Callback executed when Get minimum and maximum transmit power level supported by the controller procedure is completed
__STATIC void gapm_msg_tx_pwr_rng_handler(uint32_t dest_bf, uint16_t status, const gapm_tx_pwr_rng_t* p_rng)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapm_dev_tx_pwr_ind *p_ind = KE_MSG_ALLOC(GAPM_DEV_TX_PWR_IND, DEST_ID_GET(dest_bf), TASK_GAPM, gapm_dev_tx_pwr_ind);
        if(p_ind)
        {
            p_ind->min_tx_pwr = p_rng->min_tx_pwr;
            p_ind->max_tx_pwr = p_rng->max_tx_pwr;
            ke_msg_send(p_ind);
        }
        else
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }

    gapm_msg_default_cmp_evt_handler(dest_bf, status);
}

/// Callback executed when Get RF path compensation values procedure is completed
__STATIC void gapm_msg_rf_path_comp_handler(uint32_t dest_bf, uint16_t status, const gapm_rf_path_comp_t* p_rf_path_comp)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapm_dev_rf_path_comp_ind *p_ind = KE_MSG_ALLOC(GAPM_DEV_RF_PATH_COMP_IND, DEST_ID_GET(dest_bf),
                                                               TASK_GAPM, gapm_dev_rf_path_comp_ind);
        if(p_ind)
        {
            p_ind->tx_path_comp = p_rf_path_comp->tx;
            p_ind->rx_path_comp = p_rf_path_comp->rx;
            ke_msg_send(p_ind);
        }
        else
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }

    gapm_msg_default_cmp_evt_handler(dest_bf, status);
}

/// Callback executed when Get Antenna information procedure is completed
__STATIC void gapm_msg_antenna_inf_handler(uint32_t dest_bf, uint16_t status, const gapm_antenna_inf_t *p_info)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapm_antenna_inf_ind* p_ind = KE_MSG_ALLOC(GAPM_ANTENNA_INF_IND, DEST_ID_GET(dest_bf), TASK_GAPM, gapm_antenna_inf_ind);
        if(p_ind)
        {
            p_ind->supp_switching_sampl_rates    = p_info->supp_switching_sampl_rates;
            p_ind->antennae_num                  = p_info->antennae_num;
            p_ind->max_switching_pattern_len     = p_info->max_switching_pattern_len;
            p_ind->max_cte_len                   = p_info->max_cte_len;
            ke_msg_send(p_ind);
        }
        else
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }

    gapm_msg_default_cmp_evt_handler(dest_bf, status);
}

/// AES result
__STATIC void gapm_smp_use_enc_block_cb(uint16_t status, const gap_aes_cipher_t* p_cipher)
{
    if (status == GAP_ERR_NO_ERROR)
    {
        // Send the generated random number to the requester task.
        struct gapm_use_enc_block_ind *p_ind = KE_MSG_ALLOC(GAPM_USE_ENC_BLOCK_IND, gapm_env.aes_src_id, TASK_GAPM, gapm_use_enc_block_ind);
        if(p_ind)
        {
            memcpy(&p_ind->result[0], p_cipher, GAP_KEY_LEN);
            ke_msg_send(p_ind);
        }
        else
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }

    gapm_msg_send_cmp_evt(GAPM_USE_ENC_BLOCK, gapm_env.aes_src_id, GAP_INVALID_ACTV_IDX, status);
}


/// Random value result
__STATIC void gapm_smp_gen_rand_nb_cb(uint16_t status, const gap_aes_rand_nb_t* p_rand)
{
    if (status == GAP_ERR_NO_ERROR)
    {
        // Send the generated random number to the requester task.
        struct gapm_gen_rand_nb_ind *p_ind = KE_MSG_ALLOC(GAPM_GEN_RAND_NB_IND, gapm_env.aes_src_id, TASK_GAPM, gapm_gen_rand_nb_ind);
        if(p_ind)
        {
            memcpy(&p_ind->randnb, p_rand, GAP_AES_LEN);
            ke_msg_send(p_ind);
        }
        else
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }

    gapm_msg_send_cmp_evt(GAPM_GEN_RAND_NB, gapm_env.aes_src_id, GAP_INVALID_ACTV_IDX, status);
}


/// Callback executed when Address generation is completed
__STATIC void gapm_msg_gen_rand_addr_cmp_handler(uint16_t status, const gap_addr_t* p_addr)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapm_dev_bdaddr_ind *p_ind = KE_MSG_ALLOC(GAPM_DEV_BDADDR_IND, gapm_env.aes_src_id, TASK_GAPM, gapm_dev_bdaddr_ind);
        if(p_ind)
        {
            memcpy(p_ind->addr.addr, p_addr, GAP_BD_ADDR_LEN);
            p_ind->addr.addr_type = ADDR_RAND;
            p_ind->actv_idx       = GAP_INVALID_ACTV_IDX;
            ke_msg_send(p_ind);
        }
    }

    gapm_msg_send_cmp_evt(GAPM_GEN_RAND_ADDR, gapm_env.aes_src_id, GAP_INVALID_ACTV_IDX, status);
}


/// Result of GET Public Key procedure
__STATIC void gapm_msg_pub_key_res_cb(uint32_t dest_bf, uint16_t status, const public_key_t* p_pub_key)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapm_pub_key_ind *p_ind = KE_MSG_ALLOC(GAPM_PUB_KEY_IND, DEST_ID_GET(dest_bf), TASK_GAPM, gapm_pub_key_ind);
        if(p_ind)
        {
            memcpy(&p_ind->pub_key_x, p_pub_key->x,  32);
            memcpy(&p_ind->pub_key_y, p_pub_key->y, 32);
            ke_msg_send(p_ind);
        }
        else
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }

    gapm_msg_default_cmp_evt_handler(dest_bf, status);
}

/// Result of DH-key compute
__STATIC void gapm_msg_dh_key_res_cb(uint32_t dest_bf, uint16_t status, const gap_dh_key_t* p_dh_key)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapm_gen_dh_key_ind *p_ind = KE_MSG_ALLOC(GAPM_GEN_DH_KEY_IND, DEST_ID_GET(dest_bf), TASK_GAPM, gapm_gen_dh_key_ind);
        if(p_ind)
        {
            memcpy(&p_ind->result, p_dh_key,  32);
            ke_msg_send(p_ind);
        }
        else
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }

    gapm_msg_default_cmp_evt_handler(dest_bf, status);
}

#if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)

/// Result of LE OOB Data Generation
__STATIC void gapm_msg_le_oob_res_cb(uint32_t dest_bf, uint16_t status, const gap_oob_t* p_data)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        // Provide OOB data to application
        struct gapm_le_oob_data_ind* p_ind = KE_MSG_ALLOC(GAPM_LE_OOB_DATA_IND, DEST_ID_GET(dest_bf), TASK_GAPM, gapm_le_oob_data_ind);

        if(p_ind != NULL)
        {
            p_ind->oob = *p_data;
            ke_msg_send(p_ind);
        }
        else
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }

    gapm_msg_default_cmp_evt_handler(dest_bf, status);
}
#endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
#endif // (BLE_HOST_PRESENT)

#if (BT_HOST_PRESENT)

/// Result of LE OOB Data Generation
__STATIC void gapm_msg_bt_oob_res_cb(uint32_t dest_bf, uint16_t status, const gap_oob_t* p_oob_192, const gap_oob_t* p_oob_256)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        // Provide OOB data to application
        struct gapm_bt_oob_data_ind* p_ind = KE_MSG_ALLOC(GAPM_BT_OOB_DATA_IND, DEST_ID_GET(dest_bf), TASK_GAPM, gapm_bt_oob_data_ind);

        if(p_ind != NULL)
        {
            p_ind->oob_192 = *p_oob_192;
            p_ind->oob_256 = *p_oob_256;
            ke_msg_send(p_ind);
        }
        else
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }

    gapm_msg_default_cmp_evt_handler(dest_bf, status);
}
#endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)

#if (BLE_HOST_PRESENT)
__STATIC void gapm_msg_adv_proc_cmp(uint32_t dest_id, uint8_t proc_id, uint8_t actv_idx, uint16_t status)
{
    uint8_t operation;
    switch(proc_id)
    {
        case GAPM_ACTV_CREATE:                 { operation = GAPM_CREATE_ADV_ACTIVITY; } break;
        case GAPM_ACTV_START:                  { operation = GAPM_START_ACTIVITY; } break;
        case GAPM_ACTV_STOP:                   { operation = GAPM_STOP_ACTIVITY; } break;
        case GAPM_ACTV_DELETE:                 { operation = GAPM_DELETE_ACTIVITY; } break;
        case GAPM_ACTV_SET_ADV_DATA:           { operation = GAPM_SET_ADV_DATA; } break;
        case GAPM_ACTV_SET_SCAN_RSP_DATA:      { operation = GAPM_SET_SCAN_RSP_DATA; } break;
        case GAPM_ACTV_SET_PERIOD_ADV_DATA:    { operation = GAPM_SET_PERIOD_ADV_DATA; } break;
        case GAPM_ACTV_PERIOD_ADV_CTE_TX_CTRL: { operation = GAPM_PER_ADV_CTE_TX_CTL; } break;
        default:                               { operation = GAPM_NO_OP; } break;
    }

    gapm_msg_send_cmp_evt(operation, dest_id, actv_idx, status);
}

__STATIC void gapm_msg_adv_actv_stopped(uint32_t dest_id, uint8_t actv_idx, uint16_t reason)
{
    struct gapm_activity_stopped_ind *p_ind = KE_MSG_ALLOC(GAPM_ACTIVITY_STOPPED_IND, dest_id, TASK_GAPM,
                                                           gapm_activity_stopped_ind);
    if(p_ind)
    {
        gapm_actv_t* p_actv = gapm_actv_get(actv_idx);
        p_ind->actv_idx     = actv_idx;
        p_ind->actv_type    = p_actv->type;
        p_ind->reason       = reason;
        p_ind->per_adv_stop = true;
        ke_msg_send(p_ind);
    }
}

__STATIC void gapm_msg_adv_created(uint32_t dest_id, uint8_t actv_idx, int8_t tx_pwr)
{
    struct gapm_activity_created_ind* p_ind = KE_MSG_ALLOC(GAPM_ACTIVITY_CREATED_IND, dest_id,
                                                           TASK_GAPM, gapm_activity_created_ind);
    if(p_ind)
    {
        p_ind->actv_idx  = actv_idx;
        p_ind->actv_type = GAPM_ACTV_TYPE_ADV;
        p_ind->tx_pwr    = tx_pwr;
        ke_msg_send(p_ind);
    }

}

__STATIC void gapm_msg_adv_scan_req_received(uint32_t dest_id, uint8_t actv_idx, const gap_bdaddr_t* p_addr)
{
    struct gapm_scan_request_ind* p_ind = KE_MSG_ALLOC(GAPM_SCAN_REQUEST_IND, dest_id,
                                                           TASK_GAPM, gapm_scan_request_ind);
    if(p_ind)
    {
        p_ind->actv_idx   = actv_idx;
        p_ind->trans_addr = *p_addr;
        ke_msg_send(p_ind);
    }
}

__STATIC void gapm_msg_scan_proc_cmp(uint32_t dest_id, uint8_t proc_id, uint8_t actv_idx, uint16_t status)
{
    uint8_t operation;
    switch(proc_id)
    {
        case GAPM_ACTV_CREATE:              { operation = GAPM_CREATE_SCAN_ACTIVITY; } break;
        case GAPM_ACTV_START:               { operation = GAPM_START_ACTIVITY; } break;
        case GAPM_ACTV_STOP:                { operation = GAPM_STOP_ACTIVITY; } break;
        case GAPM_ACTV_DELETE:              { operation = GAPM_DELETE_ACTIVITY; } break;
        default:                            { operation = GAPM_NO_OP; } break;
    }
    gapm_msg_send_cmp_evt(operation, dest_id, actv_idx, status);
}

__STATIC void gapm_msg_per_sync_proc_cmp(uint32_t dest_id, uint8_t proc_id, uint8_t actv_idx, uint16_t status)
{
    uint8_t operation;
    switch(proc_id)
    {
        case GAPM_ACTV_CREATE:                { operation = GAPM_CREATE_PERIOD_SYNC_ACTIVITY; } break;
        case GAPM_ACTV_START:                 { operation = GAPM_START_ACTIVITY; } break;
        case GAPM_ACTV_STOP:                  { operation = GAPM_STOP_ACTIVITY; } break;
        case GAPM_ACTV_DELETE:                { operation = GAPM_DELETE_ACTIVITY; } break;
        case GAPM_ACTV_PERIOD_REPORT_CTRL:    { operation = GAPM_PER_ADV_REPORT_CTRL; } break;
        case GAPM_ACTV_PERIOD_IQ_REPORT_CTRL: { operation = GAPM_PER_SYNC_IQ_SAMPLING_CTRL; } break;
        default:                              { operation = GAPM_NO_OP; } break;
    }
    gapm_msg_send_cmp_evt(operation, dest_id, actv_idx, status);
}


__STATIC void gapm_msg_per_sync_established(uint32_t dest_id, uint8_t actv_idx, const gapm_per_sync_info_t* p_info)
{
    struct gapm_sync_established_ind* p_ind = KE_MSG_ALLOC(GAPM_SYNC_ESTABLISHED_IND, dest_id,
                                                           TASK_GAPM, gapm_sync_established_ind);
    if(p_ind)
    {
        p_ind->actv_idx   = actv_idx;
        p_ind->phy        = p_info->phy;
        p_ind->intv       = p_info->interval;
        p_ind->adv_sid    = p_info->adv_sid;
        p_ind->clk_acc    = p_info->clk_acc;
        p_ind->addr       = p_info->addr;
        p_ind->serv_data  = p_info->serv_data;
        ke_msg_send(p_ind);
    }
}

__STATIC void gapm_msg_per_sync_iq_report_received(uint32_t dest_id, uint8_t actv_idx, const gapm_iq_report_info_t* p_info,
                                                   uint8_t nb_sample, const gap_iq_sample_t* p_samples)
{
    uint8_t size = nb_sample * sizeof(gap_iq_sample_t);
    struct gapm_per_adv_iq_report_ind* p_ind = KE_MSG_ALLOC_DYN(GAPM_PER_ADV_IQ_REPORT_IND, dest_id,
                                                                TASK_GAPM, gapm_per_adv_iq_report_ind, size);
    if(p_ind)
    {
        p_ind->actv_idx        = actv_idx;
        p_ind->channel_idx     = p_info->channel_idx;
        p_ind->rssi            = p_info->rssi;
        p_ind->rssi_antenna_id = p_info->rssi_antenna_id;
        p_ind->cte_type        = p_info->cte_type;
        p_ind->slot_dur        = p_info->slot_dur;
        p_ind->pkt_status      = p_info->pkt_status;
        p_ind->pa_evt_cnt      = p_info->pa_evt_cnt;
        p_ind->nb_samples      = nb_sample;
        memcpy(p_ind->sample, p_samples, size);
        ke_msg_send(p_ind);
    }
}

__STATIC void gapm_msg_per_sync_big_info_report_received(uint32_t dest_id, uint8_t actv_idx, const gap_big_info_t* p_report)
{
    struct gapm_big_info_adv_report_ind* p_ind = KE_MSG_ALLOC(GAPM_BIG_INFO_ADV_REPORT_IND, dest_id,
                                                              TASK_GAPM, gapm_big_info_adv_report_ind);
    if(p_ind)
    {
        p_ind->actv_idx   = actv_idx;
        p_ind->report     = *p_report;
        ke_msg_send(p_ind);
    }
}

__STATIC void gapm_msg_init_proc_cmp(uint32_t dest_id, uint8_t proc_id, uint8_t actv_idx, uint16_t status)
{
    uint8_t operation;
    switch(proc_id)
    {
        case GAPM_ACTV_CREATE:              { operation = GAPM_CREATE_INIT_ACTIVITY; } break;
        case GAPM_ACTV_START:               { operation = GAPM_START_ACTIVITY; } break;
        case GAPM_ACTV_STOP:                { operation = GAPM_STOP_ACTIVITY; } break;
        case GAPM_ACTV_DELETE:              { operation = GAPM_DELETE_ACTIVITY; } break;
        default:                            { operation = GAPM_NO_OP; } break;
    }
    gapm_msg_send_cmp_evt(operation, dest_id, actv_idx, status);
}

__STATIC void gapm_msg_init_peer_name(uint32_t dest_id, uint8_t actv_idx, const gap_bdaddr_t* p_addr, uint16_t name_len,
                                      const uint8_t* p_name)
{
    struct gapm_peer_name_ind* p_ind = KE_MSG_ALLOC_DYN(GAPM_PEER_NAME_IND, dest_id,
                                                        TASK_GAPM, gapm_peer_name_ind, name_len);
    if(p_ind)
    {
        p_ind->addr     = *p_addr;
        p_ind->name_len = name_len;
        memcpy(p_ind->name, p_name, name_len);
        ke_msg_send(p_ind);
    }
}

#if (HOST_TEST_MODE)
__STATIC void gapm_msg_test_tx_proc_cmp(uint32_t dest_id, uint8_t proc_id, uint8_t actv_idx, uint16_t status)
{
    switch(proc_id)
    {
        case GAPM_ACTV_START:
        {
            gapm_msg_send_cmp_evt(GAPM_LE_TEST_TX_START, dest_id, actv_idx, status);
        } break;
        case GAPM_ACTV_STOP:
        {
            gapm_msg_send_cmp_evt(GAPM_LE_TEST_STOP, dest_id, actv_idx, status);
            gapm_actv_delete(actv_idx);
        } break;
        default: { /* Ignore */  } break;
    }
}

__STATIC void gapm_msg_test_rx_proc_cmp(uint32_t dest_id, uint8_t proc_id, uint8_t actv_idx, uint16_t status)
{
    switch(proc_id)
    {
        case GAPM_ACTV_START:
        {
            gapm_msg_send_cmp_evt(GAPM_LE_TEST_RX_START, dest_id, actv_idx, status);
        } break;
        case GAPM_ACTV_STOP:
        {
            gapm_msg_send_cmp_evt(GAPM_LE_TEST_STOP, dest_id, actv_idx, status);
            gapm_actv_delete(actv_idx);
        } break;
        default: { /* Ignore */  } break;
    }
}

__STATIC void gapm_msg_test_stopped(uint32_t dest_id, uint8_t actv_idx, uint16_t reason)
{
    // ignore
}

/// Callback executed when RX Test mode is stopped to provide number of packet received
__STATIC void gapm_msg_test_rx_nb_packet_received(uint32_t dest_id, uint8_t actv_idx, uint16_t nb_packet)
{
    struct gapm_le_test_end_ind* p_ind = KE_MSG_ALLOC(GAPM_LE_TEST_END_IND, dest_id, TASK_GAPM, gapm_le_test_end_ind);
    if(p_ind)
    {
        p_ind->nb_packet_received = nb_packet;
        ke_msg_send(p_ind);
    }
}

__STATIC void gapm_msg_test_rx_iq_report_received(uint32_t dest_id, uint8_t actv_idx, const gapm_iq_report_info_t* p_info,
                                                  uint8_t nb_sample, const gap_iq_sample_t* p_samples)
{
    uint8_t size = nb_sample * sizeof(gap_iq_sample_t);
    struct gapm_le_test_iq_report_ind* p_ind = KE_MSG_ALLOC_DYN(GAPM_LE_TEST_IQ_REPORT_IND, dest_id,
                                                                TASK_GAPM, gapm_le_test_iq_report_ind, size);
    if(p_ind)
    {
        p_ind->channel_idx     = p_info->channel_idx;
        p_ind->rssi            = p_info->rssi;
        p_ind->rssi_antenna_id = p_info->rssi_antenna_id;
        p_ind->cte_type        = p_info->cte_type;
        p_ind->slot_dur        = p_info->slot_dur;
        p_ind->pkt_status      = p_info->pkt_status;
        p_ind->pa_evt_cnt      = p_info->pa_evt_cnt;
        p_ind->nb_samples      = nb_sample;
        memcpy(p_ind->sample, p_samples, size);
        ke_msg_send(p_ind);
    }
}
#endif // (HOST_TEST_MODE)
#endif // (BLE_HOST_PRESENT)

#if(BT_HOST_PRESENT)
__STATIC void gapm_msg_inquiry_proc_cmp(uint32_t dest_id, uint8_t proc_id, uint8_t actv_idx, uint16_t status)
{
    uint8_t operation;
    switch(proc_id)
    {
        case GAPM_ACTV_CREATE:              { operation = GAPM_CREATE_INQUIRY_ACTIVITY; } break;
        case GAPM_ACTV_START:               { operation = GAPM_START_ACTIVITY; } break;
        case GAPM_ACTV_STOP:                { operation = GAPM_STOP_ACTIVITY; } break;
        case GAPM_ACTV_DELETE:              { operation = GAPM_DELETE_ACTIVITY; } break;
        default:                            { operation = GAPM_NO_OP; } break;
    }
    gapm_msg_send_cmp_evt(operation, dest_id, actv_idx, status);
}

__STATIC void gapm_msg_inquiry_report_received(uint32_t dest_id, uint8_t actv_idx, const gapm_inquiry_report_t* p_report,
                                           co_buf_t* p_eir_data)
{
    uint8_t eir_data_len = p_eir_data == NULL ? 0 : co_buf_data_len(p_eir_data);
    struct gapm_inquiry_report_ind* p_ind = KE_MSG_ALLOC_DYN(GAPM_INQUIRY_REPORT_IND, dest_id,
                                                             TASK_GAPM, gapm_inquiry_report_ind, eir_data_len);
    if(p_ind)
    {
        p_ind->actv_idx   = actv_idx;
        p_ind->report     = *p_report;
        p_ind->eir_length = eir_data_len;
        if(eir_data_len > 0)
        {
            memcpy(p_ind->eir_data, co_buf_data(p_eir_data), eir_data_len);
        }
        ke_msg_send(p_ind);
    }

}

__STATIC void gapm_msg_inquiry_scan_proc_cmp(uint32_t dest_id, uint8_t proc_id, uint8_t actv_idx, uint16_t status)
{
    uint8_t operation;
    switch(proc_id)
    {
        case GAPM_ACTV_CREATE:              { operation = GAPM_CREATE_INQUIRY_SCAN_ACTIVITY; } break;
        case GAPM_ACTV_START:               { operation = GAPM_START_ACTIVITY; } break;
        case GAPM_ACTV_STOP:                { operation = GAPM_STOP_ACTIVITY; } break;
        case GAPM_ACTV_DELETE:              { operation = GAPM_DELETE_ACTIVITY; } break;
        default:                            { operation = GAPM_NO_OP; } break;
    }
    gapm_msg_send_cmp_evt(operation, dest_id, actv_idx, status);
}
__STATIC void gapm_msg_page_proc_cmp(uint32_t dest_id, uint8_t proc_id, uint8_t actv_idx, uint16_t status)
{
    uint8_t operation;
    switch(proc_id)
    {
        case GAPM_ACTV_CREATE:              { operation = GAPM_CREATE_PAGE_ACTIVITY; } break;
        case GAPM_ACTV_START:               { operation = GAPM_START_ACTIVITY; } break;
        case GAPM_ACTV_STOP:                { operation = GAPM_STOP_ACTIVITY; } break;
        case GAPM_ACTV_DELETE:              { operation = GAPM_DELETE_ACTIVITY; } break;
        default:                            { operation = GAPM_NO_OP; } break;
    }
    gapm_msg_send_cmp_evt(operation, dest_id, actv_idx, status);
}

__STATIC void gapm_msg_page_scan_proc_cmp(uint32_t dest_id, uint8_t proc_id, uint8_t actv_idx, uint16_t status)
{
    uint8_t operation;
    switch(proc_id)
    {
        case GAPM_ACTV_CREATE:              { operation = GAPM_CREATE_PAGE_SCAN_ACTIVITY; } break;
        case GAPM_ACTV_START:               { operation = GAPM_START_ACTIVITY; } break;
        case GAPM_ACTV_STOP:                { operation = GAPM_STOP_ACTIVITY; } break;
        case GAPM_ACTV_DELETE:              { operation = GAPM_DELETE_ACTIVITY; } break;
        default:                            { operation = GAPM_NO_OP; } break;
    }
    gapm_msg_send_cmp_evt(operation, dest_id, actv_idx, status);
}

#endif // (BT_HOST_PRESENT)

__STATIC void gapm_msg_actv_stopped(uint32_t dest_id, uint8_t actv_idx, uint16_t reason)
{
    struct gapm_activity_stopped_ind *p_ind = KE_MSG_ALLOC(GAPM_ACTIVITY_STOPPED_IND, dest_id, TASK_GAPM,
                                                           gapm_activity_stopped_ind);
    if(p_ind)
    {
        gapm_actv_t* p_actv = gapm_actv_get(actv_idx);
        p_ind->actv_idx     = actv_idx;
        p_ind->actv_type    = p_actv->type;
        p_ind->reason       = reason;
        p_ind->per_adv_stop = false;
        ke_msg_send(p_ind);
    }
}

#if (BLE_HOST_PRESENT)
__STATIC void gapm_msg_actv_addr_updated(uint32_t dest_id, uint8_t actv_idx, const gap_addr_t* p_addr)
{
    struct gapm_dev_bdaddr_ind *p_ind = KE_MSG_ALLOC(GAPM_DEV_BDADDR_IND, dest_id, TASK_GAPM, gapm_dev_bdaddr_ind);
    if(p_ind)
    {
        memcpy(p_ind->addr.addr, p_addr, GAP_BD_ADDR_LEN);
        p_ind->addr.addr_type = ADDR_RAND;
        p_ind->actv_idx = actv_idx;
        ke_msg_send(p_ind);
    }
}

__STATIC void gapm_msg_actv_report_received(uint32_t dest_id, uint8_t actv_idx, const gapm_adv_report_info_t* p_info, co_buf_t* p_report)
{
    uint16_t data_len = co_buf_data_len(p_report);
    struct gapm_ext_adv_report_ind* p_ind = KE_MSG_ALLOC_DYN(GAPM_EXT_ADV_REPORT_IND, dest_id,
                                                             TASK_GAPM, gapm_ext_adv_report_ind, data_len);
    if(p_ind)
    {
        p_ind->actv_idx        = actv_idx;
        p_ind->info            = p_info->info;
        p_ind->trans_addr      = p_info->trans_addr;
        p_ind->target_addr     = p_info->target_addr;
        p_ind->tx_pwr          = p_info->tx_pwr;
        p_ind->rssi            = p_info->rssi;
        p_ind->phy_prim        = p_info->phy_prim;
        p_ind->phy_second      = p_info->phy_second;
        p_ind->adv_sid         = p_info->adv_sid;
        p_ind->period_adv_intv = p_info->period_adv_intv;
        p_ind->length          = data_len;
        memcpy(p_ind->data,      co_buf_data(p_report), data_len);
        ke_msg_send(p_ind);
    }
}
#endif // (BLE_HOST_PRESENT)


/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles request of host reset + Initialization of lower layers.
 *  - GAPM_RESET: software reset operation.
 *  - GAPM_PLF_RESET: Platform reset
 *
 *  Procedure:
 *   1. HCI_RESET_CMD
 *   2. HCI_SET_EVT_MASK_CMD
 *   3. HCI_LE_SET_EVT_MASK_CMD
 *   4. HCI_RD_BD_ADDR_CMD
 *   5. HCI_LE_RD_BUF_SIZE_CMD
 *   6. HCI_RD_BUF_SIZE_CMD
 ****************************************************************************************
 */
__STATIC int gapm_reset_cmd_handler(ke_msg_id_t const msgid, const struct gapm_reset_cmd *p_cmd,
                                    ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPM_RESET)
    {
        status = gapm_reset(DEST_BF(p_cmd->operation, src_id), gapm_msg_default_cmp_evt_handler);
    }
    else if (p_cmd->operation == GAPM_PLF_RESET)
    {
        // Reset the platform
        platform_reset(RESET_AND_LOAD_FW);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, GAP_INVALID_ACTV_IDX, status);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles modification of device GAP configuration such as role, security
 * parameters, etc:
 *  - GAPM_SET_DEV_CONFIG: Set device configuration
 ****************************************************************************************
 */
__STATIC int gapm_set_dev_config_cmd_handler(ke_msg_id_t const msgid, const struct gapm_set_dev_config_cmd* p_cmd,
                                             ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPM_SET_DEV_CONFIG)
    {
        const gapm_callbacks_t* p_cbs = NULL;

        #if(GAPC_PRESENT)
        p_cbs = gapc_msg_get_callback_itf();
        #endif // (GAPC_PRESENT)

        status = gapm_set_dev_config(DEST_BF(p_cmd->operation, src_id), &(p_cmd->cfg), p_cbs, gapm_msg_set_dev_config_cmp_handler);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, GAP_INVALID_ACTV_IDX, status);
    }

    return (KE_MSG_CONSUMED);
}


#if (BLE_HOST_PRESENT)
/**
 ****************************************************************************************
 * @brief Handles request of modifying local channel map:
 *  - GAPM_SET_LE_CHANNEL_MAP:  Set device channel map
 ****************************************************************************************
 */
__STATIC int gapm_set_le_channel_map_cmd_handler(ke_msg_id_t const msgid, const struct gapm_set_le_channel_map_cmd *p_cmd,
                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPM_SET_LE_CHANNEL_MAP)
    {
        status = gapm_set_le_chmap(DEST_BF(p_cmd->operation, src_id), &(p_cmd->chmap), gapm_msg_default_cmp_evt_handler);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, GAP_INVALID_ACTV_IDX, status);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles request of changing the IRK
 * - GAPM_SET_IRK:  Set IRK
 ****************************************************************************************
 */
__STATIC int gapm_set_irk_cmd_handler(ke_msg_id_t const msgid, struct gapm_set_irk_cmd *p_cmd,
                             ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPM_SET_IRK)
    {
        status = gapm_set_irk(&(p_cmd->irk));
    }

    gapm_msg_send_cmp_evt(p_cmd->operation, src_id, GAP_INVALID_ACTV_IDX, status);

    return (KE_MSG_CONSUMED);
}
#endif // (BLE_HOST_PRESENT)


/**
 ****************************************************************************************
 * @brief Handles request of changing the Device Name
 * - GAPM_SET_NAME:  Set Device name
 ****************************************************************************************
 */
__STATIC int gapm_set_name_cmd_handler(ke_msg_id_t const msgid, struct gapm_set_name_cmd *p_cmd,
                             ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t status = GAP_ERR_INVALID_PARAM;

    do
    {
        if(p_cmd->operation != GAPM_SET_NAME) break;

        // Name modification on-going
        if(gapm_msg_env.p_new_name != NULL)
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // Allocate Name for message API - Command isn't kept
        gapm_msg_env.p_new_name = (uint8_t*) ke_malloc_user(p_cmd->name_len, KE_MEM_PROFILE);
        if(gapm_msg_env.p_new_name == NULL)
        {
            status = GAP_ERR_INSUFF_RESOURCES;
            break;
        }

        memcpy(gapm_msg_env.p_new_name, p_cmd->name, p_cmd->name_len);

        status = gapm_set_name(DEST_BF(p_cmd->operation, src_id), p_cmd->name_len, gapm_msg_env.p_new_name, gapm_msg_set_name_cmp_handler);

        // An error occurs, remove name allocated for message API
        if(status != GAP_ERR_NO_ERROR)
        {
            ke_free(gapm_msg_env.p_new_name);
            gapm_msg_env.p_new_name = NULL;
        }
    } while(0);

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, GAP_INVALID_ACTV_IDX, status);
    }

    return (KE_MSG_CONSUMED);
}

    #if (BT_HOST_PRESENT)

/// Handles GAPM_SET_SDP_DEVICE_IDENTIFICATION_RECORD_CMD message
__STATIC int gapm_set_sdp_device_identification_record_cmd_handler(ke_msg_id_t const msgid,
                                                                   const struct gapm_set_sdp_device_identification_record_cmd *p_cmd,
                                                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = GAP_ERR_INVALID_PARAM;
    if(p_cmd->operation == GAPM_SET_SDP_DEVICE_IDENTIFICATION_RECORD)
    {
        status = gapm_set_sdp_device_identification_record(p_cmd->vendor_id_source, p_cmd->vendor_id,
                                                           p_cmd->product_id, p_cmd->version);
    }

    gapm_msg_send_cmp_evt(p_cmd->operation, src_id, GAP_INVALID_ACTV_IDX, status);
    return (KE_MSG_CONSUMED);
}
#endif // (BT_HOST_PRESENT)

#if (BLE_HOST_PRESENT)
__STATIC void gapm_list_set_cmd_cmp_handler(uint32_t dest_bf, uint16_t status)
{
    if(gapm_msg_env.p_temp_param != NULL)
    {
        ke_free(gapm_msg_env.p_temp_param);
        gapm_msg_env.p_temp_param = NULL;
    }

    gapm_msg_default_cmp_evt_handler(dest_bf, status);
}

/**
 ****************************************************************************************
 * @brief Handles request of setting content of lists:
 *  - GAPM_SET_WL: Set content of white list
 *  - GAPM_SET_RAL: Set content of resolving list
 *  - GAPM_SET_PAL: Set content of periodic advertiser list
 ****************************************************************************************
 */
__STATIC int gapm_list_set_cmd_handler(ke_msg_id_t const msgid,
                              struct gapm_list_set_cmd *p_cmd,
                              ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;

    if(gapm_msg_env.p_temp_param == NULL)
    {
        switch(p_cmd->operation)
        {
            case GAPM_SET_WL:
            {
                uint16_t size = sizeof(gap_bdaddr_t) * p_cmd->size;
                gap_bdaddr_t* p_array = ke_malloc_user(sizeof(gap_bdaddr_t) * p_cmd->size, KE_MEM_NON_RETENTION);
                if(p_array == NULL) break;
                memcpy(p_array, ((struct gapm_list_set_wl_cmd*)p_cmd)->wl_info, size);
                gapm_msg_env.p_temp_param = p_array;

                status = gapm_fill_white_list(DEST_BF(p_cmd->operation, src_id), p_cmd->size, p_array, gapm_list_set_cmd_cmp_handler);
            } break;
            case GAPM_SET_RAL:
            {
                uint16_t size = sizeof(gap_ral_dev_info_t) * p_cmd->size;
                gap_ral_dev_info_t* p_array = ke_malloc_user(sizeof(gap_ral_dev_info_t) * p_cmd->size, KE_MEM_NON_RETENTION);
                if(p_array == NULL) break;

                memcpy(p_array, ((struct gapm_list_set_ral_cmd*)p_cmd)->ral_info, size);
                gapm_msg_env.p_temp_param = p_array;

                status = gapm_fill_resolving_address_list(DEST_BF(p_cmd->operation, src_id), p_cmd->size, p_array, gapm_list_set_cmd_cmp_handler);
            } break;
            case GAPM_SET_PAL:
            {
                uint16_t size = sizeof(gap_per_adv_bdaddr_t) * p_cmd->size;
                gap_per_adv_bdaddr_t* p_array = ke_malloc_user(size, KE_MEM_NON_RETENTION);
                if(p_array == NULL) break;

                memcpy(p_array, ((struct gapm_list_set_pal_cmd*)p_cmd)->pal_info, size);
                gapm_msg_env.p_temp_param = p_array;

                status = gapm_fill_periodic_adv_list(DEST_BF(p_cmd->operation, src_id), p_cmd->size, p_array, gapm_list_set_cmd_cmp_handler);
            } break;
            default: { status = GAP_ERR_INVALID_PARAM; }  break;
        }

        if(status != GAP_ERR_NO_ERROR)
        {
            if(gapm_msg_env.p_temp_param != NULL)
            {
                ke_free(gapm_msg_env.p_temp_param);
                gapm_msg_env.p_temp_param = NULL;
            }

            gapm_msg_send_cmp_evt(p_cmd->operation, src_id, GAP_INVALID_ACTV_IDX, status);
        }
    }
    else
    {
        gapm_msg_send_cmp_evt(GAP_ERR_BUSY, src_id, GAP_INVALID_ACTV_IDX, status);
    }

    return (KE_MSG_CONSUMED);
}

/// result of GET local or peer random address
__STATIC void gapm_msg_get_ral_rand_addr_cb(uint32_t dest_bf, uint16_t status, const gap_addr_t* p_addr)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        // Send connection indication message to task that requests connection.
        struct gapm_ral_addr_ind * p_ind =
                KE_MSG_ALLOC(GAPM_RAL_ADDR_IND, DEST_ID_GET(dest_bf), TASK_GAPM, gapm_ral_addr_ind);

        if(p_ind)
        {
            p_ind->operation = OPERATION_GET(dest_bf);
            p_ind->addr.addr_type = ADDR_RAND;
            memcpy(p_ind->addr.addr, p_addr, GAP_BD_ADDR_LEN);
            ke_msg_send(p_ind);
        }
    }

    gapm_msg_default_cmp_evt_handler(dest_bf, status);
}

/**
 ****************************************************************************************
 * @brief Handles request of reading local or peer address:
 *  - GAPM_GET_RAL_LOC_ADDR: Get resolving local address
 *  - GAPM_GET_RAL_PEER_ADDR: Get resolving peer address
 ****************************************************************************************
 */
__STATIC int gapm_get_ral_addr_cmd_handler(ke_msg_id_t const msgid,
                                  struct gapm_get_ral_addr_cmd *p_cmd,
                                  ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status;

    switch(p_cmd->operation)
    {
        case GAPM_GET_RAL_LOC_ADDR:
        {
           status = gapm_get_ral_local_rpa(DEST_BF(p_cmd->operation, src_id), &(p_cmd->peer_identity), gapm_msg_get_ral_rand_addr_cb);
        } break;
        case GAPM_GET_RAL_PEER_ADDR:
        {
           status = gapm_get_ral_peer_rpa(DEST_BF(p_cmd->operation, src_id), &(p_cmd->peer_identity), gapm_msg_get_ral_rand_addr_cb);
        } break;
        default: { status = GAP_ERR_INVALID_PARAM; }  break;
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, GAP_INVALID_ACTV_IDX, status);
    }

    return (KE_MSG_CONSUMED);
}

/// Result of address resolution
__STATIC void gapm_msg_addr_resolve_res_cb(uint16_t status, const gap_addr_t* p_addr, const gap_sec_key_t* p_irk)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        // Indicate which key has been used to resolve the random address.
        struct gapm_addr_solved_ind *p_ind = KE_MSG_ALLOC(GAPM_ADDR_SOLVED_IND, gapm_env.aes_src_id, TASK_GAPM, gapm_addr_solved_ind);
        if(p_ind != NULL)
        {
            // Provide Address resolved
            memcpy(&(p_ind->addr), p_addr, sizeof(gap_addr_t));
            // Provide IRK used for address resolution
            memcpy(&(p_ind->irk), p_irk, sizeof(struct gap_sec_key));
            // Send the message
            ke_msg_send(p_ind);
        }
    }

    // Send the command complete event message
    gapm_msg_send_cmp_evt(GAPM_RESOLV_ADDR, gapm_env.aes_src_id, GAP_INVALID_ACTV_IDX, status);
}

/**
 ****************************************************************************************
 * @brief Handles request of solving a resolvable random address.
 ****************************************************************************************
 */
__STATIC int gapm_resolv_addr_cmd_handler(ke_msg_id_t const msgid, struct gapm_resolv_addr_cmd *p_cmd,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPM_RESOLV_ADDR)
    {
        status = gapm_resolve_address(&(p_cmd->addr), p_cmd->nb_key, p_cmd->irk, gapm_msg_addr_resolve_res_cb);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, GAP_INVALID_ACTV_IDX, status);
    }
    else
    {
        gapm_env.aes_src_id = src_id;
    }

    return (KE_MSG_CONSUMED);
}

#if (HOST_TEST_MODE)
/**
 ****************************************************************************************
 * @brief Handles request of controlling test mode
 *  - GAPM_LE_TEST_STOP: Unregister a Simplified Protocol/Service Multiplexer
 *  - GAPM_LE_TEST_RX_START: Start RX Test Mode
 *  - GAPM_LE_TEST_TX_START: Start TX Test Mode
 ****************************************************************************************
 */
__STATIC int gapm_le_test_mode_ctrl_cmd_handler(ke_msg_id_t const msgid, struct gapm_le_test_mode_ctrl_cmd *p_cmd,
                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;

    switch(p_cmd->operation)
    {
        case GAPM_LE_TEST_RX_START:
        {
            uint8_t actv_idx;
            gapm_le_test_rx_param_t param;
            gapm_le_test_cte_param_t cte_param;
            if(gapm_env.test_actv_idx != GAP_INVALID_ACTV_IDX) break;

            status = gapm_le_test_rx_create(DEST_BF(p_cmd->operation, src_id), &gapm_msg_test_rx_actv_cb, &actv_idx);
            if(status != GAP_ERR_NO_ERROR) break;

            param.channel                   = p_cmd->channel;
            param.phy                       = p_cmd->phy;
            param.modulation_idx            = p_cmd->modulation_idx;
            param.slot_dur                  = p_cmd->slot_dur;
            cte_param.cte_len               = p_cmd->cte_len;
            cte_param.cte_type              = p_cmd->cte_type;
            cte_param.switching_pattern_len = p_cmd->switching_pattern_len;

            status = gapm_le_test_rx_start_with_cte(actv_idx, &param, &cte_param, p_cmd->antenna_id);
        } break;
        case GAPM_LE_TEST_TX_START:
        {
            uint8_t actv_idx;
            gapm_le_test_tx_param_t param;
            gapm_le_test_cte_param_t cte_param;
            if(gapm_env.test_actv_idx != GAP_INVALID_ACTV_IDX) break;

            status = gapm_le_test_tx_create(DEST_BF(p_cmd->operation, src_id), &gapm_msg_test_tx_actv_cb, &actv_idx);
            if(status != GAP_ERR_NO_ERROR) break;

            param.channel                   = p_cmd->channel;
            param.tx_data_length            = p_cmd->tx_data_length;
            param.tx_pkt_payload            = p_cmd->tx_pkt_payload;
            param.phy                       = p_cmd->phy;
            param.tx_pwr_lvl                = p_cmd->tx_pwr_lvl;
            cte_param.cte_len               = p_cmd->cte_len;
            cte_param.cte_type              = p_cmd->cte_type;
            cte_param.switching_pattern_len = p_cmd->switching_pattern_len;

            status = gapm_le_test_tx_start_with_cte(actv_idx, &param, &cte_param, p_cmd->antenna_id);
        } break;

        case GAPM_LE_TEST_STOP:
        {
           status = gapm_actv_stop(gapm_env.test_actv_idx);
        } break;
        default: { status = GAP_ERR_INVALID_PARAM; }  break;
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, GAP_INVALID_ACTV_IDX, status);
    }

    return (KE_MSG_CONSUMED);
}
#endif // (HOST_TEST_MODE)

// @brief Handles request of configuring I/Q sample genertator
__STATIC int gapm_dbg_iqgen_cfg_cmd_handler(ke_msg_id_t const msgid, const struct gapm_dbg_iqgen_cfg_cmd* p_cmd,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPM_DBG_IQGEN_CFG)
    {
        status = gapm_dbg_iqgen_cfg(DEST_BF(p_cmd->operation, src_id), p_cmd->mode, p_cmd->nb_antenna, p_cmd->iq_ctrl,
                                    gapm_msg_default_cmp_evt_handler);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, GAP_INVALID_ACTV_IDX, status);
    }

    return (KE_MSG_CONSUMED);
}
#endif // (BLE_HOST_PRESENT)
#if (BT_HOST_PRESENT && HOST_TEST_MODE)

#if (BT_HOST_PRESENT && HOST_TEST_MODE)
// @brief Handles GAPM_BT_WRITE_LOOPBACK_MODE_CMD
__STATIC int gapm_bt_write_loopback_mode_cmd_handler(ke_msg_id_t const msgid, const struct gapm_bt_write_loopback_mode_cmd* p_cmd,
                                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPM_BT_WRITE_LOOPBACK_MODE)
    {
        status = gapm_bt_write_loopback_mode(DEST_BF(p_cmd->operation, src_id), p_cmd->loopback_mode, gapm_msg_default_cmp_evt_handler);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, GAP_INVALID_ACTV_IDX, status);
    }

    return (KE_MSG_CONSUMED);
}

// @brief Handles GAPM_BT_ENABLE_DEVICE_UNDER_TEST_MODE_CMD
__STATIC int gapm_bt_enable_device_under_test_mode_cmd_handler(ke_msg_id_t const msgid, const struct gapm_bt_enable_device_under_test_mode_cmd* p_cmd,
                                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPM_BT_ENABLE_DEVICE_UNDER_TEST_MODE)
    {
        status = gapm_bt_enable_device_under_test_mode(DEST_BF(p_cmd->operation, src_id),  gapm_msg_default_cmp_evt_handler);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, GAP_INVALID_ACTV_IDX, status);
    }

    return (KE_MSG_CONSUMED);
}

// @brief Handles GAPM_BT_WRITE_SIMPLE_PAIRING_DEBUG_MODE_CMD
__STATIC int gapm_bt_write_simple_pairing_debug_mode_cmd_handler(ke_msg_id_t const msgid, const struct gapm_bt_write_simple_pairing_debug_mode_cmd* p_cmd,
                                                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPM_BT_WRITE_SIMPLE_PAIRING_DEBUG_MODE)
    {
        status = gapm_bt_write_simple_pairing_debug_mode(DEST_BF(p_cmd->operation, src_id), p_cmd->enable_debug_mode, gapm_msg_default_cmp_evt_handler);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, GAP_INVALID_ACTV_IDX, status);
    }

    return (KE_MSG_CONSUMED);
}

// @brief Handles GAPM_BT_WRITE_SECURE_CONNECTIONS_TEST_MODE_CMD
__STATIC int gapm_bt_write_secure_connections_test_mode_cmd_handler(ke_msg_id_t const msgid, const struct gapm_bt_write_secure_connections_test_mode_cmd* p_cmd,
                                                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPM_BT_WRITE_SECURE_CONNECTIONS_TEST_MODE)
    {
        status = gapm_bt_write_secure_connections_test_mode(DEST_BF(p_cmd->operation, src_id), p_cmd->conidx, p_cmd->enable_dm1_acl_u_mode,
                                                            p_cmd->enable_esco_loopback_mode, gapm_msg_default_cmp_evt_handler);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, GAP_INVALID_ACTV_IDX, status);
    }

    return (KE_MSG_CONSUMED);
}

#endif // (BT_HOST_PRESENT && HOST_TEST_MODE)

/// handle result of read loopback mode procedure
__STATIC void gapm_msg_bt_read_loopback_mode_res_handler(uint32_t dest_bf, uint16_t status, uint8_t loopback_mode)
{
    if(status == GAP_ERR_NO_ERROR)
    {
        // Indicate which key has been used to resolve the random address.
        struct gapm_bt_loopback_ind *p_ind = KE_MSG_ALLOC(GAPM_BT_LOOPBACK_IND, DEST_ID_GET(dest_bf), TASK_GAPM, gapm_bt_loopback_ind);
        if(p_ind != NULL)
        {
            p_ind->loopback_mode = loopback_mode;
            // Send the message
            ke_msg_send(p_ind);
        }
    }

    gapm_msg_default_cmp_evt_handler(dest_bf, status);
}
#endif // (BT_HOST_PRESENT && HOST_TEST_MODE)

/**
 ****************************************************************************************
 * @brief Handles request of getting information about local device such as:
 * - GAPM_GET_DEV_NAME: Get Local device name indication event
 * - GAPM_GET_DEV_VERSION: Get Local device version indication event
 * - GAPM_GET_DEV_BDADDR: Get Local device BD Address indication event
 * - GAPM_GET_DEV_ADV_TX_POWER: Get device advertising power level
 * - GAPM_DBG_GET_MEM_INFO: Get memory usage (debug only)
 * - GAPM_DBG_GET_STATS_INFO: Get Statistics
 ****************************************************************************************
 */
__STATIC int gapm_get_dev_info_cmd_handler(ke_msg_id_t const msgid, const struct gapm_get_dev_info_cmd *p_cmd,
                                  ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = GAP_ERR_INVALID_PARAM;
    bool send_cmd_cmp_evt = false;
    uint8_t operation = p_cmd->operation;
    int msg_status = KE_MSG_CONSUMED;

    // check operation
    switch(operation)
    {
        // Get Local device BD Address
        case GAPM_GET_DEV_BDADDR:
        {
            struct gapm_dev_bdaddr_ind *p_bdaddr_ind =
                    KE_MSG_ALLOC(GAPM_DEV_BDADDR_IND, src_id, dest_id, gapm_dev_bdaddr_ind);

            if(p_bdaddr_ind != NULL)
            {
                status = gapm_get_identity(&p_bdaddr_ind->addr);
                p_bdaddr_ind->actv_idx = GAPM_ACTV_INVALID_IDX;
                ke_msg_send(p_bdaddr_ind);
                status = GAP_ERR_NO_ERROR;
            }
            else
            {
                status = GAP_ERR_INSUFF_RESOURCES;
            }

            send_cmd_cmp_evt = true;
        } break;

        // Get device memory usage
        case GAPM_DBG_GET_MEM_INFO:
        {
            #if (KE_PROFILING)
            uint8_t cursor;
            struct gapm_dbg_mem_info_ind meminfo;
            struct gapm_dbg_mem_info_ind * p_meminfo_msg;

            // First remove command message in order to be sure it's not taken in account.
            msg_status = KE_MSG_NO_FREE;
            ke_msg_free(ke_param2msg(p_cmd));

            // Then retrieve memory information from kernel
            meminfo.max_mem_used = ke_get_max_mem_usage();
            for(cursor = 0; cursor < KE_MEM_BLOCK_MAX ; cursor++)
            {
                meminfo.mem_used[cursor] = ke_get_mem_usage(cursor);
            }

            // Finally send indication to application that request memory information
            p_meminfo_msg = KE_MSG_ALLOC(GAPM_DBG_MEM_INFO_IND, src_id, dest_id, gapm_dbg_mem_info_ind);

            if(p_meminfo_msg != NULL)
            {
                memcpy(p_meminfo_msg, &meminfo, sizeof(struct gapm_dbg_mem_info_ind));
                ke_msg_send(p_meminfo_msg);
                status = GAP_ERR_NO_ERROR;
            }
            else
            {
                status = GAP_ERR_INSUFF_RESOURCES;
            }

            send_cmd_cmp_evt = true;
            #else
            status = GAP_ERR_NOT_SUPPORTED;
            #endif /* (KE_PROFILING) */
        }
        break;

        // Get device statistics
        case GAPM_DBG_GET_STATS_INFO:
        {
            #if (KE_PROFILING)
            uint32_t max_msg_sent   = 0;
            uint32_t max_msg_saved  = 0;
            uint32_t max_timer_used = 0;
            uint32_t max_heap_used  = 0;
            uint32_t max_stack_used = 0;
            struct gapm_dbg_stats_ind* p_ind;
            send_cmd_cmp_evt = true;

            if(ke_stats_get(&max_msg_sent, &max_msg_saved, &max_timer_used, &max_heap_used) != KE_SUCCESS)
            {
                status = GAP_ERR_NOT_SUPPORTED;
                break;
            }
            status = GAP_ERR_NO_ERROR;

            #if (RW_DEBUG_STACK_PROF)
            max_stack_used = get_stack_usage();
            #endif //RW_DEBUG_STACK_PROF

            p_ind = KE_MSG_ALLOC(GAPM_DBG_STATS_IND, src_id, dest_id, gapm_dbg_stats_ind);
            if(p_ind == NULL)
            {
                status = GAP_ERR_INSUFF_RESOURCES;
                break;
            }

            p_ind->max_msg_sent   = max_msg_sent;
            p_ind->max_msg_saved  = max_msg_saved;
            p_ind->max_timer_used = max_timer_used;
            p_ind->max_heap_used  = max_heap_used;
            p_ind->max_stack_used = max_stack_used;

            ke_msg_send(p_ind);
            #else
            status = GAP_ERR_NOT_SUPPORTED;
            #endif // KE_PROFILING
        } break;

        // Get Local device version
        case GAPM_GET_DEV_VERSION: { status = gapm_get_version(DEST_BF(p_cmd->operation, src_id), gapm_msg_version_handler); } break;

        #if(BLE_HOST_PRESENT)
        // Get device advertising Tx Power level
        case GAPM_GET_DEV_ADV_TX_POWER: { status = gapm_get_adv_tx_power(DEST_BF(p_cmd->operation, src_id), gapm_msg_adv_tx_power_handler); } break;
        // Get Suggested Default Data Length
        case GAPM_GET_SUGGESTED_DFLT_LE_DATA_LEN: { status = gapm_get_sugg_dflt_data_len(DEST_BF(p_cmd->operation, src_id), gapm_msg_sugg_dflt_data_len_handler); } break;
        // Get Maximum LE Data Length
        case GAPM_GET_MAX_LE_DATA_LEN: { status = gapm_get_max_le_data_len(DEST_BF(p_cmd->operation, src_id), gapm_msg_max_le_data_len_handler); } break;
        // Get white list size
        case GAPM_GET_WLIST_SIZE: { status = gapm_get_wlist_size(DEST_BF(p_cmd->operation, src_id), gapm_msg_list_size_handler); } break;
        // Get periodic advertiser list size
        case GAPM_GET_PAL_SIZE: { status = gapm_get_pal_size(DEST_BF(p_cmd->operation, src_id), gapm_msg_list_size_handler); } break;
        // Get resolving list size
        case GAPM_GET_RAL_SIZE: { status = gapm_get_ral_size(DEST_BF(p_cmd->operation, src_id), gapm_msg_list_size_handler); } break;
        // Get available number of advertising sets
        case GAPM_GET_NB_ADV_SETS: { status = gapm_get_nb_adv_set(DEST_BF(p_cmd->operation, src_id), gapm_msg_nb_adv_set_handler); } break;
        // Get maximum advertising data length supported by controller
        case GAPM_GET_MAX_LE_ADV_DATA_LEN: { status = gapm_get_max_adv_len(DEST_BF(p_cmd->operation, src_id), gapm_msg_max_adv_len_handler); } break;
        // Get minimum and maximum transmit power level supported by the controller
        case GAPM_GET_DEV_TX_PWR: { status = gapm_get_tx_pwr_rng(DEST_BF(p_cmd->operation, src_id), gapm_msg_tx_pwr_rng_handler); } break;
        // Get RF path compensation values
        case GAPM_GET_DEV_RF_PATH_COMP: { status = gapm_get_rf_path_comp(DEST_BF(p_cmd->operation, src_id), gapm_msg_rf_path_comp_handler); } break;
        // Get Antenna information
        case GAPM_GET_ANTENNA_INFO: { status = gapm_get_antenna_inf(DEST_BF(p_cmd->operation, src_id), gapm_msg_antenna_inf_handler); } break;
        #else
        case GAPM_GET_DEV_ADV_TX_POWER:
        case GAPM_GET_SUGGESTED_DFLT_LE_DATA_LEN:
        case GAPM_GET_MAX_LE_DATA_LEN:
        case GAPM_GET_WLIST_SIZE:
        case GAPM_GET_PAL_SIZE:
        case GAPM_GET_RAL_SIZE:
        case GAPM_GET_NB_ADV_SETS:
        case GAPM_GET_MAX_LE_ADV_DATA_LEN:
        case GAPM_GET_DEV_TX_PWR:
        case GAPM_GET_DEV_RF_PATH_COMP:
        case GAPM_GET_ANTENNA_INFO: { status = GAP_ERR_NOT_SUPPORTED; } break;
        #endif // (BLE_HOST_PRESENT)
        #if (BT_HOST_PRESENT && HOST_TEST_MODE)
        case GAPM_BT_READ_LOOPBACK_MODE: { status = gapm_bt_read_loopback_mode(DEST_BF(p_cmd->operation, src_id), gapm_msg_bt_read_loopback_mode_res_handler); } break;
        #else
        case GAPM_BT_READ_LOOPBACK_MODE: { status = GAP_ERR_NOT_SUPPORTED; } break;
        #endif // (BT_HOST_PRESENT && HOST_TEST_MODE)
        default:  { /* nothing to do */ } break;
    };

    if ((send_cmd_cmp_evt) || (status != GAP_ERR_NO_ERROR))
    {
        gapm_msg_send_cmp_evt(operation, src_id, GAP_INVALID_ACTV_IDX, status);
    }

    return (msg_status);
}


/**
 ****************************************************************************************
 * @brief Handles request to add a new profile task.
 ****************************************************************************************
 */
__STATIC int gapm_profile_task_add_cmd_handler(ke_msg_id_t const msgid, const struct gapm_profile_task_add_cmd * p_cmd,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Returned message status
    uint8_t status = GAP_ERR_NOT_SUPPORTED;

    #if (HOST_PROFILES)

    uint16_t start_hdl = p_cmd->start_hdl;

    if(!gapm_env.configured)
    {
        status = GAP_ERR_COMMAND_DISALLOWED;
    }
    else
    {
        // request to add the profile
        status = prf_add_profile(p_cmd->prf_api_id, p_cmd->sec_lvl, p_cmd->user_prio, p_cmd->param, NULL, &start_hdl);
    }

    if(status == GAP_ERR_NO_ERROR)
    {
        struct gapm_profile_added_ind *ind;
        ke_task_id_t prf_task;
        // Set application task
        prf_task             = prf_msg_api_init(p_cmd->prf_api_id, src_id);

        #if(AHI_TL_SUPPORT)
        // for external application which AHI, update task Number from task ID
        if(KE_TYPE_GET(p_cmd->app_task) == TASK_ID_AHI)
        {
            prf_task = ahi_get_id_from_task(prf_task);
        }
        #endif // (AHI_TL_SUPPORT)

        // send an indication to inform that profile has been added
        ind = KE_MSG_ALLOC(GAPM_PROFILE_ADDED_IND, src_id, dest_id, gapm_profile_added_ind);
        ind->prf_task_id = p_cmd->prf_api_id;
        ind->prf_task_nb = prf_task;
        ind->start_hdl   = start_hdl;
        ke_msg_send(ind);
    }
    #endif // (HOST_PROFILES)

    //liuj del for reconnect problem
    //gapm_msg_send_cmp_evt(p_cmd->operation, src_id, GAP_INVALID_ACTV_IDX, status);

    return (KE_MSG_CONSUMED);
}

#if (BLE_HOST_PRESENT)
// USE encryption block - should use aes api on SW
__STATIC int gapm_use_enc_block_cmd_handler(ke_msg_id_t const msgid, struct gapm_use_enc_block_cmd * p_cmd,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = GAP_ERR_INVALID_PARAM;
    if(p_cmd->operation == GAPM_USE_ENC_BLOCK)
    {
        if(p_cmd->cipher)
        {
            status = gapm_aes_cipher(p_cmd->operand_1, p_cmd->operand_2, gapm_smp_use_enc_block_cb);
        }
        else
        {
            status = gapm_aes_decipher(p_cmd->operand_1, p_cmd->operand_2, gapm_smp_use_enc_block_cb);
        }
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, GAP_INVALID_ACTV_IDX, status);
    }
    else
    {
        gapm_env.aes_src_id = src_id;
    }

    return (KE_MSG_CONSUMED);
}

// Generating number with AES encryption block - should use aes api on SW
__STATIC int gapm_gen_rand_nb_cmd_handler(ke_msg_id_t const msgid, struct gapm_operation_cmd* p_cmd,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = GAP_ERR_INVALID_PARAM;
    if(p_cmd->operation == GAPM_GEN_RAND_NB)
    {
        status =  gapm_generate_random_number(gapm_smp_gen_rand_nb_cb);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, GAP_INVALID_ACTV_IDX, status);
    }
    else
    {
        gapm_env.aes_src_id = src_id;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles request of generating a random address command
 * - GAPM_GEN_RAND_ADDR:  Generate a random address
 ****************************************************************************************
 */
__STATIC int gapm_gen_rand_addr_cmd_handler(ke_msg_id_t const msgid, struct gapm_gen_rand_addr_cmd *p_cmd,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = GAP_ERR_INVALID_PARAM;
    if(p_cmd->operation == GAPM_GEN_RAND_ADDR)
    {
        status = gapm_generate_random_address(p_cmd->rnd_type, gapm_msg_gen_rand_addr_cmp_handler);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, GAP_INVALID_ACTV_IDX, status);
    }
    else
    {
        gapm_env.aes_src_id = src_id;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles request of read of Public Key
 * - GAPM_GET_PUB_KEY : Read Public Key
 ****************************************************************************************
 */
__STATIC int gapm_get_pub_key_cmd_handler(ke_msg_id_t const msgid, struct gapm_get_pub_key_cmd* p_cmd,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = GAP_ERR_INVALID_PARAM;
    if(p_cmd->operation == GAPM_GET_PUB_KEY)
    {
       status = gapm_get_public_key(DEST_BF(p_cmd->operation, src_id), gapm_msg_pub_key_res_cb);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, GAP_INVALID_ACTV_IDX, status);
    }

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles request of generating a DH Key command
 * - GAPM_GEN_DH_KEY :  Generate a DH Key
 ****************************************************************************************
 */
__STATIC int gapm_gen_dh_key_cmd_handler(ke_msg_id_t const msgid, struct gapm_gen_dh_key_cmd * p_cmd,
                                ke_task_id_t const dest_id, ke_task_id_t const src_id)
{

    uint16_t status = GAP_ERR_INVALID_PARAM;
    if(p_cmd->operation == GAPM_GEN_DH_KEY)
    {
        status = gapm_compute_dh_key(DEST_BF(p_cmd->operation, src_id), &(p_cmd->pub_key), gapm_msg_dh_key_res_cb);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, GAP_INVALID_ACTV_IDX, status);
    }

    return (KE_MSG_CONSUMED);
}
#endif // (BLE_HOST_PRESENT)

#if (HL_LE_CENTRAL || HL_LE_PERIPHERAL || BT_HOST_PRESENT)
/**
 ****************************************************************************************
 * @brief Handles request of generating OOB Data command
 ****************************************************************************************
 */
__STATIC int gapm_gen_oob_data_cmd_handler(ke_msg_id_t const msgid, struct gapm_gen_oob_data_cmd* p_cmd,
                                  ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;
    if(p_cmd->operation == GAPM_GEN_LE_OOB_DATA)
    {
        #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
        status = gapm_generate_le_oob_data(DEST_BF(p_cmd->operation, src_id), gapm_msg_le_oob_res_cb);
        #endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    }
    else if (p_cmd->operation == GAPM_GEN_BT_OOB_DATA)
    {
        #if (BT_HOST_PRESENT)
        status = gapm_generate_bt_oob_data(DEST_BF(p_cmd->operation, src_id), gapm_msg_bt_oob_res_cb);
        #endif // (BT_HOST_PRESENT)
    }
    else
    {
        status = GAP_ERR_INVALID_PARAM;
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, GAP_INVALID_ACTV_IDX, status);
    }

    return (KE_MSG_CONSUMED);
}
#endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL || BT_HOST_PRESENT)


/**
 ****************************************************************************************
 * @brief Handles request of creating a new activity (GAPM_ACTIVITY_CREATE_CMD message):
 *  - GAPM_CREATE_ADV_ACTIVITY: Create advertising activity
 *  - GAPM_CREATE_SCAN_ACTIVITY: Create scanning activity
 *  - GAPM_CREATE_INIT_ACTIVITY: Create initiating activity
 *  - GAPM_CREATE_PERIOD_SYNC_ACTIVITY: Create periodic synchronization activity
 ****************************************************************************************
 */
__STATIC int gapm_activity_create_cmd_handler(ke_msg_id_t const msgid,
                                     struct gapm_activity_create_cmd* p_cmd,
                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = GAP_ERR_INVALID_PARAM;
    uint8_t operation = p_cmd->operation;
    uint8_t actv_idx = GAP_INVALID_ACTV_IDX;
    uint8_t type = 0;

    // check operation
    switch(operation)
    {
        #if (BLE_HOST_PRESENT)
        case GAPM_CREATE_ADV_ACTIVITY:
        {
            struct gapm_activity_create_adv_cmd* p_adv_cmd = (struct gapm_activity_create_adv_cmd*) p_cmd;
            type = GAPM_ACTV_TYPE_ADV;
            switch(p_adv_cmd->type)
            {
                case GAPM_ADV_TYPE_LEGACY:
                {
                    status = gapm_adv_legacy_create(DEST_BF(p_cmd->operation, src_id), p_adv_cmd->own_addr_type, &(p_adv_cmd->adv_param),
                                                    &gapm_msg_adv_actv_cb);
                } break;
                case GAPM_ADV_TYPE_EXTENDED:
                {
                    status = gapm_adv_ext_create(DEST_BF(p_cmd->operation, src_id), p_adv_cmd->own_addr_type, &(p_adv_cmd->adv_param),
                                                 &(p_adv_cmd->second_cfg), &gapm_msg_adv_actv_cb);
                } break;
                case GAPM_ADV_TYPE_PERIODIC:
                {
                    if(p_adv_cmd->cte_cfg.count == 0)
                    {
                        status = gapm_adv_periodic_create(DEST_BF(p_cmd->operation, src_id), p_adv_cmd->own_addr_type, &(p_adv_cmd->adv_param),
                                                          &(p_adv_cmd->second_cfg), &(p_adv_cmd->period_cfg),
                                                          &gapm_msg_adv_actv_cb);
                    }
                    else
                    {
                        status = gapm_adv_periodic_with_cte_create(DEST_BF(p_cmd->operation, src_id), p_adv_cmd->own_addr_type, &(p_adv_cmd->adv_param),
                                                                   &(p_adv_cmd->second_cfg), &(p_adv_cmd->period_cfg),
                                                                   &(p_adv_cmd->cte_cfg), p_adv_cmd->switching_pattern_len,
                                                                   p_adv_cmd->antenna_id, &gapm_msg_adv_actv_cb);
                    }
                } break;

                default: { /* nothing to do */ } break;
            }

        } break;
        case GAPM_CREATE_SCAN_ACTIVITY:
        {
            type = GAPM_ACTV_TYPE_SCAN;
            status = gapm_scan_create(DEST_BF(p_cmd->operation, src_id), p_cmd->own_addr_type, &gapm_msg_scan_actv_cb, &actv_idx);
        } break;
        case GAPM_CREATE_PERIOD_SYNC_ACTIVITY:
        {
            type = GAPM_ACTV_TYPE_PER_SYNC;
            status = gapm_per_sync_create(DEST_BF(p_cmd->operation, src_id), &gapm_msg_per_sync_actv_cb, &actv_idx);
        } break;
        case GAPM_CREATE_INIT_ACTIVITY:
        {
            type = GAPM_ACTV_TYPE_INIT;
            status = gapm_init_create(DEST_BF(p_cmd->operation, src_id), p_cmd->own_addr_type, &gapm_msg_init_actv_cb, &actv_idx);
        } break;
        #endif // (BLE_HOST_PRESENT)
        #if(BT_HOST_PRESENT)
        case GAPM_CREATE_INQUIRY_ACTIVITY:
        {
            type = GAPM_ACTV_TYPE_INQUIRY;
            status = gapm_inquiry_create(DEST_BF(p_cmd->operation, src_id), &gapm_msg_inquiry_actv_cb, &actv_idx);
        } break;
        case GAPM_CREATE_INQUIRY_SCAN_ACTIVITY:
        {
            type = GAPM_ACTV_TYPE_INQUIRY_SCAN;
            status = gapm_inquiry_scan_create(DEST_BF(p_cmd->operation, src_id), &gapm_msg_inquiry_scan_actv_cb, &actv_idx);
        } break;
        case GAPM_CREATE_PAGE_ACTIVITY:
        {
            type = GAPM_ACTV_TYPE_PAGE;
            status = gapm_page_create(DEST_BF(p_cmd->operation, src_id), &gapm_msg_page_actv_cb, &actv_idx);
        } break;
        case GAPM_CREATE_PAGE_SCAN_ACTIVITY:
        {
            type = GAPM_ACTV_TYPE_PAGE_SCAN;
            status = gapm_page_scan_create(DEST_BF(p_cmd->operation, src_id), &gapm_msg_page_scan_actv_cb, &actv_idx);
        } break;
        #endif // (BT_HOST_PRESENT)
        default:  { /* nothing to do */ } break;
    };

    if (status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(operation, src_id, GAP_INVALID_ACTV_IDX, status);
    }
    else
    {
        if(operation != GAPM_CREATE_ADV_ACTIVITY)
        {
            struct gapm_activity_created_ind* p_ind = KE_MSG_ALLOC(GAPM_ACTIVITY_CREATED_IND, src_id,
                                                                   TASK_GAPM, gapm_activity_created_ind);
            if(p_ind)
            {
                p_ind->actv_idx  = actv_idx;
                p_ind->actv_type = type;
                p_ind->tx_pwr    = 0;
                ke_msg_send(p_ind);
            }
            gapm_msg_send_cmp_evt(operation, src_id, actv_idx, status);

            #if(HL_LE_OBSERVER)
            if(   (p_cmd->own_addr_type != GAPM_STATIC_ADDR)
               && ((operation == GAPM_CREATE_INIT_ACTIVITY) || (operation == GAPM_CREATE_SCAN_ACTIVITY)))
            {
                if(gapm_le_actv_addr_is_scan_init_addr_generated())
                {
                    gapm_msg_actv_addr_updated(DEST_BF(p_cmd->operation, src_id), actv_idx, &(gapm_env.scan_init_rand_addr));
                }
            }
            #endif // (HL_LE_OBSERVER)
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles request of starting an activity (GAPM_ACTIVITY_START_CMD message):
 *  - GAPM_START_ACTIVITY: Start an activity
 ****************************************************************************************
 */
__STATIC int gapm_activity_start_cmd_handler(ke_msg_id_t const msgid, struct gapm_activity_start_cmd *p_cmd,
                                    ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = GAP_ERR_INVALID_PARAM;
    if(p_cmd->operation == GAPM_START_ACTIVITY)
    {
        gapm_actv_t* p_actv = gapm_actv_get(p_cmd->actv_idx);
        if(p_actv)
        {
            switch(p_actv->type)
            {
                #if (BLE_HOST_PRESENT)
                case GAPM_ACTV_TYPE_ADV:          { status = gapm_adv_start(p_cmd->actv_idx, (const gapm_adv_param_t*) p_cmd->u_param);             } break;
                case GAPM_ACTV_TYPE_SCAN:         { status = gapm_scan_start(p_cmd->actv_idx, (const gapm_scan_param_t*) p_cmd->u_param);           } break;
                case GAPM_ACTV_TYPE_INIT:         { status = gapm_init_start(p_cmd->actv_idx, (const gapm_init_param_t*) p_cmd->u_param);           } break;
                case GAPM_ACTV_TYPE_PER_SYNC:     { status = gapm_per_sync_start(p_cmd->actv_idx, (const gapm_per_sync_param_t*) p_cmd->u_param);   } break;
                #endif // (BLE_HOST_PRESENT)
                #if (BT_HOST_PRESENT)
                case GAPM_ACTV_TYPE_INQUIRY:      { status = gapm_inquiry_start(p_cmd->actv_idx, (const gapm_inquiry_param_t*) p_cmd->u_param);     } break;
                case GAPM_ACTV_TYPE_PAGE:         { status = gapm_page_start(p_cmd->actv_idx, (const gapm_page_param_t*) p_cmd->u_param);           } break;
                case GAPM_ACTV_TYPE_PAGE_SCAN:    { status = gapm_page_scan_start(p_cmd->actv_idx, (const gapm_page_scan_param_t*) p_cmd->u_param); } break;
                case GAPM_ACTV_TYPE_INQUIRY_SCAN:
                {
                    co_buf_t* p_eir_data = NULL;
                    const gapm_inquiry_scan_start_param_t* p_cmd_param = (const gapm_inquiry_scan_start_param_t*) p_cmd->u_param;
                    const gapm_inquiry_scan_param_t* p_scan_param =  &(p_cmd_param->scan);
                    uint8_t eir_length = p_cmd_param->eir_length;
                    const uint8_t* p_data = &(p_cmd_param->eir_data[0]);

                    if(eir_length > 0)
                    {
                        // alloc buffer for EIR data
                        if(co_buf_alloc(&p_eir_data, 0, eir_length, 0) != CO_BUF_ERR_NO_ERROR)
                        {
                            status = GAP_ERR_INSUFF_RESOURCES;
                            break;
                        }
                        memcpy(co_buf_data(p_eir_data), p_data, eir_length);
                    }

                    status = gapm_inquiry_scan_start(p_cmd->actv_idx, p_scan_param, p_eir_data);
                    if(eir_length > 0)
                    {
                        co_buf_release(p_eir_data);
                    }
                } break;
                #endif // (BT_HOST_PRESENT)

                default:                      { /* Nothing to do */ } break;
            }
        }
        else
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
        }
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, p_cmd->actv_idx, status);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles request of stopping one or all activities (GAPM_ACTIVITY_STOP_CMD message):
 *  - GAPM_STOP_ACTIVITY: Stop an activity
 *  - GAPM_STOP_ALL_ACTIVITIES: Stop all activities
 ****************************************************************************************
 */
__STATIC int gapm_activity_stop_cmd_handler(ke_msg_id_t const msgid,
                                   struct gapm_activity_stop_cmd *p_cmd,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = GAP_ERR_INVALID_PARAM;
    if(p_cmd->operation == GAPM_STOP_ACTIVITY)
    {
       status = gapm_actv_stop(p_cmd->actv_idx);
    }
    else if (p_cmd->operation == GAPM_STOP_ALL_ACTIVITIES)
    {
       status = gapm_actv_stop_all(DEST_BF(p_cmd->operation, src_id), gapm_msg_default_cmp_evt_handler);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, p_cmd->actv_idx, status);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles request of deleting one or all activities (GAPM_ACTIVITY_DELETE_CMD message)
 *  - GAPM_DELETE_ACTIVITY: Delete an activity
 *  - GAPM_DELETE_ALL_ACTIVITIES: Delete all activities
 ****************************************************************************************
 */
__STATIC int gapm_activity_delete_cmd_handler(ke_msg_id_t const msgid,
                                     struct gapm_activity_delete_cmd *p_cmd,
                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = GAP_ERR_INVALID_PARAM;
    if(p_cmd->operation == GAPM_DELETE_ACTIVITY)
    {
       status = gapm_actv_delete(p_cmd->actv_idx);
    }
    else if (p_cmd->operation == GAPM_DELETE_ALL_ACTIVITIES)
    {
       status = gapm_actv_delete_all(DEST_BF(p_cmd->operation, src_id), gapm_msg_default_cmp_evt_handler);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, p_cmd->actv_idx, status);
    }

    return (KE_MSG_CONSUMED);
}

#if (BLE_HOST_PRESENT)
#if (HL_LE_BROADCASTER)
/**
 ****************************************************************************************
 * @brief Handles Set Advertising data
 ****************************************************************************************
 */
__STATIC int gapm_set_adv_data_cmd_handler(ke_msg_id_t const msgid, struct gapm_set_adv_data_cmd const *p_cmd,
                                  ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    co_buf_t* p_data = NULL;

    if(co_buf_alloc(&p_data, 0, p_cmd->length, 0) == CO_BUF_ERR_NO_ERROR)
    {
        memcpy(co_buf_data(p_data), p_cmd->data, p_cmd->length);

        switch(p_cmd->operation)
        {
            case GAPM_SET_ADV_DATA:         { status = gapm_adv_set_data(p_cmd->actv_idx, p_data);        } break;
            case GAPM_SET_SCAN_RSP_DATA:    { status = gapm_adv_set_scan_rsp(p_cmd->actv_idx, p_data);    } break;
            case GAPM_SET_PERIOD_ADV_DATA:  { status = gapm_adv_set_period_data(p_cmd->actv_idx, p_data); } break;
            default:                        { status = GAP_ERR_INVALID_PARAM;                             } break;
        }

        co_buf_release(p_data);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, p_cmd->actv_idx, status);
    }

    return (KE_MSG_CONSUMED);
}

#if (BLE_AOD | BLE_AOA)
/**
 ****************************************************************************************
 * @brief Handles request of controlling CTE transmission
 ****************************************************************************************
 */
__STATIC int gapm_per_adv_cte_tx_ctl_cmd_handler(ke_msg_id_t const msgid, struct gapm_per_adv_cte_tx_ctl_cmd *p_cmd,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = GAP_ERR_INVALID_PARAM;
    if(p_cmd->operation == GAPM_PER_ADV_CTE_TX_CTL)
    {
       status = gapm_adv_periodic_cte_tx_ctl(p_cmd->actv_idx, p_cmd->enable);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, p_cmd->actv_idx, status);
    }

    return (KE_MSG_CONSUMED);
}
#endif // (BLE_AOD | BLE_AOA)
#endif // (HL_LE_BROADCASTER)

#if (HL_LE_OBSERVER)
/**
 ****************************************************************************************
 * @brief Handles control periodic advertising report
 ****************************************************************************************
 */
__STATIC int gapm_per_adv_report_ctrl_cmd_handler(ke_msg_id_t const msgid, struct gapm_per_adv_report_ctrl_cmd* p_cmd,
                                         ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = GAP_ERR_INVALID_PARAM;
    if(p_cmd->operation == GAPM_PER_ADV_REPORT_CTRL)
    {
       status = gapm_per_sync_report_ctrl(p_cmd->actv_idx, p_cmd->report_en_bf);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, p_cmd->actv_idx, status);
    }

    return (KE_MSG_CONSUMED);
}


#if (BLE_AOD | BLE_AOA)
/**
 ****************************************************************************************
 * @brief Handles control of IQ sampling
 ****************************************************************************************
 */
__STATIC int gapm_per_sync_iq_sampling_ctrl_cmd_handler(ke_msg_id_t const msgid, struct gapm_per_sync_iq_sampling_ctrl_cmd* p_cmd,
                                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = GAP_ERR_INVALID_PARAM;
    if(p_cmd->operation == GAPM_PER_SYNC_IQ_SAMPLING_CTRL)
    {
       status = gapm_per_sync_iq_report_ctrl(p_cmd->actv_idx, p_cmd->enable, p_cmd->slot_dur, p_cmd->max_sampl_cte,
                                             p_cmd->switching_pattern_len, &p_cmd->antenna_id[0]);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, p_cmd->actv_idx, status);
    }

    return (KE_MSG_CONSUMED);
}
#endif // (BLE_AOD | BLE_AOA)
#endif // (HL_LE_OBSERVER)


/**
 ****************************************************************************************
 * @brief Handles request of modifying local channel map:
 *  - GAPM_SET_LE_CHANNEL_MAP:  Set device channel map
 ****************************************************************************************
 */
__STATIC int gapm_set_channel_assessment_cmd_handler(ke_msg_id_t const msgid, const struct gapm_ch_assess_cmd *p_cmd,
                                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t status = GAP_ERR_INVALID_PARAM;

    if(p_cmd->operation == GAPM_CH_ASSESS_START)
    {
        status = gapm_start_channel_assessment(DEST_BF(p_cmd->operation, src_id), p_cmd->scan_win_duration, p_cmd->scan_duration_min,
                                               p_cmd->scan_duration_max, p_cmd->intv, gapm_msg_default_cmp_evt_handler);
    }
    else if (p_cmd->operation == GAPM_CH_ASSESS_STOP)
    {
        status = gapm_stop_channel_assessment(DEST_BF(p_cmd->operation, src_id), gapm_msg_default_cmp_evt_handler);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        gapm_msg_send_cmp_evt(p_cmd->operation, src_id, GAP_INVALID_ACTV_IDX, status);
    }

    return (KE_MSG_CONSUMED);
}
#endif // (BLE_HOST_PRESENT)


/// @brief Default message handler
__STATIC int gapm_default_msg_handler(ke_msg_id_t const msgid, void *event,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Check if it's a GAPM message and it's not sent to itself
    if (MSG_T(msgid) == TASK_ID_GAPM && (dest_id != src_id))
    {
        // prepare unknown message indication
        struct gapm_unknown_msg_ind* p_ind = KE_MSG_ALLOC(GAPM_UNKNOWN_MSG_IND,
                src_id, dest_id, gapm_unknown_msg_ind);

        p_ind->unknown_msg_id = msgid;

        // send event
        ke_msg_send(p_ind);
    }
    return (KE_MSG_CONSUMED);
}

#if(RW_DEBUG && BLE_HOST_PRESENT)
/// Parameters of #GAPM_DBG_SECURITY_TEST_CMD
typedef struct gapm_dbg_security_test_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    uint8_t  operation;
    /// Parameter array (32-bit aligned)
    uint32_t params[__ARRAY_EMPTY];
} gapm_dbg_security_test_cmd_t;

/// Parameters of #GAPM_DBG_SECURITY_TEST_IND
typedef struct gapm_dbg_security_test_ind
{
    /// Result array (32-bit aligned)
    uint32_t result[1]; // variable size
} gapm_dbg_security_test_ind_t;


/// Callback executed to return result
typedef void (*security_test_result_cb)(uint16_t result_size, const uint8_t* p_result);

extern bool security_dbg_test_execute(const void* p_params, security_test_result_cb cb_result);

void gapm_dbg_security_test_result_handler(uint16_t result_size, const uint8_t* p_result)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    // Send the generated random number to the requester task.
    gapm_dbg_security_test_ind_t *p_ind = KE_MSG_ALLOC_DYN_T(GAPM_DBG_SECURITY_TEST_IND, gapm_env.aes_src_id, TASK_GAPM, gapm_dbg_security_test_ind_t, result_size);
    if(p_ind)
    {
        memcpy(&p_ind->result, p_result, result_size);
        ke_msg_send(p_ind);
    }
    else
    {
        status = GAP_ERR_INSUFF_RESOURCES;
    }

    gapm_msg_send_cmp_evt(GAPM_DBG_SECURITY_TEST, gapm_env.aes_src_id, GAP_INVALID_ACTV_IDX, status);
}


/// Security test message handler
__STATIC int gapm_dbg_security_test_cmd_handler(ke_msg_id_t const msgid,  const gapm_dbg_security_test_cmd_t * p_param,
                                                ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    gapm_env.aes_src_id = src_id;
    if(!security_dbg_test_execute(p_param->params, gapm_dbg_security_test_result_handler))
    {
        gapm_msg_send_cmp_evt(p_param->operation, src_id, GAP_INVALID_ACTV_IDX, GAP_ERR_NOT_SUPPORTED);
    }

    return (KE_MSG_CONSUMED);
}

#endif // (RW_DEBUG && BLE_HOST_PRESENT)

/// @brief Message to an Unknown task received
__STATIC int gapm_unknown_task_msg_handler(ke_msg_id_t const msgid,  void * param,
                                          ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct ke_msg* msg = ke_param2msg(param);

    // inform main application that a message to an unknown task has been requested
    struct gapm_unknown_task_ind * ind = KE_MSG_ALLOC(GAPM_UNKNOWN_TASK_IND, APP_MAIN_TASK, dest_id, gapm_unknown_task_ind);
    ind->msg_id = msg->param_len;
    ind->task_id = src_id;
    ke_msg_send(ind);

    return (KE_MSG_CONSUMED);
}


/*
 * TASK VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// The default message handlers
KE_MSG_HANDLER_TAB(gapm)
{
    // Note: all messages must be sorted in ID ascending order

    /* Reset command */
    { GAPM_RESET_CMD,                                 (ke_msg_func_t) gapm_reset_cmd_handler                                 },
    { GAPM_SET_DEV_CONFIG_CMD,                        (ke_msg_func_t) gapm_set_dev_config_cmd_handler                        },
    #if (BLE_HOST_PRESENT)
    { GAPM_SET_LE_CHANNEL_MAP_CMD,                    (ke_msg_func_t) gapm_set_le_channel_map_cmd_handler                    },
    { GAPM_SET_IRK_CMD,                               (ke_msg_func_t) gapm_set_irk_cmd_handler                               },
    #endif // (BLE_HOST_PRESENT)
    { GAPM_SET_NAME_CMD,                              (ke_msg_func_t) gapm_set_name_cmd_handler                              },
    #if (BT_HOST_PRESENT)
    { GAPM_SET_SDP_DEVICE_IDENTIFICATION_RECORD_CMD,  (ke_msg_func_t) gapm_set_sdp_device_identification_record_cmd_handler  },
    #endif // (BT_HOST_PRESENT)
    { GAPM_GET_DEV_INFO_CMD ,                         (ke_msg_func_t) gapm_get_dev_info_cmd_handler                          },

    #if (BLE_HOST_PRESENT)
    /* Address resolution */
    { GAPM_RESOLV_ADDR_CMD,                           (ke_msg_func_t) gapm_resolv_addr_cmd_handler                           },
    { GAPM_GEN_RAND_ADDR_CMD,                         (ke_msg_func_t) gapm_gen_rand_addr_cmd_handler                         },
    { GAPM_USE_ENC_BLOCK_CMD,                         (ke_msg_func_t) gapm_use_enc_block_cmd_handler                         },
    { GAPM_GEN_RAND_NB_CMD,                           (ke_msg_func_t) gapm_gen_rand_nb_cmd_handler                           },
    { GAPM_GEN_DH_KEY_CMD,                            (ke_msg_func_t) gapm_gen_dh_key_cmd_handler                            },
    { GAPM_GET_PUB_KEY_CMD,                           (ke_msg_func_t) gapm_get_pub_key_cmd_handler                           },
    #endif // (BLE_HOST_PRESENT)
    #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL || BT_HOST_PRESENT)
    { GAPM_GEN_OOB_DATA_CMD,                          (ke_msg_func_t) gapm_gen_oob_data_cmd_handler                          },
    #endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL || BT_HOST_PRESENT)

    #if (BLE_HOST_PRESENT)
    /* List Management */
    { GAPM_GET_RAL_ADDR_CMD,                          (ke_msg_func_t) gapm_get_ral_addr_cmd_handler                          },
    { GAPM_LIST_SET_CMD,                              (ke_msg_func_t) gapm_list_set_cmd_handler                              },
    #endif // (BLE_HOST_PRESENT)

    /* Extended Air Operations */
    { GAPM_ACTIVITY_CREATE_CMD,                       (ke_msg_func_t) gapm_activity_create_cmd_handler                       },
    { GAPM_ACTIVITY_START_CMD,                        (ke_msg_func_t) gapm_activity_start_cmd_handler                        },
    { GAPM_ACTIVITY_STOP_CMD,                         (ke_msg_func_t) gapm_activity_stop_cmd_handler                         },
    { GAPM_ACTIVITY_DELETE_CMD,                       (ke_msg_func_t) gapm_activity_delete_cmd_handler                       },
    #if (HL_LE_BROADCASTER)
    { GAPM_SET_ADV_DATA_CMD,                          (ke_msg_func_t) gapm_set_adv_data_cmd_handler                          },
    #endif //(HL_LE_BROADCASTER)

    #if (HL_LE_OBSERVER)
    { GAPM_PER_ADV_REPORT_CTRL_CMD,                   (ke_msg_func_t) gapm_per_adv_report_ctrl_cmd_handler                   },
    #if (BLE_AOD | BLE_AOA)
    { GAPM_PER_SYNC_IQ_SAMPLING_CTRL_CMD,             (ke_msg_func_t) gapm_per_sync_iq_sampling_ctrl_cmd_handler             },
    #endif // (BLE_AOD | BLE_AOA)
    #endif //(HL_LE_OBSERVER)

    /* Extended Air Operations */
    #if (HL_LE_BROADCASTER)
    #if (BLE_AOD | BLE_AOA)
    { GAPM_PER_ADV_CTE_TX_CTL_CMD,                    (ke_msg_func_t) gapm_per_adv_cte_tx_ctl_cmd_handler                    },
    #endif // (BLE_AOD | BLE_AOA)
    #endif //(HL_LE_BROADCASTER)

    #if (BLE_HOST_PRESENT && HOST_TEST_MODE)
    /* Test Mode control */
    { GAPM_LE_TEST_MODE_CTRL_CMD,                     (ke_msg_func_t) gapm_le_test_mode_ctrl_cmd_handler                     },
    #endif // (BLE_HOST_PRESENT && HOST_TEST_MODE)

    #if (BT_HOST_PRESENT && HOST_TEST_MODE)
    { GAPM_BT_WRITE_LOOPBACK_MODE_CMD,                (ke_msg_func_t) gapm_bt_write_loopback_mode_cmd_handler                },
    { GAPM_BT_ENABLE_DEVICE_UNDER_TEST_MODE_CMD,      (ke_msg_func_t) gapm_bt_enable_device_under_test_mode_cmd_handler      },
    { GAPM_BT_WRITE_SIMPLE_PAIRING_DEBUG_MODE_CMD,    (ke_msg_func_t) gapm_bt_write_simple_pairing_debug_mode_cmd_handler    },
    { GAPM_BT_WRITE_SECURE_CONNECTIONS_TEST_MODE_CMD, (ke_msg_func_t) gapm_bt_write_secure_connections_test_mode_cmd_handler },
    #endif // (BT_HOST_PRESENT && HOST_TEST_MODE)

    #if (BLE_HOST_PRESENT)
    /* Channel Assessment control */
    {GAPM_SET_CH_ASSESS_CMD,                          (ke_msg_func_t) gapm_set_channel_assessment_cmd_handler                },
    #endif // (BT_HOST_PRESENT)

    /* Profile Management */
    { GAPM_PROFILE_TASK_ADD_CMD,                      (ke_msg_func_t) gapm_profile_task_add_cmd_handler                      },

    #if (BLE_HOST_PRESENT)
    /* I/Q Sample generator */
    { GAPM_DBG_IQGEN_CFG_CMD,                         (ke_msg_func_t) gapm_dbg_iqgen_cfg_cmd_handler                         },
    #endif // (BLE_HOST_PRESENT)

    #if(RW_DEBUG && BLE_HOST_PRESENT)
    /* Security test */
    { GAPM_DBG_SECURITY_TEST_CMD,                     (ke_msg_func_t) gapm_dbg_security_test_cmd_handler                     },
    #endif // (RW_DEBUG && BLE_HOST_PRESENT)

    { GAPM_UNKNOWN_TASK_MSG,                          (ke_msg_func_t) gapm_unknown_task_msg_handler                          },

    { KE_MSG_DEFAULT_HANDLER,                         (ke_msg_func_t) gapm_default_msg_handler                               },
};

/// GATT task instance.
__STATIC ke_state_t gapm_state[GAPM_IDX_MAX];

/// GAP Manager task descriptor
__STATIC const struct ke_task_desc TASK_DESC_GAPM = {gapm_msg_handler_tab, gapm_state, GAPM_IDX_MAX, ARRAY_LEN(gapm_msg_handler_tab)};


/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */
void gapm_msg_initialize(uint8_t init_type)
{
    switch (init_type)
    {
        case RWIP_INIT:
        {
            ke_task_create(TASK_GAPM, &TASK_DESC_GAPM);
        } break;

        case RWIP_RST:
        case RWIP_1ST_RST:
        {
            if(gapm_msg_env.p_name)
            {
                ke_free(gapm_msg_env.p_name);
            }
            if(gapm_msg_env.p_new_name)
            {
                ke_free(gapm_msg_env.p_new_name);
            }

            if(gapm_msg_env.p_temp_param != NULL)
            {
                ke_free(gapm_msg_env.p_temp_param);
            }
        }
        // no break
        default: { /* do nothing*/ } break;
    }
    memset(&(gapm_msg_env), 0, sizeof(gapm_msg_env));
}
#endif // (HOST_MSG_API)

/// @} GAPM

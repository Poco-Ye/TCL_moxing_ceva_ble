/**
 ****************************************************************************************
 *
 * @file gapc_le_con.c
 *
 * @brief Generic Access Profile Controller Implementation - LE Connection data
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_GAPC)
#include "gapc_int.h"
#include "gapc_le_con.h"
#include "co_math.h"
#include "ke_mem.h"
#include <string.h>

#include "l2cap.h"
#include "hl_hci.h"
#include "../../inc/l2cap_hl_api.h"
#include "../../inc/gatt_hl_api.h"
#include "../../inc/prf_hl_api.h"
#include "../gap_int.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * MACROS
 ****************************************************************************************
 */


/*
 * LOCAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */


/*
 * PROCEDURE STATE MACHINE
 ****************************************************************************************
 */

// ------------------ GET LE PEER FEATURES -------------------


/// Get LE Peer features procedure is granted
__STATIC uint16_t gapc_get_le_peer_features_proc_granted(uint8_t conidx, gapc_proc_info_t* p_proc)
{
    uint16_t status = HL_HCI_BASIC_CMD_SEND_WITH_CONHDL(HCI_LE_RD_REM_FEATS_CMD_OPCODE, gapc_get_conhdl(conidx),
                                                        HL_PROC_CONTINUE, gapc_proc_default_hci_stat_evt_handler)
    return (status);
}

/// Get LE Peer features procedure is finished
__STATIC void gapc_get_le_peer_features_proc_finished(uint8_t conidx, gapc_proc_info_t* p_proc, uint16_t status)
{
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);
    gapc_get_le_peer_features_cmp_cb cmp_cb = (gapc_get_le_peer_features_cmp_cb) p_proc->hdr.cmp_cb;
    const uint8_t* p_features = NULL;

    if(status == GAP_ERR_NO_ERROR)
    {
        // procedure finished is called within hci complete event handler
        const struct hci_le_rd_rem_feats_cmd_cmp_evt* p_evt = (const struct hci_le_rd_rem_feats_cmd_cmp_evt*) p_proc->p_res_info;
        p_features = (const uint8_t*)p_evt->le_feats.feats;


        //save peer feature in the environment
        memcpy(p_con->peer_features, p_evt->le_feats.feats, LE_FEATS_LEN);
        status = GAP_ERR_NO_ERROR;
    }
    else
    {
        #if (HL_LE_PERIPHERAL)
        if(GETB(p_con->hdr.info_bf, GAPC_ROLE) == ROLE_SLAVE)
        {
            switch(status)
            {
                case LL_ERR_UNKNOWN_HCI_COMMAND:
                case LL_ERR_COMMAND_DISALLOWED:
                case LL_ERR_UNSUPPORTED:
                case LL_ERR_UNKNOWN_LMP_PDU:
                case LL_ERR_UNSUPPORTED_REMOTE_FEATURE:
                {
                    // Clear parameter request feature in the environment because not supported by peer
                    p_con->peer_features[0] &= ~CO_BIT(GAP_LE_FEAT_CON_PARAM_REQ_PROC);
                }
                break;
                default: /* Nothing to do */ break;
            }
        }
        #endif // (HL_LE_PERIPHERAL)
    }

    cmp_cb(conidx, p_proc->hdr.dummy, status, p_features);
}


/// Get LE Peer features procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_get_le_peer_features_proc_itf =
{
    .granted  = (gapc_proc_simple_granted_cb)  gapc_get_le_peer_features_proc_granted,
    .finished = (gapc_proc_simple_finished_cb) gapc_get_le_peer_features_proc_finished,
};


// ------------------ GET LE CHANNEL MAP -------------------

/// Get LE channel map procedure is granted
__STATIC uint16_t gapc_get_le_chmap_proc_granted(uint8_t conidx, gapc_proc_info_t* p_proc)
{
    uint16_t status = HL_HCI_BASIC_CMD_SEND_WITH_CONHDL(HCI_LE_RD_CHNL_MAP_CMD_OPCODE, gapc_get_conhdl(conidx),
                                                        HL_PROC_FINISHED, gapc_proc_info_default_hci_cmp_evt_handler)
    return (status);
}

/// Get LE channel map procedure is finished
__STATIC void gapc_get_le_chmap_proc_finished(uint8_t conidx, gapc_proc_info_t* p_proc, uint16_t status)
{
    gapc_get_le_chmap_cmp_cb cmp_cb = (gapc_get_le_chmap_cmp_cb) p_proc->hdr.cmp_cb;
    const le_chnl_map_t* p_ch_map = NULL;
    if(status == GAP_ERR_NO_ERROR)
    {
        // procedure finished is called within hci complete event handler
        const struct hci_le_rd_chnl_map_cmd_cmp_evt* p_evt = (const struct hci_le_rd_chnl_map_cmd_cmp_evt*) p_proc->p_res_info;
        p_ch_map = (const le_chnl_map_t*) p_evt->ch_map.map;
    }

    cmp_cb(conidx, p_proc->hdr.dummy, status, p_ch_map);
}


/// Get LE channel map procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_get_le_chmap_proc_itf =
{
    .granted  = (gapc_proc_simple_granted_cb)  gapc_get_le_chmap_proc_granted,
    .finished = (gapc_proc_simple_finished_cb) gapc_get_le_chmap_proc_finished,
};

/*
 * HCI HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handle reception of HCI LE Channel Selection Algorithm event.
 *
 * @param[in] evt_code  HCI code:
 *                          - HCI Event Code for general HCI Events
 *                          - HCI LE Event sub-code for general HCI LE Meta Events
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
void gapc_hci_le_ch_sel_algo_evt_handler(uint8_t evt_code, struct hci_le_ch_sel_algo_evt const *p_evt)
{
    uint8_t conidx = gapc_get_conidx(p_evt->conhdl);
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);

    if (p_con != NULL)
    {
        p_con->chan_sel_algo = p_evt->ch_sel_algo;
    }
}

/// @brief Handles Read version command completed event
void gapc_hci_le_rd_rem_feats_cmp_evt_handler(uint8_t evt_code, struct hci_le_rd_rem_feats_cmd_cmp_evt const *p_evt)
{
    gapc_proc_info_default_hci_le_cmp_evt_handler(HL_PROC_FINISHED,
                                                  (const struct hci_basic_conhdl_le_cmd_cmp_evt*) p_evt);
}

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */
uint16_t gapc_le_con_create(uint8_t actv_idx, uint32_t dummy, bool is_name_discovery,
                            const gap_bdaddr_t* p_local_addr, const struct hci_le_enh_con_cmp_evt* p_evt,
                            uint8_t* p_conidx)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    // find an available connection index
    uint16_t conidx = gapc_avail_conidx_find();

    do
    {
        gapc_le_con_t* p_con;
        // Check if RPA is provided, otherwise use public identity
        uint8_t empty_addr[BD_ADDR_LEN] = {0,0,0,0,0,0};

        gap_le_con_param_t con_param =
        {
            .interval = p_evt->con_interval,
            .latency  = p_evt->con_latency,
            .sup_to   = p_evt->sup_to,
        };

        gap_bdaddr_t peer_addr;

        if(conidx == GAP_INVALID_CONIDX) break;

        // allocate environment variable for current GAP controller.
        p_con = (gapc_le_con_t*) ke_malloc_user(sizeof(gapc_le_con_t), KE_MEM_ENV);
        if(p_con == NULL) break;

        // clear environment data.
        memset(p_con, 0, sizeof(gapc_le_con_t));
        gapc_env.p_con[conidx] = &(p_con->hdr);

        #if(BLE_L2CAP)
        // Inform L2CAP about new connection
        status = l2cap_create(conidx, true);
        if(status != GAP_ERR_NO_ERROR) break;
        #endif // (BLE_L2CAP)

        #if (BLE_GATT)
        // Inform GATT about new connection
        status = gatt_create(conidx);
        if(status != GAP_ERR_NO_ERROR) break;
        #endif // (BLE_GATT)

        // Prepare SMP for connection
        status = gapc_le_smp_create(p_con, conidx);
        if(status != GAP_ERR_NO_ERROR) break;

        // Set the default value of the Repeated Attempt timer
        p_con->smp.rep_att_timer_val = GAPC_LE_SMP_REP_ATTEMPTS_TIMER_DEF_VAL_MS;

        // Set the default value of device features
        memset(p_con->peer_features, 0, GAP_LE_FEATS_LEN);
        p_con->peer_features[0] = CO_BIT(GAP_LE_FEAT_ENC)
                                | CO_BIT(GAP_LE_FEAT_CON_PARAM_REQ_PROC)
                                | CO_BIT(GAP_LE_FEAT_EXT_REJ_IND)
                                | CO_BIT(GAP_LE_FEAT_SLAVE_INIT_FEAT_EXCHG)
                                | CO_BIT(GAP_LE_FEAT_PING);

        // set specific connection parameters
        p_con->hdr.conhdl = p_evt->conhdl;
        p_con->hdr.conidx = conidx;
        SETB(p_con->hdr.info_bf, GAPC_ROLE, p_evt->role);
        SETB(p_con->hdr.info_bf, GAPC_LE_CON_TYPE, true);
        SETB(p_con->hdr.info_bf, GAPC_IS_NAME_DISCOVERY, is_name_discovery);

        // Retrieve peer address
        memcpy(&(peer_addr.addr), &(p_evt->peer_addr), sizeof(gap_addr_t));
        peer_addr.addr_type = p_evt->peer_addr_type & ADDR_MASK;

        if (memcmp(&(p_evt->peer_rslv_priv_addr), empty_addr, sizeof(gap_addr_t)))
        {
            memcpy(&(p_con->src[GAPC_INFO_SRC_PEER].addr), &(p_evt->peer_rslv_priv_addr), sizeof(gap_addr_t));
            p_con->src[GAPC_INFO_SRC_PEER].addr_type = ADDR_RAND;
        }
        else
        {
            p_con->src[GAPC_INFO_SRC_PEER] = peer_addr;
        }

        // keep local address information used to create the link
        if (memcmp(&(p_evt->loc_rslv_priv_addr), empty_addr, sizeof(gap_addr_t)))
        {
            memcpy(&(p_con->src[GAPC_INFO_SRC_LOCAL].addr), &(p_evt->loc_rslv_priv_addr), sizeof(gap_addr_t));
            p_con->src[GAPC_INFO_SRC_LOCAL].addr_type = ADDR_RAND;
        }
        else
        {
            p_con->src[GAPC_INFO_SRC_LOCAL] = *p_local_addr;
        }

        p_con->hdr.dummy = dummy;



        // provides connection information for EATT L2CAP collision mitigation
        gatt_con_info_set(conidx, p_evt->con_interval, p_evt->con_latency);

        /* ******** Inform other tasks that connection has been established. ******** */
        #if (HOST_PROFILES)
        // Inform profiles about new connection
        prf_con_create(conidx, true);
        #endif /* (HOST_PROFILES) */

        gapc_le_event_client_provide_con_param(conidx, &con_param);

        if(!GETB(p_con->hdr.info_bf, GAPC_IS_NAME_DISCOVERY))
        {
            gapc_env.p_con_req_cbs->le_connection_req(conidx, dummy, actv_idx, p_evt->role, &peer_addr, &con_param,
                                                      p_evt->clk_accuracy);
        }

        *p_conidx = conidx;
    } while(0);

    if(status != GAP_ERR_NO_ERROR)
    {
        gapc_con_cleanup(conidx);
    }

    return (status);
}

uint16_t gapc_le_connection_cfm(uint8_t conidx, uint32_t dummy, const gapc_bond_data_t* p_data)
{
    return (gapc_connection_cfm(conidx, dummy, p_data));
}

bool gapc_is_le_connection(uint8_t conidx)
{
    gapc_con_t* p_con = gapc_get_con_env(conidx);
    return  ((p_con) ? GETB(p_con->info_bf, GAPC_LE_CON_TYPE): false);
}

const gap_bdaddr_t* gapc_le_get_bdaddr(uint8_t conidx, uint8_t src)
{
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);
    return ((p_con != NULL) ? &(p_con->src[src]) : NULL);
}

const gap_bdaddr_t* gapc_le_get_peer_bdaddr(uint8_t conidx)
{
    return (gapc_le_get_bdaddr(conidx, GAPC_INFO_SRC_PEER));
}

const gap_bdaddr_t* gapc_le_get_local_bdaddr(uint8_t conidx)
{
    return (gapc_le_get_bdaddr(conidx, GAPC_INFO_SRC_LOCAL));
}

uint8_t gapc_enc_keysize_get(uint8_t conidx)
{
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);
    return ((p_con != NULL) ? p_con->smp.key_size : 0);
}

uint8_t gapc_get_le_channel_selection_algo(uint8_t conidx)
{
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);
    return ((p_con != NULL) ? p_con->chan_sel_algo : 0xFF);
}

bool gapc_is_le_feat_supported(uint8_t conidx, uint8_t feature)
{
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);
    return ((p_con != NULL) ? ((p_con->peer_features[feature >> 3] & CO_BIT(feature & 0x7)) != 0) : false);
}

uint16_t gapc_get_le_chmap(uint8_t conidx, uint32_t dummy, gapc_get_le_chmap_cmp_cb cmp_cb)
{
    return (gapc_proc_info_create(conidx, dummy, 0, &gapc_get_le_chmap_proc_itf, (gapc_proc_cmp_cb) cmp_cb));
}

uint16_t gapc_get_le_peer_features(uint8_t conidx, uint32_t dummy, gapc_get_le_peer_features_cmp_cb cmp_cb)
{
    return (gapc_proc_info_create(conidx, dummy, 0, &gapc_get_le_peer_features_proc_itf, (gapc_proc_cmp_cb) cmp_cb));
}


uint16_t gapc_le_event_client_register(gapc_le_event_client_t* p_client)
{
    uint16_t status = GAP_ERR_INVALID_PARAM;
    if((p_client != NULL) && (p_client->p_cbs != NULL))
    {
        co_list_push_back(&(gapc_env.le_event_clients), &(p_client->hdr));
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

uint16_t gapc_le_event_client_unregister(gapc_le_event_client_t* p_client)
{
    uint16_t status = GAP_ERR_INVALID_PARAM;
    if((p_client != NULL) && (p_client->p_cbs != NULL))
    {
        status = co_list_extract(&(gapc_env.le_event_clients), &(p_client->hdr)) ? GAP_ERR_NO_ERROR : GAP_ERR_NOT_FOUND;
    }
    return (status);
}


void gapc_le_event_client_provide_con_param(uint8_t conidx, const gap_le_con_param_t* p_param)
{
    gapc_le_event_client_t* p_client = (gapc_le_event_client_t*) co_list_pick(&(gapc_env.le_event_clients));
    while(p_client != NULL)
    {
        if(p_client->p_cbs->con_param != NULL)
        {
            p_client->p_cbs->con_param(conidx, p_param);
        }

        p_client = (gapc_le_event_client_t*) p_client->hdr.next;
    }
}

#endif // (BLE_GAPC)
/// @} GAPC

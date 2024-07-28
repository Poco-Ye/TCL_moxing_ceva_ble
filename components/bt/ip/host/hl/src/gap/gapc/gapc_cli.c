/**
 ****************************************************************************************
 * @file gapc_cli.c
 *
 * @brief  GAP (and GATT) client features for robust caching, name read and other
 *         GAP info discovery, startup of EATT channel creation
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"            // IP configuration
#if (GAPC_PRESENT)
#if (BLE_GATT_CLI)
#include "gatt.h"                   // Native API
#include "gapc_int.h"               // GAPC internals
#include "gapm.h"                   // GAPM utils

#include "../../inc/gatt_hl_api.h"  // to notify about service changed
#include "../../inc/l2cap_hl_api.h" // To enable usage of L2CAP COC

#include "co_endian.h"              // Endianess
#include "co_utils.h"               // Read/Write macros

#include <string.h>                 // For memset


/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/// Client features enabling operation state
enum gapc_cli_feat_en_proc_state
{
    /// Start client features enable operation
    GAPC_CLI_PROC_SVC_DISCOVERY_DONE = HL_PROC_EVENT_FIRST,
    /// Enable Supported client features
    GAPC_CLI_PROC_ENABLE_CLIENT_FEATURE_DONE,
    /// Enable Service changed indication
    GAPC_CLI_PROC_ENABLE_SVC_CHG_IND_DONE,
    /// Read Server supported features
    GAPC_CLI_PROC_READ_SRV_FEAT_DONE,
    /// Read peer database hash value
    GAPC_CLI_PROC_READ_DB_HASH_DONE,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Client features enable procedure object
typedef struct gapc_cli_feat_en_proc
{
    /// procedure inheritance
    hl_proc_t        hdr;
    /// Callback to execute once command completes
    gapc_proc_cmp_cb cmp_cb;
    /// Dummy parameter provided by upper layer SW
    uint32_t         dummy;
    /// Connection index
    uint8_t          conidx;
    /// Current  characteristic is service changed
    bool             svc_chg_char;
    /// Service change Client Char Configuration handle
    uint16_t         svc_chg_ccc_hdl;
    /// Client Feature value handle
    uint16_t         cli_feat_hdl;
    /// Database Hash value handle
    uint16_t         db_hash_hdl;
    /// Server Supported Features value handle
    uint16_t         srv_feat_hdl;
} gapc_cli_feat_en_proc_t;

/// Procedure used to retrieve Client information
typedef struct gapm_cli_info_proc
{
    /// Inherited Simple procedure object
    gapc_proc_simple_t hdr;
    /// Pointer to buffer that contains result data
    co_buf_t*          p_data;
    /// Characteristic value attribute handle
    uint16_t           handle;
    /// Characteristic value 16-bit UUID
    uint16_t           uuid16;
} gapc_cli_info_proc_t;

/*
 * LOCAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */
__STATIC void gapc_cli_cmp_cb(uint8_t conidx, uint8_t user_lid, uint16_t event, uint16_t status);
__STATIC void gapc_cli_svc_cb(uint8_t conidx, uint8_t user_lid, uint16_t event, uint16_t hdl, uint8_t disc_info, uint8_t nb_att, const gatt_svc_att_t* p_atts);
__STATIC void gapc_cli_att_val_cb(uint8_t conidx, uint8_t user_lid, uint16_t event, uint16_t hdl, uint16_t offset, co_buf_t* p_data);
__STATIC void gapc_cli_att_val_evt_cb(uint8_t conidx, uint8_t user_lid, uint16_t token, uint8_t evt_type, bool complete, uint16_t hdl, co_buf_t* p_data);
__STATIC void gapc_cli_svc_changed_cb(uint8_t conidx, uint8_t user_lid, bool out_of_sync, uint16_t start_hdl, uint16_t end_hdl);

__STATIC bool gapc_cli_feat_en_proc_transistion(gapc_cli_feat_en_proc_t* p_proc, uint8_t event, uint16_t status);

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Client callback hander
__STATIC const gatt_cli_cb_t gapc_cli_cb =
{
    .cb_discover_cmp    = gapc_cli_cmp_cb,
    .cb_read_cmp        = gapc_cli_cmp_cb,
    .cb_write_cmp       = gapc_cli_cmp_cb,
    .cb_att_val_get     = NULL,
    .cb_svc             = gapc_cli_svc_cb,
    .cb_svc_info        = NULL,
    .cb_inc_svc         = NULL,
    .cb_char            = NULL,
    .cb_desc            = NULL,
    .cb_att_val         = gapc_cli_att_val_cb,
    .cb_att_val_evt     = gapc_cli_att_val_evt_cb,
    .cb_svc_changed     = gapc_cli_svc_changed_cb,
};

/// Client features enable procedure state machine structure
__STATIC const hl_proc_itf_t gapc_cli_feat_en_proc_itf =
{
    .transition  = (hl_proc_transition_cb) gapc_cli_feat_en_proc_transistion,
    .cleanup     = hl_proc_cleanup,
};

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Ask GATT client to start a write request procedure.
 *
 * @param[in] conidx        Connection index
 * @param[in] dummy         dummy parameter returned when operation complete
 * @param[in] hdl           Attribute handle to modify
 * @param[in] length        Length of attribute value
 * @param[in] p_value       Pointer to attribute value
 *
 * @return Execution status (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC uint16_t gapc_cli_write(uint8_t conidx, uint16_t dummy, uint16_t hdl, uint16_t length, uint8_t* p_value)
{
    co_buf_t* p_data = NULL;
    uint16_t status;

    // allocate buffer that contains write value
    if(co_buf_alloc(&p_data, GATT_BUFFER_HEADER_LEN, length, GATT_BUFFER_TAIL_LEN) != CO_BUF_ERR_NO_ERROR)
    {
        status = ATT_ERR_INSUFF_RESOURCE;
    }
    else
    {
        // copy data into buffer
        co_buf_copy_data_from_mem(p_data, p_value, length);

        // ask for write execution
        status = gatt_cli_write(conidx, gapc_env.cli_user_lid, dummy, GATT_WRITE,  hdl, 0, p_data);
        co_buf_release(p_data);
    }
    return (status);
}

/// handler of completion of automatic client feature enable procedure
__STATIC void gapc_cli_client_features_enable_cmp(uint8_t conidx, uint32_t dummy, uint16_t status)
{
    // ignore
}

/// Create a Read information procedure
__STATIC uint16_t gapc_cli_info_read_proc_create(uint8_t conidx, uint32_t dummy, uint16_t uuid16,
                                                const gapc_proc_simple_itf_t* p_itf, gapc_proc_cmp_cb cmp_cb)
{
    uint16_t status;
    gapc_cli_info_proc_t* p_proc;

    status = gapc_proc_simple_create(conidx, dummy, cmp_cb, sizeof(gapc_cli_info_proc_t), p_itf,
                                     (gapc_proc_simple_t**) &p_proc);
    if(status == GAP_ERR_NO_ERROR)
    {
        p_proc->uuid16 = uuid16;
        p_proc->handle = GATT_INVALID_HDL;
        p_proc->p_data = NULL;
    }

    return (status);
}


/*
 * PROCEDURE STATE MACHINE
 ****************************************************************************************
 */

/// Client features enable procedure state machine transition function
__STATIC bool gapc_cli_feat_en_proc_transistion(gapc_cli_feat_en_proc_t* p_proc, uint8_t event, uint16_t status)
{
    bool is_finished = false;
    uint8_t conidx = p_proc->conidx;
    gapc_con_t* p_con = gapc_get_con_env(conidx);
    gapc_bond_t* p_bond = &(p_con->bond);

    if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            {
                // Start GATT Service discovery
                uint16_t gatt_svc_uuid = GATT_SVC_GENERIC_ATTRIBUTE;
                status = gatt_cli_discover_svc(conidx, gapc_env.cli_user_lid, GAPC_CLI_PROC_SVC_DISCOVERY_DONE,
                                               GATT_DISCOVER_SVC_PRIMARY_BY_UUID, true,
                                               GATT_MIN_HDL, GATT_MAX_HDL, GATT_UUID_16, (uint8_t*) &gatt_svc_uuid);
            } break;
            case GAPC_CLI_PROC_SVC_DISCOVERY_DONE:
            {
                if(p_proc->cli_feat_hdl != GATT_INVALID_HDL)
                {
                    // Ask for write execution
                    uint8_t cli_feat = GAPC_CLI_FEAT_SUPPORTED_MASK;
                    status = gapc_cli_write(conidx, GAPC_CLI_PROC_ENABLE_CLIENT_FEATURE_DONE, p_proc->cli_feat_hdl,
                                            1, &cli_feat);
                    break;
                }
            }
            // no break
            case GAPC_CLI_PROC_ENABLE_CLIENT_FEATURE_DONE:
            {
                // register service change indication
                if(p_bond->svc_chg_offset != 0)
                {
                    status = gatt_cli_event_register(conidx, gapc_env.cli_user_lid, p_bond->gatt_start_hdl,
                                                             p_bond->gatt_start_hdl + p_bond->gatt_nb_att);
                    if(status != GAP_ERR_NO_ERROR) break;
                }

                if(p_proc->svc_chg_ccc_hdl != GATT_INVALID_HDL)
                {
                    // Ask for write execution
                    uint16_t cli_cfg = GATT_CCC_START_IND;
                    gapc_cli_write(conidx, GAPC_CLI_PROC_ENABLE_SVC_CHG_IND_DONE, p_proc->svc_chg_ccc_hdl,
                                   2, (uint8_t*) &cli_cfg);
                    break;
                }

            }
            // no break
            case GAPC_CLI_PROC_ENABLE_SVC_CHG_IND_DONE:
            {
                if(p_proc->srv_feat_hdl != GATT_INVALID_HDL)
                {
                    // read server supported features
                    status = gatt_cli_read(conidx, gapc_env.cli_user_lid, GAPC_CLI_PROC_READ_SRV_FEAT_DONE,
                                           p_proc->srv_feat_hdl, 0, 0);
                    break;
                }
            }
            // no break
            case GAPC_CLI_PROC_READ_SRV_FEAT_DONE:
            {
                if((p_proc->db_hash_hdl != GATT_INVALID_HDL) && (gapc_env.p_info_cbs->cli_hash_info != NULL))
                {
                    // read database hash value
                    status = gatt_cli_read(conidx, gapc_env.cli_user_lid, GAPC_CLI_PROC_READ_DB_HASH_DONE,
                                           p_proc->db_hash_hdl, 0, 0);
                    break;
                }
            }
            // no break
            case GAPC_CLI_PROC_READ_DB_HASH_DONE:
            default:
            {
                is_finished = true;
            } break;
        }
    }

    // error handling
    if(status != GAP_ERR_NO_ERROR)
    {
        is_finished = true;
    }

    if(is_finished)
    {
        // Inform the application about the new configuration
        gapc_bond_info_send(conidx);

        // inform application that procedure is over
        p_proc->cmp_cb(conidx, p_proc->dummy, status);
    }

    return (is_finished);
}


/// Information read procedure granted callback
__STATIC uint16_t gapm_cli_read_info_proc_granted(uint8_t conidx, gapc_cli_info_proc_t* p_proc)
{
    uint16_t uuid16 = p_proc->uuid16;
    return (gatt_cli_read_by_uuid(conidx, gapc_env.cli_user_lid, HL_PROC_FINISHED,
                                  GATT_MIN_HDL, GATT_MAX_HDL, GATT_UUID_16, (uint8_t*) &uuid16));
}

/// Read device name procedure finished callback
__STATIC void gapc_read_device_name_proc_finished(uint8_t conidx, gapc_cli_info_proc_t* p_proc, uint16_t status)
{
    gapc_read_device_name_cmp_cb cmp_cb = (gapc_read_device_name_cmp_cb) p_proc->hdr.cmp_cb;
    co_buf_t* p_data = p_proc->p_data;

    // Provide result to upper layer application
    cmp_cb(conidx, p_proc->hdr.dummy, status, p_proc->handle, p_data);

    // release buffer
    if(p_data != NULL)
    {
        co_buf_release(p_data);
    }
}

/// Simple procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_read_device_name_proc_itf =
{
    .granted  = (gapc_proc_simple_granted_cb)  gapm_cli_read_info_proc_granted,
    .finished = (gapc_proc_simple_finished_cb) gapc_read_device_name_proc_finished,
};


/// Read appearance procedure finished callback
__STATIC void gapc_read_appearance_proc_finished(uint8_t conidx, gapc_cli_info_proc_t* p_proc, uint16_t status)
{
    gapc_read_appearance_cmp_cb cmp_cb = (gapc_read_appearance_cmp_cb) p_proc->hdr.cmp_cb;
    co_buf_t* p_data = p_proc->p_data;
    uint16_t appearance = 0;

    if(p_data != NULL)
    {
        appearance = co_btohs(co_read16p(co_buf_data(p_data)));
        co_buf_release(p_data);
    }

    // Provide result to upper layer application
    cmp_cb(conidx, p_proc->hdr.dummy, status, p_proc->handle, appearance);
}

/// Simple procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_read_appearance_proc_itf =
{
    .granted  = (gapc_proc_simple_granted_cb)  gapm_cli_read_info_proc_granted,
    .finished = (gapc_proc_simple_finished_cb) gapc_read_appearance_proc_finished,
};

/// Read peripheral preferred parameters procedure finished callback
__STATIC void gapc_read_periph_pref_param_proc_finished(uint8_t conidx, gapc_cli_info_proc_t* p_proc, uint16_t status)
{
    gapc_read_periph_pref_param_cmp_cb cmp_cb = (gapc_read_periph_pref_param_cmp_cb) p_proc->hdr.cmp_cb;
    co_buf_t* p_data = p_proc->p_data;
    gap_periph_pref_t param;
    gap_periph_pref_t* p_param = NULL;

    if(p_data != NULL)
    {
        uint8_t* p_val = co_buf_data(p_data);

        param.con_intv_min  = co_btohs(co_read16p(&(p_val[0])));
        param.con_intv_max  = co_btohs(co_read16p(&(p_val[2])));
        param.latency       = co_btohs(co_read16p(&(p_val[4])));
        param.conn_timeout  = co_btohs(co_read16p(&(p_val[6])));

        co_buf_release(p_data);
        p_param = &param;
    }

    // Provide result to upper layer application
    cmp_cb(conidx, p_proc->hdr.dummy, status, p_proc->handle, p_param);
}

/// Simple procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_read_periph_pref_param_proc_itf =
{
    .granted  = (gapc_proc_simple_granted_cb)  gapm_cli_read_info_proc_granted,
    .finished = (gapc_proc_simple_finished_cb) gapc_read_periph_pref_param_proc_finished,
};


/// Read central address resolution supported procedure finished callback
__STATIC void gapc_read_central_addr_resol_supp_proc_finished(uint8_t conidx, gapc_cli_info_proc_t* p_proc, uint16_t status)
{
    gapc_read_central_addr_resol_supp_cmp_cb cmp_cb = (gapc_read_central_addr_resol_supp_cmp_cb) p_proc->hdr.cmp_cb;
    co_buf_t* p_data = p_proc->p_data;
    uint8_t ctl_addr_resol = 0;

    if(p_data != NULL)
    {
        ctl_addr_resol = co_buf_data(p_data)[0];
        co_buf_release(p_data);
    }

    // Provide result to upper layer application
    cmp_cb(conidx, p_proc->hdr.dummy, status, p_proc->handle, ctl_addr_resol);
}

/// Simple procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_read_central_addr_resol_supp_proc_itf =
{
    .granted  = (gapc_proc_simple_granted_cb)  gapm_cli_read_info_proc_granted,
    .finished = (gapc_proc_simple_finished_cb) gapc_read_central_addr_resol_supp_proc_finished,
};

/// Read database hash procedure finished callback
__STATIC void gapc_read_database_hash_proc_finished(uint8_t conidx, gapc_cli_info_proc_t* p_proc, uint16_t status)
{
    gapc_read_database_hash_cmp_cb cmp_cb = (gapc_read_database_hash_cmp_cb) p_proc->hdr.cmp_cb;
    co_buf_t* p_data = p_proc->p_data;
    uint8_t* p_hash = (p_data != NULL) ? co_buf_data(p_data) : NULL;

    // Provide result to upper layer application
    cmp_cb(conidx, p_proc->hdr.dummy, status, p_proc->handle, p_hash);

    if(p_data != NULL)
    {
        co_buf_release(p_data);
    }
}

/// Simple procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_read_database_hash_proc_itf =
{
    .granted  = (gapc_proc_simple_granted_cb)  gapm_cli_read_info_proc_granted,
    .finished = (gapc_proc_simple_finished_cb) gapc_read_database_hash_proc_finished,
};


/// Read resolvable private address only procedure finished callback
__STATIC void gapc_read_rslv_priv_addr_only_itf_finished(uint8_t conidx, gapc_cli_info_proc_t* p_proc, uint16_t status)
{
    gapc_read_rslv_priv_addr_only_cmp_cb cmp_cb = (gapc_read_rslv_priv_addr_only_cmp_cb) p_proc->hdr.cmp_cb;
    co_buf_t* p_data = p_proc->p_data;
    uint8_t rslv_priv_addr_only = 0;

    if(p_data != NULL)
    {
        rslv_priv_addr_only = co_buf_data(p_data)[0];
        co_buf_release(p_data);
    }

    // Provide result to upper layer application
    cmp_cb(conidx, p_proc->hdr.dummy, status, p_proc->handle, rslv_priv_addr_only);
}

/// Simple procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_read_rslv_priv_addr_only_itf =
{
    .granted  = (gapc_proc_simple_granted_cb)  gapm_cli_read_info_proc_granted,
    .finished = (gapc_proc_simple_finished_cb) gapc_read_rslv_priv_addr_only_itf_finished,
};
/*
 * GATT CALLBACKS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief This function is called when GATT client user procedure is over.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] event         Event transition
 * @param[in] status        Status of the procedure (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC void gapc_cli_cmp_cb(uint8_t conidx, uint8_t user_lid, uint16_t event, uint16_t status)
{
    if(event != HL_PROC_INVALID)
    {
        gapc_proc_transition(conidx, event, status);
    }
}

/**
 ****************************************************************************************
 * @brief This function is called when a full service has been found during a discovery procedure.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] event         Event transition
 * @param[in] hdl           First handle value of following list
 * @param[in] disc_info     Discovery information (see enum #gatt_svc_disc_info)
 * @param[in] nb_att        Number of attributes
 * @param[in] p_atts        Pointer to attribute information present in a service
 ****************************************************************************************
 */
__STATIC void gapc_cli_svc_cb(uint8_t conidx, uint8_t user_lid, uint16_t event, uint16_t hdl, uint8_t disc_info,
                             uint8_t nb_att, const gatt_svc_att_t* p_atts)
{
    gapc_bond_t* p_bond = &(gapc_env.p_con[conidx]->bond);

    // check if message should be handled
    if(gapc_proc_is_active(conidx, &gapc_cli_feat_en_proc_itf))
    {
        gapc_cli_feat_en_proc_t* p_proc = (gapc_cli_feat_en_proc_t*) gapc_proc_get(conidx);

        uint8_t  cursor;
        uint16_t att_handle  = GATT_INVALID_HDL;

        // check all attributes
        for(cursor = 0 ; cursor < nb_att ; cursor++)
        {
            const gatt_svc_att_t* p_att = &(p_atts[cursor]);

            switch(p_att->att_type)
            {
                case GATT_ATT_PRIMARY_SVC:
                {
                    if(((disc_info == GATT_SVC_START) || (disc_info == GATT_SVC_CMPLT)) && (cursor == 0))
                    {
                        ASSERT_WARN(gatt_uuid16_comp(p_att->uuid, p_att->uuid_type, GATT_SVC_GENERIC_ATTRIBUTE),
                        conidx, co_read16p(p_att->uuid));

                        p_bond->gatt_start_hdl = p_att->info.svc.start_hdl;
                        p_bond->gatt_nb_att    = p_att->info.svc.end_hdl - p_att->info.svc.start_hdl;
                    }
                }
                break;
                case GATT_ATT_CHAR:
                {
                    p_proc->svc_chg_char = false;
                    att_handle = p_att->info.charac.val_hdl;
                } break;
                case GATT_ATT_VAL:
                {

                    // service changed value
                    if(gatt_uuid16_comp(p_att->uuid, p_att->uuid_type, GATT_CHAR_SERVICE_CHANGED))
                    {
                        p_proc->svc_chg_char = true;
                        p_bond->svc_chg_offset = att_handle - p_bond->gatt_start_hdl;
                    }
                    // Database Hash value
                    else if(gatt_uuid16_comp(p_att->uuid, p_att->uuid_type, GATT_CHAR_DB_HASH))
                    {
                        p_proc->db_hash_hdl = att_handle;
                    }
                    // Client supported features
                    else if(gatt_uuid16_comp(p_att->uuid, p_att->uuid_type, GATT_CHAR_CLI_SUP_FEAT))
                    {
                        p_proc->cli_feat_hdl = att_handle;
                    }
                    // Server supported features
                    else if(gatt_uuid16_comp(p_att->uuid, p_att->uuid_type, GATT_CHAR_SRV_SUP_FEAT))
                    {
                        p_proc->srv_feat_hdl = att_handle;
                    }
                } break;
                case GATT_ATT_DESC:
                {
                    // Client Char Configuration of service changed value
                    if(p_proc->svc_chg_char && gatt_uuid16_comp(p_att->uuid, p_att->uuid_type, GATT_DESC_CLIENT_CHAR_CFG))
                    {
                        p_proc->svc_chg_ccc_hdl = hdl + cursor;
                    }
                } break;
                default: { /* Nothing to do */ } break;
            }
        }
    }
}


/**
 ****************************************************************************************
 * @brief Update the server supported features
 *
 * @param[in] conidx        Connection index
 * @param[in] svr_feat      read server supported features
 ****************************************************************************************
 */
__STATIC void gapc_cli_svr_feat_upd(uint8_t conidx, uint8_t svr_feat)
{
    gapc_bond_t* p_bond = &(gapc_env.p_con[conidx]->bond);
    p_bond->srv_feat    = svr_feat & GAPC_SRV_FEAT_SUPPORTED_MASK;

    if(GETB(p_bond->srv_feat, GAPC_SRV_EATT_SUPPORTED))
    {
        l2cap_coc_enhanced_nego_set(conidx, true);

        // if link already encrypted and automatic EATT establishment enable
        if(GETB(gapc_env.cfg_flags, GAPC_CLI_AUTO_EATT) && gapc_is_sec_set(conidx, GAPC_LK_ENCRYPTED))
        {
            // Start establishment of EATT bearers
            gatt_bearer_eatt_estab(conidx);
        }
    }
}

/**
 ****************************************************************************************
 * @brief This function is called during a read procedure when attribute value is retrieved
 *        form peer device.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] event         Event Transition
 * @param[in] hdl           Attribute handle
 * @param[in] offset        Data offset
 * @param[in] p_data        Pointer to buffer that contains attribute value starting from offset
 ****************************************************************************************
 */
__STATIC void gapc_cli_att_val_cb(uint8_t conidx, uint8_t user_lid, uint16_t event, uint16_t hdl, uint16_t offset,
                                  co_buf_t* p_data)
{
    if(event == HL_PROC_INVALID)
    {
        gapc_cli_svr_feat_upd(conidx, co_buf_data(p_data)[0]);
    }
    else if (event == HL_PROC_FINISHED)
    {
        // keep information about read value
        gapc_cli_info_proc_t* p_proc = (gapc_cli_info_proc_t*) gapc_proc_get(conidx);
        ASSERT_ERR(p_proc->p_data == NULL); // ensure value isn't modified twice
        p_proc->p_data = p_data;
        p_proc->handle = hdl;
        co_buf_acquire(p_data);
    }
    else if(gapc_proc_is_active(conidx, &gapc_cli_feat_en_proc_itf))
    {
        gapc_cli_feat_en_proc_t* p_proc = (gapc_cli_feat_en_proc_t*) gapc_proc_get(conidx);
        if (p_proc->srv_feat_hdl == hdl)
        {
            gapc_cli_svr_feat_upd(conidx, co_buf_data(p_data)[0]);
        }
        else if (p_proc->db_hash_hdl == hdl)
        {
            gapc_con_t* p_con = gapc_get_con_env(conidx);
            gapc_env.p_info_cbs->cli_hash_info(conidx, p_con->dummy, hdl, co_buf_data(p_data));
        }
    }
}

/**
 ****************************************************************************************
 * @brief This function is called when a notification or an indication is received onto
 *        register handle range (@see gatt_cli_event_register).
 *
 *        @see gatt_cli_val_event_cfm must be called to confirm event reception.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] token         Procedure token that must be returned in confirmation function
 * @param[in] evt_type      Event type triggered (see enum #gatt_evt_type)
 * @param[in] complete      True if event value if complete value has been received
 *                          False if data received is equals to maximum attribute protocol value.
 *                          In such case GATT Client User should perform a read procedure.
 * @param[in] hdl           Attribute handle
 * @param[in] p_data        Pointer to buffer that contains attribute value
 ****************************************************************************************
 */
__STATIC void gapc_cli_att_val_evt_cb(uint8_t conidx, uint8_t user_lid, uint16_t token, uint8_t evt_type, bool complete,
                                     uint16_t hdl, co_buf_t* p_data)
{
    gapc_bond_t* p_bond = &(gapc_env.p_con[conidx]->bond);
    if(hdl == (p_bond->gatt_start_hdl + p_bond->svc_chg_offset))
    {
        if(co_buf_data_len(p_data) == (GATT_HANDLE_LEN * 2))
        {
            uint16_t start_hdl = co_btohs(co_read16p(&(co_buf_data(p_data)[0])));
            uint16_t end_hdl   = co_btohs(co_read16p(&(co_buf_data(p_data)[2])));

            // inform gatt that service changed indication has been received
            gatt_cli_event_svc_chg(conidx, false, start_hdl, end_hdl);
        }
        // else ignore

        // confirm peer device about proper reception of indication
        gatt_cli_att_event_cfm(conidx, user_lid, token);
    }
}

/**
 ****************************************************************************************
 * @brief Event triggered when a service change has been received or if an attribute
 *        transaction triggers an out of sync error.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] out_of_sync   True if an out of sync error has been received
 * @param[in] start_hdl     Service start handle
 * @param[in] end_hdl       Service end handle
 ****************************************************************************************
 */
__STATIC void gapc_cli_svc_changed_cb(uint8_t conidx, uint8_t user_lid, bool out_of_sync,
                                     uint16_t start_hdl, uint16_t end_hdl)
{
    // nothing to do
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t gapc_cli_setup(uint16_t att_cfg)
{
    uint16_t status;

    // register client user with a minimum MTU and with maximum priority
    status = gatt_user_cli_register(L2CAP_LE_MTU_MIN, 255, &gapc_cli_cb, &gapc_env.cli_user_lid);

    // setup automatic configuration
    SETB(gapc_env.cfg_flags, GAPC_CLI_AUTO_MTU_EXCH_EN, !GETF(att_cfg, GAPM_ATT_CLI_DIS_AUTO_MTU_EXCH));
    SETB(gapc_env.cfg_flags, GAPC_CLI_AUTO_FEAT_EN,     !GETF(att_cfg, GAPM_ATT_CLI_DIS_AUTO_FEAT_EN));
    SETB(gapc_env.cfg_flags, GAPC_CLI_AUTO_EATT,        !GETF(att_cfg, GAPM_ATT_CLI_DIS_AUTO_EATT));

    return (status);
}


void gapc_cli_con_create(uint8_t conidx, bool bond_data_present, uint16_t gatt_start_hdl, uint16_t gatt_end_hdl,
                        uint16_t svc_chg_hdl, uint8_t srv_feat)
{
    gapc_bond_t* p_bond = &(gapc_env.p_con[conidx]->bond);

    // start MTU exchange
    if(GETB(gapc_env.cfg_flags, GAPC_CLI_AUTO_MTU_EXCH_EN))
    {
        gatt_cli_mtu_exch(conidx, gapc_env.cli_user_lid);
    }

    // If bond data present, prepare reception of GATT events.
    if(bond_data_present && (gatt_start_hdl != GATT_INVALID_HDL))
    {
        p_bond->gatt_start_hdl    = gatt_start_hdl;
        p_bond->gatt_nb_att       = gatt_end_hdl - gatt_start_hdl + 1;
        p_bond->svc_chg_offset    = svc_chg_hdl - gatt_start_hdl;
        p_bond->srv_feat          = srv_feat;

        // register service changed events
        gatt_cli_event_register(conidx, gapc_env.cli_user_lid, gatt_start_hdl, gatt_end_hdl);
    }
    else
    {
        p_bond->gatt_start_hdl    = 0;
        p_bond->gatt_nb_att       = 0;
        p_bond->svc_chg_offset    = 0;

        // start robust caching if supported (not needed if bond data present)
        if(GETB(gapc_env.cfg_flags, GAPC_CLI_AUTO_FEAT_EN))
        {
            gapc_client_features_enable(conidx, 0, gapc_cli_client_features_enable_cmp);
        }
        // read supported server feature
        else if (GETB(gapc_env.cfg_flags, GAPC_CLI_AUTO_EATT))
        {
            uint16_t uuid16 = GATT_CHAR_SRV_SUP_FEAT;

            // request read server supported features
            gatt_cli_read_by_uuid(conidx, gapc_env.cli_user_lid, HL_PROC_INVALID,
                                  GATT_MIN_HDL, GATT_MAX_HDL, GATT_UUID_16, (uint8_t*) &uuid16);

        }
    }
}

void gapc_cli_link_encrypted(uint8_t conidx)
{
    gapc_bond_t* p_bond = &(gapc_env.p_con[conidx]->bond);

    // if peer accepts enhanced attribute
    if(GETB(gapc_env.cfg_flags, GAPC_CLI_AUTO_EATT) && GETB(p_bond->srv_feat, GAPC_SRV_EATT_SUPPORTED))
    {
        // Start establishment of EATT bearers
        gatt_bearer_eatt_estab(conidx);
    }
}


/*
 * EXTERNAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t gapc_client_features_enable(uint8_t conidx, uint32_t dummy, gapc_proc_cmp_cb cmp_cb)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    do
    {
        gapc_cli_feat_en_proc_t* p_proc;
        gapc_con_t* p_con = gapc_get_con_env(conidx);
        if(p_con == NULL) break;

        status = gapc_proc_create(p_con, sizeof(gapc_cli_feat_en_proc_t),
                                  &gapc_cli_feat_en_proc_itf,  (hl_proc_t**) &p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        p_proc->conidx          = conidx;
        p_proc->cmp_cb          = cmp_cb;
        p_proc->dummy           = dummy;
        p_proc->svc_chg_ccc_hdl = GATT_INVALID_HDL;
        p_proc->cli_feat_hdl    = GATT_INVALID_HDL;
        p_proc->db_hash_hdl     = GATT_INVALID_HDL;
        p_proc->srv_feat_hdl    = GATT_INVALID_HDL;
    } while(0);

    return (status);
}

uint16_t gapc_read_device_name(uint8_t conidx, uint32_t dummy, gapc_read_device_name_cmp_cb cmp_cb)
{
    return (gapc_cli_info_read_proc_create(conidx, dummy, GATT_CHAR_DEVICE_NAME, &gapc_read_device_name_proc_itf,
                                           (gapc_proc_cmp_cb)cmp_cb));
}

uint16_t gapc_read_appearance(uint8_t conidx, uint32_t dummy, gapc_read_appearance_cmp_cb cmp_cb)
{
    return (gapc_cli_info_read_proc_create(conidx, dummy, GATT_CHAR_APPEARANCE, &gapc_read_appearance_proc_itf,
                                           (gapc_proc_cmp_cb)cmp_cb));
}

uint16_t gapc_read_periph_pref_param(uint8_t conidx, uint32_t dummy, gapc_read_periph_pref_param_cmp_cb cmp_cb)
{
    return (gapc_cli_info_read_proc_create(conidx, dummy, GATT_CHAR_PERIPH_PREF_CON_PARAM,
                                           &gapc_read_periph_pref_param_proc_itf, (gapc_proc_cmp_cb)cmp_cb));
}

uint16_t gapc_read_central_addr_resol_supp(uint8_t conidx, uint32_t dummy, gapc_read_central_addr_resol_supp_cmp_cb cmp_cb)
{
    return (gapc_cli_info_read_proc_create(conidx, dummy, GATT_CHAR_CTL_ADDR_RESOL_SUPP,
                                           &gapc_read_central_addr_resol_supp_proc_itf, (gapc_proc_cmp_cb)cmp_cb));
}

uint16_t gapc_read_database_hash(uint8_t conidx, uint32_t dummy, gapc_read_database_hash_cmp_cb cmp_cb)
{
    return (gapc_cli_info_read_proc_create(conidx, dummy, GATT_CHAR_DB_HASH,
                                           &gapc_read_database_hash_proc_itf, (gapc_proc_cmp_cb)cmp_cb));
}

uint16_t gapc_read_rslv_priv_addr_only(uint8_t conidx, uint32_t dummy, gapc_read_rslv_priv_addr_only_cmp_cb cmp_cb)
{
    return (gapc_cli_info_read_proc_create(conidx, dummy, GATT_CHAR_RSLV_PRIV_ADDR_ONLY,
                                           &gapc_read_rslv_priv_addr_only_itf, (gapc_proc_cmp_cb)cmp_cb));
}

#endif // (BLE_GATT_CLI)
#endif // (GAPC_PRESENT)
/// @} GATT


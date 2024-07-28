/**
 ****************************************************************************************
 * @file gapc_svc.c
 *
 * @brief  GAP (and GATT) service handler
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
#if (BLE_GATT)
#include "gatt.h"                   // Native API
#include "gap.h"                    // HL defines
#include "gapc_int.h"               // GAPC internals
#include "../gap_int.h"             // GAP internals
#include "gapm.h"                   // GAPM utils

#include "co_endian.h"              // Endianess
#include "co_utils.h"               // Read/Write macros
#include "co_djob.h"                // To defer service changed indication
#include "co_math.h"                // Max/Min usage and bit field manipulation

#include <string.h>                 // For memset

#include "../../inc/l2cap_att.h"    // Attribute includes
#include "../../inc/gatt_hl_api.h"  // GATT setup function
#include "../../inc/l2cap_hl_api.h" // To enable usage of L2CAP COC

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */
/// This value ensure to have up to 128 client feature supported would be sufficient for several years.
#define GAPC_SVC_MAX_CLI_FEAT_LEN   (16)


// GAP database default features
#define GAPC_SVC_DEFAULT_FEAT               (0x001F)
// GAP Resolvable private address only
#define GAPC_SVC_RSLV_PRIV_ADDR_ONLY_FEAT   (0x0060)
// GAP database features in peripheral role
#define GAPC_SVC_PERIPH_FEAT                (0x0180)
// GAP database features in central role
#define GAPC_SVC_CENTRAL_FEAT               (0x0600)


/// GAP Attribute database handles
/// Generic Access Profile Service
enum
{
    // GATT Service index
    GATT_IDX_PRIM_SVC,

    GATT_IDX_CHAR_SVC_CHANGED,
    GATT_IDX_SVC_CHANGED,
    GATT_IDX_SVC_CHANGED_CFG,

    GATT_IDX_CHAR_CLI_SUP_FEAT,
    GATT_IDX_CLI_SUP_FEAT,

    GATT_IDX_CHAR_DB_HASH,
    GATT_IDX_DB_HASH,

    GATT_IDX_CHAR_SRV_SUP_FEAT,
    GATT_IDX_SRV_SUP_FEAT,

    GATT_IDX_NUMBER,

    // GAP Service index
    GAP_IDX_PRIM_SVC = GATT_IDX_NUMBER,

    GAP_IDX_CHAR_DEVNAME,
    GAP_IDX_DEVNAME,

    GAP_IDX_CHAR_ICON,
    GAP_IDX_ICON,

    GAP_IDX_CHAR_RSLV_PRIV_ADDR_ONLY,
    GAP_IDX_RSLV_PRIV_ADDR_ONLY,

    GAP_IDX_CHAR_SLAVE_PREF_PARAM,
    GAP_IDX_SLAVE_PREF_PARAM,

    GAP_IDX_CHAR_CNT_ADDR_RESOL,
    GAP_IDX_CNT_ADDR_RESOL,

    GAP_IDX_NUMBER,

    /// Maximum number of GATT attributes
    GATT_NB_ATT = GATT_IDX_NUMBER,
    /// Maximum number of GAP attributes
    GAP_NB_ATT = GAP_IDX_NUMBER - GATT_IDX_NUMBER,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Environment structure of service changed buffer
typedef struct gapc_svc_chg_buf_meta
{
    /// Defer service changed indication
    co_djob_t defer;
    /// list of connection index that should receive indication
    uint32_t  conidx_bf;
    /// Service changed Start handle
    uint16_t  start_hdl;
    /// Service changed End handle
    uint16_t  end_hdl;
} gapc_svc_chg_buf_meta_t;


/*
 * LOCAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */
__STATIC void gapc_svc_cb_event_sent(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status);
__STATIC void gapc_svc_cb_att_read_get(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t hdl, uint16_t offset, uint16_t max_length);
__STATIC void gapc_svc_cb_att_info_get(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t hdl);
__STATIC void gapc_svc_cb_att_val_set(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t hdl, uint16_t offset,co_buf_t* p_data);
__STATIC void gapc_svc_cb_hash(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status, const uint8_t* p_hash);

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// GAP Attribute database description
__STATIC const gatt_att16_desc_t gapc_svc_db[GAP_IDX_NUMBER] =
{
    // ---------------------------------- GATT SERVICE ---------------------------------------------------------------

    // GATT service
    [GATT_IDX_PRIM_SVC]          = { GATT_DECL_PRIMARY_SERVICE, PROP(RD),            0                                          },

    // Service Changed
    [GATT_IDX_CHAR_SVC_CHANGED]  = { GATT_DECL_CHARACTERISTIC,  PROP(RD),            0                                          },
    [GATT_IDX_SVC_CHANGED]       = { GATT_CHAR_SERVICE_CHANGED, PROP(I),             OPT(NO_OFFSET) | GATT_HANDLE_LEN*2         },
    [GATT_IDX_SVC_CHANGED_CFG]   = { GATT_DESC_CLIENT_CHAR_CFG, PROP(RD) | PROP(WR), OPT(NO_OFFSET) | sizeof(uint16_t)          },

    // Client Supported Features
    [GATT_IDX_CHAR_CLI_SUP_FEAT] = { GATT_DECL_CHARACTERISTIC,  PROP(RD),            0                                           },
    [GATT_IDX_CLI_SUP_FEAT]      = { GATT_CHAR_CLI_SUP_FEAT,    PROP(RD) | PROP(WR), OPT(NO_OFFSET) | GAPC_SVC_MAX_CLI_FEAT_LEN  },

    // Database Hash
    [GATT_IDX_CHAR_DB_HASH]      = { GATT_DECL_CHARACTERISTIC,  PROP(RD),            0                                           },
    [GATT_IDX_DB_HASH]           = { GATT_CHAR_DB_HASH,         PROP(RD),            OPT(NO_OFFSET) | GAP_KEY_LEN                },

    // Server Supported Features
    [GATT_IDX_CHAR_SRV_SUP_FEAT] = { GATT_DECL_CHARACTERISTIC,  PROP(RD),            0                                           },
    [GATT_IDX_SRV_SUP_FEAT]      = { GATT_CHAR_SRV_SUP_FEAT,    PROP(RD),            OPT(NO_OFFSET) | 1                          },

    // ---------------------------------- GAP SERVICE ----------------------------------------------------------------

    // GAP service
    [GAP_IDX_PRIM_SVC]                 =   {GATT_DECL_PRIMARY_SERVICE,       PROP(RD), 0                                        },

    // Device name
    [GAP_IDX_CHAR_DEVNAME]             =   {GATT_DECL_CHARACTERISTIC,        PROP(RD), 0                                        },
    [GAP_IDX_DEVNAME]                  =   {GATT_CHAR_DEVICE_NAME,           PROP(RD), GAP_MAX_NAME_SIZE                        },

    // Appearance
    [GAP_IDX_CHAR_ICON]                =   {GATT_DECL_CHARACTERISTIC,        PROP(RD), 0                                        },
    [GAP_IDX_ICON]                     =   {GATT_CHAR_APPEARANCE,            PROP(RD), OPT(NO_OFFSET) | sizeof(uint16_t)        },

    // Resolvable Private Address Only
    [GAP_IDX_CHAR_RSLV_PRIV_ADDR_ONLY] =   {GATT_DECL_CHARACTERISTIC,        PROP(RD), 0                                        },
    [GAP_IDX_RSLV_PRIV_ADDR_ONLY]      =   {GATT_CHAR_RSLV_PRIV_ADDR_ONLY,   PROP(RD), OPT(NO_OFFSET) | sizeof(uint8_t)         },

    // Peripheral parameters
    [GAP_IDX_CHAR_SLAVE_PREF_PARAM]    =   {GATT_DECL_CHARACTERISTIC,        PROP(RD), 0                                        },
    [GAP_IDX_SLAVE_PREF_PARAM]         =   {GATT_CHAR_PERIPH_PREF_CON_PARAM, PROP(RD), OPT(NO_OFFSET) | sizeof(gap_periph_pref_t)  },

    // Central Address Resolution
    [GAP_IDX_CHAR_CNT_ADDR_RESOL]      =   {GATT_DECL_CHARACTERISTIC,        PROP(RD), 0                                        },
    [GAP_IDX_CNT_ADDR_RESOL]           =   {GATT_CHAR_CTL_ADDR_RESOL_SUPP,   PROP(RD), OPT(NO_OFFSET) | sizeof(uint8_t)         },
};

/// Service callback hander
__STATIC const gatt_srv_cb_t gapc_svc_cb =
{
        .cb_event_sent    = gapc_svc_cb_event_sent,
        .cb_att_read_get  = gapc_svc_cb_att_read_get,
        .cb_att_event_get = NULL,
        .cb_att_info_get  = gapc_svc_cb_att_info_get,
        .cb_att_val_set   = gapc_svc_cb_att_val_set,
};

/// hash computation result callback
__STATIC const gatt_db_hash_cb_t gapc_svc_hash_cb =
{
        .cb_db_hash       = gapc_svc_cb_hash,
};

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Convert attribute index to attribute handle
 *
 * @param[in] att_idx       Attribute index
 *
 * @return Attribute handle
 ****************************************************************************************
 */
uint16_t gapc_svc_hdl_get(uint8_t att_idx)
{
    uint16_t handle = GATT_INVALID_HDL;

    // GATT service handles
    if(att_idx < GATT_IDX_NUMBER)
    {
        handle = gapc_env.svc.gatt_start_hdl + att_idx;
    }
    // GAP service handles
    else if(att_idx < GAP_IDX_NUMBER)
    {
        handle = gapc_env.svc.gap_start_hdl + att_idx  - GATT_IDX_NUMBER;

        // update handle according to supported features
        if((att_idx > GAP_IDX_RSLV_PRIV_ADDR_ONLY) && !GETB(gapc_env.cfg_flags, GAPC_SVC_RSLV_PRIV_ADDR_ONLY_PRES))
        {
            handle -= 2;
        }
        if((att_idx > GAP_IDX_SLAVE_PREF_PARAM) && !GETB(gapc_env.cfg_flags, GAPC_SVC_PREF_CON_PAR_PRES))
        {
            handle -= 2;
        }
    }

    return handle;
}

/**
 ****************************************************************************************
 * @brief Convert attribute handle to attribute index
 *
 * @param[in] att_idx       Attribute handle
 *
 * @return Attribute index
 ****************************************************************************************
 */
uint8_t gapc_svc_hdl_idx_get(uint16_t hdl)
{
    uint16_t att_idx = GAP_IDX_NUMBER;

    // GATT service handle range
    if((hdl >= gapc_env.svc.gatt_start_hdl) && (hdl < (gapc_env.svc.gatt_start_hdl + GATT_NB_ATT)))
    {
        att_idx = hdl - gapc_env.svc.gatt_start_hdl;
    }
    // GAP service handle range
    else if((hdl >= gapc_env.svc.gap_start_hdl) && (hdl < (gapc_env.svc.gap_start_hdl + GAP_NB_ATT)))
    {
        att_idx = hdl - gapc_env.svc.gap_start_hdl + GATT_IDX_NUMBER;

        // update attribute index according to supported features
        if((att_idx >= GAP_IDX_RSLV_PRIV_ADDR_ONLY) && !GETB(gapc_env.cfg_flags, GAPC_SVC_RSLV_PRIV_ADDR_ONLY_PRES))
        {
            att_idx += 2;
        }
        if((att_idx >= GAP_IDX_CHAR_SLAVE_PREF_PARAM) && !GETB(gapc_env.cfg_flags, GAPC_SVC_PREF_CON_PAR_PRES))
        {
            att_idx += 2;
        }
    }

    return att_idx;
}


/**
 ****************************************************************************************
 * @brief This function is called when GATT server user has initiated event send to peer
 *        device or if an error occurs.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] status        Status of the procedure (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC void gapc_svc_cb_event_sent(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status)
{
    if(conidx != GAP_INVALID_CONIDX)
    {
        // Service change handling.
        gapc_con_t* p_con = gapc_get_con_env(conidx);

        // Check if database updated since last request
        if((status == GAP_ERR_NO_ERROR) && (p_con != NULL) && !CO_BIT_GET(gapc_env.svc.cli_chg_aware_bf, conidx))
        {
            // mark that peer device is aware that database changed
            CO_BIT_SET(gapc_env.svc.cli_chg_aware_bf,       conidx, true);
            CO_BIT_SET(gapc_env.svc.cli_att_req_allowed_bf, conidx, true);

            // Inform application that client is change aware
            gapc_bond_info_send(conidx);
        }
    }
    else
    {
        // procedure is over
        gapc_env.svc.svc_chg_ind_token = GAP_INVALID_TOKEN;
    }
}

/**
 ****************************************************************************************
 * @brief Send Read confirm value
 *
 * @param[in]  conidx       Connection index
 * @param[in]  user_lid     GATT User Local identifier
 * @param[in]  token        Procedure token provided in corresponding callback
 * @param[in]  status       Status of attribute value get (see enum #hl_err)
 * @param[in]  total_length total attribute value length
 * @param[in]  length       value length
 * @param[in]  p_value      Pointer to the value
 ****************************************************************************************
 */
__STATIC uint16_t gapc_svc_read_cfm_send(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t status,
                                         uint16_t total_length, uint16_t length, const uint8_t* p_value)
{
    uint16_t ret_status;
    co_buf_t* p_data = NULL;

    if(status == GAP_ERR_NO_ERROR)
    {
        // allocate buffer that contains response
        if(co_buf_alloc(&p_data, GATT_BUFFER_HEADER_LEN, length, GATT_BUFFER_TAIL_LEN) != CO_BUF_ERR_NO_ERROR)
        {
            status = ATT_ERR_INSUFF_RESOURCE;
        }
        else
        {
            co_buf_copy_data_from_mem(p_data, p_value, length);
        }
    }

    // Immediately confirm value.
    ret_status = gatt_srv_att_read_get_cfm(conidx, user_lid, token, status, total_length, p_data);

    if(p_data!= NULL)
    {
       co_buf_release(p_data);
    }

    return (ret_status);
}

/**
 ****************************************************************************************
 * @brief This function is called when peer want to read local attribute database value.
 *
 *        @see gatt_srv_att_read_get_cfm shall be called to provide attribute value
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] token         Procedure token that must be returned in confirmation function
 * @param[in] hdl           Attribute handle
 * @param[in] offset        Data offset
 * @param[in] max_length    Maximum data length to return
 ****************************************************************************************
 */
__STATIC void gapc_svc_cb_att_read_get(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t hdl, uint16_t offset,
                                      uint16_t max_length)
{
    // Connection Index
    uint16_t status   = GAP_ERR_NO_ERROR;
    uint16_t length   = 0;
    bool     send_cfm = true;
    uint8_t  att_idx  = gapc_svc_hdl_idx_get(hdl);
    uint8_t  value[sizeof(uint16_t)]; // Size = Worst case
    const uint8_t* p_value = &(value[0]);
    gapc_con_t* p_con = gapc_get_con_env(conidx);

    switch(att_idx)
    {
        case GATT_IDX_SVC_CHANGED_CFG:
        {
            length = sizeof(uint16_t);
            co_write16p(value, (CO_BIT_GET(gapc_env.svc.svc_chg_ccc_bf, conidx) ? GATT_CCC_START_IND
                                                                                : GATT_CCC_STOP_NTFIND));
        } break;

        case GATT_IDX_CLI_SUP_FEAT:
        {
            length   = sizeof(uint8_t);
            value[0] = p_con->bond.cli_feat;
        } break;

        case GATT_IDX_DB_HASH:
        {
            status = gatt_db_hash_get(conidx, user_lid, token, &gapc_svc_hash_cb);
            if(status == GAP_ERR_NO_ERROR)
            {
                send_cfm = false;
            }
        } break;

        case GATT_IDX_SRV_SUP_FEAT:
        {
            length   = sizeof(uint8_t);
            value[0] = GAPC_SRV_EATT_SUPPORTED_BIT; // Mark Enhanced Attribute supported
        } break;

        #if (HL_LE_PERIPHERAL)
        case GAP_IDX_SLAVE_PREF_PARAM:
        {
            send_cfm = false;
            gapc_env.p_info_cbs->slave_pref_param_get(conidx, p_con->dummy, token);
        } break;
        #endif /* (HL_LE_PERIPHERAL) */

        case GAP_IDX_DEVNAME:
        {
            uint8_t name_len;
            const uint8_t* p_name = gapm_get_name(&name_len);

            // use name configured by application
            if(p_name != NULL)
            {
                if(offset > name_len)
                {
                    status = ATT_ERR_INVALID_OFFSET;
                    break;
                }

                p_value = &(p_name[offset]);
                length  = co_min(max_length, name_len - offset);
            }
            // ask directly application
            else
            {
                send_cfm = false;
                gapc_env.p_info_cbs->name_get(conidx, p_con->dummy, token, offset, max_length);
            }
        } break;

        case GAP_IDX_ICON:
        {
            send_cfm = false;
            gapc_env.p_info_cbs->appearance_get(conidx, p_con->dummy, token);
        } break;

        case GAP_IDX_RSLV_PRIV_ADDR_ONLY:
        {
            length = sizeof(uint8_t);
            value[0] = 0x00; // Use RPA after pairing
        } break;

        case GAP_IDX_CNT_ADDR_RESOL:
        {
            length = sizeof(uint8_t);
            value[0] = (gapm_is_controler_privacy_enabled() ? 1 : 0);
        } break;

        default: { status = ATT_ERR_APP_ERROR; } break;
    }

    if(send_cfm)
    {
        gapc_svc_read_cfm_send(conidx, user_lid, token, status, length, length, p_value);
    }
}


/**
 ****************************************************************************************
 * @brief This function is called during a write procedure to get information about a
 *        specific attribute handle.
 *
 *        @see gatt_srv_att_info_get_cfm shall be called to provide attribute information
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] token         Procedure token that must be returned in confirmation function
 * @param[in] hdl           Attribute handle
 ****************************************************************************************
 */
__STATIC void gapc_svc_cb_att_info_get(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t hdl)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    uint16_t length = 0;

    // retrieve attribute length information
    switch(gapc_svc_hdl_idx_get(hdl))
    {
        case GAP_IDX_DEVNAME:          { length = 0;                 } break; /* force write to start from 0 */
        default:                       { status = ATT_ERR_APP_ERROR; } break;
    }

    // return value to GATT
    gatt_srv_att_info_get_cfm(conidx, user_lid, token, status, length);
};

/**
 ****************************************************************************************
 * @brief This function is called during a write procedure to modify attribute handle.
 *
 *        @see gatt_srv_att_val_set_cfm shall be called to accept or reject attribute
 *        update.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] token         Procedure token that must be returned in confirmation function
 * @param[in] hdl           Attribute handle
 * @param[in] offset        Data offset
 * @param[in] p_data        Pointer to buffer that contains data to write starting from offset
 ****************************************************************************************
 */
__STATIC void gapc_svc_cb_att_val_set(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t hdl, uint16_t offset,
                                     co_buf_t* p_data)
{
    // Status
    uint16_t status   = ATT_ERR_APP_ERROR;
    uint8_t  att_idx  = gapc_svc_hdl_idx_get(hdl);
    uint16_t data_len = co_buf_data_len(p_data);
    bool send_cfm = true;
    gapc_con_t* p_con = gapc_get_con_env(conidx);

    switch(att_idx)
    {
        case GATT_IDX_SVC_CHANGED_CFG:
        {
            // Get the received value
            uint16_t ccc_value = co_read16p(co_buf_data(p_data));
            if(data_len != sizeof(uint16_t))
            {
                break;
            }

            // Valid values for the descriptor are:
            //    - 0x0000: Disable sending of indications
            //    - 0x0002: Enable sending of indications
            if ((ccc_value == GATT_CCC_STOP_NTFIND) || (ccc_value == GATT_CCC_START_IND))
            {
                // only update service changed information if something has changed
                if(CO_BIT_GET(gapc_env.svc.svc_chg_ccc_bf, conidx) != (ccc_value == GATT_CCC_START_IND))
                {
                    // set value
                    CO_BIT_SET(gapc_env.svc.svc_chg_ccc_bf, conidx, (ccc_value == GATT_CCC_START_IND));

                    // Inform the application about the new configuration
                    gapc_bond_info_send(conidx);
                }

                status = GAP_ERR_NO_ERROR;
            }
        } break;

        case GATT_IDX_CLI_SUP_FEAT:
        {
            uint8_t value = co_buf_data(p_data)[0] & GAPC_CLI_FEAT_SUPPORTED_MASK;
            uint8_t transition = (value ^ p_con->bond.cli_feat);

            // Reject bit transition to zero  (OLD ^ NEW) & ~(NEW)
            if((transition & ~value))
            {
                status = ATT_ERR_VALUE_NOT_ALLOWED;
                break;
            }

            // update client feature bits
            p_con->bond.cli_feat = value;

            // client requests to enable robust cache
            if (GETB(transition, GAPC_CLI_ROBUST_CACHE_EN))
            {
                // Mark robust cache supported
                CO_BIT_SET(gapc_env.svc.cli_chg_aware_bf,          conidx, true);
                CO_BIT_SET(gapc_env.svc.cli_att_req_allowed_bf,    conidx, true);
            }

            // if EATT is supported, authorize L2CAP COC negotiations
            if(GETB(transition, GAPC_CLI_EATT_SUPPORTED))
            {
                l2cap_coc_enhanced_nego_set(conidx, true);
            }

            if(transition != 0)
            {
                // Inform the application about the new configuration
                gapc_bond_info_send(conidx);
            }

            status = GAP_ERR_NO_ERROR;
        } break;
        case GAP_IDX_DEVNAME:
        {
            send_cfm = false;
            gapc_env.p_info_cbs->name_set(conidx, p_con->dummy, token, p_data);
        } break;
        // Appearance modified
        case GAP_IDX_ICON:
        {
            // length sanity check
            if(data_len != sizeof(uint16_t))
            {
                status = ATT_ERR_INVALID_ATTRIBUTE_VAL_LEN;
                break;
            }

            send_cfm = false;
            gapc_env.p_info_cbs->appearance_set(conidx, p_con->dummy, token, co_btohs(co_read16p(co_buf_data(p_data))));
        } break;

        default: { /* Nothing to do */ } break;
    }

    if(send_cfm)
    {
        // send back attribute modification status
        gatt_srv_att_val_set_cfm(conidx, user_lid, token, status);
    }
}

/**
 ****************************************************************************************
 * @brief This function is called when hash value for local attribute database hash has
 *        been computed.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution.
 * @param[in] status        Status of the operation (see enum #hl_err)
 * @param[in] p_hash        Pointer to the 128-bit database hash value
 ****************************************************************************************
 */
__STATIC void gapc_svc_cb_hash(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status, const uint8_t* p_hash)
{
    // Check if database updated since last request
    if(!CO_BIT_GET(gapc_env.svc.cli_chg_aware_bf, conidx))
    {
        // mark that peer device is authorized to do an ATT Request
        CO_BIT_SET(gapc_env.svc.cli_att_req_allowed_bf, conidx, true);
    }

    // send back database hash
    gapc_svc_read_cfm_send(conidx, user_lid, dummy, status, GATT_DB_HASH_LEN, GATT_DB_HASH_LEN, p_hash);
}


/**
 ****************************************************************************************
 * @brief Handle execution of Service Changed indication transmission
 *
 * @param[in] p_djob  Pointer to Delayed job structure.
 * @param[in] dummy   not used
 ****************************************************************************************
 */
__STATIC void gapc_svc_db_updated_defer(co_djob_t* p_djob)
{
    co_buf_t* p_data = gapc_env.svc.p_svc_chg_ind_buf;
    gapc_svc_chg_buf_meta_t* p_buf_meta = (gapc_svc_chg_buf_meta_t*) co_buf_metadata(p_data);
    uint8_t* p_val = co_buf_data(p_data);
    uint32_t conidx_bf = p_buf_meta->conidx_bf;
    gapc_env.svc.svc_chg_ind_token = gapm_token_id_get();
    gapc_env.svc.p_svc_chg_ind_buf = NULL;

    co_write16p(&(p_val[0]), co_htobs(p_buf_meta->start_hdl));
    co_write16p(&(p_val[2]), co_htobs(p_buf_meta->end_hdl));
    gatt_srv_event_mtp_send(conidx_bf, gapc_env.svc.user_lid, gapc_env.svc.svc_chg_ind_token,
                            GATT_INDICATE, gapc_svc_hdl_get(GATT_IDX_SVC_CHANGED), p_data, false);
    co_buf_release(p_data);
}
/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */
uint16_t gapc_svc_setup(uint16_t att_cfg, uint16_t gapc_svc_start_hdl, uint16_t gatt_svc_start_hdl, bool central_res_en)
{
    uint16_t status;

    do
    {
        uint32_t db_feat = GAPC_SVC_DEFAULT_FEAT;
        // initialize Start Handle
        gapc_env.svc.gap_start_hdl  = gapc_svc_start_hdl;
        gapc_env.svc.gatt_start_hdl = gatt_svc_start_hdl;
        SETB(gapc_env.cfg_flags, GAPC_SVC_PREF_CON_PAR_PRES, GETF(att_cfg, GAPM_ATT_SLV_PREF_CON_PAR_EN));
        SETB(gapc_env.cfg_flags, GAPC_DBG_BOND_INFO_TRIGGER, GETB(att_cfg, GAPM_DBG_BOND_INFO_TRIGGER));
        SETB(gapc_env.cfg_flags, GAPC_SVC_RSLV_PRIV_ADDR_ONLY_PRES, GETF(att_cfg, GAPM_ATT_RSLV_PRIV_ADDR_ONLY));

        if(GETF(att_cfg, GAPM_ATT_SLV_PREF_CON_PAR_EN))
        {
            db_feat |= GAPC_SVC_PERIPH_FEAT;
        }

        if(GETF(att_cfg, GAPM_ATT_RSLV_PRIV_ADDR_ONLY))
        {
            db_feat |= GAPC_SVC_RSLV_PRIV_ADDR_ONLY_FEAT;
        }

        if (central_res_en)
        {
            db_feat |= GAPC_SVC_CENTRAL_FEAT;
        }

        // Setup bearer for EATT establishment
        status = gatt_bearer_setup();
        if(status != GAP_ERR_NO_ERROR) break;

        // register service user with a minimum MTU and with maximum priority
        status = gatt_user_srv_register(L2CAP_LE_MTU_MIN, 255, &gapc_svc_cb, &gapc_env.svc.user_lid);
        if(status != GAP_ERR_NO_ERROR) break;

        // Add GAP service
        status = gatt_db_svc16_add(gapc_env.svc.user_lid, 0, GATT_SVC_GENERIC_ACCESS, GAP_NB_ATT,
                                   (uint8_t*) &db_feat, &(gapc_svc_db[GATT_IDX_NUMBER]), GAP_NB_ATT, &(gapc_env.svc.gap_start_hdl));
        if(status != GAP_ERR_NO_ERROR) break;

        // Add GATT service
        status = gatt_db_svc16_add(gapc_env.svc.user_lid, 0, GATT_SVC_GENERIC_ATTRIBUTE, GATT_NB_ATT,
                                   NULL, &(gapc_svc_db[0]), GATT_NB_ATT, &(gapc_env.svc.gatt_start_hdl));
        if(status != GAP_ERR_NO_ERROR) break;

        // Set appearance characteristic write permissions
        if(GETF(att_cfg, GAPM_ATT_APPEARENCE_PERM) != GAPM_WRITE_DISABLE)
        {
            status = gatt_db_att_info_set(gapc_env.svc.user_lid, gapc_svc_hdl_get(GAP_IDX_ICON),
                                          PROP(RD) | PROP(WR) | SEC_LVL_VAL(WP, GETF(att_cfg, GAPM_ATT_APPEARENCE_PERM) - 1));
            if(status != GAP_ERR_NO_ERROR) break;
        }

        // Set device name write permission
        if(GETF(att_cfg, GAPM_ATT_NAME_PERM) != GAPM_WRITE_DISABLE)
        {
            status = gatt_db_att_info_set(gapc_env.svc.user_lid, gapc_svc_hdl_get(GAP_IDX_DEVNAME),
                                          PROP(RD) | PROP(WR) | SEC_LVL_VAL(WP, GETF(att_cfg, GAPM_ATT_NAME_PERM) - 1));
        }
    } while(0);

    return (status);
}

void gapc_svc_con_create(uint8_t conidx, bool bond_data_present, uint8_t cli_info, uint8_t cli_feat)
{
    gapc_con_t* p_con = gapc_get_con_env(conidx);

    if(bond_data_present)
    {
        p_con->bond.cli_feat = (cli_feat & GAPC_CLI_FEAT_SUPPORTED_MASK);

        // Update service bond data
        CO_BIT_SET(gapc_env.svc.svc_chg_ccc_bf,         conidx,  GETB(cli_info, GAPC_CLI_SVC_CHANGED_IND_EN));
        CO_BIT_SET(gapc_env.svc.cli_chg_aware_bf,       conidx, !GETB(cli_info, GAPC_CLI_DB_UPDATED));
        CO_BIT_SET(gapc_env.svc.cli_att_req_allowed_bf, conidx, !GETB(cli_info, GAPC_CLI_DB_UPDATED));

        // Database update and service changed enabled.
        if(GETB(cli_info, GAPC_CLI_DB_UPDATED) && GETB(cli_info, GAPC_CLI_SVC_CHANGED_IND_EN))
        {
            co_buf_t* p_data;

            // Send service change update indication to all active connection that has registered services events.
            if(co_buf_alloc(&p_data, GATT_BUFFER_HEADER_LEN, GATT_HANDLE_LEN*2, GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
            {
                uint8_t* p_val = co_buf_data(p_data);

                co_write16p(&(p_val[0]), co_htobs(GATT_MIN_HDL));
                co_write16p(&(p_val[2]), co_htobs(GATT_MAX_HDL));
                gatt_srv_event_send(conidx, gapc_env.svc.user_lid, gapm_token_id_get(),
                                    GATT_INDICATE, gapc_svc_hdl_get(GATT_IDX_SVC_CHANGED), p_data);
                co_buf_release(p_data);
            }
        }
    }
    else
    {
        p_con->bond.cli_feat = 0;
        CO_BIT_SET(gapc_env.svc.svc_chg_ccc_bf,         conidx,  false);
        CO_BIT_SET(gapc_env.svc.cli_chg_aware_bf,       conidx,  true);
        CO_BIT_SET(gapc_env.svc.cli_att_req_allowed_bf, conidx,  true);
    }
}


void gapc_svc_db_updated(uint16_t start_hdl, uint16_t end_hdl)
{
    if(gapc_env.svc.p_svc_chg_ind_buf == NULL)
    {
        co_buf_t* p_data = NULL;
        uint32_t conidx_bf = 0;
        uint8_t conidx;

        // check all connection
        for (conidx = 0 ; conidx < HOST_CONNECTION_MAX ; conidx++)
        {
            gapc_con_t* p_con = gapc_get_con_env(conidx);

            // ignore if not connected
            if(p_con == NULL) { continue; }

            // If robust caching or Service changed indication enabled
            if(   CO_BIT_GET(gapc_env.svc.svc_chg_ccc_bf,         conidx)
               || GETB(p_con->bond.cli_feat, GAPC_CLI_ROBUST_CACHE_EN))
            {
                // inform that peer device is not aware of database changes
                CO_BIT_SET(gapc_env.svc.cli_chg_aware_bf,          conidx, false);
                CO_BIT_SET(gapc_env.svc.cli_att_req_allowed_bf,    conidx, false);

                // mark that an indication must be sent
                if(CO_BIT_GET(gapc_env.svc.svc_chg_ccc_bf, conidx))
                {
                    CO_BIT_SET(&conidx_bf, conidx, true);
                }
                // Inform the application about the new configuration
                gapc_bond_info_send(conidx);
            }
        }

        // Send service change update indication to all active connection that has registered services events.
        if(   (conidx_bf != 0)
           && co_buf_alloc(&p_data, GATT_BUFFER_HEADER_LEN, GATT_HANDLE_LEN*2, GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
        {
            gapc_svc_chg_buf_meta_t* p_buf_meta = (gapc_svc_chg_buf_meta_t*) co_buf_metadata(p_data);
            p_buf_meta->start_hdl              = start_hdl;
            p_buf_meta->end_hdl                = end_hdl;
            p_buf_meta->conidx_bf              = conidx_bf;

            // cancel an on-going procedure
            if(gapc_env.svc.svc_chg_ind_token != GAP_INVALID_TOKEN)
            {
                gatt_srv_event_mtp_cancel(gapc_env.svc.user_lid, gapc_env.svc.svc_chg_ind_token);
                gapc_env.svc.svc_chg_ind_token = GAP_INVALID_TOKEN;
            }

            // keep buffer pointer
            gapc_env.svc.p_svc_chg_ind_buf = p_data;

            // defer update
            co_djob_init(&(p_buf_meta->defer), gapc_svc_db_updated_defer);
            co_djob_reg(CO_DJOB_LOW, &(p_buf_meta->defer));
        }
    }
    else // update start and end handles
    {
        gapc_svc_chg_buf_meta_t* p_buf_meta = (gapc_svc_chg_buf_meta_t*) co_buf_metadata(gapc_env.svc.p_svc_chg_ind_buf);

        if(start_hdl < p_buf_meta->start_hdl)
        {
            p_buf_meta->start_hdl = start_hdl;
        }
        if(end_hdl >  p_buf_meta->end_hdl)
        {
            p_buf_meta->end_hdl = end_hdl;
        }
    }
}

bool gapc_svc_is_cli_out_of_sync(uint8_t conidx, uint8_t code, uint16_t hdl)
{
    bool out_of_sync = false;
    gapc_con_t* p_con = gapc_get_con_env(conidx);

    // Database updated
    if(GETB(p_con->bond.cli_feat, GAPC_CLI_ROBUST_CACHE_EN) && !CO_BIT_GET(gapc_env.svc.cli_chg_aware_bf, conidx))
    {
        switch(code)
        {
            case L2CAP_ATT_MTU_REQ_OPCODE:
            case L2CAP_ATT_FIND_INFO_REQ_OPCODE:
            case L2CAP_ATT_FIND_BY_TYPE_REQ_OPCODE:
            case L2CAP_ATT_RD_BY_TYPE_REQ_OPCODE:
            case L2CAP_ATT_RD_BY_GRP_TYPE_REQ_OPCODE:
            case L2CAP_ATT_EXE_WR_REQ_OPCODE:
            {
                // Attribute with handle request not yet allowed (first attempt)
                if(!CO_BIT_GET(gapc_env.svc.cli_att_req_allowed_bf, conidx))
                {
                    break;
                }
            }
            // no break

            case L2CAP_ATT_RD_REQ_OPCODE:
            case L2CAP_ATT_RD_BLOB_REQ_OPCODE:
            case L2CAP_ATT_RD_MULT_REQ_OPCODE:
            case L2CAP_ATT_WR_REQ_OPCODE:
            case L2CAP_ATT_PREP_WR_REQ_OPCODE:
            case L2CAP_ATT_RD_MULT_VAR_REQ_OPCODE:
            {
                // Database out of sync error already transmitted or Database Hash already read
                if(CO_BIT_GET(gapc_env.svc.cli_att_req_allowed_bf, conidx))
                {
                    //Mark client Change aware
                    CO_BIT_SET(gapc_env.svc.cli_chg_aware_bf, conidx, true);
                    gapc_bond_info_send(conidx);
                    break;
                }

                // no need to send it next time
                CO_BIT_SET(gapc_env.svc.cli_att_req_allowed_bf, conidx, true);
            }
            // no break

            case L2CAP_ATT_HDL_VAL_NTF_OPCODE:
            case L2CAP_ATT_MULT_HDL_VAL_NTF_OPCODE:
            case L2CAP_ATT_WR_CMD_OPCODE:
            case L2CAP_ATT_WR_SIGNED_OPCODE:
            {
                out_of_sync = true;
            } break;

            case L2CAP_ATT_HDL_VAL_IND_OPCODE:
            {
                // Always authorize a Service change indication
                if(gapc_svc_hdl_idx_get(hdl) != GATT_IDX_SVC_CHANGED)
                {
                    out_of_sync = true;
                }
            } break;

            default: { /* Nothing to do */ } break;
        }
    }

    return (out_of_sync);
}


bool gapc_svc_is_cli_mult_ntf_supported(uint8_t conidx)
{
    bool supported = false;

    if(conidx < HOST_CONNECTION_MAX)
    {
        gapc_con_t* p_con = gapc_get_con_env(conidx);

        supported = ((p_con != NULL) && GETB(p_con->bond.cli_feat, GAPC_CLI_MULT_NTF_SUPPORTED));
    }

    return supported;
}


void gapc_svc_bond_data_get(uint8_t conidx, uint8_t* p_cli_info)
{
    *p_cli_info = 0;
    SETB(*p_cli_info, GAPC_CLI_SVC_CHANGED_IND_EN,  CO_BIT_GET(gapc_env.svc.svc_chg_ccc_bf, conidx));
    SETB(*p_cli_info, GAPC_CLI_DB_UPDATED,         !CO_BIT_GET(gapc_env.svc.cli_chg_aware_bf, conidx));
}


uint16_t gapc_info_name_get_cfm(uint8_t conidx, uint16_t token, uint16_t status, uint16_t complete_length,
                                uint8_t length, const uint8_t* p_name)
{
    // send back value to peer device
    return gapc_svc_read_cfm_send(conidx, gapc_env.svc.user_lid, token, status, complete_length, length,
                                  p_name);
}

uint16_t gapc_info_appearance_get_cfm(uint8_t conidx, uint16_t token, uint16_t status, uint16_t appearance)
{
    // send back value to peer device
    return gapc_svc_read_cfm_send(conidx, gapc_env.svc.user_lid, token, status, sizeof(uint16_t), sizeof(uint16_t),
                                  (uint8_t*) &appearance);
}

#if (HL_LE_PERIPHERAL)
uint16_t gapc_info_slave_pref_param_get_cfm(uint8_t conidx, uint16_t token, uint16_t status, gap_periph_pref_t pref)
{
    // prepare data in proper format
    pref.con_intv_min =  co_htobs(pref.con_intv_min);
    pref.con_intv_max =  co_htobs(pref.con_intv_max);
    pref.conn_timeout =  co_htobs(pref.conn_timeout);
    pref.latency      =  co_htobs(pref.latency);

    // send back value to peer device
    return gapc_svc_read_cfm_send(conidx, gapc_env.svc.user_lid, token, status, sizeof(gap_periph_pref_t),
                                  sizeof(gap_periph_pref_t), (uint8_t*) &pref);
}
#endif // (HL_LE_PERIPHERAL)

uint16_t gapc_info_name_set_cfm(uint8_t conidx, uint16_t token, uint16_t status)
{
    // provide back accept or reject of attribute value modification
    return gatt_srv_att_val_set_cfm(conidx, gapc_env.svc.user_lid, token, status);
}

uint16_t gapc_info_appearance_set_cfm(uint8_t conidx, uint16_t token, uint16_t status)
{
    // provide back accept or reject of attribute value modification
    return gatt_srv_att_val_set_cfm(conidx, gapc_env.svc.user_lid, token, status);
}

#endif // (BLE_GATT)

/// @} GAPC


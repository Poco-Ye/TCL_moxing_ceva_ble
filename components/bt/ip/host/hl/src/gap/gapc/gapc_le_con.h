/**
 ****************************************************************************************
 *
 * @file gapc_le_con.h
 *
 * @brief Generic Access Profile Controller Internal Header - LE Connection
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */
#ifndef _GAPC_LE_CON_H_
#define _GAPC_LE_CON_H_

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

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPE DECLARATIONS
 ****************************************************************************************
 */

/// Pairing Information
typedef struct gapc_le_smp_pair_proc gapc_le_smp_pair_proc_t;

/// GAPC_LE_SMP environment structure
typedef struct gapc_le_smp
{
    ///TRamsaction failed repeated attempt timer
    gapc_sdt_t  rep_attempt_timer;

    /// Repeated Attempt Timer value
    uint16_t    rep_att_timer_val;

    /// Encryption key size
    uint8_t     key_size;

    /// Contains the current state of the two timers needed in the GAPC_LE_SMP task
    ///      Bit 1 - Is Repeated Attempt Timer running
    ///      Bit 2 - Has task reached a SMP Timeout
    uint8_t     timer_state;

    /// SMP channel local identifier
    uint8_t     chan_lid;
} gapc_le_smp_t;

/// GAP controller connection variable structure.
typedef struct gapc_le_con
{
    /// base connection information
    gapc_con_t          hdr;
    /* Connection parameters to keep */
    /// Security Management Protocol environment variables
    gapc_le_smp_t       smp;
    /// BD Address used for the link that should be kept
    gap_bdaddr_t        src[GAPC_INFO_SRC_MAX];
    /// LE features 8-byte array supported by peer device
    uint8_t             peer_features[GAP_LE_FEATS_LEN];
    /// Channel Selection Algorithm
    uint8_t             chan_sel_algo;
    /// Activity index of the periodic sync - speed-up activity search
    uint8_t             past_actv_idx;
} gapc_le_con_t;


/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Retrieve Low Energy Connection environment
 *
 * @param[in] conidx    Connection index
 *
 * @return Pointer to connection structure if BT classic connection
 ****************************************************************************************
 */
__INLINE gapc_le_con_t* gapc_le_con_env_get(uint8_t conidx)
{
    gapc_con_t* p_con = gapc_get_con_env(conidx);
    // Only return Low Energy environment
    return ((p_con != NULL) && GETB(p_con->info_bf, GAPC_LE_CON_TYPE) ? (gapc_le_con_t*) p_con : NULL);
}

/// Clean-up SMP pairing procedure
void gapc_le_smp_pairing_proc_cleanup(gapc_le_con_t* p_con);

/// Create LE SMP
uint16_t gapc_le_smp_create(gapc_le_con_t* p_con, uint8_t conidx);

/// Provide passkey for a LE pairing
uint16_t gapc_le_pairing_provide_passkey(uint8_t conidx, bool accept, uint32_t passkey);
/// Provide number comparison for a LE pairing
uint16_t gapc_le_pairing_numeric_compare_rsp(uint8_t conidx, bool accept);

/// Provide connection parameters to le event clients
void gapc_le_event_client_provide_con_param(uint8_t conidx, const gap_le_con_param_t* p_param);

#endif // (BLE_GAPC)
/// @} GAPC

#endif /* _GAPC_LE_CON_H_ */

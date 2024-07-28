/**
 ****************************************************************************************
 *
 * @file gapc_bt_msg.h
 *
 * @brief Generic Access Profile Controller  Message API. - BT-Classic
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */
#ifndef _GAPC_BT_MSG_H_
#define _GAPC_BT_MSG_H_


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "gapc_msg.h"
#include "gapc_bt.h"

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// @addtogroup GAPC_MSG_STRUCT_API Message Structures
/// @ingroup GAPC_MSG_API
/// @{


/// Indicate that a BT classic connection has been established
/*@TRACE*/
struct gapc_bt_connection_req_ind
{
    /// Connection index
    uint8_t      conidx;
    /// Connection handle
    uint16_t     conhdl;
    /// Peer BT address
    gap_bdaddr_t peer_addr;
    ///  Is initiator of the connection (1 = Initiator / 0 = Responder)
    bool         is_initator;
};


/// Parameter of the #GAPC_BT_SET_REQ_SEC_LVL_CMD message
/*@TRACE*/
struct gapc_bt_set_req_sec_lvl_cmd
{
    /// Connection index
    uint8_t conidx;
    /// GAP request type:
    /// - #GAPC_BT_SET_REQ_SEC_LVL
    uint8_t operation;
    /// Required security level (see enum #gap_sec_lvl)
    uint8_t sec_lvl;
};


/// Parameter of the #GAPC_BT_ENCRYPT_REQ_IND message
/*@TRACE*/
struct gapc_bt_encrypt_req_ind
{
    /// Connection index
    uint8_t conidx;
};

/// Parameter of the #GAPC_BT_ENCRYPT_CFM message
/*@TRACE*/
struct gapc_bt_encrypt_cfm
{
    /// Connection index
    uint8_t       conidx;
    /// Indicate if a link key has been found for the peer device
    bool          found;
    /// BT-Classic link key value
    gap_sec_key_t link_key;
};


/// @} GAPC_MSG_STRUCT_API

#endif /* _GAPC_BT_MSG_H_ */

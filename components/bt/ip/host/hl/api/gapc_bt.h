/**
 ****************************************************************************************
 *
 * @file gapc_bt.h
 *
 * @brief Generic Access Profile Controller - BT-Classic API.
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */


#ifndef _GAPC_BT_H_
#define _GAPC_BT_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "gapc.h"


/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/*
 * CALLBACK DEFINITIONS
 ****************************************************************************************
 */
/*
 * MACROS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/// @addtogroup GAPC_CON_REQ_API
/// @{

/**
 ****************************************************************************************
 * @brief Upper layer SW confirmation of Bluetooth classic link creation with bond data if available.
 *
 * @param[in] conidx    Connection index
 * @param[in] dummy     Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] p_data    Pointer to bond data if present, NULL otherwise
 *
 * @return Return function execution status (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapc_bt_connection_cfm(uint8_t conidx, uint32_t dummy, const gapc_bond_data_t* p_data);
/// @} GAPC_CON_REQ_API


/// @addtogroup GAPC_CON_INFO_API
/// @{


/**
 ****************************************************************************************
 * @brief Get if connection is a BT-Classic connection.
 *
 * @param[in] conidx Connection index
 *
 * @return Return true if connection is an BT-Classic connection; false otherwise.
 ****************************************************************************************
 */
bool gapc_is_bt_connection(uint8_t conidx);
/// @} GAPC_CON_INFO_API

#endif /* _GAPC_LE_H_ */

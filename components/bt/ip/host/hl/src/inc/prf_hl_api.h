/**
 ****************************************************************************************
 *
 * @file PRF_hl_api.h
 *
 * @brief Header file - PRF Internal HL API
 *
 * Copyright (C) RivieraWaves 2009-2020
 ****************************************************************************************
 */

#ifndef PRF_HL_API_H_
#define PRF_HL_API_H_

/**
 ****************************************************************************************
 * @addtogroup PRF
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#if (HOST_PROFILES)
#include "prf.h"
#if (HOST_MSG_API)
#include "ke_task.h"
#endif // (HOST_MSG_API)

/*
 * MACRO DEFINITIONS
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

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Perform Profile initialization
 *
 * @param[in] init_type  Type of initialization (see enum #rwip_init_type)
 ****************************************************************************************
 */
void prf_initialize(uint8_t init_type);

/**
 ****************************************************************************************
 * @brief Link creation event, update profiles states.
 *
 * @param[in] conidx        connection index
 * @param[in] is_le_con     True if it's a BLE connection, False if it's a BT-Classic connection
 ****************************************************************************************
 */
void prf_con_create(uint8_t conidx, bool is_le_con);

/**
 ****************************************************************************************
 * @brief Link disconnection event, clean-up profiles.
 *
 * @param[in] conidx        connection index
 * @param[in] reason        detach reason
 *
 ****************************************************************************************
 */
void prf_con_cleanup(uint8_t conidx, uint8_t reason);

#if (HOST_MSG_API)

/**
 ****************************************************************************************
 * @brief Initialize application message interface
 *
 * @param[in] api_id   Application identifier (see enum #TASK_API_ID)
 * @param[in] app_task Application task number
 *
 * @return Profile task number
 ****************************************************************************************
 */
ke_msg_id_t prf_msg_api_init(uint8_t api_id, ke_msg_id_t app_task);

#endif // (HOST_MSG_API)

#endif // (HOST_PROFILES)

/// @} PRF

#endif // PRF_HL_API_H_

/**
 ****************************************************************************************
 * @file gatt_user.c
 *
 * @brief  GATT User Manager
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GATT
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"    // IP configuration
#if (BLE_GATT)
#include "gatt.h"           // Native API
#include "gatt_int.h"       // Internals

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */


#if (BLE_GATT_CLI)
uint16_t gatt_user_cli_register(uint16_t pref_mtu, uint8_t prio_level, const gatt_cli_cb_t* p_cb, uint8_t* p_user_lid)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    uint8_t cursor;
    // Check parameters
    if((p_user_lid == NULL) || (p_cb == NULL))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else
    {
        // search for available user
        for(cursor = 0 ; cursor < BLE_GATT_USER_NB ; cursor++)
        {
            // fill user information
            if(gatt_env.users[cursor].p_cb == NULL)
            {
                gatt_env.users[cursor].p_cb     = (const gatt_user_cb_t*) p_cb;
                gatt_env.users[cursor].pref_mtu = pref_mtu;
                gatt_env.users[cursor].prio     = prio_level;
                gatt_env.users[cursor].role     = GATT_ROLE_CLIENT;
                *p_user_lid                     = cursor;
                status                          = GAP_ERR_NO_ERROR;
                break;
            }
        }
    }

    return (status);
}
#else  // !(BLE_GATT_CLI)
uint16_t gatt_user_cli_register(uint16_t pref_mtu, uint8_t prio_level, const gatt_cli_cb_t* p_cb, uint8_t* p_user_lid)
{
    return (GAP_ERR_NOT_SUPPORTED);
}
#endif // (BLE_GATT_CLI)

uint16_t gatt_user_srv_register(uint16_t pref_mtu, uint8_t prio_level, const gatt_srv_cb_t* p_cb, uint8_t* p_user_lid)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    uint8_t cursor;

    // Check parameters
    if(   (p_user_lid == NULL) || (p_cb == NULL) || (p_cb->cb_att_read_get == NULL)
       || (p_cb->cb_att_val_set == NULL))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else
    {
        // search for available user
        for(cursor = 0 ; cursor < BLE_GATT_USER_NB ; cursor++)
        {
            // fill user information
            if(gatt_env.users[cursor].p_cb == NULL)
            {
                gatt_env.users[cursor].p_cb     = (const gatt_user_cb_t*) p_cb;
                gatt_env.users[cursor].pref_mtu = pref_mtu;
                gatt_env.users[cursor].prio     = prio_level;
                gatt_env.users[cursor].role     = GATT_ROLE_SERVER;
                *p_user_lid                     = cursor;
                status                          = GAP_ERR_NO_ERROR;
                break;
            }
        }
    }

    return (status);
}

uint16_t gatt_user_unregister(uint8_t user_lid)
{
    uint16_t status;

    if((user_lid >= BLE_GATT_USER_NB) || (gatt_env.users[user_lid].p_cb == NULL))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    // check that there is no procedure on-going
    else if(gatt_proc_is_user_active(user_lid))
    {
        status = GAP_ERR_COMMAND_DISALLOWED;
    }
    else
    {
        if(gatt_env.users[user_lid].role == GATT_ROLE_SERVER)
        {
            // remove all services associated to a GATT user
            gatt_db_svc_remove_user(user_lid);
        }
        #if (BLE_GATT_CLI)
        else // if(gatt_env.users[user_lid].role == GATT_ROLE_CLIENT)
        {
            // remove all events registered to current client
            gatt_cli_event_remove_user(user_lid);
        }
        #endif // (BLE_GATT_CLI)

        // Mark gatt user removed
        gatt_env.users[user_lid].p_cb = NULL;
        gatt_env.users[user_lid].role = GATT_ROLE_NONE;

        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

gatt_user_t* gatt_user_get(uint8_t user_lid)
{
    gatt_user_t* p_user = NULL;

    if((user_lid < BLE_GATT_USER_NB) && (gatt_env.users[user_lid].role != GATT_ROLE_NONE))
    {
        p_user = &(gatt_env.users[user_lid]);
    }

    return (p_user);
}

uint16_t gatt_user_pref_mtu_get(void)
{
    uint16_t pref_mtu = L2CAP_COC_MTU_MIN;
    uint8_t  cursor;

    // search for available user
    for(cursor = 0 ; cursor < BLE_GATT_USER_NB ; cursor++)
    {
        // find greatest preferred MTU
        if((gatt_env.users[cursor].p_cb != NULL) && (gatt_env.users[cursor].pref_mtu > pref_mtu))
        {
            pref_mtu = gatt_env.users[cursor].pref_mtu;
        }
    }

    return (pref_mtu);
}

uint8_t gatt_user_cli_nb_get(void)
{
    uint8_t cli_nb = 0;
    uint8_t cursor;

    // search for available user
    for(cursor = 0 ; cursor < BLE_GATT_USER_NB ; cursor++)
    {
        // count number of clients
        if((gatt_env.users[cursor].p_cb != NULL) && (gatt_env.users[cursor].role == GATT_ROLE_CLIENT))
        {
            cli_nb += 1;
        }
    }

    return (cli_nb);
}


#if (HOST_MSG_API)
/*
 * MESSAGE HANDLER FUNCTIONS
 ****************************************************************************************
 */
#include "gatt_msg_int.h"

#if (BLE_GATT_CLI)
/// Client call-back for message handlers
extern const gatt_cli_cb_t gatt_cli_msg_cb;
#endif // (BLE_GATT_CLI)
/// Server call-back for message handlers
extern const gatt_srv_cb_t gatt_srv_msg_cb;

/**
 ****************************************************************************************
 * @brief Handle registration of a new GATT user
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_user_register_cmd_handler(gatt_user_register_cmd_t* p_cmd, uint16_t src_id)
{
    uint8_t  user_lid = GATT_INVALID_USER_LID;
    uint16_t status;

    // Add a client
    if(p_cmd->role == GATT_ROLE_CLIENT)
    {
        #if (BLE_GATT_CLI)
        status = gatt_user_cli_register(p_cmd->pref_mtu, p_cmd->prio_level, &gatt_cli_msg_cb, &(user_lid));
        #else // !(BLE_GATT_CLI)
        status = GAP_ERR_NOT_SUPPORTED;
        #endif // (BLE_GATT_CLI)
    }
    // Add a server
    else if(p_cmd->role == GATT_ROLE_SERVER)
    {
        status = gatt_user_srv_register(p_cmd->pref_mtu, p_cmd->prio_level, &gatt_srv_msg_cb, &(user_lid));
    }
    // Invalid role
    else
    {
        status   = GAP_ERR_INVALID_PARAM;
    }

    // if user register store task identifier
    if(status == GAP_ERR_NO_ERROR)
    {
        gatt_env.users[user_lid].dest_task_nbr = src_id;
    }

    // send message to host
    gatt_msg_send_basic_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, src_id, user_lid, status);
}

/**
 ****************************************************************************************
 * @brief Handle un-registration of an existing GATT user
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_user_unregister_cmd_handler(gatt_user_unregister_cmd_t* p_cmd, uint16_t src_id)
{
    // Unregister user
    uint16_t status = gatt_user_unregister(p_cmd->user_lid);

    // send message to host
    gatt_msg_send_basic_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, src_id, p_cmd->user_lid, status);
}

#endif //  (HOST_MSG_API)

#endif // (BLE_GATT)
/// @} GATT


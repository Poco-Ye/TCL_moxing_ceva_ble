/**
 ****************************************************************************************
 * @file gatt.c
 *
 * @brief  GATT Entry point
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

#if (HOST_MSG_API)
#include "ke_msg.h"         // Kernel message API
#include "ke_task.h"        // Kernel task API
#endif // (HOST_MSG_API)
#include "ke_mem.h"         // Memory allocation

#include "gap.h"            // HL defines
#include "gapc.h"           // Conversion from connection handle to connection index
#include "co_math.h"        // co_ctz usage

#include <string.h>         // For memset

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */
/// Number of instance of GATT task
#define GATT_IDX_MAX           (1)


/// Base UUID 128 (LSB First)
#define GATT_BT_UUID_128 {0xFB,0x34,0x9B,0x5F,0x80,0x00,0x00,0x80,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x00}

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// GATT Environment
gatt_env_t gatt_env;

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */



/*
 * BLE LINK Creation / Destruction
 ****************************************************************************************
 */
uint16_t gatt_create(uint8_t conidx)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    ASSERT_ERR(conidx < HOST_CONNECTION_MAX);
    ASSERT_ERR(gatt_env.p_con[conidx] == NULL);

    // Allocate Connection environemt
    gatt_con_env_t* p_con = (gatt_con_env_t*) ke_malloc_user(sizeof(gatt_con_env_t), KE_MEM_ENV);

    if(p_con != NULL)
    {
        gatt_env.p_con[conidx] = p_con;
        memset(p_con, 0, sizeof(gatt_con_env_t));

        // Create Default ATT bearer
        status = gatt_bearer_create(p_con, conidx);
    }

    return (status);
}

void gatt_cleanup(uint8_t conidx)
{
    gatt_con_env_t* p_con = gatt_env.p_con[conidx];

    if(p_con != NULL)
    {
        #if (BLE_GATT_CLI)
        // Free list of registered events
        if(p_con->p_reg_events != NULL)
        {
            ke_free(p_con->p_reg_events);
        }
        #endif // (BLE_GATT_CLI)

        // loop on all bearers
        while(p_con->bearer_bf != 0)
        {
            uint8_t bearer_lid = co_ctz(p_con->bearer_bf);
            gatt_bearer_close(conidx, bearer_lid, GAP_ERR_DISCONNECTED);
        }

        // remove environment
        ke_free(p_con);
        gatt_env.p_con[conidx] = NULL;
    }
    else
    {
        ASSERT_WARN(0, conidx, 0);
    }
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

bool gatt_uuid_comp(const uint8_t *p_uuid_a, uint8_t uuid_a_type, const uint8_t *p_uuid_b, uint8_t uuid_b_type)
{
    bool match_id  = false;

    // Both are 16 bits UUIDs
    if((uuid_a_type == GATT_UUID_16) && (uuid_b_type == GATT_UUID_16))
    {
        // compare both 16 bits uuids
        if(memcmp(p_uuid_a, p_uuid_b, GATT_UUID_16_LEN) == 0)
        {
            match_id = true;
        }
    }
    // Both are 32 bits  UUIDs
    else if((uuid_a_type == GATT_UUID_32) && (uuid_b_type == GATT_UUID_32))
    {
        // compare both 32 bits uuids
        if(memcmp(p_uuid_a, p_uuid_b, GATT_UUID_32_LEN) == 0)
        {
            match_id = true;
        }
    }
    // One of UUIDs is a 128 bits UUID
    else if((uuid_a_type == GATT_UUID_128) || (uuid_b_type == GATT_UUID_128))
    {
        uint8_t uuid128_a[GATT_UUID_128_LEN];
        uint8_t uuid128_b[GATT_UUID_128_LEN];

        // convert both uuid to 128 bits
        gatt_uuid128_convert(p_uuid_a, uuid_a_type, uuid128_a);
        gatt_uuid128_convert(p_uuid_b, uuid_b_type, uuid128_b);

        // compare both 128 bits uuids
        if(memcmp(uuid128_a, uuid128_b, GATT_UUID_128_LEN) == 0)
        {
            match_id = true;
        }
    }

    return match_id;
}

bool gatt_uuid16_comp(const uint8_t *p_uuid_a, uint8_t uuid_a_type, uint16_t uuid_b)
{
    return gatt_uuid_comp(p_uuid_a, uuid_a_type, (uint8_t*)&uuid_b, GATT_UUID_16);
}

bool gatt_is_uuid16(const uint8_t *p_uuid128)
{
    uint8_t uuid128_base[GATT_UUID_128_LEN] = GATT_BT_UUID_128;

    /* place the UUID on 12th and 13th location of UUID */
    memcpy(&(uuid128_base[12]), &(p_uuid128[12]), GATT_UUID_16_LEN);

    return (memcmp(uuid128_base, p_uuid128, GATT_UUID_128_LEN) == 0);
}

bool gatt_is_uuid32(const uint8_t *p_uuid128)
{
    uint8_t uuid128_base[GATT_UUID_128_LEN] = GATT_BT_UUID_128;

    /* place the UUID on last 4 bits location of 32 BITS UUID */
    memcpy(&(uuid128_base[12]), &(p_uuid128[12]), GATT_UUID_32_LEN);

    return (memcmp(uuid128_base, p_uuid128, GATT_UUID_128_LEN) == 0);
}

void gatt_uuid128_convert(const uint8_t *p_uuid, uint8_t uuid_type, uint8_t *p_uuid128)
{
    uint8_t uuid_len;
    uint8_t uuid128_base[GATT_UUID_128_LEN] = GATT_BT_UUID_128;
    uint8_t cursor;

    if(uuid_type == GATT_UUID_16)
    {
        // place the UUID on 12th to 13th location of UUID
        cursor    = 12;
        uuid_len  = GATT_UUID_16_LEN;
    }
    else if (uuid_type == GATT_UUID_32)
    {
        // place the UUID on 12th to 15th location of UUID
        cursor    = 12;
        uuid_len  = GATT_UUID_32_LEN;
    }
    else
    {
        // we consider it's 128 bits UUID
        cursor    = 0;
        uuid_len  = GATT_UUID_128_LEN;
    }

    /* place the UUID on 12th to 15th location of UUID */
    memcpy(&(uuid128_base[cursor]), p_uuid, uuid_len);

    /* update value */
    memcpy(&p_uuid128[0], &uuid128_base[0], GATT_UUID_128_LEN);
}


uint16_t gatt_uuid_extract(uint8_t *p_out_uuid, uint8_t* p_out_uuid_type, const uint8_t *p_in_uuid, uint8_t in_uuid_len)
{
    uint16_t status = GAP_ERR_NO_ERROR;

    if(in_uuid_len == GATT_UUID_16_LEN)
    {
        memcpy(p_out_uuid, p_in_uuid, GATT_UUID_16_LEN);
        *p_out_uuid_type  = GATT_UUID_16;
        memset(&(p_out_uuid[GATT_UUID_16_LEN]), 0, GATT_UUID_128_LEN - GATT_UUID_16_LEN);
    }
    else if (in_uuid_len == GATT_UUID_128_LEN)
    {
        uint8_t cursor = 0;
        if(gatt_is_uuid16(p_in_uuid))
        {
            // UUID is on 12th to 13th location of UUID_128
            cursor       = 12;
            in_uuid_len  = GATT_UUID_16_LEN;
            *p_out_uuid_type  = GATT_UUID_16;
        }
        else if (gatt_is_uuid32(p_in_uuid))
        {
            // UUID is on 12th to 15th location of UUID_128
            cursor       = 12;
            in_uuid_len  = GATT_UUID_32_LEN;
            *p_out_uuid_type  = GATT_UUID_32;
        }
        else
        {
            // full 128-bit UUID must be copied
            *p_out_uuid_type  = GATT_UUID_128;
        }

        // Copy uuid in output pointer
        memcpy(p_out_uuid, &(p_in_uuid[cursor]), in_uuid_len);
        memset(&(p_out_uuid[in_uuid_len]), 0, GATT_UUID_128_LEN - in_uuid_len);
    }
    else
    {
        // cannot extract UUID
        status = GAP_ERR_INVALID_PARAM;
    }

    return (status);
}

/*
 * Message Handlers
 ****************************************************************************************
 */


#if (HOST_MSG_API)
// functions present in gatt_msg.c
extern int gatt_default_msg_handler(ke_msg_id_t const msgid, void *p_param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gatt_cmd_msg_handler(ke_msg_id_t const msgid, gatt_cmd_t *p_cmd, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gatt_cfm_msg_handler(ke_msg_id_t const msgid, gatt_cfm_t *p_cfm, ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*
 * TASK DESCRIPTOR DEFINITIONS
 ****************************************************************************************
 */

/// Specifies the default message handlers
KE_MSG_HANDLER_TAB(gatt)
{
    // Note: all messages must be sorted in ID ascending order

    { GATT_CMD,                         (ke_msg_func_t) gatt_cmd_msg_handler               },
    { GATT_CFM,                         (ke_msg_func_t) gatt_cfm_msg_handler               },
    { KE_MSG_DEFAULT_HANDLER,           (ke_msg_func_t) gatt_default_msg_handler           },
};

/// Defines the place holder for the states of all the task instances.
ke_state_t gatt_state[GATT_IDX_MAX];

/// GATT task descriptor
const struct ke_task_desc TASK_DESC_GATT = {gatt_msg_handler_tab, gatt_state, GATT_IDX_MAX, ARRAY_LEN(gatt_msg_handler_tab)};
#endif // (HOST_MSG_API)

/*
 * STACK INITIALIZATION
 ****************************************************************************************
 */
void gatt_initialize(uint8_t init_type)
{
    switch(init_type)
    {
        case RWIP_INIT:
        {
            #if (HOST_MSG_API)
            // Create GATT task
            ke_task_create(TASK_GATT, &TASK_DESC_GATT);
            #endif // (HOST_MSG_API)
        } break;

        case RWIP_RST:
        {
            uint8_t conidx;
            // Clean-up database services
            gatt_db_svc_t* p_current_svc = gatt_env.p_db;
            gatt_db_svc_t* p_next_svc = NULL;

            ASSERT_WARN(BLE_GATT_CON_ENV_SIZE == (sizeof(gatt_con_env_t)),
                        BLE_GATT_CON_ENV_SIZE, sizeof(gatt_con_env_t));

            while((p_current_svc != NULL))
            {
                p_next_svc = p_current_svc->p_next;

                ke_free(p_current_svc);
                p_current_svc = p_next_svc;
            }

            for(conidx = 0 ; conidx < HOST_CONNECTION_MAX ; conidx++)
            {
                uint8_t bearer_lid;
                gatt_con_env_t* p_con = gatt_env.p_con[conidx];

                if(p_con == NULL) continue;

                // remove all on-going procedures
                while(!co_list_is_empty(&(p_con->proc_exe_queue)))
                {
                    ke_free(co_list_pop_front(&(p_con->proc_exe_queue)));
                }

                // remove all waiting procedures
                while(!co_list_is_empty(&(p_con->proc_wait_queue)))
                {
                    ke_free(co_list_pop_front(&(p_con->proc_wait_queue)));
                }

                // clean-up bearer memory
                for(bearer_lid = 0 ; bearer_lid < BLE_GATT_BEARER_PER_CON ; bearer_lid++)
                {
                    gatt_bearer_t* p_bearer = p_con->p_bearer[bearer_lid];
                    if(p_bearer != NULL)
                    {
                        ke_free(p_bearer);
                    }
                }

                #if (BLE_GATT_CLI)
                // remove registered events
                if(p_con->p_reg_events != NULL)
                {
                    ke_free(p_con->p_reg_events);
                }
                #endif // (BLE_GATT_CLI)

                ke_free(p_con);
            }
        }

        // no break

        case RWIP_1ST_RST:
        {
            uint8_t cursor;
            memset(&gatt_env, 0, sizeof(gatt_env_t));

            // initialize users
            for (cursor = 0 ; cursor < BLE_GATT_USER_NB ; cursor++)
            {
                gatt_env.users[cursor].role = GATT_ROLE_NONE;
            }
        } break;

        default: { /* Nothing to do */ } break;
    }
}


void gatt_con_info_set(uint8_t conidx, uint16_t con_interval, uint16_t con_latency)
{
    // 2 × (connSlaveLatency + 1) × connInterval (in microseconds)
    uint32_t mitigation_time = (con_interval*2500) * (1 + con_latency);
    // 2 × (connSlaveLatency + 1) × connInterval (in milliseconds)
    mitigation_time = CO_DIVIDE_ROUND(mitigation_time, 1000);

    if(mitigation_time < GATT_COLLISON_MITIGATON_TIME_MIN)
    {
        mitigation_time = GATT_COLLISON_MITIGATON_TIME_MIN;
    }
    else if (mitigation_time > GAPC_SDT_DURATION_MAX)
    {
        mitigation_time = GAPC_SDT_DURATION_MAX;
    }

    // update internal mitigation time
    gatt_env.p_con[conidx]->eatt_mitigation_time = (uint16_t) mitigation_time;
}

#endif // (BLE_GATT)
/// @} GATT


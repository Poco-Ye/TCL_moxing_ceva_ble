/**
 ****************************************************************************************
 * @file l2cap.c
 *
 * @brief  L2CAP Entry point for HCI and Native or Message API
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup L2CAP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"         // IP configuration
#if (BLE_L2CAP)
#include "l2cap.h"               // Native API
#include "l2cap_int.h"           // Internals
#include "../inc/l2cap_sig.h"    //  SIG Protocol and API
#include "../inc/l2cap_hl_api.h" // Channel un-register

#if (HOST_MSG_API)
#include "ke_task.h"             // Kernel task API
#endif // (HOST_MSG_API)

#include "ke_mem.h"              // Memory allocation

#include "hci.h"                 // HCI Interface

#include "gap.h"                 // HL defines
#include "gapc.h"                // Conversion from connection handle to connection index

#include <string.h>              // For memset

#include "co_math.h"             // For CO_BIT usage

#include "dbg.h"

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */
/// Number of instance of L2CAP task
#define L2CAP_IDX_MAX           (1)

/// Compute range of parameter structure
#define l2cap_rangeof(type, start_field, end_field)  (offsetof(type, end_field) - offsetof(type, start_field))

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// L2CAP Environment
l2cap_env_t l2cap_env;



/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */



/*
 * BLE LINK Creation / Destruction
 ****************************************************************************************
 */
uint16_t l2cap_create(uint8_t conidx, bool is_le_connection)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    l2cap_con_env_t* p_con;
    ASSERT_ERR((conidx < HOST_CONNECTION_MAX) && (l2cap_env.p_con[conidx] == NULL));

    // allocate environment
    p_con = (l2cap_con_env_t*) ke_malloc_user(sizeof(l2cap_con_env_t), KE_MEM_ENV);
    if(p_con != NULL)
    {
        memset(p_con, 0, sizeof(l2cap_con_env_t));

        p_con->nb_channel      = BLE_L2CAP_CHAN_PER_CON;
        p_con->rx_temp_exp_len = L2CAP_HEADER_LEN;
        p_con->rx_chan_lid     = L2CAP_INVALID_CHAN_LID;
        p_con->conidx          = conidx;

        #if(BT_HOST_PRESENT)
        SETB(p_con->state_bf, L2CAP_BT_CLASSIC_CONNECTION, !is_le_connection);
        #endif // (BT_HOST_PRESENT)

        // prepare L2CAP modules
        l2cap_env.p_con[conidx] = p_con;

        // initialize SIG
        l2cap_sig_create(p_con);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

void l2cap_cleanup(uint8_t conidx)
{
    l2cap_con_env_t* p_con = l2cap_env.p_con[conidx];
    l2cap_env.rx_flow_paused_bf &= ~CO_BIT(conidx);
    SETB(p_con->state_bf, L2CAP_DISCONNECTING, true);

    // clean-up L2CAP modules
    l2cap_sig_cleanup(p_con);

    // clean-up L2CAP channels
    l2cap_chan_cleanup(p_con);

    #if (HOST_MSG_API)
    {
        ASSERT_ERR((conidx < HOST_CONNECTION_MAX) && (l2cap_env.p_con[conidx] != NULL));

        while(!co_list_is_empty(&(p_con->msg_api_sdu_queue)))
        {
            // Retrieve SDUs to release
            co_buf_t* p_sdu = (co_buf_t*) co_list_pop_front(&(p_con->msg_api_sdu_queue));
            co_buf_release(p_sdu);
        }
    }
    #endif // (HOST_MSG_API)

    // free environment
    ke_free(l2cap_env.p_con[conidx]);
    l2cap_env.p_con[conidx] = NULL;
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

l2cap_con_env_t* l2cap_get_con_env(uint8_t conidx)
{
    return ((conidx < HOST_CONNECTION_MAX) ?  l2cap_env.p_con[conidx] : NULL);
}


/*
 * Message Handlers
 ****************************************************************************************
 */

#if (HOST_MSG_API)
// functions present in l2cap_msg.c
extern int l2cap_default_msg_handler(ke_msg_id_t const msgid, void *p_param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int l2cap_cmd_msg_handler(ke_msg_id_t const msgid, l2cap_cmd_t *p_cmd, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int l2cap_cfm_msg_handler(ke_msg_id_t const msgid, l2cap_cfm_t *p_cfm, ke_task_id_t const dest_id, ke_task_id_t const src_id);

/// Specifies the default message handlers
KE_MSG_HANDLER_TAB(l2cap)
{
    // Note: all messages must be sorted in ID ascending order

    { L2CAP_CMD,                        (ke_msg_func_t) l2cap_cmd_msg_handler               },
    { L2CAP_CFM,                        (ke_msg_func_t) l2cap_cfm_msg_handler               },
    { KE_MSG_DEFAULT_HANDLER,           (ke_msg_func_t) l2cap_default_msg_handler           },
};

/// Defines the place holder for the states of all the task instances.
ke_state_t l2cap_state[L2CAP_IDX_MAX];

/// L2CAP task descriptor
const struct ke_task_desc TASK_DESC_L2CAP = {l2cap_msg_handler_tab, l2cap_state, L2CAP_IDX_MAX, ARRAY_LEN(l2cap_msg_handler_tab)};
#endif // (HOST_MSG_API)


/*
 * STACK INITIALIZATION
 ****************************************************************************************
 */
void l2cap_initialize(uint8_t init_type)
{
    switch(init_type)
    {
        case RWIP_INIT:
        {
            ASSERT_WARN(BLE_L2CAP_CON_ENV_SIZE == (sizeof(l2cap_con_env_t)), BLE_L2CAP_CON_ENV_SIZE, sizeof(l2cap_con_env_t));

            // Check structure compatibility (inheritance)
            ASSERT_ERR(l2cap_rangeof(l2cap_chan_t, hdr, tx_cid) == l2cap_rangeof(l2cap_chan_coc_t, hdr, tx_cid));

            #if (HOST_MSG_API)
            // Create L2CAP task
            ke_task_create(TASK_L2CAP, &TASK_DESC_L2CAP);
            #endif // (HOST_MSG_API)
        } break;

        case RWIP_RST:
        {
            uint8_t cursor;

            // clean list of registered SPSM
            while(!co_list_is_empty(&(l2cap_env.reg_spsm)))
            {
                ke_free(co_list_pop_front(&(l2cap_env.reg_spsm)));
            }

            // perform clean-up of connection environment
            for(cursor = 0 ; cursor < HOST_CONNECTION_MAX ; cursor++)
            {
                uint8_t chan_lid;
                l2cap_con_env_t* p_con = l2cap_env.p_con[cursor];

                if(p_con == NULL) { continue; }

                // clean SIG procedure queue
                while(!co_list_is_empty(&(p_con->sig.req_proc_queue)))
                {
                    ke_free(co_list_pop_front(&(p_con->sig.req_proc_queue)));
                }

                // clean SIG Request queue
                while(!co_list_is_empty(&(p_con->sig.rsp_proc_queue)))
                {
                    ke_free(co_list_pop_front(&(p_con->sig.rsp_proc_queue)));
                }

                #if (!BLE_EMB_PRESENT)
                // Clean-up messages from receive queue
                l2cap_rx_queue_cleanup(p_con);
                #endif // (!BLE_EMB_PRESENT)

                for(chan_lid = 0 ; chan_lid < p_con->nb_channel ; chan_lid++)
                {
                    if(p_con->p_chan[chan_lid] != NULL)
                    {
                        ke_free(p_con->p_chan[chan_lid]);
                    }
                }

                // clean-up memory
                ke_free(p_con);
            }
        }
        // no break

        case RWIP_1ST_RST:
        {
            memset(&l2cap_env, 0, sizeof(l2cap_env_t));
            l2cap_env.hl_buf_nb_avail = BLE_L2CAP_NB_RX_BUF_AVAIL;
            co_djob_init(&(l2cap_env.le_tx_bg_job), l2cap_chan_tx_le_handler);
            co_djob_init(&(l2cap_env.rx_bg_job), l2cap_chan_rx_handler);

            #if (BT_HOST_PRESENT)
            co_djob_init(&(l2cap_env.bt_tx_bg_job), l2cap_chan_bt_tx_handler);
            #endif // (BT_HOST_PRESENT)
        } break;

        default: { /* Nothing to do */ } break;
    }
}

#endif // (BLE_L2CAP)
/// @} L2CAP


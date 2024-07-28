/**
 ****************************************************************************************
 * @file gatt_proc.c
 *
 * @brief  GATT Procedure Manager
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
#include "gapm.h"           // For token id generation
#include "gapc.h"           // check link level

#include "co_math.h"        // Mathematics
#include "ke_mem.h"         // Memory allocationS
#include <string.h>         // for memcmp and memcpy

#include "dbg.h"

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

#if (BLE_GATT_CLI)
// Event procedure handle defined in gatt_cli_event.c
extern uint16_t gatt_cli_event_proc_create(uint8_t conidx, uint8_t op_code, gatt_proc_pdu_handler_cb* p_pdu_hdl_cb, gatt_proc_handler_t** pp_proc);
#endif // (BLE_GATT_CLI)
// Event procedure handle defined in gatt_srv_discover.c
extern uint16_t gatt_srv_discover_proc_create(uint8_t conidx, uint8_t op_code, gatt_proc_pdu_handler_cb* p_pdu_hdl_cb, gatt_proc_handler_t** pp_proc);
// Event procedure handle defined in gatt_srv_read.c
extern uint16_t gatt_srv_read_proc_create(uint8_t conidx, uint8_t op_code, gatt_proc_pdu_handler_cb* p_pdu_hdl_cb, gatt_proc_handler_t** pp_proc);
// Event procedure handle defined in gatt_srv_write.c
extern uint16_t gatt_srv_write_proc_create(uint8_t conidx, uint8_t op_code, gatt_proc_pdu_handler_cb* p_pdu_hdl_cb, gatt_proc_handler_t** pp_proc);
extern uint16_t gatt_srv_write_exe_proc_create(uint8_t conidx, uint8_t op_code, gatt_proc_pdu_handler_cb* p_pdu_hdl_cb, gatt_proc_handler_t** pp_proc);
// Event procedure handle defined in gatt_srv_mtu_exch.c
extern uint16_t gatt_srv_mtu_exch_proc_create(uint8_t conidx, uint8_t op_code, gatt_proc_pdu_handler_cb* p_pdu_hdl_cb, gatt_proc_handler_t** pp_proc);


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


/**
 ****************************************************************************************
 * @brief Execute procedure continue callback
 *
 * @param[in] conidx     Connection index
 * @param[in] p_proc     Pointer to procedure to continue
 * @param[in] state      Operation transition state (see enum #gatt_proc_state)
 * @param[in] status     Execution status
 ****************************************************************************************
 */
__STATIC void gatt_proc_continue_exe(uint8_t conidx, struct gatt_proc* p_proc, uint8_t state, uint16_t status)
{
    SETB(p_proc->info_bf, GATT_PROC_IN_CONTINUE, true);
    p_proc->cb_continue(conidx, p_proc, state, status);
    SETB(p_proc->info_bf, GATT_PROC_IN_CONTINUE, false);

    // check if procedure must be clean up
    if(GETB(p_proc->info_bf, GATT_PROC_FREE))
    {
        ke_free(p_proc);
    }
}

/**
 ****************************************************************************************
 * @brief Function executed when an attribute transaction timeout is detected
 *        this automatically
 *
 * @param[in] p_hdl  Pointer of timer handle
 * @param[in] conidx Connection index
 ****************************************************************************************
 */
void gatt_proc_trans_timeout_cb(gapc_sdt_t* p_hdl, uint8_t conidx)
{
    // retrieve procedure
    uint8_t      bearer_lid;
    gatt_proc_t* p_proc;
    ASSERT_ERR(gatt_env.p_con[conidx] != NULL);

    // retrieve procedure
    p_proc = (gatt_proc_t *)((uint32_t)p_hdl - offsetof(gatt_proc_t, defer_timer));
    bearer_lid = p_proc->bearer_lid;
    // Inform that PDU transmission fails due to timeout
    gatt_proc_continue(conidx, p_proc, GATT_PROC_PDU_RX, GAP_ERR_TIMEOUT);
    // Close the bearer
    gatt_bearer_close(conidx, bearer_lid, GATT_ERR_ATT_BEARER_CLOSE);
}

/**
 ****************************************************************************************
 * @brief Function executed when a GATT procedure must be deferred in background
 *
 * @param[in] p_hdl  Pointer of timer handle
 * @param[in] conidx Connection index
 * @param[in] status continue status
 ****************************************************************************************
 */
void gatt_proc_continue_defer_cb(gapc_sdt_t* p_hdl, uint8_t conidx, uint16_t status)
{
    uint8_t state;
    DBG_FUNC_ENTER(gatt_proc_continue_defer_cb);
    // retrieve procedure
    gatt_proc_t* p_proc;
    ASSERT_ERR(gatt_env.p_con[conidx] != NULL);

    // retrieve procedure
    p_proc = (gatt_proc_t *)((uint32_t)p_hdl - offsetof(gatt_proc_t, defer_timer));

    DBG_SWDIAG(HL, GATT_PROC_START, 1);
    state = (GETF(p_proc->info_bf, GATT_PROC_STATE) == GATT_PROC_WAIT_START) ? GATT_PROC_START : GATT_PROC_USER_CFM;
    // update procedure state
    SETF(p_proc->info_bf, GATT_PROC_STATE, state);
    // Start procedure execution
    gatt_proc_continue_exe(conidx, p_proc, state, status);

    DBG_SWDIAG(HL, GATT_PROC_START, 0);
    DBG_FUNC_EXIT(gatt_proc_continue_defer_cb);
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t gatt_proc_create(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint8_t proc_id, uint16_t tx_length,
                          uint16_t proc_size, gatt_proc_cb continue_cb, gatt_proc_t** pp_proc)
{
    DBG_FUNC_ENTER(gatt_proc_create);
    uint16_t status = GAP_ERR_NO_ERROR;
    gatt_user_t* p_user = gatt_user_get(user_lid);
    uint8_t  role = GATT_ROLE_NONE;

    switch(GATT_PROC_TYPE_GET(proc_id))
    {
        case GATT_PROC_IND:
        case GATT_PROC_NTF: { role = GATT_ROLE_SERVER; } break;
        #if (BLE_GATT_CLI)
        case GATT_PROC_REQ:
        case GATT_PROC_CMD: { role = GATT_ROLE_CLIENT; } break;
        #endif // (BLE_GATT_CLI)
        default:            { ASSERT_ERR(0);   } break;
    }

    ASSERT_ERR(continue_cb != NULL);
    ASSERT_ERR(proc_size >= sizeof(gatt_proc_t));

    // Check if connection exists and
    if((conidx >= HOST_CONNECTION_MAX) || (gatt_env.p_con[conidx] == NULL))
    {
        status = GAP_ERR_COMMAND_DISALLOWED;
    }
    // At least one bearer must be available
    else if(gatt_env.p_con[conidx]->bearer_bf == 0)
    {
        status = GATT_ERR_NO_MORE_BEARER;
    }
    // check if user support command creation
    else if((p_user == NULL) || (p_user->role != role))
    {
        status = GAP_ERR_COMMAND_DISALLOWED;
    }
    else
    {
        gatt_proc_t* p_proc = (gatt_proc_t*) ke_malloc_user(proc_size, KE_MEM_KE_MSG);

        if(p_proc == NULL)
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
        else
        {
            // Fill procedure parameters
            p_proc->cb_continue           = continue_cb;
            p_proc->cb_pdu_handler        = NULL;
            p_proc->proc_id               = proc_id;
            // generate a specific token
            p_proc->token                 = tx_length;
            p_proc->dummy                 = dummy;
            p_proc->user_lid              = user_lid;
            p_proc->info_bf             = 0;
            SETB(p_proc->info_bf, GATT_PROC_USER_CFM, true);
            *pp_proc = p_proc;
        }
    }
    DBG_FUNC_EXIT(gatt_proc_create);

    return(status);
}


uint16_t gatt_proc_handler_alloc(uint8_t conidx, uint8_t proc_id, uint16_t proc_size, gatt_proc_cb continue_cb,
                                 gatt_proc_handler_t** pp_proc)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    gatt_proc_handler_t* p_proc;

    ASSERT_ERR(continue_cb != NULL);
    ASSERT_ERR(proc_size >= sizeof(gatt_proc_handler_t));

    p_proc = (gatt_proc_handler_t*) ke_malloc_user(proc_size, KE_MEM_KE_MSG);

    if(p_proc == NULL)
    {
        status = ATT_ERR_INSUFF_RESOURCE;
    }
    else
    {
        // Fill procedure parameters
        p_proc->cb_continue         = continue_cb;
        p_proc->info_bf             = 0;
        SETF(p_proc->info_bf, GATT_PROC_STATE, GATT_PROC_WAIT_GRANT);
        SETB(p_proc->info_bf, GATT_PROC_USER_CFM, true);
        p_proc->proc_id             = proc_id;
        p_proc->user_lid            = GATT_INVALID_USER_LID;
        // generate a specific token
        p_proc->token               = gapm_token_id_get();
        gapc_sdt_prepare(&(p_proc->defer), conidx, GAPC_SDT_GATT_PROC);

        *pp_proc = p_proc;
    }

    return(status);
}

void gatt_proc_push(uint8_t conidx, gatt_proc_t* p_proc)
{
    DBG_FUNC_ENTER(gatt_proc_push);
    gatt_con_env_t* p_con = gatt_env.p_con[conidx];
    gatt_user_t* p_user   = gatt_user_get(p_proc->user_lid);
    bool granted = false;
    ASSERT_ERR(p_user != NULL);

    // prepare procedure
    p_proc->bearer_lid = ((p_proc->proc_id == GATT_PROC_WRITE_SIGNED) || (p_proc->proc_id == GATT_PROC_MTU_EXCH))
                       ? 0 : GATT_INVALID_BEARER_LID;
    SETF(p_proc->info_bf, GATT_PROC_STATE, GATT_PROC_WAIT_GRANT);
    gapc_sdt_prepare(&(p_proc->defer_timer), conidx, GAPC_SDT_GATT_PROC);

    #if(BLE_GATT_CLI)
    // check if procedure can be granted (not mutex protected)
    if(   !GETB(p_con->state_bf, GATT_CON_WRITE_QUEUE_MUTEX)
       || ((p_proc->proc_id != GATT_PROC_WRITE_LONG) && (p_proc->proc_id != GATT_PROC_WRITE_EXE)))
    #endif // (BLE_GATT_CLI)
    {
        // Try to acquire a bearer for new procedure
        granted = gatt_bearer_acquire(conidx, GATT_INVALID_BEARER_LID, p_proc, p_user->pref_mtu);
    }

    // A bearer is granted, procedure execution will start in background
    if(granted)
    {
        // Put new procedure on front of execution queue
        co_list_push_front(&(p_con->proc_exe_queue), &(p_proc->hdr));
        gatt_proc_continue(conidx, p_proc, GATT_PROC_START, GAP_ERR_NO_ERROR);
    }
    else
    {
        uint8_t      new_prio   = p_user->prio;
        gatt_proc_t* p_proc_cur = (gatt_proc_t*) co_list_pick(&(p_con->proc_wait_queue));

        // put procedure on execution queue
        if(p_proc_cur == NULL)
        {
            co_list_push_front(&(p_con->proc_wait_queue), &(p_proc->hdr));
        }
        else
        {
            // insert new procedure according to user priority.
            while(p_proc_cur != NULL)
            {
                uint8_t cur_prio = gatt_user_get(p_proc_cur->user_lid)->prio;

                // if new priority is greater than current priority
                if(new_prio > cur_prio)
                {
                    // insert new procedure before current procedure
                    co_list_insert_before(&(p_con->proc_wait_queue), &(p_proc_cur->hdr), &(p_proc->hdr));
                    break;
                }

                p_proc_cur = (gatt_proc_t*) p_proc_cur->hdr.next;
            }

            // put procedure at end of wait queue
            if(p_proc_cur == NULL)
            {
                co_list_push_back(&(p_con->proc_wait_queue), &(p_proc->hdr));
            }
        }
    }

    DBG_FUNC_EXIT(gatt_proc_push);
}

void gatt_proc_pop(uint8_t conidx, gatt_proc_t* p_proc, bool free)
{
    DBG_FUNC_ENTER(gatt_proc_pop);
    gatt_con_env_t* p_con = gatt_env.p_con[conidx];
    ASSERT_ERR(p_con != NULL);

    DBG_SWDIAG(HL, GATT_PROC_END, 1);
    // remove from execution queue
    if(GETF(p_proc->info_bf, GATT_PROC_STATE) != GATT_PROC_WAIT_GRANT)
    {
        uint8_t proc_type = GATT_PROC_TYPE_GET(p_proc->proc_id);

        co_list_extract(&(p_con->proc_exe_queue), &(p_proc->hdr));
        gapc_sdt_stop(&(p_proc->defer_timer));

        // if procedure is GATT user initiated
        if((proc_type != GATT_PROC_CLI_HANDLER) && (proc_type != GATT_PROC_SRV_HANDLER))
        {
            // release bearer
            gatt_bearer_release(conidx, p_proc);
        }
    }
    // remove from wait queue
    else
    {
        co_list_extract(&(p_con->proc_wait_queue), &(p_proc->hdr));
    }

    SETB(p_proc->info_bf, GATT_PROC_FREE, free);

    DBG_SWDIAG(HL, GATT_PROC_END, 0);
    DBG_FUNC_EXIT(gatt_proc_pop);
}

gatt_proc_t* gatt_proc_pick(uint8_t conidx, uint16_t token)
{
    gatt_proc_t* p_proc = NULL;
    gatt_con_env_t* p_con = gatt_env.p_con[conidx];
    if(p_con != NULL)
    {
        p_proc = (gatt_proc_t*) co_list_pick(&(p_con->proc_exe_queue));
        ASSERT_ERR(p_con != NULL);

        // search procedure with a specific token
        while(p_proc != NULL)
        {
            if(p_proc->token == token)
            {
                break;
            }

            p_proc = (gatt_proc_t*) p_proc->hdr.next;
        }
    }

    return (p_proc);
}

gatt_proc_t* gatt_proc_find(uint8_t conidx, uint16_t dummy)
{
    gatt_proc_t* p_proc = NULL;
    if((conidx < HOST_CONNECTION_MAX) && (gatt_env.p_con[conidx] != NULL))
    {
        gatt_con_env_t* p_con = gatt_env.p_con[conidx];

        // search procedure with a specific dummy value in Execution queue
        p_proc = (gatt_proc_t*) co_list_pick(&(p_con->proc_exe_queue));
        while(p_proc != NULL)
        {
            uint8_t proc_type = GATT_PROC_TYPE_GET(p_proc->proc_id);

            // ensure that procedure is not initiated by GATT
            if((proc_type != GATT_PROC_CLI_HANDLER) && (proc_type != GATT_PROC_SRV_HANDLER) && (p_proc->dummy == dummy))
            {
                break;
            }

            p_proc = (gatt_proc_t*) p_proc->hdr.next;
        }

        if(p_proc == NULL)
        {
            // search procedure with a specific dummy value in wait queue
            p_proc = (gatt_proc_t*) co_list_pick(&(p_con->proc_wait_queue));
            while(p_proc != NULL)
            {
                uint8_t proc_type = GATT_PROC_TYPE_GET(p_proc->proc_id);

                // ensure that procedure is not initiated by GATT
                if((proc_type != GATT_PROC_CLI_HANDLER) && (proc_type != GATT_PROC_SRV_HANDLER) && (p_proc->dummy == dummy))
                {
                    break;
                }

                p_proc = (gatt_proc_t*) p_proc->hdr.next;
            }
        }
    }

    return (p_proc);
}

uint16_t gatt_proc_pdu_send(uint8_t conidx, gatt_proc_t* p_proc, l2cap_att_pdu_t* p_pdu, co_buf_t* p_data,
                            gatt_proc_pdu_handler_cb cb_rsp_handler)
{
    // Ask bearer to send the PDU
    uint16_t status = gatt_bearer_pdu_send(conidx, p_proc->bearer_lid, p_proc->token, p_pdu, p_data);

    // Check if the transaction timer must be started
    if(status == GAP_ERR_NO_ERROR)
    {
        if(cb_rsp_handler != NULL)
        {
            p_proc->tx_opcode = p_pdu->code;
            p_proc->tx_mtu    = gatt_bearer_mtu_get(conidx, p_proc->bearer_lid);

            gapc_sdt_timer_set(&(p_proc->defer_timer), GATT_TRANSACTION_TIMEOUT);

            p_proc->cb_pdu_handler = cb_rsp_handler;
        }
    }

    return (status);
}

void gatt_proc_check_grant(uint8_t conidx, uint8_t bearer_lid, uint8_t proc_type_bf)
{
    gatt_con_env_t* p_con = gatt_env.p_con[conidx];
    ASSERT_ERR(p_con != NULL);
    gatt_proc_t* p_proc = (gatt_proc_t*) co_list_pick(&(p_con->proc_wait_queue));

    // In Wait queue, check if a specific procedure type can be granted
    while((p_proc != NULL) && (proc_type_bf != 0))
    {
        gatt_proc_t* p_proc_next = (gatt_proc_t*) p_proc->hdr.next;

        // check if procedure expect to be granted
        if((CO_BIT(GATT_PROC_TYPE_GET(p_proc->proc_id)) & proc_type_bf) != 0)
        {
            #if(BLE_GATT_CLI)
            // check if procedure can be granted (not mutex protected)
            if(   !GETB(p_con->state_bf, GATT_CON_WRITE_QUEUE_MUTEX)
               || ((p_proc->proc_id != GATT_PROC_WRITE_LONG) && (p_proc->proc_id != GATT_PROC_WRITE_EXE)))
            #endif // (BLE_GATT_CLI)
            {
                gatt_user_t* p_user;
                bool granted;

                p_user = gatt_user_get(p_proc->user_lid);
                ASSERT_ERR(p_user != NULL);
                // Try to acquire a bearer for new procedure
                granted = gatt_bearer_acquire(conidx, bearer_lid, p_proc, p_user->pref_mtu);

                // A bearer is granted, procedure execution will start in background
                if(granted)
                {
                    proc_type_bf &= ~CO_BIT(GATT_PROC_TYPE_GET(p_proc->proc_id));
                    // extract procedure from waiting queue
                    co_list_extract(&(p_con->proc_wait_queue), &(p_proc->hdr));
                    // Put new procedure on front of execution queue
                    co_list_push_front(&(p_con->proc_exe_queue), &(p_proc->hdr));
                    gatt_proc_continue(conidx, p_proc, GATT_PROC_START, GAP_ERR_NO_ERROR);
                }
            }
        }

        p_proc = p_proc_next;
    }
}

void gatt_proc_continue(uint8_t conidx, gatt_proc_t* p_proc, uint8_t state, uint16_t status)
{
    DBG_FUNC_ENTER(gatt_proc_continue);
    DBG_SWDIAG(HL, GATT_PROC_CONT, 1);
    if((state == GATT_PROC_START) || (state == GATT_PROC_USER_CFM))
    {
        // Store state update information
        SETF(p_proc->info_bf, GATT_PROC_STATE, (state == GATT_PROC_START) ? GATT_PROC_WAIT_START : GATT_PROC_WAIT_USER_CFM);

        // defer execution of procedure state update
        gapc_sdt_defer(&(p_proc->defer_timer), status);
    }
    else
    {
        uint8_t proc_type = GATT_PROC_TYPE_GET(p_proc->proc_id);

        // update state
        if(state != GATT_PROC_ERROR)
        {
            SETF(p_proc->info_bf, GATT_PROC_STATE, state);
        }

        // If procedure has a timer or a function defer
        if(   ((state == GATT_PROC_PDU_RX) && ((proc_type == GATT_PROC_REQ) || (proc_type == GATT_PROC_IND)))
           || (state == GATT_PROC_ERROR))
        {
            gapc_sdt_stop(&(p_proc->defer_timer));
        }

        // Inform procedure of state update
        gatt_proc_continue_exe(conidx, p_proc, state, status);
    }
    DBG_SWDIAG(HL, GATT_PROC_CONT, 0);
    DBG_FUNC_EXIT(gatt_proc_continue);
}

void gatt_proc_bearer_rx_continue(uint8_t conidx, gatt_proc_t* p_proc)
{
    // Ask bearer to resume packet reception
    gatt_bearer_rx_continue(conidx, p_proc->bearer_lid);
}


uint16_t gatt_proc_mtu_get(uint8_t conidx, gatt_proc_t* p_proc)
{
    return (gatt_bearer_mtu_get(conidx, p_proc->bearer_lid));
}


uint16_t gatt_proc_handler_create(uint8_t conidx, uint8_t op_code, uint8_t bearer_lid,
                                  gatt_proc_pdu_handler_cb* p_pdu_hdl_cb, gatt_proc_handler_t** pp_proc)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    *p_pdu_hdl_cb = NULL;

    // ask procedure handler module to create the procedure
    switch(op_code)
    {
        #if (BLE_GATT_CLI)
        case L2CAP_ATT_HDL_VAL_NTF_OPCODE:
        case L2CAP_ATT_HDL_VAL_IND_OPCODE:
        case L2CAP_ATT_MULT_HDL_VAL_NTF_OPCODE:    { status = gatt_cli_event_proc_create(conidx, op_code, p_pdu_hdl_cb, pp_proc);     } break;
        #endif // (BLE_GATT_CLI)

        case L2CAP_ATT_MTU_REQ_OPCODE:             { status = gatt_srv_mtu_exch_proc_create(conidx, op_code, p_pdu_hdl_cb, pp_proc);  } break;

        case L2CAP_ATT_FIND_INFO_REQ_OPCODE:
        case L2CAP_ATT_FIND_BY_TYPE_REQ_OPCODE:
        case L2CAP_ATT_RD_BY_TYPE_REQ_OPCODE:
        case L2CAP_ATT_RD_BY_GRP_TYPE_REQ_OPCODE:  { status = gatt_srv_discover_proc_create(conidx, op_code, p_pdu_hdl_cb, pp_proc);  } break;

        case L2CAP_ATT_RD_REQ_OPCODE:
        case L2CAP_ATT_RD_BLOB_REQ_OPCODE:
        case L2CAP_ATT_RD_MULT_REQ_OPCODE:
        case L2CAP_ATT_RD_MULT_VAR_REQ_OPCODE:     { status = gatt_srv_read_proc_create(conidx, op_code, p_pdu_hdl_cb, pp_proc);      } break;

        case L2CAP_ATT_WR_CMD_OPCODE:
        case L2CAP_ATT_WR_SIGNED_OPCODE:
        case L2CAP_ATT_WR_REQ_OPCODE:
        case L2CAP_ATT_PREP_WR_REQ_OPCODE:         { status = gatt_srv_write_proc_create(conidx, op_code, p_pdu_hdl_cb, pp_proc);     } break;

        case L2CAP_ATT_EXE_WR_REQ_OPCODE:          { status = gatt_srv_write_exe_proc_create(conidx, op_code, p_pdu_hdl_cb, pp_proc); } break;

        default:                                   { status = ATT_ERR_INVALID_PDU;                                      } break;
    }

    if(status == GAP_ERR_NO_ERROR)
    {
        gatt_con_env_t* p_con = gatt_env.p_con[conidx];

        // attach bearer
        (*pp_proc)->bearer_lid = bearer_lid;
        // consider that start state is reached
        SETF((*pp_proc)->info_bf, GATT_PROC_STATE, GATT_PROC_START);
        // Put new procedure on front of execution queue
        co_list_push_front(&(p_con->proc_exe_queue), &((*pp_proc)->hdr));
    }

    return (status);
}

bool gatt_proc_is_user_active(uint8_t user_lid)
{
    uint8_t conidx;
    bool active = false;

    for(conidx = 0 ; (!active) && (conidx < HOST_CONNECTION_MAX) ; conidx++)
    {
        gatt_con_env_t* p_con = gatt_env.p_con[conidx];
        if(p_con != NULL)
        {
            // check wait queue
            gatt_proc_t* p_proc = (gatt_proc_t*) co_list_pick(&(p_con->proc_wait_queue));
            while ((p_proc != NULL) && (!active))
            {
                if(p_proc->user_lid == user_lid)
                {
                    active = true;
                }

                p_proc = (gatt_proc_t*) p_proc->hdr.next;
            }

            // check execution queue
            p_proc = (gatt_proc_t*) co_list_pick(&(p_con->proc_exe_queue));
            while ((p_proc != NULL) && (!active))
            {
                if(p_proc->user_lid == user_lid)
                {
                    active = true;
                }

                p_proc = (gatt_proc_t*) p_proc->hdr.next;
            }
        }
    }

    return (active);
}

void gatt_proc_cleanup(uint8_t conidx, uint16_t reason)
{
    gatt_con_env_t* p_con = gatt_env.p_con[conidx];
    gatt_proc_t* p_proc;
    // remove all on-going procedures
    p_proc = (gatt_proc_t*) co_list_pick(&(p_con->proc_exe_queue));

    while(   !co_list_is_empty(&(p_con->proc_exe_queue))
            || !co_list_is_empty(&(p_con->proc_wait_queue)))
    {
        gatt_proc_t* p_proc_next;

        // check procedure in execution
        while(p_proc != NULL)
        {
            // get next procedure because procedure remove itself from procedure queue.
            p_proc_next = (gatt_proc_t*) p_proc->hdr.next;
            // Mark procedure in error
            gatt_proc_continue(conidx, p_proc, GATT_PROC_ERROR, reason);
            p_proc = p_proc_next;
        }

        p_proc = (gatt_proc_t*) co_list_pick(&(p_con->proc_wait_queue));
    }
}


#if(BLE_GATT_CLI)
void gatt_proc_write_queue_mutex_set(uint8_t conidx, bool enable)
{
    gatt_con_env_t* p_con = gatt_env.p_con[conidx];
    SETB(p_con->state_bf,  GATT_CON_WRITE_QUEUE_MUTEX, enable);

    if(!enable)
    {
        // search procedure with a specific procedure type
        gatt_proc_t* p_proc = (gatt_proc_t*) co_list_pick(&(p_con->proc_wait_queue));
        while(p_proc != NULL)
        {
            if((p_proc->proc_id == GATT_PROC_WRITE_LONG) || (p_proc->proc_id == GATT_PROC_WRITE_EXE))
            {
                gatt_user_t* p_user   = gatt_user_get(p_proc->user_lid);
                bool granted;
                ASSERT_ERR(p_user != NULL);
                // Try to acquire a bearer for new procedure
                granted = gatt_bearer_acquire(conidx, GATT_INVALID_BEARER_LID, p_proc, p_user->pref_mtu);

                // A bearer is granted, procedure execution will start in background
                if(granted)
                {
                    // extract procedure from waiting queue
                    co_list_extract(&(p_con->proc_wait_queue), &(p_proc->hdr));
                    // Put new procedure on front of execution queue
                    co_list_push_front(&(p_con->proc_exe_queue), &(p_proc->hdr));
                    gatt_proc_continue(conidx, p_proc, GATT_PROC_START, GAP_ERR_NO_ERROR);
                }

                break;
            }

            p_proc = (gatt_proc_t*) p_proc->hdr.next;
        }
    }
}
#endif // (BLE_GATT_CLI)

#if (HOST_MSG_API)
void gatt_proc_cur_set(gatt_proc_t* p_proc)
{
    gatt_env.p_cur_proc = p_proc;
}

gatt_proc_t* gatt_proc_cur_get(void)
{
    return gatt_env.p_cur_proc;
}
#endif // (HOST_MSG_API)

#endif // (BLE_GATT)
/// @} GATT


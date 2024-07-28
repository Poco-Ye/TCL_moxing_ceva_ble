/**
 ****************************************************************************************
 * @file gatt_cli_mtu_exch.c
 *
 * @brief  GATT Client MTU Exchange Procedure
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
#if (BLE_GATT_CLI)
#include "gatt.h"           // Native API
#include "gatt_user.h"      // GATT User API
#include "gatt_proc.h"      // Procedure API
#include "gatt_int.h"       // GATT Internals
#include "gapc.h"           // For CSRK and Sign counter usage

#include <string.h>         // for memcmp
#include "co_math.h"        // for co_min
#include "co_endian.h"      // for host to bt number conversion

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

/// MTU exchange procedure information
typedef struct gatt_cli_mtu_exch_proc
{
    /// Procedure header - required for any Attribute procedure
    gatt_proc_t             hdr;
    /// Negotiated MTU
    uint16_t                mtu_nego;
} gatt_cli_mtu_exch_proc_t;


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
 * @brief Function called when L2CAP_ATT_MTU_RSP attribute PDU is received.
 *
 * @param[in] conidx     Connection index
 * @param[in] p_proc     Pointer to procedure under execution
 * @param[in] p_pdu      Pointer to unpacked PDU
 * @param[in] p_buf      Data of received but not yet extracted
 * @param[in] mtu        MTU size of the bearer
 *
 * @return PDU handling return status that will be provided to the procedure handler.
 ****************************************************************************************
 */
__STATIC uint16_t gatt_cli_l2cap_att_mtu_rsp_handler(uint8_t conidx, gatt_cli_mtu_exch_proc_t* p_proc, l2cap_att_mtu_rsp_t* p_pdu,
                                                    co_buf_t* p_buf, uint16_t mtu)
{
    // update value of negotiated MTU
    p_proc->mtu_nego = co_min(p_proc->mtu_nego, p_pdu->mtu_size);

    // mark that reception on bearer can continue
    gatt_proc_bearer_rx_continue(conidx, &(p_proc->hdr));
    return (GAP_ERR_NO_ERROR);
}

/**
 ****************************************************************************************
 * @brief function called when operation state is updated
 *
 * @param[in] conidx     Connection index
 * @param[in] p_proc     Pointer to procedure to continue
 * @param[in] proc_state Operation transition state (see enum #gatt_proc_state)
 * @param[in] status     Execution status
 ****************************************************************************************
 */
__STATIC void gatt_cli_mtu_exch_proc_continue(uint8_t conidx, gatt_cli_mtu_exch_proc_t* p_proc, uint8_t proc_state, uint16_t status)
{
    bool finished = false;

    switch(proc_state)
    {
        case GATT_PROC_START:
        {
            co_buf_t* p_buf     = NULL;
            l2cap_att_pdu_t pdu;

            // Allocate buffer for transmission
            if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, 0, GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
            {
                pdu.code             = L2CAP_ATT_MTU_REQ_OPCODE;
                pdu.mtu_req.mtu_size = p_proc->mtu_nego;

                // Ask for PDU transmission
                status = gatt_proc_pdu_send(conidx, &(p_proc->hdr), &pdu, p_buf,
                                            (gatt_proc_pdu_handler_cb) gatt_cli_l2cap_att_mtu_rsp_handler);

                // release buffer
                co_buf_release(p_buf);
            }
            else
            {
                status = GAP_ERR_INSUFF_RESOURCES;
            }
        } break;

        case GATT_PROC_PDU_RX:
        {
            if(status == GAP_ERR_NO_ERROR)
            {
                gatt_con_env_t* p_con = gatt_env.p_con[conidx];
                // Mark that MTU exchange has been properly executed
                SETB(p_con->state_bf, GATT_CON_INI_MTU_EXCH, true);
                // Update bearer with new MTU value
                gatt_bearer_mtu_set(conidx, p_proc->hdr.bearer_lid, p_proc->mtu_nego);
            }
            finished = true;
        } break;

        case GATT_PROC_ERROR:
        {
            finished = true;
        } break;
        default: { /*  Nothing to do */  } break;
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        finished = true;
    }

    if(finished)
    {
        // Pop Procedure
        gatt_proc_pop(conidx, &(p_proc->hdr), true);
    }
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t gatt_cli_mtu_exch(uint8_t conidx, uint8_t user_lid)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    uint8_t  bearer_lid = GATT_INVALID_BEARER_LID;
    uint16_t pref_mtu = gatt_user_pref_mtu_get();

    // check if MTU exchange not already done and if Legacy ATT bearer exists + new MTU exceed minimum MTU
    if((pref_mtu > L2CAP_LE_MTU_MIN) && gatt_bearer_mtu_exch_supported(conidx, &bearer_lid))
    {
        gatt_con_env_t* p_con = gatt_env.p_con[conidx];

        // Check that MTU exchange has not been already performed
        if(!GETB(p_con->state_bf, GATT_CON_INI_MTU_EXCH) && !GETB(p_con->state_bf, GATT_CON_RSP_MTU_EXCH))
        {
            gatt_cli_mtu_exch_proc_t* p_proc;

            // Create procedure
            status = gatt_proc_create(conidx, user_lid, 0, GATT_PROC_MTU_EXCH, L2CAP_LE_MTU_MIN,
                                      sizeof(gatt_cli_mtu_exch_proc_t),
                                      (gatt_proc_cb) gatt_cli_mtu_exch_proc_continue, (gatt_proc_t**) &p_proc);

            if(status == GAP_ERR_NO_ERROR)
            {
                // force bearer
                p_proc->hdr.bearer_lid = bearer_lid;
                // Set preferred MTU
                p_proc->mtu_nego = pref_mtu;

                // ask procedure to be granted - push it in wait list
                gatt_proc_push(conidx, &(p_proc->hdr));
            }
        }
    }

    return (status);
}

#if (HOST_MSG_API)
/*
 * MESSAGE HANDLER FUNCTIONS
 ****************************************************************************************
 */
#include "gatt_msg_int.h"

/**
 ****************************************************************************************
 * @brief Handle MTU Update exchange request
 *
 * @param[in] p_cmd     Pointer to command parameters
 * @param[in] src_id    ID of the sending task instance.
 ****************************************************************************************
 */
void gatt_cli_mtu_update_cmd_handler(gatt_cli_mtu_update_cmd_t* p_cmd, uint16_t src_id)
{
    // Ask MTU exchange request,
    uint16_t status = gatt_cli_mtu_exch(p_cmd->conidx, p_cmd->user_lid);
    // Directly send back result, no wait for end of procedure
    gatt_msg_send_proc_cmp_evt(p_cmd->cmd_code, p_cmd->dummy, p_cmd->conidx, src_id, p_cmd->user_lid, status);
}
#endif // (HOST_MSG_API)

#endif // (BLE_GATT_CLI)
#endif // (BLE_GATT)
/// @} GATT


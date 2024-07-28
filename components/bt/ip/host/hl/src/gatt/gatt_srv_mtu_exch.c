/**
 ****************************************************************************************
 * @file gatt_srv_write.c
 *
 * @brief  GATT Server MTU Exchange Procedures
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
#include "rwip_config.h"       // IP configuration
#if (BLE_GATT)
#include "gatt.h"              // Native API
#include "gatt_int.h"          // Internals
#include "gatt_db.h"           // GATT Database access
#include "gapc.h"              // For CSRK and Sign counter usage
#include "co_math.h"           // for co_min
#include "../inc/gap_hl_api.h" // to get if client is out of sync


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

/// Server Mtu Exchange procedure structure.
typedef struct gatt_srv_mtu_exch_proc
{
    /// Procedure header - required for any Attribute procedure
    gatt_proc_handler_t     hdr;
    /// Negotiated MTU
    uint16_t                mtu_nego;
    /// Use to know if MTU request has been accepted
    bool                    accept;
} gatt_srv_mtu_exch_proc_t;

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
 * @brief Function called when L2CAP_ATT_MTU_REQ attribute PDU is received.
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
__STATIC uint16_t gatt_srv_write_l2cap_att_mtu_req_handler(uint8_t conidx, gatt_srv_mtu_exch_proc_t* p_proc,
                                                           l2cap_att_mtu_req_t* p_pdu, co_buf_t* p_buf, uint16_t mtu)
{
    gatt_con_env_t* p_con = gatt_env.p_con[conidx];
    uint16_t status = GAP_ERR_NO_ERROR;
    uint8_t bearer_lid;

    // Check if MTU not already exchanged
    if(   GETB(p_con->state_bf, GATT_CON_RSP_MTU_EXCH)
		    // Check if MTU received on a legacy bearer or if MTU size not reduced
            || !gatt_bearer_mtu_exch_supported(conidx, &bearer_lid) || (bearer_lid != p_proc->hdr.bearer_lid)
            || (p_pdu->mtu_size < mtu))
    {
        status = ATT_ERR_REQUEST_NOT_SUPPORTED;
    }
    else
    {
        // update value of negotiated MTU
        uint16_t pref_mtu = gatt_user_pref_mtu_get();
        p_proc->mtu_nego = co_min(pref_mtu, p_pdu->mtu_size);
    }

    // mark that reception on bearer can continue
    gatt_proc_bearer_rx_continue(conidx, (gatt_proc_t*) p_proc);

    // Update client cache status
    gapc_svc_is_cli_out_of_sync(conidx, p_pdu->code, GATT_INVALID_HDL);

    return (status);
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
__STATIC void gatt_srv_mtu_exch_proc_continue(uint8_t conidx, gatt_srv_mtu_exch_proc_t* p_proc, uint8_t proc_state,
                                              uint16_t status)
{
    bool finished = false;

    switch(proc_state)
    {
        case GATT_PROC_PDU_RX:
        {
            l2cap_att_pdu_t pdu;
            co_buf_t* p_buf = NULL;

            // allocate buffer used for confirmation transmission
            if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, 0, GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
            {
                if(status != GAP_ERR_NO_ERROR)
                {
                    // send error response
                    pdu.code            = L2CAP_ATT_ERR_RSP_OPCODE;
                    pdu.err_rsp.op_code = L2CAP_ATT_MTU_REQ_OPCODE;
                    pdu.err_rsp.handle  = 0;
                    pdu.err_rsp.reason  = status;

                    p_proc->accept = false;
                }
                else
                {
                    // send error response
                    pdu.code             = L2CAP_ATT_MTU_RSP_OPCODE;
                    pdu.mtu_rsp.mtu_size = p_proc->mtu_nego;
                    p_proc->accept = true;
                }

                // Ask for PDU transmission
                status = gatt_proc_pdu_send(conidx, (gatt_proc_t*) p_proc, &pdu, p_buf, NULL);
                co_buf_release(p_buf);
            }
            else
            {
                status = GAP_ERR_UNEXPECTED;
            }
        } break;
        case GATT_PROC_PDU_PUSHED_TO_LL:    // Confirmation properly pushed in TX LL buffers
        {
            if(status != GAP_ERR_NO_ERROR) break;

            if(p_proc->accept)
            {
                gatt_con_env_t* p_con = gatt_env.p_con[conidx];
                // Mark that MTU exchange has been properly executed
                SETB(p_con->state_bf, GATT_CON_RSP_MTU_EXCH, true);
                // Update bearer with new MTU value
                gatt_bearer_mtu_set(conidx, p_proc->hdr.bearer_lid, p_proc->mtu_nego);
            }
        } break;
        case GATT_PROC_ERROR:               // Handle error detection
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
        gatt_proc_pop(conidx, (gatt_proc_t*) p_proc, true);
    }
}


/*
 * INTERNAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Ask GATT Server module to create a MTU Exchange procedure handler
 *
 * @param[in]  conidx       Connection index
 * @param[in]  op_code      Attribute operation code received
 * @param[out] p_pdu_hdl_cb Pointer to PDU Handler call-back
 * @param[out] pp_proc      Pointer to the procedure created, NULL if procedure not created
 *
 * @return Procedure creation status code (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gatt_srv_mtu_exch_proc_create(uint8_t conidx, uint8_t op_code, gatt_proc_pdu_handler_cb* p_pdu_hdl_cb,
                                       gatt_proc_handler_t** pp_proc)
{
    uint16_t status;

    // Allocate procedure
    status = gatt_proc_handler_alloc(conidx, GATT_PROC_HANDLE_MTU_EXCH, sizeof(gatt_srv_mtu_exch_proc_t),
                                     (gatt_proc_cb) gatt_srv_mtu_exch_proc_continue, pp_proc);

    *p_pdu_hdl_cb = (gatt_proc_pdu_handler_cb) gatt_srv_write_l2cap_att_mtu_req_handler;

    return (status);
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

#endif // (BLE_GATT)
/// @} GATT


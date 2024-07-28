/**
 ****************************************************************************************
 *
 * @file llc.c
 *
 * @brief Definition of the functions used by the logical link controller
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLC
 * @{
 ****************************************************************************************
 */
#include "rwip_config.h"


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"    // stack configuration
#include "rwip.h"           // RW definitions

#include <string.h>
#include "co_utils.h"
#include "co_bt.h"          // BLE standard definitions
#include "co_math.h"        // For co_max
#include "ke_task.h"        // Kernel task
#include "ke_mem.h"         // Kernel memory management
#include "ke_timer.h"       // Kernel timer management

#include "llc.h"            // link layer controller definitions
#include "llc_int.h"        // link layer controller internal definitions
#include "llc_llcp.h"       // link layer L2CAP manager

#include "lld.h"            // link layer driver definitions
#include "llm.h"            // link layer manager definitions

#if (BLE_PERIPHERAL || BLE_CENTRAL)

#include "dbg.h"

/*
 * DEFINES
 ****************************************************************************************
 */
/**
 * Internal LLCP transaction timer state
 *
 * title: State machine of procedure timer
 *
 * @startuml llc_trans_timer_state.png
 * "OFF" -->[enable] "STARTED"
 * "STARTED" -->[disable] "OFF"
 * "STARTED" -->[pause]"PAUSED"
 * "PAUSED"  -->[resume]"STARTED"
 * "OFF" -->[pause] "DISABLED"
 * "DISABLED" -->[resume] "OFF"
 * "DISABLED" -->[enable] "PAUSED"
 * "PAUSED" -->[disable] "DISABLED"
 * "OFF" -->[disable, resume]"OFF"
 * note left: LLCP Timer is off
 * "STARTED" -->[enable, resume]"STARTED"
 * note left #aqua: LLCP Timer is on
 * "PAUSED" -->[enable, pause] "PAUSED"
 * note left: LLCP Timer is off
 * "DISABLED" -->[disable, pause] "DISABLED"
 * note left: LLCP Timer is off
 * @enduml
 */
/*@TRACE*/
enum llc_timer_state
{
    /// Timer is OFF
    LLC_TIMER_OFF,     //!< LLC_TIMER_OFF
    /// Timer is Started
    LLC_TIMER_STARTED, //!< LLC_TIMER_STARTED
    /// Timer is Paused
    LLC_TIMER_PAUSED,  //!< LLC_TIMER_PAUSED
    /// Timer cannot be started but can enter in paused state.
    LLC_TIMER_DISABLED,//!< LLC_TIMER_DISABLED
};


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
/// LLC environment
struct llc_env_tag* llc_env[BLE_ACTIVITY_MAX];


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Enable or clear a transaction timer
 *
 * @note: assume that proc_type and link_id are valid.
 *
 * @param[in] link_id Link Identifier
 * @param[in] proc_type Local or Remote Procedure (@see enum llc_proc_type)
 * @param[in] enable  True, start the timer, False clear it.
 ****************************************************************************************
 */
__STATIC void llc_llcp_trans_timer_set(uint8_t link_id, uint8_t proc_type, bool enable)
{
    ke_task_id_t llc_id =  KE_BUILD_ID(TASK_LLC, link_id);

    uint16_t timer_id = (proc_type == LLC_PROC_LOCAL) ? LLC_LOC_LLCP_RSP_TO : LLC_REM_LLCP_RSP_TO;

    if(enable)
    {
        // if link wait for termination
        if(llc_is_disconnecting(link_id))
        {
            // Vol 6 Part B - 5.1.6 Termination Procedure
            //
            // The Link Layer shall start a timer, T_terminate, when the LL_TERMINATE_IND
            // PDU has been queued for transmission [...] to value of the connSupervisionTimeout.

            // force LLCP response timeout to supervision timeout
            ke_timer_set(timer_id, llc_id, 10*llc_env[link_id]->con_params.timeout);
        }
        else
        {
            ke_timer_set(timer_id, llc_id, 10*LLCP_RSP_TO);
        }
    }
    else
    {
        ke_timer_clear(timer_id, llc_id);
    }
}


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void ROM_VT_FUNC(llc_init)(uint8_t init_type)
{
    switch (init_type)
    {
        case RWIP_INIT:
        {
            // Create LLC task
            ke_task_create(TASK_LLC, &TASK_DESC_LLC);
        }
        break;

        case RWIP_RST:
        {
            // perform a connection clean-up
            for(int8_t link_id = (BLE_ACTIVITY_MAX-1) ; link_id >= 0 ; link_id--)
            {
                // nothing to do, continue
                if(llc_env[link_id] != NULL)
                {
                    llc_cleanup(link_id, true);
                }
            }
        }
        break;

        case RWIP_1ST_RST:
        {
            // Initialize environment
            memset(&llc_env, 0, sizeof(llc_env));
        }
        break;

        default:
        {
            // Do nothing
        }
        break;
    }
}

void ROM_VT_FUNC(llc_cleanup)(uint8_t link_id, bool reset)
{
    struct co_list_hdr* next;
    int8_t proc_type;
    struct llc_env_tag *llc_env_ptr =  llc_env[link_id];
    ke_task_id_t llc_id =  KE_BUILD_ID(TASK_LLC, link_id);

    next = co_list_pick(&(llc_env_ptr->llcp_tx_queue));

    // clean-up LLCP TX queue
    while(next != NULL)
    {
        struct co_list_hdr* current = next;
        next = current->next;
        ke_free(current);
    }

    // clean-up procedures on-going
    for(proc_type = (LLC_PROC_MAX-1) ; proc_type >= 0 ; proc_type--)
    {
        llc_procedure_t* proc = llc_env_ptr->procedure[proc_type];
        if(proc != NULL)
        {
            ke_free(ke_param2msg(proc));
        }
    }

    // stop timers and retun to default state
    llc_llcp_trans_timer_set(link_id, LLC_PROC_LOCAL, false);
    llc_llcp_trans_timer_set(link_id, LLC_PROC_REMOTE, false);
    ke_timer_clear(LLC_AUTH_PAYL_NEARLY_TO, llc_id);
    ke_timer_clear(LLC_AUTH_PAYL_REAL_TO, llc_id);
    #if (BLE_CON_CTE_REQ)
    ke_timer_clear(LLC_CTE_REQ_TO, llc_id);
    #endif // (BLE_CON_CTE_REQ)
    ke_state_set(llc_id, LLC_FREE);

    ke_free(llc_env_ptr);
    llc_env[link_id] = NULL;
}

void ROM_VT_FUNC(llc_stop)(uint8_t link_id)
{
    ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
    struct llc_env_tag* llc_env_ptr = llc_env[link_id];

    if(llc_env_ptr != NULL)
    {
        llc_llcp_state_set(link_id, LLC_LLCP_DIR_BOTH, LLC_LLCP_TERMINATE);
    }

    // Go to free state to ensure that all message in save queue are restored
    ke_state_set(llc_id, LLC_FREE);
    // use a message to fully clean-up connection environment
    ke_msg_send_basic(LLC_STOPPED_IND, llc_id, llc_id);
}


uint8_t ROM_VT_FUNC(llc_start)(uint8_t link_id, struct llc_init_parameters *con_params)
{
    // Gets the environment
    struct llc_env_tag *llc_env_ptr = NULL;
    //Status to check if the allocation is done
    uint8_t status = CO_ERROR_NO_ERROR;

    // Check if this llc_env is already used
    if (llc_env[link_id] == NULL)
    {
        // Allocate a LLC environment structure for the link
        llc_env[link_id] = (struct llc_env_tag*)ke_malloc_system(sizeof(struct llc_env_tag), KE_MEM_ENV);
        llc_env_ptr = llc_env[link_id];

        // Allocation failed
        if(!llc_env_ptr)
        {
            status = CO_ERROR_MEMORY_CAPA_EXCEED;
        }
    }
    else
    {
        ASSERT_INFO(0, link_id, llc_env[link_id]);
        status = CO_ERROR_UNDEFINED;
    }

    //If LLC env allocation done
    if(!status)
    {
        int8_t i;
        ke_state_set(KE_BUILD_ID(TASK_LLC, link_id), LLC_IDLE);

        // initialize LLCP send list queue
        co_list_init(&(llc_env_ptr->llcp_tx_queue));

        // initialize connection Information
        llc_env_ptr->link_info   = 0;
        llc_env_ptr->llcp_state  = 0;
        SETF(llc_env_ptr->llcp_state, LLC_LLCP_RX, LLC_LLCP_IDLE);
        SETF(llc_env_ptr->llcp_state, LLC_LLCP_TX, LLC_LLCP_IDLE);
        llc_env_ptr->flow_ctrl   = 0;
        llc_env_ptr->disc_reason = CO_ERROR_NO_ERROR;
        llc_env_ptr->con_stop    = false;
        llc_env_ptr->l2cap_start = false;

        // Copy connection parameters
        llc_env_ptr->con_params.interval = con_params->interval;
        llc_env_ptr->con_params.latency  = con_params->latency;
        llc_env_ptr->con_params.timeout  = con_params->timeout;
        llc_env_ptr->con_params.sca      = con_params->master_sca;
        // copy channel map
        memcpy(&(llc_env_ptr->con_params.ch_map), &(con_params->chm), sizeof(struct le_chnl_map));

        /**
         * Default peer expected supported features are same as ours except:
         * - support for LLCP Reject Indication
         * - support for Periodic Advertising Sync Transfer - Recipient feature
         */
        llm_le_features_get(&llc_env_ptr->rem_feats);
        llc_le_feature_set(link_id, BLE_FEAT_EXT_REJ_IND,            false); // not supported by default
        llc_le_feature_set(link_id, BLE_FEAT_PER_ADV_SYNC_TRANSF_RX, false); // not supported by default

        //  Data length default values initialization
        llc_env_ptr->con_params.max_tx_octets      = LE_MIN_OCTETS; //27 bytes
        llc_env_ptr->con_params.max_tx_time        = (co_rate_to_phy[con_params->rate] == PHY_CODED_VALUE) ? LE_MIN_TIME_CODED: LE_MIN_TIME; // 328 us or 2704 us for the coded PHY
        llc_env_ptr->con_params.max_rx_octets      = LE_MIN_OCTETS; //27 bytes
        llc_env_ptr->con_params.max_rx_time        = (co_rate_to_phy[con_params->rate] == PHY_CODED_VALUE) ? LE_MIN_TIME_CODED: LE_MIN_TIME; // 328 us or 2704 us for the coded PHY
        llc_env_ptr->suggested_max_tx_octets       = con_params->suggested_max_tx_octets;
        llc_env_ptr->suggested_max_tx_time         = con_params->suggested_max_tx_time;
        llc_env_ptr->suggested_max_rx_octets       = BLE_MAX_OCTETS; //251 bytes
        llc_env_ptr->suggested_max_rx_time         = BLE_MAX_TIME;

        // Current PHYS used on the link
        llc_env_ptr->con_params.tx_phy             = co_rate_to_phy[con_params->rate];
        llc_env_ptr->con_params.rx_phy             = llc_env_ptr->con_params.tx_phy;

        // Min used channels initialization
        llc_env_ptr->con_params.min_used_ch_phys   = PHY_ALL;
        llc_env_ptr->con_params.min_used_ch        = DATA_CHANNEL_USED_NB_MIN;

        // PHY default values initialization
        llc_env_ptr->phy_opt                       = con_params->phy_opt;
        llc_env_ptr->tx_phys                       = con_params->tx_phys;
        llc_env_ptr->rx_phys                       = con_params->rx_phys;

        #if BLE_PWR_CTRL
        // LE power control initialization
        memset(&llc_env_ptr->pwr.loc_tx_pwr[0], (int8_t)rwip_rf.txpwr_dbm_get(rwip_rf.txpwr_max, MOD_GFSK), CO_RATE_MAX);
        llc_env_ptr->pwr.loc_tx_pwr_rep_en = false;
        llc_env_ptr->pwr.rem_tx_pwr_rep_en = false;
        llc_env_ptr->pwr.loc_tx_pwr_chg_ind_en = false;
        llc_env_ptr->pwr.rem_tx_pwr_chg_ind_en = false;
        llc_env_ptr->pwr.path_loss_cfg = false;
        llc_env_ptr->pwr.cis_phy_msk = 0;
        #endif // BLE_PWR_CTRL

        // If TX PHY is coded, transmission time must be above 2704us (LL:4.5.10)
        if(llc_env_ptr->con_params.tx_phy == PHY_CODED_VALUE)
        {
            llc_env_ptr->suggested_max_tx_time = co_max(llc_env_ptr->suggested_max_tx_time, LE_MIN_TIME_CODED);
        }

        #if (BLE_CON_CTE_REQ)
        llc_env_ptr->cte_rx.param_received = false;
        llc_env_ptr->cte_rx.req_active = false;
        llc_env_ptr->cte_rx.cte_req_intv = 0;
        #endif // (BLE_CON_CTE_REQ)

        #if (BLE_CON_CTE_RSP)
        llc_env_ptr->cte_tx.param_received = false;
        llc_env_ptr->cte_tx.en_status = CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE;
        #endif // (BLE_CON_CTE_RSP)

        // Initialize connection role
        SETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE, (con_params->role == MASTER_ROLE));

        // Initialize PAST parameters
        llc_env_ptr->past.mode     = con_params->past_mode;
        llc_env_ptr->past.cte_type = con_params->past_cte_type;
        llc_env_ptr->past.skip     = con_params->past_skip;
        llc_env_ptr->past.sync_to  = con_params->past_sync_to;


        // initialize procedure pointers
        for(i = (LLC_PROC_MAX-1) ; i >= 0 ; i--)
        {
            llc_env_ptr->procedure[i] = NULL;
        }

        {
            struct lld_con_params params;

            params.aa              = con_params->aa;
            params.crcinit         = con_params->crcinit;
            params.winsize         = con_params->winsize;
            params.winoffset       = con_params->winoffset;
            params.interval        = con_params->interval;
            params.latency         = con_params->latency;
            params.timeout         = con_params->timeout;
            params.chm             = con_params->chm;
            params.hop_inc         = con_params->hop_inc;
            params.master_sca      = con_params->master_sca;
            params.fine_cnt_rxsync = con_params->fine_cnt_rxsync;
            params.base_cnt_rxsync = con_params->base_cnt_rxsync;
            params.first_anchor_ts = con_params->first_anchor_ts;
            params.role            = con_params->role;
            params.rate            = con_params->rate;
            params.ch_sel_2        = con_params->ch_sel_2;
            params.aux_connect_req = con_params->aux_connect_req;

            status = lld_con_start(link_id, &params);
        }

        // Set default PING timer (30s)
        llc_le_ping_set(link_id, AUTH_PAYL_TO_DFT);


        // Start automatic PHY/data length updates if needed
        phy_upd_proc_start(link_id);
        dl_upd_proc_start(link_id);
    }

    return(status);
}

uint8_t ROM_VT_FUNC(llc_role_get)(uint8_t link_id, uint8_t* role)
{
    // Command status
    uint8_t status = CO_ERROR_NO_ERROR;

    // Check if link exists or is being disconnected and if we are well master
    if (llc_is_disconnecting(link_id) || !GETB(llc_env[link_id]->link_info, LLC_INFO_MASTER_ROLE))
    {
        status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    }
    else
    {
        *role = GETB(llc_env[link_id]->link_info, LLC_INFO_MASTER_ROLE) ? MASTER_ROLE : SLAVE_ROLE;
    }

    return (status);
}


void ROM_VT_FUNC(llc_llcp_state_set)(uint8_t link_id, uint8_t dir, uint8_t state)
{
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Verify that Idle or Terminate are set for both directions at the same time
    ASSERT_INFO( ((state != LLC_LLCP_TERMINATE) || (dir == LLC_LLCP_DIR_BOTH)), dir, state);
    ASSERT_INFO( ((state != LLC_LLCP_IDLE)      || (dir == LLC_LLCP_DIR_BOTH)), dir, state);

    // if link is in terminate state, nothing to do
    if((llc_env_ptr != NULL) && (GETF(llc_env_ptr->llcp_state, LLC_LLCP_RX) != LLC_LLCP_TERMINATE) && (GETF(llc_env_ptr->llcp_state, LLC_LLCP_TX) != LLC_LLCP_TERMINATE))
    {
        // Update state for the selected direction
        switch(dir)
        {
            case LLC_LLCP_DIR_RX:
            {
                SETF(llc_env_ptr->llcp_state, LLC_LLCP_RX, state);
            }
            break;
            case LLC_LLCP_DIR_BOTH:
            {
                SETF(llc_env_ptr->llcp_state, LLC_LLCP_RX, state);
            } // No break
            case LLC_LLCP_DIR_TX:
            {
                SETF(llc_env_ptr->llcp_state, LLC_LLCP_TX, state);

                // check if some LLCP PDUs are waiting to be transfered
                llc_llcp_tx_check(link_id);
            }
            break;
            default:
            {
                ASSERT_INFO(0, link_id, dir);
            }
            break;
        }
    }
}

void ROM_VT_FUNC(llc_proc_reg)(uint8_t link_id, uint8_t proc_type, llc_procedure_t *params)
{
    ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
    uint8_t state = ke_state_get(llc_id);
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    ASSERT_INFO(proc_type < LLC_PROC_MAX, link_id, proc_type);
    ASSERT_INFO(llc_env_ptr->procedure[proc_type] == NULL, link_id, proc_type);

    // save procedure
    llc_env_ptr->procedure[proc_type] = params;

    ASSERT_INFO(state != LLC_FREE, link_id, state);

    // update internal state
    ke_state_set(llc_id, state | (1 << proc_type));
}

void ROM_VT_FUNC(llc_proc_unreg)(uint8_t link_id, uint8_t proc_type)
{
    ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
    uint8_t state = ke_state_get(llc_id);
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    ASSERT_INFO(proc_type < LLC_PROC_MAX, link_id, proc_type);
    ASSERT_INFO(llc_env_ptr->procedure[proc_type] != NULL, link_id, proc_type);

    // clear transaction timer
    llc_proc_timer_set(link_id, proc_type, false);

    // clean-up procedure
    ke_msg_free(ke_param2msg(llc_env_ptr->procedure[proc_type]));
    llc_env_ptr->procedure[proc_type] = NULL;

    if(state != LLC_FREE)
    {
        // update internal state
        ke_state_set(llc_id, state & (~(1 << proc_type)));
    }
}

uint8_t ROM_VT_FUNC(llc_proc_id_get)(uint8_t link_id, uint8_t proc_type)
{
    uint8_t proc_id = LLC_PROC_NONE;
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    ASSERT_INFO(proc_type < LLC_PROC_MAX, link_id, proc_type);

    if(llc_env_ptr->procedure[proc_type] != NULL)
    {
        proc_id = llc_env_ptr->procedure[proc_type]->id;
    }

    return proc_id;
}

void ROM_VT_FUNC(llc_proc_init)(llc_procedure_t *proc, uint8_t proc_id, llc_proc_error_cb error_cb)
{
    proc->id = proc_id;
    proc->error_cb = error_cb;
    proc->timer_state = LLC_TIMER_OFF;
    proc->state = 0;
}

uint8_t ROM_VT_FUNC(llc_proc_state_get)(llc_procedure_t *proc)
{
    return proc->state;
}

void ROM_VT_FUNC(llc_proc_state_set)(llc_procedure_t *proc, uint8_t link_id, uint8_t proc_state)
{
    //Trace the state transition
    TRC_REQ_LLC_STATE_TRANS(BLE_LINKID_TO_CONHDL(link_id), proc->id, proc->state, proc_state);

    proc->state = proc_state;
}

llc_procedure_t * ROM_VT_FUNC(llc_proc_get)(uint8_t link_id, uint8_t proc_type)
{
    llc_procedure_t * ret = NULL;
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    ASSERT_INFO(proc_type < LLC_PROC_MAX, link_id, proc_type);

    if(llc_env_ptr != NULL)
    {
        ret = llc_env_ptr->procedure[proc_type];
    }

    return ret;
}

void ROM_VT_FUNC(llc_proc_err_ind)(uint8_t link_id, uint8_t proc_type, uint8_t error_type, void* param)
{
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    ASSERT_INFO(proc_type < LLC_PROC_MAX, link_id, proc_type);

    if((llc_env_ptr != NULL) && (llc_env_ptr->procedure[proc_type] != NULL))
    {
        llc_proc_error_cb error_cb =  llc_env_ptr->procedure[proc_type]->error_cb;

        // execute callback if set by procedure
        if(error_cb != NULL)
        {
            error_cb(link_id, error_type, param);
        }
    }
}

void ROM_VT_FUNC(llc_proc_timer_set)(uint8_t link_id, uint8_t proc_type, bool enable)
{
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    ASSERT_INFO(proc_type < LLC_PROC_MAX, link_id, proc_type);

    if(llc_env_ptr != NULL)
    {
        llc_procedure_t* proc = llc_env[link_id]->procedure[proc_type];

        if(proc != NULL)
        {
            bool start_timer = false;
            bool clear_timer = false;

            // check timer state to start or not the transaction timeout
            switch (proc->timer_state)
            {
                case LLC_TIMER_OFF:
                {
                    if(enable)
                    {
                        // start the timer.
                        start_timer = true;

                        proc->timer_state = LLC_TIMER_STARTED;
                    } // else nothing to do
                }break;
                case LLC_TIMER_STARTED:
                {
                    if(enable)
                    {
                        // Restart the timer.
                        start_timer = true;
                    }
                    else
                    {
                        proc->timer_state = LLC_TIMER_OFF;
                        //  Stop the timer.
                        clear_timer = true;
                    }
                }break;
                case LLC_TIMER_PAUSED:
                {
                    if(!enable)
                    {
                        // Go into disable state
                        proc->timer_state = LLC_TIMER_DISABLED;
                    } // else nothing to do
                }break;
                case LLC_TIMER_DISABLED:
                {
                    if(enable)
                    {
                        // Go into Paused state
                        proc->timer_state = LLC_TIMER_PAUSED;
                    } // else nothing to do
                }break;
                default:
                {
                    ASSERT_INFO(0, proc->id, proc->timer_state);
                }break;
            }

            // (re)start timer
            if(start_timer)
            {
                llc_llcp_trans_timer_set(link_id, proc_type, true);
            }
            // stop timer
            else if (clear_timer)
            {
                llc_llcp_trans_timer_set(link_id, proc_type, false);
            }
            // else nothing to do
        }
    }
}

void ROM_VT_FUNC(llc_proc_timer_pause_set)(uint8_t link_id, uint8_t proc_type, bool enable)
{
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    ASSERT_INFO(proc_type < LLC_PROC_MAX, link_id, proc_type);

    if(llc_env_ptr != NULL)
    {
        llc_procedure_t* proc = llc_env[link_id]->procedure[proc_type];

        if(proc != NULL)
        {
            bool start_timer = false;
            bool clear_timer = false;

            // check timer state to start or not the transaction timeout
            switch (proc->timer_state)
            {
                case LLC_TIMER_OFF:
                {
                    if(enable)
                    {
                        // put the timer in disabled state
                        proc->timer_state = LLC_TIMER_DISABLED;
                    } // else nothing to do
                }break;
                case LLC_TIMER_STARTED:
                {
                    if(enable)
                    {
                        // Go into Paused state
                        proc->timer_state = LLC_TIMER_PAUSED;
                        // Stop the timer.
                        clear_timer = true;
                    } // else nothing to do
                }break;
                case LLC_TIMER_PAUSED:
                {
                    if(!enable)
                    {
                        // Restart the timer
                        proc->timer_state = LLC_TIMER_STARTED;
                        // Start the timer.
                        start_timer = true;
                    } // else nothing to do
                }break;
                case LLC_TIMER_DISABLED:
                {
                    if(!enable)
                    {
                        // Return to off state
                        proc->timer_state = LLC_TIMER_OFF;
                    } // else nothing to do
                }break;
                default:
                {
                    ASSERT_INFO(0, proc->id, proc->timer_state);
                }break;
            }

            // start timer
            if(start_timer)
            {
                llc_llcp_trans_timer_set(link_id, proc_type, true);
            }
            // stop timer
            else if (clear_timer)
            {
                llc_llcp_trans_timer_set(link_id, proc_type, false);
            }
            // else nothing to do
        }
    }
}

uint8_t ROM_VT_FUNC(llc_proc_collision_check)(uint8_t link_id, uint8_t proc_id)
{
    uint8_t status = CO_ERROR_NO_ERROR;
    uint8_t loc_proc_id = llc_proc_id_get(link_id, LLC_PROC_LOCAL);

    if(loc_proc_id == proc_id)
    {
        status = CO_ERROR_LMP_COLLISION;
    }
    else
    {
        switch(loc_proc_id)
        {
            case LLC_PROC_CH_MAP_UPDATE:
            case LLC_PROC_CON_UPDATE:
            case LLC_PROC_PHY_UPDATE:
            {
                status = CO_ERROR_DIFF_TRANSACTION_COLLISION;
            } break;
            default: /* Nothing to do, not considered as collision */ break;
        }
    }

    return status;
}


#endif // #if (BLE_PERIPHERAL || BLE_CENTRAL)
/// @} LLC

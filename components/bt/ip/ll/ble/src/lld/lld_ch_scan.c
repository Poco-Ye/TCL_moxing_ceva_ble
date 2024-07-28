/**
****************************************************************************************
*
* @file lld_ch_scan.c
*
* @brief LLD Channel Scanning source code
*
* Copyright (C) RivieraWaves 2009-2021
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LLDCHSCAN
 * @ingroup LLD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"         // stack configuration
#if (BLE_OBSERVER || BLE_BROADCASTER)

#include "ke_mem.h"
#include "ke_msg.h"              // kernel messages
#include "rwip.h"

#include "lld.h"                 // link driver API
#include "lld_int.h"             // link layer driver internal

#include "sch_arb.h"             // Scheduling Arbiter
#include "sch_prog.h"            // Scheduling Programmer

#include "reg_blecore.h"         // BLE core registers
#include "reg_em_ble_cs.h"       // BLE EM Control Structure
#include "reg_em_ble_tx_desc.h"  // BLE EM TX descriptors
#include "reg_em_ble_rx_desc.h"  // BLE EM RX descriptors

#include "dbg_swdiag.h"
/*
 * DEFINES
 *****************************************************************************************
 */

/// Channel Scanning event states
enum CH_SCAN_EVT_STATE
{
    CH_SCAN_EVT_WAIT,
    CH_SCAN_EVT_ACTIVE,
    CH_SCAN_EVT_END,
};


/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// LLD Channel Scanning environment structure
struct lld_ch_scan_env_tag
{
    /// Pointer to inquiry event
    struct sch_arb_elt_tag evt;
    /// current state of the channel scan
    uint8_t state;
    /// Activity index
    uint8_t act_id;
    /// Scan interval in half-slots (312.5 us)
    uint16_t intv;
};


/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LLD Channel Scanning environment variable
__STATIC struct lld_ch_scan_env_tag* lld_ch_scan_env;


/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */

__STATIC void lld_ch_scan_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);


/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Cleanup Channel Scanning environment variable
 ****************************************************************************************
 */
__STATIC void lld_ch_scan_cleanup(void)
{
    if(lld_ch_scan_env != NULL)
    {
        // Free event memory
        ke_free(lld_ch_scan_env);
        lld_ch_scan_env = NULL;
    }
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void lld_ch_scan_evt_start_cbk(struct sch_arb_elt_tag* evt)
{
    ASSERT_ERR(&(lld_ch_scan_env->evt) == evt);

    DBG_SWDIAG(LECHSCAN, EVT_START, 1);

    if(evt != NULL)
    {
        // Point to parameters
        struct lld_ch_scan_env_tag* ch_scan_evt = (struct lld_ch_scan_env_tag*) evt;
        struct sch_prog_params prog_par;

        // Push the programming to SCH PROG
        prog_par.frm_cbk        = &lld_ch_scan_frm_cbk;
        prog_par.time.hs        = evt->time.hs;
        prog_par.time.hus       = 0;
        prog_par.cs_idx         = EM_BLE_CS_ACT_ID_TO_INDEX(ch_scan_evt->act_id);
        prog_par.dummy          = 0;
        prog_par.bandwidth      = evt->duration_min;
        prog_par.prio_1         = evt->current_prio;
        prog_par.prio_2         = 0;
        prog_par.prio_3         = evt->current_prio;
        prog_par.pti_prio       = RW_BLE_PTI_PRIO_AUTO;
        prog_par.add.ble.ae_nps = 0;
        prog_par.add.ble.iso    = 0;
        prog_par.mode           = SCH_PROG_BLE;
        sch_prog_push(&prog_par);

        // Move state
        ch_scan_evt->state = CH_SCAN_EVT_ACTIVE;
    }
    else
    {
        ASSERT_ERR(0);
    }

    DBG_SWDIAG(LECHSCAN, EVT_START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void lld_ch_scan_evt_canceled_cbk(struct sch_arb_elt_tag* evt)
{
    ASSERT_ERR(&(lld_ch_scan_env->evt) == evt);

    DBG_SWDIAG(LECHSCAN, EVT_CANCELED, 1);

    if(evt != NULL)
    {
        ASSERT_ERR(((struct lld_ch_scan_env_tag*) evt)->state == CH_SCAN_EVT_WAIT);

        // Reschedule ASAP
        if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
        {
            ASSERT_ERR(0);
        }
    }
    else
    {
        ASSERT_ERR(0);
    }

    DBG_SWDIAG(LECHSCAN, EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle Rx interrupt
 ****************************************************************************************
 */
__STATIC void lld_ch_scan_rx_isr(uint32_t timestamp)
{
    DBG_SWDIAG(LECHSCAN, RX_ISR, 1);

    if(lld_ch_scan_env != NULL)
    {
        // Check if a descriptor has been used
        if (lld_rxdesc_check(EM_BLE_CS_ACT_ID_TO_INDEX(lld_ch_scan_env->act_id)))
        {
            // Get channel information from the exchange memory
            uint16_t rxchass   = em_ble_rxchass_get(lld_env.curr_rxdesc_index);
            uint8_t  rxrssi    = (rxchass & EM_BLE_RSSI_MASK) >> EM_BLE_RSSI_LSB;
            uint8_t  usedchidx = (rxchass & EM_BLE_USED_CH_IDX_MASK) >> EM_BLE_USED_CH_IDX_LSB;

            rwip_channel_assess_ble(usedchidx, false, rxrssi, timestamp, false);

            // Free RX descriptor
            lld_rxdesc_free();
        }
    }
    else
    {
        ASSERT_ERR(0);
    }

    DBG_SWDIAG(LECHSCAN, RX_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt
 ****************************************************************************************
 */
__STATIC void lld_ch_scan_eof_isr(uint32_t timestamp, bool abort)
{
    DBG_SWDIAG(LECHSCAN, EOF_ISR, 1);

    if(lld_ch_scan_env != NULL)
    {
        struct sch_arb_elt_tag* evt = &(lld_ch_scan_env->evt);

        // Remove event
        sch_arb_remove(evt, true);

        if(lld_ch_scan_env->state == CH_SCAN_EVT_END)
        {
            // Report channel scan end to LLM
            struct lld_ch_scan_end_ind* ind = KE_MSG_ALLOC(LLD_CH_SCAN_END_IND, TASK_LLM, TASK_NONE, lld_ch_scan_end_ind);
            ind->status = CO_ERROR_NO_ERROR;
            ke_msg_send(ind);

            // Free event memory
            lld_ch_scan_cleanup();
        }
        else
        {
            if (abort == false)
            {
                // Reschedule
                evt->time.hs  = CLK_ADD_2(evt->time.hs,lld_ch_scan_env->intv);
                evt->time.hus = 0;
            }

            // Move state
            lld_ch_scan_env->state = CH_SCAN_EVT_WAIT;

            if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
            {
                ASSERT_ERR(0);
            }
        }
    }
    else
    {
        ASSERT_ERR(0);
    }

    DBG_SWDIAG(LECHSCAN, EOF_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void lld_ch_scan_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    switch(irq_type)
    {
        case SCH_FRAME_IRQ_EOF:
        case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO:
        {
            lld_ch_scan_eof_isr(timestamp, false);
        } break;
        case SCH_FRAME_IRQ_SKIP:
        case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
        {
            lld_ch_scan_eof_isr(timestamp, true);
        } break;
        case SCH_FRAME_IRQ_RX:
        {
            lld_ch_scan_rx_isr(dummy);
        } break;
        default:
        {
            ASSERT_INFO(0, dummy, irq_type);
        } break;
    }
}

/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

uint8_t ROM_VT_FUNC(lld_ch_scan_start)(uint8_t act_id, struct lld_ch_scan_params* params)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    DBG_SWDIAG(LECHSCAN, START, 1);

    // Check if Channel Scanning is inactive
    if(lld_ch_scan_env == NULL)
    {
        // Allocate event
        lld_ch_scan_env = LLD_ALLOC_EVT(lld_ch_scan_env_tag);

        if(lld_ch_scan_env != NULL)
        {
            // Point to parameters
            struct lld_ch_scan_env_tag* ch_scan_evt = lld_ch_scan_env;
            struct sch_arb_elt_tag* evt = &(ch_scan_evt->evt);
            struct le_chnl_map ch_map;
            uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(act_id);

            LLD_INIT_EVT(evt, lld_ch_scan_env_tag);

            // Init state
            lld_ch_scan_env->state = CH_SCAN_EVT_WAIT;

            // Initialize event parameters
            lld_ch_scan_env->act_id = act_id;
            ch_scan_evt->intv = params->intv<<2;

            // Set permission/status of CS as R/W but uninitialized
            DBG_MEM_PERM_SET((const void*)(REG_EM_BLE_CS_BASE_ADDR + REG_EM_BLE_CS_ADDR_GET(cs_idx)), REG_EM_BLE_CS_SIZE, true, true, true);

            // Initialize event parameters (common part)
            evt->cb_cancel        = &lld_ch_scan_evt_canceled_cbk;
            evt->cb_start         = &lld_ch_scan_evt_start_cbk;
            evt->cb_stop          = NULL;
            evt->current_prio     = rwip_priority[RWIP_PRIO_CH_SCAN_IDX].value;
            evt->duration_min     = params->scan_duration_min<<1;
            evt->time.hs          = lld_read_clock();
            evt->time.hus         = 0;
            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_CH_SCAN_IDX));

            // Initialize event parameters (channel scanning part)
            // CS Format
            em_ble_cntl_format_setf(cs_idx, EM_BLE_CS_FMT_CHAN_SCAN);

            // Define channel MAP
            // Set channel map for channel scan
            ch_map.map[0] = 0xFF;
            ch_map.map[1] = 0xFF;
            ch_map.map[2] = 0xFF;
            ch_map.map[3] = 0xFF;
            ch_map.map[4] = 0x1F;

            em_ble_chmap0_llchmap0_setf(cs_idx, co_read16p(&ch_map.map[0]));
            em_ble_chmap1_llchmap1_setf(cs_idx, co_read16p(&ch_map.map[2]));
            em_ble_chmap2_llchmap2_setf(cs_idx, ch_map.map[4]);

            // Threshold
            em_ble_thrcntl_ratecntl_rxthr_setf(cs_idx, 1);
            em_ble_thrcntl_ratecntl_rxrate_setf(cs_idx, CO_RATE_1MBPS);

            // Hopping control
            em_ble_hopcntl_pack(cs_idx, /*fhen*/     true,
                                        /*hop_mode*/ LLD_HOP_MODE_CHAN_SEL_1,
                                        /*hopint*/   1,
                                        /*chidx*/    36);

            // Set event duration
            em_ble_maxevtime_set(cs_idx, params->scan_duration_max/SLOT_SIZE);
            em_ble_minevtime_set(cs_idx, params->scan_duration_max/SLOT_SIZE);

            // Set RX window size
            em_ble_rxwincntl_pack(cs_idx, 0, params->scan_win_duration);

            // Set sync word
            em_ble_syncwl_set(cs_idx, 0x5555);
            em_ble_syncwh_set(cs_idx, 0x5555);

            if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
            {
                ASSERT_ERR(0);
            }

            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            ASSERT_ERR(0);
        }
    }

    DBG_SWDIAG(LECHSCAN, START, 0);

    return (status);
}

uint8_t ROM_VT_FUNC(lld_ch_scan_stop)(void)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    DBG_SWDIAG(LECHSCAN, STOP, 1);

    GLOBAL_INT_DISABLE();

    if(lld_ch_scan_env != NULL)
    {
        // Point to parameters
        struct lld_ch_scan_env_tag* ch_scan_param = lld_ch_scan_env;
        struct sch_arb_elt_tag* evt = &(lld_ch_scan_env->evt);

        switch(ch_scan_param->state)
        {
            case CH_SCAN_EVT_WAIT:
            {
                // Remove event
                sch_arb_remove(evt, false);

                // Report channel Scanning end to LLM
                struct lld_ch_scan_end_ind* ind = KE_MSG_ALLOC(LLD_CH_SCAN_END_IND, TASK_LLM, TASK_NONE, lld_ch_scan_end_ind);
                ind->status = CO_ERROR_NO_ERROR;
                ke_msg_send(ind);

                // Free event memory
                lld_ch_scan_cleanup();
            }
            break;

            case CH_SCAN_EVT_ACTIVE:
            {
                // Move state
                lld_ch_scan_env->state = CH_SCAN_EVT_END;
            }
            break;

            default:
            {
                // Nothing to do
            }
            break;
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    DBG_SWDIAG(LECHSCAN, STOP, 0);

    return (status);
}


void ROM_VT_FUNC(lld_ch_scan_init)(uint8_t init_type)
{
    switch (init_type)
    {
        case RWIP_INIT:
        {
            // Do nothing
        }
        break;

        case RWIP_1ST_RST:
        {
            lld_ch_scan_env = NULL;
        }
        break;

        case RWIP_RST:
        {
            // clean-up allocated memory
            lld_ch_scan_cleanup();
        }
        break;

        default:
        {
            // Do nothing
        }
        break;
    }
}

#endif // (BLE_OBSERVER || BLE_BROADCASTER)
///@} LLDCHSCAN

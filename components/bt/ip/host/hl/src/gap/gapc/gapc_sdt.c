/**
 ****************************************************************************************
 *
 * @file gapc_sdt.c
 *
 * @brief Generic Access Profile Controller - Simple Defer and Timers.
 *
 * Simple API with smallest memory footprint
 *  - 1ms to 30s Timers
 *  - Job defer that reuse timer structure
 *
 *  When link is disconnected, timers and deferred job are automatically canceled
 *
 * Copyright (C) RivieraWaves 2009-2019
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPC Generic Access Profile Controller - Simple Defer and Timer
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (GAPC_PRESENT)
#include "gapc.h"                   // GAP Connection defines
#include "gapc_int.h"               // GAP Connection internals
#include "../../inc/gap_hl_api.h"   // GAP HL API
#include "co_list.h"                // List Usage
#include "co_time.h"                // Timer Usage
#include "co_djob.h"                // Defer Job Usage
#include "co_utils.h"               // Common Utilities

/*
 * DEFINES
 ****************************************************************************************
 */

/// Mask of the time value
#define GAPC_SDT_TIME_MASK      (0xFFFF)

/// Used to know if structure is in list
#define GAPC_SDT_NOT_IN_LIST   ((co_list_hdr_t*) 0xFFFFFFFF)

/// Internal information bit field
enum gapc_sdt_info_bf
{
    /// Connection index
    GAPC_SDT_CONIDX_MASK       = 0x000000FF,
    GAPC_SDT_CONIDX_LSB        = 0,
    /// Client identifier
    GAPC_SDT_CLIENT_MASK       = 0x00001F00,
    GAPC_SDT_CLIENT_LSB        = 8,
    /// True if timer elapsed
    GAPC_SDT_TIMER_ELAPSED_BIT = 0x00004000,
    GAPC_SDT_TIMER_ELAPSED_POS = 14,
    /// True if defer job else timer
    GAPC_SDT_DJOB_BIT          = 0x00008000,
    GAPC_SDT_DJOB_POS          = 15,
    /// Timer end time
    GAPC_SDT_END_TIME_MASK     = 0xFFFF0000,
    GAPC_SDT_END_TIME_LSB      = 16,
    /// Defer job dummy information
    GAPC_SDT_DUMMY_MASK        = 0xFFFF0000,
    GAPC_SDT_DUMMY_LSB         = 16,
};

/// Internal SDT State
enum gapc_sdt_state
{
    /// Native Timer must be programmed
    GAPC_SDT_STATE_TIMER_PROG_BIT       = 0x01,
    GAPC_SDT_STATE_TIMER_PROG_POS       = 0,
    /// DJob programmed
    GAPC_SDT_STATE_DJOB_PROG_BIT        = 0x02,
    GAPC_SDT_STATE_DJOB_PROG_POS        = 1,
    /// In defer handler
    GAPC_SDT_STATE_IN_JOB_HANDLER_BIT   = 0x04,
    GAPC_SDT_STATE_IN_JOB_HANDLER_POS   = 2,
    /// In timer  handler
    GAPC_SDT_STATE_IN_TIMER_HANDLER_BIT = 0x08,
    GAPC_SDT_STATE_IN_TIMER_HANDLER_POS = 3,
    /// In timer set function
    GAPC_SDT_STATE_IN_TIMER_SET_BIT     = 0x10,
    GAPC_SDT_STATE_IN_TIMER_SET_POS     = 4,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Structure of handlers
typedef struct gapc_sdt_handler
{
    /// Timer Handler
    gapc_sdt_timer_handler_t timer;
    /// Function defer handler
    gapc_sdt_defer_handler_t defer;
} gapc_sdt_handler_t;

/*
 * MACROS
 ****************************************************************************************
 */

/// Clock difference (time_a - time_b)
#define GAPC_STD_CLK_DIFF(time_a, time_b) ((int16_t) (time_a - time_b))

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

// in gapc_le_smp.c
#if(BLE_GAPC)
extern void gapc_le_smp_timeout_handler(gapc_sdt_t* p_hdl, uint8_t conidx);
#endif // (BLE_GAPC)

// in l2cap_sig.c
#if (BLE_L2CAP)
extern void l2cap_sig_trans_timeout_handler(gapc_sdt_t* p_hdl, uint8_t conidx);
extern void l2cap_sig_req_start_handler(gapc_sdt_t* p_hdl, uint8_t conidx, uint16_t dummy);
#endif // (BLE_L2CAP)

#if (BLE_GATT)
// in gatt_proc.c
extern void gatt_proc_trans_timeout_cb(gapc_sdt_t* p_hdl, uint8_t conidx);
extern void gatt_proc_continue_defer_cb(gapc_sdt_t* p_hdl, uint8_t conidx, uint16_t dummy);
// in gatt_bearer.c
extern void gatt_bearer_eatt_estab_timer_handler(gapc_sdt_t* p_hdl, uint8_t conidx);
extern void gatt_bearer_sdu_rx_bg_handler(gapc_sdt_t* p_hdl, uint8_t conidx, uint16_t dummy);
#endif // (BLE_GATT)

/// List of function handlers
const gapc_sdt_handler_t gapc_sdt_handlers[GAPC_SDT_MAX] =
{
    #if (BLE_GAPC)
    [GAPC_SDT_SMP]         = { gapc_le_smp_timeout_handler,             NULL                          },
    #endif // (BLE_GAPC)
    #if (BLE_L2CAP)
    [GAPC_SDT_L2CAP]       = { l2cap_sig_trans_timeout_handler,      l2cap_sig_req_start_handler   },
    #endif // (BLE_L2CAP)
    #if (BLE_GATT)
    [GAPC_SDT_GATT_PROC]   = { gatt_proc_trans_timeout_cb,           gatt_proc_continue_defer_cb   },
    [GAPC_SDT_GATT_BEARER] = { gatt_bearer_eatt_estab_timer_handler, gatt_bearer_sdu_rx_bg_handler },
    #endif // (BLE_GATT)
};

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Callback execute when delayed job is started
 *
 * @param[in] p_djob  Pointer to Delayed job structure.
 ****************************************************************************************
 */
__STATIC void gapc_sdt_djob_handler(co_djob_t* p_djob)
{
    gapc_sdt_t* p_hdl = (gapc_sdt_t*) co_list_pop_front(&(gapc_env.sdt_job_queue));
    SETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_DJOB_PROG, false);
    SETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_IN_JOB_HANDLER, true);

    if(p_hdl != NULL)
    {
        // mark removed from list
        p_hdl->hdr.next = GAPC_SDT_NOT_IN_LIST;

        if(GETB(p_hdl->info_bf, GAPC_SDT_DJOB))
        {
            gapc_sdt_defer_handler_t defer_handler = gapc_sdt_handlers[GETF(p_hdl->info_bf, GAPC_SDT_CLIENT)].defer;
            // execute handler
            defer_handler(p_hdl, (uint8_t) GETF(p_hdl->info_bf, GAPC_SDT_CONIDX),
                                 (uint16_t) GETF(p_hdl->info_bf, GAPC_SDT_DUMMY));
        }
        else // Timer - possible if several timer elapsed in same time
        {
            gapc_sdt_timer_handler_t timer_handler = gapc_sdt_handlers[GETF(p_hdl->info_bf, GAPC_SDT_CLIENT)].timer;
            // execute handler
            timer_handler(p_hdl, (uint8_t) GETF(p_hdl->info_bf, GAPC_SDT_CONIDX));
        }
    }

    // check if new job to be executed
    if(!co_list_is_empty(&(gapc_env.sdt_job_queue)))
    {
        co_djob_reg(CO_DJOB_LOW, &(gapc_env.sdt_job));
        SETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_DJOB_PROG, true);
    }

    SETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_IN_JOB_HANDLER, false);
}

/**
 ****************************************************************************************
 * @brief Program Native timer
 ****************************************************************************************
 */
__STATIC void gapc_sdt_timer_prog(void)
{
    gapc_sdt_t* p_hdl = (gapc_sdt_t*) co_list_pick(&(gapc_env.sdt_timer_queue));
    uint16_t cur_time = co_time_get().ms_lsb & GAPC_SDT_TIME_MASK;
    int16_t duration  = 0;
    bool    last_elt  = false;

    SETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_TIMER_PROG, false);

    // loop on list of timer
    while(p_hdl != NULL)
    {
        duration = GAPC_STD_CLK_DIFF(GETF(p_hdl->info_bf, GAPC_SDT_END_TIME), cur_time);

        // timer elapse, pushed it in job queue
        if(duration <= 0)
        {
            // remove timer from list
            co_list_pop_front(&(gapc_env.sdt_timer_queue));

            // mark timer elapsed
            SETB(p_hdl->info_bf, GAPC_SDT_TIMER_ELAPSED, true);

            // put timer in defer queue to simplify algorithm
            co_list_push_back(&(gapc_env.sdt_job_queue), &(p_hdl->hdr));

            // request Job defer
            if(   !GETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_IN_JOB_HANDLER)
               && !GETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_DJOB_PROG))
            {
                co_djob_reg(CO_DJOB_LOW, &(gapc_env.sdt_job));
                SETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_DJOB_PROG, true);
            }
        }
        else // new timer to program found
        {
            if(p_hdl == (gapc_sdt_t*) co_list_tail(&(gapc_env.sdt_timer_queue)))
            {
                last_elt = true;
            }
            break;
        }

        // check next timer - to retrieve elapsed timer and execute next ones
        p_hdl = (gapc_sdt_t*) co_list_pick(&(gapc_env.sdt_timer_queue));
    }

    // New timer to program
    if(duration > 0)
    {
        co_time_timer_set(&(gapc_env.sdt_timer), duration);

        if(last_elt)
        {
            // update max insert duration with last timer duration programmed
            gapc_env.sdt_max_push_duration = duration;
        }
    }
    // stop timer
    else
    {
        SETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_TIMER_PROG, false);
        co_time_timer_stop(&(gapc_env.sdt_timer));
        // update max insert duration
        gapc_env.sdt_max_push_duration = 0;
    }
}

/**
 ****************************************************************************************
 * @brief Callback execute when timer elapsed
 *
 * @param[in] p_env  not used
 ****************************************************************************************
 */
__STATIC void gapc_sdt_timer_handler(void* p_env)
{
//    DBG_FUNC_ENTER(gapc_sdt_timer_handler);
    gapc_sdt_t* p_hdl = (gapc_sdt_t*) co_list_pick(&(gapc_env.sdt_timer_queue));

    SETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_IN_TIMER_HANDLER, true);

    if(p_hdl != NULL)
    {
        uint16_t cur_time = co_time_get().ms_lsb & GAPC_SDT_TIME_MASK;

        if(GAPC_STD_CLK_DIFF(GETF(p_hdl->info_bf, GAPC_SDT_END_TIME), cur_time) <= 0)
        {
            gapc_sdt_timer_handler_t timer_handler = gapc_sdt_handlers[GETF(p_hdl->info_bf, GAPC_SDT_CLIENT)].timer;

            // remove timer from list
            co_list_pop_front(&(gapc_env.sdt_timer_queue));
            p_hdl->hdr.next = GAPC_SDT_NOT_IN_LIST;

            // execute handler
            timer_handler(p_hdl, (uint8_t) GETF(p_hdl->info_bf, GAPC_SDT_CONIDX));

            // update current time to re-program since execution can be long
            cur_time = co_time_get().ms_lsb & GAPC_SDT_TIME_MASK;
        }
    }

    // program next timer
    gapc_sdt_timer_prog();

    SETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_IN_TIMER_HANDLER, false);
//    DBG_FUNC_EXIT(gapc_sdt_timer_handler);
}

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

uint16_t gapc_sdt_defer(gapc_sdt_t* p_hdl, uint16_t dummy)
{
    uint16_t status = GAP_ERR_INVALID_PARAM;
    uint8_t conidx  = GETF(p_hdl->info_bf, GAPC_SDT_CONIDX);

    // check connection exists
    if((conidx > HOST_CONNECTION_MAX) || (gapc_env.p_con[conidx] == NULL))
    {
        status = GAP_ERR_COMMAND_DISALLOWED;
    }
    else if((p_hdl != NULL) && (p_hdl->hdr.next == GAPC_SDT_NOT_IN_LIST))
    {
        SETF(p_hdl->info_bf, GAPC_SDT_DUMMY,         dummy);
        SETB(p_hdl->info_bf, GAPC_SDT_DJOB,          true);
        SETB(p_hdl->info_bf, GAPC_SDT_TIMER_ELAPSED, false);

        co_list_push_back(&(gapc_env.sdt_job_queue), &(p_hdl->hdr));

        if(   !GETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_IN_JOB_HANDLER)
           && !GETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_DJOB_PROG))
        {
            co_djob_reg(CO_DJOB_LOW, &(gapc_env.sdt_job));
            SETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_DJOB_PROG, true);
        }

        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

uint16_t gapc_sdt_timer_set(gapc_sdt_t* p_hdl, uint16_t duration)
{
    uint16_t status = GAP_ERR_INVALID_PARAM;
    uint8_t conidx  = GETF(p_hdl->info_bf, GAPC_SDT_CONIDX);

    ASSERT_ERR(duration <= GAPC_SDT_DURATION_MAX);
    ASSERT_ERR(duration != 0);

    // check connection exists
    if((conidx > HOST_CONNECTION_MAX) || (gapc_env.p_con[conidx] == NULL))
    {
        status = GAP_ERR_COMMAND_DISALLOWED;
    }
    else if((p_hdl != NULL) && (duration <= GAPC_SDT_DURATION_MAX))
    {
        co_time_t cur_time   = co_time_get();
        uint16_t  end_time   = (cur_time.ms_lsb + duration) & GAPC_SDT_TIME_MASK;
        bool      empty;

        // Remove element from list
        if(p_hdl->hdr.next != GAPC_SDT_NOT_IN_LIST)
        {
            SETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_IN_TIMER_SET, true);
            gapc_sdt_stop(p_hdl);
            SETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_IN_TIMER_SET, false);
        }

        SETF(p_hdl->info_bf, GAPC_SDT_END_TIME,      end_time);
        SETB(p_hdl->info_bf, GAPC_SDT_DJOB,          false);
        SETB(p_hdl->info_bf, GAPC_SDT_TIMER_ELAPSED, false);

        empty = co_list_is_empty(&(gapc_env.sdt_timer_queue));

        // simple way to speed up new timer insertion
        if(empty || (duration >= gapc_env.sdt_max_push_duration))
        {
            if(empty)
            {
                SETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_TIMER_PROG, true);
            }

            gapc_env.sdt_max_push_duration = duration;
            co_list_push_back(&(gapc_env.sdt_timer_queue), &(p_hdl->hdr));
        }
        else
        {
            gapc_sdt_t* p_hdl_current = (gapc_sdt_t*) co_list_pick(&(gapc_env.sdt_timer_queue));
            gapc_sdt_t* p_hdl_next    = NULL;
            gapc_sdt_t* p_hdl_prev    = NULL;

            while(p_hdl_current != NULL)
            {
                p_hdl_next = (gapc_sdt_t*) p_hdl_current->hdr.next;

                // check where timer can be insert
                if((GAPC_STD_CLK_DIFF(end_time, GETF(p_hdl_current->info_bf, GAPC_SDT_END_TIME))) < 0)
                {
                    break;
                }

                p_hdl_prev = p_hdl_current;
                p_hdl_current = p_hdl_next;
            }

            // first element of the queue
            if(p_hdl_prev == NULL)
            {
                SETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_TIMER_PROG, true);
                co_list_push_front(&(gapc_env.sdt_timer_queue), &(p_hdl->hdr));
            }
            // last element of the queue
            else if (p_hdl_next == NULL)
            {
                gapc_env.sdt_max_push_duration = duration;
                co_list_push_back(&(gapc_env.sdt_timer_queue), &(p_hdl->hdr));
            }
            // insert element into the list
            else
            {
                co_list_insert_after(&(gapc_env.sdt_timer_queue), &(p_hdl_prev->hdr), &(p_hdl->hdr));
            }
        }

        // check if timer must be reprogram
        if(    GETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_TIMER_PROG)
           && !GETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_IN_TIMER_HANDLER))
        {
            // Program timer
            gapc_sdt_timer_prog();
        }

        status = GAP_ERR_NO_ERROR;
    }
    return (status);
}



uint16_t gapc_sdt_stop(gapc_sdt_t* p_hdl)
{
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if((p_hdl != NULL) && (p_hdl->hdr.next != GAPC_SDT_NOT_IN_LIST))
    {
        // element present in job queue
        if(GETB(p_hdl->info_bf, GAPC_SDT_DJOB) || GETB(p_hdl->info_bf, GAPC_SDT_TIMER_ELAPSED))
        {
            // extract element from queue
            co_list_extract(&(gapc_env.sdt_job_queue), &(p_hdl->hdr));

            // check if no job present in the queue
            if(   !GETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_IN_JOB_HANDLER)
               && co_list_is_empty(&(gapc_env.sdt_job_queue)))
            {
                SETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_DJOB_PROG, false);
                co_djob_unreg(CO_DJOB_LOW, &(gapc_env.sdt_job));
            }
        }
        // element present in timer queue
        else
        {
            gapc_sdt_t* p_hdl_current = ((gapc_sdt_t*) co_list_pick(&(gapc_env.sdt_timer_queue)));

            if(p_hdl == p_hdl_current)
            {
                co_list_pop_front(&(gapc_env.sdt_timer_queue));

                // check if timer can be reprogram
                if(   !GETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_IN_TIMER_HANDLER)
                   && !GETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_IN_TIMER_SET))
                {
                    // Program timer
                    gapc_sdt_timer_prog();
                }
                else
                {
                    // Mark if timer must be reprogram later
                    SETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_TIMER_PROG, true);
                }
            }
            else
            {
                // extract element from timer queue
                co_list_extract(&(gapc_env.sdt_timer_queue), &(p_hdl->hdr));
            }
        }

        p_hdl->hdr.next = GAPC_SDT_NOT_IN_LIST;
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

uint16_t gapc_sdt_prepare(gapc_sdt_t* p_hdl, uint8_t conidx, uint8_t client_id)
{
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if((p_hdl != NULL) && (client_id < GAPC_SDT_MAX))
    {
        p_hdl->hdr.next = GAPC_SDT_NOT_IN_LIST;
        p_hdl->info_bf  = 0;
        SETF(p_hdl->info_bf, GAPC_SDT_CLIENT, client_id);
        SETF(p_hdl->info_bf, GAPC_SDT_CONIDX, conidx);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

void gapc_sdt_cleanup(uint8_t conidx)
{
    bool first = true;
    gapc_sdt_t* p_hdl_current = (gapc_sdt_t*) co_list_pick(&(gapc_env.sdt_job_queue));
    gapc_sdt_t* p_hdl_next    = NULL;

    while(p_hdl_current != NULL)
    {
        p_hdl_next = (gapc_sdt_t*) p_hdl_current->hdr.next;

        // check where timer can be insert
        if((GETF(p_hdl_current->info_bf, GAPC_SDT_CONIDX) == conidx))
        {
            // extract element from job queue
            co_list_extract(&(gapc_env.sdt_job_queue), &(p_hdl_current->hdr));
            p_hdl_current->hdr.next = GAPC_SDT_NOT_IN_LIST;
        }

        p_hdl_current = p_hdl_next;
    }

    p_hdl_current = (gapc_sdt_t*) co_list_pick(&(gapc_env.sdt_timer_queue));

    while(p_hdl_current != NULL)
    {
        p_hdl_next = (gapc_sdt_t*) p_hdl_current->hdr.next;

        // check where timer can be insert
        if((GETF(p_hdl_current->info_bf, GAPC_SDT_CONIDX) == conidx))
        {
            // mark timers must be reprogrammed
            if(first)
            {
                SETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_TIMER_PROG, true);
            }

            // extract element from timer queue
            co_list_extract(&(gapc_env.sdt_timer_queue), &(p_hdl_current->hdr));
            p_hdl_current->hdr.next = GAPC_SDT_NOT_IN_LIST;
        }

        p_hdl_current = p_hdl_next;
        first = false;
    }

    // check if no job present in the queue
    if(   !GETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_IN_JOB_HANDLER)
       && co_list_is_empty(&(gapc_env.sdt_job_queue)))
    {
        SETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_DJOB_PROG, false);
        co_djob_unreg(CO_DJOB_LOW, &(gapc_env.sdt_job));
    }

    // check if timer must be reprogram
    if(    GETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_TIMER_PROG)
       && !GETB(gapc_env.sdt_state_bf, GAPC_SDT_STATE_IN_TIMER_HANDLER))
    {
        // Program timer
        gapc_sdt_timer_prog();
    }
}


void gapc_sdt_init(uint8_t init_type)
{
    switch (init_type)
    {
        case RWIP_RST:
        case RWIP_1ST_RST:
        {
            // initialize Djob and native timer
            co_djob_init(&(gapc_env.sdt_job), gapc_sdt_djob_handler);
            co_time_timer_init(&(gapc_env.sdt_timer), gapc_sdt_timer_handler, NULL);
        } break;

        default:  { /* Do nothing */ } break;
    }
}
#endif // (GAPC_PRESENT)
/// @} GAPC

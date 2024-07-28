/**
 ****************************************************************************************
 *
 * @file ke_timer.c
 *
 * @brief This file contains the scheduler primitives called to create or delete
 * a task. It contains also the scheduler itself.
 *
 * Copyright (C) RivieraWaves 2009-2019
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup KE_TIMER
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stddef.h>              // standard definition
#include <stdint.h>              // standard integer
#include <stdbool.h>             // standard boolean
#include "arch.h"                // architecture

#include "ke_queue.h"            // kernel queue
#include "ke_mem.h"              // kernel memory
#include "ke_int.h"              // kernel environment
#include "ke_event.h"            // kernel event
#include "ke_timer.h"            // kernel timer
#include "ke_task.h"             // kernel task

#include "dbg_swdiag.h"          // Software diag

#include "co_time.h"             // common time module

#include "dbg.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/// Timer Object
typedef struct ke_timer
{
    /// List header
    co_list_hdr_t    hdr;
    /// time value
    co_time_timer_t  timer;
    /// message identifier
    ke_msg_id_t      id;
    /// task identifier
    ke_task_id_t     task;
} ke_timer_t;

/**
 ****************************************************************************************
 * @brief retrieve a timer in the list of programmed timer
 *
 * @param[in] timer_id      Timer identifier (message identifier type).
 * @param[in] task_id       Task identifier which will be notified
 ****************************************************************************************
 */
ke_timer_t* ke_timer_get(ke_msg_id_t const timer_id, ke_task_id_t const task_id)
{
    ke_timer_t* p_timer = (struct ke_timer*) co_list_pick(&(ke_env.queue_timer));

    // Browse list of timer to find expected one
    while (p_timer != NULL)
    {
        // if message ID and targeted task matches
        if ((p_timer->id == timer_id) && (p_timer->task == task_id))
        {
            // timer is found
            break;
        }

        // else Check next timer
        p_timer = ( ke_timer_t*) p_timer->hdr.next;
    }

    return (p_timer);
}


/**
 ****************************************************************************************
 * @brief Function executed when a kernel timer has elapsed
 *
 * @param[in] p_timer Pointer to timer which has elapsed
 ****************************************************************************************
 */
__STATIC void ke_timer_expired(ke_timer_t* p_timer)
{
    // notify the task
    ke_msg_send_basic(p_timer->id, p_timer->task, TASK_NONE);

    // remove timer from kernel timer list
    co_list_extract(&(ke_env.queue_timer), &(p_timer->hdr));

    // free the memory allocated for the timer
    ke_free(p_timer);
}

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void ke_timer_flush(void)
{
    // free all timers
    while(1)
    {
        struct ke_timer *timer = (struct ke_timer*) ke_queue_pop(&ke_env.queue_timer);
        if(timer == NULL)
            break;
        ke_free(timer);
    }
}

void ke_timer_set(ke_msg_id_t const timer_id, ke_task_id_t const task_id, uint32_t delay_ms)
{
    // Get kernel timer
    ke_timer_t *p_timer = ke_timer_get(timer_id, task_id);

    if(p_timer == NULL)
    {
        // Create new one
        p_timer        = (ke_timer_t*) ke_malloc_system(sizeof(ke_timer_t), KE_MEM_KE_MSG);
        ASSERT_ERR(p_timer);
        p_timer->id    = timer_id;
        p_timer->task  = task_id;
        // initialize timer
        co_time_timer_init(&(p_timer->timer), (co_time_timer_cb) ke_timer_expired, p_timer);

        // put timer in list
        co_list_push_back(&(ke_env.queue_timer), &(p_timer->hdr));
    }

    // program timer
    co_time_timer_set(&(p_timer->timer), delay_ms);
}

void ke_timer_clear(ke_msg_id_t const timer_id, ke_task_id_t const task_id)
{
    // Get kernel timer
    ke_timer_t *p_timer = ke_timer_get(timer_id, task_id);

    if(p_timer != NULL)
    {
        // stop timer
        co_time_timer_stop(&(p_timer->timer));

        // remove timer from kernel timer list
        co_list_extract(&(ke_env.queue_timer), &(p_timer->hdr));

        // free the memory allocated for the timer
        ke_free(p_timer);
    }
}

bool ke_timer_active(ke_msg_id_t const timer_id, ke_task_id_t const task_id)
{
    return (ke_timer_get(timer_id, task_id) != NULL);
}

///@} KE_TIMER

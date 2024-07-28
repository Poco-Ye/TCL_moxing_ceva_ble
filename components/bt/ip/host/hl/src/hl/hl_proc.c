/**
 ****************************************************************************************
 *
 * @file hl_proc.c
 *
 * @brief Procedure API
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup HOST
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // IP Configuration
#include "rwip.h"           // Initialization enum
#include "ke_mem.h"         // For procedure allocation/free
#include "co_math.h"        // For bit field manipulation
#include "co_djob.h"        // Used to defer procedure transition
#include "../inc/hl_proc.h" // Host procedure API


/*
 * MACROS
 ****************************************************************************************
 */
/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
/// HL procedure environment structure
typedef struct hl_proc_env_
{
    /// Job defer object
    co_djob_t   proc_start_djob;
    /// queue of procedure (queue) which first procedure start must be deferred
    co_list_t   proc_start_queue;
} hl_proc_env_t;

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// HL procedure environment variable
__STATIC hl_proc_env_t hl_proc_env;

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Defer procedure start execution in background
 *
 * @param[in] p_djob    Pointer to djob structure
 ****************************************************************************************
 */
__STATIC void hl_proc_start_defer_handler(co_djob_t* p_djob)
{
    hl_proc_queue_t* p_proc_queue = (hl_proc_queue_t*) co_list_pop_front(&(hl_proc_env.proc_start_queue));
    if(p_proc_queue)
    {
        // Grant procedure start
        hl_proc_transition(p_proc_queue, HL_PROC_GRANTED, GAP_ERR_NO_ERROR);

        // If another procedure must be started
        if(!co_list_is_empty(&(hl_proc_env.proc_start_queue)))
        {
            co_djob_reg(CO_DJOB_LOW, p_djob);
        }
    }
}

/**
 ****************************************************************************************
 * @brief Push a new procedure in execution queue
 *
 * @note If procedure is on top of the queue, its execution start is immediately deferred
 *
 * @param[in] p_proc_queue Pointer to procedure queue
 * @param[in] p_proc       Pointer to the procedure
 *
 * @return Status of operation creation (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC uint16_t hl_proc_push(hl_proc_queue_t* p_proc_queue, hl_proc_t* p_proc)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    bool immediate_start;
    ASSERT_ERR(p_proc_queue != NULL);
    immediate_start = co_list_is_empty(&(p_proc_queue->queue));

    co_list_push_back(&(p_proc_queue->queue), &(p_proc->hdr));

    if(immediate_start)
    {
        // defer procedure start
        co_list_push_back(&(hl_proc_env.proc_start_queue), &(p_proc_queue->hdr));
        co_djob_reg(CO_DJOB_LOW, &(hl_proc_env.proc_start_djob));
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Pop first procedure present in execution queue - Free automatically procedure object
 *
 * @note If another procedure is on top of the queue, its execution start is immediately deferred
 *
 * @param[in] p_proc_queue Pointer to procedure queue
 *
 * @return Status of operation creation (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC uint16_t hl_proc_pop(hl_proc_queue_t* p_proc_queue)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    hl_proc_t* p_proc = NULL;
    ASSERT_ERR(p_proc_queue != NULL);
    p_proc = (hl_proc_t*) co_list_pop_front(&(p_proc_queue->queue));
    ASSERT_ERR(p_proc != NULL);

    p_proc->p_itf->cleanup(p_proc);

    if(!co_list_is_empty(&(p_proc_queue->queue)))
    {
        // defer procedure start
        co_list_push_back(&(hl_proc_env.proc_start_queue), &(p_proc_queue->hdr));
        co_djob_reg(CO_DJOB_LOW, &(hl_proc_env.proc_start_djob));
    }

    return (status);
}


/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

uint16_t hl_proc_create(hl_proc_queue_t* p_proc_queue, uint16_t proc_size, const hl_proc_itf_t *p_itf, hl_proc_t** pp_proc)
{
    uint16_t status = GAP_ERR_NO_ERROR;

    ASSERT_ERR(proc_size >= sizeof(hl_proc_t));
    if(proc_size < sizeof(hl_proc_t))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else
    {
        // Create procedure object
        hl_proc_t* p_proc = (hl_proc_t*) ke_malloc_user(proc_size, KE_MEM_KE_MSG);
        if(p_proc != NULL)
        {
            p_proc->p_itf = p_itf;
            *pp_proc      = p_proc;

            // automatically push procedure in execution queue
            hl_proc_push(p_proc_queue, p_proc);
        }
        else
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }

    return  (status);
}

void hl_proc_cleanup(hl_proc_t* p_proc)
{
    ke_free(p_proc);
}

void hl_proc_transition(hl_proc_queue_t* p_proc_queue,  uint8_t event, uint16_t status)
{
    hl_proc_t* p_proc = NULL;
    ASSERT_ERR(p_proc_queue != NULL);
    p_proc = (hl_proc_t*) co_list_pick(&(p_proc_queue->queue));
    ASSERT_ERR(p_proc != NULL);

    // ask procedure to perform state machine transition
    if(p_proc->p_itf->transition(p_proc, event, status))
    {
        hl_proc_pop(p_proc_queue);
    }
}

void hl_proc_free(hl_proc_t* p_proc)
{
    p_proc->p_itf->cleanup(p_proc);
}

hl_proc_t* hl_proc_get(hl_proc_queue_t* p_proc_queue)
{
    hl_proc_t* p_proc = NULL;
    ASSERT_ERR(p_proc_queue != NULL);
    p_proc = (hl_proc_t*) co_list_pick(&(p_proc_queue->queue));
    return (p_proc);
}

void hl_proc_stop(hl_proc_queue_t* p_proc_queue)
{
    hl_proc_pop(p_proc_queue);
}

void hl_proc_queue_abort(hl_proc_queue_t* p_proc_queue, uint16_t reason)
{
    // clean all procedure present in the queue
    while(!co_list_is_empty(&p_proc_queue->queue))
    {
        hl_proc_t* p_proc = (hl_proc_t*) co_list_pop_front(&(p_proc_queue->queue));
        p_proc->p_itf->transition(p_proc, HL_PROC_FINISHED, reason);
        p_proc->p_itf->cleanup(p_proc);
    }

    // remove from defer
    if(   co_list_extract(&(hl_proc_env.proc_start_queue), &(p_proc_queue->hdr))
       && co_list_is_empty(&(hl_proc_env.proc_start_queue)))
    {
        co_djob_unreg(CO_DJOB_LOW, &(hl_proc_env.proc_start_djob));
    }
}


void hl_proc_queue_initialize(hl_proc_queue_t* p_proc_queue)
{
    // clean all procedure present in the queue
    while(!co_list_is_empty(&p_proc_queue->queue))
    {
        hl_proc_t* p_proc = (hl_proc_t*) co_list_pop_front(&(p_proc_queue->queue));
        p_proc->p_itf->cleanup(p_proc);
    }
}

bool hl_proc_is_waiting_grant(hl_proc_queue_t* p_proc_queue)
{
    return (co_list_find(&(hl_proc_env.proc_start_queue), &(p_proc_queue->hdr)));
}


/**
 ****************************************************************************************
 * @brief Initialize Procedure module
 *
 * @param[in] init_type  Type of initialization (see enum #rwip_init_type)
 ****************************************************************************************
 */
void hl_proc_initialize(uint8_t init_type)
{
    co_djob_init(&(hl_proc_env.proc_start_djob), hl_proc_start_defer_handler);
    co_list_init(&(hl_proc_env.proc_start_queue));
}

/// @} HOST

/**
 ****************************************************************************************
 *
 * @file hl_proc.h

 * @brief HOST procedure API
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */


#ifndef HL_PROC_H_
#define HL_PROC_H_

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
#include "rwip_config.h"
#include <stdint.h>

#include "co_list.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Procedure Event type (some additional procedure event can be added per procedure
enum hl_proc_event
{
    /// Procedure start granted
    HL_PROC_GRANTED,
    /// Default handler for procedure termination (if procedure is simple)
    HL_PROC_FINISHED,
    /// First event identifier available for a procedure
    HL_PROC_EVENT_FIRST,

    /// Simple transition for procedure that requires only 3 transition events
    HL_PROC_CONTINUE = HL_PROC_EVENT_FIRST,

    /// Invalid procedure state transition
    HL_PROC_INVALID = 0xFF,
};


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

typedef struct hl_proc hl_proc_t;

/// Type used to redefine p_proc type in transition handler
typedef bool (*hl_proc_transition_cb) (hl_proc_t* p_proc, uint8_t event, uint16_t status);

/// Procedure Interface that a procedure shall follow
typedef struct hl_proc_itf
{
    /**
     ****************************************************************************************
     * @brief Function called when an event is trigger that creates a transition in procedure state machine
     *
     * @param[in] p_proc     Pointer to procedure object
     * @param[in] event      Event type receive that induce procedure state transition
     * @param[in] status     Status linked to event transition (see enum #hl_err)
     *
     * @return True if procedure is finished and can be automatically clean-up, False if procedure continue
     ****************************************************************************************
     */
    bool (*transition) (hl_proc_t* p_proc, uint8_t event, uint16_t status);

    /**
     ****************************************************************************************
     * @brief Ask Procedure to clean-up itself
     *
     * @note Shall call at the end @see hl_proc_cleanup ;
     *       it's recommended to point directly to this function
     *
     * @param[in] p_proc     Pointer to procedure object
     ****************************************************************************************
     */
    void (*cleanup) (hl_proc_t* p_proc);
} hl_proc_itf_t;

/// Structure used to retrieve information about User initiated procedure
typedef struct hl_proc
{
    /// Procedure is part of a procedure queue
    co_list_hdr_t        hdr;
    /// Procedure Interface
    const hl_proc_itf_t* p_itf;
} hl_proc_t;

/// Procedure queue object, the procedure start transition is deferred
typedef struct hl_proc_queue
{
    /// Needed to defer a procedure transition execution
    co_list_hdr_t        hdr;
    /// Procedure queue
    co_list_t            queue;
} hl_proc_queue_t;


/*
 * MACROS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Create a new procedure
 *
 * @param[in]  proc_size    Size of procedure parameters shall be at least sizeof(hl_proc_t)
 * @param[in]  p_itf        Procedure interface
 * @param[out] pp_proc      Pointer to new procedure
 *
 * @return Status of operation creation (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t hl_proc_create(hl_proc_queue_t* p_proc_queue, uint16_t proc_size, const hl_proc_itf_t *p_itf, hl_proc_t** pp_proc);


/**
 ****************************************************************************************
 * @brief Ask a cleanup of procedure memory - shall be called to destroy procedure
 *
 * @param[in] p_proc       Pointer to the procedure
 ****************************************************************************************
 */
void hl_proc_cleanup(hl_proc_t* p_proc);

/**
 ****************************************************************************************
 * @brief Ask procedure to perform a transition
 *
 * This function shall be called to ask procedure on top of execution queue to perform a transition
 *
 * @param[in] p_proc_queue  Pointer to procedure queue
 * @param[in] event         Event type receive that induce procedure state transition
 * @param[in] status        Status code associated to the procedure transition state
 ****************************************************************************************
 */
void hl_proc_transition(hl_proc_queue_t* p_proc_queue, uint8_t event, uint16_t status);

/**
 ****************************************************************************************
 * @brief Free procedure object
 *
 * @note Shall not be called if procedure is in execution queue
 *
 * @param[in] p_proc Pointer to procedure object
 *
 * @return Status of operation creation (see enum #hl_err)
 ****************************************************************************************
 */
void hl_proc_free(hl_proc_t* p_proc);

/**
 ****************************************************************************************
 * @brief Get pointer to the procedure on top of execution queue
 *
 * @param[in] p_proc_queue Pointer to procedure queue
 *
 * @return Procedure on top of execution queue, NULL if no procedure under execution
 ****************************************************************************************
 */
hl_proc_t* hl_proc_get(hl_proc_queue_t* p_proc_queue);


/**
 ****************************************************************************************
 * @brief Stop procedure without doing a transition state
 *
 * @param[in] proc_type  Procedure type (see enum #gapm_proc_type)
 ****************************************************************************************
 */
void hl_proc_stop(hl_proc_queue_t* p_proc_queue);

/**
 ****************************************************************************************
 * @brief Abort and clean procedures present in a procedure queue queue
 *        Inform all procedures aborted
 *
 * @param[in] p_proc_queue Pointer to procedure queue
 * @param[in] reason       Queue clean reason (see enum #hl_err)
 ****************************************************************************************
 */
void hl_proc_queue_abort(hl_proc_queue_t* p_proc_queue, uint16_t reason);

/**
 ****************************************************************************************
 * @brief Initialize procedures queue
 *
 * @param[in] p_proc_queue Pointer to procedure queue
 ****************************************************************************************
 */
void hl_proc_queue_initialize(hl_proc_queue_t* p_proc_queue);


/**
 ****************************************************************************************
 * @brief Get if procedure is waiting to be granted.
 *
 * @param[in] p_proc_queue Pointer to procedure queue
 *
 * @return True if waiting for grant, false otherwise.
 ****************************************************************************************
 */
bool hl_proc_is_waiting_grant(hl_proc_queue_t* p_proc_queue);

/// @} HOST

#endif /* HL_PROC_H_ */

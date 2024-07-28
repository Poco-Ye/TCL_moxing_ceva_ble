/**
 ****************************************************************************************
 *
 * @file gapm_actv.c
 *
 * @brief Generic Access Profile Manager - Activity manager module.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPM_ACTV Generic Access Profile Manager - Activity manager module.
 * @ingroup GAPM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <string.h>

#include "rwip_config.h"
#include "rwip.h"

#include "gap.h"
#include "gapm_int.h"
#include "gapc.h"
#include "ke_mem.h"
#include "hci.h"
#include "co_utils.h"
#include "co_math.h"

/*
 * DEFINES
 ****************************************************************************************
 */


/// Mask of available activities
#define GAPM_ACTV_MASK    (CO_BIT(GAPM_ACTV_NB) - 1)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */



/*
 * TYPE DEFINITION
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */

__STATIC bool gapm_actv_proc_transition(gapm_actv_proc_t* p_proc, uint8_t event, uint16_t status);

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Activity control procedure interface
__STATIC const hl_proc_itf_t gapm_actv_proc_itf =
{
    .transition  = (hl_proc_transition_cb) gapm_actv_proc_transition,
    .cleanup     = hl_proc_cleanup,
};



/*
 * PROCEDURE TRANSITION - State machine
 ****************************************************************************************
 */

/// Start Activity procedure handler
__STATIC uint16_t gapm_actv_proc_start_transition(gapm_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event,
                                                  uint16_t status, bool* p_finished)
{
    status = p_actv->p_itf->cb_start_transition(p_actv, p_proc, event, status, p_finished);
    if(*p_finished)
    {
        if(status == GAP_ERR_NO_ERROR)
        {
            // Increase number of started activities
            gapm_env.nb_started_actvs++;
            // Update activity state
            p_actv->state = GAPM_ACTV_STARTED;
        }

        gapm_env.actv_busy_bf &= ~CO_BIT(p_actv->idx);
    }
    return (status);
}

/// Stop Activity procedure handler
__STATIC uint16_t gapm_actv_proc_stop_transition(gapm_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event,
                                                 uint16_t status, bool* p_finished)
{
    status = p_actv->p_itf->cb_stop_transition(p_actv, p_proc, event, status, p_finished);
    if(*p_finished)
    {
        if(status == GAP_ERR_NO_ERROR)
        {
            gapm_actv_stopped(p_actv, p_proc->event_status);
        }
        gapm_env.actv_busy_bf &= ~CO_BIT(p_actv->idx);
    }
    return (status);
}

/// Delete Activity procedure handler
__STATIC uint16_t gapm_actv_proc_delete_transition(gapm_actv_t *p_actv, gapm_actv_proc_t* p_proc, uint8_t event,
                                                   uint16_t status, bool* p_finished)
{
    status = p_actv->p_itf->cb_delete_transition(p_actv, p_proc, event, status, p_finished);
    if(*p_finished)
    {
        const gapm_actv_cb_t* p_cbs = p_actv->p_cbs;
        uint32_t dummy = p_actv->dummy;
        gapm_env.actv_busy_bf &= ~CO_BIT(p_actv->idx);

        if(status == GAP_ERR_NO_ERROR)
        {
            // Decrease number of created activities
            gapm_env.nb_created_actvs--;
            // Free the activity structure
            p_actv->p_itf->cb_clean(p_actv, false);
        }

        // Send procedure complete since activity has been deleted
        p_cbs->proc_cmp(dummy, p_proc->proc_id, p_proc->actv_idx, status);
    }
    return (status);
}

/**
 ****************************************************************************************
 * @brief Function called when an event is trigger that creates a transition in procedure state machine
 *
 * @param[in] p_proc     Pointer to procedure object
 * @param[in] event      Event type receive that induce procedure state transition
 * @param[in] status     Status linked to event transition (see enum #hl_err)
 *
 * @return True if procedure is finished, false otherwise
 ****************************************************************************************
 */
__STATIC bool gapm_actv_proc_transition(gapm_actv_proc_t* p_proc, uint8_t event, uint16_t status)
{
    bool finished = true;
    gapm_actv_t *p_actv = gapm_actv_get(p_proc->actv_idx);
    ASSERT_ERR(p_actv != NULL);
    status = p_proc->transition_cb(p_actv, p_proc, event, status, &finished);

    if(finished)
    {
        // inform upper layer sw about procedure termination
        if(p_proc->trigger_cmp)
        {
            gapm_actv_send_proc_cmp(p_actv, p_proc->proc_id, status);
        }
    }

    return (finished);
}

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/// Perform stop and control execution of procedure complete callback
__STATIC uint16_t gapm_actv_stop_create(uint8_t actv_idx, bool quiet)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    gapm_actv_t* p_actv =  gapm_actv_get(actv_idx);

    do
    {
        gapm_actv_proc_t* p_proc;
        // Check if activity exists and if it is in GAPM_ACTV_STARTED state
        if (!p_actv || (p_actv->state != GAPM_ACTV_STARTED)) break;

        // start procedure
        status = gapm_actv_proc_create(p_actv, GAPM_ACTV_STOP, sizeof(gapm_actv_proc_t), !quiet,
                                       gapm_actv_proc_stop_transition, &p_proc);
        if(status != GAP_ERR_NO_ERROR) break;
        gapm_env.actv_busy_bf |= CO_BIT(p_actv->idx);
    } while(0);

    return (status);
}


/*
 * EXTERNAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void gapm_actv_initialize(uint8_t init_type)
{
    if (init_type == RWIP_RST)
    {
        // clean-up activity memory
        while(gapm_env.actv_bf != 0)
        {
            uint8_t actv_idx = co_ctz(gapm_env.actv_bf);
            gapm_actv_t* p_actv = gapm_env.p_actvs[actv_idx];
            p_actv->p_itf->cb_clean(p_actv, true);
        }
    }
    else if (init_type == RWIP_1ST_RST)
    {
        memset(&(gapm_env.p_actvs), 0, sizeof(gapm_env.p_actvs));
    }

    // Initialize counters of activities
    gapm_env.nb_created_actvs = 0;
    gapm_env.nb_started_actvs = 0;
    gapm_env.nb_connect_actvs = 0;

    #if (BLE_HOST_PRESENT)
    // Initialize number of devices in the white list
    gapm_env.nb_dev_wl     = 0;

    #if (HL_LE_BROADCASTER)
    gapm_env.nb_adv_actv   = 0;
    #endif //(HL_LE_BROADCASTER)

    #if (HL_LE_OBSERVER)
    gapm_env.scan_actv_idx         = GAPM_ACTV_INVALID_IDX;
    #endif //(HL_LE_OBSERVER)
    #if (HL_LE_CENTRAL)
    gapm_env.init_actv_idx         = GAPM_ACTV_INVALID_IDX;
    #endif //(HL_LE_CENTRAL)

    gapm_env.test_actv_idx         = GAPM_ACTV_INVALID_IDX;
    #endif // (BLE_HOST_PRESENT)

    #if(BT_HOST_PRESENT)
    gapm_env.inquiry_actv_idx      = GAPM_ACTV_INVALID_IDX;
    gapm_env.inquiry_scan_actv_idx = GAPM_ACTV_INVALID_IDX;
    gapm_env.page_scan_actv_idx    = GAPM_ACTV_INVALID_IDX;
    gapm_env.page_actv_idx         = GAPM_ACTV_INVALID_IDX;
    gapm_env.sdp_rec_hdl_cnt       = 0;
    #endif // (BT_HOST_PRESENT)
    gapm_env.actv_busy_bf  = 0;
}


gapm_actv_t* gapm_actv_get(uint8_t actv_idx)
{
    gapm_actv_t* p_actv = NULL;
    if(actv_idx < GAPM_ACTV_NB)
    {
        p_actv = gapm_env.p_actvs[actv_idx];
    }
    return (p_actv);
}

uint16_t gapm_actv_create(uint8_t type, uint8_t sub_type, uint32_t dummy, uint8_t size,
                          const gapm_actv_itf_t* p_itf, const gapm_actv_cb_t* p_cbs, gapm_actv_t** pp_actv)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    uint8_t actv_idx = GAPM_ACTV_INVALID_IDX;
    uint32_t ava_actv_bf = ~gapm_env.actv_bf & GAPM_ACTV_MASK;


    // Sanity check on callback interface
    if ((p_cbs == NULL) || (p_cbs->proc_cmp == NULL) || (p_cbs->stopped == NULL))
    {
        status = GAP_ERR_MISSING_CALLBACK;
    }
    else if(ava_actv_bf != 0)
    {
        gapm_actv_t *p_actv = NULL;
        actv_idx = co_ctz(ava_actv_bf);

        // Allocated activity structure
        ASSERT_ERR(size >= sizeof(gapm_actv_t));
        p_actv = (gapm_actv_t *)ke_malloc_user(size, KE_MEM_ENV);

        if (p_actv != NULL)
        {
            memset(p_actv, 0, size); // TODO [NATIVE API] remove - no reason to be present

            // initialize activity
            p_actv->idx           = actv_idx;
            p_actv->state         = GAPM_ACTV_CREATING;
            p_actv->p_itf         = p_itf;
            p_actv->p_cbs         = p_cbs;
            p_actv->type          = type;
            p_actv->subtype       = sub_type;
            p_actv->dummy         = dummy;

            // Store the pointer to the activity structure
            gapm_env.p_actvs[actv_idx] = p_actv;
            gapm_env.actv_bf |= CO_BIT(actv_idx);

            status   = GAP_ERR_NO_ERROR;
            *pp_actv = p_actv;
        }
    }

    return (status);
}

uint16_t gapm_actv_proc_create(gapm_actv_t* p_actv, uint8_t proc_id, uint16_t size, bool trigger_cmp,
                               gapm_actv_proc_transition_cb transition_cb, gapm_actv_proc_t** pp_proc)
{
    uint16_t status;

    do
    {
        gapm_actv_proc_t *p_proc;

        if((gapm_env.actv_busy_bf & CO_BIT(p_actv->idx)) != 0)
        {
            status = GAP_ERR_BUSY;
            break;
        }

        // create procedure and start it
        ASSERT_ERR(size >= sizeof(gapm_actv_proc_t));
        status = gapm_proc_create(GAPM_PROC_AIR, size, &gapm_actv_proc_itf, (hl_proc_t**)&p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        p_proc->actv_idx       = p_actv->idx;
        p_proc->proc_id        = proc_id;
        p_proc->trigger_cmp    = trigger_cmp;
        p_proc->transition_cb  = transition_cb;
        p_proc->event_status   = GAP_ERR_NO_ERROR;

        *pp_proc = p_proc;
    } while(0);

    return (status);
}

void gapm_actv_clean(gapm_actv_t* p_actv, bool reset)
{
    uint8_t actv_idx = p_actv->idx;

    // Free the allocated memory
    ke_free(p_actv);
    gapm_env.p_actvs[actv_idx] = NULL;
    gapm_env.actv_bf &= ~CO_BIT(actv_idx);
}

uint16_t gapm_actv_info_get(uint8_t actv_idx, uint8_t* p_act_type, uint8_t* p_act_sub_type)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;

    gapm_actv_t *p_actv = gapm_actv_get(actv_idx);
    if(p_actv != NULL)
    {
        *p_act_type     = p_actv->type;
        *p_act_sub_type = p_actv->subtype;
        status = GAP_ERR_NO_ERROR;
    }
    return (status);
}


void gapm_actv_created(gapm_actv_t *p_actv)
{
    // Increase number of created activities
    gapm_env.nb_created_actvs++;

    // Update activity state
    p_actv->state = GAPM_ACTV_CREATED;
}


void gapm_actv_send_proc_cmp(gapm_actv_t *p_actv, uint8_t proc_id, uint16_t status)
{
    p_actv->p_cbs->proc_cmp(p_actv->dummy, proc_id, p_actv->idx, status);
}


void gapm_actv_stopped(gapm_actv_t *p_actv, uint16_t reason)
{
    // Decrease number of started activities
    gapm_env.nb_started_actvs--;

    // Update activity state
    p_actv->state = GAPM_ACTV_CREATED;

    // inform upper layer application
    p_actv->p_cbs->stopped(p_actv->dummy, p_actv->idx, reason);
}

uint16_t gapm_actv_start(gapm_actv_t *p_actv, uint16_t size, gapm_actv_proc_t** pp_proc)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;

    do
    {
        // start procedure
        status = gapm_actv_proc_create(p_actv, GAPM_ACTV_START, size, true,
                                       gapm_actv_proc_start_transition, pp_proc);
        if(status != GAP_ERR_NO_ERROR) break;
        gapm_env.actv_busy_bf |= CO_BIT(p_actv->idx);
    } while(0);

    return (status);
}

uint16_t gapm_actv_stop_quiet(uint8_t actv_idx)
{
    return (gapm_actv_stop_create(actv_idx, true));
}

uint16_t gapm_actv_stop(uint8_t actv_idx)
{
    return (gapm_actv_stop_create(actv_idx, false));
}

uint16_t gapm_actv_stop_all(uint32_t dummy, gapm_proc_cmp_cb cmp_cb)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

    // TODO [NATIVE API] Not yet supported

    return (status);
}

uint16_t gapm_actv_delete(uint8_t actv_idx)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    gapm_actv_t* p_actv =  gapm_actv_get(actv_idx);

    do
    {
        gapm_actv_proc_t* p_proc;

        // Check if activity exists and if it is in GAPM_ACTV_CREATE state
        if (!p_actv || (p_actv->state != GAPM_ACTV_CREATED)) break;

        // start procedure
        status = gapm_actv_proc_create(p_actv, GAPM_ACTV_DELETE, sizeof(gapm_actv_proc_t), false,
                                       gapm_actv_proc_delete_transition, &p_proc);
        if(status != GAP_ERR_NO_ERROR) break;
        gapm_env.actv_busy_bf |= CO_BIT(p_actv->idx);
    } while(0);

    return (status);
}

uint16_t gapm_actv_delete_all(uint32_t dummy, gapm_proc_cmp_cb cmp_cb)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;

//    do
//    {
//        if (!cmp_cb) break;
//
//        // check if activity exists and all are stopped // TODO [NATIVE API] Stop can be performed in parallel
//        // TODO [NATIVE API] add sonething to check if an activity is busy
//        // busy means starting, stopping, creating, deleting
//        if((gapm_env.actv_bf == 0) || (gapm_env.started_actvs != 0))
//        {
//            status = GAP_ERR_COMMAND_DISALLOWED;
//            break;
//        }
//
//        // start procedure
//        status = gapm_actv_proc_create(GAP_INVALID_ACTV_IDX, false, dummy, cmp_cb);
//    } while(0); TODO [NATIVE API] not yet supported


    return (status);
}



/// @} GAPM_ACTV

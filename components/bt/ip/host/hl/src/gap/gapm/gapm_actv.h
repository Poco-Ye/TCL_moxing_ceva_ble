/**
 ****************************************************************************************
 *
 * @file gapm_actv.h
 *
 * @brief Generic Access Profile Manager -  Activity interface
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


#ifndef _GAPM_ACTV_H_
#define _GAPM_ACTV_H_

/**
 ****************************************************************************************
 * @ingroup GAPM
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#include "co_buf.h"
#include "hl_hci.h"

#include <stdint.h>
#include <stdbool.h>


/*
 * DEFINES
 ****************************************************************************************
 */

/// Number of supported activities in host
#define GAPM_ACTV_NB            (HOST_ACTIVITY_MAX)
/// Invalid activity identifier
#define GAPM_ACTV_INVALID_IDX   (0xFF)


/*
 * INTERNAL API TYPES
 ****************************************************************************************
 */

/// Activity states
enum gapm_actv_state
{
    /// Activity is being created - next state is CREATED
    GAPM_ACTV_CREATING = 0,
    /// Activity has been successfully created
    GAPM_ACTV_CREATED,
    /// Activity has been successfully started
    GAPM_ACTV_STARTED,
};


/*
 * MACROS
 ****************************************************************************************
 */



/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


typedef struct gapm_actv gapm_actv_t;
typedef struct gapm_actv_proc gapm_actv_proc_t;

/**
 ****************************************************************************************
 * @brief Function called to destruct an activity
 *
 * @param[in]  p_actv      Pointer to activity structure
 * @param[in]  reset       True if reset on-going, false otherwise
 *
 *****************************************************************************************
 */
typedef void (*gapm_actv_clean_cb)(gapm_actv_t* p_actv, bool reset);

/**
 ****************************************************************************************
 * @brief Function called by activity manager to update state machine of an activity procedure
 *
 * @note A returned error status is not considered as a finished procedure, it's considered only
 *       if finished status is set to true.
 *
 * @param[in]  p_actv      Pointer to activity structure
 * @param[in]  p_proc      Pointer to procedure structure
 * @param[in]  event       Event Transition
 * @param[in]  status      Status code of event transition (see enum #hl_err)
 * @param[out] p_finished  Return if procedure is over
 *
 * @return Function execution status of procedure transition (see enum #hl_err)
 *****************************************************************************************
 */
typedef uint16_t (*gapm_actv_proc_transition_cb)(gapm_actv_t* p_actv, gapm_actv_proc_t* p_proc, uint8_t event,
                                                 uint16_t status, bool* p_finished);


/// Activity control procedure structure
typedef struct gapm_actv_proc
{
    /// Inherited from default procedure object
    hl_proc_t                    hdr;
    /// Pointer to transition callback to execute each procedure event received
    gapm_actv_proc_transition_cb transition_cb;
    /// Activity index
    uint8_t                      actv_idx;
    /// Procedure identifier (see enum #gapm_actv_proc_id)
    uint8_t                      proc_id;
    /// Trigger complete event at end of procedure execution
    /// True if all activity must be stopped or deleted
    bool                         trigger_cmp;
    /// Event status to return ; could be different from procedure status (example: stop status)
    uint8_t                      event_status;
} gapm_actv_proc_t;


/// Activity interface
typedef struct gapm_actv_itf
{
    /// Function called to destruct activity object
    gapm_actv_clean_cb              cb_clean;
    /// Function to be called for activity start
    gapm_actv_proc_transition_cb    cb_start_transition;
    /// Function to be called for activity stop
    gapm_actv_proc_transition_cb    cb_stop_transition;
    /// Function to be called for activity delete
    gapm_actv_proc_transition_cb    cb_delete_transition;
    /// Function to be called for address renewal
    gapm_actv_proc_transition_cb    cb_addr_renew_transition;
} gapm_actv_itf_t;

/// GAP Manager activity structure (common part for advertising, scanning,
/// initiating and periodic synchronization activities)
typedef struct gapm_actv
{
    /// Activity mandatory function interface
    const gapm_actv_itf_t* p_itf;
    /// Activity application callback interface
    const gapm_actv_cb_t*  p_cbs;
    /// Dummy parameter provided by upper layer
    uint32_t               dummy;
    /// Identifier
    uint8_t                idx;
    /// Type (see enum #gapm_actv_type)
    uint8_t                type;
    /// Subtype - meaning depends on activity type
    ///  - Advertising activity: see enum #gapm_adv_type
    ///  - Scanning activity: see enum #gapm_scan_type
    ///  - Initiating activity: see enum #gapm_init_type
    ///  - Periodic Synchronization activity: see enum #gapm_per_sync_type
    uint8_t                subtype;
    /// State (see enum #gapm_actv_state)
    uint8_t                state;
} gapm_actv_t;

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
 * @brief  Initialize the environment by cleaning all existing activities
 *
 * @param[in] init_type  Type of initialization (see enum #rwip_init_type)
 ****************************************************************************************
 */
void gapm_actv_initialize(uint8_t init_type);

/**
 ****************************************************************************************
 * @brief  Initialize the environment of address management
 *
 * @param[in] init_type  Type of initialization (see enum #rwip_init_type)
 ****************************************************************************************
 */
void gapm_le_actv_initialize(uint8_t init_type);

/**
 ****************************************************************************************
 * @brief Allocate an activity structure for a given identifier. Provided identifer is
 * then no more marked as available.
 * It is considered here that provided activity identifier is an available one.
 *
 * @param[in]  type           Type of activity (see enum #gapm_actv_type)
 * @param[in]  sub_type       Subtype - meaning depends on activity type
 *                              - Advertising activity: see enum #gapm_adv_type
 *                              - Scanning activity: see enum #gapm_scan_type
 *                              - Initiating activity: see enum #gapm_init_type
 *                              - Periodic Synchronization activity: see enum #gapm_per_sync_type
 * @param[in]  dummy          Dummy parameter provided by upper layer
 * @param[in]  size           Size of the activity structure to be allocated
 * @param[in]  p_itf          Activity mandatory function interface
 * @param[in]  p_cbs          Activity application callback interface
 * @param[out] pp_actv        Pointer to the allocated activity structure
 *
 * @return Activity creation status (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapm_actv_create(uint8_t type, uint8_t sub_type, uint32_t dummy, uint8_t size,
                          const gapm_actv_itf_t* p_itf, const gapm_actv_cb_t* p_cbs, gapm_actv_t** pp_actv);

/**
 ****************************************************************************************
 * @brief Default destructor of an activity
 *
 * @param[in] p_actv      Pointer to the activity
 * @param[in] reset       True if activity is clean-up due to a reset, False otherwise
 ****************************************************************************************
 */
void gapm_actv_clean(gapm_actv_t* p_actv, bool reset);

/**
 ****************************************************************************************
 * @brief Get Activity pointer from Activity index
 *
 * @param[in] actv_idx      Activity index
 * @param[in] size          Size of the activity structure to be allocated
 *
 * @return Pointer to the allocated activity structure
 ****************************************************************************************
 */
gapm_actv_t* gapm_actv_get(uint8_t actv_idx);


/**
 ****************************************************************************************
 * Start an activity
 *
 * @param[in]  p_actv         Pointer to activity structure
 * @param[in]  size           Size of the procedure
 * @param[out] pp_proc        Pointer to the allocated procedure structure.
 *
 * @return function execution status (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapm_actv_start(gapm_actv_t *p_actv, uint16_t size, gapm_actv_proc_t** pp_proc);

/**
 ****************************************************************************************
 * @brief Stop quietly an activity (no procedure complete callback function executed)
 *
 * @param[in] actv_idx          Activity local index
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapm_actv_stop_quiet(uint8_t actv_idx);

/**
 ****************************************************************************************
 * Create an activity procedure, used to centralize all activity procedure transition
 *
 * @param[in]  p_actv         Pointer to activity structure
 * @param[in]  proc_id        Procedure identifier (see enum #gapm_actv_proc_id)
 * @param[in]  size           Size of the procedure
 * @param[in]  trigger_cmp    Trigger complete event at end of procedure execution
 * @param[in]  transition_cb  Pointer to transition callback to execute each procedure event received
 * @param[out] pp_proc        Pointer to the allocated procedure structure.
 *
 * @return function execution status (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapm_actv_proc_create(gapm_actv_t* p_actv, uint8_t proc_id, uint16_t size, bool trigger_cmp,
                               gapm_actv_proc_transition_cb transition_cb, gapm_actv_proc_t** pp_proc);

/**
 ****************************************************************************************
 * @brief Notify the activity module that activity creation procedure is over.
 *
 * @param[in] p_actv    Pointer to the structure describing the created activity
 ****************************************************************************************
 */
void gapm_actv_created(gapm_actv_t *p_actv);

/**
 ****************************************************************************************
 * @brief Notify the activity module that an activity has well been stopped.
 *
 * @param[in] p_actv    Pointer to the structure describing the stopped activity
 * @param[in] reason    Activity stop reason (see enum #hl_err)
 ****************************************************************************************
 */
void gapm_actv_stopped(gapm_actv_t *p_actv, uint16_t reason);


/**
 ****************************************************************************************
 * @brief Inform upper layer software about a procedure termination
 *
 * @param[in] p_actv    Pointer to the structure describing the stopped activity
 * @param[in] proc_id   Procedure identifier (see enum #gapm_actv_proc_id)
 * @param[in] status    Procedure execution status  (see enum #hl_err)
 ****************************************************************************************
 */
void gapm_actv_send_proc_cmp(gapm_actv_t *p_actv, uint8_t proc_id, uint16_t status);


/**
 ****************************************************************************************
 * @brief Get own address type to be sent in HCI commands.
 *
 * @param[in] own_addr_type     Address type provided by application
 *
 * @return HCI Own address type value
 *      - 0: Public Device Address
 *      - 1: Random Device Address
 *      - 2: Controller generates the RPA based on the local IRK from the resolving list.
 * If the resolving list contains no matching entry, use the public address
 *      - 3: Controller generates the RPA based on the local IRK from the resolving list.
 * If the resolving list contains no matching entry, use the random address that has
 * been set using HCI LE Set Advertising Set Random Address.
 ****************************************************************************************
 */
uint8_t gapm_le_actv_get_hci_own_addr_type(uint8_t own_addr_type);

/**
 ****************************************************************************************
 * @brief Send GAPM_DEV_BDADDR_IND message to the application. Provide the random address used
 * for one activity.
 *
 * @param[in] p_actv    Pointer to the structure activity for which address has been generated
 ****************************************************************************************
 */
void gapm_le_actv_send_new_bdaddr(gapm_actv_t *p_actv, const gap_addr_t* p_addr);


/**
 ****************************************************************************************
 * @brief Try to start the renewal address timer
 ****************************************************************************************
 */
void gapm_le_actv_addr_renew_timer_start(void);

#if (HL_LE_OBSERVER)

/**
 ****************************************************************************************
 * @brief Change type of Scan or init address type
 *
 * @param[in] own_addr_type New own address type (see enum #gapm_own_addr)
 *
 ****************************************************************************************
 */
void gapm_le_actv_addr_scan_init_type_set(uint8_t own_addr_type);

/**
 ****************************************************************************************
 * @brief Function to be used to check if address type provided for a scanning or an
 * initiating activity is valid or not.
 * As a same address shall be shared between all these activities, all created scanning and
 * initiating activities shall have the same address type.
 *
 * @param[in]  addr_type       Address type to be checked
 *
 * @return true if provided address type is valid, else false
 ****************************************************************************************
 */
bool gapm_le_actv_addr_is_type_valid(uint8_t addr_type);

#if(HOST_MSG_API)
/**
 ****************************************************************************************
 * @brief function used to know if init/scan random address already generated
 *
 * @return true if already generated, false otherwise
 ****************************************************************************************
 */
bool gapm_le_actv_addr_is_scan_init_addr_generated(void);
#endif // (HOST_MSG_API)

/**
 ****************************************************************************************
 * @brief Send HCI LE Set Random Address command to the controller.
 *
 * @param[in] event         Event transition to trigger once command completes
 * @param[in] cmd_evt_func  Pointer to HCI command complete or status event handler
 *
 * @return status of function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapm_le_actv_addr_send_hci_le_set_rand_addr_cmd(uint8_t event, hl_hci_cmd_evt_func_t cmd_evt_func);

/**
 ****************************************************************************************
 * @brief Append data to a buffer. if buffer isn't big enough, a new buffer is allocated
 *
 * @param[in,out] pp_buf    Pointer to Buffer pointer
 * @param[in]     length    Length of data to append to buffer
 * @param[in]     p_data    Pointer to data
 *
 *  -- TODO [NATIVE API] could be a co_buf function
 *
 * @return True if operation succeed, False otherwise
 ****************************************************************************************
 */
bool gapm_scan_append(co_buf_t** pp_buf, uint16_t length, const uint8_t* p_data);
#endif // (HL_LE_OBSERVER)

#if(HL_LE_CENTRAL || HL_LE_PERIPHERAL)

/**
 ****************************************************************************************
 * @brief Create an LE connection with received connection information
 *
 * @param[in]  p_actv            Pointer to activity structure
 * @param[in]  is_name_discovery True if a name discovery procedure handle connection establishment, false otherwise
 * @param[in]  p_evt             HCI event that contains connection information
 * @param[in]  own_addr_type     Activity Own address type
 * @param[in]  p_rpa             Pointer to resolvable random address genrated by host
 *                              (valid if own_addr_type != GAPM_STATIC_ADDR)
 * @param[out] p_condix          Returns allocated connection index
 *
 * @return Status of LE Connection creation (see enum #hl_error)
 ****************************************************************************************
 */
uint16_t gapm_le_actv_con_create(gapm_actv_t *p_actv, bool is_name_discovery, const struct hci_le_enh_con_cmp_evt* p_evt,
                                 uint8_t own_addr_type, const gap_addr_t* p_rpa, uint8_t* p_conidx);


/**
 ****************************************************************************************
 * @brief Initiate disconnection of LE Link (error case) using hci
 *
 * @param[in] conhdl            Connection handle
 * @param[in] reason            Disconnection reason (see enum #co_error)
 ****************************************************************************************
 */
void gapm_le_actv_send_hci_disconnect(uint16_t conhdl, uint8_t reason);
#endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)


#if (BT_HOST_PRESENT)
/**
 ****************************************************************************************
 * @brief Create an BT connection with received connection information
 *
 * @param[in]  p_actv            Pointer to activity structure
 * @param[in]  is_initiator      True if connection initiator, connection responder otherwise
 * @param[in]  p_peer_addr       Pointer to peer device address
 * @param[in]  enc_en            True if link is encrypted at establishment, False otherwise
 * @param[out] p_condix          Returns allocated connection index
 *
 * @return Status of LE Connection creation (see enum #hl_error)
 ****************************************************************************************
 */

uint16_t gapm_bt_actv_con_create(gapm_actv_t* p_actv, bool is_initiator, uint16_t conhdl, const gap_addr_t* p_peer_addr,
                                 bool enc_en, uint8_t* p_conidx);

#endif // (BT_HOST_PRESENT)
/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */

/// @} GAPM

#endif /* _GAPM_ACTV_H_ */

/**
 ****************************************************************************************
 *
 * @file gapc_proc.h
 *
 * @brief GAP Connection Procedure management
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */


#ifndef _GAPC_PROC_H_
#define _GAPC_PROC_H_

/**
 ****************************************************************************************
 * @addtogroup GAPC
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#if (GAPC_PRESENT)
#include "gapc.h"
#include "../../inc/hl_proc.h"
#include "co_bt.h"

/*
 * MACROS
 ****************************************************************************************
 */

/// Create a token for HCI command
#define GAPC_PROC_TOKEN(conidx, event)    ((((conidx) & 0xFF) << 8) | ((event) & 0xFF))
/// Retrieve connection index from token
#define GAPC_PROC_TOKEN_GET_CONIDX(token) ((token) >> 8)
/// Retrieve connection event transition from token
#define GAPC_PROC_TOKEN_GET_EVENT(token)  ((token) & 0xFF)

/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
typedef struct gapc_con gapc_con_t;
typedef struct gapc_proc_simple gapc_proc_simple_t;

/// Callback executed once procedure is granted
typedef uint16_t (*gapc_proc_simple_granted_cb)(uint8_t conidx, gapc_proc_simple_t* p_proc);
/// Callback executed once procedure is finished
typedef void (*gapc_proc_simple_finished_cb)(uint8_t conidx, gapc_proc_simple_t* p_proc, uint16_t status);

/// Simple procedure interface
typedef struct gapc_proc_simple_itf
{
    /// Callback executed once procedure is granted
    gapc_proc_simple_granted_cb  granted;
    /// Callback executed once procedure is finished successfully or due to an error
    gapc_proc_simple_finished_cb finished;
} gapc_proc_simple_itf_t;

/// Simple procedure object
typedef struct gapc_proc_simple
{
    /// procedure inheritance
    hl_proc_t                     hdr;
    /// Simple procedure interface
    const gapc_proc_simple_itf_t* p_itf;
    /// Callback to execute once command completes
    gapc_proc_cmp_cb              cmp_cb;
    /// Dummy parameter provided by upper layer SW
    uint32_t                      dummy;
    /// Connection index
    uint8_t                       conidx;
} gapc_proc_simple_t;


/// Procedure used to retrieve Client information
typedef struct gapc_proc_info
{
    /// Inherited Simple procedure object
    gapc_proc_simple_t hdr;
    /// Information used to retrieve command to send
    uint32_t           hci_cmd_info;
    /// Pointer to result information
    const void*        p_res_info;
} gapc_proc_info_t;

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
 * @param[in]  p_con        Pointer to connection structure
 * @param[in]  proc_size    Size of procedure parameters shall be at least sizeof(hl_proc_t)
 * @param[in]  p_itf        Procedure interface
 * @param[out] pp_proc      Pointer to new procedure
 *
 * @return Status of operation creation (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapc_proc_create(gapc_con_t* p_con, uint16_t proc_size, const hl_proc_itf_t *p_itf,
                          hl_proc_t** pp_proc);

/**
 ****************************************************************************************
 * @brief Get if given procedure type is active.
 *
 * @param[in] conidx     Connection Index
 * @param[in] p_itf      Procedure interface used to discriminate procedure type
 *
 * @return True if current procedure is active, False otherwise
 ****************************************************************************************
 */
bool gapc_proc_is_active(uint8_t conidx, const hl_proc_itf_t *p_itf);

/**
 ****************************************************************************************
 * @brief Ask procedure to perform a transition
 *
 * This function shall be called to ask procedure on top of execution queue to perform a transition
 *
 * @param[in] conidx     Connection Index
 * @param[in] event      Event type receive that induce procedure state transition
 * @param[in] status     Status code associated to the procedure transition state
 ****************************************************************************************
 */
void gapc_proc_transition(uint8_t conidx, uint8_t event, uint16_t status);

/**
 ****************************************************************************************
 * @brief Ask procedure to perform a transition if current procedure is active
 *
 * This function shall be called to ask procedure on top of execution queue to perform a transition
 *
 * @param[in] conidx     Connection Index
 * @param[in] p_itf      Procedure interface used to discriminate procedure type
 * @param[in] event      Event type receive that induce procedure state transition
 * @param[in] status     Status code associated to the procedure transition state
 *
 * @return True if current procedure is active, False otherwise
 ****************************************************************************************
 */
bool gapc_proc_transition_if_active(uint8_t conidx, const hl_proc_itf_t *p_itf, uint8_t event, uint16_t status);

/**
 ****************************************************************************************
 * @brief Get pointer to the procedure on top of execution queue
 *
 * @param[in] conidx     Connection Index
 *
 * @return Procedure on top of execution queue, NULL if no procedure under execution
 ****************************************************************************************
 */
hl_proc_t* gapc_proc_get(uint8_t conidx);


/**
 ****************************************************************************************
 * @brief Initialize procedure queues for reset
 *
 * @param[in] conidx     Connection Index
 ****************************************************************************************
 */
void gapc_proc_reset(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Initialize procedure queues for link disconnection (abort on-going procedures)
 *
 * @param[in] conidx     Connection Index
 * @param[in] reason     Procedures abort reasons
 ****************************************************************************************
 */
void gapc_proc_cleanup(uint8_t conidx, uint16_t reason);

/**
 ****************************************************************************************
 * @brief Default command status event handler to continue procedure execution
 *
 * @param[in] opcode    HCI code:
 * @param[in] token     Token that contains event transition information
 * @param[in] p_evt     Pointer to command complete or status event parameters. (specific to the HCI command)
 *
 ****************************************************************************************
 */
void gapc_proc_default_hci_stat_evt_handler(uint16_t opcode, uint16_t token,  struct hci_cmd_stat_event const *p_evt);


/**
 ****************************************************************************************
 * @brief Default command complete event handler to continue procedure execution
 *
 * @param[in] opcode    HCI code:
 * @param[in] token     Token that contains event transition information
 * @param[in] p_evt     Pointer to command complete or status event parameters. (specific to the HCI command)
 *
 ****************************************************************************************
 */
void gapc_proc_default_hci_cmp_evt_handler(uint16_t opcode, uint16_t token,  struct hci_basic_conhdl_cmd_cmp_evt const *p_evt);

/**
 ****************************************************************************************
 * @brief Command status event handler to ignore result of HCI command
 *
 * @param[in] opcode    HCI code:
 * @param[in] event     Event Transition information
 * @param[in] p_evt     Pointer to command complete or status event parameters. (specific to the HCI command)
 *
 ****************************************************************************************
 */
void gapc_proc_ignore_hci_evt_handler(uint16_t opcode, uint16_t event, const void *p_evt);


/**
 ****************************************************************************************
 * @brief Command used to create a procedure for grant state and finished state
 *
 * @param[in]  conidx     Connection Index
 * @param[in]  dummy      Dummy parameter provided by upper layer SW
 * @param[in]  cmp_cb     Function called once procedure is completed
 * @param[in]  proc_size  Size of procedure parameters
 * @param[in]  p_itf      Pointer to procedure interface
 * @param[out] pp_proc    Pointer to new procedure
 *
 * @return Status of operation creation (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapc_proc_simple_create(uint8_t conidx, uint32_t dummy, gapc_proc_cmp_cb cmp_cb, uint16_t proc_size,
                                 const gapc_proc_simple_itf_t* p_itf, gapc_proc_simple_t** pp_proc);

/**
 ****************************************************************************************
 * @brief Ask procedure to perform simple procedure transition if current procedure is active
 *
 * This function shall be called to ask procedure on top of execution queue to perform a transition
 *
 * @param[in] conidx     Connection Index
 * @param[in] p_itf      interface use to discriminate the active procedure type
 * @param[in] event      Event type receive that induce procedure state transition
 * @param[in] status     Status code associated to the procedure transition state (@see hl_err)
 ****************************************************************************************
 */
bool gapc_proc_simple_transition_if_active(uint8_t conidx, const gapc_proc_simple_itf_t* p_itf, uint8_t event,
                                           uint16_t status);

/**
 ****************************************************************************************
 * @brief Default finish procedure callback
 *
 * @param[in] conidx     Connection Index
 * @param[in] p_proc     Pointer to default procedure
 * @param[in] status     Status code associated to the procedure transition state (@see hl_err)
 ****************************************************************************************
 */
void gapc_proc_simple_default_finished_cb(uint8_t conidx, gapc_proc_simple_t* p_proc, uint16_t status);

#if(BLE_GAPC)
/**
 ****************************************************************************************
 * @brief Command used to create an LE procedure
 *
 * @param[in]  conidx     Connection Index
 * @param[in]  proc_size  Size of procedure parameters
 * @param[in]  p_itf      Procedure interface
 * @param[out] pp_proc    Pointer to new procedure
 *
 * @return Status of operation creation (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapc_proc_le_create(uint8_t conidx, uint16_t proc_size, const hl_proc_itf_t *p_itf, hl_proc_t** pp_proc);
#endif // (BLE_GAPC)
#endif // (GAPC_PRESENT)


/**
 ****************************************************************************************
 * @brief Default Information HCI command complete event handler (with connection handle)
 *
 * @param[in] opcode        HCI command operation code
 * @param[in] event         Procedure event transition
 * @param[in] p_evt         Pointer to HCI event received
 ****************************************************************************************
 */
void gapc_proc_info_default_hci_cmp_evt_handler(uint16_t opcode, uint16_t event,
                                                const struct hci_basic_conhdl_cmd_cmp_evt *p_evt);

/**
 ****************************************************************************************
 * @brief Default Information HCI LE command complete event handler (with connection handle)
 *
 * @param[in] event         Procedure event transition
 * @param[in] p_evt         Pointer to HCI event received
 ****************************************************************************************
 */
void gapc_proc_info_default_hci_le_cmp_evt_handler(uint16_t event, const struct hci_basic_conhdl_le_cmd_cmp_evt *p_evt);

/**
 ****************************************************************************************
 * @brief Get information about connection or peer device
 *
 * @param[in] conidx       Connection index
 * @param[in] dummy        Upper layer software dummy parameter returned each time an event occurs on connection
 * @param[in] hci_cmd_info Optional command information
 * @param[in] p_itf        Pointer to simple procedure interface
 * @param[in] cmp_cb       Function called when procedure is over.
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapc_proc_info_create(uint8_t conidx, uint32_t dummy, uint32_t hci_cmd_info,
                               const gapc_proc_simple_itf_t* p_itf, gapc_proc_cmp_cb cmp_cb);
/// @} GAPC

#endif /* _GAPC_PROC_H_ */

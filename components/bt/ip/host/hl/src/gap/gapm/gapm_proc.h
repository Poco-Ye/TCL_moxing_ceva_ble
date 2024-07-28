/**
 ****************************************************************************************
 *
 * @file gapm_proc.h

 * @brief GAP Manager procedure API
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */


#ifndef GAPM_PROC_H_
#define GAPM_PROC_H_

/**
 ****************************************************************************************
 * @addtogroup GAPM
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#include "../../inc/hl_proc.h" // Host Proc API
#include "hl_hci.h"      // Host HCI interface
#include <stdint.h>

#include "co_list.h"

/*
 * MACROS
 ****************************************************************************************
 */

#define GAPM_INFO_PROC_START(dummy, opcode, cmd_evt_cb,  res_cb)\
        gapm_proc_info_start((dummy), (opcode), (hl_hci_cmd_evt_func_t) (cmd_evt_cb), (gapm_proc_info_res_cb) (res_cb))


/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// default callback handler to provide information result
typedef void (*gapm_proc_info_res_cb)(uint32_t dummy, uint16_t status, const void* p_info);

/// Structure of get controller information procedure
typedef struct gapm_proc_info
{
    /// Procedure inheritance
    hl_proc_t             hdr;
    ///  Function that handle HCI command complete event
    hl_hci_cmd_evt_func_t hci_cmd_evt_cb;
    /// Dummy parameter provided by upper layer software
    uint32_t              dummy;
    /// Callback to execute at end of procedure execution
    gapm_proc_info_res_cb res_cb;
    /// HCI Operation command to execute once procedure is granted
    uint16_t              hci_cmd_opcode;
} gapm_proc_info_t;

/// Structure of set controller configuration
typedef struct gapm_proc_set_cfg
{
    /// Procedure inheritance
    hl_proc_t           hdr;
    /// Callback to execute at end of procedure execution
    gapm_proc_cmp_cb    cmp_cb;
    /// Dummy parameter provided by upper layer software
    uint32_t            dummy;
} gapm_proc_set_cfg_t;

typedef struct gapm_proc_send_hci_cmd gapm_proc_send_hci_cmd_t;

/// Callback executed prepare data to send
typedef void (*gapm_proc_prepare_hci_cmd_cb)(gapm_proc_send_hci_cmd_t* p_proc, void* p_cmd);

/// Structure that provide HCI command information
typedef struct gapm_proc_hci_cmd_info
{
    /// Function used to prepare HCI data
    gapm_proc_prepare_hci_cmd_cb  prep_cb;
    /// Command operation code
    uint16_t                      cmd_op_code;
    /// Command length
    uint16_t                      cmd_length;
} gapm_proc_hci_cmd_info_t;

/// Structure of set controller configuration
typedef struct gapm_proc_send_hci_cmd
{
    /// Procedure inheritance
    gapm_proc_set_cfg_t             hdr;
    /// Pointer to HCI Command send info
    const gapm_proc_hci_cmd_info_t* p_info;
} gapm_proc_send_hci_cmd_t;

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
/// Default command complete handler for procedure transition
void gapm_default_cfg_hci_cmd_cmp_evt_handler(uint16_t opcode, uint16_t event, struct hci_basic_cmd_cmp_evt const *p_evt);



/**
 ****************************************************************************************
 * @brief Create a new procedure
 *
 * @param[in]  proc_type    Indentifier of the queue where procedure will be pushed
 * @param[in]  proc_size    Size of procedure parameters shall be at least sizeof(hl_proc_t)
 * @param[in]  p_itf        Procedure interface
 * @param[out] pp_proc      Pointer to new procedure
 *
 * @return Status of operation creation (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapm_proc_create(uint8_t proc_type, uint16_t proc_size, const hl_proc_itf_t *p_itf, hl_proc_t** pp_proc);

/**
 ****************************************************************************************
 * @brief Ask procedure to perform a transition
 *
 * This function shall be called to ask procedure on top of execution queue to perform a transition
 *
 * @param[in] proc_type  Procedure type (see enum #gapm_proc_type)
 * @param[in] event      Event type receive that induce procedure state transition
 * @param[in] status     Status code associated to the procedure transition state
 ****************************************************************************************
 */
void gapm_proc_transition(uint8_t proc_type,  uint8_t event, uint16_t status);

/**
 ****************************************************************************************
 * @brief Get pointer to the procedure on top of execution queue
 *
 * @param[in] proc_type  Procedure type (see enum #gapm_proc_type)
 *
 * @return Procedure on top of execution queue, NULL if no procedure under execution
 ****************************************************************************************
 */
hl_proc_t* gapm_proc_get(uint8_t proc_type);

/**
 ****************************************************************************************
 * @brief Stop procedure without doing a transition state
 *
 * @param[in] proc_type  Procedure type (see enum #gapm_proc_type)
 ****************************************************************************************
 */
void gapm_proc_stop(uint8_t proc_type);

/**
 ***************************************************************************************
 * @brief Start new simple procedure to retrieve controller information using a simple HCI command
 *
 * @param[in] dummy           Dummy parameter provided by upper layer application
 * @param[in] hci_cmd_opcode  HCI Operation command to execute once procedure is granted
 * @param[in] cmd_evt_cb      Function that handle HCI command complete event
 * @param[in] res_cb          Function called when procedure is over
 *
 * @return Execution status (see enum #hl_err).
 ***************************************************************************************
 */
uint16_t gapm_proc_info_start(uint32_t dummy, uint16_t hci_cmd_opcode, hl_hci_cmd_evt_func_t cmd_evt_cb,
                              gapm_proc_info_res_cb res_cb);

/**
 ***************************************************************************************
 * @brief Stop on-going Get information procedure and return result callback

 * @param[out] p_dummy           Pointer to dummy parameter provided by upper layer application
 * @param[out] p_res_cb          Pointer to function called when procedure is over
 *
 * @return Execution status (see enum #hl_err).
 ***************************************************************************************
 */
uint16_t gapm_proc_info_stop(uint32_t* p_dummy, gapm_proc_info_res_cb* p_res_cb);

/**
 ****************************************************************************************
 * @brief Create a new procedure for controller configuration
 *
 * @param[in]  proc_size    Size of procedure parameters shall be at least sizeof(hl_proc_t)
 * @param[in]  p_itf        Procedure interface
 * @param[in]  dummy        Dummy parameter provided by upper layer application
 * @param[in]  cmp_cb       Function called when procedure is over
 * @param[out] pp_proc      Pointer to new procedure
 *
 * @return Status of operation creation (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapm_proc_set_cfg_create(uint16_t proc_size, const hl_proc_itf_t *p_itf, uint32_t dummy,
                                  gapm_proc_cmp_cb cmp_cb, gapm_proc_set_cfg_t** pp_proc);


/**
 ****************************************************************************************
 * @brief Create a new procedure to send a hci command (with basic command response)
 *
 * @param[in]  proc_size    Size of procedure parameters shall be at least sizeof(hl_proc_t)
 * @param[in]  p_info       Pointer to HCI command information
 * @param[in]  dummy        Dummy parameter provided by upper layer application
 * @param[in]  cmp_cb       Function called when procedure is over
 * @param[out] pp_proc      Pointer to new procedure
 *
 * @return Status of operation creation (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapm_proc_send_hci_cmd_create(uint16_t proc_size, const gapm_proc_hci_cmd_info_t* p_info, uint32_t dummy,
                                       gapm_proc_cmp_cb cmp_cb, gapm_proc_send_hci_cmd_t** pp_proc);


/**
 ***************************************************************************************
 * @brief Inform upper layer software that procedure execution is completed

 * @param[out] p_proc            Pointer to the configuration
 * @param[out] status            Status of procedure execution
 ***************************************************************************************
 */
void gapm_proc_execute_cmp_cb(gapm_proc_set_cfg_t* p_proc, uint16_t status);


/**
 ****************************************************************************************
 * @brief Initialize Procedure module
 *
 * @param[in] init_type  Type of initialization (see enum #rwip_init_type)
 ****************************************************************************************
 */
void gapm_proc_initialize(uint8_t init_type);


/**
 ****************************************************************************************
 * @brief Empty Function in case no function is useful to prepare HCI data
 * @param[in] p_proc_dummy  Dummy controller configuration pointer
 * @param[in] p_cmd_dummy   Dummy Command pointer
 *
 ****************************************************************************************
 */
void gapm_proc_empty(void* p_proc_dummy, void* p_cmd_dummy);


/// @} GAPM

#endif /* GAPM_PROC_H_ */

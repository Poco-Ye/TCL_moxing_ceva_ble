/**
 ****************************************************************************************
 *
 * @file envs.h
 *
 * @brief Header file - Environmental Sensing Service Profile - Native API.
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */

#ifndef _ENVS_H_
#define _ENVS_H_

/**
 ****************************************************************************************
 * @addtogroup ENVS
 * @ingroup Profile
 * @brief Environmental Sensing Service Profile - Native API.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "envs_msg.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Buffer header size that must be present in buffer provided by application
#define ENVS_BUFFER_HEADER_LEN   (GATT_BUFFER_HEADER_LEN)

/// Buffer tail size that must be present in buffer provided by application
#define ENVS_BUFFER_TAIL_LEN     (GATT_BUFFER_TAIL_LEN)


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/*
 * NATIVE API CALLBACKS
 ****************************************************************************************
 */

/// Environment sensor server callback set
typedef struct envs_cb
{
    /**
     ****************************************************************************************
     * @brief Completion of indicate procedure
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (see enum #hl_err)
     ****************************************************************************************
     */
    void (*cb_indicate_cmp)(uint8_t conidx, uint16_t status);

    /**
     ****************************************************************************************
     * @brief Completion of notify procedure
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (see enum #hl_err)
     ****************************************************************************************
     */
    void (*cb_notify_cmp)(uint8_t conidx, uint16_t status);

    /**
     ****************************************************************************************
     * @brief Function called when peer device wants to modify CCC configuration
     *
     * Write request must be confirmed by application using #envs_write_ccc_cfm
     *
     * @param[in] conidx        Connection index
     * @param[in] token         Token information that must be returned in confirmation
     * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
     * @param[in] es_char_inst  Characteristic Instance
     * @param[in] ccc           Actual data for the CCC descriptor
     ****************************************************************************************
     */
    void (*cb_write_ccc_req)(uint8_t conidx, uint32_t token, uint8_t es_char_id, uint8_t es_char_inst,
                             uint16_t ccc);

    /**
     ****************************************************************************************
     * @brief Function called when peer device wants to update trigger
     *
     * Write request must be confirmed by application using #envs_write_trigger_cfm
     *
     * @param[in] conidx        Connection index
     * @param[in] token         Token information that must be returned in confirmation
     * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
     * @param[in] es_char_inst  Characteristic Instance
     * @param[in] trigger_idx   Descriptor idx @ref envs_desc_idx (TRIG1 TRIG2 TRIG3) 0,1,2
     * @param[in] p_trigger     Pointer to trigger data
     ****************************************************************************************
     */
    void (*cb_write_trigger_req)(uint8_t conidx, uint32_t token, uint8_t es_char_id, uint8_t es_char_inst,
                                 uint8_t trigger_idx, const envp_trigger_t* p_trigger);


    /**
     ****************************************************************************************
     * @brief Function called when peer device wants to update configuration
     *
     * Write request must be confirmed by application using #envs_write_config_cfm
     *
     * @param[in] conidx        Connection index
     * @param[in] token         Token information that must be returned in confirmation
     * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
     * @param[in] es_char_inst  Characteristic Instance
     * @param[in] trigger_logic Actual data for the CFG descriptor @ref enum envp_es_trig_trigger_logic
     ****************************************************************************************
     */
    void (*cb_write_config_req)(uint8_t conidx, uint32_t token, uint8_t es_char_id, uint8_t es_char_inst,
                                uint8_t trigger_logic);

    /**
     ****************************************************************************************
     * @brief Function called when peer device wants to update user description
     *
     * Write request must be confirmed by application using #envs_write_user_desc_cfm
     *
     * @param[in] conidx        Connection index
     * @param[in] token         Token information that must be returned in confirmation
     * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
     * @param[in] es_char_inst  Characteristic Instance
     * @param[in] p_user_desc   Pointer to buffer that contains UTF-8 user description.
     ****************************************************************************************
     */
    void (*cb_write_user_desc_req)(uint8_t conidx, uint32_t token, uint8_t es_char_id, uint8_t es_char_inst,
                                   co_buf_t* p_user_desc);

    /**
     ****************************************************************************************
     * @brief Function called when peer device wants to read value
     *
     * Read information must be returned by application using #envs_read_value_cfm
     *
     * @param[in] conidx        Connection index
     * @param[in] token         Token information that must be returned in confirmation
     * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
     * @param[in] es_char_inst  Characteristic Instance
     ****************************************************************************************
     */
    void (*cb_read_value_req)(uint8_t conidx, uint32_t token, uint8_t es_char_id, uint8_t es_char_inst);

    /**
     ****************************************************************************************
     * @brief Function called when peer device wants to read CCC configuration
     *
     * Read information must be returned by application using #envs_read_ccc_cfm
     *
     * @param[in] conidx        Connection index
     * @param[in] token         Token information that must be returned in confirmation
     * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
     * @param[in] es_char_inst  Characteristic Instance
     ****************************************************************************************
     */
    void (*cb_read_ccc_req)(uint8_t conidx, uint32_t token, uint8_t es_char_id, uint8_t es_char_inst);


    /**
     ****************************************************************************************
     * @brief Function called when peer device wants to read trigger
     *
     * Read information must be returned by application using #envs_read_trigger_cfm
     *
     * @param[in] conidx        Connection index
     * @param[in] token         Token information that must be returned in confirmation
     * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
     * @param[in] es_char_inst  Characteristic Instance
     * @param[in] trigger_idx   Descriptor idx @ref envs_desc_idx (TRIG1 TRIG2 TRIG3) 0,1,2
     ****************************************************************************************
     */
    void (*cb_read_trigger_req)(uint8_t conidx, uint32_t token, uint8_t es_char_id, uint8_t es_char_inst,
                                uint8_t trigger_idx);

    /**
     ****************************************************************************************
     * @brief Function called when peer device wants to read configuration
     *
     * Read information must be returned by application using #envs_read_config_cfm
     *
     * @param[in] conidx        Connection index
     * @param[in] token         Token information that must be returned in confirmation
     * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
     * @param[in] es_char_inst  Characteristic Instance
     ****************************************************************************************
     */
    void (*cb_read_config_req)(uint8_t conidx, uint32_t token, uint8_t es_char_id, uint8_t es_char_inst);

    /**
     ****************************************************************************************
     * @brief Function called when peer device wants to read user description
     *
     * Read information must be returned by application using #envs_read_user_desc_cfm
     *
     * @param[in] conidx        Connection index
     * @param[in] token         Token information that must be returned in confirmation
     * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
     * @param[in] es_char_inst  Characteristic Instance
     * @param[in] offset        Data offset
     * @param[in] max_len       Maximum string length to return
     ****************************************************************************************
     */
    void (*cb_read_user_desc_req)(uint8_t conidx, uint32_t token, uint8_t es_char_id, uint8_t es_char_inst,
                                  uint16_t offset, uint16_t max_len);
    /**
     ****************************************************************************************
     * @brief Function called when peer device wants to read measurement information
     *
     * Read information must be returned by application using #envs_read_meas_cfm
     *
     * @param[in] conidx        Connection index
     * @param[in] token         Token information that must be returned in confirmation
     * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
     * @param[in] es_char_inst  Characteristic Instance
     ****************************************************************************************
     */
    void (*cb_read_meas_req)(uint8_t conidx, uint32_t token, uint8_t es_char_id, uint8_t es_char_inst);
} envs_cb_t;

/*
 * NATIVE API FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Update range value of a characteristic
 *
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 * @param[in] p_range       Pointer to actual data for the RANGE descriptor
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envs_range_upd(uint8_t es_char_id, uint8_t es_char_inst, const union envp_range* p_range);


/**
 ****************************************************************************************
 * @brief Inform peer device about database change increment value
 *
 * Wait for #cb_indicate_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] ind_flag      Describe the Condition of Descriptor change Indication
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envs_indicate(uint8_t conidx, uint8_t es_char_id, uint16_t ind_flag);

/**
 ****************************************************************************************
 * @brief Inform peer device about database change increment value
 *
 * Wait for #cb_notify_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 * @param[in] p_value       Pointer to actual data for the VAL descriptor
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envs_notify(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst, const union envp_val_char* p_value);

/**
 ****************************************************************************************
 * @brief Send back status of CCC configuration update
 *
 * @param[in] conidx        Connection index
 * @param[in] token         Token information received in request
 * @param[in] status        Status of the request at application level (see enum #hl_err)
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envs_write_ccc_cfm(uint8_t conidx, uint32_t token, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst);

/**
 ****************************************************************************************
 * @brief Send back status of trigger update
 *
 * @param[in] conidx        Connection index
 * @param[in] token         Token information received in request
 * @param[in] status        Status of the request at application level (see enum #hl_err)
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envs_write_trigger_cfm(uint8_t conidx, uint32_t token, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst);

/**
 ****************************************************************************************
 * @brief Send back status of configuration update
 *
 * @param[in] conidx        Connection index
 * @param[in] token         Token information received in request
 * @param[in] status        Status of the request at application level (see enum #hl_err)
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envs_write_config_cfm(uint8_t conidx, uint32_t token, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst);

/**
 ****************************************************************************************
 * @brief Send back status of User description update
 *
 * @param[in] conidx        Connection index
 * @param[in] token         Token information received in request
 * @param[in] status        Status of the request at application level (see enum #hl_err)
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envs_write_user_desc_cfm(uint8_t conidx, uint32_t token, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst);


/**
 ****************************************************************************************
 * @brief Send back value data
 *
 * @param[in] conidx        Connection index
 * @param[in] token         Token information received in request
 * @param[in] status        Status of the request at application level (see enum #hl_err)
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 * @param[in] p_value       Pointer to actual Characteristic Value
 *
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envs_read_value_cfm(uint8_t conidx, uint32_t token, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                             const union envp_val_char* p_value);

/**
 ****************************************************************************************
 * @brief Send back CCC configuration
 *
 * @param[in] conidx        Connection index
 * @param[in] token         Token information received in request
 * @param[in] status        Status of the request at application level (see enum #hl_err)
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 * @param[in] ccc           Actual data for the CCC descriptor
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envs_read_ccc_cfm(uint8_t conidx, uint32_t token, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                           uint16_t ccc);

/**
 ****************************************************************************************
 * @brief Send back trigger data
 *
 * @param[in] conidx        Connection index
 * @param[in] token         Token information received in request
 * @param[in] status        Status of the request at application level (see enum #hl_err)
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 * @param[in] trigger_idx   Descriptor idx @ref envs_desc_idx (TRIG1 TRIG2 TRIG3) 0,1,2
 * @param[in] p_trigger     Pointer to trigger data
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envs_read_trigger_cfm(uint8_t conidx, uint32_t token, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                               uint8_t trigger_idx, const envp_trigger_t* p_trigger);


/**
 ****************************************************************************************
 * @brief Send back configuration value
 *
 * @param[in] conidx        Connection index
 * @param[in] token         Token information received in request
 * @param[in] status        Status of the request at application level (see enum #hl_err)
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 * @param[in] trigger_logic Actual data for the CFG descriptor @ref enum envp_es_trig_trigger_logic
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envs_read_config_cfm(uint8_t conidx, uint32_t token, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                              uint8_t trigger_logic);


/**
 ****************************************************************************************
 * @brief Send back user description value
 *
 * @param[in] conidx        Connection index
 * @param[in] token         Token information received in request
 * @param[in] status        Status of the request at application level (see enum #hl_err)
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 * @param[in] total_len     Total size of UTF-8 string (without considering offset)
 * @param[in] p_user_desc   Pointer to buffer that contains UTF-8 user description starting from offset.
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envs_read_user_desc_cfm(uint8_t conidx, uint32_t token, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                                 uint16_t total_len, co_buf_t* p_user_desc);

/**
 ****************************************************************************************
 * @brief Send back measurement descriptor
 *
 * @param[in] conidx        Connection index
 * @param[in] token         Token information received in request
 * @param[in] status        Status of the request at application level (see enum #hl_err)
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 * @param[in] p_meas        Pointer to actual data for the MEAS descriptor
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envs_read_meas_cfm(uint8_t conidx, uint32_t token, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                            const envp_es_meas_desc_t* p_meas);

/// @} ENVS

#endif //(_ENVS_H_)


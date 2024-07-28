/**
 ****************************************************************************************
 *
 * @file envc.h
 *
 * @brief Header file - The Environmental Sensing Collector/Client Role - Native API.
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */

#ifndef _ENVC_H_
#define _ENVC_H_

/**
 ****************************************************************************************
 * @addtogroup ENVC
 * @ingroup Profile
 * @brief  The Environmental Sensing Profile Collector  - Native API.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "envc_msg.h"

/*
 * DEFINES
 ****************************************************************************************
 */
/// Buffer header size that must be present in buffer provided by application
#define ENVC_BUFFER_HEADER_LEN   (GATT_BUFFER_HEADER_LEN)

/// Buffer tail size that must be present in buffer provided by application
#define ENVC_BUFFER_TAIL_LEN     (GATT_BUFFER_TAIL_LEN)


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * NATIVE API CALLBACKS
 ****************************************************************************************
 */

/// Environment sensor client callback set
typedef struct envc_cb
{
    /**
     ****************************************************************************************
     * @brief Completion of enable procedure
     *
     * @param[in] conidx                Connection index
     * @param[in] status                Status of the procedure execution (see enum #hl_err)
     * @param[in] p_es                  Pointer to peer database description bond data
     ****************************************************************************************
     */
    void (*cb_enable_cmp)(uint8_t conidx, uint16_t status, const envc_es_content_t* p_es);

    /**
     ****************************************************************************************
     * @brief Completion of read value procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the request at application level (see enum #hl_err)
     * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
     * @param[in] es_char_inst  Characteristic Instance
     * @param[in] p_value       Pointer to actual Characteristic Value
     *
     ****************************************************************************************
     */
    void (*cb_read_value_cmp)(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                              const union envp_val_char* p_value);


    /**
     ****************************************************************************************
     * @brief Completion of read CCC configuration procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the request at application level (see enum #hl_err)
     * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
     * @param[in] es_char_inst  Characteristic Instance
     * @param[in] ccc           Actual data for the CCC descriptor
     *
     ****************************************************************************************
     */
    void (*cb_read_ccc_cmp)(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst, uint16_t ccc);

    /**
     ****************************************************************************************
     * @brief Completion of read trigger procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the request at application level (see enum #hl_err)
     * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
     * @param[in] es_char_inst  Characteristic Instance
     * @param[in] trigger_idx   Descriptor idx @ref envs_desc_idx (TRIG1 TRIG2 TRIG3) 0,1,2
     * @param[in] p_trigger     Pointer to trigger data
     *
     ****************************************************************************************
     */
    void (*cb_read_trigger_cmp)(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                                uint8_t trigger_idx, const envp_trigger_t* p_trigger);

    /**
     ****************************************************************************************
     * @brief Completion of read configuration procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the request at application level (see enum #hl_err)
     * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
     * @param[in] es_char_inst  Characteristic Instance
     * @param[in] trigger_logic Actual data for the CFG descriptor @ref enum envp_es_trig_trigger_logic
     *
     ****************************************************************************************
     */
    void (*cb_read_config_cmp)(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                               uint8_t trigger_logic);

    /**
     ****************************************************************************************
     * @brief Completion of read user description procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the request at application level (see enum #hl_err)
     * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
     * @param[in] es_char_inst  Characteristic Instance
     * @param[in] p_user_desc   Pointer to buffer that contains UTF-8 user description starting from offset.
     *
     ****************************************************************************************
     */
    void (*cb_read_user_desc_cmp)(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                                  co_buf_t* p_user_desc);

    /**
     ****************************************************************************************
     * @brief Completion of read extended properties procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the request at application level (see enum #hl_err)
     * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
     * @param[in] es_char_inst  Characteristic Instance
     * @param[in] ext_prop      Extended Properties value
     *
     ****************************************************************************************
     */
    void (*cb_read_ext_prop_cmp)(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                                 uint16_t ext_prop);

    /**
     ****************************************************************************************
     * @brief Completion of read measurement descriptor procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the request at application level (see enum #hl_err)
     * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
     * @param[in] es_char_inst  Characteristic Instance
     * @param[in] p_meas        Pointer to measurement information
     *
     ****************************************************************************************
     */
    void (*cb_read_meas_desc_cmp)(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                                  const envp_es_meas_desc_t* p_meas);

    /**
     ****************************************************************************************
     * @brief Completion of read range procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the request at application level (see enum #hl_err)
     * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
     * @param[in] es_char_inst  Characteristic Instance
     * @param[in] p_range       Pointer to range value
     *
     ****************************************************************************************
     */
    void (*cb_read_value_range_cmp)(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                                    const union envp_range* p_range);

    /**
     ****************************************************************************************
     * @brief Completion of write CCC configuration procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (see enum #hl_err)
     * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
     * @param[in] es_char_inst  Characteristic Instance
     *
     ****************************************************************************************
     */
    void (*cb_write_ccc_cmp)(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst);

    /**
     ****************************************************************************************
     * @brief Completion of write trigger procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (see enum #hl_err)
     * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
     * @param[in] es_char_inst  Characteristic Instance
     * @param[in] trigger_idx   Descriptor idx @ref envs_desc_idx (TRIG1 TRIG2 TRIG3) 0,1,2
     *
     ****************************************************************************************
     */
    void (*cb_write_trigger_cmp)(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                                 uint8_t trigger_idx);

    /**
     ****************************************************************************************
     * @brief Completion of write configuration procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (see enum #hl_err)
     * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
     * @param[in] es_char_inst  Characteristic Instance
     *
     ****************************************************************************************
     */
    void (*cb_write_config_cmp)(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst);
    /**
     ****************************************************************************************
     * @brief Completion of write user description procedure.
     *
     * @param[in] conidx        Connection index
     * @param[in] status        Status of the procedure execution (see enum #hl_err)
     * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
     * @param[in] es_char_inst  Characteristic Instance
     *
     ****************************************************************************************
     */
    void (*cb_write_user_desc_cmp)(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst);

    /**
     ****************************************************************************************
     * @brief Function called when one of the descriptors in the server has changed
     *
     * @param[in] conidx         Connection index
     * @param[in] es_char_id     Characteristic Type idx see enum #envp_es_char
     ****************************************************************************************
     */
    void (*cb_desc_chg)(uint8_t conidx, uint8_t es_char_id, uint16_t flags);

    /**
     ****************************************************************************************
     * @brief Function called when value update is received from server
     *
     * @param[in] conidx         Connection index
     * @param[in] es_char_id     Characteristic Type idx see enum #envp_es_char
     * @param[in] es_char_inst   Characteristic Instance
     * @param[in] p_value        Pointer to environment sensor value
     ****************************************************************************************
     */
    void (*cb_value)(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst, const union envp_val_char* p_value);
} envc_cb_t;

/*
 * NATIVE API FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Restore bond data of a known peer device (at connection establishment)
 *
 * Wait for #cb_enable_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] con_type      Connection type
 * @param[in] p_es          Pointer to peer database description bond data
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envc_enable(uint8_t conidx, uint8_t con_type, const envc_es_content_t* p_es);

/**
 ****************************************************************************************
 * @brief Perform a read value procedure.
 *
 * Wait for #cb_read_value_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envc_read_value(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst);

/**
 ****************************************************************************************
 * @brief Perform a read CCC configuration procedure.
 *
 * Wait for #cb_read_ccc_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envc_read_ccc(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst);

/**
 ****************************************************************************************
 * @brief Perform a read trigger data procedure.
 *
 * Wait for #cb_read_trigger_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 * @param[in] trigger_idx   Descriptor idx @ref envs_desc_idx (TRIG1 TRIG2 TRIG3) 0,1,2
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envc_read_trigger(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst, uint8_t trigger_idx);

/**
 ****************************************************************************************
 * @brief Perform a read configuration procedure.
 *
 * Wait for #cb_read_config_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envc_read_config(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst);

/**
 ****************************************************************************************
 * @brief Perform a read user description procedure.
 *
 * Wait for #cb_read_user_desc_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envc_read_user_desc(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst);

/**
 ****************************************************************************************
 * @brief Perform a read extended properties procedure.
 *
 * Wait for #cb_read_ext_prop_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envc_read_ext_prop(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst);

/**
 ****************************************************************************************
 * @brief Perform a read measurement descriptor procedure.
 *
 * Wait for #cb_read_meas_desc_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envc_read_meas_desc(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst);

/**
 ****************************************************************************************
 * @brief Perform a read range procedure.
 *
 * Wait for #cb_read_value_range_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envc_read_value_range(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst);

/**
 ****************************************************************************************
 * @brief Perform a CCC configuration update procedure.
 *
 * Wait for #cb_write_ccc_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 * @param[in] ccc           Actual data for the CCC descriptor
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envc_write_ccc(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst, uint16_t ccc);

/**
 ****************************************************************************************
 * @brief Perform a trigger data update procedure.
 *
 * Wait for #cb_write_trigger_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 * @param[in] trigger_idx   Descriptor idx @ref envs_desc_idx (TRIG1 TRIG2 TRIG3) 0,1,2
 * @param[in] p_trigger     Pointer to trigger data
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envc_write_trigger(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst,
                            uint8_t trigger_idx, const envp_trigger_t* p_trigger);


/**
 ****************************************************************************************
 * @brief Perform a configuration update procedure.
 *
 * Wait for #cb_write_config_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 * @param[in] trigger_logic Actual data for the CFG descriptor @ref enum envp_es_trig_trigger_logic
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envc_write_config(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst, uint8_t trigger_logic);

/**
 ****************************************************************************************
 * @brief Perform a user description update procedure.
 *
 * Wait for #cb_write_user_desc_cmp execution before starting a new procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 * @param[in] p_user_desc   Pointer to buffer that contains UTF-8 user description.
 *
 * @return Status of the function execution (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t envc_write_user_desc(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst, co_buf_t* p_user_desc);


/// @} ENVC

#endif //(_ENVC_H_)

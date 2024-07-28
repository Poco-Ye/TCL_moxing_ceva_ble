/**
 ****************************************************************************************
 *
 * @file envs_msg.h
 *
 * @brief Header file - Environmental Sensing Service Profile - Message API.
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */

#ifndef _ENVS_MSG_H_
#define _ENVS_MSG_H_

/**
 ****************************************************************************************
 * @addtogroup ENVS
 * @ingroup Profile
 * @brief Environmental Sensing Service Profile - Message API.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#include "envp_common.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Descriptor enabled mask
#define ENVS_DESC_EN_MASK ((ENVS_DESC_EN_RANGE<<1)-1)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Messages for Environmental Sensing Service Profile Sensor
/*@TRACE*/
enum envs_msg_id
{
    /// App update valid value range
    ENVS_UPD_RANGE_CMD                = MSG_ID(ENVS, 0x00),
    /// APP request to Send IN
    ENVS_INDICATE_CMD                 = MSG_ID(ENVS, 0x01),
    /// APP request to Send NTF
    ENVS_NOTIFY_CMD                   = MSG_ID(ENVS, 0x02),

    /// Send CCC descriptor write update to the APP
    ENVS_WR_CCC_REQ_IND               = MSG_ID(ENVS, 0x03),
    /// Send TRIGGER descriptor write update to the APP
    ENVS_WR_TRIGGER_REQ_IND           = MSG_ID(ENVS, 0x04),
    /// Send TRIG CONFIG descriptor write update to the APP
    ENVS_WR_CONFIG_REQ_IND            = MSG_ID(ENVS, 0x05),
    /// Send USER_DESCRIPTION descriptor write update to the APP
    ENVS_WR_USER_DESCRIPTION_REQ_IND  = MSG_ID(ENVS, 0x06),

    /// write confirmation of CCC descriptor from the APP
    ENVS_WR_CCC_CFM                   = MSG_ID(ENVS, 0x07),
    /// write confirmation of TRIGGER descriptor from the APP
    ENVS_WR_TRIGGER_CFM               = MSG_ID(ENVS, 0x08),
    /// write confirmation of TRIG CONFIG descriptor from the APP
    ENVS_WR_CONFIG_CFM                = MSG_ID(ENVS, 0x09),
    /// write confirmation of USER_DESCRIPTION descriptor from the APP
    ENVS_WR_USER_DESCRIPTION_CFM      = MSG_ID(ENVS, 0x0A),

    /// read request characteristic value from the APP
    ENVS_RD_VALUE_REQ_IND             = MSG_ID(ENVS, 0x0B),
    /// read request of CCC descriptor from the APP
    ENVS_RD_CCC_REQ_IND               = MSG_ID(ENVS, 0x0C),
    /// read request of TRIGGER descriptor from the APP
    ENVS_RD_TRIGGER_REQ_IND           = MSG_ID(ENVS, 0x0D),
    /// read request of TRIG CONFIG descriptor from the APP
    ENVS_RD_CONFIG_REQ_IND            = MSG_ID(ENVS, 0x0E),
    /// read request of USER_DESCRIPTION descriptor from the APP
    ENVS_RD_USER_DESCRIPTION_REQ_IND  = MSG_ID(ENVS, 0x0F),
    /// read request of MEAS descriptor
    ENVS_RD_MEAS_REQ_IND              = MSG_ID(ENVS, 0x10),

    /// read response of CCC descriptor from the APP
    ENVS_RD_CCC_CFM                   = MSG_ID(ENVS, 0x11),
    /// read response of TRIGGER descriptor from the APP
    ENVS_RD_TRIGGER_CFM               = MSG_ID(ENVS, 0x12),
    /// read response of TRIG CONFIG descriptor from the APP
    ENVS_RD_CONFIG_CFM                = MSG_ID(ENVS, 0x13),
    /// read response of USER_DESCRIPTION descriptor from the APP
    ENVS_RD_USER_DESCRIPTION_CFM      = MSG_ID(ENVS, 0x14),
    /// read response of characteristic value from the APP
    ENVS_RD_VALUE_CFM                 = MSG_ID(ENVS, 0x15),
    /// read response of MEAS descriptor
    ENVS_RD_MEAS_CFM                  = MSG_ID(ENVS, 0x16),

    /// Send a complete event status to the application
    ENVS_CMP_EVT                      = MSG_ID(ENVS, 0x17),
};

/// Used in Complete event to indicate the operation
enum envs_op_codes
{
    ENVS_RESERVED_OP_CODE               = 0,
    ENVS_UPD_RANGE_OP_CODE              = 1,
    ENVS_NOTIFY_OP_CODE                 = 2 ,
    ENVS_INDICATE_OP_CODE               = 3
};


/// USER enabling characteristic and it's features
/// these bits are in the same order as @ref enum envs_desc_idx
/// and these reflects the Attribute in the ATT database - don't reorder
enum envs_feature_enable_list
{
    /// descriptors disabled
    ENVS_DESC_DIASBLED    = 0,
    ENVS_DESC_EN_CCC      = (1<<0),
    ENVS_DESC_EN_MEAS     = (1<<1),
    ENVS_DESC_EN_TRIG_1   = (1<<2),
    ENVS_DESC_EN_TRIG_2   = (1<<3),
    ENVS_DESC_EN_TRIG_3   = (1<<4),
    ENVS_DESC_EN_CFG      = (1<<5),
    ENVS_DESC_EN_USR_DESC = (1<<6),
    ENVS_DESC_EN_RANGE    = (1<<7),
};

/// USER configuration of characteristic features write possibility
enum envs_feature_cfg_list
{
    /// ALL descriptors Read Only
    ENVS_FEATURE_RO,
    /// Write to User Description
    ENVS_FEATURE_WR_DESC,
    /// Write to Trigger
    ENVS_FEATURE_WR_TRIG,
    /// Write to User Description and Trigger
    ENVS_FEATURE_WR_DESC_TRIG,
};

/*
 * STRUCTURES
 ****************************************************************************************
 */

/// Characteristic instance
typedef struct envs_char_instance
{
    /// characteristic index from the list @ref envp_es_char
    uint8_t es_char_id;
    /// bitfield from @ref enum envs_feature_enable_list
    uint8_t desc_en;
    /// enum from @ref enum envs_feature_cfg_list
    uint8_t desc_rw;
} envs_char_instance_t;



/// Parameters of the initialization function
/// profile init per characteristic configuration
struct envs_db_cfg
{
    /// not used
    uint8_t option;
    /// number of elements of characteristic instance
    uint8_t nb_chars;
    /// char instances
    envs_char_instance_t instance[__ARRAY_EMPTY];
};

/*
 * Messages Exchange Parameters
 ****************************************************************************************
 */

/// Parameters of the @ref ENVS_INDICATE_CMD message
struct envs_indicate_cmd
{
    /// Connection index
    uint8_t  conidx;
    /// Characteristic Type idx see enum #envp_es_char
    uint8_t  es_char_id;
    /// Describe the Condition of Descriptor change Indication
    uint16_t ind_flag;
};

/// Parameters of the @ref ENVS_NOTIFY_CMD message
struct envs_notify_cmd
{
    /// Connection index
    uint8_t             conidx;
    /// Characteristic Type idx see enum #envp_es_char
    uint8_t             es_char_id;
    /// Characteristic Instance
    uint8_t             es_char_inst;
    /// Actual data for the VAL descriptor
    union envp_val_char value;
};

/*
 * APP UPDATE
 * To update mostly static descriptors
 ****************************************************************************************
 */

/// Parameters of the @ref ENVS_UPD_RANGE_CMD message
/// to be used for updating the characteristic or descriptor individually
struct envs_upd_range_cmd
{
    /// Characteristic Type idx see enum #envp_es_char
    uint8_t          es_char_id;
    /// Characteristic Instance
    uint8_t          es_char_inst;
    /// Actual data for the RANGE descriptor
    union envp_range range;
};

/*
 * Parameters of the Request to the APP when receive write request 
 ****************************************************************************************
 */
/// Parameters of the @ref ENVS_WR_TRIGGER_REQ_IND message
/// request APP for the TRIG Descriptor
struct envs_wr_trigger_req_ind
{
    /// Connection index
    uint8_t        conidx;
    /// Characteristic Type idx see enum #envp_es_char
    uint8_t        es_char_id;
    /// Characteristic Instance
    uint8_t        es_char_inst;
    /// Descriptor idx @ref envs_desc_idx (TRIG1 TRIG2 TRIG3) 0,1,2
    uint8_t        trigger_idx;
    /// Token value to return in confirmation
    uint32_t       token;
    /// Trigger data
    envp_trigger_t trigger;
};

/// Parameters of the @ref ENVS_WR_CCC_REQ_IND message
/// request APP for the CCC Descriptor
struct envs_wr_ccc_req_ind
{
    /// Connection index
    uint8_t  conidx;
    /// Characteristic Type idx see enum #envp_es_char
    uint8_t  es_char_id;
    /// Characteristic Instance
    uint8_t  es_char_inst;
    /// Token value to return in confirmation
    uint32_t token;
    /// Actual data for the CCC descriptor
    uint16_t ccc;
};

/// Parameters of the @ref ENVS_WR_CONFIG_REQ_IND message
/// request APP for the CFG Descriptor
struct envs_wr_config_req_ind
{
    /// Connection index
    uint8_t  conidx;
    /// Characteristic Type idx see enum #envp_es_char
    uint8_t  es_char_id;
    /// Characteristic Instance
    uint8_t  es_char_inst;
    /// Token value to return in confirmation
    uint32_t token;
    /// Actual data for the CFG descriptor @ref enum envp_es_trig_trigger_logic
    uint8_t  trigger_logic;
};

/// Parameters of the @ref ENVS_WR_USER_DESCRIPTION_REQ_IND message
/// same for all connections
/// request APP for the USER Description
struct envs_wr_user_description_req_ind
{
    /// Connection index
    uint8_t     conidx;
    /// Characteristic Type idx see enum #envp_es_char
    uint8_t     es_char_id;
    /// Characteristic Instance
    uint8_t     es_char_inst;
    /// Token value to return in confirmation
    uint32_t    token;
    /// Actual data for the USR_DESCRIPTION descriptor
    prf_utf_8_t user_desc;
};

/*
 * Parameters of the Write Confirmation from the APP
 ****************************************************************************************
 */
/// Parameters of the @ref ENVS_WR_CCC_CFM message
struct envs_wr_ccc_cfm
{
    /// Connection index
    uint8_t  conidx;
    /// Characteristic Type idx see enum #envp_es_char
    uint8_t  es_char_id;
    /// Characteristic Instance
    uint8_t  es_char_inst;
    /// Token value provided in request
    uint32_t token;
    /// Operation Status
    uint16_t status;
};

/// Parameters of the @ref ENVS_WR_TRIGGER_CFM message
struct envs_wr_trigger_cfm
{
    /// Connection index
    uint8_t  conidx;
    /// Characteristic Type idx see enum #envp_es_char
    uint8_t  es_char_id;
    /// Characteristic Instance
    uint8_t  es_char_inst;
    /// Token value provided in request
    uint32_t token;
    /// Operation Status
    uint16_t status;
};

/// Parameters of the @ref ENVS_WR_CONFIG_CFM message
struct envs_wr_config_cfm
{
    /// Connection index
    uint8_t  conidx;
    /// Characteristic Type idx see enum #envp_es_char
    uint8_t  es_char_id;
    /// Characteristic Instance
    uint8_t  es_char_inst;
    /// Token value provided in request
    uint32_t token;
    /// Operation Status
    uint16_t status;
};

/// Parameters of the @ref ENVS_WR_USER_DESCRIPTION_CFM message
struct envs_wr_user_description_cfm
{
    /// Connection index
    uint8_t  conidx;
    /// Characteristic Type idx see enum #envp_es_char
    uint8_t  es_char_id;
    /// Characteristic Instance
    uint8_t  es_char_inst;
    /// Token value provided in request
    uint32_t token;
    /// Descriptor idx @ref envs_desc_idx
    uint16_t status;
};


/*
 * Parameters of the Read Request to the APP
 ****************************************************************************************
 */
/// Parameters of the @ref ENVS_RD_CCC_REQ message
struct envs_rd_ccc_req_ind
{
    /// Connection index
    uint8_t  conidx;
    /// Characteristic Type idx see enum #envp_es_char
    uint8_t  es_char_id;
    /// Characteristic Instance
    uint8_t  es_char_inst;
    /// Token value to return in confirmation
    uint32_t token;
};

/// Parameters of the @ref ENVS_RD_TRIGGER_REQ message
struct envs_rd_trigger_req_ind
{
    /// Connection index
    uint8_t  conidx;
    /// Characteristic Type idx see enum #envp_es_char
    uint8_t  es_char_id;
    /// Characteristic Instance
    uint8_t  es_char_inst;
    /// Trigger instance
    uint8_t  trigger_idx;
    /// Token value to return in confirmation
    uint32_t token;
};

/// Parameters of the @ref ENVS_RD_CONFIG_REQ_IND message
struct envs_rd_config_req_ind
{
    /// Connection index
    uint8_t  conidx;
    /// Characteristic Type idx see enum #envp_es_char
    uint8_t  es_char_id;
    /// Characteristic Instance
    uint8_t  es_char_inst;
    /// Token value to return in confirmation
    uint32_t token;
};

/// Parameters of the @ref ENVS_RD_USER_DESCRIPTION_REQ_IND message
struct envs_rd_user_description_req_ind
{
    /// Connection index
    uint8_t  conidx;
    /// Characteristic Type idx see enum #envp_es_char
    uint8_t  es_char_id;
    /// Characteristic Instance
    uint8_t  es_char_inst;
    /// Token value to return in confirmation
    uint32_t token;
    /// Data offset
    uint16_t offset;
    /// Maximum string length to return
    uint16_t max_len;
};

/// Parameters of the @ref ENVS_RD_VALUE_REQ_IND message
struct envs_rd_value_req_ind
{
    /// Connection index
    uint8_t  conidx;
    /// Characteristic Type idx see enum #envp_es_char
    uint8_t  es_char_id;
    /// Characteristic Instance
    uint8_t  es_char_inst;
    /// Token value to return in confirmation
    uint32_t token;
};

/// Parameters of the @ref ENVS_RD_MEAS_REQ_IND message
struct envs_rd_meas_req_ind
{
    /// Connection index
    uint8_t  conidx;
    /// Characteristic Type idx see enum #envp_es_char
    uint8_t  es_char_id;
    /// Characteristic Instance
    uint8_t  es_char_inst;
    /// Token value to return in confirmation
    uint32_t token;
};

/*
 * Parameters of the Read Request Confirmation from the APP
 ****************************************************************************************
 */
/// Parameters of the @ref ENVS_RD_CCC_CFM message
struct envs_rd_ccc_cfm
{
    /// Connection index
    uint8_t  conidx;
    /// Characteristic Type idx see enum #envp_es_char
    uint8_t  es_char_id;
    /// Characteristic Instance
    uint8_t  es_char_inst;
    /// Token value provided in request
    uint32_t token;
    /// Operation Status
    uint16_t status;
    //// Actual data for the CCC descriptor
    uint16_t ccc;
};

/// Parameters of the @ref ENVS_RD_TRIGGER_CFM message
struct envs_rd_trigger_cfm
{
    /// Connection index
    uint8_t        conidx;
    /// Characteristic Type idx see enum #envp_es_char
    uint8_t        es_char_id;
    /// Characteristic Instance
    uint8_t        es_char_inst;
    /// Descriptor idx @ref envs_desc_idx (TRIG1 TRIG2 TRIG3) 0,1,2
    uint8_t        trigger_idx;
    /// Token value provided in request
    uint32_t       token;
    /// Operation Status
    uint16_t       status;
    /// Trigger data
    envp_trigger_t trigger;
};

/// Parameters of the @ref ENVS_RD_CONFIG_CFM message
struct envs_rd_config_cfm
{
    /// Connection index
    uint8_t  conidx;
    /// Characteristic Type idx see enum #envp_es_char
    uint8_t  es_char_id;
    /// Characteristic Instance
    uint8_t  es_char_inst;
    /// Token value provided in request
    uint32_t token;
    /// Operation Status
    uint16_t status;
    /// Actual data for the CFG descriptor @ref enum envp_es_trig_trigger_logic
    uint8_t  trigger_logic;
};

/// Parameters of the @ref ENVS_RD_USER_DESCRIPTION_CFM message
struct envs_rd_user_description_cfm
{
    /// Connection index
    uint8_t     conidx;
    /// Characteristic Type idx see enum #envp_es_char
    uint8_t     es_char_id;
    /// Characteristic Instance
    uint8_t     es_char_inst;
    /// Token value provided in request
    uint32_t    token;
    /// Operation Status
    uint16_t    status;
    /// Total description length without offset
    uint16_t    total_len;
    /// Actual data for the USR_DESCRIPTION descriptor
    prf_utf_8_t user_desc;
};

/// Parameters of the @ref ENVS_RD_VALUE_CFM message
struct envs_rd_value_cfm
{
    /// Connection index
    uint8_t             conidx;
    /// Characteristic Type idx see enum #envp_es_char
    uint8_t             es_char_id;
    /// Characteristic Instance
    uint8_t             es_char_inst;
    /// Token value provided in request
    uint32_t            token;
    /// status of read
    uint16_t            status;
    //// Actual Characteristic Value
    union envp_val_char value;
};

/// Parameters of the @ref ENVS_RD_MEAS_CFM message
struct envs_rd_meas_cfm
{
    /// Connection index
    uint8_t             conidx;
    /// Characteristic Type idx see enum #envp_es_char
    uint8_t             es_char_id;
    /// Characteristic Instance
    uint8_t             es_char_inst;
    /// Token value provided in request
    uint32_t            token;
    /// status of read
    uint16_t            status;
    /// Actual data for the MEAS descriptor
    envp_es_meas_desc_t meas;
};

/*
 * Confirmation to the APP after Update, Indicate, Notify messages
 ****************************************************************************************
 */
/// Parameters of the @ref ENVS_CMP_EVT message
struct envs_cmp_evt
{
    /// Connection index
    uint8_t  conidx;
    /// Operation Code
    uint8_t  operation;
    /// Operation Status
    uint16_t status;
};


/// @} ENVS

#endif //(_ENVS_MSG_H_)


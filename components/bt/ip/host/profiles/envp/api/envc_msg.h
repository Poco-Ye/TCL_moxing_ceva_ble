/**
 ****************************************************************************************
 *
 * @file envc_msg.h
 *
 * @brief Header file - The Environmental Sensing Collector/Client Role - Message API.
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */

#ifndef _ENVC_MSG_H_
#define _ENVC_MSG_H_

/**
 ****************************************************************************************
 * @addtogroup ENVC
 * @ingroup Profile
 * @brief  The Environmental Sensing Profile Collector  - Message API.
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

/// ES Measurement Descriptors
#define ENVC_ES_MAX_CHARS_ALLOWED    (22)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */


/// Message IDs
/*@TRACE*/
enum envc_msg_ids
{
    /// Enable the Profile Collector task - at connection
    ENVC_ENABLE_REQ              = MSG_ID(ENVC, 0x00),
    /// Response to Enable the Profile Collector task - at connection
    ENVC_ENABLE_RSP              = MSG_ID(ENVC, 0x01),
    /// Information on discovered characteristic
    ENVC_ES_INFO_IND             = MSG_ID(ENVC, 0x02),

    ///*** ES CHARACTERISTIC/DESCRIPTOR READ REQUESTS
    /// Read the Characteristic Value of a measurement sensor characteristic
    ENVC_RD_VALUE_CMD            = MSG_ID(ENVC, 0x03),
    /// Read the Descriptors value of a measurement sensor characteristic
    ENVC_RD_DESC_CMD             = MSG_ID(ENVC, 0x04),

    ///*** ES CHARACTERISTIC/DESCRIPTOR READ RESPONSES
    /// Response to Read the Characteristic Value of a measurement sensor characteristic
    ENVC_RD_VALUE_IND            = MSG_ID(ENVC, 0x05),
    /// Response to Read the CCC of a measurement sensor characteristic
    ENVC_RD_NTF_CFG_IND          = MSG_ID(ENVC, 0x06),
    /// Response to Read the measurement descriptor of a measurement sensor characteristic
    ENVC_RD_MEAS_DESCRIPTOR_IND  = MSG_ID(ENVC, 0x07),
    /// Response to Read one of the Triggers of a measurement sensor characteristic
    ENVC_RD_TRIGGER_IND          = MSG_ID(ENVC, 0x08),
    /// Response to Read the trigger config of a measurement sensor characteristic
    ENVC_RD_CONFIG_IND           = MSG_ID(ENVC, 0x09),
    /// Response to Read the sensor range of a measurement sensor characteristic
    ENVC_RD_RANGE_IND            = MSG_ID(ENVC, 0x0A),
    /// Response to Read the user description (name) of a measurement sensor characteristic
    ENVC_RD_USER_DESCRIPTION_IND = MSG_ID(ENVC, 0x0B),
    /// Response toRead the extended properties of a measurement sensor characteristic
    ENVC_RD_EXT_PROP_IND         = MSG_ID(ENVC, 0x0C),

    ///***  CHARACTERISTIC/DESCRIPTOR WRITE REQUESTS
    /// Write the CCC of a measurement sensor characteristic
    ENVC_WR_NTF_CFG_CMD          = MSG_ID(ENVC, 0x0D),
    /// Write one of the Trigger values of a measurement sensor characteristic
    ENVC_WR_TRIGGER_CMD          = MSG_ID(ENVC, 0x0E),
    /// Write one of the Trigger Config of a measurement sensor characteristic
    ENVC_WR_CONFIG_CMD           = MSG_ID(ENVC, 0x0F),
    /// Write the user description (name) of a measurement sensor characteristic
    ENVC_WR_USER_DESCRIPTION_CMD = MSG_ID(ENVC, 0x10),
    /// Write the extended of a measurement sensor characteristic

    /// Characteristic Value Notification from peer
    ENVC_VALUE_IND               = MSG_ID(ENVC, 0x11),
    /// Descriptor Value Change Ind - indicating one of the descriptors in the Server has changed
    ENVC_DVC_IND                 = MSG_ID(ENVC, 0x12),
    /// Complete Event Information
    ENVC_CMP_EVT                 = MSG_ID(ENVC, 0x13),
};

/// Optional Descriptors used for each ES Characteristic
enum envc_meas_descriptors
{
    /// Client Characteristic Configuration
    ENVP_DESCR_CCC                  = 1,
    /// Measurement Descriptor
    ENVP_DESCR_MEASUREMENT          = 2,
    /// Descriptor for first Trigger
    ENVP_DESCR_TRIGGER_1            = 3,
    /// Descriptor for second Trigger
    ENVP_DESCR_TRIGGER_2            = 4,
    /// Descriptor for third Trigger
    ENVP_DESCR_TRIGGER_3            = 5,
    /// Descriptor for configuration
    ENVP_DESCR_CONFIG               = 6,
    /// Descriptor for User Description
    ENVP_DESCR_USER_DESCRIPTION     = 7,
    /// Descriptor for Valid Range
    ENVP_DESCR_VALUE_RANGE          = 8,
    /// Descriptor for External Properties
    ENVP_DESCR_EXT_PROP             = 9,
};

/// Operation Codes for pending operations in the client.
enum envc_op_codes
{
    /// Reserved operation code
    ENVC_RESERVED_OP_CODE               = 0x00,

    /// Read op-codes
    ENVC_READ_VALUE_OP_CODE             = 0x01,
    ENVC_READ_CCC_OP_CODE               = 0x02,
    ENVC_READ_CONFIG_OP_CODE            = 0x03,
    ENVC_READ_MEAS_DESCRIPTOR_OP_CODE   = 0x04,
    ENVC_READ_TRIGGER_OP_CODE           = 0x05,
    ENVC_READ_USER_DESCRIPTION_OP_CODE  = 0x06,
    ENVC_READ_RANGE_OP_CODE             = 0x07,
    ENVC_READ_EXT_PROP_OP_CODE          = 0x08,

    /// Write op-codes.
    ENVC_WRITE_CCC_OP_CODE              = 0x09,
    ENVC_WRITE_CONFIG_OP_CODE           = 0x0A,
    ENVC_WRITE_TRIGGER_OP_CODE          = 0x0B,
    ENVC_WRITE_USER_DESCRIPTION_OP_CODE = 0x0C,
};

/**
 * Structure containing the characteristics handles, value handles and descriptors for
 * the Environment Sensing Service
 */
typedef struct envc_es_char_info
{
    /// ES Characteristic Identifier
    uint8_t  es_char_id;
    /// ES Characteristic Instance
    uint8_t  es_char_inst;
    /// Characteristic handle
    uint16_t es_char_hdl;
    /// Value handle
    uint16_t es_val_hdl;
    /// End Handle
    uint16_t es_end_hdl;

    /// Characteristic properties
    uint8_t  prop;

    /// Characteristic Descriptor Handles
    ///
    /// CCC handle
    uint16_t ccc_desc_hdl;
    /// Measurement Descriptor handle
    uint16_t meas_desc_hdl;
    /// Handle for First Trigger Descriptor
    uint16_t trigger_1_hdl;
    /// Handle for Second Trigger Descriptor
    uint16_t trigger_2_hdl;
    /// Handle for Third Trigger Descriptor
    uint16_t trigger_3_hdl;
    /// Handle for Config Descriptor (trigger logic)
    uint16_t config_desc_hdl;
    /// Handle for User Description descriptor (sensor name)
    uint16_t user_description_hdl;
    /// Handle for Valid Range Descriptor
    uint16_t range_desc_hdl;
    /// Handle for External Properties
    uint16_t exp_prop_desc_hdl;
} envc_es_char_info_t;

/**
 *  Server Information required for a connection
 */
typedef struct envc_es_content
{
    /// Service info
    prf_svc_t           svc;
    /// Number of ES Characteristic Discovered
    uint8_t             nb_chars_discovered;
    /// Array of containing the handle information of each characteristic
    envc_es_char_info_t chars[ENVC_ES_MAX_CHARS_ALLOWED];
} envc_es_content_t;


/*
 * API MESSAGE STRUCTURES
 ****************************************************************************************
 */


/// Parameters of the @ref ENVC_ENABLE_REQ message
struct envc_enable_req
{
    /// Connection index
    uint8_t           conidx;
    /// Connection type
    uint8_t           con_type;
    /// Existing handle values ENVC ES
    envc_es_content_t es;
};

/// Parameters of the @ref ENVC_ENABLE_RSP message
struct envc_enable_rsp
{
    /// Connection index
    uint8_t   conidx;
    /// status
    uint16_t  status;
    /// service - start/end handle
    prf_svc_t svc;
    /// Number of discovered Characteristics
    uint8_t   nb_chars_discovered;
};

/// Parameters of the @ref ENVC_RD_DESC_CMD message
struct envc_rd_desc_cmd
{
    /// Connection index
    uint8_t  conidx;
    /// ES Sensor Id see enum #envp_es_char
    uint8_t  es_char_id;
    /// ES Sensor Instance
    uint8_t  es_char_inst;
    /// Descriptor type
    uint8_t  desc_type;
};

/// Parameters of the @ref ENVC_RD_VALUE_CMD message
struct envc_rd_value_cmd
{
    /// Connection index
    uint8_t  conidx;
    /// ES Characteristic ID
    uint8_t  es_char_id;
    /// ES Characteristic Instance
    uint8_t  es_char_inst;
};

/// Parameters of the @ref ENVC_WR_TRIGGER_CMD message
struct envc_wr_trigger_cmd
{
    /// Connection index
    uint8_t             conidx;
    /// ES Characteristic ID
    uint8_t             es_char_id;
    /// ES Characteristic Instance
    uint8_t             es_char_inst;
    /// Trigger Identifier (0,1,2)
    uint8_t             trigger_idx;
    /// Trigger data
    envp_trigger_t      trigger;
};

/// Parameters of the @ref ENVC_WR_NTF_CMD message
struct envc_wr_ntf_cmd
{
    /// Connection index
    uint8_t  conidx;
    /// ES Characteristic ID
    uint8_t  es_char_id;
    /// ES Characteristic Instance
    uint8_t  es_char_inst;
    /// Flags setting
    uint8_t  flags;

};

/// Parameters of the @ref ENVC_WR_CONFIG_CMD message
struct envc_wr_config_cmd
{
    /// Connection index
    uint8_t  conidx;
    /// ES Characteristic ID
    uint8_t  es_char_id;
    /// ES Characteristic Instance
    uint8_t  es_char_inst;
    /// ES Char Configuration
    uint8_t  trigger_logic;

};

/// Parameters of the @ref ENVC_WR_USER_DESCRIPTION_CMD message
struct envc_wr_user_description_cmd
{
    /// Connection index
    uint8_t     conidx;
    /// ES Characteristic ID
    uint8_t     es_char_id;
    /// ES Characteristic Instance
    uint8_t     es_char_inst;
    /// UFT String representing name
    prf_utf_8_t user_desc;

};

/// Parameters of the @ref ENVC_ES_INFO_IND message
struct envc_es_info_ind
{
    /// Connection index
    uint8_t             conidx;
    /// Characteristic information
    envc_es_char_info_t info;
};


/// Parameters of the @ref ENVC_RD_VALUE_IND message
struct envc_rd_value_ind
{
    /// Connection index
    uint8_t             conidx;
    /// ES Sensor Id see enum #envp_es_char
    uint8_t             es_char_id;
    /// ES Sensor Instance
    uint8_t             es_char_inst;
    /// The ES Char Value
    union envp_val_char es_char_value;
};

/// Parameters of the @ref ENVC_RD_NTF_CFG_IND message
struct envc_rd_ntf_cfg_ind
{
    /// Connection index
    uint8_t  conidx;
    /// ES Sensor Id see enum #envp_es_char
    uint8_t  es_char_id;
    /// ES Sensor Instance
    uint8_t  es_char_inst;
    /// The Client Characteristic Configuration Value
    uint16_t ccc;
};

/// Parameters of the @ref ENVC_RD_MEAS_DESCRIPTOR_IND message
struct envc_rd_meas_descriptor_ind
{
    /// Connection index
    uint8_t             conidx;
    /// ES Sensor Id see enum #envp_es_char
    uint8_t             es_char_id;
    /// ES Sensor Instance
    uint8_t             es_char_inst;
    /// The measurement descriptor value
    envp_es_meas_desc_t meas;
};

/// Parameters of the @ref ENVC_RD_TRIGGER_IND message
struct envc_rd_trigger_ind
{
    /// Connection index
    uint8_t             conidx;
    /// ES Sensor Id see enum #envp_es_char
    uint8_t             es_char_id;
    /// ES Sensor Instance
    uint8_t             es_char_inst;
    /// Trigger index
    uint8_t             trigger_idx;
    /// Trigger data
    envp_trigger_t      trigger;
};

/// Parameters of the @ref ENVC_RD_CONFIG_IND message
struct envc_rd_config_ind
{
    /// Connection index
    uint8_t  conidx;
    /// ES Sensor Id see enum #envp_es_char
    uint8_t  es_char_id;
    /// ES Sensor Instance
    uint8_t  es_char_inst;
    /// Configuration of the Triggers
    uint8_t  trigger_logic;
};

/// Parameters of the @ref ENVC_RD_RANGE_IND message
struct envc_rd_range_ind
{
    /// Connection index
    uint8_t          conidx;
    /// ES Sensor Id see enum #envp_es_char
    uint8_t          es_char_id;
    /// ES Sensor Instance
    uint8_t          es_char_inst;
    /// ES Range Value
    union envp_range range;
};

/// Parameters of the @ref ENVC_RD_EXT_PROP_IND message
struct envc_rd_ext_prop_ind
{
    /// Connection index
    uint8_t  conidx;
    /// ES Sensor Id see enum #envp_es_char
    uint8_t  es_char_id;
    /// ES Sensor Instance
    uint8_t  es_char_inst;
    /// Extended Properties
    uint16_t ext_prop;
};

/// Parameters of the @ref ENVC_RD_USER_DESCRIPTION_IND message
struct envc_rd_user_description_ind
{
    /// Connection index
    uint8_t     conidx;
    /// ES Sensor Id see enum #envp_es_char
    uint8_t     es_char_id;
    /// ES Sensor Instance
    uint8_t     es_char_inst;
    /// UTF String for name of Sensor
    prf_utf_8_t user_desc;
};

/// Parameters of the @ref ENVC_VALUE_IND message
struct envc_value_ind
{
    /// Connection index
    uint8_t             conidx;
    /// ES Sensor Id see enum #envp_es_char
    uint8_t             es_char_id;
    /// ES Sensor Instance
    uint8_t             es_char_inst;
    /// ES Value
    union envp_val_char value;
};

/// Parameters of the @ref ENVC_DVC_IND message
struct envc_dvc_ind
{
    /// Connection index
    uint8_t  conidx;
    /// ES Sensor Id see enum #envp_es_char
    uint8_t  es_char_id;
    /// Flags indicating change type and source
    uint16_t flags;
};

/// Parameters of the @ref ENVC_CMP_EVT message
struct envc_cmp_evt
{
    /// Connection index
    uint8_t  conidx;
    /// Operation code
    uint8_t  operation;
    /// Status
    uint16_t status;
};

/// @} ENVC

#endif //(_ENVC_MSG_H_)

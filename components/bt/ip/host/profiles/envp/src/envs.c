/**
 ****************************************************************************************
 *
 * @file envs.c
 *
 * @brief Environmental Sensing Service Profile implementation.
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup ENVS
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_ENV_SERVER)

#include "envs.h"
#include "gap.h"
#include "gatt.h"
#include "prf_utils.h"
#include "prf.h"

#include "co_utils.h"
#include "co_endian.h"

#include <string.h>
#include "ke_mem.h"


/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

#define BITS_IN_BYTE                  (8)


/// Content of ENVS token
enum envs_token_bf
{
    /// GATT procedure token
    ENVS_TOKEN_GATT_TOKEN_MASK   = 0x0000FFFF,
    ENVS_TOKEN_GATT_TOKEN_LSB    = 0,
    /// Characteristic idx
    ENVS_TOKEN_ES_CHAR_ID_MASK   = 0x00FF0000,
    ENVS_TOKEN_ES_CHAR_ID_LSB    = 16,
    /// Characteristic Instance
    ENVS_TOKEN_ES_CHAR_INST_MASK = 0x3F000000,
    ENVS_TOKEN_ES_CHAR_INST_LSB  = 24,
    /// Trigger idx
    ENVS_TOKEN_TRIGGER_IDX_MASK  = 0xC0000000,
    ENVS_TOKEN_TRIGGER_IDX_LSB   = 30,
};

/// Environment sensor Service - Attribute List
enum envs_desc_idx
{
    ENVS_IDX_CHAR_CHAR = 0,
    ENVS_IDX_CHAR_VAL,
    ENVS_IDX_DESC_CCC,        /// CCC
    ENVS_IDX_DESC_MEAS,       /// Descriptor - Measurement; if_multiple_instances_of_same_characteristic_optional_otherwise
    ENVS_IDX_DESC_TRIG_1,     /// Trigger Setting; if_notify_supported
    ENVS_IDX_DESC_TRIG_2,     /// Trigger Setting; if_notify_supported
    ENVS_IDX_DESC_TRIG_3,     /// Trigger Setting; if_notify_supported
    ENVS_IDX_DESC_CFG,        /// Configuration
    ENVS_IDX_DESC_USR_DESC,   /// User Description
    ENVS_IDX_DESC_RANGE,      /// Valid Range
    ENVS_IDX_DESC_EXT,        /// Extended

    ENVS_IDX_ATT_NB
};

/// Bit field of ntf/ind configuration
enum envs_ntf_ind_cfg_bf
{
    /// DB Change Increment notification
    ENVS_CFG_DB_CHG_INC_NTF_BIT = 0x01,
    ENVS_CFG_DB_CHG_INC_NTF_POS = 0,
    /// Control point indication
    ENVS_CFG_CTRL_PT_IND_BIT    = 0x02,
    ENVS_CFG_CTRL_PT_IND_POS    = 1,
};

/*
 * TYPES DEFINITION
 ****************************************************************************************
 */

/// ongoing operation information
typedef struct envs_buf_meta
{
    /// Operation
    uint8_t   operation;
    /// used to know on which device interval update has been requested, and to prevent
    /// indication to be triggered on this connection index
    uint8_t   conidx;
    /// Handle to notify / indicate
    uint16_t  hdl;
    /// Event type (Notify or indicate)
    uint8_t   evt_type;
} envs_buf_meta_t;

/// Environment sensor characteristic information
typedef struct envs_char_info
{
    /// Valid value range
    union envp_range range;
    /// characteristic index from the list @ref envp_es_char
    uint8_t          es_char_id;
    /// bitfield from @ref enum envs_feature_enable_list
    uint8_t          desc_en;
    /// enum from @ref enum envs_feature_cfg_list
    uint8_t          desc_rw;
} envs_char_info_t;

/// ENVS Environment Variable
typedef struct envs_env
{
    /// profile environment
    prf_hdr_t            prf_env;
    /// Operation Event TX wait queue
    co_list_t            wait_queue;
    /// Service Attribute Start Handle
    uint16_t             start_hdl;
    /// GATT user local identifier
    uint8_t              user_lid;
    /// Operation On-going
    bool                 op_ongoing;
    /// Prevent recursion in execute_operation function
    bool                 in_exe_op;

    /// Number of Characteristics
    uint8_t              nb_chars;
    /// char instances list with enabled features and write condition
    /// this put to the end of structure to allocate in single memory block
    envs_char_info_t     instance[__ARRAY_EMPTY];
} envs_env_t;

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// characteristic value sizes
__STATIC const uint8_t envs_val_range_size[] =
{
    0, /* ENVS_IDX_CHAR_VAL_CHANGE */
    ENVS_ES_APRNT_WIND_DIR_SIZE,
    ENVS_ES_APRNT_WIND_SPEED_SIZE,
    ENVS_ES_DEW_POINT_SIZE,
    ENVS_ES_ELEVATION_SIZE,
    ENVS_ES_GUST_FACTOR_SIZE,
    ENVS_ES_HEAT_INDEX_SIZE,
    ENVS_ES_HUMIDITY_SIZE,
    ENVS_ES_IRRADIANCE_SIZE,
    ENVS_ES_POLLEN_CONC_SIZE,
    ENVS_ES_RAINFALL_SIZE,
    ENVS_ES_PRESSURE_SIZE,
    ENVS_ES_TEMPERATURE_SIZE,
    ENVS_ES_TRUE_WIND_DIR_SIZE,
    ENVS_ES_TRUE_WIND_SPEED_SIZE,
    ENVS_ES_UV_INDEX_SIZE,
    ENVS_ES_WIND_CHILL_SIZE,
    ENVS_ES_BAR_PRES_TREND_SIZE,
    ENVS_ES_MAGN_DECLINE_SIZE,
    ENVS_ES_MAGN_FLUX_2D_SIZE,
    ENVS_ES_MAGN_FLUX_3D_SIZE
};
/*
 * ATTRIBUTES DATABASE
 ****************************************************************************************
 */

/// default read perm
#define RD_P    (PROP(RD)  | SEC_LVL(RP,  NOT_ENC))
/// default write perm
#define WR_P    (PROP(WR)  | SEC_LVL(WP,  AUTH))
/// default notify perm
#define NTF_P   (PROP(N)   | SEC_LVL(NIP, AUTH))
/// ind perm
#define IND_P   (PROP(I)   | SEC_LVL(NIP, AUTH))

#define WR_P_UA (PROP(WR)  | SEC_LVL(WP,  NO_AUTH))


/// Full ENVS Database Description - Used to add attributes into the database
const gatt_att16_desc_t envs_att_db_header[] =
{
    // ATT UUID                      | Permission        | EXT PERM | MAX ATT SIZE
    // Environmental Sensing Service Service Declaration
    { GATT_DECL_PRIMARY_SERVICE,           RD_P,              0                 },
    // Descriptor Value Changed Characteristic Declaration
    { GATT_DECL_CHARACTERISTIC,            RD_P,              0                 },
    // Descriptor Value Changed Characteristic Value
    { GATT_CHAR_DESCRIPTOR_VALUE_CHANGED,  IND_P,             0                 },
    // Client Characteristic Configuration Descriptor
    { GATT_DESC_CLIENT_CHAR_CFG,           RD_P | WR_P,       OPT(NO_OFFSET)    },
};

const gatt_att16_desc_t envs_att_db_descriptors1[] =
{
    // Apparent Wind Direction Characteristic Declaration
    { GATT_DECL_CHARACTERISTIC,            RD_P,              0                 },
};


const gatt_att16_desc_t envs_att_db_descriptors3[] =
{
    // Client Characteristic Configuration Descriptor
    { GATT_DESC_CLIENT_CHAR_CFG,           RD_P | WR_P,       OPT(NO_OFFSET)                                        },
    // Environmental Sensing Measurement Descriptor
    { GATT_DESC_ES_MEASUREMENT,            RD_P,              OPT(NO_OFFSET) | ENVS_ES_MEAS_SIZE                    },
    // Environmental Sensing Trigger Setting Descriptor
    { GATT_DESC_ES_TRIGGER_SETTING,        RD_P | WR_P,       OPT(NO_OFFSET) | ENVS_TRIG_SIZE                       },
    // Environmental Sensing Trigger Setting Descriptor
    { GATT_DESC_ES_TRIGGER_SETTING,        RD_P | WR_P,       OPT(NO_OFFSET) | ENVS_TRIG_SIZE                       },
    // Environmental Sensing Trigger Setting Descriptor
    { GATT_DESC_ES_TRIGGER_SETTING,        RD_P | WR_P,       OPT(NO_OFFSET) | ENVS_TRIG_SIZE                       },
    // Environmental Sensing Configuration Descriptor
    { GATT_DESC_ES_CONFIGURATION,          RD_P | WR_P,       OPT(NO_OFFSET) | ENVS_CFG_SIZE                        },
    // Characteristic user description Descriptor
    { GATT_DESC_CHAR_USER_DESCRIPTION,     RD_P | WR_P_UA,    ENVS_USR_DESC_MAX_LEN                                 },
    // Valid Range Descriptor
    { GATT_DESC_VALID_RANGE,               RD_P,              OPT(NO_OFFSET) | (2 * ENVS_ES_APRNT_WIND_DIR_SIZE)    },
    // Characteristic Extended Properties Descriptor
    { GATT_DESC_CHAR_EXT_PROPERTIES,       RD_P,              GATT_EXT_WRITABLE_AUX                                 },
};

const gatt_att16_desc_t envs_att_db_chars[] =
{
    // Descriptor Value Changed Characteristic Value
    [ENVP_ES_DESCRIPTOR_VALUE_CHANGED_CHAR] =   { GATT_CHAR_DESCRIPTOR_VALUE_CHANGED, IND_P,         ENVP_DESC_CHANGE_MAX_LEN      },
    // Apparent Wind Direction Characteristic
    [ENVP_ES_APRNT_WIND_DIRECTION_CHAR]     =   { GATT_CHAR_APRNT_WIND_DIRECTION,     RD_P | NTF_P,  ENVS_ES_APRNT_WIND_DIR_SIZE   },
    // Apparent Wind Speed Characteristic
    [ENVP_ES_APRNT_WIND_SPEED_CHAR]         =   { GATT_CHAR_APRNT_WIND_SPEED,         RD_P | NTF_P,  ENVS_ES_APRNT_WIND_SPEED_SIZE },
    // Dew Point Characteristic
    [ENVP_ES_DEW_POINT_CHAR]                =   { GATT_CHAR_DEW_POINT,                RD_P | NTF_P,  ENVS_ES_DEW_POINT_SIZE        },
    // Elevation Characteristic
    [ENVP_ES_ELEVATION_CHAR]                =   { GATT_CHAR_ELEVATION,                RD_P | NTF_P,  ENVS_ES_ELEVATION_SIZE        },
    // Gust Factor Characteristic
    [ENVP_ES_GUST_FACTOR_CHAR]              =   { GATT_CHAR_GUST_FACTOR,              RD_P | NTF_P,  ENVS_ES_GUST_FACTOR_SIZE      },
    // Heat Index Characteristic
    [ENVP_ES_HEAT_INDEX_CHAR]               =   { GATT_CHAR_HEAT_INDEX,               RD_P | NTF_P,  ENVS_ES_HEAT_INDEX_SIZE       },
    // Humidity Characteristic
    [ENVP_ES_HUMIDITY_CHAR]                 =   { GATT_CHAR_HUMIDITY,                 RD_P | NTF_P,  ENVS_ES_HUMIDITY_SIZE         },
    // Irradiance Characteristic
    [ENVP_ES_IRRADIANCE_CHAR]               =   { GATT_CHAR_IRRADIANCE,               RD_P | NTF_P,  ENVS_ES_IRRADIANCE_SIZE       },
    // Pollen Concentration Characteristic
    [ENVP_ES_POLLEN_CONC_CHAR]              =   { GATT_CHAR_POLLEN_CONC,              RD_P | NTF_P,  ENVS_ES_POLLEN_CONC_SIZE      },
    // Rainfall Characteristic
    [ENVP_ES_RAINFALL_CHAR]                 =   { GATT_CHAR_RAINFALL,                 RD_P | NTF_P,  ENVS_ES_RAINFALL_SIZE         },
    // Pressure Characteristic
    [ENVP_ES_PRESSURE_CHAR]                 =   { GATT_CHAR_PRESSURE,                 RD_P | NTF_P,  ENVS_ES_PRESSURE_SIZE         },
    // Temperature Characteristic
    [ENVP_ES_TEMPERATURE_CHAR]              =   { GATT_CHAR_TEMPERATURE,              RD_P | NTF_P,  ENVS_ES_TEMPERATURE_SIZE      },
    // True Wind Direction Characteristic
    [ENVP_ES_TRUE_WIND_DIR_CHAR]            =   { GATT_CHAR_TRUE_WIND_DIR,            RD_P | NTF_P,  ENVS_ES_TRUE_WIND_DIR_SIZE    },
    // True Wind Speed Characteristic
    [ENVP_ES_TRUE_WIND_SPEED_CHAR]          =   { GATT_CHAR_TRUE_WIND_SPEED,          RD_P | NTF_P,  ENVS_ES_TRUE_WIND_SPEED_SIZE  },
    // UV Index Characteristic
    [ENVP_ES_UV_INDEX_CHAR]                 =   { GATT_CHAR_UV_INDEX,                 RD_P | NTF_P,  ENVS_ES_UV_INDEX_SIZE         },
    // Wind Chill Characteristic
    [ENVP_ES_WIND_CHILL_CHAR]               =   { GATT_CHAR_WIND_CHILL,               RD_P | NTF_P,  ENVS_ES_WIND_CHILL_SIZE       },
    // Barometric Pressure Trend Characteristic
    [ENVP_ES_BAR_PRES_TREND_CHAR]           =   { GATT_CHAR_BAR_PRES_TREND,           RD_P | NTF_P,  ENVS_ES_BAR_PRES_TREND_SIZE   },
    // Magnetic Declination Characteristic
    [ENVP_ES_MAGN_DECLINE_CHAR]             =   { GATT_CHAR_MAGN_DECLINE,             RD_P | NTF_P,  ENVS_ES_MAGN_DECLINE_SIZE     },
    // Magnetic Flux Density - 2D Characteristic
    [ENVP_ES_MAGN_FLUX_2D_CHAR]             =   { GATT_CHAR_MAGN_FLUX_2D,             RD_P | NTF_P,  ENVS_ES_MAGN_FLUX_2D_SIZE     },
    // Magnetic Flux Density - 3D Characteristic
    [ENVP_ES_MAGN_FLUX_3D_CHAR]             =   { GATT_CHAR_MAGN_FLUX_3D,             RD_P | NTF_P,  ENVS_ES_MAGN_FLUX_3D_SIZE     },
};


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
__STATIC void envs_exe_operation(envs_env_t* p_envs_env);

/**
 ****************************************************************************************
 * @brief count enabled attributes for characteristic
 *
 * @param[in]  p_en_flag  Characteristic Attribute index see @enum envs_att_list.
 * @param[out] p_en_flag  Handle for the GATT request.
 *
 * @return number of enabled attributes
 ****************************************************************************************
 */
__STATIC uint8_t envs_count_enabled_desc(uint8_t *p_en_flag)
{
    uint8_t cfg = *p_en_flag;
    uint8_t trig_nb;
    uint8_t enabled_desc_nb;

    cfg &= ENVS_DESC_EN_MASK;
    //check the ES Configuration is only present if more than 1 TRIG is enabled
    trig_nb = cfg & ( ENVS_DESC_EN_TRIG_1 | ENVS_DESC_EN_TRIG_2 | ENVS_DESC_EN_TRIG_3);
    trig_nb = NB_ONE_BITS(trig_nb);

    if (cfg)
    {
        enabled_desc_nb = NB_ONE_BITS(cfg);
        // add least 2 enabled_desc_nb with characteristic attribute(ENVS_IDX_CHAR_CHAR) and value(ENVS_IDX_CHAR_VAL)
        enabled_desc_nb += 2;
        // add 1 for ENVS_IDX_DESC_EXT
        if (cfg & ENVS_DESC_EN_USR_DESC)
        {
            enabled_desc_nb += 1;
        }
    }
    else
    {
        //no feature present
        enabled_desc_nb = 0;
    }

    *p_en_flag = cfg;

    return enabled_desc_nb;
}

/**
 ****************************************************************************************
 * @brief convert Characteristic type , Instance of it and Descriptor to 'handle'
 *
 * @param[in] es_char_id   Characteristic type index see @enum envs_char_idx.
 * @param[in] es_char_inst   Characteristic Instance.
 * @param[in] desc_idx    Descriptor see @ref envs_desc_idx.
 * @param[out] p_handle_raw  Handle from the GATT request.
 * @return check if the attribute is enabled in this configuration
 ****************************************************************************************
 */
__STATIC uint16_t envs_char_2_handle(uint8_t es_char_id, uint8_t es_char_inst, uint8_t desc_idx, uint16_t *p_handle_raw)
{
    envs_env_t *p_envs_env = PRF_ENV_GET(ENVS, envs);
    uint16_t handle = p_envs_env->start_hdl;
    uint8_t char_idx;
    // Bitfield for the ATT database
    // 10 bits of attribute per characteristic
    uint16_t flag;
    // temporally keep each cfg byte
    uint8_t cfg;

    // enabled_desc_nb count for configuration flags
    uint8_t enabled_desc_nb;
    // temp index
    uint8_t i;
    uint16_t status = GAP_ERR_NO_ERROR;

    do
    {
        // First descriptor is of special case
        // containing only Value, CCC descriptors
        if (es_char_id == ENVP_ES_DESCRIPTOR_VALUE_CHANGED_CHAR)
        {
            handle++;

            if ((p_envs_env->instance[0].desc_en) && (desc_idx <= ENVS_IDX_DESC_CCC))
            {
                handle += desc_idx;
            }
            else
            {
                status = ATT_ERR_INVALID_HANDLE;
                break;
            }
        }
        else
        {
            char_idx = 0;

            // Find position in the list of the Characteristic
            // go through the list finding the repeat(instance) of the given Characteristic
            for (i = 1; i < p_envs_env->nb_chars; i++)
            {
                if (es_char_id == p_envs_env->instance[i].es_char_id)
                {
                    if (es_char_inst)
                    {
                        es_char_inst--;
                    }
                    else
                    {
                        char_idx = i;
                        break;
                    }
                }
            }

            // not found
            if (char_idx == 0)
            {
                status = ATT_ERR_INVALID_HANDLE;
                break;
            }

            // skip the Service declaration handle
            handle += 1;

            // if ENVS_IDX_VAL_CHANGE_CHAR is present
            // then will contain 3 handles: CHAR declaration,VAL & CCC
            if (p_envs_env->instance[0].desc_en)
            {
                handle += 3;
            }

            // up to the position of found Characteristic in the list
            // add counted enabled descriptors to handle
            for (i = 1; i < char_idx; i++)
            {
                cfg = p_envs_env->instance[i].desc_en;
                enabled_desc_nb = envs_count_enabled_desc(&cfg);
                handle += enabled_desc_nb;
            }

            // for the Characteristic in search of specific descriptor
            // need to match to the bitfield of enabled descriptors
            // for the specific descriptor count enabled descriptors
            cfg = p_envs_env->instance[char_idx].desc_en;
            enabled_desc_nb = envs_count_enabled_desc(&cfg);
            // and prepare bitfield of all enabled descriptors
            flag = cfg;
            // add must have attributes:  CHAR declaration,VALUE
            flag <<= 2;
            flag |= (1<<ENVS_IDX_CHAR_CHAR)|(1<<ENVS_IDX_CHAR_VAL); //

            // add 'Extended descriptor' if 'USER Description descriptor' is present
            if (cfg & ENVS_DESC_EN_USR_DESC)
            {
                flag |= (1<<ENVS_IDX_DESC_EXT); //
            }

            // check if the descriptor in search is present
            if ((flag & (1<<desc_idx)) == 0)
            {
                status = ATT_ERR_INVALID_HANDLE;
                break;
            }

            // now count enabled descriptors before the searched one
            for (i = 0; i < desc_idx; i++)
            {
                if ((flag & 1) == 1)
                {
                    handle++;
                }
                flag >>=1;
            }
        }

        *p_handle_raw = handle;
    } while(0);

    return (status);
}

/**
 ****************************************************************************************
 * @brief convert 'handle' to Characteristic type , Instance of it and Descriptor
 *
 * @param[in]  handle_raw       Handle from the GATT request.
 * @param[out] p_es_char_id     Characteristic type index see @enum envs_char_idx.
 * @param[out] p_es_char_inst   Characteristic Instance.
 * @param[out] p_desc_idx       Descriptor see @ref envs_desc_idx.
 *
 ****************************************************************************************
 */
__STATIC void envs_handle_2_char(uint16_t handle_raw, uint8_t *p_es_char_id, uint8_t *p_es_char_inst, uint8_t *p_desc_idx,
                                 uint8_t* p_char_idx)
{
    // result Characteristic Index
    uint8_t res_char = 0;
    // result Characteristic instance
    uint8_t res_inst;
    // result Characteristic Descritor
    uint8_t res_desc;
    envs_env_t *p_envs_env = PRF_ENV_GET(ENVS, envs);
    uint16_t handle = handle_raw - p_envs_env->start_hdl;

    // Bitfield for the ATT database
    // 10 bits of attribute per characteristic
    uint16_t flag = 0;
    // temporally keep each cfg byte
    uint8_t cfg;

    // number of enabled attributes - count from enabled filed in the list of Characteristics
    // if ENVS_IDX_VAL_CHANGE_CHAR is present then VAL & CCC also otherwise only ENVS_IDX_SVC
    uint8_t enabled_desc_nb  = (p_envs_env->instance[0].desc_en ? 4 : 1);
    // temp index
    uint8_t i;

    // move through list of characteristics
    // when remainer of handle is smaller than number of enabled descriptors
    // can find exact match
    if (handle < enabled_desc_nb)
    {
        if (handle != 0)
        {
            // first Characteristic have the limited selection of descriptors
            flag = (1<<ENVS_IDX_DESC_CCC)|(1<<ENVS_IDX_CHAR_CHAR)|(1<<ENVS_IDX_CHAR_VAL);
            // remove the Service Descriptor handle
            handle--;
        }
        else
        {
            flag = 0;
        }
    }
    else
    {
        handle -= enabled_desc_nb;

        // move through the list of Characteristics
        while (handle)
        {
            res_char++;
            cfg = p_envs_env->instance[res_char].desc_en;
            enabled_desc_nb = envs_count_enabled_desc(&cfg);
            // When we count to be smaller than the number of enabled descriptors
            // got the Characteristic
            if (handle <= enabled_desc_nb)
            {
                flag = cfg;
                // add must have attributes:  CHAR declaration,VALUE
                flag <<= 2;
                flag |= (1<<ENVS_IDX_CHAR_CHAR)|(1<<ENVS_IDX_CHAR_VAL);
                // ENVS_IDX_DESC_EXT is the highest bit enabled
                if (cfg & ENVS_DESC_EN_USR_DESC)
                {
                    flag |= (1<<ENVS_IDX_DESC_EXT);
                }
                break;
            }
            handle -=  enabled_desc_nb;
        }
    }

    // got the Characteristic
    *p_es_char_id = p_envs_env->instance[res_char].es_char_id;
    // find instance in the list of the Characteristics
    res_inst = 0;
    // Count all the same type before the current in list
    for (i = 1; i < res_char; i++)
    {
        if (*p_es_char_id == p_envs_env->instance[i].es_char_id)
        {
            res_inst++;
        }
    }

    *p_es_char_inst = res_inst;

    // Find Descriptor Idx by counting the enabled descriptors for the found Characteristic
    // cfg keeps the bits for enabled descriptors
    res_desc = 0;

    while (handle != 0)
    {
        res_desc++;
        if ((flag & 1) == 1)
        {
            handle--;
            if (handle == 0)
            {
                //found position
                break;
            }
        }
        flag >>= 1; //shift by 1
    }

    *p_desc_idx = res_desc;
    *p_char_idx = res_char;
}

/**
 ****************************************************************************************
 * @brief  This function fully manages notification of measurement and vector
 ****************************************************************************************
 */
__STATIC void envs_exe_operation(envs_env_t* p_envs_env)
{
    if(!p_envs_env->in_exe_op)
    {
        p_envs_env->in_exe_op = true;

        while(!co_list_is_empty(&(p_envs_env->wait_queue)) && !(p_envs_env->op_ongoing))
        {
            const envs_cb_t* p_cb = (const envs_cb_t*) p_envs_env->prf_env.p_cb;
            uint16_t status = GAP_ERR_NO_ERROR;
            co_buf_t* p_buf = (co_buf_t*) co_list_pop_front(&(p_envs_env->wait_queue));
            envs_buf_meta_t* p_meta = (envs_buf_meta_t*) co_buf_metadata(p_buf);
            uint8_t conidx    = p_meta->conidx;
            uint8_t operation = p_meta->operation;

            status = gatt_srv_event_send(conidx, p_envs_env->user_lid, p_meta->operation, p_meta->evt_type,
                                         p_meta->hdl, p_buf);
            co_buf_release(p_buf);

            if(status == GAP_ERR_NO_ERROR)
            {
                p_envs_env->op_ongoing = true;
            }
            else
            {
                switch(operation)
                {
                    case ENVS_INDICATE_OP_CODE:
                    {
                        p_cb->cb_indicate_cmp(conidx, status);
                    } break;
                    case ENVS_NOTIFY_OP_CODE:
                    {
                        p_cb->cb_notify_cmp(conidx, status);
                    } break;

                    default: { ASSERT_ERR(0); } break;
                }
            }
        }

        p_envs_env->in_exe_op = false;
    }
}

/**
 ****************************************************************************************
 * @brief Unpack Trigger data
 *
 * @param[in]   p_buf       Input buffer
 * @param[in]   es_char_id  Characteristic Type idx see enum #envp_es_char
 * @param[in]   p_range     Value range available
 * @param[out]  p_trigger   Unpacked trigger structure
 *
 * @return Unpacking status
 ****************************************************************************************
 */
__STATIC uint16_t envs_unpack_trigger(co_buf_t* p_buf, uint8_t es_char_id, union envp_range* p_range, envp_trigger_t* p_trigger)
{
    uint16_t status = PRF_ERR_INVALID_PARAM;

    memset(p_trigger, 0, sizeof(envp_trigger_t));

    do
    {
        p_trigger->condition = co_buf_data(p_buf)[0];
        if(co_buf_head_release(p_buf, 1) != CO_BUF_ERR_NO_ERROR) { break; }

        // exit due to invalid params
        if(p_trigger->condition >= ENVP_TRIG_RESERVED) { break; }

        status = GAP_ERR_NO_ERROR;

        /*
         * Trigger Conditions
         *     0 - Inactive
         *     1 - Timer Based                        - ENVP_TRIG_INTERVAL_FIXED
         *     2 - Timer Based                        - ENVP_TRIG_INTERVAL_NO_LESS_THAN
         *     3 - Neither Timer or measurement based - ENVP_TRIG_PREV_VALUE_DIFFERENT
         *
         *     Note 4-9 are Measurement Value based triggers
         *     4 - Measurement Based                  - ENVP_TRIG_VALUE_LESS
         *     ...
         *     9 - Measurement Based                  - ENVP_TRIG_VALUE_NOT_EQUAL
         */

        // If NOT a measurement based Trigger. - if (condition < 4 )
        if (p_trigger->condition < ENVP_TRIG_VALUE_LESS)
        {
            // If a Timer based trigger (condition == 1 or condition == 2) - setup the timer value
            if ((p_trigger->condition == ENVP_TRIG_INTERVAL_FIXED) ||
                (p_trigger->condition == ENVP_TRIG_INTERVAL_NO_LESS_THAN))
            {
                // Add a uint24 to the data field.
                p_trigger->time_based_trigger = co_btoh24(co_read24p(co_buf_data(p_buf)));
                co_buf_head_release(p_buf, 3);
            }
        }
        // If measurement based trigger  - if condition is between 4 & 9.
        else if ((p_trigger->condition > ENVP_TRIG_PREV_VALUE_DIFFERENT) &&
                 (p_trigger->condition < ENVP_TRIG_RESERVED))
        {

            // In this case the size of the operand is dependant on the size of the
            // value field of the measurement characteristic.
            switch(es_char_id)
            {
                // First all the uint16 chars
                case ENVP_ES_APRNT_WIND_DIRECTION_CHAR:
                case ENVP_ES_APRNT_WIND_SPEED_CHAR:
                case ENVP_ES_HUMIDITY_CHAR:
                case ENVP_ES_IRRADIANCE_CHAR:
                case ENVP_ES_RAINFALL_CHAR:
                case ENVP_ES_TRUE_WIND_DIR_CHAR:
                case ENVP_ES_TRUE_WIND_SPEED_CHAR:
                case ENVP_ES_MAGN_DECLINE_CHAR:
                {
                    uint16_t value_u16 = co_btohs(co_read16p(co_buf_data(p_buf)));

                    if ((value_u16 < p_range->u16.lower) || (value_u16 > p_range->u16.upper))
                    {
                        status = PRF_OUT_OF_RANGE;
                    }
                    else
                    {
                        p_trigger->meas_value_trigger.humidity = value_u16;
                    }
                } break;

                // uint24
                case ENVP_ES_POLLEN_CONC_CHAR:
                {
                    uint32_t value_u24 = co_btoh24(co_read24p(co_buf_data(p_buf)));

                    if ((value_u24 < p_range->u32.lower) || (value_u24 > p_range->u32.upper))
                    {
                        status = PRF_OUT_OF_RANGE;
                    }
                    else
                    {
                        p_trigger->meas_value_trigger.pollen_concentration = value_u24;
                    }
                } break;

                // now the uint32 chars
                case ENVP_ES_PRESSURE_CHAR:
                {
                    uint32_t value_u32 = co_btohl(co_read32p(co_buf_data(p_buf)));

                    if ((value_u32 < p_range->u32.lower) || (value_u32 > p_range->u32.upper))
                    {
                        status = PRF_OUT_OF_RANGE;
                    }
                    else
                    {
                        p_trigger->meas_value_trigger.pressure = value_u32;
                    }
                } break;

                // the int24 chars
                case ENVP_ES_ELEVATION_CHAR:
                {
                    int32_t value_s24 = ((((int32_t) co_btoh24(co_read24p(co_buf_data(p_buf)))) << 8) >> 8);

                    if ((value_s24 < p_range->s32.lower) || (value_s24 > p_range->s32.upper))
                    {
                        status = PRF_OUT_OF_RANGE;
                    }
                    else
                    {
                        p_trigger->meas_value_trigger.elevation = value_s24;
                    }
                } break;

                // int16
                case ENVP_ES_TEMPERATURE_CHAR:
                {
                    int16_t value_s16 = (int16_t) co_btohs(co_read16p(co_buf_data(p_buf)));

                    if ((value_s16 < p_range->s16.lower) || (value_s16 > p_range->s16.upper))
                    {
                        status = PRF_OUT_OF_RANGE;
                    }
                    else
                    {
                        p_trigger->meas_value_trigger.temperature = value_s16;
                    }
                } break;

                // uint8
                case ENVP_ES_GUST_FACTOR_CHAR:
                case ENVP_ES_UV_INDEX_CHAR:
                case ENVP_ES_BAR_PRES_TREND_CHAR:
                {
                    uint8_t value_u8 = co_buf_data(p_buf)[0];

                    if ((value_u8 < p_range->u8.lower) || (value_u8 > p_range->u8.upper))
                    {
                        status = PRF_OUT_OF_RANGE;
                    }
                    else
                    {
                        p_trigger->meas_value_trigger.gust_factor = value_u8;
                    }
                } break;

                // int8
                case ENVP_ES_DEW_POINT_CHAR:
                case ENVP_ES_HEAT_INDEX_CHAR:
                case ENVP_ES_WIND_CHILL_CHAR:
                {
                    int8_t value_s8 = (int8_t) co_buf_data(p_buf)[0];

                    if ((value_s8 < p_range->s8.lower) || (value_s8 > p_range->s8.upper))
                    {
                        status = PRF_OUT_OF_RANGE;
                    }
                    else
                    {
                        p_trigger->meas_value_trigger.dew_point = value_s8;
                    }
                } break;

                default:
                {
                    status = ATT_ERR_ATTRIBUTE_NOT_FOUND;
                } break;
            }
        }
    } while(0);

    return (status);
}


/**
 ****************************************************************************************
 * @brief Pack Characteristic value depending on Characteristic type
 *
 * @param[in]         es_char_type   Characteristic Type
 * @param[in]         p_value        Input value to pack
 * @param[in]         p_buf          Pointer to outpur buffer
 *
 * @return status of the operation.
 ****************************************************************************************
 */
uint16_t envs_pack_es_char_value(int8_t es_char_type, const union envp_val_char *p_value, co_buf_t* p_buf)
{
    uint8_t status = GAP_ERR_NO_ERROR;

    switch(es_char_type)
    {
        // First all the uint16 chars
        case ENVP_ES_APRNT_WIND_DIRECTION_CHAR:
        {
            co_write16p(co_buf_tail(p_buf), co_htobs(p_value->apparent_wind_direction));
            co_buf_tail_reserve(p_buf, 2);
        } break;

        case ENVP_ES_APRNT_WIND_SPEED_CHAR:
        {
            co_write16p(co_buf_tail(p_buf), co_htobs(p_value->apparent_wind_speed));
            co_buf_tail_reserve(p_buf, 2);
        } break;

        case ENVP_ES_HUMIDITY_CHAR:
        {
            co_write16p(co_buf_tail(p_buf), co_htobs(p_value->humidity));
            co_buf_tail_reserve(p_buf, 2);
        } break;

        case ENVP_ES_IRRADIANCE_CHAR:
        {
            co_write16p(co_buf_tail(p_buf), co_htobs(p_value->irradiance));
            co_buf_tail_reserve(p_buf, 2);
        } break;

        case ENVP_ES_RAINFALL_CHAR:
        {
            co_write16p(co_buf_tail(p_buf), co_htobs(p_value->rainfall));
            co_buf_tail_reserve(p_buf, 2);
        } break;

        case ENVP_ES_TRUE_WIND_DIR_CHAR:
        {
            co_write16p(co_buf_tail(p_buf), co_htobs(p_value->true_wind_direction));
            co_buf_tail_reserve(p_buf, 2);
        } break;

        case ENVP_ES_TRUE_WIND_SPEED_CHAR:
        {
            co_write16p(co_buf_tail(p_buf), co_htobs(p_value->true_wind_speed));
            co_buf_tail_reserve(p_buf, 2);
        } break;

        case ENVP_ES_MAGN_DECLINE_CHAR:
        {
            co_write16p(co_buf_tail(p_buf), co_htobs(p_value->magnetic_declination));
            co_buf_tail_reserve(p_buf, 2);
        } break;

        // uint24
        case ENVP_ES_POLLEN_CONC_CHAR:
        {
            co_write24p(co_buf_tail(p_buf), co_htob24(p_value->pollen_concentration));
            co_buf_tail_reserve(p_buf, 3);
        } break;

        // now the uint32 chars
        case ENVP_ES_PRESSURE_CHAR:
        {
            co_write32p(co_buf_tail(p_buf), co_htobl(p_value->pressure));
            co_buf_tail_reserve(p_buf, 4);
        } break;

        // the int24 chars
        case ENVP_ES_ELEVATION_CHAR:
        {
            co_write24p(co_buf_tail(p_buf), co_htob24(p_value->elevation));
            co_buf_tail_reserve(p_buf, 3);
        } break;

         // int16
         case ENVP_ES_TEMPERATURE_CHAR:
         {
            co_write16p(co_buf_tail(p_buf), co_htobs(p_value->temperature));
            co_buf_tail_reserve(p_buf, 2);
         } break;

        // uint8
        case ENVP_ES_GUST_FACTOR_CHAR:
        {
            co_buf_tail(p_buf)[0] = p_value->gust_factor;
            co_buf_tail_reserve(p_buf, 1);
        } break;

        case ENVP_ES_UV_INDEX_CHAR:
        {
            co_buf_tail(p_buf)[0] = p_value->uv_index;
            co_buf_tail_reserve(p_buf, 1);
        } break;

        case ENVP_ES_BAR_PRES_TREND_CHAR:
        {
            co_buf_tail(p_buf)[0] = p_value->barometric_pressure_trend;
            co_buf_tail_reserve(p_buf, 1);
        } break;

        // int8
        case ENVP_ES_DEW_POINT_CHAR:
        {
            co_buf_tail(p_buf)[0] = p_value->dew_point;
            co_buf_tail_reserve(p_buf, 1);
        } break;

        case ENVP_ES_HEAT_INDEX_CHAR:
        {
            co_buf_tail(p_buf)[0] = p_value->heat_index;
            co_buf_tail_reserve(p_buf, 1);
        } break;

        case ENVP_ES_WIND_CHILL_CHAR:
        {
            co_buf_tail(p_buf)[0] = p_value->wind_chill;
            co_buf_tail_reserve(p_buf, 1);
        } break;

        case ENVP_ES_MAGN_FLUX_2D_CHAR:
        {
            co_write16p(co_buf_tail(p_buf), co_htobs(p_value->mag_flux_dens_2d.x_axis));
            co_buf_tail_reserve(p_buf, 2);
            co_write16p(co_buf_tail(p_buf), co_htobs(p_value->mag_flux_dens_2d.y_axis));
            co_buf_tail_reserve(p_buf, 2);
        } break;

        case ENVP_ES_MAGN_FLUX_3D_CHAR:
        {
            co_write16p(co_buf_tail(p_buf), co_htobs(p_value->mag_flux_dens_3d.x_axis));
            co_buf_tail_reserve(p_buf, 2);
            co_write16p(co_buf_tail(p_buf), co_htobs(p_value->mag_flux_dens_3d.y_axis));
            co_buf_tail_reserve(p_buf, 2);
            co_write16p(co_buf_tail(p_buf), co_htobs(p_value->mag_flux_dens_3d.z_axis));
            co_buf_tail_reserve(p_buf, 2);
        } break;

        default:
        {
            status = PRF_ERR_INVALID_PARAM;
        } break;
    }

    return (status);
}



/**
 ****************************************************************************************
 * @brief Send back status of write operation
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
__STATIC uint16_t envs_write_cfm(uint8_t conidx, uint32_t token, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst)
{
    envs_env_t* p_envs_env = PRF_ENV_GET(ENVS, envs);

    if((es_char_id != GETF(token, ENVS_TOKEN_ES_CHAR_ID)) || (es_char_inst != GETF(token, ENVS_TOKEN_ES_CHAR_INST)))
    {
       status = GAP_ERR_INVALID_PARAM;
    }
    else if(p_envs_env != NULL)
    {
        // Return buffer that contains report data requested by peer device
        status = gatt_srv_att_val_set_cfm(conidx, p_envs_env->user_lid, GETF(token, ENVS_TOKEN_GATT_TOKEN), status);
    }
    else
    {
        status = PRF_ERR_REQ_DISALLOWED;
    }

    return (status);
}

/*
 * GATT USER SERVICE HANDLERS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief This function is called when peer want to read local attribute database value.
 *
 *        #gatt_srv_att_read_get_cfm shall be called to provide attribute value
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] token         Procedure token that must be returned in confirmation function
 * @param[in] hdl           Attribute handle
 * @param[in] offset        Data offset
 * @param[in] max_length    Maximum data length to return
 ****************************************************************************************
 */
__STATIC void envs_cb_att_read_get(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t hdl, uint16_t offset,
                                   uint16_t max_length)
{
    co_buf_t* p_buf = NULL;
    envs_env_t *p_envs_env = PRF_ENV_GET(ENVS, envs);
    uint16_t  status      = GAP_ERR_NO_ERROR;
    bool      send_cfm    = false;

    if(p_envs_env == NULL)
    {
        status = PRF_APP_ERROR;
        send_cfm = true;
    }
    else
    {
        uint32_t envs_token = 0;
        const envs_cb_t* p_cb  = (const envs_cb_t*) p_envs_env->prf_env.p_cb;
        uint8_t es_char_id;
        uint8_t es_char_inst;
        uint8_t desc_idx;
        uint8_t char_idx;

        // retrieve handle information
        envs_handle_2_char(hdl, &es_char_id, &es_char_inst, &desc_idx, &char_idx);


        SETF(envs_token, ENVS_TOKEN_GATT_TOKEN,   token);
        SETF(envs_token, ENVS_TOKEN_ES_CHAR_ID,   es_char_id);
        SETF(envs_token, ENVS_TOKEN_ES_CHAR_INST, es_char_inst);

        switch (desc_idx)
        {
            // Characteristic value
            case ENVS_IDX_CHAR_VAL:
            {
                // request read of value
                p_cb->cb_read_value_req(conidx, envs_token, es_char_id, es_char_inst);
            } break;
            // Trigger Setting; if_notify_supported
            case ENVS_IDX_DESC_TRIG_1:
            // Trigger Setting; if_notify_supported
            case ENVS_IDX_DESC_TRIG_2:
            // Trigger Setting; if_notify_supported
            case ENVS_IDX_DESC_TRIG_3:
            {
                uint8_t trigger_idx = desc_idx - ENVS_IDX_DESC_TRIG_1;
                SETF(envs_token, ENVS_TOKEN_TRIGGER_IDX, trigger_idx);
                // request read trigger value
                p_cb->cb_read_trigger_req(conidx, envs_token, es_char_id, es_char_inst, trigger_idx);
            } break;
            // Configuration
            case ENVS_IDX_DESC_CFG:
            {
                // read descriptor configuration
                p_cb->cb_read_config_req(conidx, envs_token, es_char_id, es_char_inst);
            } break;
            // Client Characteristic Configuration
            case ENVS_IDX_DESC_CCC:
            {
                // read Client Characteristic configuration
                p_cb->cb_read_ccc_req(conidx, envs_token, es_char_id, es_char_inst);
            } break;

            // User Description
            case ENVS_IDX_DESC_USR_DESC:
            {
                // read User Description
                p_cb->cb_read_user_desc_req(conidx, envs_token, es_char_id, es_char_inst, offset, max_length);
            } break;

            // Measurement info
            case ENVS_IDX_DESC_MEAS:
            {
                // read Measurement info
                p_cb->cb_read_meas_req(conidx, envs_token, es_char_id, es_char_inst);
            } break;

            // value range
            case ENVS_IDX_DESC_RANGE:
            {
                union envp_range* p_range = &(p_envs_env->instance[char_idx].range);
                if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, 0, L2CAP_LE_MTU_MIN + GATT_BUFFER_TAIL_LEN) != CO_BUF_ERR_NO_ERROR)
                {
                    status = ATT_ERR_INSUFF_RESOURCE;
                    break;
                }


                // In this case the size of the operand is dependant on the size of the
                // value field of the measurement characteristic.
                switch(es_char_id)
                {
                    // First all the uint16 chars
                    case ENVP_ES_APRNT_WIND_DIRECTION_CHAR:
                    case ENVP_ES_APRNT_WIND_SPEED_CHAR:
                    case ENVP_ES_HUMIDITY_CHAR:
                    case ENVP_ES_IRRADIANCE_CHAR:
                    case ENVP_ES_RAINFALL_CHAR:
                    case ENVP_ES_TRUE_WIND_DIR_CHAR:
                    case ENVP_ES_TRUE_WIND_SPEED_CHAR:
                    case ENVP_ES_MAGN_DECLINE_CHAR:
                    {
                        co_write16p(co_buf_tail(p_buf), co_htobs(p_range->u16.lower));
                        co_buf_tail_reserve(p_buf, 2);
                        co_write16p(co_buf_tail(p_buf), co_htobs(p_range->u16.upper));
                        co_buf_tail_reserve(p_buf, 2);
                    } break;

                    // uint24
                    case ENVP_ES_POLLEN_CONC_CHAR:
                    {
                        co_write24p(co_buf_tail(p_buf), co_htob24(p_range->u32.lower));
                        co_buf_tail_reserve(p_buf, 3);
                        co_write24p(co_buf_tail(p_buf), co_htob24(p_range->u32.upper));
                        co_buf_tail_reserve(p_buf, 3);
                    } break;

                    // now the uint32 chars
                    case ENVP_ES_PRESSURE_CHAR:
                    {
                        co_write32p(co_buf_tail(p_buf), co_htobl(p_range->u32.lower));
                        co_buf_tail_reserve(p_buf, 4);
                        co_write32p(co_buf_tail(p_buf), co_htobl(p_range->u32.upper));
                        co_buf_tail_reserve(p_buf, 4);
                    } break;

                    // the int24 chars
                    case ENVP_ES_ELEVATION_CHAR:
                    {
                        co_write24p(co_buf_tail(p_buf), co_htob24(p_range->s32.lower));
                        co_buf_tail_reserve(p_buf, 3);
                        co_write24p(co_buf_tail(p_buf), co_htob24(p_range->s32.upper));
                        co_buf_tail_reserve(p_buf, 3);
                    } break;

                    // int16
                    case ENVP_ES_TEMPERATURE_CHAR:
                    case ENVP_ES_MAGN_FLUX_2D_CHAR:
                    case ENVP_ES_MAGN_FLUX_3D_CHAR:
                    {
                        co_write16p(co_buf_tail(p_buf), co_htobs(p_range->s16.lower));
                        co_buf_tail_reserve(p_buf, 2);
                        co_write16p(co_buf_tail(p_buf), co_htobs(p_range->s16.upper));
                        co_buf_tail_reserve(p_buf, 2);
                    } break;

                    // uint8
                    case ENVP_ES_GUST_FACTOR_CHAR:
                    case ENVP_ES_UV_INDEX_CHAR:
                    case ENVP_ES_BAR_PRES_TREND_CHAR:
                    {
                        co_buf_tail(p_buf)[0] = p_range->u8.lower;
                        co_buf_tail_reserve(p_buf, 1);
                        co_buf_tail(p_buf)[0] = p_range->u8.upper;
                        co_buf_tail_reserve(p_buf, 1);
                    } break;

                    // int8
                    case ENVP_ES_DEW_POINT_CHAR:
                    case ENVP_ES_HEAT_INDEX_CHAR:
                    case ENVP_ES_WIND_CHILL_CHAR:
                    {
                        co_buf_tail(p_buf)[0] = p_range->s8.lower;
                        co_buf_tail_reserve(p_buf, 1);
                        co_buf_tail(p_buf)[0] = p_range->s8.upper;
                        co_buf_tail_reserve(p_buf, 1);
                    } break;

                    default:
                    {
                        status = ATT_ERR_ATTRIBUTE_NOT_FOUND;
                    } break;
                }

                send_cfm = true;
            } break;

            default:
            {
                status    = ATT_ERR_REQUEST_NOT_SUPPORTED;
                send_cfm = true;
            } break;
        }
    }

    if(send_cfm)
    {
        uint16_t data_len = 0;
        if(p_buf != NULL)
        {
            data_len = co_buf_data_len(p_buf);
        }

        // send back result of read data
        gatt_srv_att_read_get_cfm(conidx, user_lid, token, status, data_len, p_buf);

        if(p_buf != NULL)
        {
            co_buf_release(p_buf);
        }
    }
}


/**
 ****************************************************************************************
 * @brief This function is called during a write procedure to modify attribute handle.
 *
 *        #gatt_srv_att_val_set_cfm shall be called to accept or reject attribute
 *        update.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] token         Procedure token that must be returned in confirmation function
 * @param[in] hdl           Attribute handle
 * @param[in] offset        Value offset
 * @param[in] p_data        Pointer to buffer that contains data to write starting from offset
 ****************************************************************************************
 */
__STATIC void envs_cb_att_val_set(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t hdl, uint16_t offset,
                                  co_buf_t* p_data)
{
    bool send_cfm = false;
    envs_env_t *p_envs_env = PRF_ENV_GET(ENVS, envs);
    uint16_t  status      = PRF_APP_ERROR;

    if(p_envs_env != NULL)
    {
        uint32_t envs_token = 0;
        const envs_cb_t* p_cb  = (const envs_cb_t*) p_envs_env->prf_env.p_cb;
        uint8_t es_char_id;
        uint8_t es_char_inst;
        uint8_t desc_idx;
        uint8_t char_idx;

        // retrieve handle information
        envs_handle_2_char(hdl, &es_char_id, &es_char_inst, &desc_idx, &char_idx);

        SETF(envs_token, ENVS_TOKEN_GATT_TOKEN,   token);
        SETF(envs_token, ENVS_TOKEN_ES_CHAR_ID,   es_char_id);
        SETF(envs_token, ENVS_TOKEN_ES_CHAR_INST, es_char_inst);

        status = GAP_ERR_NO_ERROR;

        switch (desc_idx)
        {
            // Trigger Setting; if_notify_supported
            case ENVS_IDX_DESC_TRIG_1:
            // Trigger Setting; if_notify_supported
            case ENVS_IDX_DESC_TRIG_2:
            // Trigger Setting; if_notify_supported
            case ENVS_IDX_DESC_TRIG_3:
            {
                envp_trigger_t trigger;
                uint8_t trigger_idx = desc_idx - ENVS_IDX_DESC_TRIG_1;

                status = envs_unpack_trigger(p_data, es_char_id, &(p_envs_env->instance[char_idx].range), &trigger);
                if(status != GAP_ERR_NO_ERROR)
                {
                    send_cfm = true;
                    break;
                }

                SETF(envs_token, ENVS_TOKEN_TRIGGER_IDX, trigger_idx);
                // request read trigger value
                p_cb->cb_write_trigger_req(conidx, envs_token, es_char_id, es_char_inst, trigger_idx, &trigger);
            } break;
            // Configuration
            case ENVS_IDX_DESC_CFG:
            {
                uint8_t trigger_logic = co_buf_data(p_data)[0];

                // check if trigger logic is in valid range
                if(trigger_logic > ENVP_TRIG_BOOLEAN_OR)
                {
                    send_cfm = true;
                    status = PRF_APP_ERROR;
                    break;
                }

                // read descriptor configuration
                p_cb->cb_write_config_req(conidx, envs_token, es_char_id, es_char_inst, trigger_logic);
            } break;
            // Client Characteristic Configuration
            case ENVS_IDX_DESC_CCC:
            {
                uint16_t ccc = co_btohs(co_read16p(co_buf_data(p_data)));

                // read Client Characteristic configuration
                p_cb->cb_write_ccc_req(conidx, envs_token, es_char_id, es_char_inst, ccc);
            } break;

            // User Description
            case ENVS_IDX_DESC_USR_DESC:
            {
                // read User Description
                p_cb->cb_write_user_desc_req(conidx, envs_token, es_char_id, es_char_inst, p_data);
            } break;

            default:
            {
                status    = ATT_ERR_REQUEST_NOT_SUPPORTED;
                send_cfm = true;
            } break;
        }
    }

    if(send_cfm)
    {
        gatt_srv_att_val_set_cfm(conidx, user_lid, token, status);
    }
}

/**
 ****************************************************************************************
 * @brief This function is called when GATT server user has initiated event send to peer
 *        device or if an error occurs.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] status        Status of the procedure (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC void envs_cb_event_sent(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status)
{
    // Consider job done
    envs_env_t *p_envs_env = PRF_ENV_GET(ENVS, envs);
    if(p_envs_env != NULL)
    {
        const envs_cb_t* p_cb  = (const envs_cb_t*) p_envs_env->prf_env.p_cb;
        p_envs_env->op_ongoing = false;

        switch(dummy)
        {
            case ENVS_INDICATE_OP_CODE:
            {
                p_cb->cb_indicate_cmp(conidx, status);
            } break;
            case ENVS_NOTIFY_OP_CODE:
            {
                p_cb->cb_notify_cmp(conidx, status);
            } break;
            default: { /* Nothing to do */ } break;
        }

        // continue operation execution
        envs_exe_operation(p_envs_env);
    }
}

/// Service callback hander
__STATIC const gatt_srv_cb_t envs_cb =
{
        .cb_event_sent    = envs_cb_event_sent,
        .cb_att_read_get  = envs_cb_att_read_get,
        .cb_att_event_get = NULL,
        .cb_att_info_get  = NULL,
        .cb_att_val_set   = envs_cb_att_val_set,
};

/*
 * PROFILE NATIVE HANDLERS
 ****************************************************************************************
 */


uint16_t envs_range_upd(uint8_t es_char_id, uint8_t es_char_inst, const union envp_range* p_range)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    envs_env_t* p_envs_env = PRF_ENV_GET(ENVS, envs);

    if(p_range == NULL)
    {
       status = GAP_ERR_INVALID_PARAM;
    }
    else if(p_envs_env == NULL)
    {
        status = PRF_ERR_REQ_DISALLOWED;
    }
    else
    {
        uint8_t i;
        uint8_t char_idx;

        // First descriptor is of special case
        // containing only Value, CCC descriptors
        if (es_char_id == ENVP_ES_DESCRIPTOR_VALUE_CHANGED_CHAR)
        {
            status = GAP_ERR_INVALID_PARAM;
        }
        else
        {
            char_idx = 0;

            // Find position in the list of the Characteristic
            // go through the list finding the repeat(instance) of the given Characteristic
            for (i = 1; i < p_envs_env->nb_chars; i++)
            {
                if (es_char_id == p_envs_env->instance[i].es_char_id)
                {
                    if (es_char_inst)
                    {
                        es_char_inst--;
                    }
                    else
                    {
                        char_idx = i;
                        break;
                    }
                }
            }

            // not found
            if (char_idx == 0)
            {
                status = GAP_ERR_NOT_FOUND;
            }
            else
            {
                // copy new range
                memcpy(&(p_envs_env->instance[char_idx].range), p_range, sizeof(union envp_range));
            }
        }
    } while(0);

    return (status);
}

uint16_t envs_indicate(uint8_t conidx, uint8_t es_char_id, uint16_t ind_flag)
{
    envs_env_t* p_envs_env = PRF_ENV_GET(ENVS, envs);
    uint16_t status = PRF_ERR_REQ_DISALLOWED;

    if(es_char_id >= ENVP_ES_CHAR_MAX)
    {
        status = PRF_ERR_INVALID_PARAM;
    }
    else if(p_envs_env != NULL)
    {
        uint16_t  hdl;
        co_buf_t* p_buf;

        status = envs_char_2_handle(ENVP_ES_DESCRIPTOR_VALUE_CHANGED_CHAR, 0, ENVS_IDX_CHAR_VAL, &hdl);
        if(status == GAP_ERR_NO_ERROR)
        {
            // allocate buffer for indication
            if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, 0, L2CAP_LE_MTU_MIN + GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
            {
                envs_buf_meta_t* p_buf_meta = (envs_buf_meta_t*) co_buf_metadata(p_buf);
                p_buf_meta->operation = ENVS_INDICATE_OP_CODE;
                p_buf_meta->conidx    = conidx;
                p_buf_meta->hdl       = hdl;
                p_buf_meta->evt_type  = GATT_INDICATE;

                // pack measured value in database
                co_write16p(co_buf_tail(p_buf), co_htobs(ind_flag));
                co_buf_tail_reserve(p_buf, 2);
                // UUID of the Characteristic
                co_write16p(co_buf_tail(p_buf), envs_att_db_chars[es_char_id].uuid16);
                co_buf_tail_reserve(p_buf, 2);

                // put event on wait queue
                co_list_push_back(&(p_envs_env->wait_queue), &(p_buf->hdr));
                // execute operation
                envs_exe_operation(p_envs_env);
                status = GAP_ERR_NO_ERROR;
            }
            else
            {
                status = GAP_ERR_INSUFF_RESOURCES;
            }
        }
    }

    return (status);
}

uint16_t envs_notify(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst, const union envp_val_char* p_value)
{

    envs_env_t* p_envs_env = PRF_ENV_GET(ENVS, envs);
    uint16_t status = PRF_ERR_REQ_DISALLOWED;

    if(p_value == NULL)
    {
        status = PRF_ERR_INVALID_PARAM;
    }
    else if(p_envs_env != NULL)
    {
        uint16_t  hdl;

        status = envs_char_2_handle(es_char_id, es_char_inst, ENVS_IDX_CHAR_VAL, &hdl);
        if(status == GAP_ERR_NO_ERROR)
        {
            co_buf_t* p_buf;
            // allocate buffer for indication
            if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, 0, L2CAP_LE_MTU_MIN + GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
            {
                envs_buf_meta_t* p_buf_meta = (envs_buf_meta_t*) co_buf_metadata(p_buf);
                p_buf_meta->operation = ENVS_NOTIFY_OP_CODE;
                p_buf_meta->conidx    = conidx;
                p_buf_meta->hdl       = hdl;
                p_buf_meta->evt_type  = GATT_NOTIFY;

                // First Determine if Valid Char and Pack Data Field based on char value characteristics
                status = envs_pack_es_char_value(es_char_id, p_value, p_buf);

                if(status == GAP_ERR_NO_ERROR)
                {
                    // put event on wait queue
                    co_list_push_back(&(p_envs_env->wait_queue), &(p_buf->hdr));
                    // execute operation
                    envs_exe_operation(p_envs_env);
                }
                else
                {
                    co_buf_release(p_buf);
                }
            }
            else
            {
                status = GAP_ERR_INSUFF_RESOURCES;
            }
        }
    }

    return (status);
}

uint16_t envs_write_ccc_cfm(uint8_t conidx, uint32_t token, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst)
{
    return (envs_write_cfm(conidx, token, status, es_char_id, es_char_inst));
}

uint16_t envs_write_trigger_cfm(uint8_t conidx, uint32_t token, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst)
{
    return (envs_write_cfm(conidx, token, status, es_char_id, es_char_inst));
}

uint16_t envs_write_config_cfm(uint8_t conidx, uint32_t token, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst)
{
    return (envs_write_cfm(conidx, token, status, es_char_id, es_char_inst));
}

uint16_t envs_write_user_desc_cfm(uint8_t conidx, uint32_t token, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst)
{
    return (envs_write_cfm(conidx, token, status, es_char_id, es_char_inst));
}

uint16_t envs_read_value_cfm(uint8_t conidx, uint32_t token, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                             const union envp_val_char* p_value)
{
    envs_env_t* p_envs_env = PRF_ENV_GET(ENVS, envs);

    if((es_char_id != GETF(token, ENVS_TOKEN_ES_CHAR_ID)) || (es_char_inst != GETF(token, ENVS_TOKEN_ES_CHAR_INST)))
    {
       status = GAP_ERR_INVALID_PARAM;
    }
    else if((status == GAP_ERR_NO_ERROR) && (p_value == NULL))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else if(p_envs_env != NULL)
    {
        co_buf_t* p_buf = NULL;
        uint16_t  data_len = 0;

        if(status == GAP_ERR_NO_ERROR)
        {
            if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, 0, L2CAP_LE_MTU_MIN + GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
            {
                status = envs_pack_es_char_value(es_char_id, p_value, p_buf);
                data_len = co_buf_data_len(p_buf);
            }
            else
            {
                status = ATT_ERR_INSUFF_RESOURCE;
            }
        }

        // Return buffer that contains report data requested by peer device
        status = gatt_srv_att_read_get_cfm(conidx, p_envs_env->user_lid, GETF(token, ENVS_TOKEN_GATT_TOKEN),
                                           status, data_len, p_buf);
        if(p_buf != NULL)
        {
            co_buf_release(p_buf);
        }
    }
    else
    {
        status = PRF_ERR_REQ_DISALLOWED;
    }

    return (status);

}

uint16_t envs_read_ccc_cfm(uint8_t conidx, uint32_t token, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                           uint16_t ccc)
{
    envs_env_t* p_envs_env = PRF_ENV_GET(ENVS, envs);

    if((es_char_id != GETF(token, ENVS_TOKEN_ES_CHAR_ID)) || (es_char_inst != GETF(token, ENVS_TOKEN_ES_CHAR_INST)))
    {
       status = GAP_ERR_INVALID_PARAM;
    }
    else if(p_envs_env != NULL)
    {
        co_buf_t* p_buf = NULL;
        uint16_t  data_len = 0;

        if(status == GAP_ERR_NO_ERROR)
        {
            if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, 0, L2CAP_LE_MTU_MIN + GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
            {
                co_write16p(co_buf_tail(p_buf), co_htobs(ccc));
                co_buf_tail_reserve(p_buf, 2);
                data_len = co_buf_data_len(p_buf);
            }
            else
            {
                status = ATT_ERR_INSUFF_RESOURCE;
            }
        }

        // Return buffer that contains report data requested by peer device
        status = gatt_srv_att_read_get_cfm(conidx, p_envs_env->user_lid, GETF(token, ENVS_TOKEN_GATT_TOKEN),
                                           status, data_len, p_buf);
        if(p_buf != NULL)
        {
            co_buf_release(p_buf);
        }
    }
    else
    {
        status = PRF_ERR_REQ_DISALLOWED;
    }

    return (status);
}

uint16_t envs_read_trigger_cfm(uint8_t conidx, uint32_t token, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                               uint8_t trigger_idx, const envp_trigger_t* p_trigger)
{
    envs_env_t* p_envs_env = PRF_ENV_GET(ENVS, envs);

    if(   (es_char_id != GETF(token, ENVS_TOKEN_ES_CHAR_ID)) || (es_char_inst != GETF(token, ENVS_TOKEN_ES_CHAR_INST))
       || (trigger_idx != GETF(token, ENVS_TOKEN_TRIGGER_IDX)))
    {
       status = GAP_ERR_INVALID_PARAM;
    }
    else if ((status == GAP_ERR_NO_ERROR) && (p_trigger == NULL))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else if(p_envs_env != NULL)
    {
        co_buf_t* p_buf = NULL;
        uint16_t  data_len = 0;

        if(status == GAP_ERR_NO_ERROR)
        {
            if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, 0, L2CAP_LE_MTU_MIN + GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
            {
                // Check the values passed from Application are valid
                if (p_trigger->condition > ENVP_TRIG_VALUE_NOT_EQUAL)
                {
                    status = PRF_ERR_INVALID_PARAM;
                }
                else if (   (p_trigger->condition > ENVP_TRIG_PREV_VALUE_DIFFERENT) && (p_trigger->condition < ENVP_TRIG_VALUE_EQUAL)
                         && (   (es_char_id == ENVP_ES_BAR_PRES_TREND_CHAR)
                             || (es_char_id == ENVP_ES_MAGN_FLUX_2D_CHAR)
                             || (es_char_id == ENVP_ES_MAGN_FLUX_3D_CHAR)))
                {
                    status = PRF_ERR_INVALID_PARAM;
                }
                else if (trigger_idx > 2)
                {
                    status = PRF_ERR_INVALID_PARAM;
                }
                else
                {
                    co_buf_tail(p_buf)[0] = p_trigger->condition;
                    co_buf_tail_reserve(p_buf, 1);

                    if (p_trigger->condition < ENVP_TRIG_VALUE_LESS)
                    {
                        if ((p_trigger->condition == ENVP_TRIG_INTERVAL_FIXED) ||
                            (p_trigger->condition == ENVP_TRIG_INTERVAL_NO_LESS_THAN))
                        {
                            co_write24p(co_buf_tail(p_buf), co_htob24(p_trigger->time_based_trigger));
                            co_buf_tail_reserve(p_buf, 3);
                        }
                    }
                    else if ((p_trigger->condition > ENVP_TRIG_PREV_VALUE_DIFFERENT) &&
                             (p_trigger->condition < ENVP_TRIG_RESERVED))
                    {
                        // In this case the size of the operand is dependant on the size of the
                        // value field of the measurement characteristic.
                        status = envs_pack_es_char_value(es_char_id,&p_trigger->meas_value_trigger, p_buf);
                    }
                }

                data_len = co_buf_data_len(p_buf);
            }
            else
            {
                status = ATT_ERR_INSUFF_RESOURCE;
            }
        }

        // Return buffer that contains report data requested by peer device
        status = gatt_srv_att_read_get_cfm(conidx, p_envs_env->user_lid, GETF(token, ENVS_TOKEN_GATT_TOKEN),
                                           status, data_len, p_buf);
        if(p_buf != NULL)
        {
            co_buf_release(p_buf);
        }
    }
    else
    {
        status = PRF_ERR_REQ_DISALLOWED;
    }

    return (status);

}

uint16_t envs_read_config_cfm(uint8_t conidx, uint32_t token, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                              uint8_t trigger_logic)
{
    envs_env_t* p_envs_env = PRF_ENV_GET(ENVS, envs);

    if((es_char_id != GETF(token, ENVS_TOKEN_ES_CHAR_ID)) || (es_char_inst != GETF(token, ENVS_TOKEN_ES_CHAR_INST)))
    {
       status = GAP_ERR_INVALID_PARAM;
    }
    else if(p_envs_env != NULL)
    {
        co_buf_t* p_buf = NULL;
        uint16_t  data_len = 0;

        if(status == GAP_ERR_NO_ERROR)
        {
            if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, 0, L2CAP_LE_MTU_MIN + GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
            {
                co_buf_tail(p_buf)[0] = trigger_logic;
                co_buf_tail_reserve(p_buf, 1);
                data_len = co_buf_data_len(p_buf);
            }
            else
            {
                status = ATT_ERR_INSUFF_RESOURCE;
            }
        }

        // Return buffer that contains report data requested by peer device
        status = gatt_srv_att_read_get_cfm(conidx, p_envs_env->user_lid, GETF(token, ENVS_TOKEN_GATT_TOKEN),
                                           status, data_len, p_buf);
        if(p_buf != NULL)
        {
            co_buf_release(p_buf);
        }
    }
    else
    {
        status = PRF_ERR_REQ_DISALLOWED;
    }

    return (status);

}

uint16_t envs_read_user_desc_cfm(uint8_t conidx, uint32_t token, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                                 uint16_t total_len, co_buf_t* p_user_desc)
{
    envs_env_t* p_envs_env = PRF_ENV_GET(ENVS, envs);

    if((es_char_id != GETF(token, ENVS_TOKEN_ES_CHAR_ID)) || (es_char_inst != GETF(token, ENVS_TOKEN_ES_CHAR_INST)))
    {
       status = GAP_ERR_INVALID_PARAM;
    }
    else if ((status == GAP_ERR_NO_ERROR) && (p_user_desc == NULL))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    if((co_buf_head_len(p_user_desc) < ENVS_BUFFER_HEADER_LEN) || (co_buf_tail_len(p_user_desc) < ENVS_BUFFER_TAIL_LEN))
    {
        status = GAP_ERR_INVALID_BUFFER;
    }
    else if(p_envs_env != NULL)
    {
        // Return buffer that contains report data requested by peer device
        status = gatt_srv_att_read_get_cfm(conidx, p_envs_env->user_lid, GETF(token, ENVS_TOKEN_GATT_TOKEN),
                                           status, total_len, p_user_desc);
    }
    else
    {
        status = PRF_ERR_REQ_DISALLOWED;
    }

    return (status);
}

uint16_t envs_read_meas_cfm(uint8_t conidx, uint32_t token, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                            const envp_es_meas_desc_t* p_meas)
{
    envs_env_t* p_envs_env = PRF_ENV_GET(ENVS, envs);

    if((es_char_id != GETF(token, ENVS_TOKEN_ES_CHAR_ID)) || (es_char_inst != GETF(token, ENVS_TOKEN_ES_CHAR_INST)))
    {
       status = GAP_ERR_INVALID_PARAM;
    }
    else if((status == GAP_ERR_NO_ERROR) && (p_meas == NULL))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else if(p_envs_env != NULL)
    {
        co_buf_t* p_buf = NULL;
        uint16_t  data_len = 0;

        if(status == GAP_ERR_NO_ERROR)
        {
            if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, 0, L2CAP_LE_MTU_MIN + GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
            {
                co_write16p(co_buf_tail(p_buf), co_htobs(p_meas->flags));
                co_buf_tail_reserve(p_buf, 2);
                co_buf_tail(p_buf)[0] = p_meas->samp_func;
                co_buf_tail_reserve(p_buf, 1);
                co_write24p(co_buf_tail(p_buf), co_htob24(p_meas->meas_period));
                co_buf_tail_reserve(p_buf, 3);
                co_write24p(co_buf_tail(p_buf), co_htob24(p_meas->update_period));
                co_buf_tail_reserve(p_buf, 3);
                co_buf_tail(p_buf)[0] = p_meas->app;
                co_buf_tail_reserve(p_buf, 1);
                co_buf_tail(p_buf)[0] = p_meas->meas_uncertainty;
                co_buf_tail_reserve(p_buf, 1);
                data_len = co_buf_data_len(p_buf);
            }
            else
            {
                status = ATT_ERR_INSUFF_RESOURCE;
            }
        }

        // Return buffer that contains report data requested by peer device
        status = gatt_srv_att_read_get_cfm(conidx, p_envs_env->user_lid, GETF(token, ENVS_TOKEN_GATT_TOKEN),
                                           status, data_len, p_buf);
        if(p_buf != NULL)
        {
            co_buf_release(p_buf);
        }
    }
    else
    {
        status = PRF_ERR_REQ_DISALLOWED;
    }

    return (status);


}


#if (HOST_MSG_API)
/*
 * PROFILE MSG HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Send a ENVS_CMP_EVT message to the application.
 * @param[in] conidx connection index
 * @param[in] operation Operation completed
 * @param[in] status status of the operation
 ****************************************************************************************
 */
__STATIC void envs_send_cmp_evt(uint8_t conidx, uint8_t operation, uint16_t status)
{
    struct envs_cmp_evt *p_evt;

    // Send the message
    p_evt = KE_MSG_ALLOC(ENVS_CMP_EVT, PRF_DST_TASK(ENVS), PRF_SRC_TASK(ENVS), envs_cmp_evt);

    if(p_evt != NULL)
    {
        p_evt->conidx     = conidx;
        p_evt->operation  = operation;
        p_evt->status     = status;
        ke_msg_send(p_evt);
    }
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref ENVS_UPD_RANGE_CMD message.
 * @param[in] msgid Id of the message received
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int envs_upd_range_cmd_handler(ke_msg_id_t const msgid, struct envs_upd_range_cmd *p_param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct envs_cmp_evt *p_cmp_evt;
    uint16_t status = envs_range_upd(p_param->es_char_id, p_param->es_char_inst, &(p_param->range));

    // send completed information to APP task that contains error status
    p_cmp_evt = KE_MSG_ALLOC(ENVS_CMP_EVT, src_id, dest_id, envs_cmp_evt);

    if(p_cmp_evt)
    {
        p_cmp_evt->conidx     = GAP_INVALID_CONIDX;
        p_cmp_evt->operation  = ENVS_UPD_RANGE_OP_CODE;
        p_cmp_evt->status     = status;
        ke_msg_send(p_cmp_evt);
    }

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref ENVS_INDICATE_CMD message. APP request to Send IND.
 * @param[in] msgid Id of the message received.
 * @param[in] p_param Pointer to the p_parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int envs_indicate_cmd_handler(ke_msg_id_t const msgid, struct envs_indicate_cmd *p_param,
                                       ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = envs_indicate(p_param->conidx, p_param->es_char_id, p_param->ind_flag);

    if(status != GAP_ERR_NO_ERROR)
    {
        struct envs_cmp_evt *p_cmp_evt;
        // send completed information to APP task that contains error status
        p_cmp_evt = KE_MSG_ALLOC(ENVS_CMP_EVT, src_id, dest_id, envs_cmp_evt);

        if(p_cmp_evt)
        {
            p_cmp_evt->conidx     = p_param->conidx;
            p_cmp_evt->operation  = ENVS_INDICATE_OP_CODE;
            p_cmp_evt->status     = status;
            ke_msg_send(p_cmp_evt);
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_WRITE_REQ_IND message.
 * @param[in] msgid Id of the message received.
 * @param[in] p_param Pointer to the p_parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int envs_notify_cmd_handler(ke_msg_id_t const msgid, struct envs_notify_cmd *p_param,
                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = envs_notify(p_param->conidx, p_param->es_char_id, p_param->es_char_inst, &(p_param->value));

    if(status != GAP_ERR_NO_ERROR)
    {
        struct envs_cmp_evt *p_cmp_evt;
        // send completed information to APP task that contains error status
        p_cmp_evt = KE_MSG_ALLOC(ENVS_CMP_EVT, src_id, dest_id, envs_cmp_evt);

        if(p_cmp_evt)
        {
            p_cmp_evt->conidx     = p_param->conidx;
            p_cmp_evt->operation  = ENVS_NOTIFY_OP_CODE;
            p_cmp_evt->status     = status;
            ke_msg_send(p_cmp_evt);
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref ENVS_WR_CCC_CFM message. confirmation from the APP.
 * @param[in] msgid Id of the message received.
 * @param[in] p_param Pointer to the p_parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int envs_wr_ccc_cfm_handler(ke_msg_id_t const msgid, struct envs_wr_ccc_cfm *p_param,
                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    envs_write_ccc_cfm(p_param->conidx, p_param->token, p_param->status, p_param->es_char_id, p_param->es_char_inst);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref ENVS_WR_TRIGGER_CFM message. Confirmation from the APP.
 * @param[in] msgid Id of the message received.
 * @param[in] p_param Pointer to the p_parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int envs_wr_trigger_cfm_handler(ke_msg_id_t const msgid, struct envs_wr_trigger_cfm *p_param,
                                         ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    envs_write_trigger_cfm(p_param->conidx, p_param->token, p_param->status, p_param->es_char_id, p_param->es_char_inst);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref ENVS_WR_CONFIG_CFM message. Confirmation from the APP.
 * @param[in] msgid Id of the message received.
 * @param[in] p_param Pointer to the p_parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int envs_wr_config_cfm_handler(ke_msg_id_t const msgid, struct envs_wr_config_cfm *p_param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    envs_write_config_cfm(p_param->conidx, p_param->token, p_param->status, p_param->es_char_id, p_param->es_char_inst);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref ENVS_WR_USER_DESCRIPTION_CFM message. Confirmation from the APP.
 * @param[in] msgid Id of the message received.
 * @param[in] p_param Pointer to the p_parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int envs_wr_user_description_cfm_handler(ke_msg_id_t const msgid, struct envs_wr_user_description_cfm *p_param,
                                                  ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    envs_write_user_desc_cfm(p_param->conidx, p_param->token, p_param->status, p_param->es_char_id, p_param->es_char_inst);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref ENVS_RD_CCC_CFM message. APP read response of one of the CCC read.
 * @param[in] msgid Id of the message received.
 * @param[in] p_param Pointer to the p_parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int envs_rd_ccc_cfm_handler(ke_msg_id_t const msgid, struct envs_rd_ccc_cfm *p_param,
                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    envs_read_ccc_cfm(p_param->conidx, p_param->token, p_param->status, p_param->es_char_id, p_param->es_char_inst,
                      p_param->ccc);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref ENVS_RD_TRIGGER_CFM message.
 * @brief APP read response of one of the Triggers for a characteristic.
 * @param[in] msgid Id of the message received.
 * @param[in] p_param Pointer to the p_parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int envs_rd_trigger_cfm_handler(ke_msg_id_t const msgid, struct envs_rd_trigger_cfm *p_param,
                                         ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    envs_read_trigger_cfm(p_param->conidx, p_param->token, p_param->status, p_param->es_char_id, p_param->es_char_inst,
                          p_param->trigger_idx, &(p_param->trigger));

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref ENVS_RD_CONFIG_CFM message.
 * @brief APP read response of one of the Trigger Configs  for a characteristic.
 * @param[in] msgid Id of the message received.
 * @param[in] p_param Pointer to the p_parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int envs_rd_config_cfm_handler(ke_msg_id_t const msgid, struct envs_rd_config_cfm *p_param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    envs_read_config_cfm(p_param->conidx, p_param->token, p_param->status, p_param->es_char_id, p_param->es_char_inst,
                         p_param->trigger_logic);
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref ENVS_RD_USER_DESCRIPTION_CFM message.
 * @brief APP read response of the User Description (name) for a characteristic.
 * @param[in] msgid Id of the message received.
 * @param[in] p_param Pointer to the p_parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int envs_rd_user_description_cfm_handler(ke_msg_id_t const msgid, struct envs_rd_user_description_cfm *p_param,
                                                  ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    co_buf_t* p_buf = NULL;
    uint16_t status = p_param->status;

    // allocate buffer that contains user description
    if(status == GAP_ERR_NO_ERROR)
    {
        if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, p_param->user_desc.length, GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
        {
            co_buf_copy_data_from_mem(p_buf, p_param->user_desc.str, p_param->user_desc.length);
        }
        else
        {
            status = ATT_ERR_INSUFF_RESOURCE;
        }
    }

    envs_read_user_desc_cfm(p_param->conidx, p_param->token, p_param->status, p_param->es_char_id, p_param->es_char_inst,
                            p_param->total_len, p_buf);

    if(p_buf != NULL)
    {
        co_buf_release(p_buf);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref ENVS_RD_VALUE_CFM message.
 * @param[in] msgid Id of the message received.
 * @param[in] p_param Pointer to the p_parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int envs_rd_value_cfm_handler(ke_msg_id_t const msgid, struct envs_rd_value_cfm *p_param,
                                       ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    envs_read_value_cfm(p_param->conidx, p_param->token, p_param->status, p_param->es_char_id, p_param->es_char_inst,
                        &(p_param->value));
    return (KE_MSG_CONSUMED);
}



/**
 ****************************************************************************************
 * @brief Handles reception of the @ref ENVS_UPD_MEAS_CMD message.APP request to change MEAS descriptor.
 * @param[in] msgid Id of the message received.
 * @param[in] p_param Pointer to the p_parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int envs_rd_meas_cfm_handler(ke_msg_id_t const msgid, struct envs_rd_meas_cfm *p_param,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    envs_read_meas_cfm(p_param->conidx, p_param->token, p_param->status, p_param->es_char_id, p_param->es_char_inst,
                       &(p_param->meas));
    return (KE_MSG_CONSUMED);
}

/// Default State handlers definition
KE_MSG_HANDLER_TAB(envs)
{
    // Note: all messages must be sorted in ID ascending order

    // APP request to change RANGE descriptor
    {ENVS_UPD_RANGE_CMD,             (ke_msg_func_t) envs_upd_range_cmd_handler             },
    // APP request to Send IND
    {ENVS_INDICATE_CMD,              (ke_msg_func_t) envs_indicate_cmd_handler              },
    // APP request to Send NTF
    {ENVS_NOTIFY_CMD,                (ke_msg_func_t) envs_notify_cmd_handler                },

    // APP Confirms/Rejects a change in the CCC for a characteristic
    {ENVS_WR_CCC_CFM,                (ke_msg_func_t) envs_wr_ccc_cfm_handler                },
    // APP Confirms/Rejects a change in one of the Triggers  for a characteristic
    {ENVS_WR_TRIGGER_CFM,            (ke_msg_func_t) envs_wr_trigger_cfm_handler            },
    // APP Confirms/Rejects a change in one of the Trigger Configs  for a characteristic
    {ENVS_WR_CONFIG_CFM,             (ke_msg_func_t) envs_wr_config_cfm_handler             },
    // APP Confirms/Rejects a change in the User Description (name) for a characteristic
    {ENVS_WR_USER_DESCRIPTION_CFM,   (ke_msg_func_t) envs_wr_user_description_cfm_handler   },

    // Response from the APP
    // APP read response of the CCC for a characteristic
    {ENVS_RD_CCC_CFM,                (ke_msg_func_t) envs_rd_ccc_cfm_handler                },
    // APP read response of one of the Triggers  for a characteristic
    {ENVS_RD_TRIGGER_CFM,            (ke_msg_func_t) envs_rd_trigger_cfm_handler            },
    // APP read response of one of the Trigger Configs  for a characteristic
    {ENVS_RD_CONFIG_CFM,             (ke_msg_func_t) envs_rd_config_cfm_handler             },
    // APP read response of the User Description (name) for a characteristic
    {ENVS_RD_USER_DESCRIPTION_CFM,   (ke_msg_func_t) envs_rd_user_description_cfm_handler   },
    // APP read response of the Value for a characteristic
    {ENVS_RD_VALUE_CFM,              (ke_msg_func_t) envs_rd_value_cfm_handler              },
    // APP read response of MEAS descriptor
    {ENVS_RD_MEAS_CFM,               (ke_msg_func_t) envs_rd_meas_cfm_handler               },

};

/**
 ****************************************************************************************
 * @brief Completion of indicate procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] status        Status of the procedure execution (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC void envs_cb_indicate_cmp(uint8_t conidx, uint16_t status)
{
    envs_send_cmp_evt(conidx, ENVS_INDICATE_OP_CODE, status);
}

/**
 ****************************************************************************************
 * @brief Completion of notify procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] status        Status of the procedure execution (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC void envs_cb_notify_cmp(uint8_t conidx, uint16_t status)
{
    envs_send_cmp_evt(conidx, ENVS_NOTIFY_OP_CODE, status);
}

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
__STATIC void envs_cb_write_ccc_req(uint8_t conidx, uint32_t token, uint8_t es_char_id, uint8_t es_char_inst,
                                    uint16_t ccc)
{
    struct envs_wr_ccc_req_ind *p_evt;

    // Send the message
    p_evt = KE_MSG_ALLOC(ENVS_WR_CCC_REQ_IND, PRF_DST_TASK(ENVS), PRF_SRC_TASK(ENVS), envs_wr_ccc_req_ind);

    if(p_evt != NULL)
    {
        p_evt->conidx       = conidx;
        p_evt->es_char_id   = es_char_id;
        p_evt->es_char_inst = es_char_inst;
        p_evt->token        = token;
        p_evt->ccc          = ccc;
        ke_msg_send(p_evt);
    }
}

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
__STATIC void envs_cb_write_trigger_req(uint8_t conidx, uint32_t token, uint8_t es_char_id, uint8_t es_char_inst,
                                        uint8_t trigger_idx, const envp_trigger_t* p_trigger)
{
    struct envs_wr_trigger_req_ind *p_evt;

    // Send the message
    p_evt = KE_MSG_ALLOC(ENVS_WR_TRIGGER_REQ_IND, PRF_DST_TASK(ENVS), PRF_SRC_TASK(ENVS), envs_wr_trigger_req_ind);

    if(p_evt != NULL)
    {
        p_evt->conidx       = conidx;
        p_evt->es_char_id   = es_char_id;
        p_evt->es_char_inst = es_char_inst;
        p_evt->trigger_idx  = trigger_idx;
        p_evt->token        = token;
        memcpy(&(p_evt->trigger), p_trigger, sizeof(envp_trigger_t));
        ke_msg_send(p_evt);
    }
}


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
__STATIC void envs_cb_write_config_req(uint8_t conidx, uint32_t token, uint8_t es_char_id, uint8_t es_char_inst,
                                       uint8_t trigger_logic)
{
    struct envs_wr_config_req_ind *p_evt;

    // Send the message
    p_evt = KE_MSG_ALLOC(ENVS_WR_CONFIG_REQ_IND, PRF_DST_TASK(ENVS), PRF_SRC_TASK(ENVS), envs_wr_config_req_ind);

    if(p_evt != NULL)
    {
        p_evt->conidx        = conidx;
        p_evt->es_char_id    = es_char_id;
        p_evt->es_char_inst  = es_char_inst;
        p_evt->token         = token;
        p_evt->trigger_logic = trigger_logic;
        ke_msg_send(p_evt);
    }
}

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
__STATIC void envs_cb_write_user_desc_req(uint8_t conidx, uint32_t token, uint8_t es_char_id, uint8_t es_char_inst,
                                          co_buf_t* p_user_desc)
{
    struct envs_wr_user_description_req_ind *p_evt;
    uint16_t data_len = co_buf_data_len(p_user_desc);

    // Send the message
    p_evt = KE_MSG_ALLOC_DYN(ENVS_WR_USER_DESCRIPTION_REQ_IND, PRF_DST_TASK(ENVS),
                             PRF_SRC_TASK(ENVS), envs_wr_user_description_req_ind, data_len);

    if(p_evt != NULL)
    {
        p_evt->conidx           = conidx;
        p_evt->es_char_id       = es_char_id;
        p_evt->es_char_inst     = es_char_inst;
        p_evt->token            = token;
        p_evt->user_desc.length = data_len;
        co_buf_copy_data_to_mem(p_user_desc, p_evt->user_desc.str, data_len);
        ke_msg_send(p_evt);
    }
}

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
__STATIC void envs_cb_read_value_req(uint8_t conidx, uint32_t token, uint8_t es_char_id, uint8_t es_char_inst)
{
    struct envs_rd_value_req_ind *p_evt;

    // Send the message
    p_evt = KE_MSG_ALLOC(ENVS_RD_VALUE_REQ_IND, PRF_DST_TASK(ENVS), PRF_SRC_TASK(ENVS), envs_rd_value_req_ind);

    if(p_evt != NULL)
    {
        p_evt->conidx       = conidx;
        p_evt->es_char_id   = es_char_id;
        p_evt->es_char_inst = es_char_inst;
        p_evt->token        = token;
        ke_msg_send(p_evt);
    }
}

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
__STATIC void envs_cb_read_ccc_req(uint8_t conidx, uint32_t token, uint8_t es_char_id, uint8_t es_char_inst)
{
    struct envs_rd_ccc_req_ind *p_evt;

    // Send the message
    p_evt = KE_MSG_ALLOC(ENVS_RD_CCC_REQ_IND, PRF_DST_TASK(ENVS), PRF_SRC_TASK(ENVS), envs_rd_ccc_req_ind);

    if(p_evt != NULL)
    {
        p_evt->conidx       = conidx;
        p_evt->es_char_id   = es_char_id;
        p_evt->es_char_inst = es_char_inst;
        p_evt->token        = token;
        ke_msg_send(p_evt);
    }
}


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
__STATIC void envs_cb_read_trigger_req(uint8_t conidx, uint32_t token, uint8_t es_char_id, uint8_t es_char_inst,
                                       uint8_t trigger_idx)
{
    struct envs_rd_trigger_req_ind *p_evt;

    // Send the message
    p_evt = KE_MSG_ALLOC(ENVS_RD_TRIGGER_REQ_IND, PRF_DST_TASK(ENVS), PRF_SRC_TASK(ENVS), envs_rd_trigger_req_ind);

    if(p_evt != NULL)
    {
        p_evt->conidx       = conidx;
        p_evt->es_char_id   = es_char_id;
        p_evt->es_char_inst = es_char_inst;
        p_evt->trigger_idx  = trigger_idx;
        p_evt->token        = token;
        ke_msg_send(p_evt);
    }
}

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
__STATIC void envs_cb_read_config_req(uint8_t conidx, uint32_t token, uint8_t es_char_id, uint8_t es_char_inst)
{
    struct envs_rd_config_req_ind *p_evt;

    // Send the message
    p_evt = KE_MSG_ALLOC(ENVS_RD_CONFIG_REQ_IND, PRF_DST_TASK(ENVS), PRF_SRC_TASK(ENVS), envs_rd_config_req_ind);

    if(p_evt != NULL)
    {
        p_evt->conidx       = conidx;
        p_evt->es_char_id   = es_char_id;
        p_evt->es_char_inst = es_char_inst;
        p_evt->token        = token;
        ke_msg_send(p_evt);
    }
}

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
__STATIC void envs_cb_read_user_desc_req(uint8_t conidx, uint32_t token, uint8_t es_char_id, uint8_t es_char_inst,
                                         uint16_t offset, uint16_t max_len)
{
    struct envs_rd_user_description_req_ind *p_evt;

    // Send the message
    p_evt = KE_MSG_ALLOC(ENVS_RD_USER_DESCRIPTION_REQ_IND, PRF_DST_TASK(ENVS),
                         PRF_SRC_TASK(ENVS), envs_rd_user_description_req_ind);

    if(p_evt != NULL)
    {
        p_evt->conidx       = conidx;
        p_evt->es_char_id   = es_char_id;
        p_evt->es_char_inst = es_char_inst;
        p_evt->token        = token;
        p_evt->offset       = offset;
        p_evt->max_len      = max_len;
        ke_msg_send(p_evt);
    }
}

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
__STATIC void envs_cb_read_meas_req(uint8_t conidx, uint32_t token, uint8_t es_char_id, uint8_t es_char_inst)
{
    struct envs_rd_meas_req_ind *p_evt;

    // Send the message
    p_evt = KE_MSG_ALLOC(ENVS_RD_MEAS_REQ_IND, PRF_DST_TASK(ENVS), PRF_SRC_TASK(ENVS), envs_rd_meas_req_ind);

    if(p_evt != NULL)
    {
        p_evt->conidx       = conidx;
        p_evt->es_char_id   = es_char_id;
        p_evt->es_char_inst = es_char_inst;
        p_evt->token        = token;
        ke_msg_send(p_evt);
    }
}


/// Default Message handle
__STATIC const envs_cb_t envs_msg_cb =
{
    .cb_indicate_cmp        = envs_cb_indicate_cmp,
    .cb_notify_cmp          = envs_cb_notify_cmp,
    .cb_write_ccc_req       = envs_cb_write_ccc_req,
    .cb_write_trigger_req   = envs_cb_write_trigger_req,
    .cb_write_config_req    = envs_cb_write_config_req,
    .cb_write_user_desc_req = envs_cb_write_user_desc_req,
    .cb_read_value_req      = envs_cb_read_value_req,
    .cb_read_ccc_req        = envs_cb_read_ccc_req,
    .cb_read_trigger_req    = envs_cb_read_trigger_req,
    .cb_read_config_req     = envs_cb_read_config_req,
    .cb_read_user_desc_req  = envs_cb_read_user_desc_req,
    .cb_read_meas_req       = envs_cb_read_meas_req,
};
#endif // (HOST_MSG_API)

/*
 * PROFILE DEFAULT HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialization of the ENVS module.
 * This function performs all the initializations of the Profile module.
 *  - Creation of database (if it's a service)
 *  - Allocation of profile required memory
 *  - Initialization of task descriptor to register application
 *      - Task State array
 *      - Number of tasks
 *      - Default task handler
 *
 * @param[out]    p_env        Collector or Service allocated environment data.
 * @param[in,out] p_start_hdl  Service start handle (0 - dynamically allocated), only applies for services.
 * @param[in]     sec_lvl      Security level (see enum #gatt_svc_info_bf)
 * @param[in]     user_prio    GATT User priority
 * @param[in]     p_param      Configuration parameters of profile collector or service (32 bits aligned)
 * @param[in]     p_cb         Callback structure that handles event from profile
 *
 * @return status code to know if profile initialization succeed or not.
 ****************************************************************************************
 */
__STATIC uint16_t envs_init(prf_data_t *p_env, uint16_t *p_start_hdl, uint8_t sec_lvl, uint8_t user_prio,
                          const struct envs_db_cfg *p_params, const envs_cb_t* p_cb)
{
    //------------------ create the attribute database for the profile -------------------

    // DB Creation Status
    uint16_t status = GAP_ERR_NO_ERROR;
    uint8_t  user_lid = GATT_INVALID_USER_LID;
    uint8_t* p_temp_data = NULL;
    envs_env_t* p_envs_env = NULL;

    do
    {
        // Bitfield for the ATT database
        // 11 bits per Characteristic
        uint16_t flag;
        // this can be shifted up to 7 but kept for the shift in the all array of config
        uint16_t bit_start;
        // temporally keep each cfg byte, offset in config array, shift of the feature in config array
        uint8_t cfg;
        uint8_t offs;
        uint8_t flag_shift;
        uint32_t bitmask;
        // to calculate the trigger number in the characteristic to add the boolean logic descriptor
        uint8_t trig_nb;

        //------------------ create the attribute database for the profile -------------------
        gatt_att16_desc_t* p_envs_att_db = NULL;
        // used to modify Extended Properties descriptor
        gatt_att16_desc_t *p_envs_att_ext;
        // used to modify Write permission for triggers
        gatt_att16_desc_t *p_envs_att_rw;
        uint8_t *p_att;

        // Service content flag
        // Service attribute flags
        uint8_t *p_cfg_flag;
        uint32_t db_size;
        // number of attribute to process;
        uint8_t nb_att;
        // calculate storage for Characteristic value
        uint16_t size_of_value;

        // cursors
        uint8_t i;
        uint8_t j;

        #if (HOST_MSG_API)
        if(p_cb == NULL)
        {
            p_cb = &(envs_msg_cb);
        }
        #endif // (HOST_MSG_API)

        if(   (p_params == NULL) || (p_start_hdl == NULL) || (p_cb == NULL)
           || (p_cb->cb_indicate_cmp == NULL)       || (p_cb->cb_notify_cmp == NULL)
           || (p_cb->cb_write_ccc_req == NULL)      || (p_cb->cb_write_trigger_req == NULL)
           || (p_cb->cb_write_config_req == NULL)   || (p_cb->cb_write_user_desc_req == NULL)
           || (p_cb->cb_read_value_req == NULL)     || (p_cb->cb_read_ccc_req == NULL)
           || (p_cb->cb_read_trigger_req == NULL)   || (p_cb->cb_read_config_req == NULL)
           || (p_cb->cb_read_user_desc_req == NULL) || (p_cb->cb_read_meas_req == NULL))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // limit the number of characteristic
        if (p_params->nb_chars > NUMBER_CHARACTERISTICS_MAX)
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }

        // register ENVS user
        status = gatt_user_srv_register(ENVS_USR_DESC_MAX_LEN + GATT_WRITE_HEADER_LEN, user_prio, &envs_cb, &user_lid);
        if(status != GAP_ERR_NO_ERROR) break;

        //-------------------- allocate memory required for the profile  ---------------------
        p_envs_env = (envs_env_t *) ke_malloc_user(sizeof(envs_env_t) + sizeof(envs_char_info_t) * p_params->nb_chars, KE_MEM_PROFILE);

        if(p_envs_env == NULL)
        {
            status = GAP_ERR_INSUFF_RESOURCES;
            break;
        }

        // copy characteristic configuration
        p_envs_env->nb_chars = p_params->nb_chars;
        for(i = 0 ; i < p_params->nb_chars ; i++)
        {
            p_envs_env->instance[i].es_char_id = p_params->instance[i].es_char_id;
            p_envs_env->instance[i].desc_en    = p_params->instance[i].desc_en;
            p_envs_env->instance[i].desc_rw    = p_params->instance[i].desc_rw;
        }

        // ----------------------------------------------------------------------------
        // --------------------- Configure Database
        // ----------------------------------------------------------------------------

        // calculate size of the attribute database
        db_size  = (sizeof(envs_att_db_descriptors1) + sizeof(envs_att_db_chars[0]) + sizeof(envs_att_db_descriptors3));
        db_size *= p_envs_env->nb_chars; // multiply it by number of characteristics value present
        db_size += sizeof(envs_att_db_header);

        //calculate the number of attributes
        nb_att = db_size / sizeof(gatt_att16_desc_t);

        /// allocate temporary attribute database + configuration bit field - will be free at end of function
        p_temp_data   = (uint8_t *)ke_malloc_user(db_size + CO_DIVIDE_CEIL(nb_att, BITS_IN_BYTE), KE_MEM_NON_RETENTION);
        if(p_temp_data == NULL)
        {
            status = GAP_ERR_INSUFF_RESOURCES;
            break;
        }

        p_envs_att_db = (gatt_att16_desc_t *)p_temp_data;
        p_cfg_flag    = p_temp_data + db_size;

        // round size to a nearest byte
        memset(p_cfg_flag, 0, CO_DIVIDE_CEIL(nb_att, BITS_IN_BYTE));

        // Check if Characteristic have multiple instances and set ES_MEASUREMENT enabled for all of it
        for (i = 1; i < p_envs_env->nb_chars; i++)
        {
            if ((p_envs_env->instance[i].desc_en & ENVS_DESC_EN_MEAS) != ENVS_DESC_EN_MEAS)
            {
                uint8_t char_id = p_envs_env->instance[i].es_char_id;

                for (j = 1; j < p_envs_env->nb_chars; j++)
                {
                    if (i != j)
                    {
                        if (char_id == p_envs_env->instance[j].es_char_id)
                        {
                            // we found other instance of this characteristic!
                            // force it to have ES_MEASUREMENT descriptor
                            p_envs_env->instance[i].desc_en |= ENVS_DESC_EN_MEAS;
                            break;
                        }
                    }
                }
            }
        }

        // copy characteristic instances to the attribute configuration array
        memcpy(p_envs_att_db, envs_att_db_header, sizeof(envs_att_db_header));
        p_att = p_temp_data + sizeof(envs_att_db_header);

        // add bifield here according to configuration
        // if ENVS_IDX_VAL_CHANGE_CHAR is present otherwise only ENVS_IDX_SVC
        p_cfg_flag[0] =(p_envs_env->instance[ENVP_ES_DESCRIPTOR_VALUE_CHANGED_CHAR].desc_en ? 15 : 1);

        bit_start = 4;

        size_of_value = 0;

        // Create database structure by combining descriptors and creating the enabled attributes bitfield
        for (i = 1; i <p_envs_env->nb_chars; i++)
        {
            size_of_value += envs_val_range_size[p_envs_env->instance[i].es_char_id];
            // copy characteristic instances to the attribute configuration array
            p_envs_att_ext = memcpy(p_att, envs_att_db_descriptors1, sizeof(envs_att_db_descriptors1));
            p_att += sizeof(envs_att_db_descriptors1);
            // copy characteristic instances to the attribute configuration array
            memcpy(p_att, &envs_att_db_chars[p_envs_env->instance[i].es_char_id], sizeof(envs_att_db_chars[0]));
            p_att += sizeof(envs_att_db_chars[0]);
            // copy characteristic instances to the attribute configuration array
            p_envs_att_rw = memcpy(p_att, envs_att_db_descriptors3, sizeof(envs_att_db_descriptors3));
            p_att += sizeof(envs_att_db_descriptors3);

            // Enable RW according to the configuration ///
            // configure Write permissions
            if ((p_envs_env->instance[i].desc_rw != ENVS_FEATURE_WR_DESC) &&
                    (p_envs_env->instance[i].desc_rw != ENVS_FEATURE_WR_DESC_TRIG))
            {
                // remove write permissions from USD DESCRIPTOR
                p_envs_att_rw[6].info &= ~PROP(WR);
            }

            // remove write permissions from TRIGGER
            if (   (p_envs_env->instance[i].desc_rw != ENVS_FEATURE_WR_TRIG)
                && (p_envs_env->instance[i].desc_rw != ENVS_FEATURE_WR_DESC_TRIG))
            {
                p_envs_att_rw[2].info &= ~PROP(WR);
                p_envs_att_rw[3].info &= ~PROP(WR);
                p_envs_att_rw[4].info &= ~PROP(WR);
                p_envs_att_rw[5].info &= ~PROP(WR);
            }

            // Valid range is double size of the Characteristic value
            p_envs_att_rw[7].ext_info = (2 * envs_att_db_chars[p_envs_env->instance[i].es_char_id].ext_info) | OPT(NO_OFFSET);

            flag = 0;
            cfg  = p_envs_env->instance[i].desc_en;
            cfg &= ENVS_DESC_EN_MASK;

            //check the ES Configuration is only present if more than 1 TRIG is enabled
            trig_nb = cfg & ( ENVS_DESC_EN_TRIG_1 | ENVS_DESC_EN_TRIG_2 | ENVS_DESC_EN_TRIG_3);
            trig_nb = NB_ONE_BITS(trig_nb);

            // have more than 1 TRIG - need to add CFG descriptor
            if (1 < trig_nb)
            {   //more than one trigger - need CFG descriptor
                cfg |= ENVS_DESC_EN_CFG;
            }
            else
            {   //only single TRIG - no need for CFG descriptor
                cfg &= ~ENVS_DESC_EN_CFG;
            }

            // if we have a TRIGGER - need CCC
            if (trig_nb)
            {
                cfg |= ENVS_DESC_EN_CCC;
            }

            // write back the updated descriptor configuration
            p_envs_env->instance[i].desc_en = cfg;

            if (cfg)
            {
                flag = cfg;
                // add least 2 bits with characteristic attribute(ENVS_IDX_CHAR_CHAR) and value(ENVS_IDX_CHAR_VAL)
                flag <<= 2;
                // ENVS_IDX_DESC_EXT is the highest bit enabled
                flag |= (1<<ENVS_IDX_CHAR_CHAR)|(1<<ENVS_IDX_CHAR_VAL);
                if (cfg & ENVS_DESC_EN_USR_DESC)
                {
                    // Add Extended Properties Descriptor present bit
                    p_envs_att_ext[0].info |= PROP(EXT);
                    // Add Corresponding bit to the configuration
                    flag |= (1<< ENVS_IDX_DESC_EXT);
                }
            }
            else
            {
                //no feature present
            }

            //Find the byte to put the next enables attributes bitfield in 'flag'
            offs = bit_start / BITS_IN_BYTE;
            // the start bit in this byte to start writing
            flag_shift = bit_start - (offs*BITS_IN_BYTE);
            bitmask = flag << flag_shift;
            // prepare bitfield
            p_cfg_flag[offs] |= (uint8_t)(bitmask & 0xff);

            if ((bitmask>>BITS_IN_BYTE) & 0xff)
            {
                p_cfg_flag[offs+1] |= (uint8_t)((bitmask>>BITS_IN_BYTE) & 0xff);
            }
            if ((bitmask>>16) & 0xff)
            {
                p_cfg_flag[offs+2] |= (uint8_t)((bitmask>>16) & 0xff);
            }

            //shift by the number of bits as the count of all attributes per characteristic
            bit_start += ENVS_IDX_ATT_NB;
        }

        // ----------------------------------------------------------------------------
        // --------------------- Insert Database
        // ----------------------------------------------------------------------------

        // Add GAP service
        status = gatt_db_svc16_add(user_lid, sec_lvl, GATT_SVC_ENVIRONMENTAL_SENSING, nb_att,
                                   p_cfg_flag, p_envs_att_db, nb_att, p_start_hdl);
        if(status != GAP_ERR_NO_ERROR) break;


        // ----------------------------------------------------------------------------
        // --------------------- Initialize profile environement
        // ----------------------------------------------------------------------------

        // allocate ENVS required environment variable
        p_env->p_env = (prf_hdr_t *) p_envs_env;
        p_envs_env->start_hdl       = *p_start_hdl;
        p_envs_env->user_lid        = user_lid;
        p_envs_env->op_ongoing      = false;
        p_envs_env->in_exe_op       = false;
        co_list_init(&(p_envs_env->wait_queue));

        // initialize profile environment variable
        p_envs_env->prf_env.p_cb     = p_cb;
        #if (HOST_MSG_API)
        p_env->desc.msg_handler_tab  = envs_msg_handler_tab;
        p_env->desc.msg_cnt          = ARRAY_LEN(envs_msg_handler_tab);
        #endif // (HOST_MSG_API)

    } while(0);

    if((status != GAP_ERR_NO_ERROR) && (user_lid != GATT_INVALID_USER_LID))
    {
        gatt_user_unregister(user_lid);

        // clean-up in case of error
        if(p_envs_env != NULL)
        {
             ke_free(p_envs_env);
        }
    }

    // remove temporary data
    if(p_temp_data != NULL)
    {
        ke_free(p_temp_data);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Destruction of the profile module - due to a reset or profile remove.
 *
 * This function clean-up allocated memory.
 *
 * @param[in,out]    p_env        Collector or Service allocated environment data.
 * @param[in]        reason       Destroy reason (see enum #prf_destroy_reason)
 *
 * @return status of the destruction, if fails, profile considered not removed.
 ****************************************************************************************
 */
__STATIC uint16_t envs_destroy(prf_data_t *p_env, uint8_t reason)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    envs_env_t *p_envs_env = (envs_env_t *) p_env->p_env;

    if(reason != PRF_DESTROY_RESET)
    {
        status = gatt_user_unregister(p_envs_env->user_lid);
    }

    if(status == GAP_ERR_NO_ERROR)
    {
        if(reason != PRF_DESTROY_RESET)
        {
            // remove buffer in wait queue
            while(!co_list_is_empty(&p_envs_env->wait_queue))
            {
                co_buf_t* p_buf = (co_buf_t*) co_list_pop_front(&p_envs_env->wait_queue);
                co_buf_release(p_buf);
            }
        }

        // free profile environment variables
        p_env->p_env = NULL;
        ke_free(p_envs_env);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief @brief Handles Connection creation
 *
 * @param[in,out]    env          Collector or Service allocated environment data.
 * @param[in]        conidx       Connection index
 * @param[in]        is_le_con    True if it's a BLE connection, False if it's a BT-Classic connection
 ****************************************************************************************
 */
__STATIC void envs_con_create(prf_data_t *p_env, uint8_t conidx, bool is_le_con)
{
    // Nothing to do
}

/**
 ****************************************************************************************
 * @brief Handles Disconnection
 *
 * @param[in,out]    p_env      Collector or Service allocated environment data.
 * @param[in]        conidx     Connection index
 * @param[in]        reason     Detach reason
 ****************************************************************************************
 */
__STATIC void envs_con_cleanup(prf_data_t *p_env, uint8_t conidx, uint16_t reason)
{
    // Nothing to do
}



/// ENVS Task interface required by profile manager
const prf_task_cbs_t envs_itf =
{
    .cb_init          = (prf_init_cb) envs_init,
    .cb_destroy       = envs_destroy,
    .cb_con_create    = envs_con_create,
    .cb_con_cleanup   = envs_con_cleanup,
};

/**
 ****************************************************************************************
 * @brief Retrieve service profile interface
 *
 * @return service profile interface
 ****************************************************************************************
 */
const prf_task_cbs_t *envs_prf_itf_get(void)
{
    return &envs_itf;
}
#endif //(BLE_ENV_SERVER)

/// @} ENVS

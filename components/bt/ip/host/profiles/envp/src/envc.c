/**
 ****************************************************************************************
 *
 * @file envc.c
 *
 * @brief Environmental Sensor Collector implementation.
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup ENVC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_ENV_CLIENT)

#include "envc.h"
#include "gap.h"
#include "gatt.h"
#include "prf_utils.h"
#include "prf.h"

#include "co_utils.h"
#include "co_endian.h"
#include "co_time.h"

#include <string.h>
#include "ke_mem.h"


/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximum number of Client task instances
/// Invalid Characterisitic index
#define ENVC_INVALID_IDX (0xFF)




/// Content of ENVC dummy bit field
enum envc_dummy_bf
{
    /// Characteristic Index
    ENVC_DUMMY_CHAR_IDX_MASK    = 0x00FF,
    ENVC_DUMMY_CHAR_IDX_LSB     = 0,
    /// Characteristic idx
    ENVC_DUMMY_OPERATION_MASK   = 0x3F00,
    ENVC_DUMMY_OPERATION_LSB    = 8,
    /// Trigger idx
    ENVC_DUMMY_TRIGGER_IDX_MASK = 0xC000,
    ENVC_DUMMY_TRIGGER_IDX_LSB  = 14,
};


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/// Environment variable for each Connections
typedef struct envc_cnx_env
{
    /// Peer database discovered handle mapping
    envc_es_content_t   es;
    /// counter used to check service uniqueness
    uint8_t             nb_svc;
    /// counter used to read characteristic information
    uint8_t             char_idx;
    /// Client is in discovering state
    bool                discover;
} envc_cnx_env_t;

/// Client environment variable
typedef struct envc_env
{
    /// profile environment
    prf_hdr_t            prf_env;
    /// Environment variable pointer for each connections
    envc_cnx_env_t*      p_env[HOST_CONNECTION_MAX];
    /// GATT User local identifier
    uint8_t              user_lid;
} envc_env_t;


/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// State machine used to retrieve Measurement service characteristics information
/// For the ENVP we will not used the state machine to get the Chars as there are
__STATIC const prf_char_def_t envc_es_char[ ] =
{
    [ENVP_ES_DESCRIPTOR_VALUE_CHANGED_CHAR] = { GATT_CHAR_DESCRIPTOR_VALUE_CHANGED, ATT_REQ(PRES, OPT), (PROP(I))                        },

    [ENVP_ES_APRNT_WIND_DIRECTION_CHAR]     = { GATT_CHAR_APRNT_WIND_DIRECTION,     ATT_REQ(PRES, OPT), (PROP(RD) | PROP(N) | PROP(EXT)) },

    [ENVP_ES_APRNT_WIND_SPEED_CHAR]         = { GATT_CHAR_APRNT_WIND_SPEED,         ATT_REQ(PRES, OPT), (PROP(RD) | PROP(N) | PROP(EXT)) },

    [ENVP_ES_DEW_POINT_CHAR]                = { GATT_CHAR_DEW_POINT,                ATT_REQ(PRES, OPT), (PROP(RD) | PROP(N) | PROP(EXT)) },

    [ENVP_ES_ELEVATION_CHAR]                = { GATT_CHAR_ELEVATION,                ATT_REQ(PRES, OPT), (PROP(RD) | PROP(N) | PROP(EXT)) },

    [ENVP_ES_GUST_FACTOR_CHAR]              = { GATT_CHAR_GUST_FACTOR,              ATT_REQ(PRES, OPT), (PROP(RD) | PROP(N) | PROP(EXT)) },

    [ENVP_ES_HEAT_INDEX_CHAR]               = { GATT_CHAR_HEAT_INDEX,               ATT_REQ(PRES, OPT), (PROP(RD) | PROP(N) | PROP(EXT)) },

    [ENVP_ES_HUMIDITY_CHAR]                 = { GATT_CHAR_HUMIDITY,                 ATT_REQ(PRES, OPT), (PROP(RD) | PROP(N) | PROP(EXT)) },

    [ENVP_ES_IRRADIANCE_CHAR]               = { GATT_CHAR_IRRADIANCE,               ATT_REQ(PRES, OPT), (PROP(RD) | PROP(N) | PROP(EXT)) },

    [ENVP_ES_POLLEN_CONC_CHAR]              = { GATT_CHAR_POLLEN_CONC,              ATT_REQ(PRES, OPT), (PROP(RD) | PROP(N) | PROP(EXT)) },

    [ENVP_ES_RAINFALL_CHAR]                 = { GATT_CHAR_RAINFALL,                 ATT_REQ(PRES, OPT), (PROP(RD) | PROP(N) | PROP(EXT)) },

    [ENVP_ES_PRESSURE_CHAR]                 = { GATT_CHAR_PRESSURE,                 ATT_REQ(PRES, OPT), (PROP(RD) | PROP(N) | PROP(EXT)) },

    [ENVP_ES_TEMPERATURE_CHAR]              = { GATT_CHAR_TEMPERATURE,              ATT_REQ(PRES, OPT), (PROP(RD) | PROP(N) | PROP(EXT)) },

    [ENVP_ES_TRUE_WIND_DIR_CHAR]            = { GATT_CHAR_TRUE_WIND_DIR,            ATT_REQ(PRES, OPT), (PROP(RD) | PROP(N) | PROP(EXT)) },

    [ENVP_ES_TRUE_WIND_SPEED_CHAR]          = { GATT_CHAR_TRUE_WIND_SPEED,          ATT_REQ(PRES, OPT), (PROP(RD) | PROP(N) | PROP(EXT)) },

    [ENVP_ES_UV_INDEX_CHAR]                 = { GATT_CHAR_UV_INDEX,                 ATT_REQ(PRES, OPT), (PROP(RD) | PROP(N) | PROP(EXT)) },

    [ENVP_ES_WIND_CHILL_CHAR]               = { GATT_CHAR_WIND_CHILL,               ATT_REQ(PRES, OPT), (PROP(RD) | PROP(N) | PROP(EXT)) },

    [ENVP_ES_BAR_PRES_TREND_CHAR]           = { GATT_CHAR_BAR_PRES_TREND,           ATT_REQ(PRES, OPT), (PROP(RD) | PROP(N) | PROP(EXT)) },

    [ENVP_ES_MAGN_DECLINE_CHAR]             = { GATT_CHAR_MAGN_DECLINE,             ATT_REQ(PRES, OPT), (PROP(RD) | PROP(N) | PROP(EXT)) },

    [ENVP_ES_MAGN_FLUX_2D_CHAR]             = { GATT_CHAR_MAGN_FLUX_2D,             ATT_REQ(PRES, OPT), (PROP(RD) | PROP(N) | PROP(EXT)) },

    [ENVP_ES_MAGN_FLUX_3D_CHAR]             = { GATT_CHAR_MAGN_FLUX_3D,             ATT_REQ(PRES, OPT), (PROP(RD) | PROP(N) | PROP(EXT)) },
};

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Given a UUID return the ES Char ID.
 *
 * @param[in]  p_uuid     UUID value
 * @param[in]  uuid_type  UUID type (see enum #gatt_uuid_type)
 *
 * @return es_char_id - identified for the characteristic.
 ****************************************************************************************
 */
__STATIC uint8_t envc_map_uuid_to_char_id(const uint8_t* p_uuid, uint8_t uuid_type)
{
    uint8_t idx;
    uint8_t char_idx = ENVC_INVALID_IDX;

    for(idx = 0 ; idx <= ENVP_ES_MAGN_FLUX_3D_CHAR ; idx++)
    {
        if (gatt_uuid16_comp(p_uuid, uuid_type, envc_es_char[idx].uuid))
        {
            char_idx = idx;
            break;
        }
    }

    return char_idx;
}

/**
 ****************************************************************************************
 * @brief Gets a new instance number for an ES characteristic.
 *
 * @param[in] es_char_id -- an identifier for the ES characteristic
 * @param[in] p_chars - Pointer to the array describing each of the ES characteristic
 * @return instance number for the characteristic.
 ****************************************************************************************
 */
__STATIC uint8_t envc_get_char_instance_number(uint8_t es_char_id, envc_es_char_info_t *p_chars)
{
    uint8_t char_idx = 0;
    uint8_t instance = 0;

    while((p_chars[char_idx].es_char_hdl != GATT_INVALID_HDL) && (char_idx < ENVC_ES_MAX_CHARS_ALLOWED))
    {
        if (p_chars[char_idx].es_char_id == es_char_id)
        {
            instance++;
        }

        char_idx++;
    }

    return instance;
}


/**
 ****************************************************************************************
 * @brief Returns attribute value handle for Environmental Sensing Characteristic Descriptor
 * given the char Id and instance and the descriptor type.
 *
 * @param[in] conidx          identifier for the connection.
 * @param[in] operation       requested operation
 * @param[in] es_char_id      identifier of the required char.
 * @param[in] es_char_inst    instance of the required char.
 * @param[in] trigger_idx     Trigger index
 * @param[in] p_char_idx      Pointer to the characteristic index
 *
 * @return attribute handle
 ****************************************************************************************
 */
__STATIC uint16_t envc_find_hdl(uint8_t conidx, uint8_t operation, uint8_t es_char_id, uint8_t es_char_inst,
                                uint8_t trigger_idx, uint8_t* p_char_idx)
{
    // First find the char
    uint8_t char_idx;
    envc_env_t *p_envc_env = PRF_ENV_GET(ENVC, envc);
    envc_es_char_info_t *p_chars = p_envc_env->p_env[conidx]->es.chars;
    uint16_t handle = GATT_INVALID_HDL;

    for(char_idx = 0 ; (char_idx < ENVC_ES_MAX_CHARS_ALLOWED) ; char_idx++)
    {
        if (   (p_chars[char_idx].es_char_hdl != GATT_INVALID_HDL) && (p_chars[char_idx].es_char_id == es_char_id)
            && (p_chars[char_idx].es_char_inst == es_char_inst))
        {
            envc_es_char_info_t *p_current_char = &p_chars[char_idx];

            switch(operation)
            {
                case ENVC_WRITE_TRIGGER_OP_CODE:
                case ENVC_READ_TRIGGER_OP_CODE:
                {
                    if(trigger_idx == 0)
                    {
                        handle = p_current_char->trigger_1_hdl;
                    }
                    else if (trigger_idx == 1)
                    {
                        handle = p_current_char->trigger_2_hdl;
                    }
                    else
                    {
                        handle = p_current_char->trigger_3_hdl;
                    }
                } break;
                case ENVC_READ_VALUE_OP_CODE:            { handle = p_chars[char_idx].es_val_hdl;          } break;
                case ENVC_WRITE_CCC_OP_CODE:
                case ENVC_READ_CCC_OP_CODE:              { handle = p_current_char->ccc_desc_hdl;          } break;
                case ENVC_READ_MEAS_DESCRIPTOR_OP_CODE:  { handle = p_current_char->meas_desc_hdl;         } break;
                case ENVC_WRITE_CONFIG_OP_CODE:
                case ENVC_READ_CONFIG_OP_CODE:           { handle = p_current_char->config_desc_hdl;       } break;
                case ENVC_WRITE_USER_DESCRIPTION_OP_CODE:
                case ENVC_READ_USER_DESCRIPTION_OP_CODE: { handle = p_current_char->user_description_hdl;  } break;
                case ENVC_READ_RANGE_OP_CODE:            { handle = p_current_char->range_desc_hdl;        } break;
                case ENVC_READ_EXT_PROP_OP_CODE:         { handle = p_current_char->exp_prop_desc_hdl;     } break;
                default:                                 { handle = GATT_INVALID_HDL;                      } break;
            }

            *p_char_idx = char_idx;
            break;
        }
    }

    return handle;
}

/**
 ****************************************************************************************
 * @brief Returns a structure for Environmental Sensing Characteristic given the Attribute handle.
 *        Generic event received after every simple read command sent to peer server.
 *
 * @param[in] handle -  attribute handle of the required char.
 * @param[in] p_chars - Pointer to the array describing each of the ES characteristics.
 * @return pointer to envc_es_char_info
 ****************************************************************************************
 */
envc_es_char_info_t* envc_find_es_char_info(uint16_t handle, envc_es_char_info_t* p_chars)
{
    envc_es_char_info_t* p_res = NULL;
    uint8_t char_idx = 0;

    while (char_idx < ENVC_ES_MAX_CHARS_ALLOWED)
    {
        if (    (p_chars[char_idx].es_char_hdl != GATT_INVALID_HDL) && (p_chars[char_idx].es_end_hdl >= handle)
             && (p_chars[char_idx].es_char_hdl <= handle))
        {
            p_res = &p_chars[char_idx];
            break;
        }
        else
        {
            char_idx++;
        }
    }

    return (p_res);
}

/**
 ****************************************************************************************
 * @brief UnPack Characteristic value depending on Characteristic type
 *
 * @param[in]      es_char_type   Characteristic Type
 * @param[in]      p_buf          pointer to input data buffer
 * @param[out]     p_value        pointer to output ES Measurement Union
 *
 * @return status of the operation.
 ****************************************************************************************
 */
uint16_t envc_unpack_es_value(uint8_t es_char_id, co_buf_t* p_buf, union envp_val_char *p_value)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    switch(es_char_id)
    {

        case ENVP_ES_APRNT_WIND_DIRECTION_CHAR: { p_value->apparent_wind_direction = co_btohs(co_read16p(co_buf_data(p_buf)));              } break;
        case ENVP_ES_APRNT_WIND_SPEED_CHAR:     { p_value->apparent_wind_speed = co_btohs(co_read16p(co_buf_data(p_buf)));                  } break;
        case ENVP_ES_DEW_POINT_CHAR:            { p_value->dew_point = (int8_t)co_buf_data(p_buf)[0];                                       } break;
        case ENVP_ES_ELEVATION_CHAR:            { p_value->elevation = (((int32_t)co_btoh24(co_read24p(co_buf_data(p_buf))) << 8) >> 8);    } break;
        case ENVP_ES_GUST_FACTOR_CHAR:          { p_value->gust_factor = (uint8_t)co_buf_data(p_buf)[0];                                    } break;
        case ENVP_ES_HEAT_INDEX_CHAR:           { p_value->heat_index = (int8_t)co_buf_data(p_buf)[0];                                      } break;
        case ENVP_ES_HUMIDITY_CHAR:             { p_value->humidity = co_btohs(co_read16p(co_buf_data(p_buf)));                             } break;
        case ENVP_ES_IRRADIANCE_CHAR:           { p_value->irradiance = co_btohs(co_read16p(co_buf_data(p_buf)));                           } break;
        case ENVP_ES_POLLEN_CONC_CHAR:          { p_value->pollen_concentration = (uint32_t)co_btoh24(co_read24p(co_buf_data(p_buf)));      } break;
        case ENVP_ES_RAINFALL_CHAR:             { p_value->rainfall = co_btohs(co_read16p(co_buf_data(p_buf)));                             } break;
        case ENVP_ES_PRESSURE_CHAR:             { p_value->pressure = co_btohl(co_read32p(co_buf_data(p_buf)));                             } break;
        case ENVP_ES_TEMPERATURE_CHAR:          { p_value->temperature = (int16_t)co_btohs(co_read16p(co_buf_data(p_buf)));                 } break;
        case ENVP_ES_TRUE_WIND_DIR_CHAR:        { p_value->true_wind_direction = co_btohs(co_read16p(co_buf_data(p_buf)));                  } break;
        case ENVP_ES_TRUE_WIND_SPEED_CHAR:      { p_value->true_wind_speed = co_btohs(co_read16p(co_buf_data(p_buf)));                      } break;
        case ENVP_ES_UV_INDEX_CHAR:             { p_value->uv_index =  co_buf_data(p_buf)[0];                                               } break;
        case ENVP_ES_WIND_CHILL_CHAR:           { p_value->wind_chill = (int8_t) co_buf_data(p_buf)[0];                                     } break;
        case ENVP_ES_BAR_PRES_TREND_CHAR:       { p_value->barometric_pressure_trend = co_buf_data(p_buf)[0];                               } break;
        case ENVP_ES_MAGN_DECLINE_CHAR:         { p_value->magnetic_declination = co_btohs(co_read16p(co_buf_data(p_buf)));                 } break;

        case ENVP_ES_MAGN_FLUX_2D_CHAR:
        {
            p_value->mag_flux_dens_2d.x_axis = (int16_t)co_btohs(co_read16p(co_buf_data(p_buf)));
            co_buf_head_release(p_buf, 2);
            p_value->mag_flux_dens_2d.y_axis = (int16_t)co_btohs(co_read16p(co_buf_data(p_buf)));
        } break;

        case ENVP_ES_MAGN_FLUX_3D_CHAR:
        {
            p_value->mag_flux_dens_3d.x_axis = (int16_t)co_btohs(co_read16p(co_buf_data(p_buf)));
            co_buf_head_release(p_buf, 2);
            p_value->mag_flux_dens_3d.y_axis = (int16_t)co_btohs(co_read16p(co_buf_data(p_buf)));
            co_buf_head_release(p_buf, 2);
            p_value->mag_flux_dens_3d.z_axis = (int16_t)co_btohs(co_read16p(co_buf_data(p_buf)));
        } break;

        case ENVP_ES_DESCRIPTOR_VALUE_CHANGED_CHAR: {  /* Nothing to do */                  } break;
        default:                                    { status = ATT_ERR_ATTRIBUTE_NOT_FOUND; } break;

    }

    return status;
}

/**
 ****************************************************************************************
 * @brief Send discovery results to application.
 *
 * @param p_envc_env    Client Role task environment
 * @param conidx        Connection index
 * @param status        Response status code
 *****************************************************************************************
 */
__STATIC void envc_enable_cmp(envc_env_t* p_envc_env, uint8_t conidx, uint16_t status)
{
    const envc_cb_t* p_cb = (const envc_cb_t*) p_envc_env->prf_env.p_cb;

    if(p_envc_env != NULL)
    {
        envc_cnx_env_t* p_con_env = p_envc_env->p_env[conidx];

        p_cb->cb_enable_cmp(conidx, status, &(p_con_env->es));

        if (status != GAP_ERR_NO_ERROR)
        {
            // clean-up environment variable allocated for task instance
            ke_free(p_con_env);
            p_envc_env->p_env[conidx] = NULL;
        }
        else
        {
             p_con_env->discover = false;

             // Register profile handle to catch gatt indications
             gatt_cli_event_register(conidx, p_envc_env->user_lid, p_con_env->es.svc.shdl,
                                     p_con_env->es.svc.ehdl);
        }
    }
}

/**
 ****************************************************************************************
 * @brief Send read result to application,.
 *
 * @param[in] conidx        Connection index
 * @param[in] status        Status of the request
 * @param[in] dummy         Bit field that contains operation information
 * @param[in] length        Length of data value
 * @param[in] p_data        Pointer of buffer that contains data value
 ****************************************************************************************
 */
__STATIC void envc_read_attribute_cmp(uint8_t conidx, uint16_t status, uint16_t dummy, co_buf_t* p_data)
{
    envc_env_t* p_envc_env = PRF_ENV_GET(ENVC, envc);

    uint8_t char_idx    = GETF(dummy, ENVC_DUMMY_CHAR_IDX);
    uint8_t operation   = GETF(dummy, ENVC_DUMMY_OPERATION);
    uint8_t trigger_idx = GETF(dummy, ENVC_DUMMY_TRIGGER_IDX);

    if(p_envc_env != NULL)
    {
        const envc_cb_t* p_cb = (const envc_cb_t*) p_envc_env->prf_env.p_cb;
        envc_es_char_info_t *p_char = &(p_envc_env->p_env[conidx]->es.chars[char_idx]);
        uint8_t es_char_id   = p_char->es_char_id;
        uint8_t es_char_inst = p_char->es_char_inst;


        switch (operation)
        {
            case ENVC_READ_VALUE_OP_CODE:
            {
                union envp_val_char value;
                memset(&value, 0, sizeof(union envp_val_char));
                if(status == GAP_ERR_NO_ERROR)
                {
                    envc_unpack_es_value(es_char_id, p_data, &value);
                }

                p_cb->cb_read_value_cmp(conidx, status, es_char_id, es_char_inst, &value);
            } break;

            case ENVC_READ_CCC_OP_CODE :
            {
                uint16_t ccc = 0;

                if(status == GAP_ERR_NO_ERROR)
                {
                    ccc = co_btohs(co_read16p(co_buf_data(p_data)));
                }

                p_cb->cb_read_ccc_cmp(conidx, status, es_char_id, es_char_inst, ccc);
            } break;

            case ENVC_READ_MEAS_DESCRIPTOR_OP_CODE :
            {
                envp_es_meas_desc_t meas;
                memset(&meas, 0, sizeof(envp_es_meas_desc_t));

                if(status == GAP_ERR_NO_ERROR)
                {
                     meas.flags             = co_btohs(co_read16p(co_buf_data(p_data)));
                     co_buf_head_release(p_data, 2);
                     meas.samp_func         = co_buf_data(p_data)[0];
                     co_buf_head_release(p_data, 1);
                     meas.meas_period       = co_btoh24(co_read24p(co_buf_data(p_data)));
                     co_buf_head_release(p_data, 3);
                     meas.update_period     = co_btoh24(co_read24p(co_buf_data(p_data)));
                     co_buf_head_release(p_data, 3);
                     meas.app               = co_buf_data(p_data)[0];
                     co_buf_head_release(p_data, 1);
                     meas.meas_uncertainty  = co_buf_data(p_data)[0];
                     co_buf_head_release(p_data, 1);
                }

                p_cb->cb_read_meas_desc_cmp(conidx, status, es_char_id, es_char_inst, &meas);
            } break;

            case ENVC_READ_CONFIG_OP_CODE :
            {
                uint8_t trigger_logic = 0;

                if(status == GAP_ERR_NO_ERROR)
                {
                     trigger_logic = co_buf_data(p_data)[0];
                }

                p_cb->cb_read_config_cmp(conidx, status, es_char_id, es_char_inst, trigger_logic);
            } break;

            case ENVC_READ_USER_DESCRIPTION_OP_CODE :
            {
                p_cb->cb_read_user_desc_cmp(conidx, status, es_char_id, es_char_inst, p_data);
            } break;

            case ENVC_READ_TRIGGER_OP_CODE :
            {
                envp_trigger_t trigger;
                memset(&trigger, 0, sizeof(envp_trigger_t));

                if(status == GAP_ERR_NO_ERROR)
                {
                    trigger.condition               = co_buf_data(p_data)[0];
                    co_buf_head_release(p_data, 1);


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
                    if (trigger.condition < ENVP_TRIG_VALUE_LESS)
                    {
                        // If a Timer based trigger (condition == 1 or condition == 2) - setup the timer value
                        if (   (trigger.condition == ENVP_TRIG_INTERVAL_FIXED)
                            || (trigger.condition == ENVP_TRIG_INTERVAL_NO_LESS_THAN))
                        {
                            trigger.time_based_trigger = co_btoh24(co_read24p(co_buf_data(p_data)));
                            co_buf_head_release(p_data, 3);
                        }
                    }
                    // If measurement based trigger  - if condition is between 4 & 9.
                    else if (   (trigger.condition > ENVP_TRIG_PREV_VALUE_DIFFERENT)
                             && (trigger.condition < ENVP_TRIG_RESERVED))
                    {
                        // In this case the size of the operand is dependant on the size of the
                        // value field of the measurement characteristic.

                        envc_unpack_es_value(es_char_id, p_data, &(trigger.meas_value_trigger));
                    }
                }

                p_cb->cb_read_trigger_cmp(conidx, status, es_char_id, es_char_inst, trigger_idx, &trigger);
            } break;

            case ENVC_READ_RANGE_OP_CODE :
            {
                union envp_range range;
                memset(&range, 0, sizeof(union envp_range));

                if(status == GAP_ERR_NO_ERROR)
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
                            range.u16.lower       = co_btohs(co_read16p(co_buf_data(p_data)));
                            co_buf_head_release(p_data, 2);
                            range.u16.upper       = co_btohs(co_read16p(co_buf_data(p_data)));
                            co_buf_head_release(p_data, 2);
                        } break;

                        // uint24
                        case ENVP_ES_POLLEN_CONC_CHAR:
                        {
                            range.u32.lower       = co_btoh24(co_read24p(co_buf_data(p_data)));
                            co_buf_head_release(p_data, 3);
                            range.u32.upper       = co_btoh24(co_read24p(co_buf_data(p_data)));
                            co_buf_head_release(p_data, 3);
                        } break;

                        // now the uint32 chars
                        case ENVP_ES_PRESSURE_CHAR:
                        {
                            range.u32.lower       = co_btohl(co_read32p(co_buf_data(p_data)));
                            co_buf_head_release(p_data, 4);
                            range.u32.upper       = co_btohl(co_read32p(co_buf_data(p_data)));
                            co_buf_head_release(p_data, 4);
                        } break;

                        // the int24 chars
                        case ENVP_ES_ELEVATION_CHAR:
                        {
                            range.s32.lower       = (((int32_t) co_btoh24(co_read24p(co_buf_data(p_data)))) << 8) >> 8;
                            co_buf_head_release(p_data, 3);
                            range.s32.upper       = (((int32_t) co_btoh24(co_read24p(co_buf_data(p_data)))) << 8) >> 8;
                            co_buf_head_release(p_data, 3);
                        } break;

                        // int16
                        case ENVP_ES_MAGN_FLUX_2D_CHAR:
                        case ENVP_ES_MAGN_FLUX_3D_CHAR:
                        case ENVP_ES_TEMPERATURE_CHAR:
                        {
                            range.s16.lower       = (int16_t) co_btohs(co_read16p(co_buf_data(p_data)));
                            co_buf_head_release(p_data, 2);
                            range.s16.upper       = (int16_t) co_btohs(co_read16p(co_buf_data(p_data)));
                            co_buf_head_release(p_data, 2);
                        } break;

                        // uint8
                        case ENVP_ES_GUST_FACTOR_CHAR:
                        case ENVP_ES_UV_INDEX_CHAR:
                        case ENVP_ES_BAR_PRES_TREND_CHAR:
                        {
                            range.u8.lower       = co_buf_data(p_data)[0];
                            co_buf_head_release(p_data, 1);
                            range.u8.upper       = co_buf_data(p_data)[0];
                            co_buf_head_release(p_data, 1);
                        } break;

                        // int8
                        case ENVP_ES_DEW_POINT_CHAR:
                        case ENVP_ES_HEAT_INDEX_CHAR:
                        case ENVP_ES_WIND_CHILL_CHAR:
                        {
                            range.s8.lower       = (int8_t) co_buf_data(p_data)[0];
                            co_buf_head_release(p_data, 1);
                            range.s8.upper       = (int8_t) co_buf_data(p_data)[0];
                            co_buf_head_release(p_data, 1);
                        } break;

                        default: { /* Nothing to do */ } break;
                    }
                }

                p_cb->cb_read_value_range_cmp(conidx, status, es_char_id, es_char_inst, &range);
            } break;

            case ENVC_READ_EXT_PROP_OP_CODE :
            {
                uint16_t ext_prop = 0;

                if(status == GAP_ERR_NO_ERROR)
                {
                    ext_prop = co_btohs(co_read16p(co_buf_data(p_data)));
                }

                p_cb->cb_read_ext_prop_cmp(conidx, status, es_char_id, es_char_inst, ext_prop);
            } break;

            default:
            {
                ASSERT_ERR(0);
            } break;
        }
    }
}

/**
 ****************************************************************************************
 * @brief Perform Attribute read procedure.
 *
 * @param[in] conidx        Connection index
 * @param[in] operation     Requested operation
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 * @param[in] trigger_idx   Descriptor idx @ref envs_desc_idx (TRIG1 TRIG2 TRIG3) 0,1,2
 *
 * @return Status of procedure start
 ****************************************************************************************
 */
__STATIC uint16_t envc_read_attribute(uint8_t conidx, uint8_t operation, uint8_t es_char_id, uint8_t es_char_inst,
                                      uint8_t trigger_idx)
{
    uint16_t status = PRF_ERR_REQ_DISALLOWED;
    // Client environment
    envc_env_t* p_envc_env = PRF_ENV_GET(ENVC, envc);

    if (es_char_id >= ENVP_ES_CHAR_MAX)
    {
        status = PRF_APP_ERROR;
    }
    else if(p_envc_env != NULL)
    {
        if ((conidx < HOST_CONNECTION_MAX) && (p_envc_env->p_env[conidx] != NULL) && (!p_envc_env->p_env[conidx]->discover))
        {
            uint16_t hdl;
            uint8_t  char_idx = 0;

            hdl = envc_find_hdl(conidx, operation, es_char_id, es_char_inst, trigger_idx, &char_idx);

            if(hdl == GATT_INVALID_HDL)
            {
                status = PRF_ERR_INEXISTENT_HDL;
            }
            else
            {
                uint16_t dummy = 0;
                SETF(dummy, ENVC_DUMMY_CHAR_IDX,    char_idx);
                SETF(dummy, ENVC_DUMMY_OPERATION,   operation);
                SETF(dummy, ENVC_DUMMY_TRIGGER_IDX, trigger_idx);

                // perform read request
                status = gatt_cli_read(conidx, p_envc_env->user_lid, dummy, hdl, 0, 0);
            }
        }
    }

    return (status);
}



/**
 ****************************************************************************************
 * @brief Perform Attribute write procedure.
 *
 * @param[in] conidx        Connection index
 * @param[in] operation     Requested operation
 * @param[in] es_char_id    Characteristic Type idx see enum #envp_es_char
 * @param[in] es_char_inst  Characteristic Instance
 * @param[in] trigger_idx   Descriptor idx @ref envs_desc_idx (TRIG1 TRIG2 TRIG3) 0,1,2
 * @param[in] p_buf         Buffer that contains value to write
 *
 * @return Status of procedure start
 ****************************************************************************************
 */
__STATIC uint16_t envc_write_attribute(uint8_t conidx, uint8_t operation, uint8_t es_char_id, uint8_t es_char_inst,
                                       uint8_t trigger_idx, co_buf_t* p_buf)
{
    uint16_t status = PRF_ERR_REQ_DISALLOWED;
    // Client environment
    envc_env_t* p_envc_env = PRF_ENV_GET(ENVC, envc);

    if (es_char_id >= ENVP_ES_CHAR_MAX)
    {
        status = PRF_APP_ERROR;
    }
    else if(p_envc_env != NULL)
    {
        if ((conidx < HOST_CONNECTION_MAX) && (p_envc_env->p_env[conidx] != NULL) && (!p_envc_env->p_env[conidx]->discover))
        {
            uint16_t hdl;
            uint8_t  char_idx = 0;

            hdl = envc_find_hdl(conidx, operation, es_char_id, es_char_inst, trigger_idx, &char_idx);

            if(hdl == GATT_INVALID_HDL)
            {
                status = PRF_ERR_INEXISTENT_HDL;
            }
            else
            {
                uint16_t dummy = 0;
                SETF(dummy, ENVC_DUMMY_CHAR_IDX,    char_idx);
                SETF(dummy, ENVC_DUMMY_OPERATION,   operation);
                SETF(dummy, ENVC_DUMMY_TRIGGER_IDX, trigger_idx);

                // perform write request
                status = gatt_cli_write(conidx, p_envc_env->user_lid, dummy, GATT_WRITE, hdl, 0, p_buf);
            }
        }
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Checks the validity of discovered Characteristic descriptors for a given characteristic
 *
 * @param[in] conidx -- connection identifier
 * @param[in] p_current_char -- pointer to structure describing discovered characteristic incl descriptors
 * @param[in] p_char_list - Pointer to the array describing each of the ES characteristic
 *
 * @return Error_Code
 ****************************************************************************************
 */
uint16_t envc_check_svc_char_desc_validity(uint8_t conidx, envc_es_char_info_t *p_current_char,
                                           envc_es_char_info_t *p_char_list)
{
    uint16_t status = GAP_ERR_NO_ERROR;

    // First check the Characteristic Declaration
    uint8_t num_triggers = 0;
    uint8_t num_instant = 0;

    // First check num triggers available - if more than one then Config Descriptor is mandatory
    if (p_current_char->trigger_1_hdl != GATT_INVALID_HDL)
    {
        num_triggers++;
    }

    if (p_current_char->trigger_2_hdl != GATT_INVALID_HDL)
    {
        num_triggers++;
    }

    if (p_current_char->trigger_3_hdl != GATT_INVALID_HDL)
    {
        num_triggers++;
    }

    if (((num_triggers == 0) && (p_current_char->prop & PROP(N))) ||
        ((p_current_char->prop & PROP(N)) && (p_current_char->ccc_desc_hdl == GATT_INVALID_HDL)) ||
        ((num_triggers > 1) && (p_current_char->config_desc_hdl == GATT_INVALID_HDL)))
    {
        status = PRF_ERR_STOP_DISC_CHAR_MISSING;
    }

    if (status == GAP_ERR_NO_ERROR)
    {
        // If more than one instant of the Char exist then the measurement descriptor is required
        num_instant = envc_get_char_instance_number(p_current_char->es_char_id, p_char_list);

        if ((num_instant > 1) && (p_current_char->meas_desc_hdl == GATT_INVALID_HDL))
        {
            status = PRF_ERR_STOP_DISC_CHAR_MISSING;
        }
    }

    return status;
}


/*
 * GATT USER CLIENT HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief This function is called when a full service has been found during a discovery procedure.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] hdl           First handle value of following list
 * @param[in] disc_info     Discovery information (see enum #gatt_svc_disc_info)
 * @param[in] nb_att        Number of attributes
 * @param[in] p_atts        Pointer to attribute information present in a service
 ****************************************************************************************
 */
__STATIC void envc_svc_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t hdl, uint8_t disc_info,
                          uint8_t nb_att, const gatt_svc_att_t* p_atts)
{
    // Get the address of the environment
    envc_env_t* p_envc_env = PRF_ENV_GET(ENVC, envc);

    if(p_envc_env != NULL)
    {
        envc_cnx_env_t* p_con_env = p_envc_env->p_env[conidx];

        ASSERT_INFO(p_con_env != NULL, conidx, user_lid);

        if (p_con_env->nb_svc == 0)
        {
            uint8_t att_idx;
            envc_es_char_info_t *p_current_char = NULL;

            if((p_con_env->char_idx != ENVC_INVALID_IDX) && (p_con_env->char_idx < ENVC_ES_MAX_CHARS_ALLOWED))
            {
               p_current_char = &p_con_env->es.chars[p_con_env->char_idx];
            }

            // Store only 1 range even if several ones have been received
            if((disc_info == GATT_SVC_CMPLT) || (disc_info == GATT_SVC_START))
            {
                p_con_env->es.svc.shdl = hdl;
            }

            if((disc_info == GATT_SVC_CMPLT) || (disc_info == GATT_SVC_END))
            {
                p_con_env->es.svc.ehdl = hdl + nb_att -1;
            }

            for (att_idx = 0; att_idx < nb_att ; att_idx++)
            {
                uint16_t att_hdl = hdl + att_idx;
                // extract characteristic information
                if(p_atts[att_idx].att_type == GATT_ATT_CHAR)
                {
                    uint8_t es_char_id = envc_map_uuid_to_char_id(p_atts[att_idx].uuid, p_atts[att_idx].uuid_type);

                    if((p_con_env->char_idx != ENVC_INVALID_IDX) && (p_current_char->es_end_hdl == GATT_INVALID_HDL))
                    {
                        p_current_char->es_end_hdl = att_hdl - 1;
                    }

                    if(es_char_id != ENVC_INVALID_IDX)
                    {
                        p_con_env->char_idx++;
                        p_con_env->es.nb_chars_discovered++;

                        if(p_con_env->char_idx < ENVC_ES_MAX_CHARS_ALLOWED)
                        {
                            p_current_char = &p_con_env->es.chars[p_con_env->char_idx];
                            p_current_char->es_char_id   = es_char_id;
                            p_current_char->es_char_inst = envc_get_char_instance_number(es_char_id, p_con_env->es.chars);

                            p_current_char->es_char_hdl = att_hdl;
                            p_current_char->es_val_hdl  = p_atts[att_idx].info.charac.val_hdl;
                            p_current_char->prop        = p_atts[att_idx].info.charac.prop;
                        }
                    }
                }
                else if(   (p_atts[att_idx].att_type == GATT_ATT_DESC)
                        && (p_con_env->char_idx != ENVC_INVALID_IDX) && (p_con_env->char_idx < ENVC_ES_MAX_CHARS_ALLOWED)
                        && (p_atts[att_idx].uuid_type == GATT_UUID_16))
                {
                    // retrieve descriptor information
                    uint16_t uuid16 = co_read16p(p_atts[att_idx].uuid);
                    switch(uuid16)
                    {
                        case GATT_DESC_CLIENT_CHAR_CFG:       { p_current_char->ccc_desc_hdl         = att_hdl; } break;
                        case GATT_DESC_ES_MEASUREMENT:        { p_current_char->meas_desc_hdl        = att_hdl; } break;
                        case GATT_DESC_ES_CONFIGURATION:      { p_current_char->config_desc_hdl      = att_hdl; } break;
                        case GATT_DESC_CHAR_USER_DESCRIPTION: { p_current_char->user_description_hdl = att_hdl; } break;
                        case GATT_DESC_VALID_RANGE:           { p_current_char->range_desc_hdl       = att_hdl; } break;
                        case GATT_DESC_CHAR_EXT_PROPERTIES:   { p_current_char->exp_prop_desc_hdl    = att_hdl; } break;
                        case GATT_DESC_ES_TRIGGER_SETTING:
                        {
                            if (p_current_char->trigger_1_hdl == GATT_INVALID_HDL)
                            {
                                p_current_char->trigger_1_hdl = att_hdl;
                            }
                            else if (p_current_char->trigger_2_hdl == GATT_INVALID_HDL)
                            {
                                p_current_char->trigger_2_hdl = att_hdl;
                            }
                            else if (p_current_char->trigger_3_hdl == GATT_INVALID_HDL)
                            {
                                p_current_char->trigger_3_hdl = att_hdl;
                            }
                        } break;
                        default:{  /* Nothing to do */ } break;
                    }
                }
            }

            // fill end handle of last characteristic
            if((disc_info == GATT_SVC_CMPLT) || (disc_info == GATT_SVC_END))
            {
                if((p_con_env->char_idx != ENVC_INVALID_IDX) && (p_current_char->es_end_hdl == GATT_INVALID_HDL))
                {
                    if(p_current_char != NULL)
                    {
                        p_current_char->es_end_hdl = hdl + nb_att - 1;
                    }
                }
            }

        }

        if((disc_info == GATT_SVC_CMPLT) || (disc_info == GATT_SVC_END))
        {
            p_con_env->nb_svc++;
        }
    }
}

/**
 ****************************************************************************************
 * @brief This function is called when GATT client user discovery procedure is over.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] status        Status of the procedure (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC void envc_discover_cmp_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status)
{
    // Get the address of the environment
    envc_env_t* p_envc_env = PRF_ENV_GET(ENVC, envc);

    if(p_envc_env != NULL)
    {
        envc_cnx_env_t* p_con_env = p_envc_env->p_env[conidx];

        if (p_con_env->nb_svc ==  1)
        {
            uint8_t char_idx;

            // browse all found characteristics
            for(char_idx = 0 ; char_idx < p_con_env->es.nb_chars_discovered ; char_idx++)
            {
                // Check that the Characteristic and descriptors are valid
                // Ensure all descriptors are in place.
                status = envc_check_svc_char_desc_validity(conidx, &(p_con_env->es.chars[char_idx]), &(p_con_env->es.chars[0]));
                if(status != GAP_ERR_NO_ERROR) break;
            }
        }
        // too much services
        else if (p_con_env->nb_svc > 1)
        {
            status = PRF_ERR_MULTIPLE_SVC;
        }
        // no services found
        else
        {
            status = PRF_ERR_STOP_DISC_CHAR_MISSING;
        }

        envc_enable_cmp(p_envc_env, conidx, status);
    }
}

/**
 ****************************************************************************************
 * @brief This function is called during a read procedure when attribute value is retrieved
 *        form peer device.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] hdl           Attribute handle
 * @param[in] offset        Data offset
 * @param[in] p_data        Pointer to buffer that contains attribute value starting from offset
 ****************************************************************************************
 */
__STATIC void envc_att_val_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t hdl, uint16_t offset,
                              co_buf_t* p_data)
{
    envc_read_attribute_cmp(conidx, GAP_ERR_NO_ERROR, dummy, p_data);
}

/**
 ****************************************************************************************
 * @brief This function is called when GATT client user read procedure is over.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] status        Status of the procedure (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC void envc_read_cmp_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status)
{
    if(status != GAP_ERR_NO_ERROR)
    {
        envc_read_attribute_cmp(conidx, status, dummy, NULL);
    }
}

/**
 ****************************************************************************************
 * @brief This function is called when GATT client user write procedure is over.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] status        Status of the procedure (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC void envc_write_cmp_cb(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status)
{
    envc_env_t* p_envc_env = PRF_ENV_GET(ENVC, envc);

    if(p_envc_env != NULL)
    {
        const envc_cb_t* p_cb = (const envc_cb_t*) p_envc_env->prf_env.p_cb;
        uint8_t char_idx    = GETF(dummy, ENVC_DUMMY_CHAR_IDX);
        uint8_t operation   = GETF(dummy, ENVC_DUMMY_OPERATION);
        uint8_t trigger_idx = GETF(dummy, ENVC_DUMMY_TRIGGER_IDX);

        envc_es_char_info_t *p_char = &(p_envc_env->p_env[conidx]->es.chars[char_idx]);
        uint8_t es_char_id   = p_char->es_char_id;
        uint8_t es_char_inst = p_char->es_char_inst;

        // inform application about end of write procedure
        switch(operation)
        {
            case ENVC_WRITE_CCC_OP_CODE:              { p_cb->cb_write_ccc_cmp(conidx, status, es_char_id, es_char_inst);                  } break;
            case ENVC_WRITE_CONFIG_OP_CODE:           { p_cb->cb_write_config_cmp(conidx, status, es_char_id, es_char_inst);               } break;
            case ENVC_WRITE_TRIGGER_OP_CODE:          { p_cb->cb_write_trigger_cmp(conidx, status, es_char_id, es_char_inst, trigger_idx); } break;
            case ENVC_WRITE_USER_DESCRIPTION_OP_CODE: { p_cb->cb_write_user_desc_cmp(conidx, status, es_char_id, es_char_inst);            } break;
            default:                                  { /* Nothing to do */                                                                } break;
        }
    }
}


/**
 ****************************************************************************************
 * @brief This function is called when a notification or an indication is received onto
 *        register handle range (see #gatt_cli_event_register).
 *
 *        #gatt_cli_val_event_cfm must be called to confirm event reception.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] token         Procedure token that must be returned in confirmation function
 * @param[in] evt_type      Event type triggered (see enum #gatt_evt_type)
 * @param[in] complete      True if event value if complete value has been received
 *                          False if data received is equals to maximum attribute protocol value.
 *                          In such case GATT Client User should perform a read procedure.
 * @param[in] hdl           Attribute handle
 * @param[in] p_data        Pointer to buffer that contains attribute value
 ****************************************************************************************
 */
__STATIC void envc_att_val_evt_cb(uint8_t conidx, uint8_t user_lid, uint16_t token, uint8_t evt_type, bool complete,
                                  uint16_t hdl, co_buf_t* p_data)
{
    envc_env_t* p_envc_env = PRF_ENV_GET(ENVC, envc);

    if(p_envc_env != NULL)
    {
        const envc_cb_t* p_cb = (const envc_cb_t*) p_envc_env->prf_env.p_cb;
        envc_cnx_env_t* p_con_env = p_envc_env->p_env[conidx];
        envc_es_content_t* p_es = &(p_con_env->es);
        envc_es_char_info_t *p_es_char;

        // retrieve characteristic
        p_es_char = envc_find_es_char_info(hdl, p_es->chars);

        if(p_es_char != NULL)
        {
            switch (evt_type)
            {
                case (GATT_NOTIFY):
                {
                    // Check that the Handle is for a Value and not one of the descriptors
                    if (   (p_es_char->es_val_hdl == hdl)
                        && (p_es_char->es_char_id > ENVP_ES_DESCRIPTOR_VALUE_CHANGED_CHAR)
                        && (p_es_char->es_char_id < ENVP_ES_CHAR_MAX))
                    {
                        union envp_val_char value;
                        memset(&value, 0, sizeof(union envp_val_char));

                        if (envc_unpack_es_value(p_es_char->es_char_id, p_data, &value) == GAP_ERR_NO_ERROR)
                        {
                            // inform application about value update
                            p_cb->cb_value(conidx, p_es_char->es_char_id, p_es_char->es_char_inst, &value);
                        }
                    }  // Ignore the Notify as Att-Handle is invalid
                } break;

                case (GATT_INDICATE):
                {
                    uint16_t uuid16;
                    uint16_t flags;
                    uint8_t  ind_es_char_id;

                    flags  = co_btohs(co_read16p(co_buf_data(p_data)));
                    co_buf_head_release(p_data, 2);
                    uuid16 = co_read16p(co_buf_data(p_data)); // keep uuid LSB first
                    ind_es_char_id = envc_map_uuid_to_char_id((uint8_t*) &uuid16, GATT_UUID_16);

                    if (   (p_es_char->es_val_hdl == hdl)
                        && (p_es_char->es_char_id == ENVP_ES_DESCRIPTOR_VALUE_CHANGED_CHAR)
                        && (ind_es_char_id < ENVP_ES_CHAR_MAX ))
                    {
                        // inform application about descriptor change
                        p_cb->cb_desc_chg(conidx, ind_es_char_id, flags);
                    } // else ignore event
                } break;

                default:  { /* Nothing to do */ } break;
            }
        }
    }

    // confirm event handling
    gatt_cli_att_event_cfm(conidx, user_lid, token);
}

/**
 ****************************************************************************************
 * @brief Event triggered when a service change has been received or if an attribute
 *        transaction triggers an out of sync error.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] out_of_sync   True if an out of sync error has been received
 * @param[in] start_hdl     Service start handle
 * @param[in] end_hdl       Service end handle
 ****************************************************************************************
 */
__STATIC void envc_svc_changed_cb(uint8_t conidx, uint8_t user_lid, bool out_of_sync, uint16_t start_hdl, uint16_t end_hdl)
{
    // Do Nothing
}

/// Client callback hander
__STATIC const gatt_cli_cb_t envc_cb =
{
    .cb_discover_cmp    = envc_discover_cmp_cb,
    .cb_read_cmp        = envc_read_cmp_cb,
    .cb_write_cmp       = envc_write_cmp_cb,
    .cb_att_val_get     = NULL,
    .cb_svc             = envc_svc_cb,
    .cb_svc_info        = NULL,
    .cb_inc_svc         = NULL,
    .cb_char            = NULL,
    .cb_desc            = NULL,
    .cb_att_val         = envc_att_val_cb,
    .cb_att_val_evt     = envc_att_val_evt_cb,
    .cb_svc_changed     = envc_svc_changed_cb,
};

/*
 * PROFILE NATIVE API
 ****************************************************************************************
 */

uint16_t envc_enable(uint8_t conidx, uint8_t con_type, const envc_es_content_t* p_es)
{
    uint16_t status = PRF_ERR_REQ_DISALLOWED;
    // Client environment
    envc_env_t* p_envc_env = PRF_ENV_GET(ENVC, envc);

    if(p_envc_env != NULL)
    {
        if ((conidx < HOST_CONNECTION_MAX) && (p_envc_env->p_env[conidx] == NULL))
        {
            // allocate environment variable for task instance
            p_envc_env->p_env[conidx] = (struct envc_cnx_env *) ke_malloc_user(sizeof(struct envc_cnx_env), KE_MEM_PROFILE);

            if(p_envc_env->p_env[conidx] != NULL)
            {
                memset(p_envc_env->p_env[conidx], 0, sizeof(struct envc_cnx_env));

                // Config connection, start discovering
                if (con_type == PRF_CON_DISCOVERY)
                {
                    uint16_t gatt_svc_uuid = GATT_SVC_ENVIRONMENTAL_SENSING;

                    // start discovery
                    status = gatt_cli_discover_svc(conidx, p_envc_env->user_lid, 0, GATT_DISCOVER_SVC_PRIMARY_BY_UUID, true,
                                                   GATT_MIN_HDL, GATT_MAX_HDL, GATT_UUID_16, (uint8_t*) &gatt_svc_uuid);

                    // Go to DISCOVERING state
                    p_envc_env->p_env[conidx]->discover   = true;
                    p_envc_env->p_env[conidx]->char_idx   = ENVC_INVALID_IDX;
                }
                // normal connection, get saved att details
                else
                {
                    memcpy(&(p_envc_env->p_env[conidx]->es), p_es, sizeof(envc_es_content_t));
                    status = GAP_ERR_NO_ERROR;

                    // send APP confirmation that can start normal connection to TH
                    envc_enable_cmp(p_envc_env, conidx, GAP_ERR_NO_ERROR);
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
//
//uint16_t envc_read_sensor_feat(uint8_t conidx)
//{
//    uint16_t status = envc_read_val(conidx, ENVC_RD_CP_FEAT);
//    return (status);
//}
//
//uint16_t envc_read_sensor_loc(uint8_t conidx)
//{
//    uint16_t status = envc_read_val(conidx, ENVC_RD_SENSOR_LOC);
//    return (status);
//}
//
//uint16_t envc_read_cfg(uint8_t conidx, uint8_t desc_code)
//{
//    uint16_t status;
//
//    switch(desc_code)
//    {
//        case ENVC_RD_WR_CP_MEAS_CL_CFG:
//        case ENVC_RD_WR_CP_MEAS_SV_CFG:
//        case ENVC_RD_WR_VECTOR_CFG:
//        case ENVC_RD_WR_CTNL_PT_CFG:    { status = envc_read_val(conidx, desc_code);  } break;
//        default:                        { status = PRF_ERR_INEXISTENT_HDL;            } break;
//    }
//
//    return (status);
//}
//
//uint16_t envc_write_cfg(uint8_t conidx, uint8_t desc_code, uint16_t cfg_val)
//{
//    uint16_t status = PRF_ERR_REQ_DISALLOWED;
//    // Client environment
//    envc_env_t* p_envc_env = PRF_ENV_GET(ENVC, envc);
//
//    if(p_envc_env != NULL)
//    {
//        if ((conidx < HOST_CONNECTION_MAX) && (p_envc_env->p_env[conidx] != NULL) && (!p_envc_env->p_env[conidx]->discover))
//        {
//            envc_cnx_env_t* p_con_env = p_envc_env->p_env[conidx];
//            uint16_t hdl;
//            uint16_t cfg_en_val = 0;
//            envc_es_content_t* p_es = &(p_con_env->es);
//
//            switch(desc_code)
//            {
//                case ENVC_RD_WR_CP_MEAS_CL_CFG: { hdl        = p_es->descs[ENVC_DESC_CP_MEAS_CL_CFG].desc_hdl;
//                                                  cfg_en_val =  PRF_CLI_START_NTF;                              } break;
//                case ENVC_RD_WR_CP_MEAS_SV_CFG: { hdl        = p_es->descs[ENVC_DESC_CP_MEAS_SV_CFG].desc_hdl;
//                                                  cfg_en_val =  PRF_SRV_START_BCST;                             } break;
//                case ENVC_RD_WR_VECTOR_CFG:     { hdl        = p_es->descs[ENVC_DESC_VECTOR_CL_CFG].desc_hdl;
//                                                  cfg_en_val =  PRF_CLI_START_NTF;                              } break;
//                case ENVC_RD_WR_CTNL_PT_CFG:    { hdl        = p_es->descs[ENVC_DESC_CTNL_PT_CL_CFG].desc_hdl;
//                                                  cfg_en_val =  PRF_CLI_START_IND;                              } break;
//                default:                        { hdl = GATT_INVALID_HDL;                                       } break;
//            }
//
//            if(hdl == GATT_INVALID_HDL)
//            {
//                status = PRF_ERR_INEXISTENT_HDL;
//            }
//            else if((cfg_val != PRF_CLI_STOP_NTFIND) && (cfg_val != cfg_en_val))
//            {
//                status = PRF_ERR_INVALID_PARAM;
//            }
//            else
//            {
//                // Force endianess
//                cfg_val = co_htobs(cfg_val);
//                status = prf_gatt_write(conidx, p_envc_env->user_lid, desc_code, GATT_WRITE,
//                                        hdl, sizeof(uint16_t), (uint8_t *)&cfg_val);
//            }
//        }
//    }
//
//    return (status);
//}
//
//uint16_t envc_ctnl_pt_req(uint8_t conidx, uint8_t req_op_code, const union cpp_ctnl_pt_req_val* p_value)
//{
//    uint16_t status = PRF_ERR_REQ_DISALLOWED;
//    // Client environment
//    envc_env_t* p_envc_env = PRF_ENV_GET(ENVC, envc);
//
//    if(p_value == NULL)
//    {
//        status = PRF_ERR_INVALID_PARAM;
//    }
//    else if(p_envc_env != NULL)
//    {
//        if ((conidx < HOST_CONNECTION_MAX) && (p_envc_env->p_env[conidx] != NULL) && (!p_envc_env->p_env[conidx]->discover))
//        {
//            envc_cnx_env_t* p_con_env = p_envc_env->p_env[conidx];
//            envc_es_content_t* p_es = &(p_con_env->es);
//            uint16_t hdl = p_es->chars[CPP_CPS_CTNL_PT_CHAR].val_hdl;
//
//            if(hdl == GATT_INVALID_HDL)
//            {
//                status = PRF_ERR_INEXISTENT_HDL;
//            }
//            // reject if there is an ongoing control point operation
//            else if(p_con_env->ctrl_pt_op != CPP_CTNL_PT_RESERVED)
//            {
//                status = PRF_ERR_REQ_DISALLOWED;
//            }
//            else
//            {
//                co_buf_t* p_buf = NULL;
//
//                // allocate buffer for event transmission
//                if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, 0, CPP_CP_CNTL_PT_REQ_MAX_LEN + GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
//                {
//                    status = envc_pack_ctnl_pt_req(p_buf, req_op_code, p_value);
//
//                    if(status == GAP_ERR_NO_ERROR)
//                    {
//                        status = gatt_cli_write(conidx, p_envc_env->user_lid, ENVC_IND_CTNL_PT, GATT_WRITE, hdl, 0, p_buf);
//                        if(status == GAP_ERR_NO_ERROR)
//                        {
//                            // save on-going operation
//                            p_con_env->ctrl_pt_op = req_op_code;
//                        }
//                    }
//                    co_buf_release(p_buf);
//                }
//                else
//                {
//                    status = GAP_ERR_INSUFF_RESOURCES;
//                }
//
//            }
//        }
//    }
//
//    return (status);
//}


uint16_t envc_read_value(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst)
{
    return envc_read_attribute(conidx, ENVC_READ_VALUE_OP_CODE, es_char_id, es_char_inst, 0);
}

uint16_t envc_read_ccc(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst)
{
    return envc_read_attribute(conidx, ENVC_READ_CCC_OP_CODE, es_char_id, es_char_inst, 0);
}

uint16_t envc_read_trigger(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst, uint8_t trigger_idx)
{
    return envc_read_attribute(conidx, ENVC_READ_TRIGGER_OP_CODE, es_char_id, es_char_inst, trigger_idx);
}

uint16_t envc_read_config(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst)
{
    return envc_read_attribute(conidx, ENVC_READ_CONFIG_OP_CODE, es_char_id, es_char_inst, 0);
}

uint16_t envc_read_user_desc(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst)
{
    return envc_read_attribute(conidx, ENVC_READ_USER_DESCRIPTION_OP_CODE, es_char_id, es_char_inst, 0);
}

uint16_t envc_read_ext_prop(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst)
{
    return envc_read_attribute(conidx, ENVC_READ_EXT_PROP_OP_CODE, es_char_id, es_char_inst, 0);
}

uint16_t envc_read_meas_desc(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst)
{
    return envc_read_attribute(conidx, ENVC_READ_MEAS_DESCRIPTOR_OP_CODE, es_char_id, es_char_inst, 0);
}

uint16_t envc_read_value_range(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst)
{
    return envc_read_attribute(conidx, ENVC_READ_RANGE_OP_CODE, es_char_id, es_char_inst, 0);
}

uint16_t envc_write_ccc(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst, uint16_t ccc)
{
    co_buf_t* p_buf;
    uint16_t status;

    if ((es_char_id == ENVP_ES_DESCRIPTOR_VALUE_CHANGED_CHAR) && ((ccc & 0xFFFD) != 0) )
    {
        // Only Bit 1 - can be set for the DVC
        status = PRF_APP_ERROR;
    }
    else if ((ccc & 0xFFFE) != 0)
    {
        // Only Bit 0 - can be set for the Other ES Characteristics.
        status = PRF_APP_ERROR;
    }
    else if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, sizeof(uint16_t), GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
    {
        // prepare buffer
        co_write16p(co_buf_data(p_buf), co_htobs(ccc));
        // write data
        status = envc_write_attribute(conidx, ENVC_WRITE_CCC_OP_CODE, es_char_id, es_char_inst, 0, p_buf);
        // release buffer
        co_buf_release(p_buf);
    }
    else
    {
        status = GAP_ERR_INSUFF_RESOURCES;
    }

    return (status);
}

uint16_t envc_write_trigger(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst,
                            uint8_t trigger_idx, const envp_trigger_t* p_trigger)
{
    co_buf_t* p_buf;
    uint16_t status = GAP_ERR_NO_ERROR;
    if(p_trigger == NULL)
    {
        status = PRF_ERR_INVALID_PARAM;
    }
    // Check the values passed from Application are valid
    else if (p_trigger->condition > ENVP_TRIG_VALUE_NOT_EQUAL)
    {
        status = PRF_ERR_INVALID_PARAM;
    }
    else if (   (p_trigger->condition > ENVP_TRIG_PREV_VALUE_DIFFERENT) && (p_trigger->condition < ENVP_TRIG_VALUE_EQUAL)
             && (   (es_char_id == ENVP_ES_BAR_PRES_TREND_CHAR) || (es_char_id == ENVP_ES_MAGN_FLUX_2D_CHAR)
                 || (es_char_id == ENVP_ES_MAGN_FLUX_3D_CHAR)))
    {
        status = PRF_ERR_INVALID_PARAM;
    }
    else if (trigger_idx > 2)
    {
        status = PRF_ERR_INVALID_PARAM;
    }
    else if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, 0, L2CAP_LE_MTU_MIN + GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
    {
        // pack data
        co_buf_tail(p_buf)[0] = p_trigger->condition;
        co_buf_tail_reserve(p_buf, 1);

        if (p_trigger->condition < ENVP_TRIG_VALUE_LESS)
        {
            if ((p_trigger->condition == ENVP_TRIG_INTERVAL_FIXED) || (p_trigger->condition == ENVP_TRIG_INTERVAL_NO_LESS_THAN))
            {
                // Add a uint24 to the data field.
                co_write24p(co_buf_tail(p_buf), co_htob24(p_trigger->time_based_trigger));
                co_buf_tail_reserve(p_buf, 3);
            }
        }
        else if ((p_trigger->condition > ENVP_TRIG_PREV_VALUE_DIFFERENT) && (p_trigger->condition < ENVP_TRIG_RESERVED))
        {
            // In this case the size of the operand is dependant on the size of the
            // value field of the measurement characteristic.
            switch (es_char_id)
            {
                // First all the uint16 chars
                case ENVP_ES_APRNT_WIND_DIRECTION_CHAR:
                {
                    co_write16p(co_buf_tail(p_buf), co_htobs(p_trigger->meas_value_trigger.apparent_wind_direction));
                    co_buf_tail_reserve(p_buf, 2);
                } break;
                case ENVP_ES_APRNT_WIND_SPEED_CHAR:
                {
                    co_write16p(co_buf_tail(p_buf), co_htobs(p_trigger->meas_value_trigger.apparent_wind_speed));
                    co_buf_tail_reserve(p_buf, 2);
                } break;
                case ENVP_ES_HUMIDITY_CHAR:
                {
                    co_write16p(co_buf_tail(p_buf), co_htobs(p_trigger->meas_value_trigger.humidity));
                    co_buf_tail_reserve(p_buf, 2);
                } break;
                case ENVP_ES_IRRADIANCE_CHAR:
                {
                    co_write16p(co_buf_tail(p_buf), co_htobs(p_trigger->meas_value_trigger.irradiance));
                    co_buf_tail_reserve(p_buf, 2);
                } break;
                case ENVP_ES_RAINFALL_CHAR:
                {
                    co_write16p(co_buf_tail(p_buf), co_htobs(p_trigger->meas_value_trigger.rainfall));
                    co_buf_tail_reserve(p_buf, 2);
                } break;
                case ENVP_ES_TRUE_WIND_DIR_CHAR:
                {
                    co_write16p(co_buf_tail(p_buf), co_htobs(p_trigger->meas_value_trigger.true_wind_direction));
                    co_buf_tail_reserve(p_buf, 2);
                } break;
                case ENVP_ES_TRUE_WIND_SPEED_CHAR:
                {
                    co_write16p(co_buf_tail(p_buf), co_htobs(p_trigger->meas_value_trigger.true_wind_speed));
                    co_buf_tail_reserve(p_buf, 2);
                } break;
                case ENVP_ES_MAGN_DECLINE_CHAR:
                {
                    co_write16p(co_buf_tail(p_buf), co_htobs(p_trigger->meas_value_trigger.magnetic_declination));
                    co_buf_tail_reserve(p_buf, 2);
                } break;
                // uint24
                case ENVP_ES_POLLEN_CONC_CHAR:
                {
                    co_write24p(co_buf_tail(p_buf), co_htob24(p_trigger->meas_value_trigger.pollen_concentration));
                    co_buf_tail_reserve(p_buf, 3);
                } break;
                // now the uint32 chars
                case ENVP_ES_PRESSURE_CHAR:
                {
                    co_write32p(co_buf_tail(p_buf), co_htobl(p_trigger->meas_value_trigger.pressure));
                    co_buf_tail_reserve(p_buf, 4);
                } break;
                // the int24 chars
                case ENVP_ES_ELEVATION_CHAR:
                {
                    co_write24p(co_buf_tail(p_buf), co_htob24(p_trigger->meas_value_trigger.elevation));
                    co_buf_tail_reserve(p_buf, 3);
                } break;
                // int16
                case ENVP_ES_TEMPERATURE_CHAR:
                {
                    co_write16p(co_buf_tail(p_buf), co_htobs(p_trigger->meas_value_trigger.temperature));
                    co_buf_tail_reserve(p_buf, 2);
                } break;
                // uint8
                case ENVP_ES_GUST_FACTOR_CHAR:
                {
                    co_buf_tail(p_buf)[0] = p_trigger->meas_value_trigger.gust_factor;
                    co_buf_tail_reserve(p_buf, 1);
                } break;
                case ENVP_ES_UV_INDEX_CHAR:
                {
                    co_buf_tail(p_buf)[0] = p_trigger->meas_value_trigger.uv_index;
                    co_buf_tail_reserve(p_buf, 1);
                } break;
                case ENVP_ES_BAR_PRES_TREND_CHAR:
                {
                    co_buf_tail(p_buf)[0] = p_trigger->meas_value_trigger.barometric_pressure_trend;
                    co_buf_tail_reserve(p_buf, 1);
                } break;
                // int8
                case ENVP_ES_DEW_POINT_CHAR:
                {
                    co_buf_tail(p_buf)[0] = (uint8_t) p_trigger->meas_value_trigger.dew_point;
                    co_buf_tail_reserve(p_buf, 1);
                } break;
                case ENVP_ES_HEAT_INDEX_CHAR:
                {
                    co_buf_tail(p_buf)[0] = (uint8_t) p_trigger->meas_value_trigger.heat_index;
                    co_buf_tail_reserve(p_buf, 1);
                } break;
                case ENVP_ES_WIND_CHILL_CHAR:
                {
                    co_buf_tail(p_buf)[0] = (uint8_t) p_trigger->meas_value_trigger.wind_chill;
                    co_buf_tail_reserve(p_buf, 1);
                } break;
                default: { status = PRF_ERR_INVALID_PARAM; } break;
            }
        }

        if(status == GAP_ERR_NO_ERROR)
        {
            // write data
            status = envc_write_attribute(conidx, ENVC_WRITE_TRIGGER_OP_CODE, es_char_id, es_char_inst, trigger_idx, p_buf);
        }
        // release buffer
        co_buf_release(p_buf);
    }
    else
    {
        status = GAP_ERR_INSUFF_RESOURCES;
    }

    return (status);
}

uint16_t envc_write_config(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst, uint8_t trigger_logic)
{
    co_buf_t* p_buf;
    uint16_t status;

    if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, sizeof(uint8_t), GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
    {
        // prepare buffer
        co_buf_data(p_buf)[0] = trigger_logic;
        // write data
        status = envc_write_attribute(conidx, ENVC_WRITE_CONFIG_OP_CODE, es_char_id, es_char_inst, 0, p_buf);
        // release buffer
        co_buf_release(p_buf);
    }
    else
    {
        status = GAP_ERR_INSUFF_RESOURCES;
    }

    return (status);
}

uint16_t envc_write_user_desc(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst, co_buf_t* p_user_desc)
{
    uint16_t status;
    if(p_user_desc == NULL)
    {
        status = PRF_ERR_INVALID_PARAM;
    }
    else
    {
        // execute attribute write
        status =  envc_write_attribute(conidx, ENVC_WRITE_USER_DESCRIPTION_OP_CODE, es_char_id, es_char_inst, 0,
                                       p_user_desc);
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
 * @brief Send a ENVC_CMP_EVT message to the task which enabled the profile
 * @param[in] conidx Connection index
 * @param[in] operation Operation
 * @param[in] status Satus
 ****************************************************************************************
 */
__STATIC void envc_send_cmp_evt(uint8_t conidx, uint8_t operation, uint16_t status)
{
    struct envc_cmp_evt *p_evt;

    // Send the message
    p_evt = KE_MSG_ALLOC(ENVC_CMP_EVT, PRF_DST_TASK(ENVC), PRF_SRC_TASK(ENVC), envc_cmp_evt);
    if(p_evt)
    {
        p_evt->conidx     = conidx;
        p_evt->operation  = operation;
        p_evt->status     = status;

        ke_msg_send(p_evt);
    }
}

/**
 ****************************************************************************************
 * @brief  Message handler example
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int envc_enable_req_handler(ke_msg_id_t const msgid, struct envc_enable_req const *p_param,
                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = envc_enable(p_param->conidx, p_param->con_type, &(p_param->es));

    // send an error if request fails
    if (status != GAP_ERR_NO_ERROR)
    {
        struct envc_enable_rsp *p_rsp = KE_MSG_ALLOC(ENVC_ENABLE_RSP, src_id, dest_id, envc_enable_rsp);
        if(p_rsp != NULL)
        {
            p_rsp->conidx = p_param->conidx;
            p_rsp->status = status;
            ke_msg_send(p_rsp);
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref ENVC_RD_VALUE_CMD  message from the application.
 * .To read the Characteristic value of a given characteristic in the peer server.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the p_parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int envc_rd_value_cmd_handler(ke_msg_id_t const msgid, struct envc_rd_value_cmd *p_param,
                                       ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = envc_read_value(p_param->conidx, p_param->es_char_id, p_param->es_char_inst);

    // send error response if request fails
    if (status != GAP_ERR_NO_ERROR)
    {
        struct envc_cmp_evt *p_evt = KE_MSG_ALLOC(ENVC_CMP_EVT, src_id, dest_id, envc_cmp_evt);
        if(p_evt)
        {
            p_evt->conidx     = p_param->conidx;
            p_evt->operation  = ENVC_READ_VALUE_OP_CODE;
            p_evt->status     = status;

            ke_msg_send(p_evt);
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref ENVC_RD_DESC_CMD  message from the application.
 * .To read the CCC value of a given characteristic in the peer server.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the p_parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int envc_rd_desc_cmd_handler(ke_msg_id_t const msgid,
        struct envc_rd_desc_cmd *p_param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    uint16_t status;
    uint8_t conidx       = p_param->conidx;
    uint8_t es_char_id   = p_param->es_char_id;
    uint8_t es_char_inst = p_param->es_char_inst;

    switch(p_param->desc_type)
    {
        case ENVP_DESCR_CCC:                { status = envc_read_ccc(conidx, es_char_id, es_char_inst);              } break;
        case ENVP_DESCR_MEASUREMENT:        { status = envc_read_meas_desc(conidx, es_char_id, es_char_inst);        } break;
        case ENVP_DESCR_TRIGGER_1:
        case ENVP_DESCR_TRIGGER_2:
        case ENVP_DESCR_TRIGGER_3:          { status = envc_read_trigger(conidx, es_char_id, es_char_inst,
                                                                         p_param->desc_type - ENVP_DESCR_TRIGGER_1); } break;
        case ENVP_DESCR_CONFIG:             { status = envc_read_config(conidx, es_char_id, es_char_inst);           } break;
        case ENVP_DESCR_USER_DESCRIPTION:   { status = envc_read_user_desc(conidx, es_char_id, es_char_inst);        } break;
        case ENVP_DESCR_VALUE_RANGE:        { status = envc_read_value_range(conidx, es_char_id, es_char_inst);      } break;
        case ENVP_DESCR_EXT_PROP:           { status = envc_read_ext_prop(conidx, es_char_id, es_char_inst);         } break;
        default:                            { status = PRF_ERR_INVALID_PARAM;                                        } break;
    }

    // send error response if request fails
    if (status != GAP_ERR_NO_ERROR)
    {
        struct envc_cmp_evt *p_evt = KE_MSG_ALLOC(ENVC_CMP_EVT, src_id, dest_id, envc_cmp_evt);
        if(p_evt)
        {
            p_evt->conidx     = p_param->conidx;
            p_evt->operation  = ENVC_READ_VALUE_OP_CODE;
            p_evt->status     = status;

            ke_msg_send(p_evt);
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref ENVC_WR_CHAR_NTF_CFG_CMD message.
 * Allows the application to write new CCC values to a Characteristic in the peer server
 * @param[in] msgid Id of the message received.
 * @param[in] p_param Pointer to the p_parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int envc_wr_ntf_cfg_cmd_handler(ke_msg_id_t const msgid,
                                   struct envc_wr_ntf_cmd *p_param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    uint16_t status = envc_write_ccc(p_param->conidx, p_param->es_char_id, p_param->es_char_inst, p_param->flags);

    // send error response if request fails
    if (status != GAP_ERR_NO_ERROR)
    {
        struct envc_cmp_evt *p_evt = KE_MSG_ALLOC(ENVC_CMP_EVT, src_id, dest_id, envc_cmp_evt);
        if(p_evt)
        {
            p_evt->conidx     = p_param->conidx;
            p_evt->operation  = ENVC_WRITE_CCC_OP_CODE;
            p_evt->status     = status;

            ke_msg_send(p_evt);
        }
    }

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref ENVC_WR_CHAR_CONFIG_CMD message.
 * Allows the application to write a new trigger config value to a Characteristic in the peer server
 * @param[in] msgid Id of the message received.
 * @param[in] p_param Pointer to the p_parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

__STATIC int envc_wr_config_cmd_handler(ke_msg_id_t const msgid, struct envc_wr_config_cmd *p_param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = envc_write_config(p_param->conidx, p_param->es_char_id, p_param->es_char_inst, p_param->trigger_logic);

    // send error response if request fails
    if (status != GAP_ERR_NO_ERROR)
    {
        struct envc_cmp_evt *p_evt = KE_MSG_ALLOC(ENVC_CMP_EVT, src_id, dest_id, envc_cmp_evt);
        if(p_evt)
        {
            p_evt->conidx     = p_param->conidx;
            p_evt->operation  = ENVC_WRITE_CONFIG_OP_CODE;
            p_evt->status     = status;

            ke_msg_send(p_evt);
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref ENVC_WR_CHAR_USER_DESCRIPTION_CMD message.
 * Allows the application to write a new name for to a Characteristic in the peer server
 * @param[in] msgid Id of the message received.
 * @param[in] p_param Pointer to the p_parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

__STATIC int envc_wr_user_description_cmd_handler(ke_msg_id_t const msgid,
                                   struct envc_wr_user_description_cmd *p_param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    uint16_t status;
    uint16_t data_len = p_param->user_desc.length;
    co_buf_t* p_buf;

    if(co_buf_alloc(&p_buf, GATT_BUFFER_HEADER_LEN, data_len, GATT_BUFFER_TAIL_LEN) == CO_BUF_ERR_NO_ERROR)
    {
        // copy user description
        co_buf_copy_data_from_mem(p_buf, p_param->user_desc.str, data_len);
        // write user description
        status = envc_write_user_desc(p_param->conidx, p_param->es_char_id, p_param->es_char_inst, p_buf);
        // release buffer
        co_buf_release(p_buf);
    }
    else
    {
        status = GAP_ERR_INSUFF_RESOURCES;
    }

    // send error response if request fails
    if (status != GAP_ERR_NO_ERROR)
    {
        struct envc_cmp_evt *p_evt = KE_MSG_ALLOC(ENVC_CMP_EVT, src_id, dest_id, envc_cmp_evt);
        if(p_evt)
        {
            p_evt->conidx     = p_param->conidx;
            p_evt->operation  = ENVC_WRITE_USER_DESCRIPTION_OP_CODE;
            p_evt->status     = status;

            ke_msg_send(p_evt);
        }
    }
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref ENVC_WR_CHAR_TRIGGER_CMD message.
 * Allows the application to write a new trigger value for a given trigger id to a Characteristic in the peer server
 * @param[in] msgid Id of the message received.
 * @param[in] p_param Pointer to the p_parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

__STATIC int envc_wr_trigger_cmd_handler(ke_msg_id_t const msgid, struct envc_wr_trigger_cmd *p_param,
                                         ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t status = envc_write_trigger(p_param->conidx, p_param->es_char_id, p_param->es_char_inst, p_param->trigger_idx,
                                         &(p_param->trigger));

    // send error response if request fails
    if (status != GAP_ERR_NO_ERROR)
    {
        struct envc_cmp_evt *p_evt = KE_MSG_ALLOC(ENVC_CMP_EVT, src_id, dest_id, envc_cmp_evt);
        if(p_evt)
        {
            p_evt->conidx     = p_param->conidx;
            p_evt->operation  = ENVC_WRITE_TRIGGER_OP_CODE;
            p_evt->status     = status;

            ke_msg_send(p_evt);
        }
    }

    return (KE_MSG_CONSUMED);
}

/// Default State handlers definition
KE_MSG_HANDLER_TAB(envc)
{
    // Note: all messages must be sorted in ID ascending order

    { ENVC_ENABLE_REQ,               (ke_msg_func_t) envc_enable_req_handler                },
    { ENVC_RD_VALUE_CMD,             (ke_msg_func_t) envc_rd_value_cmd_handler              },
    { ENVC_RD_DESC_CMD,              (ke_msg_func_t) envc_rd_desc_cmd_handler               },
    { ENVC_WR_NTF_CFG_CMD,           (ke_msg_func_t) envc_wr_ntf_cfg_cmd_handler            },
    { ENVC_WR_TRIGGER_CMD,           (ke_msg_func_t) envc_wr_trigger_cmd_handler            },
    { ENVC_WR_CONFIG_CMD,            (ke_msg_func_t) envc_wr_config_cmd_handler             },
    { ENVC_WR_USER_DESCRIPTION_CMD,  (ke_msg_func_t) envc_wr_user_description_cmd_handler   },

};


/**
 ****************************************************************************************
 * @brief Completion of enable procedure
 *
 * @param[in] conidx        Connection index
 * @param[in] status        Status of the procedure execution (see enum #hl_err)
 * @param[in] p_es          Pointer to peer database description bond data
 ****************************************************************************************
 */
__STATIC void envc_cb_enable_cmp(uint8_t conidx, uint16_t status, const envc_es_content_t* p_es)
{
    ke_task_id_t src_id  = PRF_SRC_TASK(ENVC);
    ke_task_id_t dest_id = PRF_DST_TASK(ENVC);
    struct envc_enable_rsp* p_rsp;

    // Send APP the details of the discovered attributes on ENVC
    p_rsp = KE_MSG_ALLOC(ENVC_ENABLE_RSP, dest_id, src_id, envc_enable_rsp);
    if(p_rsp != NULL)
    {
        if(status == GAP_ERR_NO_ERROR)
        {
            uint8_t i;

            for(i = 0 ; i < p_es->nb_chars_discovered ; i++)
            {
                struct envc_es_info_ind* p_ind;
                p_ind = KE_MSG_ALLOC(ENVC_ES_INFO_IND, dest_id, src_id, envc_es_info_ind);
                if(p_ind != NULL)
                {
                    p_ind->conidx = conidx;
                    memcpy(&(p_ind->info), &(p_es->chars[i]), sizeof(envc_es_char_info_t));
                    ke_msg_send(p_ind);
                }
            }
        }

        p_rsp->conidx = conidx;
        p_rsp->status = status;
        p_rsp->nb_chars_discovered = p_es->nb_chars_discovered;
        memcpy(&(p_rsp->svc),&(p_es->svc), sizeof(prf_svc_t));
        ke_msg_send(p_rsp);
    }
}

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
__STATIC void envc_cb_read_value_cmp(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                                     const union envp_val_char* p_value)
{
    ke_task_id_t src_id  = PRF_SRC_TASK(ENVC);
    ke_task_id_t dest_id = PRF_DST_TASK(ENVC);

    if(status == GAP_ERR_NO_ERROR)
    {
        struct envc_rd_value_ind *p_ind = KE_MSG_ALLOC(ENVC_RD_VALUE_IND, dest_id, src_id, envc_rd_value_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx       = conidx;
            p_ind->es_char_id   = es_char_id;
            p_ind->es_char_inst = es_char_inst;
            memcpy(&(p_ind->es_char_value), p_value, sizeof(union envp_val_char));
            ke_msg_send(p_ind);
        }
    }

    envc_send_cmp_evt(conidx, ENVC_READ_VALUE_OP_CODE, status);
}

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
__STATIC void envc_cb_read_ccc_cmp(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst, uint16_t ccc)
{
    ke_task_id_t src_id  = PRF_SRC_TASK(ENVC);
    ke_task_id_t dest_id = PRF_DST_TASK(ENVC);

    if(status == GAP_ERR_NO_ERROR)
    {
        struct envc_rd_ntf_cfg_ind *p_ind = KE_MSG_ALLOC(ENVC_RD_NTF_CFG_IND, dest_id, src_id, envc_rd_ntf_cfg_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx       = conidx;
            p_ind->es_char_id   = es_char_id;
            p_ind->es_char_inst = es_char_inst;
            p_ind->ccc          = ccc;
            ke_msg_send(p_ind);
        }
    }

    envc_send_cmp_evt(conidx, ENVC_READ_CCC_OP_CODE, status);
}


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
__STATIC void envc_cb_read_trigger_cmp(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                                       uint8_t trigger_idx, const envp_trigger_t* p_trigger)
{
    ke_task_id_t src_id  = PRF_SRC_TASK(ENVC);
    ke_task_id_t dest_id = PRF_DST_TASK(ENVC);

    if(status == GAP_ERR_NO_ERROR)
    {
        struct envc_rd_trigger_ind *p_ind = KE_MSG_ALLOC(ENVC_RD_TRIGGER_IND, dest_id, src_id, envc_rd_trigger_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx       = conidx;
            p_ind->es_char_id   = es_char_id;
            p_ind->es_char_inst = es_char_inst;
            p_ind->trigger_idx  = trigger_idx;
            memcpy(&(p_ind->trigger), p_trigger, sizeof(envp_trigger_t));
            ke_msg_send(p_ind);
        }
    }

    envc_send_cmp_evt(conidx, ENVC_READ_TRIGGER_OP_CODE, status);
}


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
__STATIC void envc_cb_read_config_cmp(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                                      uint8_t trigger_logic)
{
    ke_task_id_t src_id  = PRF_SRC_TASK(ENVC);
    ke_task_id_t dest_id = PRF_DST_TASK(ENVC);

    if(status == GAP_ERR_NO_ERROR)
    {
        struct envc_rd_config_ind *p_ind = KE_MSG_ALLOC(ENVC_RD_CONFIG_IND, dest_id, src_id, envc_rd_config_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx         = conidx;
            p_ind->es_char_id     = es_char_id;
            p_ind->es_char_inst   = es_char_inst;
            p_ind->trigger_logic  = trigger_logic;
            ke_msg_send(p_ind);
        }
    }

    envc_send_cmp_evt(conidx, ENVC_READ_CONFIG_OP_CODE, status);
}


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
__STATIC void envc_cb_read_user_desc_cmp(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                                         co_buf_t* p_user_desc)
{
    ke_task_id_t src_id  = PRF_SRC_TASK(ENVC);
    ke_task_id_t dest_id = PRF_DST_TASK(ENVC);

    if(status == GAP_ERR_NO_ERROR)
    {
        uint16_t data_len = co_buf_data_len(p_user_desc);

        struct envc_rd_user_description_ind *p_ind = KE_MSG_ALLOC_DYN(ENVC_RD_USER_DESCRIPTION_IND, dest_id, src_id,
                                                                      envc_rd_user_description_ind, data_len);
        if(p_ind != NULL)
        {
            p_ind->conidx           = conidx;
            p_ind->es_char_id       = es_char_id;
            p_ind->es_char_inst     = es_char_inst;
            p_ind->user_desc.length = data_len;
            co_buf_copy_data_to_mem(p_user_desc, p_ind->user_desc.str, data_len);
            ke_msg_send(p_ind);
        }
    }

    envc_send_cmp_evt(conidx, ENVC_READ_USER_DESCRIPTION_OP_CODE, status);
}


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
__STATIC void envc_cb_read_ext_prop_cmp(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                                        uint16_t ext_prop)
{
    ke_task_id_t src_id  = PRF_SRC_TASK(ENVC);
    ke_task_id_t dest_id = PRF_DST_TASK(ENVC);

    if(status == GAP_ERR_NO_ERROR)
    {
        struct envc_rd_ext_prop_ind *p_ind = KE_MSG_ALLOC(ENVC_RD_EXT_PROP_IND, dest_id, src_id, envc_rd_ext_prop_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx       = conidx;
            p_ind->es_char_id   = es_char_id;
            p_ind->es_char_inst = es_char_inst;
            p_ind->ext_prop     = ext_prop;
            ke_msg_send(p_ind);
        }
    }

    envc_send_cmp_evt(conidx, ENVC_READ_EXT_PROP_OP_CODE, status);
}


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
__STATIC void envc_cb_read_meas_desc_cmp(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                                         const envp_es_meas_desc_t* p_meas)
{
    ke_task_id_t src_id  = PRF_SRC_TASK(ENVC);
    ke_task_id_t dest_id = PRF_DST_TASK(ENVC);

    if(status == GAP_ERR_NO_ERROR)
    {
        struct envc_rd_meas_descriptor_ind *p_ind = KE_MSG_ALLOC(ENVC_RD_MEAS_DESCRIPTOR_IND, dest_id, src_id,
                                                                 envc_rd_meas_descriptor_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx         = conidx;
            p_ind->es_char_id     = es_char_id;
            p_ind->es_char_inst   = es_char_inst;
            memcpy(&(p_ind->meas), p_meas, sizeof(envp_es_meas_desc_t));
            ke_msg_send(p_ind);
        }
    }

    envc_send_cmp_evt(conidx, ENVC_READ_MEAS_DESCRIPTOR_OP_CODE, status);
}


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
__STATIC void envc_cb_read_value_range_cmp(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                                           const union envp_range* p_range)
{
    ke_task_id_t src_id  = PRF_SRC_TASK(ENVC);
    ke_task_id_t dest_id = PRF_DST_TASK(ENVC);

    if(status == GAP_ERR_NO_ERROR)
    {
        struct envc_rd_range_ind *p_ind = KE_MSG_ALLOC(ENVC_RD_RANGE_IND, dest_id, src_id, envc_rd_range_ind);
        if(p_ind != NULL)
        {
            p_ind->conidx         = conidx;
            p_ind->es_char_id     = es_char_id;
            p_ind->es_char_inst   = es_char_inst;
            memcpy(&(p_ind->range), p_range, sizeof(union envp_range));
            ke_msg_send(p_ind);
        }
    }

    envc_send_cmp_evt(conidx, ENVC_READ_RANGE_OP_CODE, status);
}


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
__STATIC void envc_cb_write_ccc_cmp(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst)
{
    envc_send_cmp_evt(conidx, ENVC_WRITE_CCC_OP_CODE, status);
}


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
__STATIC void envc_cb_write_trigger_cmp(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst,
                                        uint8_t trigger_idx)
{
    envc_send_cmp_evt(conidx, ENVC_WRITE_TRIGGER_OP_CODE, status);
}


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
__STATIC void envc_cb_write_config_cmp(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst)
{
    envc_send_cmp_evt(conidx, ENVC_WRITE_CONFIG_OP_CODE, status);
}

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
__STATIC void envc_cb_write_user_desc_cmp(uint8_t conidx, uint16_t status, uint8_t es_char_id, uint8_t es_char_inst)
{
    envc_send_cmp_evt(conidx, ENVC_WRITE_USER_DESCRIPTION_OP_CODE, status);
}


/**
 ****************************************************************************************
 * @brief Function called when one of the descriptors in the server has changed
 *
 * @param[in] conidx         Connection index
 * @param[in] es_char_id     Characteristic Type idx see enum #envp_es_char
 ****************************************************************************************
 */
__STATIC void envc_cb_desc_chg(uint8_t conidx, uint8_t es_char_id, uint16_t flags)
{
    ke_task_id_t src_id  = PRF_SRC_TASK(ENVC);
    ke_task_id_t dest_id = PRF_DST_TASK(ENVC);

    struct envc_dvc_ind *p_ind = KE_MSG_ALLOC(ENVC_DVC_IND, dest_id, src_id, envc_dvc_ind);
    if(p_ind != NULL)
    {
        p_ind->conidx       = conidx;
        p_ind->es_char_id   = es_char_id;
        p_ind->flags        = flags;
        ke_msg_send(p_ind);
    }
}


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
__STATIC void envc_cb_value(uint8_t conidx, uint8_t es_char_id, uint8_t es_char_inst, const union envp_val_char* p_value)
{
    ke_task_id_t src_id  = PRF_SRC_TASK(ENVC);
    ke_task_id_t dest_id = PRF_DST_TASK(ENVC);

    struct envc_value_ind *p_ind = KE_MSG_ALLOC(ENVC_VALUE_IND, dest_id, src_id, envc_value_ind);
    if(p_ind != NULL)
    {
        p_ind->conidx            = conidx;
        p_ind->es_char_id        = es_char_id;
        p_ind->es_char_inst      = es_char_inst;
        memcpy(&(p_ind->value), p_value, sizeof(union envp_val_char));
        ke_msg_send(p_ind);
    }
}

/// Default Message handle
__STATIC const envc_cb_t envc_msg_cb =
{
    .cb_enable_cmp           = envc_cb_enable_cmp,
    .cb_read_value_cmp       = envc_cb_read_value_cmp,
    .cb_read_ccc_cmp         = envc_cb_read_ccc_cmp,
    .cb_read_trigger_cmp     = envc_cb_read_trigger_cmp,
    .cb_read_config_cmp      = envc_cb_read_config_cmp,
    .cb_read_user_desc_cmp   = envc_cb_read_user_desc_cmp,
    .cb_read_ext_prop_cmp    = envc_cb_read_ext_prop_cmp,
    .cb_read_meas_desc_cmp   = envc_cb_read_meas_desc_cmp,
    .cb_read_value_range_cmp = envc_cb_read_value_range_cmp,
    .cb_write_ccc_cmp        = envc_cb_write_ccc_cmp,
    .cb_write_trigger_cmp    = envc_cb_write_trigger_cmp,
    .cb_write_config_cmp     = envc_cb_write_config_cmp,
    .cb_write_user_desc_cmp  = envc_cb_write_user_desc_cmp,
    .cb_desc_chg             = envc_cb_desc_chg,
    .cb_value                = envc_cb_value,
};
#endif // (HOST_MSG_API)


/*
 * PROFILE DEFAULT HANDLERS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Initialization of the Client module.
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
__STATIC uint16_t envc_init(prf_data_t* p_env, uint16_t* p_start_hdl, uint8_t sec_lvl, uint8_t user_prio,
                          const void* p_params, const envc_cb_t* p_cb)
{
    uint8_t conidx;
    // DB Creation Status
    uint16_t status = GAP_ERR_NO_ERROR;
    uint8_t user_lid = GATT_INVALID_USER_LID;

    do
    {
        envc_env_t* p_envc_env;

        #if (HOST_MSG_API)
        if(p_cb == NULL)
        {
            p_cb = &(envc_msg_cb);
        }
        #endif // (HOST_MSG_API)

        if(   (p_params == NULL) || (p_cb == NULL)  || (p_cb->cb_enable_cmp == NULL)
           || (p_cb->cb_read_value_cmp == NULL)     || (p_cb->cb_read_ccc_cmp == NULL)
           || (p_cb->cb_read_trigger_cmp == NULL)   || (p_cb->cb_read_config_cmp == NULL)
           || (p_cb->cb_read_user_desc_cmp == NULL) || (p_cb->cb_read_ext_prop_cmp == NULL)
           || (p_cb->cb_read_meas_desc_cmp == NULL) || (p_cb->cb_read_value_range_cmp == NULL)
           || (p_cb->cb_write_ccc_cmp == NULL)      || (p_cb->cb_write_trigger_cmp == NULL)
           || (p_cb->cb_write_config_cmp == NULL)   || (p_cb->cb_write_user_desc_cmp == NULL)
           || (p_cb->cb_desc_chg == NULL)           || (p_cb->cb_value == NULL) )
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // register ENVC user
        status = gatt_user_cli_register(ENVS_USR_DESC_MAX_LEN + GATT_WRITE_HEADER_LEN, user_prio, &envc_cb, &user_lid);
        if(status != GAP_ERR_NO_ERROR) break;

        //-------------------- allocate memory required for the profile  ---------------------
        p_envc_env = (envc_env_t*) ke_malloc_user(sizeof(envc_env_t), KE_MEM_PROFILE);

        if(p_envc_env != NULL)
        {
            // allocate ENVC required environment variable
            p_env->p_env = (prf_hdr_t *) p_envc_env;

            // initialize environment variable
            p_envc_env->prf_env.p_cb    = p_cb;
            #if (HOST_MSG_API)
            p_env->desc.msg_handler_tab = envc_msg_handler_tab;
            p_env->desc.msg_cnt         = ARRAY_LEN(envc_msg_handler_tab);
            #endif // (HOST_MSG_API)

            p_envc_env->user_lid = user_lid;
            for (conidx = 0; conidx < HOST_CONNECTION_MAX; conidx++)
            {
                p_envc_env->p_env[conidx] = NULL;
            }
        }
    } while(0);


    if((status != GAP_ERR_NO_ERROR) && (user_lid != GATT_INVALID_USER_LID))
    {
        gatt_user_unregister(user_lid);
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
__STATIC uint16_t envc_destroy(prf_data_t *p_env, uint8_t reason)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    envc_env_t* p_envc_env = (envc_env_t*) p_env->p_env;

    if(reason != PRF_DESTROY_RESET)
    {
        status = gatt_user_unregister(p_envc_env->user_lid);
    }

    if(status == GAP_ERR_NO_ERROR)
    {
        uint8_t conidx;

        // cleanup environment variable for each task instances
        for (conidx = 0; conidx < HOST_CONNECTION_MAX; conidx++)
        {
            if (p_envc_env->p_env[conidx] != NULL)
            {
                ke_free(p_envc_env->p_env[conidx]);
            }
        }

        // free profile environment variables
        p_env->p_env = NULL;
        ke_free(p_envc_env);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Handles Connection creation
 *
 * @param[in,out]    env          Collector or Service allocated environment data.
 * @param[in]        conidx       Connection index
 * @param[in]        is_le_con    True if it's a BLE connection, False if it's a BT-Classic connection
 ****************************************************************************************
 */
__STATIC void envc_con_create(prf_data_t *p_env, uint8_t conidx, bool is_le_con)
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
__STATIC void envc_con_cleanup(prf_data_t *p_env, uint8_t conidx, uint16_t reason)
{
    envc_env_t* p_envc_env = (envc_env_t*) p_env->p_env;

    // clean-up environment variable allocated for task instance
    if (p_envc_env->p_env[conidx] != NULL)
    {
        ke_free(p_envc_env->p_env[conidx]);
        p_envc_env->p_env[conidx] = NULL;
    }
}

/// ENVC Task interface required by profile manager
const prf_task_cbs_t envc_itf =
{
    .cb_init          = (prf_init_cb) envc_init,
    .cb_destroy       = envc_destroy,
    .cb_con_create    = envc_con_create,
    .cb_con_cleanup   = envc_con_cleanup,
};

/**
 ****************************************************************************************
 * @brief Retrieve client profile interface
 *
 * @return Client profile interface
 ****************************************************************************************
 */
const prf_task_cbs_t* envc_prf_itf_get(void)
{
    return &envc_itf;
}

#endif //(BLE_ENV_CLIENT)

/// @} ENVP

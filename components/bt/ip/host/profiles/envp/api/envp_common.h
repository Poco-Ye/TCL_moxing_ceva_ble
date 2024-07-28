/**
 ****************************************************************************************
 *
 * @file envp_common.h
 *
 * @brief Header File - Environmental Sensing Service Profile common types.
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 *
 ****************************************************************************************
 */


#ifndef _ENVP_COMMON_H_
#define _ENVP_COMMON_H_

#include "rwip_config.h"

#if (BLE_ENV_CLIENT || BLE_ENV_SERVER)

#include "prf_types.h"
#include <stdint.h> 
#include "co_math.h"
/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximum possible Characteristic in the database
#define NUMBER_CHARACTERISTICS_MAX (100)

/// Sizes of the description attribute - Descriptor Change
#define ENVP_DESC_CHANGE_MAX_LEN    (16+2)

/// Size of measurement block sent on the request
/// uint32_t is replaced by uint24_t
#define ENVS_ES_MEAS_SIZE (11)

/// Environmental Sensing Configuration Descriptor maximum length
#define ENVS_CFG_SIZE (1)

/// User Description Descriptor maximum length
#define ENVS_USR_DESC_MAX_LEN (256)
/// Client Characteristic Configuration Descriptor size
#define ENVS_CCC_SIZE (2)
/// Environmental Sensing Trigger Setting Descriptor size
#define ENVS_TRIG_SIZE (7)
/// Extended Properties Descriptor size
#define ENVS_EXT_SIZE (2)

/// Apparent Wind Direction Characteristic size
#define ENVS_ES_APRNT_WIND_DIR_SIZE   (2)
/// Apparent Wind Speed Characteristic size
#define ENVS_ES_APRNT_WIND_SPEED_SIZE (2)
/// Dew Point Characteristic size
#define ENVS_ES_DEW_POINT_SIZE        (1)
/// Elevation Characteristic size
#define ENVS_ES_ELEVATION_SIZE        (3)
/// Gust Factor Characteristic size
#define ENVS_ES_GUST_FACTOR_SIZE      (1)
/// Heat Index Characteristic size
#define ENVS_ES_HEAT_INDEX_SIZE       (1)
/// Humidity Characteristic size
#define ENVS_ES_HUMIDITY_SIZE         (2)
/// Irradiance Characteristic size
#define ENVS_ES_IRRADIANCE_SIZE       (2)
/// Pollen Concentration Characteristic size
#define ENVS_ES_POLLEN_CONC_SIZE      (3)
/// Rainfall Characteristic size
#define ENVS_ES_RAINFALL_SIZE         (2)
/// Pressure Characteristic size
#define ENVS_ES_PRESSURE_SIZE         (4)
/// Temperature Characteristic size
#define ENVS_ES_TEMPERATURE_SIZE      (2)
/// True Wind Direction Characteristic size
#define ENVS_ES_TRUE_WIND_DIR_SIZE    (2)
/// True Wind Speed Characteristic size
#define ENVS_ES_TRUE_WIND_SPEED_SIZE  (2)
/// UV Index Characteristic size
#define ENVS_ES_UV_INDEX_SIZE         (1)
/// Wind Chill Characteristic size
#define ENVS_ES_WIND_CHILL_SIZE       (1)
/// Barometric Pressure Trend Characteristic size
#define ENVS_ES_BAR_PRES_TREND_SIZE   (1)
/// Magnetic Declination Characteristic size
#define ENVS_ES_MAGN_DECLINE_SIZE     (2)
/// Magnetic Flux Density - 2D Characteristic size
#define ENVS_ES_MAGN_FLUX_2D_SIZE     (4)
/// Magnetic Flux Density - 3D Characteristic size
#define ENVS_ES_MAGN_FLUX_3D_SIZE     (6)

/// Maximum possible size of datavalue for notification
#define ENVS_NTF_MAX_SIZE (ENVS_ES_MAGN_FLUX_3D_SIZE)
/// Maximum number of TRIG configurations in characteristic
#define TRIGS_MAX 3

/// Flag bits for the Change Characteristic Descriptor
#define ENVS_CHANGE_BY_SERVER  0
#define ENVS_CHANGE_BY_CLIENT (1<<0)
#define ENVS_CHANGE_TRIG      (1<<1)
#define ENVS_CHANGE_CFG       (1<<2)
#define ENVS_CHANGE_MEAS      (1<<3)
#define ENVS_CHANGE_USR_DESC  (1<<3)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// ES Descriptor Value Changed Characteristic bitfield
enum envp_es_desc_value_changed_char_bf
{
    /// Source of Change
    /// 0: Server 1: Client
    ENVS_CHANGE_SOURCE_POS = 0,
    ENVS_CHANGE_SOURCE_BIT = CO_BIT(ENVS_CHANGE_SOURCE_POS),

    /// Change to one or more ES Trigger Setting Descriptors
    /// 0: False 1: True
    ENVS_CHANGE_TRIG_POS = 1,
    ENVS_CHANGE_TRIG_BIT = CO_BIT(ENVS_CHANGE_TRIG_POS),

    /// Change to ES Configuration Descriptor
    /// 0: False 1: True
    ENVS_CHANGE_CFG_POS = 2,
    ENVS_CHANGE_CFG_BIT = CO_BIT(ENVS_CHANGE_CFG_POS),

    /// Change to ES Measurement Descriptor
    /// 0: False 1: True
    ENVS_CHANGE_MEAS_POS = 3,
    ENVS_CHANGE_MEAS_BIT = CO_BIT(ENVS_CHANGE_MEAS_POS),

    /// Change to Characteristic User Description Descriptor
    /// 0: False 1: True
    ENVS_CHANGE_USR_DESC_POS = 4,
    ENVS_CHANGE_USR_DESC_BIT = CO_BIT(ENVS_CHANGE_USR_DESC_POS),

    // Bit 5 - 15 RFU
};

/// ES Measurement Descriptor - Sampling Functions Table 3.5
enum envp_es_meas_desc_sampling_func
{
    ENVP_SAMP_FUNC_UNSPECIFIED = 0,
    ENVP_SAMP_FUNC_INSTANTANEOUS,
    ENVP_SAMP_FUNC_ARITHMETIC_MEAN,
    ENVP_SAMP_FUNC_RMS,
    ENVP_SAMP_FUNC_MAXIMUM,
    ENVP_SAMP_FUNC_MINIMUM,
    ENVP_SAMP_FUNC_ACCUMULATED,
    ENVP_SAMP_FUNC_COUNT,
    ENVP_SAMP_FUNC_RESERVED,
};

/// ES Measurement Descriptor - Applications Table 3.8
enum envp_es_meas_desc_application
{
    ENVP_MEAS_APP_UNSPECIFIED                               = 0x00,
    ENVP_MEAS_APP_AIR                                       = 0x01,
    ENVP_MEAS_APP_WATER                                     = 0x02,
    ENVP_MEAS_APP_BAROMETRIC                                = 0x03,
    ENVP_MEAS_APP_SOIL                                      = 0x04,
    ENVP_MEAS_APP_INFRARED                                  = 0x05,
    ENVP_MEAS_APP_MAP_DATABASE                              = 0x06,
    ENVP_MEAS_APP_BAROMETRIC_ELEVATION_SOURCE               = 0x07,
    ENVP_MEAS_APP_GPS_ONLY_ELEVATION_SOURCE                 = 0x08,
    ENVP_MEAS_APP_GPS_AND_MAP_DATABASE_ELEVATION_SOURCE     = 0x09,
    ENVP_MEAS_APP_VERTICAL_DATUM_ELEVATION_SOURCE           = 0x0A,
    ENVP_MEAS_APP_ONSHORE                                   = 0x0B,
    ENVP_MEAS_APP_ONBOARD_VESSEL_OR_VEHICLE                 = 0x0C,
    ENVP_MEAS_APP_FRONT                                     = 0x0D,
    ENVP_MEAS_APP_BACK_REAR                                 = 0x0E,
    ENVP_MEAS_APP_UPPER                                     = 0x0F,
    ENVP_MEAS_APP_LOWER                                     = 0x10,
    ENVP_MEAS_APP_PRIMARY                                   = 0x11,
    ENVP_MEAS_APP_SECONDARY                                 = 0x12,
    ENVP_MEAS_APP_OUTDOOR                                   = 0x13,
    ENVP_MEAS_APP_INDOOR                                    = 0x14,
    ENVP_MEAS_APP_TOP                                       = 0x15,
    ENVP_MEAS_APP_BOTTOM                                    = 0x16,
    ENVP_MEAS_APP_MAIN                                      = 0x17,
    ENVP_MEAS_APP_BACKUP                                    = 0x18,
    ENVP_MEAS_APP_AUXILIARY                                 = 0x19,
    ENVP_MEAS_APP_SUPPLEMENTARY                             = 0x1A,
    ENVP_MEAS_APP_INSIDE                                    = 0x1B,
    ENVP_MEAS_APP_OUTSIDE                                   = 0x1C,
    ENVP_MEAS_APP_LEFT                                      = 0x1D,
    ENVP_MEAS_APP_RIGHT                                     = 0x1E,
    ENVP_MEAS_APP_INTERNAL                                  = 0x1F,
    ENVP_MEAS_APP_EXTERNAL                                  = 0x20,
    ENVP_MEAS_APP_SOLAR                                     = 0x21,
    // Value 0x22 - 0xFF RFU
};

/// ES Measurement Descriptor - Applications Table 3.11
enum envp_es_trig_condition
{
    ENVP_TRIG_INACTIVE               = 0x00,
    ENVP_TRIG_INTERVAL_FIXED         = 0x01,
    ENVP_TRIG_INTERVAL_NO_LESS_THAN  = 0x02,
    ENVP_TRIG_PREV_VALUE_DIFFERENT   = 0x03,
    ENVP_TRIG_VALUE_LESS             = 0x04,
    ENVP_TRIG_VALUE_LESS_OR_EQUAL    = 0x05,
    ENVP_TRIG_VALUE_GREATER          = 0x06,
    ENVP_TRIG_VALUE_GREATER_OR_EQUAL = 0x07,
    ENVP_TRIG_VALUE_EQUAL            = 0x08,
    ENVP_TRIG_VALUE_NOT_EQUAL        = 0x09,
    ENVP_TRIG_RESERVED               = 0x0A,
    // Value 0x0A - 0xFF RFU
};

/// ES Characteristic ID
enum envp_es_char
{
    /// Descriptor Value Changed Characteristic
    ENVP_ES_DESCRIPTOR_VALUE_CHANGED_CHAR = 0,
    /// Apparent Wind Direction Characteristic
    ENVP_ES_APRNT_WIND_DIRECTION_CHAR     = 1,
    /// Apparent Wind Speed Characteristic
    ENVP_ES_APRNT_WIND_SPEED_CHAR         = 2,
    /// Dew Point Characteristic
    ENVP_ES_DEW_POINT_CHAR                = 3,
    /// Elevation Characteristic
    ENVP_ES_ELEVATION_CHAR                = 4,
    /// Gust Factor Characteristic
    ENVP_ES_GUST_FACTOR_CHAR              = 5,
    /// Heat Index Characteristic
    ENVP_ES_HEAT_INDEX_CHAR               = 6,
    /// Humidity Characteristic
    ENVP_ES_HUMIDITY_CHAR                 = 7,
    /// Irradiance Characteristic
    ENVP_ES_IRRADIANCE_CHAR               = 8,
    /// Pollen Concentration Characteristic
    ENVP_ES_POLLEN_CONC_CHAR              = 9,
    /// Rainfall Characteristic
    ENVP_ES_RAINFALL_CHAR                 = 10,
    /// Pressure Characteristic
    ENVP_ES_PRESSURE_CHAR                 = 11,
    /// Temperature Characteristic
    ENVP_ES_TEMPERATURE_CHAR              = 12,
    /// True Wind Direction Characteristic
    ENVP_ES_TRUE_WIND_DIR_CHAR            = 13,
    /// True Wind Speed Characteristic
    ENVP_ES_TRUE_WIND_SPEED_CHAR          = 14,
    /// UV Index Characteristic
    ENVP_ES_UV_INDEX_CHAR                 = 15,
    /// Wind Chill Characteristic
    ENVP_ES_WIND_CHILL_CHAR               = 16,
    /// Barometric Pressure Trend Characteristic
    ENVP_ES_BAR_PRES_TREND_CHAR           = 17,
    /// Magnetic Declination Characteristic
    ENVP_ES_MAGN_DECLINE_CHAR             = 18,
    /// Magnetic Flux Density - 2D Characteristic
    ENVP_ES_MAGN_FLUX_2D_CHAR             = 19,
    /// Magnetic Flux Density - 2D Characteristic
    ENVP_ES_MAGN_FLUX_3D_CHAR             = 20,
    ENVP_ES_CHAR_MAX
};

/// Trigger Logic Value
enum envp_es_trig_trigger_logic
{
    ENVP_TRIG_BOOLEAN_AND            = 0x00,
    ENVP_TRIG_BOOLEAN_OR             = 0x01,
    // value 0x02 - 0xFF RFU
};

/*
 * STRUCTURES
 ****************************************************************************************
 */

/**
 * ES Measurement
 * not used in the service
 * do not care what is the MEAS value
 * only change would trigger update INDication of VAL_CHANGE
 */
typedef struct envp_es_meas_desc
{
    /// Flags
    uint16_t    flags;
    /// Sampling Function - es_meas_desc_sampling_func
    uint8_t     samp_func;
    /// Measurement Period [ UINT24, Exponent Base 10, Exponent=0, Unit=seconds, Resolution 1 second]
    uint32_t    meas_period;
    /// Update Interval [ UINT24, Exponent Base 10, Exponent=0, Unit=seconds, Resolution 1 second]
    uint32_t    update_period;
    /// Application
    uint8_t     app;
    /// Measurement Uncertainty [Exponent Base 2. Exponent=-1, Unit=%, Resolution 0.5%]
    uint8_t     meas_uncertainty;
} envp_es_meas_desc_t;

/// Magnetic Flux Density - 2D
typedef struct magnetic_flux_dens_2d
{
    /// X axis
    int16_t x_axis;
    /// Y axis
    int16_t y_axis;
} magnetic_flux_dens_2d_t;

/// Magnetic Flux Density - 2D
typedef struct magnetic_flux_dens_3d
{
    /// X axis
    int16_t x_axis;
    /// Y axis
    int16_t y_axis;
    /// Z axis
    int16_t z_axis;
} magnetic_flux_dens_3d_t;

/// ESS characteristics value
union envp_val_char
{
    /// Apparent Wind Direction
    uint16_t                apparent_wind_direction;
    /// Apparent Wind Speed
    uint16_t                apparent_wind_speed;
    /// Dew Point
    int8_t                  dew_point;
    /// Elevation, actually a sint24
    int32_t                 elevation;
    /// Gust Factor
    uint8_t                 gust_factor;
    /// Heat Index
    int8_t                  heat_index;
    /// Humidity
    uint16_t                humidity;
    /// Irradiance
    uint16_t                irradiance;
    /// Pollen Concentration, actually a uint24
    uint32_t                pollen_concentration;
    /// Rainfall
    uint16_t                rainfall;
    /// Pressure
    uint32_t                pressure;
    /// Temperature
    int16_t                 temperature;
    /// True Wind Direction
    uint16_t                true_wind_direction;
    /// True Wind Speed
    uint16_t                true_wind_speed;
    /// UV Index
    uint8_t                 uv_index;
    /// Wind Chill
    int8_t                  wind_chill;
    /// Barometric Pressure Trend
    uint8_t                 barometric_pressure_trend;
    /// Magnetic Declination
    uint16_t                magnetic_declination;
    /// Magnetic Flux Density - 2D
    magnetic_flux_dens_2d_t mag_flux_dens_2d;
    /// Magnetic Flux Density - 3D
    magnetic_flux_dens_3d_t mag_flux_dens_3d;
};

/// ESS Trigger
typedef struct envp_trigger
{
    /// Condition
    uint8_t             condition;
    /// 24bit Time based trigger value - used for conditions (1-2)
    uint32_t            time_based_trigger;
    /// The trigger measurement value - used for conditions (4-9)
    /// the relevant element of the union is based on es_char_id
    union envp_val_char meas_value_trigger;
} envp_trigger_t;


/// 16 bits unsigned integer range
typedef struct range_u16
{
    /// Lower bound
    uint16_t lower;
    /// Upper bound
    uint16_t upper;
} range_u16_t;

/// 8 bits signed integer range
typedef struct range_s8
{
    /// Lower bound
    int8_t lower;
    /// Upper bound
    int8_t upper;
} range_s8_t;

/// 32 bits or 24 bits signed integer range
typedef struct range_s32
{
    /// Lower bound
    int32_t lower;
    /// Upper bound
    int32_t upper;
} range_s32_t;

/// 8 bits unsigned integer range
typedef struct range_u8
{
    /// Lower bound
    uint8_t lower;
    /// Upper bound
    uint8_t upper;
} range_u8_t;

/// 32 bits unsigned integer range or 24 bits unsigned integer range
typedef struct range_u32
{
    /// Lower bound
    uint32_t lower;
    /// Upper bound
    uint32_t upper;
} range_u32_t;

/// 16 bits signed integer range
typedef struct range_s16
{
    /// Lower bound
    int16_t lower;
    /// Upper bound
    int16_t upper;
} range_s16_t;

/// Value Range Descriptor value
union envp_range
{
    /// Pressure range
    /// Pollen Concentration range
    range_u32_t  u32;

    /// Elevation range
    range_s32_t  s32;

    /// Apparent Wind Direction range
    /// Apparent Wind Speed range
    /// Humidity range
    /// Irradiance range
    /// Rainfall range
    /// True Wind Direction range
    /// True Wind Speed range
    /// Magnetic Declination range
    range_u16_t  u16;

    /// Temperature range
    /// Magnetic Flux Density - 2D range
    /// Magnetic Flux Density - 3D range
    range_s16_t  s16;

    /// Gust Factor range
    /// True Wind Speed range
    /// Barometric Pressure Trend range
    range_u8_t   u8;

    /// Dew Point range
    /// Heat Index range
    /// Wind Chill range
    range_s8_t   s8;
};

#endif /* (BLE_ENV_CLIENT || BLE_ENV_SERVER) */
#endif /* _ENVP_COMMON_H_ */


#ifndef __RF_XRC443_H__

#include <string.h>             // for memcpy
#include "co_int.h"
/**
 ****************************************************************************************
 * @addtogroup RF
 * @{
 ****************************************************************************************
 */

/**
 * DEFINES
 ****************************************************************************************
 */

/// SUPPORTED RADIO PHY
#define RF_BLE_PHY_1MBPS_SUPPORT                    1
#define RF_BLE_PHY_2MBPS_SUPPORT                    1
#define RF_BLE_PHY_CODED_SUPPORT                    1
#define RF_STABLE_MOD_IDX_TX_SUPPORT                0
#define RF_STABLE_MOD_IDX_RX_SUPPORT                0
#define RF_PWR_CLASS_1_SUPPORT                      0

/// Radio power-up duration
#define RF_POWERUP_TIME_US                          60
/// Radio power-down duration
#define RF_POWERDOWN_TIME_US                        26

/// RF SW-Driven SPI transfers area definition
#define RF_SW_SPI_SIZE_MAX                          136
/// RF HW-Driven SPI transfers area definition
#define RF_HW_SPI_SIZE_MAX                          100

/// Frequency table area definition - Word size in bytes i.e. 32-bit words
#define RF_FREQ_TABLE_ELT_SIZE                      sizeof(uint32_t)
/// Frequency table area definition - VCO sub-band word size in bytes i.e. 8-bit words
#define RF_VCO_TABLE_ELT_SIZE                       sizeof(uint8_t)

/// Display information for radio
#define RF_BOARD_STR                               "xrc443"
#define RF_BOARD_ID_READ                            0


typedef struct rf_conf{
        uint32_t add;
        uint32_t value;
}rfconf;

#define __RF_XRC443_H__
#endif

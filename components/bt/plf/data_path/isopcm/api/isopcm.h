/**
 ****************************************************************************************
 *
 * @file isopcm.h
 *
 * @brief Declaration of the functions used for Isochronous Payload over HCI Transport layer
 *
 * Copyright (C) RivieraWaves 2009-2017
 *
 *
 ****************************************************************************************
 */

#ifndef ISOPCM_H_
#define ISOPCM_H_

/**
 ****************************************************************************************
 * @addtogroup ISOPCM
 * @ingroup ISO
 * @brief Isochronous Payload over HCI Transport Layer
 *
 * This module implements the primitives used for Isochronous Payload over PCM
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_ISO_PCM)

#include <stdint.h>
#include <stdbool.h>

#include "data_path.h"

/*
 * MACROS
 ****************************************************************************************
 */
/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * ENUMERATIONS
 ****************************************************************************************
 */


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

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
 * @brief Initialize Isochronous over PCM
 *
 * @param[in] init_type  Type of initialization (@see enum rwip_init_type)
 ****************************************************************************************
 */
void isopcm_init(uint8_t init_type);

/**
 ****************************************************************************************
 * @brief Retrieve the isochronous data path interface
 *
 * @param[in] Data path direction (@see enum iso_rx_tx_select)
 *
 * @return isochronous data path interface
 ****************************************************************************************
 */
const struct data_path_itf* isopcm_itf_get(uint8_t  direction);

#endif //(BLE_ISO_PCM)

/// @} ISOPCM

#endif // ISOPCM_H_

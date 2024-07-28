/**
 ****************************************************************************************
 *
 * @file isoohci_int.h
 *
 * @brief Declaration of the functions used for Isochronous Payload over PCM
 *
 * Copyright (C) RivieraWaves 2009-2017
 *
 ****************************************************************************************
 */

#ifndef ISOPCM_INT_H_
#define ISOPCM_INT_H_

/**
 ****************************************************************************************
 * @addtogroup ISOPCM_INT
 * @ingroup ISOPCM
 * @brief Isochronous Payload over PCM
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

#include "arch.h"
#include "isopcm.h"
#include "pcm.h"    // PCM driver

#include "co_djob.h"


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
 * PROTOCOL STRUCTURES
 ****************************************************************************************
 */


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Isochronous RX Buffer management structure
struct isopcm_rx_info
{
    /// Packet length
    uint16_t sdu_len;
    /// Number of Buffer
    uint8_t  nb_buffer;
    /// Current SDU index
    uint8_t  sdu_idx;
    /// channel index
    uint8_t  channel;

    /// Buffer information
    uint8_t* p_buf[__ARRAY_EMPTY];
};


/// Isochronous TX Buffer management structure
struct isopcm_tx_info
{
    /// delayed job
    co_djob_t djob;
    /// Packet length
    uint16_t sdu_len;
    /// Number of Buffer
    uint8_t  nb_buffer;
    /// Programmed Buffer index
    uint8_t  prog_idx;
    /// Number of channel
    uint8_t  nb_channel;
    /// used to know if some data is loading
    bool     data_loading;

    /// Buffer information
    uint8_t* p_buf[__ARRAY_EMPTY];
};

/// Isochronous Payload over PCM
struct isopcm_env_tag
{
    /// Transmitted PCM data path info
    struct isopcm_tx_info* p_tx;
    /// Received PCM data path info
    struct isopcm_rx_info* p_rx;
};
/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

// Environment variables
extern struct isopcm_env_tag isopcm_env;

// RX PCM interface
extern const struct data_path_itf isopcm_rx_itf;


// TX PCM interface
extern const struct data_path_itf isopcm_tx_itf;
/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */


#endif //(BLE_ISO_PCM)

/// @} ISOPCM

#endif // ISOPCM_INT_H_

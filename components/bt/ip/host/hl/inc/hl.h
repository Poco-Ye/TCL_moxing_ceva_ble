/**
 ****************************************************************************************
 *
 * @file hl.h
 *
 * @brief Entry points of the BT/BLE HL software
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 *
 ****************************************************************************************
 */

#ifndef HL_H_
#define HL_H_

#include <stdint.h>
#include <stdbool.h>                // standard boolean definitions
/**
 ****************************************************************************************
 * @ingroup ROOT
 * @addtogroup HOST
 * @brief Entry points of the BT/BLE Host stack
 *
 * This module contains the primitives that allow an application accessing and running the
 * BT/BLEHost protocol stack
 *
 * @{
 ****************************************************************************************
 */


/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Initialize the BLE Host stack.
 *
 * @param[in] init_type  Type of initialization (see enum #rwip_init_type)
 ****************************************************************************************
 */
void hl_initialize(uint8_t init_type);

/// @} HL

#endif // HL_H_

/**
 ****************************************************************************************
 *
 * @file boot.h
 *
 * @brief This file contains the definitions used for boot code.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */


#ifndef _BOOT_H_
#define _BOOT_H_

/**
 ****************************************************************************************
 * @defgroup BOOT
 * @ingroup DRIVERS
 * @brief Definition of boot code values.
 *
 * @{
 *
 ****************************************************************************************
 */

/*
 * INCLUDE
 ****************************************************************************************
 */
#include <stdint.h>            // standard integer functions

/*
 * DEFINES
 ****************************************************************************************
 */

/// Platform sleep pattern
#define BLANK_AREA_PATTERN         0xA5A5A5A5

/// Stack initialization pattern
#define STACK_INIT_PATTERN         0xF3F3F3F3

/// FW location in RAM : 0x00000000
#define FW_RAM_ADDRESS    (0x00000000)


/*
 * LINKER VARIABLES
 ****************************************************************************************
 */

/// Low/high boundaries of data sections (from linker script)
extern uint32_t _sram[];
extern uint32_t _sdata[];
extern uint32_t _edata[];
extern uint32_t _ssbss[];
extern uint32_t _esbss[];
extern uint32_t _sbss[];
extern uint32_t _ebss[];
extern uint32_t _heap_bottom[];
extern uint32_t _heap_top[];
extern uint32_t _stack_bottom[];
extern uint32_t _stack_top[];
extern uint32_t _debugger_register_save_area[];
extern uint32_t _unloaded_area_start[];
extern uint32_t _unloaded_area_end[];
extern const uint32_t _data_load[];



/// @} BOOT
#endif // _BOOT_H_

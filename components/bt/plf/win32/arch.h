/**
 ****************************************************************************************
 *
 * @file arch.h
 *
 * @brief This file contains the definitions of the macros and functions that are
 * architecture dependent.  The implementation of those is implemented in the
 * appropriate architecture directory.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */


#ifndef _ARCH_H_
#define _ARCH_H_

/**
 ****************************************************************************************
 * @defgroup BLUEGRIP
 * @brief BlueGRiP IP Platform
 *
 * This module contains reference platform components - RICOW.
 *
 *
 * @{
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @defgroup DRIVERS
 * @ingroup BLUEGRIP
 * @brief Reference IP Platform Drivers
 *
 * This module contains the necessary drivers to run the platform with the
 * RW BT SW protocol stack.
 *
 * This has the declaration of the platform architecture API.
 *
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
// required to define GLOBAL_INT_** macros as inline assembly
#include "ll.h"            // low level functions
#include "compiler.h"      // inline functions

/*
 * CPU WORD SIZE
 ****************************************************************************************
 */
/// APS3 is a 32-bit CPU
#define CPU_WORD_SIZE   4

/*
 * CPU Endianness
 ****************************************************************************************
 */
/// APS3 is little endian
#define CPU_LE          1

/*
 * DEBUG configuration
 ****************************************************************************************
 */
#if defined(CFG_DBG)
#define PLF_DEBUG          1
#else //CFG_DBG
#define PLF_DEBUG          0
#endif //CFG_DBG

#ifdef CFG_ROM
#define DBG_SWDIAG(bank , field , value)
#endif //CFG_ROM

/*
 * GAIA
 ****************************************************************************************
 */

/// GAIA board
#if defined(CFG_GAIA)
#define PLF_GAIA          1
#else // CFG_GAIA
#define PLF_GAIA          0
#endif // CFG_GAIA

/*
 * Bubble
 ****************************************************************************************
 */

/// Bubble board
#if defined(CFG_BUBBLE)
#define PLF_BUBBLE          1
#else // CFG_GAIA
#define PLF_BUBBLE          0
#endif // CFG_GAIA

/*
 * PLF CLK Speed (MHz)
 ****************************************************************************************
 */
#ifndef CFG_ROM

#if (PLF_BUBBLE)
#if (defined(CFG_RF_RIPPLE) || defined(CFG_RF_RIPPLE_DF))
#define PLF_CLK_MHZ         26
#elif defined(CFG_RF_AU50XX)
#define PLF_CLK_MHZ         24
#else // CFG_RF_xxx
#define PLF_CLK_MHZ         16
#endif // CFG_RF_xxx
#endif // PLF_BUBBLE

#if (PLF_GAIA)
#if (defined(CFG_RF_BTIPT) || defined(CFG_RF_CALYPSO) || defined (CFG_CAL_HPMDM_BYPASS) || defined(CFG_RF_AU50XX))
#define PLF_CLK_MHZ         24
#else // CFG_RF_xxx
#define PLF_CLK_MHZ         16
#endif // CFG_RF_xxx
#endif // PLF_GAIA

#if ((PLF_GAIA) && (defined(CFG_RF_CALYPSO) || defined(CFG_CAL_HPMDM_BYPASS) ))
// The platform clock for Gaia Calypso and HPMDM BYPASS is twice the BT/BLE baseband clock
#define BB_CLK_MHZ          (PLF_CLK_MHZ/2)
#else // CFG_RF_xxx
#define BB_CLK_MHZ          (PLF_CLK_MHZ)
#endif // CFG_RF_xxx

#endif // CFG_ROM
/*
 * Display
 ****************************************************************************************
 */

/// Display controller enable/disable
#if defined(CFG_DISPLAY) && !defined(RP_HWSIM_BYPASS)
#define PLF_DISPLAY          1
#else // CFG_DISPLAY
#define PLF_DISPLAY          0
#endif // CFG_DISPLAY

/*
* LCD
****************************************************************************************
*/

// LCD enabled by default, or if Display menu configured
#define PLF_LCD             (1 || (PLF_DISPLAY)) && !defined(RP_HWSIM_BYPASS)

/*
* MAILBOX
****************************************************************************************
*/

// Mailbox enabled for GAIA by default, and otherwise disabled
#define PLF_MBOX            (1 && (PLF_GAIA)) && !defined(RP_HWSIM_BYPASS)


/*
* Bi-Directional RAM for Mailbox I/F
****************************************************************************************
*/
#if (PLF_GAIA)
/// BRAM location in address map
#define BRAM_BASE_ADDRESS	(0x50000000)

#define BRAM_MAILBOX_OFFSET       (0x00000)
#define BRAM_MAILBOX_MAX          (0x0FFFF)

#define BRAM_RF_DATA_OFFSET       (0x10000)
#define BRAM_RF_DATA_MAX          (0x1FFF0)
#define BRAM_RF_DATA_CSUM_OFFSET  (0x1FFF0)

#define BRAM_FW_SIZE_OFFSET1      (0x1FFF4)
#define BRAM_FW_CSUM_OFFSET1      (0x1FFF8)
#define BRAM_HOSTIO_VER_OFFSET1   (0x1FFFC)

#define BRAM_TFT_MSG_OFFSET       (0x20000)
#define BRAM_TFT_MSG_MAX          (0x2FFFF)

#define BRAM_RF_PROG_OFFSET       (0x30000)
#define BRAM_RF_PROG_MAX          (0x3FFF0)
#define BRAM_RF_PROG_CSUM_OFFSET  (0x3FFF0)

#define BRAM_FW_SIZE_OFFSET2      (0x3FFF4)
#define BRAM_FW_CSUM_OFFSET2      (0x3FFF8)
#define BRAM_HOSTIO_VER_OFFSET2   (0x3FFFC)

// Expected HostIO major version (8 bits) - incremented when the Linux / RISCV firmware API is impacted (memory map change in FPGA, new API format)
#define BRAM_HOSTIO_VER_MAJOR     (4)


/// GAIA LEDS location in address map
#define GAIA_LEDS_BASE_ADDRESS (0x48000000)
/// GAIA PB location in address map
#define GAIA_PB_BASE_ADDRESS (GAIA_LEDS_BASE_ADDRESS + 0x20)
#endif //(PLF_GAIA)

/*
 * NVDS
 ****************************************************************************************
 */

/// NVDS
#ifdef CFG_NVDS
#define PLF_NVDS             1
#else // CFG_NVDS
#define PLF_NVDS             0
#endif // CFG_NVDS

/*
 * UART
 ****************************************************************************************
 */

/// UART
#define PLF_UART             1

#define PLF_UART2            PLF_GAIA

/*
 * SPI
 ****************************************************************************************
 */

/// SPI
#define PLF_SPI              1


/*
 * I2C
 ****************************************************************************************
 */

/// I2C
#define PLF_I2C              1

/*
 * PS2
 ****************************************************************************************
 */

/// PS2
#ifdef CFG_PS2
#define PLF_PS2             1
#else // CFG_PS2
#define PLF_PS2             0
#endif // CFG_PS2

/*
 * Accelerometer
 ****************************************************************************************
 */

/// Accelerometer
#define PLF_ACC              0

/*
 * RTC
 ****************************************************************************************
 */

/// RTC
#ifdef CFG_RTC
#define PLF_RTC             1
#else // CFG_RTC
#define PLF_RTC             0
#endif // CFG_RTC

/*
 * LE AUDIO PATH
 ****************************************************************************************
 */
/// AUDIO path enabled
#ifdef CFG_PCM
#define PLF_PCM   1
#else // CFG_PCM
#define PLF_PCM   0
#endif // CFG_PCM

/*
 * Joystick
 ****************************************************************************************
 */

/// Display controller enable/disable
#define PLF_JOYSTICK         (PLF_BUBBLE)

/*
 * NVDS mapping in flash/ram
 ****************************************************************************************
 */

#if (PLF_GAIA)
#if defined(CFG_OLD_MMAP)
/// NVDS location in RAM : 0x0010F800
#define NVDS_BASE_ADDRESS        (0x0010F800)
#else // !defined(CFG_OLD_MMAP)
/// NVDS location in RAM : 0x001F0000
#define NVDS_BASE_ADDRESS        (0x001F0000)
#endif // !defined(CFG_OLD_MMAP)
#else // !(PLF_GAIA)
/// NVDS location in FLASH : 0x00F80000 (15.5 MB)
#define NVDS_BASE_ADDRESS        (0x00F80000)
#endif // !(PLF_GAIA)

/// NVDS size in FLASH/RAM : 0x00000800 (2K)
#define NVDS_MEM_SIZE            (0x00000800)


/*
 * DEFINES
 ****************************************************************************************
 */

/// Possible errors detected by FW
#define    RESET_NO_ERROR         0x00000000
#define    RESET_MEM_ALLOC_FAIL   0xF2F2F2F2

/// Reset platform and stay in ROM
#define    RESET_TO_ROM           0xA5A5A5A5
/// Reset platform and reload FW
#define    RESET_AND_LOAD_FW      0xC3C3C3C3

/// Exchange memory size limit
#if defined(BT_DUAL_MODE) && BT_DUAL_MODE
#define    EM_SIZE_LIMIT          0x10000
#else
#define    EM_SIZE_LIMIT          0x8000
#endif


/**
 * EM Fetch time (in us)
 *  - EM fetch: 30us (worst case at 26Mhz)
 *  - HW logic: 10us (worst case at 26Mhz)
 */
#define PLF_EM_FETCH_TIME_US        40

/**
 * EM update time (in us):
 *    - HW CS Update is 18 access
 *    - HW Tx Desc Update is 1 access
 *    - HW Rx Desc Update is 5 access
 *        => EM update at 26MHz Tx, Rx and CS is (18+1+5)*0.04*4 = 4us
 *    - HW logic: 10us (worst case)
 */
#define PLF_EM_UPDATE_TIME_US       14

/*
 * EXPORTED FUNCTION DECLARATION
 ****************************************************************************************
 */

#if 1//(RW_DEBUG_STACK_PROF)
/**
****************************************************************************************
* @brief Initialise stack memory area.
*
* This function initialises the stack memory with pattern for use in stack profiling.
****************************************************************************************
*/
void stack_init(void);

/**
 ****************************************************************************************
 * @brief Compute size of SW stack used.
 *
 * This function is compute the maximum size stack used by SW.
 *
 * @return Size of stack used (in bytes)
 ****************************************************************************************
 */
uint32_t get_stack_usage(void);
#endif //(RW_DEBUG_STACK_PROF)

#if PLF_GAIA
/**
 ****************************************************************************************
 * @brief Fetch the HostIO version information
 *
 * This function fetches the HostIO version from BRAM.
 *
 * @return HostIO version, or 0 if error.
 ****************************************************************************************
 */
uint32_t get_hostio_version(void);

/**
 ****************************************************************************************
 * @brief Save NVDS RAM to platform storage
 *
 * This function instructs the HostIO to save the NVDS RAM to SD Card.
 *
 ****************************************************************************************
 */
void plf_nvds_save(void);

#endif

/**
 ****************************************************************************************
 * @brief Re-boot FW.
 *
 * This function is used to re-boot the FW when error has been detected, it is the end of
 * the current FW execution.
 * After waiting transfers on UART to be finished, and storing the information that
 * FW has re-booted by itself in a non-loaded area, the FW restart by branching at FW
 * entry point.
 *
 * Note: when calling this function, the code after it will not be executed.
 *
 * @param[in] error      Error detected by FW
 ****************************************************************************************
 */
void platform_reset(uint32_t error);

#if (PLF_DEBUG)
/**
 ****************************************************************************************
 * @brief Print the assertion error reason and loop forever.
 *
 * @param condition C string containing the condition.
 * @param file C string containing file where the assertion is located.
 * @param line Line number in the file where the assertion is located.
 ****************************************************************************************
 */
void assert_err(const char *condition, const char * file, int line);

/**
 ****************************************************************************************
 * @brief Print the assertion error reason and loop forever.
 * The parameter value that is causing the assertion will also be disclosed.
 *
 * @param param0 parameter value 0.
 * @param param1 parameter value 1.
 * @param file C string containing file where the assertion is located.
 * @param line Line number in the file where the assertion is located.
 ****************************************************************************************
 */
void assert_param(int param0, int param1, const char * file, int line);

/**
 ****************************************************************************************
 * @brief Print the assertion warning reason.
 *
 * @param param0 parameter value 0.
 * @param param1 parameter value 1.
 * @param file C string containing file where the assertion is located.
 * @param line Line number in the file where the assertion is located.
 ****************************************************************************************
 */
void assert_warn(int param0, int param1, const char * file, int line);

/**
 ****************************************************************************************
 * @brief Dump data value into FW.
 *
 * @param data start pointer of the data.
 * @param length data size to dump
 ****************************************************************************************
 */
void dump_data(uint8_t* data, uint16_t length);

#endif //PLF_DEBUG


/*
 * ASSERTION CHECK
 ****************************************************************************************
 */
#if (PLF_DEBUG)
/// Assertions showing a critical error that could require a full system reset
#define ASSERT_ERR(cond)                              \
    do {                                              \
        if (!(cond)) {                                \
            assert_err(#cond, __MODULE__, __LINE__);  \
        }                                             \
    } while(0)

/// Assertions showing a critical error that could require a full system reset
#define ASSERT_INFO(cond, param0, param1)             \
    do {                                              \
        if (!(cond)) {                                \
            assert_param((int)param0, (int)param1, __MODULE__, __LINE__);  \
        }                                             \
    } while(0)

/// Assertions showing a non-critical problem that has to be fixed by the SW
#define ASSERT_WARN(cond, param0, param1)                             \
    do {                                              \
        if (!(cond)) {                                \
            assert_warn((int)param0, (int)param1, __MODULE__, __LINE__); \
        }                                             \
    } while(0)

/// DUMP data array present in the SW.
#define DUMP_DATA(data, length) \
    dump_data((uint8_t*)data, length)
#else
/// Assertions showing a critical error that could require a full system reset
#define ASSERT_ERR(cond)

/// Assertions showing a critical error that could require a full system reset
#define ASSERT_INFO(cond, param0, param1)

/// Assertions showing a non-critical problem that has to be fixed by the SW
#define ASSERT_WARN(cond, param0, param1)

/// DUMP data array present in the SW.
#define DUMP_DATA(data, length)
#endif //PLF_DEBUG


/// DUMP HCI packet
#define DUMP_HCI(type, direction, data, length)
/// DUMP HCI packet
#define DUMP_HCI_2(type, direction, hdr_data, hdr_length, data, length)
/// DUMP HCI packet in unpacked format
#define DUMP_UPK_HCI(evttype, direction, code, data, length)
/// DUMP String using printf format
#define DUMP_STR(level, format, ...)
/// Print a string using printf format when argument list is available
#define PRINT_STR(level, format, arg_list)

/// Trace data into a VCD
#define DBG_DATA_TRACE(data, size)

/// Trace data allocation
#define DBG_DATA_ALLOC(...)

/// Trace data free
#define DBG_DATA_FREE(data)

/// Trace Function Enter
#define DBG_FUNC_ENTER(func)

/// Trace Function Exit
#define DBG_FUNC_EXIT(func)

/// Control memory access
#define DBG_MEM_GRANT_CTRL(mem_ptr, enable)

/// Set permission onto a specific memory block
#define DBG_MEM_PERM_SET(mem_ptr, size, write_en, read_en, init_clr)

/// Mark memory initialized
#define DBG_MEM_INIT(mem_ptr, size)

/// Object allocated in shared memory - check linker script
#define __SHARED __attribute__ ((section("shram")))

/// @} DRIVERS


uint8_t flash_identify(uint8_t* id, void(*callback)(void));




#endif // _ARCH_H_

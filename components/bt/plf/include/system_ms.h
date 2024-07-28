/**
 ****************************************************************************************
 *
 * @file system_ms.h
 *
 * @brief define arm cm4p SOC architecture
 *
 * Copyright (C) ASR
 *
 ****************************************************************************************
 */


/*************************   **************************************/
#ifndef __SYSTEM_MS_H__
#define __SYSTEM_MS_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#ifdef CORTEX_M3
#include "core_cm3.h"
#else
#include "NUCLEI_N.h"
#include "nmsis_core.h"
#endif
#include "module_base_address.h"
	 
//hightest interrupt priority is configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY&((1<<__NVIC_PRIO_BITS)-1)=5
//lowest interrupt priority is configLIBRARY_LOWEST_INTERRUPT_PRIORITY&((1<<__NVIC_PRIO_BITS)-1)=7
//set normal interrupt priority 6
#define configLIBRARY_NORMAL_INTERRUPT_PRIORITY 6

/******************************************************************************/
/*                         macro definition                              */
/******************************************************************************/
#define SYS_REG_BASE                0x40110000

#define MODE_SEL_REG                (SYS_REG_BASE + 0x9C)
#define FLASH_BOOT_MODE             0
#define UART_BOOT_MODE              1
#define FLASH_DIRECTLY_BOOT_MODE    2
#define MODE_SEL_MASK               3

#define PINMUX_CTRL_REG0            (SYS_REG_BASE + 0x68) //pad0-7
#define PINMUX_CTRL_REG1            (SYS_REG_BASE + 0x6C) //pad8-15
#define PINMUX_CTRL_REG2            (SYS_REG_BASE + 0x70) //pad16-23
#define PINMUX_CTRL_REG3            (SYS_REG_BASE + 0x74) //pad24-31

#define FLASH_EMPTY_DATA            0xFFFFFFFF
#define FLASH_MAX_SIZE              0x80000 //0x80000 - 512KB   0x100000 - 1MB

#define FLASH_START_ADDR            0x20000000
#define BT_START_ADDR               FLASH_START_ADDR

#if (0x100000 == FLASH_MAX_SIZE)
#define APP_START_ADDR              0x20048000 //
#define OTA_START_ADDR              0x2009C000 //
#define OTA_INFO_ADDR               0x20008000 //param1
#define OTA_BK_ADDR                 0x20009000 //param4 for OTA breakpoint
#define NVDS_START_ADDR             0x2000A000 //param3

#define BT_MAX_SIZE                 0x8000
#define IMAGE_MAX_SIZE              0x54000 //336KB
#define APP_MAX_SIZE                IMAGE_MAX_SIZE
#define OTA_MAX_SIZE                IMAGE_MAX_SIZE
#define OTA_INFO_SIZE               0x1000
#define OTA_BK_MAX_SIZE             0x1000
#define NVDS_MAX_SIZE               0x2000

#elif (0x80000 == FLASH_MAX_SIZE)
#ifdef _DACHUANG_VENDOR_
#define APP_START_ADDR              FLASH_START_ADDR
#define OTA_START_ADDR              0x20069000
#define OTA_INFO_ADDR               0x20008000 //param1
#define OTA_BK_ADDR                 0x20009000 //param4 for OTA breakpoint
#define NVDS_START_ADDR             0x2000A000 //param3

#define BT_MAX_SIZE                 0x8000
#define IMAGE_MAX_SIZE              0x21000
#define APP_MAX_SIZE                0x21000
#define OTA_MAX_SIZE                0x17000
#define OTA_INFO_SIZE               0x1000
#define OTA_BK_MAX_SIZE             0x1000
#define NVDS_MAX_SIZE               0x2000
#else
#define APP_START_ADDR              FLASH_START_ADDR //0x10048000
#define OTA_START_ADDR              0x20064000 //
#define OTA_INFO_ADDR               0x20008000 //param1
#define OTA_BK_ADDR                 0x20009000 //param4 for OTA breakpoint
#define NVDS_START_ADDR             0x2000A000 //param3

#define BT_MAX_SIZE                 0x8000  //32KB
#define IMAGE_MAX_SIZE              0x1C000 //112KB
#define APP_MAX_SIZE                IMAGE_MAX_SIZE
#define OTA_MAX_SIZE                IMAGE_MAX_SIZE
#define OTA_INFO_SIZE               0x1000  //4KB
#define OTA_BK_MAX_SIZE             0x1000  //4KB
#define NVDS_MAX_SIZE               0x2000  //8KB
#endif
#endif //FLASH_MAX_SIZE



#define VTOR_NEW_ADDR               0x10000000
#define VTOR_OLD_ADDR               0x20000000

#define RAM_START_ADDR              0x10000000
#define RAM_LAYOUT_REG              (SYS_REG_BASE + 0x84)
#define RAM_56K_EM_32K              0
#define RAM_64K_EM_24K              2
#define RAM_72K_EM_16K              3


extern uint32_t SystemCoreClock;
extern uint32_t SystemCoreClockSel;

#define XTAL_16M                    16000000 //16M
#define FPGA_CLK                    32000000 //32M

#define DIG_SEL_XO16M               0x00000001
#define DIG_SEL_DPLL_32M            0x00000002
#define DIG_SEL_DPLL_48M            0x00000004
#define DIG_SEL_DPLL_64M            0x00000008
#define DIG_SEL_DPLL_96M            0x00000010
#define DIG_SEL_DPLL_128M           0x00000020
#define DIG_SEL_RCO16M              0x00000040

#define CLK_8M                      8000000
#define CLK_16M                     16000000
#define CLK_32M                     32000000
#define CLK_48M                     48000000
#define CLK_64M                     64000000
#define CLK_96M                     96000000
#define CLK_128M                    128000000

#define SYSTEM_CLOCK                (CLK_32M) // soc clock

#ifdef _IC_CODE_
#define UART_CLOCK                  (CLK_32M)
#else
#define UART_CLOCK                  (24000000) 
#endif // _IC_CODE_
#define LPUART_CLOCK                32000   




/**
 *****************************************************************************************
 * @brief Invoke the wait for interrupt procedure of the processor.
 *
 * @warning It is suggested that this macro is called while the interrupts are disabled
 * to have performed the checks necessary to decide to move to sleep mode.
 *****************************************************************************************
 */
 #if 0
#define WFI()                                                                       \
    /* The stop instruction signals external hardware that it should stop     */    \
    /* the processor clock so that the processor can enter a low power mode.  */    \
    __asm volatile( "wfi" );
#endif
void vector_table_reloc(void);

#endif //__SYSTEM_MS_H__


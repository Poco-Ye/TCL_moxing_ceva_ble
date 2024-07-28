/**
 ****************************************************************************************
 *
 * @file ll.h
 *
 * @brief Declaration of low level functions.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef LL_H_
#define LL_H_

//#include "co_math.h"
#include <stdint.h>              // standard integer functions

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

//#ifdef cpu_zeroriscy
//#define INTE_EN CO_BIT(3)
//#else
//#define INTE_EN CO_BIT(0)
//#endif
#define INTE_EN 0x08

/**
 ****************************************************************************************
 * @brief Enable interrupts globally in the system.
 * This macro must be used when the initialization phase is over and the interrupts
 * can start being handled by the system.
 ****************************************************************************************
 */
#define GLOBAL_INT_START()                                                        \
do {                                                                              \
    int mstatus;                                                                  \
    __asm__ volatile ("csrr %0, mstatus": "=r" (mstatus));                        \
    mstatus |= INTE_EN;                                                           \
    __asm__ volatile ("csrw mstatus, %0" : /* no output */ : "r" (mstatus));      \
} while(0)

/**
 ****************************************************************************************
 * @brief Disable interrupts globally in the system.
 * This macro must be used when the system wants to disable all the interrupt
 * it could handle.
 ****************************************************************************************
 */
#define GLOBAL_INT_STOP()                                                         \
do {                                                                              \
    int mstatus;                                                                  \
    __asm__ volatile ("csrr %0, mstatus": "=r" (mstatus));                        \
    mstatus &= ~(INTE_EN);                                                        \
    __asm__ volatile ("csrw mstatus, %0" : /* no output */ : "r" (mstatus));      \
    /*asm("csrw 0x300, %0" : : "r" (0x0) );                                   */  \
} while(0)

/**
 ****************************************************************************************
 * @brief Disable interrupts globally in the system.
 * This macro must be used in conjunction with the @ref GLOBAL_INT_RESTORE macro since this
 * last one will close the brace that the current macro opens.  This means that both
 * macros must be located at the same scope level.
 ****************************************************************************************
 */
#define GLOBAL_INT_DISABLE()                                                      \
do {                                                                              \
    int irq_rest;                                                                 \
    int mstatus_tmp;                                                              \
                                                                                  \
    __asm__ volatile ("csrr %0, mstatus": "=r" (mstatus_tmp));                    \
                                                                                  \
    /*Store mstatus in a local variable*/                                         \
    irq_rest = mstatus_tmp;                                                       \
                                                                                  \
    /* Clear IE bit */                                                            \
    mstatus_tmp &= ~(INTE_EN);                                                    \
                                                                                  \
    __asm__ volatile ("csrw mstatus, %0" : /* no output */ : "r" (mstatus_tmp));  \


/**
 ****************************************************************************************
 * @brief Restore interrupts from the previous global disable.
 * @sa GLOBAL_INT_DISABLE
 ****************************************************************************************
 */
#define GLOBAL_INT_RESTORE()                                                      \
    /* Restore mstatus to its previous value */                                   \
    __asm__ volatile ("csrw mstatus, %0" : /* no output */ : "r" (irq_rest));     \
} while(0);

/**
 *****************************************************************************************
 * @brief Wait for interrupt.
 * Wait on next interrupt (sleep mode).
 *****************************************************************************************
 */

#define WFI()                                                                            \
do {                                                                                     \
    __asm__ volatile ("wfi":::);                                                           \
} while (0)

/**
 *****************************************************************************************
 * @brief Jump to RAM entry point.
 *
 * @Jump to RAM entry point from boot ROM.
 *****************************************************************************************
 */
#define JUMP_TO_RAM_ENTRY_POINT()                                                   \
    /* Jump to RAM entry point */                                                   \
    asm("ecall" :: "r"(0x0));


#endif // LL_H_

/*
 * Copyright (c) 2009-2018 Arm Limited. All rights reserved.
 * Copyright (c) 2019 Nuclei Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/******************************************************************************
 * @file     system_NUCLEI_N.c
 * @brief    NMSIS Nuclei Device Peripheral Access Layer Source File for
 *           Nuclei N Device
 * @version  V1.10
 * @date     30. July 2021
 ******************************************************************************/

#include <stdint.h>
//added by jiangde
#include "NUCLEI_N.h"
#include "log.h"

#include "ms_common.h"

void _premain_init (void);
/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
#define XTAL            (1000000U)       /* Oscillator frequency             */

//#define SYSTEM_CLOCK    (3 * XTAL)  // ??
#define SYSTEM_CLOCK    48000000U


/**
 * \defgroup  NMSIS_Core_SystemConfig       System Device Configuration
 * \brief Functions for system init, clock setup and interrupt/exception/nmi functions available in system_<device>.c.
 * \details
 * Nuclei provides a template file **system_Device.c** that must be adapted by
 * the silicon vendor to match their actual device. As a <b>minimum requirement</b>,
 * this file must provide:
 *  -  A device-specific system configuration function, \ref SystemInit.
 *  -  A global variable that contains the system frequency, \ref SystemCoreClock.
 *  -  A global eclic configuration initialization, \ref ECLIC_Init.
 *  -  Global \ref _premain_init and \ref _postmain_fini functions called right before calling main function.
 *  -  Vendor customized interrupt, exception and nmi handling code, see \ref NMSIS_Core_IntExcNMI_Handling
 *
 * The file configures the device and, typically, initializes the oscillator (PLL) that is part
 * of the microcontroller device. This file might export other functions or variables that provide
 * a more flexible configuration of the microcontroller system.
 *
 * And this file also provided common interrupt, exception and NMI exception handling framework template,
 * Silicon vendor can customize these template code as they want.
 *
 * \note Please pay special attention to the static variable \c SystemCoreClock. This variable might be
 * used throughout the whole system initialization and runtime to calculate frequency/time related values.
 * Thus one must assure that the variable always reflects the actual system clock speed.
 *
 * \note \ref _init function and \ref _fini function are now deprecated, but still need to provide a empty implementation
 * here, due to newlibc still need it. Please put your pre- and post- main steps in \ref _premain_init and \ref _postmain_fini
 *
 * \attention
 * Be aware that a value stored to \c SystemCoreClock during low level initialization (i.e. \c SystemInit()) might get
 * overwritten by C libray startup code and/or .bss section initialization.
 * Thus its highly recommended to call \ref SystemCoreClockUpdate at the beginning of the user \c main() routine.
 *
 * @{
 */

/*----------------------------------------------------------------------------
  System Core Clock Variable
 *----------------------------------------------------------------------------*/
/**
 * \brief      Variable to hold the system core clock value
 * \details
 * Holds the system core clock, which is the system clock frequency supplied to the SysTick
 * timer and the processor core clock. This variable can be used by debuggers to query the
 * frequency of the debug timer or to configure the trace clock speed.
 *
 * \attention
 * Compilers must be configured to avoid removing this variable in case the application
 * program is not using it. Debugging systems require the variable to be physically
 * present in memory so that it can be examined to configure the debugger.
 */
uint32_t SystemCoreClock = SYSTEM_CLOCK;  /* System Clock Frequency (Core Clock) */


/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/

/**
 * \brief      Function to update the variable \ref SystemCoreClock
 * \details
 * Updates the variable \ref SystemCoreClock and must be called whenever the core clock is changed
 * during program execution. The function evaluates the clock register settings and calculates
 * the current core clock.
 */
void SystemCoreClockUpdate (void)  /* Get Core Clock Frequency */
{
    SystemCoreClock = SYSTEM_CLOCK;
}

/**
 * \brief      Function to Initialize the system.
 * \details
 * Initializes the microcontroller system. Typically, this function configures the
 * oscillator (PLL) that is part of the microcontroller device. For systems
 * with a variable clock speed, it updates the variable \ref SystemCoreClock.
 * SystemInit is called from the file <b>startup<i>_device</i></b>.
 */
void SystemInit (void)
{
    SystemCoreClock = SYSTEM_CLOCK;
	
	  /* __ICACHE_PRESENT and __DCACHE_PRESENT are defined in <Device>.h */
#if defined(__ICACHE_PRESENT) && __ICACHE_PRESENT == 1
	  EnableICache();
#endif
#if defined(__DCACHE_PRESENT) && __DCACHE_PRESENT == 1
	  EnableDCache();
#endif

  //  _premain_init();
}

/**
 * \defgroup  NMSIS_Core_IntExcNMI_Handling   Interrupt and Exception and NMI Handling
 * \brief Functions for interrupt, exception and nmi handle available in system_<device>.c.
 * \details
 * Nuclei provide a template for interrupt, exception and NMI handling. Silicon Vendor could adapat according
 * to their requirement. Silicon vendor could implement interface for different exception code and
 * replace current implementation.
 *
 * @{
 */
/** \brief Max exception handler number, don't include the NMI(0xFFF) one */
#define MAX_SYSTEM_EXCEPTION_NUM        12
/**
 * \brief      Store the exception handlers for each exception ID
 * \note
 * - This SystemExceptionHandlers are used to store all the handlers for all
 * the exception codes Nuclei N/NX core provided.
 * - Exception code 0 - 11, totally 12 exceptions are mapped to SystemExceptionHandlers[0:11]
 * - Exception for NMI is also re-routed to exception handling(exception code 0xFFF) in startup code configuration, the handler itself is mapped to SystemExceptionHandlers[MAX_SYSTEM_EXCEPTION_NUM]
 */
static unsigned long SystemExceptionHandlers[MAX_SYSTEM_EXCEPTION_NUM+1];

/**
 * \brief      Exception Handler Function Typedef
 * \note
 * This typedef is only used internal in this system_NUCLEI_N.c file.
 * It is used to do type conversion for registered exception handler before calling it.
 */
typedef void (*EXC_HANDLER) (unsigned long mcause, unsigned long sp);

/**
 * \brief      System Default Exception Handler
 * \details
 * This function provided a default exception and NMI handling code for all exception ids.
 * By default, It will just print some information for debug, Vendor can customize it according to its requirements.
 */
static void system_default_exception_handler(unsigned long mcause, unsigned long sp)
{
	ms_log_write(MS_LOG_ERROR, MS_DRIVER,"MCAUSE: 0x%lx\r\n", mcause);
	ms_log_write(MS_LOG_ERROR, MS_DRIVER,"MEPC  : 0x%lx\r\n", __RV_CSR_READ(CSR_MEPC));
	ms_log_write(MS_LOG_ERROR, MS_DRIVER,"MTVAL : 0x%lx\r\n", __RV_CSR_READ(CSR_MBADADDR));
    Exception_DumpFrame(sp);
    while(1);
}

/**
 * \brief      Initialize all the default core exception handlers
 * \details
 * The core exception handler for each exception id will be initialized to \ref system_default_exception_handler.
 * \note
 * Called in \ref _init function, used to initialize default exception handlers for all exception IDs
 */
static void Exception_Init(void)
{
    for (int i = 0; i < MAX_SYSTEM_EXCEPTION_NUM+1; i++) {
        SystemExceptionHandlers[i] = (unsigned long)system_default_exception_handler;
    }
}

/**
 * \brief      Dump Exception Frame
 * \details
 * This function provided feature to dump exception frame stored in stack.
 */
void Exception_DumpFrame(unsigned long sp)
{
    EXC_Frame_Type *exc_frame = (EXC_Frame_Type *)sp;

 #ifndef __riscv_32e
    ms_log_write(MS_LOG_ERROR, MS_DRIVER, "ra: 0x%x, tp: 0x%x, t0: 0x%x, t1: 0x%x, t2: 0x%x, t3: 0x%x, t4: 0x%x, t5: 0x%x, t6: 0x%x\r\n" \
            "\t\ta0: 0x%x, a1: 0x%x, a2: 0x%x, a3: 0x%x, a4: 0x%x, a5: 0x%x, a6: 0x%x, a7: 0x%x\r\n" \
            "\t\tmcause: 0x%x, mepc: 0x%x, msubm: 0x%x\r\n", exc_frame->ra, exc_frame->tp, exc_frame->t0, \
            exc_frame->t1, exc_frame->t2, exc_frame->t3, exc_frame->t4, exc_frame->t5, exc_frame->t6, \
            exc_frame->a0, exc_frame->a1, exc_frame->a2, exc_frame->a3, exc_frame->a4, exc_frame->a5, \
            exc_frame->a6, exc_frame->a7, exc_frame->mcause, exc_frame->mepc, exc_frame->msubm);
 #else
    ms_log_write(MS_LOG_ERROR, MS_DRIVER,"ra: 0x%x, tp: 0x%x, t0: 0x%x, t1: 0x%x, t2: 0x%x\r\n" \
            "\t\ta0: 0x%x, a1: 0x%x, a2: 0x%x, a3: 0x%x, a4: 0x%x, a5: 0x%x\r\n" \
            "\t\tmcause: 0x%x, mepc: 0x%x, msubm: 0x%x\r\n", exc_frame->ra, exc_frame->tp, exc_frame->t0, \
            exc_frame->t1, exc_frame->t2, exc_frame->a0, exc_frame->a1, exc_frame->a2, exc_frame->a3, \
            exc_frame->a4, exc_frame->a5, exc_frame->mcause, exc_frame->mepc, exc_frame->msubm);
 #endif
}

/**
 * \brief       Register an exception handler for exception code EXCn
 * \details
 * * For EXCn < \ref MAX_SYSTEM_EXCEPTION_NUM, it will be registered into SystemExceptionHandlers[EXCn-1].
 * * For EXCn == NMI_EXCn, it will be registered into SystemExceptionHandlers[MAX_SYSTEM_EXCEPTION_NUM].
 * \param   EXCn    See \ref EXCn_Type
 * \param   exc_handler     The exception handler for this exception code EXCn
 */
void Exception_Register_EXC(uint32_t EXCn, unsigned long exc_handler)
{
    if ((EXCn < MAX_SYSTEM_EXCEPTION_NUM) && (EXCn >= 0)) {
        SystemExceptionHandlers[EXCn] = exc_handler;
    } else if (EXCn == NMI_EXCn) {
        SystemExceptionHandlers[MAX_SYSTEM_EXCEPTION_NUM] = exc_handler;
    }
}

/**
 * \brief       Get current exception handler for exception code EXCn
 * \details
 * * For EXCn < \ref MAX_SYSTEM_EXCEPTION_NUM, it will return SystemExceptionHandlers[EXCn-1].
 * * For EXCn == NMI_EXCn, it will return SystemExceptionHandlers[MAX_SYSTEM_EXCEPTION_NUM].
 * \param   EXCn    See \ref EXCn_Type
 * \return  Current exception handler for exception code EXCn, if not found, return 0.
 */
unsigned long Exception_Get_EXC(uint32_t EXCn)
{
    if ((EXCn < MAX_SYSTEM_EXCEPTION_NUM) && (EXCn >= 0)) {
        return SystemExceptionHandlers[EXCn];
    } else if (EXCn == NMI_EXCn) {
        return SystemExceptionHandlers[MAX_SYSTEM_EXCEPTION_NUM];
    } else {
        return 0;
    }
}

/**
 * \brief      Common NMI and Exception handler entry
 * \details
 * This function provided a command entry for NMI and exception. Silicon Vendor could modify
 * this template implementation according to requirement.
 * \remarks
 * - RISCV provided common entry for all types of exception. This is proposed code template
 *   for exception entry function, Silicon Vendor could modify the implementation.
 * - For the core_exception_handler template, we provided exception register function \ref Exception_Register_EXCn
 *   which can help developer to register your exception handler for specific exception number.
 */
uint32_t core_exception_handler(unsigned long mcause, unsigned long sp)
{
    uint32_t EXCn = (uint32_t)(mcause & 0X00000fff);
    EXC_HANDLER exc_handler;

    ms_log_write(MS_LOG_ERROR, MS_DRIVER, "-------------------fatal exception-------------\r\n" );
    switch (EXCn) {
    case 0:  ms_log_write(MS_LOG_ERROR, MS_DRIVER, "!<  Instruction address misaligned\r\n" );
        break;
    case 1:  ms_log_write(MS_LOG_ERROR, MS_DRIVER, "!<  Instruction access fault\r\n" );
        break;
    case 2:  ms_log_write(MS_LOG_ERROR, MS_DRIVER, "!<  Illegal instruction\r\n" );
        break;
    case 3:  ms_log_write(MS_LOG_ERROR, MS_DRIVER, "!<  Beakpoint\r\n" );
        break;
    case 4:  ms_log_write(MS_LOG_ERROR, MS_DRIVER, "!<  Load address misaligned\r\n" );
        break;
    case 5:  ms_log_write(MS_LOG_ERROR, MS_DRIVER, "!<  Load access fault\r\n" );
        break;
    case 6:  ms_log_write(MS_LOG_ERROR, MS_DRIVER, "!<  Store or AMO address misaligned\r\n" );
        break;
    case 7:  ms_log_write(MS_LOG_ERROR, MS_DRIVER, "!<  Store or AMO access fault\r\n" );
        break;
    case 8:  ms_log_write(MS_LOG_ERROR, MS_DRIVER, "!<  Environment call from User mode\r\n" );
        break;
    case 11:  ms_log_write(MS_LOG_ERROR, MS_DRIVER, "!<  Environment call from Machine mode\r\n" );
        break;
    case 0xfff:  ms_log_write(MS_LOG_ERROR, MS_DRIVER, "!<  NMI interrupt\r\n" );
        break;
    default:  ms_log_write(MS_LOG_ERROR, MS_DRIVER, "!unknown exception\r\n" );
    break;
    }

    if ((EXCn < MAX_SYSTEM_EXCEPTION_NUM) && (EXCn >= 0)) {
        exc_handler = (EXC_HANDLER)SystemExceptionHandlers[EXCn];
    } else if (EXCn == NMI_EXCn) {
        exc_handler = (EXC_HANDLER)SystemExceptionHandlers[MAX_SYSTEM_EXCEPTION_NUM];
    } else {
        exc_handler = (EXC_HANDLER)system_default_exception_handler;
    }
    if (exc_handler != NULL) {
        exc_handler(mcause, sp);
    }
    return 0;
}

/**
 * \brief Initialize Global ECLIC Config
 * \details
 * ECLIC needs be initialized after boot up,
 * Vendor could also change the initialization
 * configuration.
 */
void ECLIC_Init(void)
{
    ECLIC_SetMth(0);
    ECLIC_SetCfgNlbits(__ECLIC_INTCTLBITS);
}

/**
 * \brief  Initialize a specific IRQ and register the handler
 * \details
 * This function set vector mode, trigger mode and polarity, interrupt level and priority,
 * assign handler for specific IRQn.
 * \param [in]  IRQn        NMI interrupt handler address
 * \param [in]  shv         \ref ECLIC_NON_VECTOR_INTERRUPT means non-vector mode, and \ref ECLIC_VECTOR_INTERRUPT is vector mode
 * \param [in]  trig_mode   see \ref ECLIC_TRIGGER_Type
 * \param [in]  lvl         interupt level
 * \param [in]  priority    interrupt priority
 * \param [in]  handler     interrupt handler, if NULL, handler will not be installed
 *
 * \return       -1 means invalid input parameter. 0 means successful.
 * \remarks
 * - This function use to configure specific eclic interrupt and register its interrupt handler and enable its interrupt.
 * - If the vector table is placed in read-only section(FLASHXIP mode), handler could not be installed
 */
int32_t ECLIC_Register_IRQ(IRQn_Type IRQn, uint8_t shv, ECLIC_TRIGGER_Type trig_mode, uint8_t lvl, uint8_t priority, void *handler)
{
    if ((IRQn > SOC_INT_MAX) || (shv > ECLIC_VECTOR_INTERRUPT) \
        || (trig_mode > ECLIC_NEGTIVE_EDGE_TRIGGER )) {
        return -1;
    }

    /* set interrupt vector mode */
    ECLIC_SetShvIRQ(IRQn, shv);
    /* set interrupt trigger mode and polarity */
    ECLIC_SetTrigIRQ(IRQn, trig_mode);
    /* set interrupt level */
    ECLIC_SetLevelIRQ(IRQn, lvl);
    /* set interrupt priority */
    ECLIC_SetPriorityIRQ(IRQn, priority);
    if (handler != NULL) {
        /* set interrupt handler entry to vector table */
        ECLIC_SetVector(IRQn, (rv_csr_t)handler);
    }
    /* enable interrupt */
    ECLIC_EnableIRQ(IRQn);
    return 0;
}
/** @} */ /* End of Doxygen Group NMSIS_Core_ExceptionAndNMI */

/**
 * \brief early init function before main
 * \details
 * This function is executed right before main function.
 * For RISC-V gnu toolchain, _init function might not be called
 * by __libc_init_array function, so we defined a new function
 * to do initialization
 */
void _premain_init(void)
{

    // TODO: Add code to set the system clock frequency value SystemCoreClock

    // TODO: Add code to initialize necessary gpio and basic uart for debug print

    /* Initialize exception default handlers */
    Exception_Init();
    /* ECLIC initialization, mainly MTH and NLBIT settings */
    ECLIC_Init();
}

/**
 * \brief finish function after main
 * \param [in]  status     status code return from main
 * \details
 * This function is executed right after main function.
 * For RISC-V gnu toolchain, _fini function might not be called
 * by __libc_fini_array function, so we defined a new function
 * to do initialization
 */
void _postmain_fini(int status)
{
    /* TODO: Add your own finishing code here, called after main */
}

/**
 * \brief _init function called in __libc_init_array()
 * \details
 * This `__libc_init_array()` function is called during startup code,
 * user need to implement this function, otherwise when link it will
 * error init.c:(.text.__libc_init_array+0x26): undefined reference to `_init'
 * \note
 * Please use \ref _premain_init function now
 */
void _init(void)
{
    /* Don't put any code here, please use _premain_init now */
}

/**
 * \brief _fini function called in __libc_fini_array()
 * \details
 * This `__libc_fini_array()` function is called when exit main.
 * user need to implement this function, otherwise when link it will
 * error fini.c:(.text.__libc_fini_array+0x28): undefined reference to `_fini'
 * \note
 * Please use \ref _postmain_fini function now
 */
void _fini(void)
{
    /* Don't put any code here, please use _postmain_fini now */
}




extern char _sbss[];
extern char _ebss[];
extern char _sdata[];
extern char _edata[];
extern char _sidata[];
extern char _sfunc[];
extern char _efunc[];
extern char _sifunc[];
extern char _sfunc_retram[];
extern char _efunc_retram[];
extern char _sifunc_retram[];
extern char _srwip_section[];
extern char _erwip_section[];
extern char _srwip_bss_section[];
extern char _rwip_start[];

void SystemLoopCopyFuncInit(void)
{
    char *addr_sram, *addr_flash;

    addr_sram = _sfunc;
    addr_flash = _sifunc;

    while(addr_sram < _efunc)
    {
        REG_WR(addr_sram, REG_RD(addr_flash));
        addr_sram += 4;
        addr_flash += 4;
    }

}

void SystemLoopCopyRetramFuncInit(void)
{
    char *addr_sram, *addr_flash;

	
    /*critical retram func segment*/
    addr_sram =_sfunc_retram; //(unsigned int *)0x10030c00;//_sfunc_retram;
    addr_flash = _sifunc_retram;
    while(addr_sram < _efunc_retram)
    {
        REG_WR(addr_sram, REG_RD(addr_flash));
        addr_sram += 4;
        addr_flash += 4;
	 
    }
	
}

void SystemLoopCopyRwipSection(void)
{
    char *addr_rwip_ram, *addr_flash;

    /*critical retram func segment*/
    addr_rwip_ram =_srwip_section;
    addr_flash = _rwip_start;
    while(addr_rwip_ram < _srwip_bss_section)
    {
        REG_WR(addr_rwip_ram, REG_RD(addr_flash));
        addr_rwip_ram += 4;
        addr_flash += 4;
    }

    while(addr_rwip_ram < _erwip_section)
    {
        REG_WR(addr_rwip_ram, 0x00);
	 addr_rwip_ram += 4;
    }

}

void SystemLoopCopyInit(void)
{
   // uint32_t pre_mode = ms_get_pre_mode();

   
   // if(MODE_POWERDOWN == pre_mode)
   // { 
        //FPGA platform sram is always on, AISC must be init when wakeup from deepsleep mode 
        SystemLoopCopyFuncInit();
        SystemLoopCopyRwipSection();
        SystemLoopCopyRetramFuncInit();
   // }
}

#if 0
void SystemLoopCopyInit(void)
{
    // do nothing  if BT not support
}
#endif

/** @} */ /* End of Doxygen Group NMSIS_Core_SystemAndClock */
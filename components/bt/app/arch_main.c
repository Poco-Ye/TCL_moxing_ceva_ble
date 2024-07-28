/**
 ****************************************************************************************
 *
 * @file arch_main.c
 *
 * @brief Main loop of the application.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */
/**
 * @addtogroup DRIVERS
 * @{
 */

/// Enable use of WFI (simulated for Zeroriscy CPU) 
#define WFI_EN

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stddef.h>    // standard definitions

#include <stdio.h>     // io definitions
#include <string.h>    // string manipulation and functions
#include "co_math.h"   // math definitions
#include "rwip.h"      // rw functions
#include "arch.h"      // architectural platform definitions
//#include "boot.h"      // boot functions
//#include "counter.h"   // counter functions
//#include "intcntl.h"   // interrupt controller functions
#include "gpio.h"      // gpio functions



#include "dbg.h"
#include "ms_uart.h"
#include "atiny_log.h"
#include "ms_pinmux_hal.h"

#include "pm_sleep.h"
#include "pwr.h"
#include "ms_pwr.h"
#include "ms_clock_hal.h"
//#include "ms_wdt.h"

#include "ble.h"
#include "mdm.h"
#include "ms_clock_hal.h"

#include "ms_section.h"

#include "ble.h"
#include "mdm.h"
#include "reg_ipcore.h"
#if (NVDS_SUPPORT)	
#include "nvds.h"
#endif
#include "log.h"
#include "lld.h" 
// #include "system.h"
#include "flash.h"
#include "timer.h"

/*
 * FUNCTION extern 
 ************************
 */
void sys_init(void);
//void UART0_init(void);
//void app_uart_config(uint8_t port);
//void app_uart_clk_config(void);




void user_clk_cfg(void)
{
    // set system clock
    //ms_set_clk_enable(UART0_CLK, 1);
    //ms_set_ctrl_clk_enable(UART0_CTRL_CLK, 1);
    //ms_set_clk_enable(UART2_CLK, 1);
    //ms_set_ctrl_clk_enable(UART2_CTRL_CLK, 1);
    //ms_set_ctrl_clk_enable(GPIO_CTRL_CLK, 1);
    //ms_set_clk_enable(RTC_CLK, 1);
   // ms_set_ctrl_clk_enable(RTC_CTRL_CLK, 1);	
    // set_clk_enable(UART0_CLK, 1);
    // set_ctrl_clk_enable(UART0_CTRL_CLK, 1);

	
    MS_CLOCK_HAL_CLK_ENABLE_UART1();

	/*MS_CLOCK_HAL_CLK_ENABLE_UART2_COMM_CLK();
	MS_CLOCK_HAL_CLK_ENABLE_UART2_PCLK(); */  // pyxue temporary, no sleep mode support
	//set_clk_enable(UART1_CLK, 1);
    //set_ctrl_clk_enable(UART1_CTRL_CLK, 1);
    //set_clk_enable(UART2_CLK, 1);
    //set_ctrl_clk_enable(UART2_CTRL_CLK, 1);

#if 0
    ms_set_clk_enable(KSCAN_CLK, 1);
    ms_set_ctrl_clk_enable(KSCAN_CTRL_CLK, 1);		
    ms_set_clk_enable(IR_CLK, 1);
    ms_set_ctrl_clk_enable(IR_CTRL_CLK, 1);		
    ms_set_clk_enable(TIMER0_CLK, 1);
    ms_set_clk_enable(TIMER1_CLK, 1);	
    ms_set_ctrl_clk_enable(TIMER_CTRL_CLK, 1);		
#endif
    MS_CLOCK_HAL_CLK_ENABLE_KSCAN();
    MS_CLOCK_HAL_CLK_ENABLE_TIMERx(0);
    MS_CLOCK_HAL_CLK_ENABLE_TIMERx(1);
    MS_CLOCK_HAL_CLK_ENABLE_IR();
    MS_CLOCK_HAL_CLK_ENABLE_CPU_TIMER();

   // set_clk_enable(KSCAN_CLK, 1);
   // set_ctrl_clk_enable(KSCAN_CTRL_CLK, 1);      
   // set_clk_enable(IR_CLK, 1);
   // set_ctrl_clk_enable(IR_CTRL_CLK, 1);     
   // set_clk_enable(TIMER0_CLK, 1);
   // set_clk_enable(TIMER1_CLK, 1);   
   // set_ctrl_clk_enable(TIMER_CTRL_CLK, 1);

}



/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

/// Description of unloaded RAM area content
struct unloaded_area_tag
{
    /// status error
    uint32_t error;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
/// Creation of uart external interface api
const struct rwip_eif_api uart_api =
{
    uart0_read_noblock,
    uart0_write,
    NULL,
    NULL,
};*/


#if PLF_DEBUG
/// Variable to enable infinite loop on assert
static volatile int dbg_assert_block = 1;
#endif //PLF_DEBUG

/// Pointer to access unloaded RAM area
static struct unloaded_area_tag* unloaded_area;

/// Variable storing the reason of platform reset
static uint32_t error = RESET_NO_ERROR;

/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if PLF_DEBUG
void assert_err(const char *condition, const char * file, int line)
{
    char *fptr = (char *)&file[strlen(file)];

    // Find start of file name in path
    while ((fptr > file) && (*fptr != '/') && (*fptr != '\\'))
        fptr--;
    fptr++;

    TRC_REQ_SW_ASS_ERR(fptr, line, 0, 0);

    intcntl_set_interrupt_disable_all();

    rwip_assert(fptr, line, 0, 0, ASSERT_TYPE_ERROR);

    while(dbg_assert_block);
}

void assert_param(int param0, int param1, const char * file, int line)
{
    char *fptr = (char *)&file[strlen(file)];

    // Find start of file name in path
    while ((fptr > file) && (*fptr != '/') && (*fptr != '\\'))
        fptr--;
    fptr++;

    intcntl_set_interrupt_disable_all();

    TRC_REQ_SW_ASS_ERR(fptr, line, param0, param1);

    rwip_assert(fptr, line, param0, param1, ASSERT_TYPE_ERROR);

    while(dbg_assert_block);
}

void assert_warn(int param0, int param1, const char * file, int line)
{
    char *fptr = (char *)&file[strlen(file)];

    // Find start of file name in path
    while ((fptr > file) && (*fptr != '/') && (*fptr != '\\'))
        fptr--;
    fptr++;

    TRC_REQ_SW_ASS_WARN(fptr, line, param0, param1);

    rwip_assert(fptr, line, param0, param1, ASSERT_TYPE_WARNING);
}

void dump_data(uint8_t* data, uint16_t length)
{

}
#endif //PLF_DEBUG

#if (RW_DEBUG_STACK_PROF)
void stack_init(void)
{
   uint32_t* ptr = (uint32_t*)&ptr;
   ASSERT_ERR((_stack_top > (uint32_t*)&ptr) && ((uint32_t*)&ptr > _stack_bottom));
   while ((uint32_t)ptr > (uint32_t)_stack_bottom)
       *--ptr = STACK_INIT_PATTERN;
}

uint32_t get_stack_usage(void)
{
    uint32_t* ptr = _stack_bottom;

    while(*(ptr++) == STACK_INIT_PATTERN);

    return ((uint32_t)_stack_top - (uint32_t)ptr);
}
#endif //(RW_DEBUG_STACK_PROF)

#if PLF_GAIA
uint32_t get_hostio_version(void)
{
    volatile uint32_t* m_ver;
    uint32_t ver_info1;
    uint32_t ver_info2;

    // Workaround initial BRAM read issue
    m_ver = (uint32_t*)((uint32_t)BRAM_BASE_ADDRESS + BRAM_HOSTIO_VER_OFFSET1);
    ver_info1 = *(uint32_t*)m_ver; // dummy read
    ver_info1 = *(uint32_t*)m_ver; // re-read as volatile
    m_ver = (uint32_t*)((uint32_t)BRAM_BASE_ADDRESS + BRAM_HOSTIO_VER_OFFSET2);
    ver_info2 = *(uint32_t*)m_ver; // dummy read
    ver_info2 = *(uint32_t*)m_ver; // re-read as volatile

    // If inconsistent version info, return 0 to indicate error
    if (ver_info1 != ver_info2)
        ver_info1 = 0;

    return ver_info1;
}

void plf_nvds_save(void)
{

}
#endif

void platform_reset(uint32_t error)
{
    // Disable interrupts
    GLOBAL_INT_STOP();
/*  pyxue
    #if PLF_UART
    // Wait UART transfer finished
    uart_finish_transfers();
    #endif //PLF_UART
*/
    // Store information in unloaded area
    unloaded_area->error = error;

    #if (RW_DEBUG_STACK_PROF)
    // Initialise stack memory area
    stack_init();
    #endif //(RW_DEBUG_STACK_PROF)

    // Reset platform
    //plf_reset();  // to be supported pyxue

    // wait perform HW Reset to be executed.
    while(1);
}


/**
 ****************************************************************************************
 * @brief Main init function.
 ****************************************************************************************
 */

void main_init(PWR_MODE_Type startmode)
{
    /*
     ************************************************************************************
     * Platform initialization
     ************************************************************************************
     */

    // Initialize unloaded RAM area
    //unloaded_area_init();

    // Initialize the Interrupt Controller

    //intcntl_init();

    // Initialize GPIO module
    //gpio_init();

    // Initialize counter component
    //counter_init();

    #if (PLF_NVDS)
    // Initialize the NVDS
	 nvds_init((uint8_t *)NVDS_BASE_ADDRESS, NVDS_MEM_SIZE);
	{
		//static uint8_t aNvds[NVDS_MEM_SIZE];
		//nvds_init((uint8_t *)aNvds, NVDS_MEM_SIZE);
	}
    #endif // PLF_NVDS

    #if PLF_RTC
    // Initialize RTC module
    //rtc_init();
    #endif // PLF_RTC

    #if PLF_PS2
    // Initialize PS2 module
    //ps2_init();
    #endif //PLF_PS2
    #if HCI_TL_SUPPORT
    // Initialize the UART
    uart_init();
    #endif //PLF_UART
    #if PLF_ACC
    // Initialize the ACC
    //accel_init();
    #endif //PLF_ACC

    // Initialize platform
    //plf_init();

    #if (PLF_PCM || BT_EMB_PRESENT)
    // Initialize PLF Codec
    //plf_pcm_codec_init();
    #endif // (PLF_PCM || BT_EMB_PRESENT)

    #if (PLF_PCM)
    // Initialize LE audio path
    //pcm_init();
    #endif //PLF_PCM

    /*
     ************************************************************************************
     * RW SW stack initialization
     ************************************************************************************
     */

    // Initialize RW SW stack
    MS_LOGI(MS_DRIVER, "rwip_init\r\n");
    rwip_init(error);
    ATINY_LOG(LOG_INFO,"rwip_init init!");	

    #if BT_EMB_PRESENT
    // Enable BT IRQ
   // intcntl_enable_irq(INTCNTL_BT_IP);
    #endif //BT_EMB_PRESENT

    #if BLE_EMB_PRESENT
    // Enable BLE IRQ
    //intcntl_enable_irq(INTCNTL_BLE_IP);
    #endif //BLE_EMB_PRESENT

    #if (BT_DUAL_MODE)
    // Enable BTDM IRQ
    //intcntl_enable_irq(INTCNTL_BTDM_IP);
    #endif // (BT_DUAL_MODE)
    
    #if ((BT_EMB_PRESENT) || (BLE_EMB_PRESENT))
    // Enable COMMON IRQ
   //intcntl_enable_irq(INTCNTL_COMMON_IP);
    #endif // ((BT_EMB_PRESENT) || (BLE_EMB_PRESENT))

    #if  (BLE_EMB_PRESENT && BLE_ISO_PRESENT)
    // Enable Bluetooth timestamp IRQ
    //intcntl_enable_irq(INTCNTL_BTS);
    #endif //  (BLE_EMB_PRESENT && BLE_ISO_PRESENT)

    #if PLF_GAIA && !defined(RP_HWSIM_BYPASS)
    // Assert if incompatible HostIO version
    ASSERT_INFO((((get_hostio_version()>>24)&0xFF) >= BRAM_HOSTIO_VER_MAJOR),
            ((get_hostio_version()>>24)&0xFF), BRAM_HOSTIO_VER_MAJOR);
    #endif //PLF_GAIA && !defined(RP_HWSIM_BYPASS)
}


extern uint32_t ble_deepsleep_statusget(void);

/**
 ****************************************************************************************
 * @brief Main loop function.
 ****************************************************************************************
 */
RAM_FUNCTION void main_loop(void)
{

    /*
     ************************************************************************************
     * Main loop
     ************************************************************************************
     */
     // start interrupt handling
    GLOBAL_INT_START();

    for (;;)
    {
        // schedule all pending events
        rwip_schedule();
        RWIP_TASK_SCHEDULE();

        // Add delay for test idle task haimin.zhang @ 20220427
        vTaskDelay(10);
    }
}

const struct rwip_eif_api* rwip_eif_get(uint8_t idx)
{

    const struct rwip_eif_api* ret = NULL;
/*    switch(idx)
    {
        case 0:
        {
            ret = &uart_api;
        }
        break;
        default:
        {
            ASSERT_ERR(0);
        }
        break;
    } */
    return ret;
}
#if 0

void app_uart_config(uint8_t port)
{
    /* uart config structrue init */
    ms_uart_dev_t uart_init_struct = {0};

    ms_uart_struct_init(&uart_init_struct);
    if(UART1_INDEX == port)
    {
        uart_init_struct.config.baud_rate = UART_BAUDRATE_115200;
    }

    uart_init_struct.port = port;

    /* initialize uart */
    ms_uart_init(&uart_init_struct);
}

void UART0_atiny_log_init(void)
{
   app_uart_clk_config();
   ms_pinmux_config(PAD14,PIN14_UART0_TXD);
	ms_pinmux_config(PAD15,PIN15_UART0_RXD);
	app_uart_config(UART0_INDEX);
	atiny_set_log_level(LOG_DEBUG);
	printf_uart_register(UART0_INDEX);
}

void UART1_atiny_log_init(void)
{
       app_uart_clk_config();
       ms_pinmux_config(PAD6,PIN06_UART1_TXD);
	ms_pinmux_config(PAD7,PIN07_UART1_RXD);
	app_uart_config(UART1_INDEX);
	atiny_set_log_level(LOG_DEBUG);
	printf_uart_register(UART1_INDEX);
}

void app_uart_clk_config(void)
{
    set_clk_enable(UART0_CLK, 1);
    set_ctrl_clk_enable(UART0_CTRL_CLK, 1);

    set_clk_enable(UART1_CLK, 1);
    set_ctrl_clk_enable(UART1_CTRL_CLK, 1);	
}
#endif


void sys_init(void)
{
    mdm_init(BLE_1M_UNCODED);
		
	 	//enable BLE event FIFO interrupt
    //NVIC_EnableIRQ(BLE_IRQn);
    ECLIC_EnableIRQ(BLE_IRQn);
    ble_clk_init();
    ble_2chip_init();
	 MS_LOGI(LOG_INFO,"ble_2chip_init\r\n");
    //ms_dmac_init();  pyxue temp
}

/*
 * MAIN FUNCTION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Main function.
 *
 * This function is called right after the booting process has completed.
 *
 * @return status   exit status
 ****************************************************************************************
 */

#ifdef CFG_ROM_VT
void my_patch(void);
#endif // CFG_ROM_VT

int blemain(void)
{
    // Branch to main loop
    main_loop();
    return 0;
}





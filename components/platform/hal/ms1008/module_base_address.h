//--------------------------------------------------------------------
// Copyright (c) 2021 by MooreSilicon.
// All rights reserved.
// MooreSilicon Confidential Proprietary.
//--------------------------------------------------------------------
// Project name: Bt_soc
//    File name: module_base_address.h
//       Author: naijc
//        Dates: 2021-07-12 15:17:12
//      Version: V1.0
//-------------------------------------------------------------------
//      Purpose:  
//
//-------------------------------------------------------------------
#ifndef MODULE_BASE_ADDRESS__H
#define MODULE_BASE_ADDRESS__H


//#define SRAM0_BASE_ADDR             0x10000000
//#define SRAM1_BASE_ADDR             0x10008000
//#define SRAM2_BASE_ADDR             0x10010000
//#define EM_RAM_BASE_ADDR            0x10018000


#define RWIP_RAM_BASE_ADDR              0x0FFE0000 
#define RWIP_RAM_BASE_ADDR_END          0x0FFE2000  //8K LENGTH = 0x2000

#define RAM_BASE_ADDR              0x0FFE2000
#define RAM_ADDR_END               0x10018000    //216K LENGTH = 0x36000

#define RET_RAM_BASE_ADDR           0x10030000
#define RET_RAM_BASE_ADDR_END       0x10031000 // 4K  LENGTH = 0x800

#define FLASH_BASE_ADDR             0x20000000
#define FLASH_BASE_ADDR_END         0x20080000  //512K LENGTH = 0x80000

#define ROM_BASE_ADDR               0x00001000
#define ROM_BASE_ADDR_END           0x00020000  //124K  LENGTH = 0x1F000

#define DMAC_BASE_ADDR              0x21000000
#define BLE_BASE_ADDR               0x21001000

#define UART0_BASE_ADDR             0x40000000
#define UART1_BASE_ADDR             0x40001000
#define UART2_BASE_ADDR             0x40002000
#define SPI0_BASE_ADDR              0x40003000
#define I2C0_BASE_ADDR              0x40004000
#define RTC_BASE_ADDR               0x40005000
#define TRNG_BASE_ADDR              0x40006000
#define IR_BASE_ADDR                0x40007000
#define WDT_BASE_ADDR               0x40008000
#define I2S0_BASE_ADDR              0x40009000
#define I2S1_BASE_ADDR              0x4000a000
#define TIMER_BASE_ADDR             0x40011000
#define QSPI_BASE_ADDR              0x40012000
#define AUDIO_ADC_BASE_ADDR         0x40013000
#define KEY_SCAN_BASE_ADDR          0x40015000
#define SPI1_BASE_ADDR              0x40017000
#define I2C1_BASE_ADDR              0x40019000
#define AUX_ADC_BASE_ADDR           0x4001a000
#define PWM_BASE_ADDR               0x40020000
#define GPIO_BASE_ADDR              0x40030000
#define SYS_CTRL_BASE_ADDR          0x40110000

#define AUD_WRAP_BASE_ADDR          AUDIO_ADC_BASE_ADDR

#define PERI_BASE_ADDR              0x21000000
#define PERI_END_ADDR               0x41000000


#endif

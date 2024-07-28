//--------------------------------------------------------------------
// Copyright (c) 2021 by MooreSilicon.
// All rights reserved.
// MooreSilicon Confidential Proprietary.
//--------------------------------------------------------------------
// Project name: Bt_soc
//    File name: ble/src/ble.c
//       Author: liuyd
//        Dates: 2021-10-26 15:51:50
//      Version: V1.0
//-------------------------------------------------------------------
//      Purpose:  
//
//-------------------------------------------------------------------
#ifndef BLE__C
#define BLE__C

#include "reg_ipcore.h"
#include "ble.h"

#include "ms_clock_hal.h"
#include "ms_ble_hal.h"
#include "log.h"
#include "ms_sys_ctrl_regs.h"
#include "ms_interrupt.h"
#include "_reg_em_ble_cs.h"
#include "reg_ipcore.h"


//added by jiande for BLE clk_sel
void set_ble_clk_sel(UINT32 clk_sel)
{
/*

      reg_SYS_CTRL_BLE_CTRL   ble_ctrl; 
	ble_ctrl.reg_value = inl(SYS_CTRL_BASE_ADDR + SYS_CTRL_BLE_CTRL_OFFSET); 
	ble_ctrl.BLE_CLK_SEL = clk_sel;
	outl(SYS_CTRL_BASE_ADDR + SYS_CTRL_BLE_CTRL_OFFSET, ble_ctrl.reg_value); */

    ms_ctrl_ble_clk_selec_hal(clk_sel);

}


void ble_clk_init(void)
{
    //set_high_freq_clk_source(XCLK_SRC_PLL);
//    set_high_freq_clk_source(HF_CLK_SRC_PLL);   pyxue already set in  env_init.c system_clock_init()
    //set_ahb_clk_source(HCLK_SRC_PLL_OR_OSC16M); // ahb_src_clk from PLL 96M
//     set_ahb_clk_source(HCLK_SRC_HI_FREQ); // ahb_src_clk from PLL 96M
    //BLE BB hgclk and system clk = 96M / 4 = 24M
    
 //   set_clk_div(AHB_DIV, 3);  // pyxue ??
    //get 24MHz / 2 = 12MHz for mdm TX clk and ble clk
    //set_clk_div(MDM1_DIV, 1);  //pyxue
    MS_CLOCK_HAL_CLK_ENABLE_BLE();    
    ms_clock_hal_set_ble_div(1);
    ms_clock_hal_peripre_clk_div_toggle();
	ms_clock_hal_MDM0_clk_div_toggle();
    //ms_clock_hal_BLE_clk_div_toggle();
    //MS_LOGI(MS_DRIVER, "ble_clk_init  sys clock DIV_TOG %x\r\n", SYS_CTRL->DIV_TOG);
	//MS_LOGI(MS_DRIVER, "SYS_CTRL->CLK_SEL %x\r\n", SYS_CTRL->CLK_SEL); 	
    //MS_LOGI(MS_DRIVER, "SYS_CTRL->CLK_EN0 %x\r\n", SYS_CTRL->CLK_EN0); 		
    //MS_LOGI(MS_DRIVER, "SYS_CTRL->CLK_DIV3 %x\r\n", SYS_CTRL->CLK_DIV3); 	
    //CAUTION: remember to modified the master_clk_freq accordingly in
    //tb_pkg.sv of BLE UT bench when this value is modified.
    
    set_ble_clk_sel(24);
	ms_ctrl_set_blemdm_clock_bypass_hal(0);
    while (!ms_clock_hal_pll_is_locked()); 
}

void set_rf_mode(void)
{
  /*     reg_SYS_CTRL_LP_KEEP	     lp_keep;
       enum mode {deep_sleep = 1, sleep = 2, standby = 4, TX = 8, RX = 16} rf_mode_value = standby;

	// enable rf, added by liaozf
	lp_keep.reg_value = inl(SYS_CTRL_BASE_ADDR + SYS_CTRL_LP_KEEP_OFFSET);
	lp_keep.RF_MODE_SW = rf_mode_value;
	lp_keep.RF_MODE_HW_VLD = 0;
	outl(SYS_CTRL_BASE_ADDR + SYS_CTRL_LP_KEEP_OFFSET, lp_keep.reg_value); */

  //  uint32_t intvalue;
	enum mode {deep_sleep = 1, sleep = 2, standby = 4, TX = 8, RX = 16} rf_mode_value = standby;

    ms_ctrl_ble_rf_hw_invalid_hal();   // clear HW VLD
    ms_ctrl_ble_rf_swcontrol_parameter_hal(rf_mode_value);

    MS_LOGI(MS_DRIVER, "set_rf_mode SYS_CTRL->LP_KEEP %x\r\n", SYS_CTRL->LP_KEEP); 	
			
}
    
void ble_init(void){
    set_rf_mode();
    //ble_clk_init();
    //added by jiangde
    ble_rf_init();
    //set CLKN to 0 to sync with UT status for FPGA
   // set_ble_CLKN(0);
}

void ble_ExtRC_init2(void);

void ble_init2(void)
	{

   // ble_ExtRC_init2();

}


void set_ble_CLKN(UINT32 clkn)
{
    reg_BLE_SLOTCLK slotclk;

    slotclk.reg_value = inl(IP_RWDMCNTL_ADDR + IP_SLOTCLK_OFFSET);
    slotclk.SCLK = clkn;
    slotclk.CLKN_UPD = 1;
    outl(IP_RWDMCNTL_ADDR + IP_SLOTCLK_OFFSET, slotclk.reg_value); 
}


void ble_2chip_init(void)
{
    reg_BLE_TIMGENCNTL        timegencntl;
    reg_BLE_RADIOPWRUPDN0     radiopwrupdn0;
    reg_BLE_RADIOTXRXTIM0     radiotxrxtim0;
    reg_BLE_RADIOPWRUPDN1     radiopwrupdn1;
    reg_BLE_RADIOTXRXTIM1     radiotxrxtim1;

    ble_init();
    timegencntl.reg_value = inl(IP_RWDMCNTL_ADDR + BLE_TIMGENCNTL_OFFSET);
    //delay prefetch time to delay the TX packet sending time
    //unit: 0.5us
    timegencntl.PREFETCH_TIME += 256;
    outl(IP_RWDMCNTL_ADDR + BLE_TIMGENCNTL_OFFSET, timegencntl.reg_value);

    ////////////////////////////////////////////////////////////////
    //                         BLE 1M
    ////////////////////////////////////////////////////////////////
    //power up delay for TX / RX
    radiopwrupdn0.reg_value = inl(IP_RWDMCNTL_ADDR + BLE_RADIOPWRUPDN0_OFFSET);
    //added for DTM
    //radiopwrupdn0.SYNC_POSITION0 = 26;

    radiopwrupdn0.TXPWRUP0 = 0x70;
    radiopwrupdn0.RXPWRUP0 = 0x70;
    outl(IP_RWDMCNTL_ADDR + BLE_RADIOPWRUPDN0_OFFSET, radiopwrupdn0.reg_value);

    //TX / RX path delay is approximately equal to modem TX delay
    radiotxrxtim0.reg_value = inl(IP_RWDMCNTL_ADDR + BLE_RADIOTXRXTIM0_OFFSET);
    radiotxrxtim0.TXPATHDLY0 = 0x3;
    radiotxrxtim0.RXPATHDLY0 = 0x1d;
    //added for DTM
    radiotxrxtim0.RFRXTMDA0 = 29;
    outl(IP_RWDMCNTL_ADDR + BLE_RADIOTXRXTIM0_OFFSET, radiotxrxtim0.reg_value);
    
    ////////////////////////////////////////////////////////////////
    //                         BLE 2M
    ////////////////////////////////////////////////////////////////
    //power up delay for TX / RX
    radiopwrupdn1.reg_value = inl(IP_RWDMCNTL_ADDR + BLE_RADIOPWRUPDN1_OFFSET);
    //added for DTM
    //radiopwrupdn0.SYNC_POSITION0 = 26;

    radiopwrupdn1.TXPWRUP1 = 0x70;
    radiopwrupdn1.RXPWRUP1 = 0x70;
    outl(IP_RWDMCNTL_ADDR + BLE_RADIOPWRUPDN1_OFFSET, radiopwrupdn1.reg_value);

    //TX / RX path delay is approximately equal to modem TX delay
    radiotxrxtim1.reg_value = inl(IP_RWDMCNTL_ADDR + BLE_RADIOTXRXTIM1_OFFSET);
    radiotxrxtim1.TXPATHDLY1 = 0x3;
    radiotxrxtim1.RXPATHDLY1 = 18;
    //added for DTM
    radiotxrxtim1.RFRXTMDA1 = 29;
    outl(IP_RWDMCNTL_ADDR + BLE_RADIOTXRXTIM1_OFFSET, radiotxrxtim1.reg_value);
}


void ble_deepsleep_clear(void)
{
    ip_deepslcntl_set(0);
}

uint32_t ble_deepsleep_statusget(void)
{
    uint32_t status;

	status= ip_deepslcntl_get();
	
    return status;
}


// field definitions
#define BLE_EXTWKUPDSB_BIT            ((uint32_t)0x80000000)
#define BLE_EXTWKUPDSB_POS            31
#define BLE_DEEP_SLEEP_STAT_BIT       ((uint32_t)0x00008000)
#define BLE_DEEP_SLEEP_STAT_POS       15
#define BLE_DEEP_SLEEP_CORR_EN_BIT    ((uint32_t)0x00000008)
#define BLE_DEEP_SLEEP_CORR_EN_POS    3
#define BLE_DEEP_SLEEP_ON_BIT         ((uint32_t)0x00000004)
#define BLE_DEEP_SLEEP_ON_POS         2
#define BLE_RADIO_SLEEP_EN_BIT        ((uint32_t)0x00000002)
#define BLE_RADIO_SLEEP_EN_POS        1
#define BLE_OSC_SLEEP_EN_BIT          ((uint32_t)0x00000001)
#define BLE_OSC_SLEEP_EN_POS          0


void ble_sleep_enter(void) 
{
  // configure the ble sleep time, to be supported pyxue 3/16
  
    uint32_t twosc = 4;
    uint32_t twrm = 2;

    ip_deepslwkup_set(32000*10);
    ip_enbpreset_set(twrm + (twosc << 10) + ((twosc + 1) << 21));
    ip_deepslcntl_set(BLE_DEEP_SLEEP_ON_BIT |    // RW BLE Core sleep
                      BLE_RADIO_SLEEP_EN_BIT |   // Radio sleep
                      BLE_OSC_SLEEP_EN_BIT);  
} 

#endif

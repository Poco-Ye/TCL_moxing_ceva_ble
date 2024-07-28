//--------------------------------------------------------------------
// Copyright (c) 2021 by MooreSilicon.
// All rights reserved.
// MooreSilicon Confidential Proprietary.
//--------------------------------------------------------------------
// Project name: Bt_soc
//    File name: mdm.c
//       Author: jiangd
//        Dates: 2021-11-30 15:51:50
//      Version: V1.0
//-------------------------------------------------------------------
//      Purpose:  
//
//-------------------------------------------------------------------
#ifndef MDM__C
#define MDM__C

//#include "ms_sys_ctrl.h"
#include "ms_clock_hal.h"

#include "mdm.h"
#include "mdm_reg.h"
#include "ble.h"
#include "reg_modemhp.h"
__REG_MODEMHP_H_

void mdm_clk_init(void)
{
    //fix RX clk to 96MHz / 2 = 48MHz
  //  set_clk_div(MDM0_DIV, 1);  pyxue
    MS_CLOCK_HAL_CLK_ENABLE_MDM();
    ms_clock_hal_set_mdm0_div(1);
    ms_clock_hal_MDM0_clk_div_toggle();
}

void mdm_settings_init(ble_packet_rate rate)
{
    reg_MDM_CNTL    mdm_cntl;

    mdm_cntl.reg_value = inl(REG_MODEMHP_BASE_ADDR + MDM_CNTL_MDMHP_MDM_CNTL_OFFSET);
    //rx_valid is a 12MHz pulse
    mdm_cntl.RX_VALID_MODE = 0;
    if (rate == BLE_1M_UNCODED) {
        //delay in us to enable modem TX path after bb_tx_en asserted
        mdm_cntl.TX_STARTUPDEL = 0x70;
        //delay in us to enable modem RX path after bb_rx_en asserted
        mdm_cntl.RX_STARTUPDEL = 0x70;
		  mdm_cntl.FMTXEN = 0;
    } else if (rate == BLE_2M_UNCODED) {
        //delay in us to enable modem TX path after bb_tx_en asserted
        mdm_cntl.TX_STARTUPDEL = 0x68;
        //delay in us to enable modem RX path after bb_rx_en asserted
        mdm_cntl.RX_STARTUPDEL = 0x68;
		  mdm_cntl.FMTXEN = 0;
    }
    outl(REG_MODEMHP_BASE_ADDR + MDM_CNTL_MDMHP_MDM_CNTL_OFFSET, mdm_cntl.reg_value);
}

void mdm_init(uint32_t  rate)
{
    mdm_clk_init();
    mdm_settings_init(rate);
}

#endif


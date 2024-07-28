//--------------------------------------------------------------------
// Copyright (c) 2021 by MooreSilicon.
// All rights reserved.
// MooreSilicon Confidential Proprietary.
//--------------------------------------------------------------------
// Project name: Bt_soc
//    File name: ble_rf.c
//       Author: jiangd
//        Dates: 2021-11-03 15:51:50
//      Version: V1.0
//-------------------------------------------------------------------
//      Purpose:  
//
//-------------------------------------------------------------------
#ifndef BLE_RF__C
#define BLE_RF__C

#include "ble.h"
#include "_reg_em_ble_cs.h"
#include "reg_ipcore.h"

void ble_rf_init(void)
{
    #ifdef RW_BLE_EXTRC_INST
    ble_ExtRC_init();
    #endif
}


void hop_table(void)
{
    //the following code are converted from ExtRC
    //LE_WR_EM 00000100 02010025
    outl(REG_EM_BLE_CS_BASE_ADDR + 0x00000100, 0x03020100);
    //LE_WR_EM 00000104 06050403
    outl(REG_EM_BLE_CS_BASE_ADDR + 0x00000104, 0x07060504);
    //LE_WR_EM 00000108 0A090807
    outl(REG_EM_BLE_CS_BASE_ADDR + 0x00000108, 0x0B0A0908);
    //LE_WR_EM 0000010C 0D0C0B26
    outl(REG_EM_BLE_CS_BASE_ADDR + 0x0000010C, 0x0F0E0D0C);
    //LE_WR_EM 00000110 11100F0E
    outl(REG_EM_BLE_CS_BASE_ADDR + 0x00000110, 0x13121110);
    //LE_WR_EM 00000114 15141312
    outl(REG_EM_BLE_CS_BASE_ADDR + 0x00000114, 0x17161514);
    //LE_WR_EM 00000118 19181716
    outl(REG_EM_BLE_CS_BASE_ADDR + 0x00000118, 0x1B1A1918);
    //LE_WR_EM 0000011C 1D1C1B1A
    outl(REG_EM_BLE_CS_BASE_ADDR + 0x0000011C, 0x1F1E1D1C);
    //LE_WR_EM 00000120 21201F1E
    outl(REG_EM_BLE_CS_BASE_ADDR + 0x00000120, 0x23222120);
    //LE_WR_EM 00000124 27242322
    outl(REG_EM_BLE_CS_BASE_ADDR + 0x00000124, 0x27262524);

}


void ble_ExtRC_init(void)
{
 
#if 1
    //the following code are converted from ExtRC
    //LE_WR_EM 00000100 02010025
    outl(REG_EM_BLE_CS_BASE_ADDR + 0x00000100, 0x03020100);
    //LE_WR_EM 00000104 06050403
    outl(REG_EM_BLE_CS_BASE_ADDR + 0x00000104, 0x07060504);
    //LE_WR_EM 00000108 0A090807
    outl(REG_EM_BLE_CS_BASE_ADDR + 0x00000108, 0x0B0A0908);
    //LE_WR_EM 0000010C 0D0C0B26
    outl(REG_EM_BLE_CS_BASE_ADDR + 0x0000010C, 0x0F0E0D0C);
    //LE_WR_EM 00000110 11100F0E
    outl(REG_EM_BLE_CS_BASE_ADDR + 0x00000110, 0x13121110);
    //LE_WR_EM 00000114 15141312
    outl(REG_EM_BLE_CS_BASE_ADDR + 0x00000114, 0x17161514);
    //LE_WR_EM 00000118 19181716
    outl(REG_EM_BLE_CS_BASE_ADDR + 0x00000118, 0x1B1A1918);
    //LE_WR_EM 0000011C 1D1C1B1A
    outl(REG_EM_BLE_CS_BASE_ADDR + 0x0000011C, 0x1F1E1D1C);
    //LE_WR_EM 00000120 21201F1E
    outl(REG_EM_BLE_CS_BASE_ADDR + 0x00000120, 0x23222120);
    //LE_WR_EM 00000124 27242322
    outl(REG_EM_BLE_CS_BASE_ADDR + 0x00000124, 0x27262524);
#else
#endif

    //#--------------------------------------------------
    //# ExtRC RF Settings
    //#--------------------------------------------------
    //LE_WR_RG 00000070 00000000
    outl(IP_RWDMCNTL_ADDR + BLE_RADIOCNTL0_OFFSET, 0x00000000);

    //`ifdef RW_BLE_CORRELATOR_INST
    //LE_WR_RG 00000074 00001020
    //`endif
    //`ifndef RW_BLE_CORRELATOR_INST
    //LE_WR_RG 00000074 00005020
    //`endif
    #ifdef RW_BLE_CORRELATOR_INST
    outl(IP_RWDMCNTL_ADDR + BLE_RADIOCNTL1_OFFSET, 0x00001020);
    #else
    outl(IP_RWDMCNTL_ADDR + BLE_RADIOCNTL1_OFFSET, 0x00005020);
    #endif

    //# Set phymsk to support LR + 2Mbps
    //LE_WR_RG 00000078 C8C00040
    outl(IP_RWDMCNTL_ADDR + BLE_RADIOCNTL2_OFFSET, 0xC8C00040);
    //LE_WR_RG 0000007C E400E400
    outl(IP_RWDMCNTL_ADDR + BLE_RADIOCNTL3_OFFSET, 0xE400E400);

    //#### 1 Mbps ####
    //LE_WR_RG 00000080 00300731
    outl(IP_RWDMCNTL_ADDR + BLE_RADIOPWRUPDN0_OFFSET, 0x00300731);
    //LE_WR_RG 00000090 00040201
    outl(IP_RWDMCNTL_ADDR + BLE_RADIOTXRXTIM0_OFFSET, 0x00040201);

    //#### 2 Mbps ####
    //LE_WR_RG 00000084 00310731
    outl(IP_RWDMCNTL_ADDR + BLE_RADIOPWRUPDN1_OFFSET, 0x00310731);
    //LE_WR_RG 00000094 00010101
    outl(IP_RWDMCNTL_ADDR + BLE_RADIOTXRXTIM1_OFFSET, 0x00010101);

    //`ifdef RW_BLE_LONG_RANGE_INST
    #ifdef RW_BLE_LONG_RANGE_INST
    //#### 125 Kbps ####
    //#  Bits           Field Name   Reset Value
    //# -----   ------------------   -----------
    //# 31:24       SYNC_POSITION2   0x0
    //# 23:16             RXPWRUP2   0x0
    //# 14:08             TXPWRDN2   0x0
    //# 07:00             TXPWRUP2   0x0
    //
    //LE_WR_RG 00000088 00310731
    outl(IP_RWDMCNTL_ADDR + BLE_RADIOPWRUPDN2_OFFSET, 0x00310731);
    //
    //#  Bits           Field Name   Reset Value
    //# -----   ------------------   -----------
    //# 31:24      RXFLUSHPATHDLY2   0x0
    //# 22:16            RFRXTMDA2   0x0
    //# 15:08           RXPATHDLY2   0x0
    //# 06:00           TXPATHDLY2   0x0
    //
    //LE_WR_RG 00000098 0F020201
    outl(IP_RWDMCNTL_ADDR + BLE_RADIOTXRXTIM2_OFFSET, 0x0F020201);
    //
    //#### 500 Kbps ####
    //#  Bits           Field Name   Reset Value
    //# -----   ------------------   -----------
    //# 14:08             TXPWRDN3   0x0
    //# 07:00             TXPWRUP3   0x0
    //
    //LE_WR_RG 0000008C 00310731
    outl(IP_RWDMCNTL_ADDR + BLE_RADIOPWRUPDN3_OFFSET, 0x00310731);
    //
    //#  Bits           Field Name   Reset Value
    //# -----   ------------------   -----------
    //# 31:24      RXFLUSHPATHDLY3   0x0
    //# 06:00           TXPATHDLY3   0x0
    //
    //LE_WR_RG 0000009C 0D000001
    outl(IP_RWDMCNTL_ADDR + BLE_RADIOTXRXTIM3_OFFSET, 0x0D000001);
    //`endif
    #endif

    //`ifdef RW_BLE_LONG_RANGE_RX_INST
    #ifdef RW_BLE_LONG_RANGE_RX_INST
    //#  Bits           Field Name   Reset Value
    //# -----   ------------------   -----------
    //# 31:24      RXFLUSHPATHDLY2   0x0   0x03 = 3
    //# 23:16            RFRXTMDA2   0x0   0x00 = 0
    //# 15:08           RXPATHDLY2   0x0   0x03 = 3
    //# 06:00           TXPATHDLY2   0x0   0x06 = 6
    //
    //LE_WR_RG 00000098 0F454501
    outl(IP_RWDMCNTL_ADDR + BLE_RADIOTXRXTIM2_OFFSET, 0x0F454501);
    //
    //#  Bits           Field Name   Reset Value
    //# -----   ------------------   -----------
    //# 31:24      RXFLUSHPATHDLY3   0x0   0x03 = 3
    //# 22:16            RFRXTMDA3   0x0   0x00 = 0
    //# 06:00           TXPATHDLY3   0x0   0x06 = 06
    //
    //LE_WR_RG 0000009C 0D300006
    outl(IP_RWDMCNTL_ADDR + BLE_RADIOTXRXTIM3_OFFSET, 0x0D300006);
    //`endif
    #endif

    //#----------------------------------------------------
    //# Init Prefetch Time and Prefetch Abort Time APFM
    //#----------------------------------------------------
    //# BLE_PREFETCHABORT_TIME_US 160 us
    //# BLE_PREFETCH_TIME_US 120 us
    //
    //LE_WR_RG 000000E0 014000F0
    outl(IP_RWDMCNTL_ADDR + BLE_TIMGENCNTL_OFFSET, 0x014000F0);

}

#endif

//--------------------------------------------------------------------
// Copyright (c) 2021 by MooreSilicon.
// All rights reserved.
// MooreSilicon Confidential Proprietary.
//--------------------------------------------------------------------
// Project name: Bt_soc
//    File name: ceva_ble_reg.h
//       Author: tuzf
//        Dates: 2021-11-02 11:43:31
//      Version: V1.0
//-------------------------------------------------------------------
//      Purpose: ceva_ble_reg definition and structure
//
//-------------------------------------------------------------------

#ifndef BLE_REG__H
#define BLE_REG__H

    #define   BLE_CNTL_OFFSET               0x000
    typedef union {
        struct {
            UINT32 RXWINSZDEF:                  4;
            UINT32 reserved_0:                  4;
            UINT32 RWBLE_EN:                    1;
            UINT32 ADVERTFILT_EN:               1;
            UINT32 ANONYMOUS_ADVERT_FILT_EN:    1;
            UINT32 RXCTEERR_RETX_EN:            1;
            UINT32 HOP_REMAP_DSB:               1;
            UINT32 CRC_DSB:                     1;
            UINT32 WHIT_DSB:                    1;
            UINT32 LRFEC_DSB:                   1;
            UINT32 LRPMAP_DSB:                  1;
            UINT32 CRYPT_DSB:                   1;
            UINT32 NESN_DSB:                    1;
            UINT32 SN_DSB:                      1;
            UINT32 MD_DSB:                      1;
            UINT32 NPI_DSB:                     1;
            UINT32 CIE_DSB:                     1;
            UINT32 reserved_1:                  1;
            UINT32 SCAN_ABORT:                  1;
            UINT32 ADVERT_ABORT:                1;
            UINT32 RFTEST_ABORT:                1;
            UINT32 SWINT_REQ:                   1;
            UINT32 RADIOCNTL_SOFT_RST:          1;
            UINT32 REG_SOFT_RST:                1;
            UINT32 MASTER_TGSOFT_RST:           1;
            UINT32 MASTER_SOFT_RST:             1;
        };
        UINT32 reg_value;
    } reg_BLE_CNTL;

    #define   BLE_VERSION_OFFSET                0x004
    typedef union {
        struct {
            UINT32 BUILD:                       8;
            UINT32 UPG:                         8;
            UINT32 REL:                         8;
            UINT32 TYP:                         8;
        };
        UINT32 reg_value;
    } reg_BLE_VERSION;

    #define   BLE_CONF_OFFSET              0x008
    typedef union {
        struct {
            UINT32 ADDR_WIDTH:                  5;
            UINT32 reserved_0:                  1;
            UINT32 BUSTYPE:                     1;
            UINT32 INTMODE:                     1;
            UINT32 CLK_SEL:                     6;
            UINT32 DECIPHER:                    1;
            UINT32 USEDBG:                      1;
            UINT32 RFIF:                        7;
            UINT32 reserved_1:                  1;
            UINT32 USEISO:                      1;
            UINT32 reserved_2:                  1;
            UINT32 USETXLR:                     1;
            UINT32 USERXLR:                     1;
            UINT32 CORRELATOR:                  1;
            UINT32 WLANCOEX:                    1;
            UINT32 DF:                          1;
            UINT32 DMMODE:                      1;
        };
        UINT32 reg_value;
    } reg_BLE_CONF;

    #define   BLE_INTCNTL0_OFFSET               0x00C
    typedef union {
        struct {
            UINT32 STARTEVTINTMSK:              1;
            UINT32 ENDEVTINTMSK:                1;
            UINT32 SKIPEVTINTMSK:               1;
            UINT32 TXINTMSK:                    1;
            UINT32 RXINTMSK:                    1;
            UINT32 ISOTXINTMSK:                 1;
            UINT32 ISORXINTMSK:                 1;
            UINT32 HOPINTMSK:                   1;
            UINT32 reserved_0:                  8;
            UINT32 ERRORINTMSK:                 1;
            UINT32 reserved_1:                  15;
        };
        UINT32 reg_value;
    } reg_BLE_INTCNTL0;

    #define   BLE_INTSTAT0_OFFSET               0x010
    typedef union {
        struct {
            UINT32 reserved_0:                  7;
            UINT32 HOPINTSTAT:                  1;
            UINT32 reserved_1:                  8;
            UINT32 ERRORINTSTAT:                1;
            UINT32 reserved_2:                  15;
        };
        UINT32 reg_value;
    } reg_BLE_INTSTAT0;
  
    #define   BLE_INTACK0_OFFSET                0x014
    typedef union {
        struct {
            UINT32 reserved_0:                  7;
            UINT32 HOPINTACK:                   1;
            UINT32 reserved_1:                  8;
            UINT32 ERRORINTACK:                 1;
            UINT32 reserved_2:                  15;
        };
        UINT32 reg_value;
    } reg_BLE_INTACK0;

    #define   BLE_INTCNTL1_OFFSET               0x018
    typedef union {
        struct {
            UINT32 CLKNINTMSK:                  1;
            UINT32 SLPINTMSK:                   1;
            UINT32 CRYPTINTMSK:                 1;
            UINT32 SWINTMSK:                    1;
            UINT32 FINETGTINTMSK:               1;
            UINT32 TIMESTAMPTGT1INTMSK:         1;
            UINT32 TIMESTAMPTGT2INTMSK:         1;
            UINT32 TIMESTAMPTGT3INTMSK:         1;
            UINT32 reserved_0:                  7;
            UINT32 FIFOINTMSK:                  1;
            UINT32 reserved_1:                  8;
            UINT32 CLKNINTSRVAL:                4;
            UINT32 CLKNINTSRMSK:                3;
            UINT32 reserved_2:                  1;
        };
        UINT32 reg_value;
    } reg_BLE_INTCNTL1;

    #define   BLE_INTSTAT1_OFFSET               0x01C
    typedef union {
        struct {
            UINT32 CLKNINTSTAT:                 1;
            UINT32 SLPINTSTAT:                  1;
            UINT32 CRYPTINTSTAT:                1;
            UINT32 SWINTSTAT:                   1;
            UINT32 FINETGTINTSTAT:              1;
            UINT32 TIMESTAMPTGT1INTSTAT:        1;
            UINT32 TIMESTAMPTGT2INTSTAT:        1;
            UINT32 TIMESTAMPTGT3INTSTAT:        1;
            UINT32 reserved_0:                  7;
            UINT32 FIFOINTSTAT:                 1;
            UINT32 reserved_1:                  16;
        };
        UINT32 reg_value;
    } reg_BLE_INTSTAT1;

    #define   BLE_INTACK1_OFFSET                0x020
    typedef union {
        struct {
            UINT32 CLKNINTACK:                  1;
            UINT32 SLPINTACK:                   1;
            UINT32 CRYPTINTACK:                 1;
            UINT32 SWINTACK:                    1;
            UINT32 FINETGTINTACK:               1;
            UINT32 TIMESTAMPTGT1INTACK:         1;
            UINT32 TIMESTAMPTGT2INTACK:         1;
            UINT32 TIMESTAMPTGT3INTACK:         1;
            UINT32 reserved_0:                  7;
            UINT32 FIFOINTACK:                  1;
            UINT32 reserved_1:                  16;
        };
        UINT32 reg_value;
    } reg_BLE_INTACK1;

    #define   BLE_ACTFIFOSTAT_OFFSET            0x024
    typedef union {
        struct {
            UINT32 STARTACTINTSTAT:             1;
            UINT32 ENDACTINTSTAT:               1;
            UINT32 SKIPACTINTSTAT:              1;
            UINT32 TXINTSTAT:                   1;
            UINT32 RXINTSTAT:                   1;
            UINT32 ISOTXINTSTAT:                1;
            UINT32 ISORXINTSTAT:                1;
            UINT32 reserved_0:                  8;
            UINT32 ACTFLAG:                     1;
            UINT32 reserved_1:                  8;
            UINT32 CURRENT_ET_IDX:              4;
            UINT32 SKIP_ET_IDX:                 4;
        };
        UINT32 reg_value;
    } reg_BLE_ACTFIFOSTAT;

    #define   BLE_CURRENTRXDESCPTR_OFFSET       0x028
    typedef union {
        struct {
            UINT32 CURRENTRXDESCPTR:            14;
            UINT32 reserved_0:                  18;
        };
        UINT32 reg_value;
    } reg_BLE_CURRENTRXDESCPTR;

    #define   BLE_ETPTR_OFFSET                  0x02C
    typedef union {
        struct {
            UINT32 ETPTR:                       14;
            UINT32 reserved_0:                  18;
        };
        UINT32 reg_value;
    } reg_BLE_ETPTR;

    #define   BLE_DEEPSLCNTL_OFFSET             0x030
    typedef union {
        struct {
            UINT32 OSC_SLEEP_EN:                1;
            UINT32 RADIO_SLEEP_EN:              1;
            UINT32 DEEP_SLEEP_ON:               1;
            UINT32 DEEP_SLEEP_CORR_EN:          1;
            UINT32 reserved_0:                  11;
            UINT32 DEEP_SLEEP_STAT:             1;
            UINT32 reserved_1:                  15;
            UINT32 EXTWKUPDSB:                  1;
        };
        UINT32 reg_value;
    } reg_BLE_DEEPSLCNTL;

    #define   BLE_DEEPSLWKUP_OFFSET             0x034
    typedef union {
        struct {
            UINT32 DEEPSLTIME:                  32;
        };
        UINT32 reg_value;
    } reg_BLE_DEEPSLWKUP;

    #define   BLE_DEEPSLSTAT_OFFSET             0x038
    typedef union {
        struct {
            UINT32 DEEPSLDUR:                   32;
        };
        UINT32 reg_value;
    } reg_BLE_DEEPSLSTAT;

    #define   BLE_ENBPRESET_OFFSET              0x03C
    typedef union {
        struct {
            UINT32 TWRM:                        10;
            UINT32 TWOSC:                       11;
            UINT32 TWEXT:                       11;
        };
        UINT32 reg_value;
    } reg_BLE_ENBPRESET;

    #define   BLE_FINECNTCORR_OFFSET            0x040
    typedef union {
        struct {
            UINT32 FINECNTCORR:                 10;
            UINT32 reserved_0:                  22;
        };
        UINT32 reg_value;
    } reg_BLE_FINECNTCORR;

    #define   BLE_CLKNCNTCORR_OFFSET            0x044
    typedef union {
        struct {
            UINT32 CLKNCNTCORR:                 28;
            UINT32 reserved_0:                  3;
            UINT32 ABS_DELTA:                   1;
        };
        UINT32 reg_value;
    } reg_BLE_CLKNCNTCORR;

    #define   BLE_DIAGCNTL_OFFSET               0x050
    typedef union {
        struct {
            UINT32 DIAG0:                       7;
            UINT32 DIAG0_EN:                    1;
            UINT32 DIAG1:                       7;
            UINT32 DIAG1_EN:                    1;
            UINT32 DIAG2:                       7;
            UINT32 DIAG2_EN:                    1;
            UINT32 DIAG3:                       7;
            UINT32 DIAG3_EN:                    1;
        };
        UINT32 reg_value;
    } reg_BLE_DIAGCNTL;

    #define   BLE_DIAGSTAT_OFFSET               0x054
    typedef union {
        struct {
            UINT32 DIAG0STAT:                   8;
            UINT32 DIAG1STAT:                   8;
            UINT32 DIAG2STAT:                   8;
            UINT32 DIAG3STAT:                   8;
        };
        UINT32 reg_value;
    } reg_BLE_DIAGSTAT;

    #define   BLE_DEBUGADDMAX_OFFSET            0x058
    typedef union {
        struct {
            UINT32 EM_ADDMAX:                   16;
            UINT32 REG_ADDMAX:                  16;
        };
        UINT32 reg_value;
    } reg_BLE_DEBUGADDMAX;

    #define   BLE_DEBUGADDMIN_OFFSET            0x05C
    typedef union {
        struct {
            UINT32 EM_ADDMIN:                   16;
            UINT32 REG_ADDMIN:                  16;
        };
        UINT32 reg_value;
    } reg_BLE_DEBUGADDMIN;

    #define   BLE_ERRORTYPESTAT_OFFSET          0x060
    typedef union {
        struct {
            UINT32 TXCRYPT_ERROR:               1;
            UINT32 RXCRYPT_ERROR:               1;
            UINT32 PKTCNTL_EMACC_ERROR:         1;
            UINT32 RADIO_EMACC_ERROR:           1;
            UINT32 ACT_SCHDL_ENTRY_ERROR:       1;
            UINT32 ACT_SCHDL_APFM_ERROR:        1;
            UINT32 EVT_CNTL_APFM_ERROR:         1;
            UINT32 LIST_ERROR:                  1;
            UINT32 IFS_UNDERRUN:                1;
            UINT32 ADV_UNDERRUN:                1;
            UINT32 LLCHMAP_ERROR:               1;
            UINT32 CSFORMAT_ERROR:              1;
            UINT32 TXDESC_EMPTY_ERROR:          1;
            UINT32 RXDESC_EMPTY_ERROR:          1;
            UINT32 TXDATA_PTR_ERROR:            1;
            UINT32 RXDATA_PTR_ERROR:            1;
            UINT32 RAL_ERROR:                   1;
            UINT32 RAL_UNDERRUN:                1;
            UINT32 TMAFS_ERROR:                 1;
            UINT32 TXAEHEADER_PTR_ERROR:        1;
            UINT32 PHY_ERROR:                   1;
            UINT32 FIFOINTOVF:                  1;
            UINT32 DFCNTL_EMACC_ERROR:          1;
            UINT32 FREQSEL_ERROR:               1;
            UINT32 reserved_0:                  8;
        };
        UINT32 reg_value;
    } reg_BLE_ERRORTYPESTAT;

    #define   BLE_SWPROFILING_OFFSET            0x064
    typedef union {
        struct {
            UINT32 SWPROFVAL:                   32;
        };
        UINT32 reg_value;
    } reg_BLE_SWPROFILING;


    #define   BLE_RADIOCNTL0_OFFSET             0x070
    typedef union {
        struct {
            UINT32 SPIGO:                       1;
            UINT32 SPICOMP:                     1;
            UINT32 reserved_0:                  2;
            UINT32 SPIFREQ:                     2;
            UINT32 reserved_1:                  1;
            UINT32 SPICFG:                      1;
            UINT32 reserved_2:                  8;
            UINT32 SPIPTR:                      14;
            UINT32 reserved_3:                  2;
        };
        UINT32 reg_value;
    } reg_BLE_RADIOCNTL0;

    #define   BLE_RADIOCNTL1_OFFSET             0x074
    typedef union {
        struct {
            UINT32 SUBVERSION:                  4;
            UINT32 XRFSEL:                      6;
            UINT32 reserved_0:                  2;
            UINT32 JEF_SELECT:                  1;
            UINT32 DPCORR_EN:                   1;
            UINT32 SYNC_PULSE_SRC:              1;
            UINT32 SYNC_PULSE_MODE:             1;
            UINT32 FORCEAGC_LENGTH:             12;
            UINT32 TXDNSL:                      1;
            UINT32 RXDNSL:                      1;
            UINT32 FORCEIQ:                     1;
            UINT32 FORCEAGC_EN:                 1;
        };
        UINT32 reg_value;
    } reg_BLE_RADIOCNTL1;

    #define   BLE_RADIOCNTL2_OFFSET             0x078
    typedef union {
        struct {
            UINT32 FREQTABLE_PTR:               14;
            UINT32 reserved_0:                  2;
            UINT32 SYNCERR:                     3;
            UINT32 reserved_1:                  1;
            UINT32 LRSYNCERR:                   2;
            UINT32 PHYMSK:                      2;
            UINT32 LRVTBFLUSH:                  5;
            UINT32 RXCITERMBYPASS:              1;
            UINT32 LRSYNCCOMPMODE:              2;
        };
        UINT32 reg_value;
    } reg_BLE_RADIOCNTL2;

    #define   BLE_RADIOCNTL3_OFFSET             0x07C
    typedef union {
        struct {
            UINT32 TXVALID_BEH:                 2;
            UINT32 reserved_0:                  6;
            UINT32 TXRATE0CFG:                  2;
            UINT32 TXRATE1CFG:                  2;
            UINT32 TXRATE2CFG:                  2;
            UINT32 TXRATE3CFG:                  2;
            UINT32 RXVALID_BEH:                 2;
            UINT32 RXSYNC_ROUTING:              1;
            UINT32 reserved_1:                  1;
            UINT32 GETRSSIDELAY:                3;
            UINT32 reserved_2:                  1;
            UINT32 RXRATE0CFG:                  2;
            UINT32 RXRATE1CFG:                  2;
            UINT32 RXRATE2CFG:                  2;
            UINT32 RXRATE3CFG:                  2;
        };
        UINT32 reg_value;
    } reg_BLE_RADIOCNTL3;

    #define   BLE_RADIOPWRUPDN0_OFFSET          0x080
    typedef union {
        struct {
            UINT32 TXPWRUP0:                    8;
            UINT32 TXPWRDN0:                    7;
            UINT32 reserved_0:                  1;
            UINT32 RXPWRUP0:                    8;
            UINT32 SYNC_POSITION0:              8;
        };
        UINT32 reg_value;
    } reg_BLE_RADIOPWRUPDN0;

    #define   BLE_RADIOPWRUPDN1_OFFSET          0x084
    typedef union {
        struct {
            UINT32 TXPWRUP1:                    8;
            UINT32 TXPWRDN1:                    7;
            UINT32 reserved_0:                  1;
            UINT32 RXPWRUP1:                    8;
            UINT32 SYNC_POSITION1:              8;
        };
        UINT32 reg_value;
    } reg_BLE_RADIOPWRUPDN1;

    #define   BLE_RADIOPWRUPDN2_OFFSET          0x088
    typedef union {
        struct {
            UINT32 TXPWRUP2:                    8;
            UINT32 TXPWRDN2:                    7;
            UINT32 reserved_0:                  1;
            UINT32 RXPWRUP2:                    8;
            UINT32 SYNC_POSITION2:              8;
        };
        UINT32 reg_value;
    } reg_BLE_RADIOPWRUPDN2;

    #define   BLE_RADIOPWRUPDN3_OFFSET          0x08C
    typedef union {
        struct {
            UINT32 TXPWRUP3:                    8;
            UINT32 TXPWRDN3:                    7;
            UINT32 reserved_0:                  17;
        };
        UINT32 reg_value;
    } reg_BLE_RADIOPWRUPDN3;

    #define   BLE_RADIOTXRXTIM0_OFFSET          0x090
    typedef union {
        struct {
            UINT32 TXPATHDLY0:                  7;
            UINT32 reserved_0:                  1;
            UINT32 RXPATHDLY0:                  7;
            UINT32 reserved_1:                  1;
            UINT32 RFRXTMDA0:                   7;
            UINT32 reserved_2:                  9;
        };
        UINT32 reg_value;
    } reg_BLE_RADIOTXRXTIM0;

    #define   BLE_RADIOTXRXTIM1_OFFSET          0x094
    typedef union {
        struct {
            UINT32 TXPATHDLY1:                  7;
            UINT32 reserved_0:                  1;
            UINT32 RXPATHDLY1:                  7;
            UINT32 reserved_1:                  1;
            UINT32 RFRXTMDA1:                   7;
            UINT32 reserved_2:                  9;
        };
        UINT32 reg_value;
    } reg_BLE_RADIOTXRXTIM1;

    #define   BLE_RADIOTXRXTIM2_OFFSET          0x098
    typedef union {
        struct {
            UINT32 TXPATHDLY2:                  7;
            UINT32 reserved_0:                  1;
            UINT32 RXPATHDLY2:                  8;
            UINT32 RFRXTMDA2:                   8;
            UINT32 RXFLUSHPATHDLY2:             8;
        };
        UINT32 reg_value;
    } reg_BLE_RADIOTXRXTIM2;

    #define   BLE_RADIOTXRXTIM3_OFFSET          0x09C
    typedef union {
        struct {
            UINT32 TXPATHDLY3:                  7;
            UINT32 reserved_0:                  9;
            UINT32 RFRXTMDA3:                   7;
            UINT32 reserved_1:                  1;
            UINT32 RXFLUSHPATHDLY3:             8;
        };
        UINT32 reg_value;
    } reg_BLE_RADIOTXRXTIM3;

    #define   BLE_SPIPTRCNTL0_OFFSET            0x0A0
    typedef union {
        struct {
            UINT32 TXONPTR:                     14;
            UINT32 reserved_0:                  2;
            UINT32 TXOFFPTR:                    14;
            UINT32 reserved_1:                  2;
        };
        UINT32 reg_value;
    } reg_BLE_SPIPTRCNTL0;

    #define   BLE_SPIPTRCNTL1_OFFSET            0x0A4
    typedef union {
        struct {
            UINT32 RXONPTR:                     14;
            UINT32 reserved_0:                  2;
            UINT32 RXOFFPTR:                    14;
            UINT32 reserved_1:                  2;
        };
        UINT32 reg_value;
    } reg_BLE_SPIPTRCNTL1;

    #define   BLE_SPIPTRCNTL2_OFFSET            0x0A8
    typedef union {
        struct {
            UINT32 RSSIPTR:                     14;
            UINT32 reserved_0:                  2;
            UINT32 RXLENGTHPTR:                 14;
            UINT32 reserved_1:                  2;
        };
        UINT32 reg_value;
    } reg_BLE_SPIPTRCNTL2;

    #define   BLE_SPIPTRCNTL3_OFFSET            0x0AC
    typedef union {
        struct {
            UINT32 RXPKTTYPPTR:                 14;
            UINT32 reserved_0:                  2;
            UINT32 CTESAMPPTR:                  14;
            UINT32 reserved_1:                  2;
        };
        UINT32 reg_value;
    } reg_BLE_SPIPTRCNTL3;

    #define   BLE_AESCNTL_OFFSET                0x0B0
    typedef union {
        struct {
            UINT32 AES_START:                   1;
            UINT32 AES_MODE:                    1;
            UINT32 reserved_0:                  30;
        };
        UINT32 reg_value;
    } reg_BLE_AESCNTL;

    #define   BLE_AESKEY31_0_OFFSET             0x0B4
    typedef union {
        struct {
            UINT32 AESKEY31_0:                  32;
        };
        UINT32 reg_value;
    } reg_BLE_AESKEY31_0;

    #define   BLE_AESKEY63_32_OFFSET             0x0B8
    typedef union {
        struct {
            UINT32 AESKEY63_32:                  32;
        };
        UINT32 reg_value;
    } reg_BLE_AESKEY63_32;

    #define   BLE_AESKEY95_64_OFFSET             0x0BC
    typedef union {
        struct {
            UINT32 AESKEY95_64:                  32;
        };
        UINT32 reg_value;
    } reg_BLE_AESKEY95_64;

    #define   BLE_AESKEY127_96_OFFSET            0x0C0
    typedef union {
        struct {
            UINT32 AESKEY127_96:                 32;
        };
        UINT32 reg_value;
    } reg_BLE_AESKEY127_96;

    #define   BLE_AESPTR_OFFSET                  0x0C4
    typedef union {
        struct {
            UINT32 AESPTR:                       14;
            UINT32 reserved_0:                   18;
        };
        UINT32 reg_value;
    } reg_BLE_AESPTR;

    #define   BLE_TXMICVAL_OFFSET                0x0C8
    typedef union {
        struct {
            UINT32 TXMICVAL:                     32;
        };
        UINT32 reg_value;
    } reg_BLE_TXMICVAL;

    #define   BLE_RXMICVAL_OFFSET                0x0CC
    typedef union {
        struct {
            UINT32 RXMICVAL:                     32;
        };
        UINT32 reg_value;
    } reg_BLE_RXMICVAL;

    #define   BLE_RFTESTCNTL_OFFSET              0x0D0
    typedef union {
        struct {
            UINT32 TXLENGTH:                     8;
            UINT32 reserved_0:                   3;
            UINT32 TXPKTCNTEN:                   1;
            UINT32 TXPLDSRC:                     1;
            UINT32 PRBSTYPE:                     1;
            UINT32 TXLENGTHSRC:                  1;
            UINT32 INFINITETX:                   1;
            UINT32 reserved_1:                   8;
            UINT32 PERCOUNT_MODE:                2;
            UINT32 reserved_2:                   1;
            UINT32 RXPKTCNTEN:                   1;
            UINT32 reserved_3:                   3;
            UINT32 INFINITERX:                   1;
        };
        UINT32 reg_value;
    } reg_BLE_RFTESTCNTL;

    #define   BLE_RFTESTTXSTAT_OFFSET            0x0D4
    typedef union {
        struct {
            UINT32 TXPKTCNT:                     32;
        };
        UINT32 reg_value;
    } reg_BLE_RFTESTTXSTAT;

    #define   BLE_RFTESTRXSTAT_OFFSET            0x0D8
    typedef union {
        struct {
            UINT32 RXPKTCNT:                     32;
        };
        UINT32 reg_value;
    } reg_BLE_RFTESTRXSTAT;

    #define   BLE_TIMGENCNTL_OFFSET              0x0E0
    typedef union {
        struct {
            UINT32 PREFETCH_TIME:                9;
            UINT32 reserved_0:                   7;
            UINT32 PREFETCHABORT_TIME:           10;
            UINT32 reserved_1:                   6;
        };
        UINT32 reg_value;
    } reg_BLE_TIMGENCNTL;

    #define   BLE_FINETIMTGT_OFFSET              0x0E4
    typedef union {
        struct {
            UINT32 FINETARGET:                   28;
            UINT32 reserved_0:                   4;
        };
        UINT32 reg_value;
    } reg_BLE_FINETIMTGT;

    #define   BLE_CLKNTGT1_OFFSET                0x0E8
    typedef union {
        struct {
            UINT32 CLKNTGT1:                     28;
            UINT32 reserved_0:                   4;
        };
        UINT32 reg_value;
    } reg_BLE_CLKNTGT1;

    #define   BLE_HMICROSECTGT1_OFFSET           0x0EC
    typedef union {
        struct {
            UINT32 HMICROSECTGT1:                10;
            UINT32 reserved_0:                   22;
        };
        UINT32 reg_value;
    } reg_BLE_HMICROSECTGT1;

    #define   BLE_CLKNTGT2_OFFSET                0x0F0
    typedef union {
        struct {
            UINT32 CLKNTGT2:                     28;
            UINT32 reserved_0:                   4;
        };
        UINT32 reg_value;
    } reg_BLE_CLKNTGT2;

    #define   BLE_HMICROSECTGT2_OFFSET           0x0F4
    typedef union {
        struct {
            UINT32 HMICROSECTGT2:                10;
            UINT32 reserved_0:                   22;
        };
        UINT32 reg_value;
    } reg_BLE_HMICROSECTGT2;

    #define   BLE_CLKNTGT3_OFFSET                0x0F8
    typedef union {
        struct {
            UINT32 CLKNTGT3:                     28;
            UINT32 reserved_0:                   4;
        };
        UINT32 reg_value;
    } reg_BLE_CLKNTGT3;

    #define   BLE_HMICROSECTGT3_OFFSET           0x0FC
    typedef union {
        struct {
            UINT32 HMICROSECTGT3:                10;
            UINT32 reserved_0:                   22;
        };
        UINT32 reg_value;
    } reg_BLE_HMICROSECTGT3;

    #define   BLE_SLOTCLK_OFFSET                 0x100
    typedef union {
        struct {
            UINT32 SCLK:                         28;
            UINT32 reserved_0:                   2;
            UINT32 CLKN_UPD:                     1;
            UINT32 SAMP:                         1;
        };
        UINT32 reg_value;
    } reg_BLE_SLOTCLK;

    #define   BLE_FINETIMECNT_OFFSET             0x104
    typedef union {
        struct {
            UINT32 FINECNT:                      10;
            UINT32 reserved_0:                   22;
        };
        UINT32 reg_value;
    } reg_BLE_FINETIMECNT;

    #define   BLE_ACTSCHCNTL_OFFSET              0x110
    typedef union {
        struct {
            UINT32 ENTRY_IDX:                    4;
            UINT32 reserved_0:                   27;
            UINT32 START_ACT:                    1;
        };
        UINT32 reg_value;
    } reg_BLE_ACTSCHCNTL;

    #define   BLE_STARTEVTCLKNTS_OFFSET          0x114
    typedef union {
        struct {
            UINT32 STARTEVTCLKNTS:               28;
            UINT32 reserved_0:                   4;
        };
        UINT32 reg_value;
    } reg_BLE_STARTEVTCLKNTS;

    #define   BLE_STARTEVTFINECNTTS_OFFSET       0x118
    typedef union {
        struct {
            UINT32 STARTEVTFINECNTTS:            10;
            UINT32 reserved_0:                   22;
        };
        UINT32 reg_value;
    } reg_BLE_STARTEVTFINECNTTS;

    #define   BLE_ENDEVTCLKNTS_OFFSET            0x11C
    typedef union {
        struct {
            UINT32 ENDEVTCLKNTS:                 28;
            UINT32 reserved_0:                   4;
        };
        UINT32 reg_value;
    } reg_BLE_ENDEVTCLKNTS;

    #define   BLE_ENDEVTFINECNTTS_OFFSET         0x120
    typedef union {
        struct {
            UINT32 ENDEVTFINECNTTS:              10;
            UINT32 reserved_0:                   22;
        };
        UINT32 reg_value;
    } reg_BLE_ENDEVTFINECNTTS;

    #define   BLE_SKIPEVTCLKNTS_OFFSET           0x124
    typedef union {
        struct {
            UINT32 SKIPEVTCLKNTS:                28;
            UINT32 reserved_0:                   4;
        };
        UINT32 reg_value;
    } reg_BLE_SKIPEVTCLKNTS;

    #define   BLE_SKIPEVTFINECNTTS_OFFSET        0x128
    typedef union {
        struct {
            UINT32 SKIPEVTFINECNTTS:             10;
            UINT32 reserved_0:                   22;
        };
        UINT32 reg_value;
    } reg_BLE_SKIPEVTFINECNTTS;

    #define   BLE_ADVTIM_OFFSET                  0x130
    typedef union {
        struct {
            UINT32 ADVINT:                       14;
            UINT32 reserved_0:                   2;
            UINT32 RX_AUXPTR_THR:                8;
            UINT32 TX_AUXPTR_THR:                8;
        };
        UINT32 reg_value;
    } reg_BLE_ADVTIM;

    #define   BLE_ACTSCANCNTL_OFFSET             0x134
    typedef union {
        struct {
            UINT32 UPPERLIMIT:                   8;
            UINT32 reserved_0:                   8;
            UINT32 BACKOFF:                      8;
            UINT32 reserved_1:                   8;
        };
        UINT32 reg_value;
    } reg_BLE_ACTSCANCNTL;

    #define   BLE_WPALCNTL_OFFSET                0x140
    typedef union {
        struct {
            UINT32 WPALBASEPTR:                  14;
            UINT32 reserved_0:                   2;
            UINT32 WPALNBDEV:                    8;
            UINT32 reserved_1:                   8;
        };
        UINT32 reg_value;
    } reg_BLE_WPALCNTL;

    #define   BLE_WPALCURRENTPTR_OFFSET          0x144
    typedef union {
        struct {
            UINT32 WPALCURRENTPTR:               14;
            UINT32 reserved_0:                   18;
        };
        UINT32 reg_value;
    } reg_BLE_WPALCURRENTPTR;

    #define   BLE_SEARCH_TIMEOUT_OFFSET          0x148
    typedef union {
        struct {
            UINT32 SEARCH_TIMEOUT:               6;
            UINT32 reserved_0:                   26;
        };
        UINT32 reg_value;
    } reg_BLE_SEARCH_TIMEOUT;

    #define   BLE_COEXIFCNTL0_OFFSET             0x150
    typedef union {
        struct {
            UINT32 WLANCOEX_EN:                  1;
            UINT32 SYNCGEN_EN:                   1;
            UINT32 MWSCOEX_EN:                   1;
            UINT32 MWSWCI_EN:                    1;
            UINT32 WLANRXMSK:                    2;
            UINT32 WLANTXMSK:                    2;
            UINT32 MWSRXMSK:                     2;
            UINT32 MWSTXMSK:                     2;
            UINT32 MWSRXFREQMSK:                 2;
            UINT32 MWSTXFREQMSK:                 2;
            UINT32 WLCTXPRIOMODE:                2;
            UINT32 WLCRXPRIOMODE:                2;
            UINT32 MWSSCANFREQMSK:               2;
            UINT32 reserved_0:                   10;
        };
        UINT32 reg_value;
    } reg_BLE_COEXIFCNTL0;

    #define   BLE_COEXIFCNTL1_OFFSET             0x154
    typedef union {
        struct {
            UINT32 WLCPDELAY:                    7;
            UINT32 reserved_0:                   1;
            UINT32 WLCPDURATION:                 7;
            UINT32 reserved_1:                   1;
            UINT32 WLCPTXTHR:                    5;
            UINT32 reserved_2:                   3;
            UINT32 WLCPRXTHR:                    5;
            UINT32 reserved_3:                   3;
        };
        UINT32 reg_value;
    } reg_BLE_COEXIFCNTL1;

    #define   BLE_COEXIFCNTL2_OFFSET             0x158
    typedef union {
        struct {
            UINT32 TX_ANT_DELAY:                 4;
            UINT32 reserved_0:                   4;
            UINT32 RX_ANT_DELAY:                 4;
            UINT32 reserved_1:                   20;
        };
        UINT32 reg_value;
    } reg_BLE_COEXIFCNTL2;

    #define   BLE_BLEMPRIO0_OFFSET               0x15C
    typedef union {
        struct {
            UINT32 BLEM0:                        4;
            UINT32 BLEM1:                        4;
            UINT32 BLEM2:                        4;
            UINT32 BLEM3:                        4;
            UINT32 BLEM4:                        4;
            UINT32 BLEM5:                        4;
            UINT32 BLEM6:                        4;
            UINT32 BLEM7:                        4;
        };
        UINT32 reg_value;
    } reg_BLE_BLEMPRIO0;

    #define   BLE_BLEMPRIO1_OFFSET               0x160
    typedef union {
        struct {
            UINT32 BLEM8:                        4;
            UINT32 BLEM9:                        4;
            UINT32 BLEM10:                       4;
            UINT32 BLEM11:                       4;
            UINT32 BLEM12:                       4;
            UINT32 BLEM13:                       4;
            UINT32 BLEM14:                       4;
            UINT32 BLEM15:                       4;
        };
        UINT32 reg_value;
    } reg_BLE_BLEMPRIO1;

    #define   BLE_BLEMPRIO2_OFFSET               0x164
    typedef union {
        struct {
            UINT32 BLEM16:                       4;
            UINT32 BLEM17:                       4;
            UINT32 BLEM18:                       4;
            UINT32 reserved_0:                   16;
            UINT32 BLEMDEFAULT:                  4;
        };
        UINT32 reg_value;
    } reg_BLE_BLEMPRIO2;

    #define   BLE_RALCNTL_OFFSET                 0x170
    typedef union {
        struct {
            UINT32 RALBASEPTR:                   14;
            UINT32 reserved_0:                   2;
            UINT32 RALNBDEV:                     8;
            UINT32 reserved_1:                   8;
        };
        UINT32 reg_value;
    } reg_BLE_RALCNTL;

    #define   BLE_RALCURRENTPTR_OFFSET           0x174
    typedef union {
        struct {
            UINT32 RALCURRENTPTR:                14;
            UINT32 reserved_0:                   18;
        };
        UINT32 reg_value;
    } reg_BLE_RALCURRENTPTR;

    #define   BLE_RAL_LOCAL_RND_OFFSET           0x178
    typedef union {
        struct {
            UINT32 LRND_VAL:                     22;
            UINT32 reserved_0:                   9;
            UINT32 LRND_INIT:                    1;
        };
        UINT32 reg_value;
    } reg_BLE_RAL_LOCAL_RND;

    #define   BLE_RAL_PEER_RND_OFFSET            0x17C
    typedef union {
        struct {
            UINT32 PRND_VAL:                     22;
            UINT32 reserved_0:                   9;
            UINT32 PRND_INIT:                    1;
        };
        UINT32 reg_value;
    } reg_BLE_RAL_PEER_RND;

    #define   BLE_DFCNTL0_1US_OFFSET             0x180
    typedef union {
        struct {
            UINT32 TXSWSTINST0_1US:              8;
            UINT32 reserved_0:                   8;
            UINT32 RXSWSTINST0_1US:              8;
            UINT32 RXSAMPSTINST0_1US:            8;
        };
        UINT32 reg_value;
    } reg_BLE_DFCNTL0_1US;

    #define   BLE_DFCNTL0_2US_OFFSET             0x184
    typedef union {
        struct {
            UINT32 TXSWSTINST0_2US:              8;
            UINT32 reserved_0:                   8;
            UINT32 RXSWSTINST0_2US:              8;
            UINT32 RXSAMPSTINST0_2US:            8;
        };
        UINT32 reg_value;
    } reg_BLE_DFCNTL0_2US;

    #define   BLE_DFCNTL1_1US_OFFSET             0x188
    typedef union {
        struct {
            UINT32 TXSWSTINST1_1US:              8;
            UINT32 reserved_0:                   8;
            UINT32 RXSWSTINST1_1US:              8;
            UINT32 RXSAMPSTINST1_1US:            8;
        };
        UINT32 reg_value;
    } reg_BLE_DFCNTL1_1US;

    #define   BLE_DFCNTL1_2US_OFFSET             0x18C
    typedef union {
        struct {
            UINT32 TXSWSTINST1_2US:              8;
            UINT32 reserved_0:                   8;
            UINT32 RXSWSTINST1_2US:              8;
            UINT32 RXSAMPSTINST1_2US:            8;
        };
        UINT32 reg_value;
    } reg_BLE_DFCNTL1_2US;

    #define   BLE_DFCURRENTPTR_OFFSET            0x190
    typedef union {
        struct {
            UINT32 DFCURRENTPTR:                 14;
            UINT32 reserved_0:                   18;
        };
        UINT32 reg_value;
    } reg_BLE_DFCURRENTPTR;

    #define   BLE_DFANTCNTL_OFFSET               0x194
    typedef union {
        struct {
            UINT32 TXPRIMANTID:                  7;
            UINT32 TXPRIMIDCNTLEN:               1;
            UINT32 RXPRIMANTID:                  7;
            UINT32 RXPRIMIDCNTLEN:               1;
            UINT32 reserved_0:                   16;
        };
        UINT32 reg_value;
    } reg_BLE_DFANTCNTL;

    #define   BLE_DFIFCNTL_OFFSET                0x198
    typedef union {
        struct {
            UINT32 SYMBOL_ORDER:                 1;
            UINT32 MSB_LSB_ORDER:                1;
            UINT32 IF_WIDTH:                     2;
            UINT32 SAMPVALID_BEH:                2;
            UINT32 SAMPREQ_BEH:                  1;
            UINT32 ANTSWITCH_BEH:                1;
            UINT32 reserved_0:                   24;
        };
        UINT32 reg_value;
    } reg_BLE_DFIFCNTL;

    #define   BLE_FREQSELCNTL_OFFSET             0x1A0
    typedef union {
        struct {
            UINT32 FREQSEL_START:                1;
            UINT32 FREQSEL_MODE:                 1;
            UINT32 reserved_0:                   14;
            UINT32 NBLOOPS:                      8;
            UINT32 reserved_1:                   8;
        };
        UINT32 reg_value;
    } reg_BLE_FREQSELCNTL;

    #define   BLE_FREQSELPTR_OFFSET              0x1A4
    typedef union {
        struct {
            UINT32 FREQSELPTR:                   14;
            UINT32 reserved_0:                   18;
        };
        UINT32 reg_value;
    } reg_BLE_FREQSELPTR;

    #define   BLE_FREQSEL_CS1_SEED_OFFSET        0x1A8
    typedef union {
        struct {
            UINT32 FREQSEL_HOPINT:               5;
            UINT32 reserved_0:                   11;
            UINT32 FREQSEL_LAST_CHIDX:           6;
            UINT32 reserved_1:                   10;
        };
        UINT32 reg_value;
    } reg_BLE_FREQSEL_CS1_SEED;

    #define   BLE_FREQSEL_CS2_SEED_OFFSET        0x1AC
    typedef union {
        struct {
            UINT32 FREQSEL_EVTCNT:               16;
            UINT32 CHANNEL_IDENTIFIER:           16;
        };
        UINT32 reg_value;
    } reg_BLE_FREQSEL_CS2_SEED;

    #define   BLE_FREQSEL_LLCHMAP0_OFFSET        0x1B0
    typedef union {
        struct {
            UINT32 FREQSEL_LLCHMAP0:             32;
        };
        UINT32 reg_value;
    } reg_BLE_FREQSEL_LLCHMAP0;

    #define   BLE_FREQSEL_LLCHMAP1_OFFSET        0x1B4
    typedef union {
        struct {
            UINT32 FREQSEL_LLCHMAP1:             5;
            UINT32 reserved_0:                   27;
        };
        UINT32 reg_value;
    } reg_BLE_FREQSEL_LLCHMAP1;

    #define   BLE_ISOCNTCNTL_OFFSET              0x1C0
    typedef union {
        struct {
            UINT32 ISOCORRMODE:                  1;
            UINT32 ISO_PHASE_SHIFT_MODE:         1;
            UINT32 ISO_CLKSHIFT_MODE:            1;
            UINT32 reserved_0:                   27;
            UINT32 ISO_UPD:                      1;
            UINT32 ISOSAMP:                      1;
        };
        UINT32 reg_value;
    } reg_BLE_ISOCNTCNTL;

    #define   BLE_ISOCNTSAMP_OFFSET              0x1C4
    typedef union {
        struct {
            UINT32 ISOCNTSAMP:                   32;
        };
        UINT32 reg_value;
    } reg_BLE_ISOCNTSAMP;

    #define   BLE_ISOCNTCORR_OFFSET              0x1C8
    typedef union {
        struct {
            UINT32 ISOCNTCORR:                   32;
        };
        UINT32 reg_value;
    } reg_BLE_ISOCNTCORR;

    #define   BLE_ISOCNTCORR_HUS_OFFSET          0x1CC
    typedef union {
        struct {
            UINT32 ISOCNTCORR_HUS:               1;
            UINT32 reserved_0:                   31;
        };
        UINT32 reg_value;
    } reg_BLE_ISOCNTCORR_HUS;

    #define   BLE_ISOINCTNL_OFFSET               0x1D0
    typedef union {
        struct {
            UINT32 ISOINTMSK:                    8;
            UINT32 reserved_0:                   24;
        };
        UINT32 reg_value;
    } reg_BLE_ISOINCTNL;

    #define   BLE_ISOINTSTAT_OFFSET              0x1D4
    typedef union {
        struct {
            UINT32 ISOINTSTAT:                   8;
            UINT32 reserved_0:                   24;
        };
        UINT32 reg_value;
    } reg_BLE_ISOINTSTAT;

    #define   BLE_ISOINTACK_OFFSET               0x1D8
    typedef union {
        struct {
            UINT32 ISOINTACK:                    8;
            UINT32 reserved_0:                   24;
        };
        UINT32 reg_value;
    } reg_BLE_ISOINTACK;

    #define   BLE_ISOGPIOCNTL_OFFSET             0x1E0
    typedef union {
        struct {
            UINT32 ISOGPIOMSK:                   8;
            UINT32 reserved_0:                   23;
            UINT32 ISOGPIOBEH:                   1;
        };
        UINT32 reg_value;
    } reg_BLE_ISOGPIOCNTL;

    #define   BLE_ISOTIMERTGT0_OFFSET          0x1F0
    typedef union {
        struct {
            UINT32 ISOTIMERTGT0:               32;
        };
        UINT32 reg_value;
    } reg_BLE_ISOTIMERTGT0;

    #define   BLE_ISOTIMERTGT1_OFFSET          0x1F4
    typedef union {
        struct {
            UINT32 ISOTIMERTGT1:               32;
        };
        UINT32 reg_value;
    } reg_BLE_ISOTIMERTGT1;

    #define   BLE_ISOTIMERTGT2_OFFSET          0x1F8
    typedef union {
        struct {
            UINT32 ISOTIMERTGT2:               32;
        };
        UINT32 reg_value;
    } reg_BLE_ISOTIMERTGT2;

    #define   BLE_ISOTIMERTGT3_OFFSET          0x1FC
    typedef union {
        struct {
            UINT32 ISOTIMERTGT3:               32;
        };
        UINT32 reg_value;
    } reg_BLE_ISOTIMERTGT3;

    #define   BLE_ISOTIMERTGT4_OFFSET          0x200
    typedef union {
        struct {
            UINT32 ISOTIMERTGT4:               32;
        };
        UINT32 reg_value;
    } reg_BLE_ISOTIMERTGT4;

    #define   BLE_ISOTIMERTGT5_OFFSET          0x204
    typedef union {
        struct {
            UINT32 ISOTIMERTGT5:               32;
        };
        UINT32 reg_value;
    } reg_BLE_ISOTIMERTGT5;

    #define   BLE_ISOTIMERTGT6_OFFSET          0x208
    typedef union {
        struct {
            UINT32 ISOTIMERTGT6:               32;
        };
        UINT32 reg_value;
    } reg_BLE_ISOTIMERTGT6;


    #define   BLE_ISOTIMERTGT7_OFFSET          0x20C
    typedef union {
        struct {
            UINT32 ISOTIMERTGT7:               32;
        };
        UINT32 reg_value;
    } reg_BLE_ISOTIMERTGT7;


    // Standard Registers
    #define BLE_CNTL                            ( BLE_BASE_ADDR + BLE_CNTL_OFFSET               )
    #define BLE_VERSION                         ( BLE_BASE_ADDR + BLE_VERSION_OFFSET            )
    #define BLE_CONF                            ( BLE_BASE_ADDR + BLE_CONF_OFFSET               )
    // Interrupt Generator Registers
    #define BLE_INTCNTL0                        ( BLE_BASE_ADDR + BLE_INTCNTL0_OFFSET           )
    #define BLE_INTSTAT0                        ( BLE_BASE_ADDR + BLE_INTSTAT0_OFFSET           )
    #define BLE_INTACK0                         ( BLE_BASE_ADDR + BLE_INTACK0_OFFSET            )
    #define BLE_INTCNTL1                        ( BLE_BASE_ADDR + BLE_INTCNTL1_OFFSET           )
    #define BLE_INTSTAT1                        ( BLE_BASE_ADDR + BLE_INTSTAT1_OFFSET           )
    #define BLE_INTACK1                         ( BLE_BASE_ADDR + BLE_INTACK1_OFFSET            )
    #define BLE_ACTFIFOSTAT                     ( BLE_BASE_ADDR + BLE_ACTFIFOSTAT_OFFSET        )
    // Misc Control Registers
    #define BLE_CURRENTRXDESCPTR                ( BLE_BASE_ADDR + BLE_CURRENTRXDESCPTR_OFFSET   )
    #define BLE_ETPTR                           ( BLE_BASE_ADDR + BLE_ETPTR_OFFSET              )
    // Deep Sleep Registers
    #define BLE_DEEPSLCNTL                      ( BLE_BASE_ADDR + BLE_DEEPSLCNTL_OFFSET         )
    #define BLE_DEEPSLWKUP                      ( BLE_BASE_ADDR + BLE_DEEPSLWKUP_OFFSET         )
    #define BLE_DEEPSLSTAT                      ( BLE_BASE_ADDR + BLE_DEEPSLSTAT_OFFSET         )
    #define BLE_ENBPRESET                       ( BLE_BASE_ADDR + BLE_ENBPRESET_OFFSET          )
    #define BLE_FINECNTCORR                     ( BLE_BASE_ADDR + BLE_FINECNTCORR_OFFSET        )
    #define BLE_CLKNCNTCORR                     ( BLE_BASE_ADDR + BLE_CLKNCNTCORR_OFFSET        )
    //Validation Registers
    #define BLE_DIAGCNTL                        ( BLE_BASE_ADDR + BLE_DIAGCNTL_OFFSET           )
    #define BLE_DIAGSTAT                        ( BLE_BASE_ADDR + BLE_DIAGSTAT_OFFSET           )
    #define BLE_DEBUGADDMAX                     ( BLE_BASE_ADDR + BLE_DEBUGADDMAX_OFFSET        )
    #define BLE_DEBUGADDMIN                     ( BLE_BASE_ADDR + BLE_DEBUGADDMIN_OFFSET        )
    #define BLE_ERRORTYPESTAT                   ( BLE_BASE_ADDR + BLE_ERRORTYPESTAT_OFFSET      )
    #define BLE_SWPROFILING                     ( BLE_BASE_ADDR + BLE_SWPROFILING_OFFSET        )
    // Radio Registers
    #define BLE_RADIOCNTL0                      ( BLE_BASE_ADDR + BLE_RADIOCNTL0_OFFSET         )
    #define BLE_RADIOCNTL1                      ( BLE_BASE_ADDR + BLE_RADIOCNTL1_OFFSET         )
    #define BLE_RADIOCNTL2                      ( BLE_BASE_ADDR + BLE_RADIOCNTL2_OFFSET         )
    #define BLE_RADIOCNTL3                      ( BLE_BASE_ADDR + BLE_RADIOCNTL3_OFFSET         )
    #define BLE_RADIOPWRUPDN0                   ( BLE_BASE_ADDR + BLE_RADIOPWRUPDN0_OFFSET      )
    #define BLE_RADIOPWRUPDN1                   ( BLE_BASE_ADDR + BLE_RADIOPWRUPDN1_OFFSET      )
    #define BLE_RADIOPWRUPDN2                   ( BLE_BASE_ADDR + BLE_RADIOPWRUPDN2_OFFSET      )
    #define BLE_RADIOPWRUPDN3                   ( BLE_BASE_ADDR + BLE_RADIOPWRUPDN3_OFFSET      )
    #define BLE_RADIOTXRXTIM0                   ( BLE_BASE_ADDR + BLE_RADIOTXRXTIM0_OFFSET      )
    #define BLE_RADIOTXRXTIM1                   ( BLE_BASE_ADDR + BLE_RADIOTXRXTIM1_OFFSET      )
    #define BLE_RADIOTXRXTIM2                   ( BLE_BASE_ADDR + BLE_RADIOTXRXTIM2_OFFSET      )
    #define BLE_RADIOTXRXTIM3                   ( BLE_BASE_ADDR + BLE_RADIOTXRXTIM3_OFFSET      )
    #define BLE_SPIPTRCNTL0                     ( BLE_BASE_ADDR + BLE_SPIPTRCNTL0_OFFSET        )
    #define BLE_SPIPTRCNTL1                     ( BLE_BASE_ADDR + BLE_SPIPTRCNTL1_OFFSET        )
    #define BLE_SPIPTRCNTL2                     ( BLE_BASE_ADDR + BLE_SPIPTRCNTL2_OFFSET        )
    #define BLE_SPIPTRCNTL3                     ( BLE_BASE_ADDR + BLE_SPIPTRCNTL3_OFFSET        )
    // Encryption Registers
    #define BLE_AESCNTL                         ( BLE_BASE_ADDR + BLE_AESCNTL_OFFSET            )
    #define BLE_AESKEY31_0                      ( BLE_BASE_ADDR + BLE_AESKEY31_0_OFFSET         )
    #define BLE_AESKEY63_32                     ( BLE_BASE_ADDR + BLE_AESKEY63_32_OFFSET        )
    #define BLE_AESKEY95_64                     ( BLE_BASE_ADDR + BLE_AESKEY95_64_OFFSET        )
    #define BLE_AESKEY127_96                    ( BLE_BASE_ADDR + BLE_AESKEY127_96_OFFSET       )
    #define BLE_AESPTR                          ( BLE_BASE_ADDR + BLE_AESPTR_OFFSET             )
    #define BLE_TXMICVAL                        ( BLE_BASE_ADDR + BLE_TXMICVAL_OFFSET           )
    #define BLE_RXMICVAL                        ( BLE_BASE_ADDR + BLE_RXMICVAL_OFFSET           )
    // Regulatory Body and RF Testing Registers
    #define BLE_RFTESTCNTL                      ( BLE_BASE_ADDR + BLE_RFTESTCNTL_OFFSET         )
    #define BLE_RFTESTTXSTAT                    ( BLE_BASE_ADDR + BLE_RFTESTTXSTAT_OFFSET       )
    #define BLE_RFTESTRXSTAT                    ( BLE_BASE_ADDR + BLE_RFTESTRXSTAT_OFFSET       )
    // Timing Generator Registers
    #define BLE_TIMGENCNTL                      ( BLE_BASE_ADDR + BLE_TIMGENCNTL_OFFSET         )
    #define BLE_FINETIMTGT                      ( BLE_BASE_ADDR + BLE_FINETIMTGT_OFFSET         )
    #define BLE_CLKNTGT1                        ( BLE_BASE_ADDR + BLE_CLKNTGT1_OFFSET           )
    #define BLE_HMICROSECTGT1                   ( BLE_BASE_ADDR + BLE_HMICROSECTGT1_OFFSET      )
    #define BLE_CLKNTGT2                        ( BLE_BASE_ADDR + BLE_CLKNTGT2_OFFSET           )
    #define BLE_HMICROSECTGT2                   ( BLE_BASE_ADDR + BLE_HMICROSECTGT2_OFFSET      )
    #define BLE_CLKNTGT3                        ( BLE_BASE_ADDR + BLE_CLKNTGT3_OFFSET           )
    #define BLE_HMICROSECTGT3                   ( BLE_BASE_ADDR + BLE_HMICROSECTGT3_OFFSET      )
    #define BLE_SLOTCLK                         ( BLE_BASE_ADDR + BLE_SLOTCLK_OFFSET            )
    #define BLE_FINETIMECNT                     ( BLE_BASE_ADDR + BLE_FINETIMECNT_OFFSET        )
    // Activity Scheduler Control Registers
    #define BLE_ACTSCHCNTL                      ( BLE_BASE_ADDR + BLE_ACTSCHCNTL_OFFSET         )
    #define BLE_STARTEVTCLKNTS                  ( BLE_BASE_ADDR + BLE_STARTEVTCLKNTS_OFFSET     )
    #define BLE_STARTEVTFINECNTTS               ( BLE_BASE_ADDR + BLE_STARTEVTFINECNTTS_OFFSET  )
    #define BLE_ENDEVTCLKNTS                    ( BLE_BASE_ADDR + BLE_ENDEVTCLKNTS_OFFSET       )
    #define BLE_ENDEVTFINECNTTS                 ( BLE_BASE_ADDR + BLE_ENDEVTFINECNTTS_OFFSET    )
    #define BLE_SKIPEVTCLKNTS                   ( BLE_BASE_ADDR + BLE_SKIPEVTCLKNTS_OFFSET      )
    #define BLE_SKIPEVTFINECNTTS                ( BLE_BASE_ADDR + BLE_SKIPEVTFINECNTTS_OFFSET   )
    // Advertising Timer and Scanning Timer Registers
    #define BLE_ADVTIM                          ( BLE_BASE_ADDR + BLE_ADVTIM_OFFSET             )
    #define BLE_ACTSCANCNTL                      ( BLE_BASE_ADDR + BLE_ACTSCANCNTL_OFFSET         )
    // Device Filtering Registers
    #define BLE_WPALCNTL                        ( BLE_BASE_ADDR + BLE_WPALCNTL_OFFSET           )
    #define BLE_WPALCURRENTPTR                  ( BLE_BASE_ADDR + BLE_WPALCURRENTPTR_OFFSET     )
    #define BLE_SEARCH_TIMEOUT                   ( BLE_BASE_ADDR + BLE_SEARCH_TIMEOUT_OFFSET      )
    // WLAN Coexistence Registers
    #define BLE_COEXIFCNTL0                     ( BLE_BASE_ADDR + BLE_COEXIFCNTL0_OFFSET        )
    #define BLE_COEXIFCNTL1                     ( BLE_BASE_ADDR + BLE_COEXIFCNTL1_OFFSET        )
    #define BLE_COEXIFCNTL2                     ( BLE_BASE_ADDR + BLE_COEXIFCNTL2_OFFSET        )
    #define BLE_BLEMPRIO0                       ( BLE_BASE_ADDR + BLE_BLEMPRIO0_OFFSET          )
    #define BLE_BLEMPRIO1                       ( BLE_BASE_ADDR + BLE_BLEMPRIO1_OFFSET          )
    #define BLE_BLEMPRIO2                       ( BLE_BASE_ADDR + BLE_BLEMPRIO2_OFFSET          )
    // Resovling Address List Registers
    #define BLE_RALCNTL                         ( BLE_BASE_ADDR + BLE_RALCNTL_OFFSET            )
    #define BLE_RALCURRENTPTR                   ( BLE_BASE_ADDR + BLE_RALCURRENTPTR_OFFSET      )
    #define BLE_RAL_LOCAL_RND                   ( BLE_BASE_ADDR + BLE_RAL_LOCAL_RND_OFFSET      )
    #define BLE_RAL_PEER_RND                    ( BLE_BASE_ADDR + BLE_RAL_PEER_RND_OFFSET       )
    // AoA/AoD Registers
    #define BLE_DFCNTL0_1US                     ( BLE_BASE_ADDR + BLE_DFCNTL0_1US_OFFSET        )
    #define BLE_DFCNTL0_2US                     ( BLE_BASE_ADDR + BLE_DFCNTL0_2US_OFFSET        )
    #define BLE_DFCNTL1_1US                     ( BLE_BASE_ADDR + BLE_DFCNTL1_1US_OFFSET        )
    #define BLE_DFCNTL1_2US                     ( BLE_BASE_ADDR + BLE_DFCNTL1_2US_OFFSET        )
    #define BLE_DFCURRENTPTR                    ( BLE_BASE_ADDR + BLE_DFCURRENTPTR_OFFSET       )
    #define BLE_DFANTCNTL                       ( BLE_BASE_ADDR + BLE_DFANTCNTL_OFFSET          )
    #define BLE_DFIFCNTL                        ( BLE_BASE_ADDR + BLE_DFIFCNTL_OFFSET           )
    // Frequency Selection Registers
    #define BLE_FREQSELCNTL                     ( BLE_BASE_ADDR + BLE_FREQSELCNTL_OFFSET        )
    #define BLE_FREQSELPTR                      ( BLE_BASE_ADDR + BLE_FREQSELPTR_OFFSET         )
    #define BLE_FREQSEL_CS1_SEED                ( BLE_BASE_ADDR + BLE_FREQSEL_CS1_SEED_OFFSET   )
    #define BLE_FREQSEL_CS2_SEED                ( BLE_BASE_ADDR + BLE_FREQSEL_CS2_SEED_OFFSET   )
    #define BLE_FREQSEL_LLCHMAP0                ( BLE_BASE_ADDR + BLE_FREQSEL_LLCHMAP0_OFFSET   )
    #define BLE_FREQSEL_LLCHMAP1                ( BLE_BASE_ADDR + BLE_FREQSEL_LLCHMAP1_OFFSET   )
    // ISO Bluetooth TimeStamp Registers
    #define BLE_ISOCNTCNTL                      ( BLE_BASE_ADDR + BLE_ISOCNTCNTL_OFFSET         )
    #define BLE_ISOCNTSAMP                      ( BLE_BASE_ADDR + BLE_ISOCNTSAMP_OFFSET         )
    #define BLE_ISOCNTCORR                      ( BLE_BASE_ADDR + BLE_ISOCNTCORR_OFFSET         )
    #define BLE_ISOCNTCORR_HUS                  ( BLE_BASE_ADDR + BLE_ISOCNTCORR_HUS_OFFSET     )
    #define BLE_ISOINCTNL                      ( BLE_BASE_ADDR + BLE_ISOINCTNL_OFFSET         )
    #define BLE_ISOINTSTAT                      ( BLE_BASE_ADDR + BLE_ISOINTSTAT_OFFSET         )
    #define BLE_ISOINTACK                       ( BLE_BASE_ADDR + BLE_ISOINTACK_OFFSET          )
    #define BLE_ISOGPIOCNTL                     ( BLE_BASE_ADDR + BLE_ISOGPIOCNTL_OFFSET        )
    #define BLE_ISOTIMERTGT0                    ( BLE_BASE_ADDR + BLE_ISOTIMERTGT0_OFFSET       )
    #define BLE_ISOTIMERTGT1                    ( BLE_BASE_ADDR + BLE_ISOTIMERTGT1_OFFSET       )
    #define BLE_ISOTIMERTGT2                    ( BLE_BASE_ADDR + BLE_ISOTIMERTGT2_OFFSET       )
    #define BLE_ISOTIMERTGT3                    ( BLE_BASE_ADDR + BLE_ISOTIMERTGT3_OFFSET       )
    #define BLE_ISOTIMERTGT4                    ( BLE_BASE_ADDR + BLE_ISOTIMERTGT4_OFFSET       )
    #define BLE_ISOTIMERTGT5                    ( BLE_BASE_ADDR + BLE_ISOTIMERTGT5_OFFSET       )
    #define BLE_ISOTIMERTGT6                    ( BLE_BASE_ADDR + BLE_ISOTIMERTGT6_OFFSET       )
    #define BLE_ISOTIMERTGT7                    ( BLE_BASE_ADDR + BLE_ISOTIMERTGT7_OFFSET       )

    #define   BLE_CNTL_RESET_VALUE                           0x00000000
    #define   BLE_VERSION_RESET_VALUE                        0x0B001200
    #define   BLE_CONF_RESET_VALUE                           0x4c02c88f//
    #define   BLE_INTCNTL0_RESET_VALUE                       0x00000003
    #define   BLE_INTSTAT0_RESET_VALUE                       0x00000000
    #define   BLE_INTACK0_RESET_VALUE                        0x00000000
    #define   BLE_INTCNTL1_RESET_VALUE                       0x00008003
    #define   BLE_INTSTAT1_RESET_VALUE                       0x00000000
    #define   BLE_INTACK1_RESET_VALUE                        0x00000000
    #define   BLE_ACTFIFOSTAT_RESET_VALUE                    0x00000000
    #define   BLE_CURRENTRXDESCPTR_RESET_VALUE               0x00000000
    #define   BLE_ETPTR_RESET_VALUE                          0x00000000
    #define   BLE_DEEPSLCNTL_RESET_VALUE                     0x00000000
    #define   BLE_DEEPSLWKUP_RESET_VALUE                     0x00000000
    #define   BLE_DEEPSLSTAT_RESET_VALUE                     0x00000000
    #define   BLE_ENBPRESET_RESET_VALUE                      0x00000000
    #define   BLE_FINECNTCORR_RESET_VALUE                    0x00000000
    #define   BLE_CLKNCNTCORR_RESET_VALUE                    0x00000000
    #define   BLE_DIAGCNTL_RESET_VALUE                       0x00000000
    #define   BLE_DIAGSTAT_RESET_VALUE                       0x00000000
    #define   BLE_DEBUGADDMAX_RESET_VALUE                    0x00000000
    #define   BLE_DEBUGADDMIN_RESET_VALUE                    0x00000000
    #define   BLE_ERRORTYPESTAT_RESET_VALUE                  0x00000000
    #define   BLE_SWPROFILING_RESET_VALUE                    0x00000000
    #define   BLE_RADIOCNTL0_RESET_VALUE                     0x00000002
    #define   BLE_RADIOCNTL1_RESET_VALUE                     0x00000000
    #define   BLE_RADIOCNTL2_RESET_VALUE                     0xC8000040
    #define   BLE_RADIOCNTL3_RESET_VALUE                     0xE440E400
    #define   BLE_RADIOPWRUPDN0_RESET_VALUE                  0x00000000
    #define   BLE_RADIOPWRUPDN1_RESET_VALUE                  0x00000000
    #define   BLE_RADIOPWRUPDN2_RESET_VALUE                  0x00000000
    #define   BLE_RADIOPWRUPDN3_RESET_VALUE                  0x00000000
    #define   BLE_RADIOTXRXTIM0_RESET_VALUE                  0x00000000
    #define   BLE_RADIOTXRXTIM1_RESET_VALUE                  0x00000000
    #define   BLE_RADIOTXRXTIM2_RESET_VALUE                  0x00000000
    #define   BLE_RADIOTXRXTIM3_RESET_VALUE                  0x00000000
    #define   BLE_SPIPTRCNTL0_RESET_VALUE                    0x00000000
    #define   BLE_SPIPTRCNTL1_RESET_VALUE                    0x00000000
    #define   BLE_SPIPTRCNTL2_RESET_VALUE                    0x00000000
    #define   BLE_SPIPTRCNTL3_RESET_VALUE                    0x00000000
    #define   BLE_AESCNTL_RESET_VALUE                        0x00000000
    #define   BLE_AESKEY31_0_RESET_VALUE                     0x00000000
    #define   BLE_AESKEY63_32_RESET_VALUE                    0x00000000
    #define   BLE_AESKEY95_64_RESET_VALUE                    0x00000000
    #define   BLE_AESKEY127_96_RESET_VALUE                   0x00000000
    #define   BLE_AESPTR_RESET_VALUE                         0x00000000
    #define   BLE_TXMICVAL_RESET_VALUE                       0x00000000
    #define   BLE_RXMICVAL_RESET_VALUE                       0x00000000
    #define   BLE_RFTESTCNTL_RESET_VALUE                     0x00000000
    #define   BLE_RFTESTTXSTAT_RESET_VALUE                   0x00000000
    #define   BLE_RFTESTRXSTAT_RESET_VALUE                   0x00000000
    #define   BLE_TIMGENCNTL_RESET_VALUE                     0x01FE0096
    #define   BLE_FINETIMTGT_RESET_VALUE                     0x00000000
    #define   BLE_CLKNTGT1_RESET_VALUE                       0x00000000
    #define   BLE_HMICROSECTGT1_RESET_VALUE                  0x00000000
    #define   BLE_CLKNTGT2_RESET_VALUE                       0x00000000
    #define   BLE_HMICROSECTGT2_RESET_VALUE                  0x00000000
    #define   BLE_CLKNTGT3_RESET_VALUE                       0x00000000
    #define   BLE_HMICROSECTGT3_RESET_VALUE                  0x00000000
    #define   BLE_SLOTCLK_RESET_VALUE                        0x00000000
    #define   BLE_FINETIMECNT_RESET_VALUE                    0x00000000
    #define   BLE_ACTSCHCNTL_RESET_VALUE                     0x00000000
    #define   BLE_STARTEVTCLKNTS_RESET_VALUE                 0x00000000
    #define   BLE_STARTEVTFINECNTTS_RESET_VALUE              0x00000000
    #define   BLE_ENDEVTCLKNTS_RESET_VALUE                   0x00000000
    #define   BLE_ENDEVTFINECNTTS_RESET_VALUE                0x00000000
    #define   BLE_SKIPEVTCLKNTS_RESET_VALUE                  0x00000000
    #define   BLE_SKIPEVTFINECNTTS_RESET_VALUE               0x00000000
    #define   BLE_ADVTIM_RESET_VALUE                         0x00000000
    #define   BLE_ACTSCANCNTL_RESET_VALUE                    0x00010001
    #define   BLE_WPALCNTL_RESET_VALUE                       0x00000000
    #define   BLE_WPALCURRENTPTR_RESET_VALUE                 0x00000000//
    #define   BLE_SEARCH_TIMEOUT_RESET_VALUE                 0x00000010
    #define   BLE_COEXIFCNTL0_RESET_VALUE                    0x00000000//
    #define   BLE_COEXIFCNTL1_RESET_VALUE                    0x00000000
    #define   BLE_COEXIFCNTL2_RESET_VALUE                    0x00000000
    #define   BLE_BLEMPRIO0_RESET_VALUE                      0x00000000//
    #define   BLE_BLEMPRIO1_RESET_VALUE                      0x00000000//
    #define   BLE_BLEMPRIO2_RESET_VALUE                      0x00000000//
    #define   BLE_RALCNTL_RESET_VALUE                        0x00000000
    #define   BLE_RALCURRENTPTR_RESET_VALUE                  0x00000000
    #define   BLE_RAL_LOCAL_RND_RESET_VALUE                  0x003F0F0F
    #define   BLE_RAL_PEER_RND_RESET_VALUE                   0x0030F0F0
    #define   BLE_DFCNTL0_1US_RESET_VALUE                    0x00000000
    #define   BLE_DFCNTL0_2US_RESET_VALUE                    0x00000000
    #define   BLE_DFCNTL1_1US_RESET_VALUE                    0x00000000
    #define   BLE_DFCNTL1_2US_RESET_VALUE                    0x00000000
    #define   BLE_DFCURRENTPTR_RESET_VALUE                   0x00000000
    #define   BLE_DFANTCNTL_RESET_VALUE                      0x00000000
    #define   BLE_DFIFCNTL_RESET_VALUE                       0x0000000C
    #define   BLE_FREQSELCNTL_RESET_VALUE                    0x00000000
    #define   BLE_FREQSELPTR_RESET_VALUE                     0x00000000
    #define   BLE_FREQSEL_CS1_SEED_RESET_VALUE               0x00000000
    #define   BLE_FREQSEL_CS2_SEED_RESET_VALUE               0x00000000
    #define   BLE_FREQSEL_LLCHMAP0_RESET_VALUE               0x00000000
    #define   BLE_FREQSEL_LLCHMAP1_RESET_VALUE               0x00000000
    #define   BLE_ISOCNTCNTL_RESET_VALUE                     0x00000000
    #define   BLE_ISOCNTSAMP_RESET_VALUE                     0x00000000
    #define   BLE_ISOCNTCORR_RESET_VALUE                     0x00000000
    #define   BLE_ISOCNTCORR_HUS_RESET_VALUE                 0x00000000
    #define   BLE_ISOINCTNL_RESET_VALUE                      0x00000000
    #define   BLE_ISOINTSTAT_RESET_VALUE                     0x00000000
    #define   BLE_ISOINTACK_RESET_VALUE                      0x00000000
    #define   BLE_ISOGPIOCNTL_RESET_VALUE                    0x00000000
    #define   BLE_ISOTIMERTGT0_RESET_VALUE                   0x00000000
    #define   BLE_ISOTIMERTGT1_RESET_VALUE                   0x00000000
    #define   BLE_ISOTIMERTGT2_RESET_VALUE                   0x00000000
    #define   BLE_ISOTIMERTGT3_RESET_VALUE                   0x00000000
    #define   BLE_ISOTIMERTGT4_RESET_VALUE                   0x00000000
    #define   BLE_ISOTIMERTGT5_RESET_VALUE                   0x00000000
    #define   BLE_ISOTIMERTGT6_RESET_VALUE                   0x00000000
    #define   BLE_ISOTIMERTGT7_RESET_VALUE                   0x00000000


    #define   BLE_CNTL_MASK                                  0x207FFF0F////
    #define   BLE_VERSION_MASK                               0x00000000
    #define   BLE_CONF_MASK                                  0x00000000
    #define   BLE_INTCNTL0_MASK                              0x000100FF
    #define   BLE_INTSTAT0_MASK                              0x00000000
    #define   BLE_INTACK0_MASK                               0x00000000/////
    #define   BLE_INTCNTL1_MASK                              0x7F0080FF
    #define   BLE_INTSTAT1_MASK                              0x00000000
    #define   BLE_INTACK1_MASK                               0x00000000/////
    #define   BLE_ACTFIFOSTAT_MASK                           0x00000000
    #define   BLE_CURRENTRXDESCPTR_MASK                      0x00003FFF
    #define   BLE_ETPTR_MASK                                 0x00003FFF
    #define   BLE_DEEPSLCNTL_MASK                            0x8000000F
    #define   BLE_DEEPSLWKUP_MASK                            0xFFFFFFFF
    #define   BLE_DEEPSLSTAT_MASK                            0x00000000
    #define   BLE_ENBPRESET_MASK                             0xFFFFFFFF
    #define   BLE_FINECNTCORR_MASK                           0x000003FF
    #define   BLE_CLKNCNTCORR_MASK                           0x8FFFFFFF
    #define   BLE_DIAGCNTL_MASK                              0xFFFFFFFF
    #define   BLE_DIAGSTAT_MASK                              0x00000000
    #define   BLE_DEBUGADDMAX_MASK                           0xFFFFFFFF
    #define   BLE_DEBUGADDMIN_MASK                           0xFFFFFFFF
    #define   BLE_ERRORTYPESTAT_MASK                         0x00000000
    #define   BLE_SWPROFILING_MASK                           0xFFFFFFFF
    #define   BLE_RADIOCNTL0_MASK                            0x3FFF00B0
    #define   BLE_RADIOCNTL1_MASK                            0xFFFFF3FF
    #define   BLE_RADIOCNTL2_MASK                            0xFFF73FFF
    #define   BLE_RADIOCNTL3_MASK                            0xFF77FF03
    #define   BLE_RADIOPWRUPDN0_MASK                         0xFFFF7FFF
    #define   BLE_RADIOPWRUPDN1_MASK                         0xFFFF7FFF
    #define   BLE_RADIOPWRUPDN2_MASK                         0xFFFF7FFF
    #define   BLE_RADIOPWRUPDN3_MASK                         0x00007FFF
    #define   BLE_RADIOTXRXTIM0_MASK                         0x007F7F7F
    #define   BLE_RADIOTXRXTIM1_MASK                         0x007F7F7F
    #define   BLE_RADIOTXRXTIM2_MASK                         0xFFFFFF7F
    #define   BLE_RADIOTXRXTIM3_MASK                         0xFF7F007F
    #define   BLE_SPIPTRCNTL0_MASK                           0x3FFF3FFF//
    #define   BLE_SPIPTRCNTL1_MASK                           0x3FFF3FFF
    #define   BLE_SPIPTRCNTL2_MASK                           0x3FFF3FFF
    #define   BLE_SPIPTRCNTL3_MASK                           0x3FFF3FFF
    #define   BLE_AESCNTL_MASK                               0x00000003
    #define   BLE_AESKEY31_0_MASK                            0xFFFFFFFF
    #define   BLE_AESKEY63_32_MASK                           0xFFFFFFFF
    #define   BLE_AESKEY95_64_MASK                           0xFFFFFFFF
    #define   BLE_AESKEY127_96_MASK                          0xFFFFFFFF
    #define   BLE_AESPTR_MASK                                0x00003FFF
    #define   BLE_TXMICVAL_MASK                              0x00000000
    #define   BLE_RXMICVAL_MASK                              0x00000000
    #define   BLE_RFTESTCNTL_MASK                            0x8B00F8FF
    #define   BLE_RFTESTTXSTAT_MASK                          0x00000000
    #define   BLE_RFTESTRXSTAT_MASK                          0x00000000
    #define   BLE_TIMGENCNTL_MASK                            0x03FF01FF
    #define   BLE_FINETIMTGT_MASK                            0x0FFFFFFF
    #define   BLE_CLKNTGT1_MASK                              0x0FFFFFFF
    #define   BLE_HMICROSECTGT1_MASK                         0x000003FF
    #define   BLE_CLKNTGT2_MASK                              0x0FFFFFFF
    #define   BLE_HMICROSECTGT2_MASK                         0x000003FF
    #define   BLE_CLKNTGT3_MASK                              0x0FFFFFFF
    #define   BLE_HMICROSECTGT3_MASK                         0x000003FF
    #define   BLE_SLOTCLK_MASK                               0x0FFFFFFF
    #define   BLE_FINETIMECNT_MASK                           0x00000000
    #define   BLE_ACTSCHCNTL_MASK                            0x0000000F
    #define   BLE_STARTEVTCLKNTS_MASK                        0x00000000
    #define   BLE_STARTEVTFINECNTTS_MASK                     0x00000000
    #define   BLE_ENDEVTCLKNTS_MASK                          0x00000000
    #define   BLE_ENDEVTFINECNTTS_MASK                       0x00000000
    #define   BLE_SKIPEVTCLKNTS_MASK                         0x00000000
    #define   BLE_SKIPEVTFINECNTTS_MASK                      0x00000000
    #define   BLE_ADVTIM_MASK                                0xFFFF3FFF
    #define   BLE_ACTSCANCNTL_MASK                           0x01FF01FF
    #define   BLE_WPALCNTL_MASK                              0x00FF3FFF
    #define   BLE_WPALCURRENTPTR_MASK                        0x00003FFF
    #define   BLE_SEARCH_TIMEOUT_MASK                        0x0000003F
    #define   BLE_COEXIFCNTL0_MASK                           0x003FFFFF
    #define   BLE_COEXIFCNTL1_MASK                           0x1F1F7F7F
    #define   BLE_COEXIFCNTL2_MASK                           0x00000F0F
    #define   BLE_BLEMPRIO0_MASK                             0xFFFFFFFF
    #define   BLE_BLEMPRIO1_MASK                             0xFFFFFFFF
    #define   BLE_BLEMPRIO2_MASK                             0xF0000FFF
    #define   BLE_RALCNTL_MASK                               0x00FF3FFF
    #define   BLE_RALCURRENTPTR_MASK                         0x00003FFF
    #define   BLE_RAL_LOCAL_RND_MASK                         0x803FFFFF
    #define   BLE_RAL_PEER_RND_MASK                          0x803FFFFF
    #define   BLE_DFCNTL0_1US_MASK                           0xFFFF00FF
    #define   BLE_DFCNTL0_2US_MASK                           0xFFFF00FF
    #define   BLE_DFCNTL1_1US_MASK                           0xFFFF00FF
    #define   BLE_DFCNTL1_2US_MASK                           0xFFFF00FF
    #define   BLE_DFCURRENTPTR_MASK                          0x00003FFF
    #define   BLE_DFANTCNTL_MASK                             0x0000FFFF
    #define   BLE_DFIFCNTL_MASK                              0x000000FF
    #define   BLE_FREQSELCNTL_MASK                           0x00FF0002
    #define   BLE_FREQSELPTR_MASK                            0x00003FFF
    #define   BLE_FREQSEL_CS1_SEED_MASK                      0x003F001F
    #define   BLE_FREQSEL_CS2_SEED_MASK                      0xFFFFFFFF
    #define   BLE_FREQSEL_LLCHMAP0_MASK                      0xFFFFFFFF
    #define   BLE_FREQSEL_LLCHMAP1_MASK                      0x0000001F
    #define   BLE_ISOCNTCNTL_MASK                            0xC0000007
    #define   BLE_ISOCNTSAMP_MASK                            0xFFFFFFFF
    #define   BLE_ISOCNTCORR_MASK                            0xFFFFFFFF
    #define   BLE_ISOCNTCORR_HUS_MASK                        0x00000001
    #define   BLE_ISOINCTNL_MASK                             0x000000FF
    #define   BLE_ISOINTSTAT_MASK                            0x00000000
    #define   BLE_ISOINTACK_MASK                             0x00000000
    #define   BLE_ISOGPIOCNTL_MASK                           0x800000FF
    #define   BLE_ISOTIMERTGT0_MASK                          0xFFFFFFFF
    #define   BLE_ISOTIMERTGT1_MASK                          0xFFFFFFFF
    #define   BLE_ISOTIMERTGT2_MASK                          0xFFFFFFFF
    #define   BLE_ISOTIMERTGT3_MASK                          0xFFFFFFFF
    #define   BLE_ISOTIMERTGT4_MASK                          0xFFFFFFFF
    #define   BLE_ISOTIMERTGT5_MASK                          0xFFFFFFFF
    #define   BLE_ISOTIMERTGT6_MASK                          0xFFFFFFFF
    #define   BLE_ISOTIMERTGT7_MASK                          0xFFFFFFFF

#endif

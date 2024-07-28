//--------------------------------------------------------------------
// Copyright (c) 2021 by MooreSilicon.
// All rights reserved.
// MooreSilicon Confidential Proprietary.
//--------------------------------------------------------------------
// Project name: Bt_soc
//    File name: ceva_ble_reg.h
//       Author: tuzf
//        Dates: 2021-11-25 11:43:31
//      Version: V1.0
//-------------------------------------------------------------------
//      Purpose: ceva_modem_reg definition and structure
//
//-------------------------------------------------------------------

#ifndef MDM_REG__H
#define MDM_REG__H

#include "type_def.h"


    #define    MDM_VERSION_OFFSET                    0x00
    #define    MDM_CONFIG_OFFSET                     0x04
    #define    MDM_CNTL_OFFSET                       0x30
    #define    MDM_DIAGCNTL_OFFSET                   0x34
    #define    MDM_GSG_CNTL_OFFSET                   0x38
    #define    MDM_GSG_VCO_BLE_OFFSET                0x3C
    #define    MDM_FM2P_SWLAT_BLE_OFFSET             0x40
    #define    MDM_RXRSSI_SCALING_CNTL_OFFSET        0x54
    #define    MDM_RXRSSI_SOP_CNTL_OFFSET            0x58
    #define    MDM_RXSOP_CNTL_BLE_OFFSET             0x5C
    #define    MDM_RXSOP_CNTL_LR_OFFSET              0x60
    #define    MDM_RXSOP_CNTL_BT_OFFSET              0x64
    #define    MDM_RXSYNC_CNTL_BLE_OFFSET            0xAC
    #define    MDM_RXSYNC_CNTL_BT_OFFSET             0xB0
    #define    MDM_RXSYNC_CNTL_LR_OFFSET             0xB4
    #define    MDM_RXHPDEMOD_CFO_CNTL_OFFSET         0xB8
    #define    MDM_RXSTOEST_CNTL0_OFFSET             0xCC
    #define    MDM_RXSTOEST_CNTL1_OFFSET             0xD0
    #define    MDM_RXSTOEST_KALMAN_CNTL_OFFSET       0xD4
    #define    MDM_RXSTOCOMP_CNTL_BTBLE_OFFSET       0xD8
    #define    MDM_RX_CNTL_LR_OFFSET                 0xDC
    #define    MDM_RXLPDEMOD_CNTL_OFFSET             0xE0
    #define    MDM_STAT_RXRSSI_OFFSET                0x1C
    #define    MDM_STAT_RXSYNC0_OFFSET               0x10
    #define    MDM_STAT_RXSTO_OFFSET                 0x20
    #define    MDM_RXRSSI_THR_CNTL_OFFSET            0x50
    #define    MDM_RXTSI_COEFF_LUT_CNTL_OFFSET       0x68
    #define    MDM_RXTSI_COEFF_LUT_VAL_OFFSET        0x6C
    #define    MDM_RXSWC_COEFF_TAU0_OFFSET           0xA0
    #define    MDM_RXSWC_COEFF_TAU1_OFFSET           0xA4
    #define    MDM_RXSWC_COEFF_TAU2_OFFSET           0xA8
    #define    MDM_HPDEMOD_QCOEFF_BLE1_OFFSET        0xBC
    #define    MDM_HPDEMOD_QCOEFF_BLE2_OFFSET        0xC0
    #define    MDM_HPDEMOD_QCOEFF_BT_OFFSET          0xC4
    #define    MDM_HPDEMOD_RHO_OFFSET                0xC8
    #define    MDM_STAT_RXTSI0_OFFSET                0x08
    #define    MDM_STAT_RXTSI1_OFFSET                0x0C
    #define    MDM_STAT_RXSYNC1_OFFSET               0x14
    #define    MDM_STAT_RXSYNC2_OFFSET               0x18
    #define    MDM_STAT_RXLR_OFFSET                  0x24

    #define    MDM_VERSION_RESET_VALUE               0x00010302
    #define    MDM_CONFIG_RESET_VALUE                0x00000000//Reset values depend on the configuration parameters
    #define    MDM_CNTL_RESET_VALUE                  0x2A2101B5
    #define    MDM_DIAGCNTL_RESET_VALUE              0x07063700
    #define    MDM_GSG_CNTL_RESET_VALUE              0x01000100
    #define    MDM_GSG_VCO_BLE_RESET_VALUE           0xBA0AB909
    #define    MDM_FM2P_SWLAT_BLE_RESET_VALUE        0x00030006
    #define    MDM_RXRSSI_SCALING_CNTL_RESET_VALUE   0x00000000
    #define    MDM_RXRSSI_SOP_CNTL_RESET_VALUE       0x0A060400
    #define    MDM_RXSOP_CNTL_BLE_RESET_VALUE        0x00130013
    #define    MDM_RXSOP_CNTL_LR_RESET_VALUE         0x00003013
    #define    MDM_RXSOP_CNTL_BT_RESET_VALUE         0x00000013
    #define    MDM_RXSYNC_CNTL_BLE_RESET_VALUE       0x0800011A
    #define    MDM_RXSYNC_CNTL_BT_RESET_VALUE        0x05A130C8
    #define    MDM_RXSYNC_CNTL_LR_RESET_VALUE        0x099A0008
    #define    MDM_RXHPDEMOD_CFO_CNTL_RESET_VALUE    0x00000000
    #define    MDM_RXSTOEST_CNTL0_RESET_VALUE        0xCF9F00C2
    #define    MDM_RXSTOEST_CNTL1_RESET_VALUE        0x01F303E7
    #define    MDM_RXSTOEST_KALMAN_CNTL_RESET_VALUE  0x30F901F3
    #define    MDM_RXSTOCOMP_CNTL_BTBLE_RESET_VALUE  0x10001000
    #define    MDM_RX_CNTL_LR_RESET_VALUE            0x00000000
    #define    MDM_RXLPDEMOD_CNTL_RESET_VALUE        0x04100000
    #define    MDM_STAT_RXRSSI_RESET_VALUE           0x00000000
    #define    MDM_STAT_RXSYNC0_RESET_VALUE          0x00000000
    #define    MDM_STAT_RXSTO_RESET_VALUE            0x00000000
    #define    MDM_RXRSSI_THR_CNTL_RESET_VALUE       0x00CCD0CD
    #define    MDM_RXTSI_COEFF_LUT_CNTL_RESET_VALUE  0x00000000
    #define    MDM_RXTSI_COEFF_LUT_VAL_RESET_VALUE   0x00000000
    #define    MDM_RXSWC_COEFF_TAU0_RESET_VALUE      0x151F0806
    #define    MDM_RXSWC_COEFF_TAU1_RESET_VALUE      0x0124040B
    #define    MDM_RXSWC_COEFF_TAU2_RESET_VALUE      0x2F250210
    #define    MDM_HPDEMOD_QCOEFF_BLE1_RESET_VALUE   0x01000432
    #define    MDM_HPDEMOD_QCOEFF_BLE2_RESET_VALUE   0x00000432
    #define    MDM_HPDEMOD_QCOEFF_BT_RESET_VALUE     0x01000432
    #define    MDM_HPDEMOD_RHO_RESET_VALUE           0x00000000
    #define    MDM_STAT_RXTSI0_RESET_VALUE           0x00000000
    #define    MDM_STAT_RXTSI1_RESET_VALUE           0x00000000
    #define    MDM_STAT_RXSYNC1_RESET_VALUE          0x00000000
    #define    MDM_STAT_RXSYNC2_RESET_VALUE          0x00000000
    #define    MDM_STAT_RXLR_RESET_VALUE             0x00000000

    #define    MDM_VERSION_MASK                      0x00000000
    #define    MDM_CONFIG_MASK                       0x00000000
    #define    MDM_CNTL_MASK                         0xFFFFF7F7
    #define    MDM_DIAGCNTL_MASK                     0xFFFFFFFF
    #define    MDM_GSG_CNTL_MASK                     0x7F0FFF07
    #define    MDM_GSG_VCO_BLE_MASK                  0xFF1FFF1F
    #define    MDM_FM2P_SWLAT_BLE_MASK               0x001F001F
    #define    MDM_RXRSSI_SCALING_CNTL_MASK          0xFFFFFFFF
    #define    MDM_RXRSSI_SOP_CNTL_MASK              0xFF7F3F00
    #define    MDM_RXSOP_CNTL_BLE_MASK               0x30FF30FF
    #define    MDM_RXSOP_CNTL_LR_MASK                0x000030FF
    #define    MDM_RXSOP_CNTL_BT_MASK                0x000030FF
    #define    MDM_RXSYNC_CNTL_BLE_MASK              0x9FFFF3FF
    #define    MDM_RXSYNC_CNTL_BT_MASK               0x00000000
    #define    MDM_RXSYNC_CNTL_LR_MASK               0x1FFF00FF
    #define    MDM_RXHPDEMOD_CFO_CNTL_MASK           0x0001007F
    #define    MDM_RXSTOEST_CNTL0_MASK               0xFFFF0FF6
    #define    MDM_RXSTOEST_CNTL1_MASK               0x0FFF0FFF
    #define    MDM_RXSTOEST_KALMAN_CNTL_MASK         0x00000000
    #define    MDM_RXSTOCOMP_CNTL_BTBLE_MASK         0x17FF17FF
    #define    MDM_RX_CNTL_LR_MASK                   0x717F17FF
    #define    MDM_RXLPDEMOD_CNTL_MASK               0x071110FF
    #define    MDM_STAT_RXRSSI_MASK                  0x00000000
    #define    MDM_STAT_RXSYNC0_MASK                 0x00000000
    #define    MDM_STAT_RXSTO_MASK                   0x00000000
    #define    MDM_RXRSSI_THR_CNTL_MASK              0x00FFFFFF
    #define    MDM_RXTSI_COEFF_LUT_CNTL_MASK         0x9F003FFF
    #define    MDM_RXTSI_COEFF_LUT_VAL_MASK          0x00000000
    #define    MDM_RXSWC_COEFF_TAU0_MASK             0x7F7F7F7F
    #define    MDM_RXSWC_COEFF_TAU1_MASK             0x7F7F7F7F
    #define    MDM_RXSWC_COEFF_TAU2_MASK             0x7F7F7F7F
    #define    MDM_HPDEMOD_QCOEFF_BLE1_MASK          0x0FFF0FFF
    #define    MDM_HPDEMOD_QCOEFF_BLE2_MASK          0x0FFF0FFF
    #define    MDM_HPDEMOD_QCOEFF_BT_MASK            0x0FFF0FFF
    #define    MDM_HPDEMOD_RHO_MASK                  0xFF9FF1FF
    #define    MDM_STAT_RXTSI0_MASK                  0x00000000
    #define    MDM_STAT_RXTSI1_MASK                  0x00000000
    #define    MDM_STAT_RXSYNC1_MASK                 0x00000000
    #define    MDM_STAT_RXSYNC2_MASK                 0x00000000
    #define    MDM_STAT_RXLR_MASK                    0x00000000

    typedef union {
        struct {
            UINT32 BUILD:                    8;
            UINT32 UPG:                      8;
            UINT32 REL:                      8;
            UINT32 reserved_0:               8;
        };
        UINT32 reg_value;
    } reg_MDM_VERSION;

    typedef union {
        struct {
            UINT32 BLE_LR:                   1;
            UINT32 OQPSK_154:                1;
            UINT32 reserved_0:               14;
            UINT32 BTDM:                     1;
            UINT32 reserved_1:               15;
        };
        UINT32 reg_value;
    } reg_MDM_CONFIG;

    typedef union {
        struct {
            UINT32 FMTXEN:                   1;
            UINT32 FMTX2PEN:                 1;
            UINT32 TX_VALID_MODE:            1;
            UINT32 reserved_0:               1;
            UINT32 HPLP_MODE:                2;
            UINT32 HPLP_BB_IF:               1;
            UINT32 RX_VALID_MODE:            1;
            UINT32 RXCTES_EN:                1;
            UINT32 CTES_SOURCE:              1;
            UINT32 CTES_FORCE_EN:            1;
            UINT32 reserved_1:               1;
            UINT32 CTES_SAMP_DLY:            4;
            UINT32 RX_STARTUPDEL:            8;
            UINT32 TX_STARTUPDEL:            8;
        };
        UINT32 reg_value;
    } reg_MDM_CNTL;

    typedef union {
        struct {
            UINT32 GSG_DEN:                  3;
            UINT32 reserved_0:               5;
            UINT32 GSG_NOM:                  8;
            UINT32 GSG_DPHI_DEN_BLE:         4;
            UINT32 reserved_1:               4;
            UINT32 GSG_DPHI_NOM_BLE:         7;
            UINT32 reserved_2:               1;
        };
        UINT32 reg_value;
    } reg_MDM_GSG_CNTL;

    typedef union {
        struct {
            UINT32 GSG_VCO_DEN_BLE1M:        5;
            UINT32 reserved_0:               3;
            UINT32 GSG_VCO_NOM_BLE1M:        8;
            UINT32 GSG_VCO_DEN_BLE2M:        5;
            UINT32 reserved_1:               3;
            UINT32 GSG_VCO_NOM_BLE2M:        8;
        };
        UINT32 reg_value;
    } reg_MDM_GSG_VCO_BLE;

    typedef union {
        struct {
            UINT32 FM2P_SWLAT_BLE1M:         5;
            UINT32 reserved_0:               11;
            UINT32 FM2P_SWLAT_BLE2M:         5;
            UINT32 reserved_1:               11;
        };
        UINT32 reg_value;
    } reg_MDM_FM2P_SWLAT_BLE;

    typedef union {
        struct {
            UINT32 RSSI_SCALING_BLE1:        8;
            UINT32 RSSI_SCALING_BLE2:        8;
            UINT32 RSSI_SCALING_BT:          8;
            UINT32 RSSI_SCALING_LR:          8;
        };
        UINT32 reg_value;
    } reg_MDM_RXRSSI_SCALING_CNTL;

    typedef union {
        struct {
            UINT32 reserved_0:               8;
            UINT32 SOP_THR:                  6;
            UINT32 reserved_1:               2;
            UINT32 SOP_KAL_Q:                7;
            UINT32 reserved_2:               1;
            UINT32 SOP_KAL_R:                8;
        };
        UINT32 reg_value;
    } reg_MDM_RXRSSI_SOP_CNTL;

    typedef union {
        struct {
            UINT32 SOP_DEL_BLE1:             8;
            UINT32 reserved_0:               4;
            UINT32 SOP_MODE_BLE1:            1;
            UINT32 SOP_HIGHTHR_EN_BLE1:      1;
            UINT32 reserved_1:               2;
            UINT32 SOP_DEL_BLE2:             8;
            UINT32 reserved_2:               4;
            UINT32 SOP_MODE_BLE2:            1;
            UINT32 SOP_HIGHTHR_EN_BLE2:      1;
            UINT32 reserved_3:               2;
        };
        UINT32 reg_value;
    } reg_MDM_RXSOP_CNTL_BLE;

    typedef union {
        struct {
            UINT32 SOP_DEL_LR:               8;
            UINT32 reserved_0:               4;
            UINT32 SOP_MODE_LR:              1;
            UINT32 SOP_HIGHTHR_EN_LR:        1;
            UINT32 reserved_1:               18;
        };
        UINT32 reg_value;
    } reg_MDM_RXSOP_CNTL_LR;

    typedef union {
        struct {
            UINT32 SOP_DEL_BT:               8;
            UINT32 reserved_0:               4;
            UINT32 SOP_MODE_BT:              1;
            UINT32 SOP_HIGHTHR_EN_BT:        1;
            UINT32 reserved_1:               18;
        };
        UINT32 reg_value;
    } reg_MDM_RXSOP_CNTL_BT;

    typedef union {
        struct {
            UINT32 SYNC_LLHOOD_THRES_BLE:    10;
            UINT32 reserved_0:               2;
            UINT32 SYNC_NB_ERR_MAX_BLE:      4;
            UINT32 SYNC_TSI_THRES_BLE:       13;
            UINT32 reserved_1:               2;
            UINT32 SYNC_MIN_EN:              1;
        };
        UINT32 reg_value;
    } reg_MDM_RXSYNC_CNTL_BLE;

    typedef union {
        struct {
            UINT32 SYNC_LLHOOD_THRES_BT:     10;
            UINT32 reserved_0:               2;
            UINT32 SYNC_NB_ERR_MAX_BT:       4;
            UINT32 SYNC_TSI_THRES_BT:        13;
            UINT32 reserved_1:               3;
        };
        UINT32 reg_value;
    } reg_MDM_RXSYNC_CNTL_BT;

    typedef union {
        struct {
            UINT32 SYNC_WIN_MARGIN_US_LR:    8;
            UINT32 reserved_0:               8;
            UINT32 SYNC_TSI_THRES_LR:        13;
            UINT32 reserved_1:               3;
        };
        UINT32 reg_value;
    } reg_MDM_RXSYNC_CNTL_LR;

    typedef union {
        struct {
            UINT32 HP_CFO_VAL:               7;
            UINT32 reserved_0:               9;
            UINT32 HP_CFO_FORCE:             1;
            UINT32 reserved_1:               15;
        };
        UINT32 reg_value;
    } reg_MDM_RXHPDEMOD_CFO_CNTL;

    typedef union {
        struct {
            UINT32 reserved_0:               1;
            UINT32 STO_PREPROC_EN:           1;
            UINT32 STO_SCALING_FORCE:        1;
            UINT32 reserved_1:               1;
            UINT32 STO_SCALING:              4;
            UINT32 STO_SCALING_CAP_BLE:      1;
            UINT32 STO_SCALING_CAP_BT:       1;
            UINT32 STO_SCALING_CAP_LR:       1;
            UINT32 STO_SCALING_CAP_154:      1;
            UINT32 reserved_2:               4;
            UINT32 STO_STOP_DEL:             12;
            UINT32 STOEST_EN_BLE:            1;
            UINT32 STOEST_EN_BT:             1;
            UINT32 STOEST_EN_LR:             1;
            UINT32 STOEST_EN_154:            1;
        };
        UINT32 reg_value;
    } reg_MDM_RXSTOEST_CNTL0;

    typedef union {
        struct {
            UINT32 STO_START_DEL_1MHZ:       12;
            UINT32 reserved_0:               4;
            UINT32 STO_START_DEL_2MHZ:       12;
            UINT32 reserved_1:               4;
        };
        UINT32 reg_value;
    } reg_MDM_RXSTOEST_CNTL1;

    typedef union {
        struct {
            UINT32 STO_KALMAN_DEL_1MHZ:      12;
            UINT32 reserved_0:               4;
            UINT32 STO_KALMAN_DEL_2MHZ:      12;
            UINT32 STO_KALMAN_QR:            3;
            UINT32 reserved_1:               1;
        };
        UINT32 reg_value;
    } reg_MDM_RXSTOEST_KALMAN_CNTL;

    typedef union {
        struct {
            UINT32 STO_COMP_VAL_BLE:         11;
            UINT32 reserved_0:               1;
            UINT32 STO_COMP_FORCE_BLE:       1;
            UINT32 reserved_1:               3;
            UINT32 STO_COMP_VAL_BT:          11;
            UINT32 reserved_2:               1;
            UINT32 STO_COMP_FORCE_BT:        1;
            UINT32 reserved_3:               3;
        };
        UINT32 reg_value;
    } reg_MDM_RXSTOCOMP_CNTL_BTBLE;

    typedef union {
        struct {
            UINT32 STO_COMP_VAL_LR:          11;
            UINT32 reserved_0:               1;
            UINT32 STO_COMP_FORCE_LR:        1;
            UINT32 reserved_1:               3;
            UINT32 LR_CFO_VAL:               7;
            UINT32 reserved_2:               1;
            UINT32 LR_CFO_FORCE:             1;
            UINT32 reserved_3:               3;
            UINT32 LR_CI_VAL:                2;
            UINT32 LR_CI_FORCE:              1;
            UINT32 reserved_4:               1;
        };
        UINT32 reg_value;
    } reg_MDM_RX_CNTL_LR;

    typedef union {
        struct {
            UINT32 LP_CFO_VAL:               8;
            UINT32 reserved_0:               4;
            UINT32 LP_CFO_FORCE:             1;
            UINT32 reserved_1:               3;
            UINT32 LP_DFE_BYPASS:            1;
            UINT32 reserved_2:               3;
            UINT32 LP_CFD_COMP_EN:           1;
            UINT32 reserved_3:               3;
            UINT32 LP_CFD_GAMMA:             3;
            UINT32 reserved_4:               5;
        };
        UINT32 reg_value;
    } reg_MDM_RXLPDEMOD_CNTL;

    typedef union {
        struct {
            UINT32 STAT_RSSI_EST_DBV:        8;
            UINT32 STAT_RX_NHP_LP_SEL:       1;
            UINT32 reserved_0:               3;
            UINT32 STAT_RSSI_EST_LIN:        18;
            UINT32 reserved_1:               2;
        };
        UINT32 reg_value;
    } reg_MDM_STAT_RXRSSI;

    typedef union {
        struct {
            UINT32 STAT_NB_SYNC_ERR:         8;
            UINT32 reserved_0:               8;
            UINT32 STAT_SYNC_OK:             1;
            UINT32 reserved_1:               3;
            UINT32 STAT_CFOEST:              9;
            UINT32 reserved_2:               3;
        };
        UINT32 reg_value;
    } reg_MDM_STAT_RXSYNC0;

    typedef union {
        struct {
            UINT32 STAT_STO_EST:             20;
            UINT32 STAT_STO_SCALING:         4;
            UINT32 STAT_STO_INDEX0:          8;
        };
        UINT32 reg_value;
    } reg_MDM_STAT_RXSTO;

    typedef union {
        struct {
            UINT32 RSSI_SELTHR_BLE1:         8;
            UINT32 RSSI_SELTHR_BLE2:         8;
            UINT32 RSSI_SELTHR_BT:           8;
            UINT32 reserved_0:               8;
        };
        UINT32 reg_value;
    } reg_MDM_RXRSSI_THR_CNTL;

    typedef union {
        struct {
            UINT32 LUT_RXTSI_COEFF_IN:       14;
            UINT32 reserved_0:               10;
            UINT32 LUT_RXTSI_COEFF_ADD:      5;
            UINT32 reserved_1:               2;
            UINT32 LUT_RXTSI_COEFF_WEN:      1;
        };
        UINT32 reg_value;
    } reg_MDM_RXTSI_COEFF_LUT_CNTL;

    typedef union {
        struct {
            UINT32 LUT_RXTSI_COEFF_VAL:      14;
            UINT32 reserved_0:               18;
        };
        UINT32 reg_value;
    } reg_MDM_RXTSI_COEFF_LUT_VAL;

    typedef union {
        struct {
            UINT32 SWC_COEFF_TAU0_D11:       7;
            UINT32 reserved_0:               1;
            UINT32 SWC_COEFF_TAU0_D12:       7;
            UINT32 reserved_1:               1;
            UINT32 SWC_COEFF_TAU0_D21:       7;
            UINT32 reserved_2:               1;
            UINT32 SWC_COEFF_TAU3_D11:       7;
            UINT32 reserved_3:               1;
        };
        UINT32 reg_value;
    } reg_MDM_RXSWC_COEFF_TAU0;

    typedef union {
        struct {
            UINT32 SWC_COEFF_TAU1_D11:       7;
            UINT32 reserved_0:               1;
            UINT32 SWC_COEFF_TAU1_D12:       7;
            UINT32 reserved_1:               1;
            UINT32 SWC_COEFF_TAU1_D21:       7;
            UINT32 reserved_2:               1;
            UINT32 SWC_COEFF_TAU3_D12:       7;
            UINT32 reserved_3:               1;
        };
        UINT32 reg_value;
    } reg_MDM_RXSWC_COEFF_TAU1;

    typedef union {
        struct {
            UINT32 SWC_COEFF_TAU2_D11:       7;
            UINT32 reserved_0:               1;
            UINT32 SWC_COEFF_TAU2_D12:       7;
            UINT32 reserved_1:               1;
            UINT32 SWC_COEFF_TAU2_D21:       7;
            UINT32 reserved_2:               1;
            UINT32 SWC_COEFF_TAU3_D21:       7;
            UINT32 reserved_3:               1;
        };
        UINT32 reg_value;
    } reg_MDM_RXSWC_COEFF_TAU2;

    typedef union {
        struct {
            UINT32 HP_Q0_0_BLE1:             4;
            UINT32 HP_Q0_1_BLE1:             4;
            UINT32 HP_Q0_2_BLE1:             4;
            UINT32 reserved_0:               4;
            UINT32 HP_Q1_0_BLE1:             4;
            UINT32 HP_Q1_1_BLE1:             4;
            UINT32 HP_Q1_2_BLE1:             4;
            UINT32 reserved_1:               4;
        };
        UINT32 reg_value;
    } reg_MDM_HPDEMOD_QCOEFF_BLE1;

    typedef union {
        struct {
            UINT32 HP_Q0_0_BLE2:             4;
            UINT32 HP_Q0_1_BLE2:             4;
            UINT32 HP_Q0_2_BLE2:             4;
            UINT32 reserved_0:               4;
            UINT32 HP_Q1_0_BLE2:             4;
            UINT32 HP_Q1_1_BLE2:             4;
            UINT32 HP_Q1_2_BLE2:             4;
            UINT32 reserved_1:               4;
        };
        UINT32 reg_value;
    } reg_MDM_HPDEMOD_QCOEFF_BLE2;

    typedef union {
        struct {
            UINT32 HP_Q0_0_BT:               4;
            UINT32 HP_Q0_1_BT:               4;
            UINT32 HP_Q0_2_BT:               4;
            UINT32 reserved_0:               4;
            UINT32 HP_Q1_0_BT:               4;
            UINT32 HP_Q1_1_BT:               4;
            UINT32 HP_Q1_2_BT:               4;
            UINT32 reserved_1:               4;
        };
        UINT32 reg_value;
    } reg_MDM_HPDEMOD_QCOEFF_BT;

    typedef union {
        struct {
            UINT32 HP_RHO_BLE1:              9;
            UINT32 reserved_0:               3;
            UINT32 HP_RHO_BLE2:              9;
            UINT32 reserved_1:               2;
            UINT32 HP_RHO_BT:                9;
        };
        UINT32 reg_value;
    } reg_MDM_HPDEMOD_RHO;

    typedef union {
        struct {
            UINT32 STAT_TSI_START:           14;
            UINT32 reserved_0:               2;
            UINT32 STAT_TSI_STOP:            14;
            UINT32 reserved_1:               2;
        };
        UINT32 reg_value;
    } reg_MDM_STAT_RXTSI0;

    typedef union {
        struct {
            UINT32 STAT_PEAK_CORR_INDEX:     14;
            UINT32 reserved_0:               2;
            UINT32 STAT_PEAK_CORR_TAU:       2;
            UINT32 reserved_1:               14;
        };
        UINT32 reg_value;
    } reg_MDM_STAT_RXTSI1;

    typedef union {
        struct {
            UINT32 STAT_ZCIND:               30;
            UINT32 STAT_ZCIND_OK:            1;
            UINT32 STAT_PSI_DL_SEL:          1;
        };
        UINT32 reg_value;
    } reg_MDM_STAT_RXSYNC1;

    typedef union {
        struct {
            UINT32 STAT_SYNC_WORD_LSB:       32;
        };
        UINT32 reg_value;
    } reg_MDM_STAT_RXSYNC2;

    typedef union {
        struct {
            UINT32 STAT_LR_SYNC_IND:         4;
            UINT32 reserved_0:               12;
            UINT32 STAT_LR_CI:               2;
            UINT32 reserved_1:               14;
        };
        UINT32 reg_value;
    } reg_MDM_STAT_RXLR;


#endif

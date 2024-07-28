//--------------------------------------------------------------------
// Copyright (c) 2021 by MooreSilicon.
// All rights reserved.
// MooreSilicon Confidential Proprietary.
//--------------------------------------------------------------------
// Project name: Bt_soc
//    File name: ble.h
//       Author: liuyd
//        Dates: 2021-10-26 14:09:40
//      Version: V1.0
//-------------------------------------------------------------------
//      Purpose:  
//
//-------------------------------------------------------------------

#ifndef BLE__H
#define BLE__H

    #include "global.h"
    #include "ble_reg.h"
    #include "ble_em.h"
    // #include "ble_intr.h"

    //RTL options
    #define RW_BLE_EXTRC_INST
    #define RW_BLE_ADDRESS_WIDTH 15
    #define RW_BLE_LPCLKFREQ  32.77
    typedef enum {
        BLE_1M_UNCODED,
        BLE_2M_UNCODED,
        BLE_125K_CODED,
        BLE_500K_CODED
    } ble_packet_rate;

    typedef enum {
        BLE_ADV_RPT_ALL,
        BLE_ADV_RPT_CORR_ONLY
    } ble_adv_report_policy;

    typedef enum {
        BT_MODE_NONE,
        BT_MODE_BR_EDR,
        BT_MODE_LE
    } bt_mode;

    typedef enum {
        BLE_ACL_EVENT,
        BLE_ISO_EVENT
    } ble_event_type;

    typedef enum {
        BLE_ISO_RETRANS_EVENT,
        BLE_ISO_RSVD_EVENT
    } ble_iso_event_type;

    typedef enum {
        BLE_AE_PRIMARY_CH,
        BLE_AE_SECONDARY_CH
    } ble_adv_ext_start_point;

    typedef enum {
        BLE_SPA_BY_PRIO,
        BLE_SPA_UNCOND
    } ble_spa_policy;

    typedef enum {
        BLE_PP_DURATION_UNIT_US,
        BLE_PP_DURATION_UNIT_HSLOT,
    } ble_primary_priority_duration_unit;

    typedef enum {
        BLE_FMT_MST_CONN               = 0x2,
        BLE_FMT_SLV_CONN               = 0x3,
        BLE_FMT_LO_DUTY_CYC_ADV        = 0x4,
        BLE_FMT_HI_DUTY_CYC_ADV        = 0x5,
        BLE_FMT_EXT_ADV                = 0x6,
        BLE_FMT_PASSIVE_SCANNER        = 0x8,
        BLE_FMT_ACTIVE_SCANNER         = 0x9,
        BLE_FMT_EXT_PASSIVE_SCANNER    = 0xa,
        BLE_FMT_EXT_ACTIVE_SCANNER     = 0xb,
        BLE_FMT_CH_SCANNER             = 0xc,
        BLE_FMT_INIT                   = 0xe,
        BLE_FMT_EXT_INIT               = 0xf,
        BLE_FMT_TX_TEST_MODE           = 0x1c,
        BLE_FMT_RX_TEST_MODE           = 0x1d,
        BLE_FMT_TXRX_TEST_MODE         = 0x1e
    } ble_format_type;

    typedef enum {
        BLE_HAS_MIC,
        BLE_NO_MIC
    } ble_mic_mode;

    typedef enum {
        BLE_AES_MODE_CCM,
        BLE_AES_MODE_CTR
    } ble_aes_mode;

    typedef enum {
        BLE_BD_ADDR_PUBLIC,
        BLE_BD_ADDR_PRIVATE
    } ble_bd_addr_type;

    typedef enum {
        BLE_FREQ_HOP_SCH_1,
        BLE_FREQ_HOP_SCH_2,
        BLE_FREQ_HOP_SW,
        BLE_FREQ_HOP_PRIM_ADV_CH
    } ble_freq_hop_mode;

    typedef enum {
        BLE_RX_WIN_UNIT_US,
        BLE_RX_WIN_UNIT_HSLOT
    } ble_rx_win_unit;

    #define BLE_XMEM_BASE_ADDR             EM_RAM_BASE_ADDR

    extern void ble_clk_init(void);
    extern void ble_init(void);
    extern void set_ble_CLKN(UINT32);
    extern void ble_set_rx_half_win_size(ble_packet_rate, UINT32);
    extern void ble_set_ET_prefetch_en(UINT32);
    extern void ble_reg_reset(void);
    extern void ble_timing_gen_reset(void);
    extern void ble_core_reset(void);
    extern void ble_set_adv_report_policy(ble_adv_report_policy);
    extern void ble_rf_init(void);
    extern void ble_ExtRC_init(void);
    extern void LE_ET(UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32);
    extern void LE_PPT_AD(UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32);
    extern void LE_PPT_DA(UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32, UINT32);
    extern void LE_PPR(UINT32, UINT32, UINT32, UINT32);

void LE_PC_LL(
    UINT32 cs_offset
    ,UINT16 FORMAT
    ,UINT16 DNABORT
    ,UINT16 RXBSY_EN
    ,UINT16 TXBSY_EN
    ,UINT16 PRIV_NPUB
    ,UINT16 RXCRYPT_EN
    ,UINT16 TXCRYPT_EN
    ,UINT16 CRYPT_MODE
    ,UINT16 MIC_MODE
    ,UINT16 NULLRXLLIDFLT
    ,UINT16 LINKBLB
    ,UINT16 ISOTYPE
    ,UINT16 ISOSYNCEN
    ,UINT16 ISOSYNCMODE
    ,UINT16 GROUP_LBL
    ,UINT16 STREAM_LBL
    ,UINT16 TXRATE
    ,UINT16 RXRATE
    ,UINT16 TXTHR
    ,UINT16 RXTHR
    ,UINT32 *BDADDR   
    ,UINT32 SYNCWORD
    ,UINT32 CRCINIT
    ,UINT16 MAXCTEBUFF         /*RXMAXCTEBUFF*/
    ,UINT16 CH_IDX
    ,UINT16 HOPINT
    ,UINT16 HOP_MODE
    ,UINT16 FH_EN
    ,UINT16 TXPWR
    ,UINT16 EXT_PA_EN
    ,UINT16 NESN
    ,UINT16 SN
    ,UINT16 LASTEMPTY
    ,UINT16 DFEN
    ,UINT16 DFFILTEREN
    ,UINT16 DFTYPE
    ,UINT16 DFSAMPCNTL
    ,UINT16 DFSWCNTL
    ,UINT16 DFRSPEN
    ,UINT16 RXWINSZ
    ,UINT16 RXWIDE
    ,UINT16 TXISODESCPTR
    ,UINT16 RXISODESCPTR
    ,UINT16 ACLTXDESCPTR
    ,UINT16 ANT_PATT_LENGTH   /*RX_ANT_PATT_LENGTH*/
    ,UINT16 MAXSAMPLE_CTE     
    ,UINT16 ANTENNA_ID_PTR
    ,UINT16 DFRSP
    ,UINT16 MINEVTIME
    ,UINT16 MAXEVTIME
    ,UINT32 *LLCHMAP           /*32 bit LLCHMAP_M LLCHMAP_H*/
    ,UINT16 RXMAXBUF
    ,UINT16 ISORXMAXBUF
    ,UINT16 RXMAXTIME
    ,UINT32 *SK                /*SK7~0*/
    ,UINT32 *IV                /*IV0~3*/
    ,UINT32 *TXCCMPKTCNT
    ,UINT32 *RXCCMPKTCNT
    ,UINT16 EVENT_CNT
    ,UINT32 *EVENT_CNT_OFFSET
    ,UINT16 SUBEVTCNT
    ,UINT16 FLUSHCNT
    ,UINT16 ISOCIE
    ,UINT16 ISOSN
    ,UINT16 ISONESN
    ,UINT16 ISOWAITACK
    ,UINT16 ISOLASTCIE
    ,UINT16 ISOLASTNESN
    ,UINT16 ISOLASTSN
    ,UINT16 PRNRETX
    ,UINT16 RETXMAPIDX
    ,UINT16 HOP_SEQ_PTR
);
//
extern void LE_PC_AD(
    UINT32 cs_offset,
    UINT32 format,
    UINT32 dnabort,
    UINT32 rxbsy_en,
    UINT32 txbsy_en,
    UINT32 linklbl,
    UINT32 priv_npub,
    UINT32 txrate,
    UINT32 rxrate,
    UINT32 auxrate,
    UINT32 txthr,
    UINT32 rxthr,
    UINT32* bdaddr,
    UINT32 syncword,
    UINT32 crcinit,
    UINT32 ral_en,
    UINT32 ral_mode,
    UINT32 local_rpa_sel,
    UINT32 peradv_filt_en,
    UINT32 ral_resolution_enable,
    UINT32 filter_policy,
    UINT32 ch_idx,
    UINT32 hopint,
    UINT32 hop_mode,
    UINT32 fh_en,
    UINT32 txpwr,
    UINT32 ext_pa_en,
    UINT32 rxwinsz,
    UINT32 rxwide,
    UINT32 acltxdescptr,
    UINT32 winoffset,
    UINT32 maxevtime,
    UINT32* llchmap,
    UINT32 advchmap,
    UINT32 ch_aux,
    UINT32 max_chain_rxbytes,
    UINT32 max_chain_rxdesc,
    UINT32 rxmaxbuf,
    UINT32 rxmaxtime,
    UINT32* adv_bd_addr,
    UINT32 auxtxdescptr,
    UINT32 winoffset_2m,
    UINT32 conninterval_2m,
    UINT32 winoffset_coded,
    UINT32 conninterval_coded,
    UINT32 prev_adv_pkt_type,
    UINT32 prev_adv_mode,
    UINT32 prev_lam,
    UINT32 prev_pam,
    UINT32 prev_cte,
    UINT32 event_cnt,
    UINT32 dfen,
    UINT32 dffilteren,
    UINT32 dftype,
    UINT32 dfsampcntl,
    UINT32 dfswcntl,
    UINT32 dfrspen,
    UINT32 dfrsp,
    UINT32 maxctebuff,
    UINT32 maxsample_cte,
    UINT32 ant_patt_length,
    UINT32 antenna_id_ptr
);

extern void LE_SL(UINT32 mode,
                  UINT32 sleep_duration,
                  UINT32 twosc,
                  UINT32 twrm,
                  UINT32 extwkupdsb 
); 

extern void set_sleep_correction(); 

//TEST functions
extern UINT8* get_rx_data_ptr(RxDescriptor*);
extern UINT32 get_rx_data_len(RxDescriptor*);
extern void ble_2chip_init(void);
extern void set_rf_mode(void);
extern void ble_deepsleep_clear(void);
extern uint32_t ble_deepsleep_statusget(void);

extern void ble_sleep_enter(void);
#endif

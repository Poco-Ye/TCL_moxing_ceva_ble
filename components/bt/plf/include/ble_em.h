//--------------------------------------------------------------------
// Copyright (c) 2021 by MooreSilicon.
// All rights reserved.
// MooreSilicon Confidential Proprietary.
//--------------------------------------------------------------------
// Project name: Bt_soc
//    File name: ./ble_em.h
//       Author: zhiy
//        Dates: 2021-11-02 10:32:42
//      Version: V1.0
//-------------------------------------------------------------------
//      Purpose:  
//
//-------------------------------------------------------------------
#ifndef BLE_EM__H
#define BLE_EM__H

//exchange table
typedef struct {
    UINT32 MODE          :              3;
    UINT32 STATUS        :              3;
    UINT32 ISO           :              1;
    UINT32 RSVD          :              1;
    UINT32 AE_NPS        :              1;
    UINT32 SIC           :              1;
    UINT32 SPA           :              1;
    UINT32 SCH_PRIO1     :              5;
    UINT32 RAWSTP0       :             16;
    UINT32 RAWSTP1       :             12;
    UINT32 reserved_0    :              4;
    UINT32 FINESTP       :             10;
    UINT32 reserved_1    :              6;
    UINT32 CSPTR         :             14;
    UINT32 reserved_2    :              2;
    UINT32 PRIOD         :             15;
    UINT32 PRIOD_UNIT    :              1;
    UINT32 SCH_PRIO2     :              5;
    UINT32 reserved_3    :              3;
    UINT32 SCH_PRIO3     :              5;
    UINT32 reserved_4    :              3;
    UINT32 reserved_5    :              8;
    UINT32 PTI_PRIO      :              5;
    UINT32 reserved_6    :              3;
} ExTable;
    
typedef struct {
    //0x00 CXCNTL/FRCNTL
    struct{
        UINT16 FORMAT          :      5;
        UINT16 reserved_0      :      3;
        UINT16 DNABORT         :      1;
        UINT16 RXBSY_EN        :      1;
        UINT16 TXBSY_EN        :      1;
        UINT16 reserved_1      :      5;
    } field_CXCNTL_FRCNTL;
    //0X02 LINKCNTL
    struct{
        UINT16 PRIV_NPUB       :      1;
        UINT16 RXCRYPT_EN      :      1;
        UINT16 TXCRYPT_EN      :      1;
        UINT16 CRYPT_MODE      :      1;
        UINT16 MIC_MODE        :      1;
        UINT16 NULLRXLLIDFLT   :      1;
        UINT16 SAS             :      1;
        UINT16 reserved_2      :      1;
        UINT16 LINKLBL         :      5;
        UINT16 reserved_3      :      1;
        UINT16 HP_LP_MODE      :      2;
    } field_LINKCNTL;
    //0x04 ISOLINKCNTL
    struct{
        UINT16 ISOTYPE         :      2;
        UINT16 ISOSYNCEN       :      1;
        UINT16 ISOSYNCMODE     :      1;
        UINT16 reserved_4      :      4;
        UINT16 GROUP_LBL       :      3;
        UINT16 STREAM_LBL      :      5;
    } field_ISOLINKCNTL;
    //0x06 THRCNTL_RATECNTL
    struct{
        UINT16 TXRATE          :      2;
        UINT16 RXRATE          :      2;
        UINT16 AUXRATE         :      2;
        UINT16 reserved_5      :      2;
        UINT16 TXTHR           :      4;
        UINT16 RXTHR           :      4;
    } field_THRCNTL_RATECNTL;
    //0x08 BDADDRL
    struct{
        UINT16 BDADDRL         :     16;
    } field_BDADDRL;
    //0x0A BDADDRM
    struct{
        UINT16 BDADDRM         :     16;
    } field_BDADDRM;
    //0x0C BDADDRU   
    struct{    
        UINT16 BDADDRU         :     16;
    } field_BDADDRU;
    //0x0E SYNCWL  
    struct{
        UINT16 SYNCWORDL       :     16;
    } field_SYNCWORDL;
    //0x10 SYNCWH
    struct{
        UINT16 SYNCWORDH       :     16;
    } field_SYNCWORDH;
    //0x12 CRCINIT0
    struct{
        UINT16 CRCINITL        :     16;
    } field_CRCINIT0;
    //0x14 CRCINIT1/RXMAXCTEBUF
    struct{
        UINT16 CRCINITH        :      8;
        UINT16 RXMAXCTEBUF     :      8;
    } field_CRCINIT1_RXMAXCTEBUF;
    //0x16 FLTPOL/RALCNTL
    struct{
        UINT16 RAL_EN          :      1;
        UINT16 RAL_MODE        :      1;
        UINT16 LOCAL_RPA_SEL   :      1;
        UINT16 PERADV_FILT_EN  :      1;
        UINT16 RAL_RESOL_EN    :      1;
        UINT16 reserved_6      :      3;
        UINT16 FILTER_POLICY   :      8;
    } field_FLTPOL_RALCNTL;
    //0x18 HOPCNTL
    struct{
        UINT16 CH_IDX          :      6;
        UINT16 reserved_7      :      2;
        UINT16 HOPINT          :      5;
        UINT16 HOP_MODE        :      2;
        UINT16 FH_EN           :      1;
    } field_HOPCNTL;
    //0x1A TXRXCNTL
    struct{
        UINT16 TXPWR           :      8;
        UINT16 EXT_PA_EN       :      1;
        UINT16 ENDS_ON_SAC     :      1;
        UINT16 RXMAFSERR       :      1;
        UINT16 RXBFMICERR      :      1;
        UINT16 NESN            :      1;
        UINT16 SN              :      1;
        UINT16 LASTEMPTY       :      1;
        UINT16 RXBUFF_FULL     :      1;
    } field_TXRXCNTL;   
    //0x1C RXDFCNTL
    struct{
        UINT16 DFEN            :      1;
        UINT16 DFFILTEREN      :      1;
        UINT16 DFTYPE          :      2;
        UINT16 DFSAMPCNTL      :      2;
        UINT16 DFSWCNTL        :      2;
        UINT16 DFRSPEN         :      1;
        UINT16 reserved_8      :      7;
    } field_RXDFCNTL;
    //0x1E RXWINCNTL
    struct{
        UINT16 RXWINSZ         :     15;
        UINT16 RXWIDE          :      1;
    } field_RXWINCNTL;
    //0x20 ISOTXDESCPTR
    struct{
        UINT16 ISOTXDESCPTR    :     14;
        UINT16 reserved_9      :      2;
    } field_ISOTXDESCPTR;
    //0x22 ISORXDESCPTR
    struct{
        UINT16 ISORXDESCPTR    :     14;
        UINT16 reserved_10     :      2;
    } field_ISORXDESCPTR;
    //0x24 ACLTXDESCPTR
    struct{
        UINT16 ACLTXDESCPTR    :     14;
        UINT16 reserved_11     :      2;
    } field_ACLTXDESCPTR;
    //0x26 RXDFANTPATTCNTL
    struct{
        UINT16 RX_ANT_PATT_LENGTH  :         7;
        UINT16 reserved_12         :         1;
        UINT16 MAX_SAMP_CTE        :         5;
        UINT16 reserved_13         :         2;
        UINT16 DFRSP               :         1;
    } field_RXDFANTPATTCNTL;
    //0x28 RXDFANTSWPTR
    struct{
        UINT16 RX_ANTENNA_ID_PTR   :        14;
        UINT16 reserved_14         :         2;
    } field_RXDFANTSWPTR;
    //0x2A TXDFANTPATTCNTL
    struct{
        UINT16 TX_ANT_PATT_LENGTH  :         7;
        UINT16 reserved_15         :         6;
    } field_TXDFANTPATTCNTL;
    //0x2C TXDFANTSWPTR
    struct{
        UINT16 TX_ANTENNA_ID_PTR   :        14;
        UINT16 reserved_16         :         2;
    } field_TXDFANTSWPTR;
    //0x2E 
    union{
        struct{
            UINT16 WINOFFSET :       16;
        } field_WINOFFSET;
        struct{
            UINT16 MINEVTIME :       16;
        } field_MINEVTIME;
    };
    //0x30 MAXEVTIME
    struct{
        UINT16 MAXEVTIME     :        16;
    } field_MAXEVTIME;
    //0x32
    union{
        struct{
            UINT16 CONNINTERVAL :    16;
        } field_CONNINTERVAL;
        struct{
            UINT16 LLCHMAP0   :      16; 
        } field_CHMAP0;
    };
    //0x34 CHMAP1
    struct{
        UINT16 LLCHMAP1      :      16;
    } field_CHMAP1;
    //0x36 CHMAP2
    struct{
        UINT16 LLCHMAP2      :       5;
        UINT16 ADVCHMAP      :       3;
        UINT16 reserved_17   :       2;
        UINT16 CH_AUX        :       6;
    } field_CHMAP2;
    //0x38 
    union{
        struct{
            UINT16 MAXCHRXBYTE  : 11;
            UINT16 MAXCHRXDESC  :  5;
        } field_RXMAXAUXCHAIN;
        struct{
            UINT16 ACLRXMAXBUF :  8;
            UINT16 ISORXMAXBUF  :  8;
        } field_RXMAXBUF;
    };
    //0x3A RXMAXTIME
    struct{
        UINT16 RXMAXTIME     :      13;
        UINT16 reserved_18   :       3;
    } field_RXMAXTIME;
    //0x3C
    union{
        struct{
            UINT16 SK0           :    16;
        } field_SK0;
        struct{
            UINT16 ADV_BD_ADDR0  :    16;
        } field_ADV_BD_ADDR0;
        struct{
            UINT16 PEER_RALPTR :  14;
            UINT16 reserved_19 :   2;
        } field_PEER_PALPTR;
    };
    //0x3E
    union{
        struct{
            UINT16 SK1           :    16;
        } field_SK1;
        struct{
            UINT16 ADV_BD_ADDR1  :    16;
        } field_ADV_BD_ADDR1;
    };
    //0x40
    union{
        struct{
            UINT16 SK2           :    16;
        } field_SK2;
        struct{
            UINT16 ADV_BD_ADDR2  :    16;
        } field_ADV_BD_ADDR2;
    };
    //0x42
    union{
        struct{
            UINT16 SK3              :    16;
        } field_SK3;
        struct{
            UINT16 ADV_BD_ADDR3 :     1;
            UINT16 reserved_20  :    15;
        } field_ADV_BD_ADDR3;
    };
    //0x44
    union{
        struct{
            UINT16 SK4           :    16;
        } field_SK4;
        struct{
            UINT16 AUXTXDESCPTR:  14;
            UINT16 reserved_21 :   2;
        } field_AUXTXDESCPTR;
    };
    //0x46
    union{
        struct{
            UINT16 SK5           :    16;
        } field_SK5;
        struct{
            UINT16 WINOFFSET_2M  :    16;
        } field_WINOFFSET_2M;
    };
    //0x48
    union{
        struct{
            UINT16 SK6           :    16;
        } field_SK6;
        struct{
            UINT16 CONNINTERVAL_2M:   16;
        } field_CONNINTERVAL_2M;
    };
    //0x4A
    union{
        struct{
            UINT16 SK7           :    16;
        } field_SK7;
        struct{
            UINT16 WINOFFSET_LR  :    16;   
        } field_WINOFFSET_LR;
    };
    //0x4C
    union{
        struct{
            UINT16 IV0           :    16;
        } field_IV0;
        struct{
            UINT16 CONNINTERVAL_LR:   16;
        } field_CONNINTERVAL_LR;
    };
    //0x4E IV1
    struct{
        UINT16 IV1               :    16;
    } field_IV1;
    //0x50 IV2
    struct{
        UINT16 IV2               :    16;
    } field_IV2;
    //0x52 IV3
    struct{
        UINT16 IV3               :    16;
    } field_IV3;
    //0x54
    union{
        struct{
            UINT16 TXWINOFFSET   :    16;
        } field_TXWINOFFSET;
        struct{
            UINT16 TXCCMPKTCNT0   :    16;
        } field_TXCCMPKTCNT0;
    };
    //0x56
    union{
        struct{
            UINT16 PREV_ADV_PKT_TYPE: 3;
            UINT16 PREV_ADV_MODE    : 2;
            UINT16 PREV_LAM         : 1;
            UINT16 PREV_PAM         : 1;
            UINT16 PREV_CTE         : 1;
            UINT16 reserved_22      : 8;
        } field_EXTADVSTAT;
        struct{
            UINT16 TXCCMPKTCNT1   :    16;
        } field_TXCCMPKTCNT1;
    };
    //0x58 TXCCMPKTCNT2
    struct{
        UINT16 TXCCMPKTCNT2       :     7;
        UINT16 reserved_23        :     9;
    } field_TXCCMPKTCNT2;
    //0x5A RXCCMPKTCNT0 
    struct{
        UINT16 RXCCMPKTCNT0       :     16;
    } field_RXCCMPKTCNT0;
    //0x5C RXCCMPKTCNT1      
    struct{        
        UINT16 RXCCMPKTCNT1       :     16;
    } field_RXCCMPKTCNT1;
    //0x5E RXCCMPKTCNT2
    struct{ 
        UINT16 RXCCMPKTCNT2       :      7;
        UINT16 reserved_24        :      9;
    } field_RXCCMPKTCNT2;
    //0x60 EVTCNT
    struct{
        UINT16 EVENT_CNT          :     16;
    } field_EVTCNT;
    //0x62
    union{
        struct{
            UINT16 EVENT_CNT_OFFSET0:    16;
        } field_EVTCNTOFFSET0;
        struct{
            UINT16 NSE         :     8;
            UINT16 reserved_25 :     8;
        } field_NSE;
    };
    //0x64 EVTCNTOFFSET1
    struct{
        UINT16 EVENT_CNT_OFFSET1   :    16;
    } field_EVTCNTOFFSET1;
    //0x66 EVTCNTOFFSET2
    struct{
        UINT16 EVENT_CNT_OFFSET2   :     7;
        UINT16 reserved_26         :     9;
    } field_EVTCNTOFFSET2;
    //0x68 ISOEVTCNTL
    struct{
        UINT16 SUBEVTCNT           :     8;
        UINT16 FLUSHCNT            :     8;
    } field_ISOEVTCNTL;
    //0x6A ISOTXRXCNTL
    struct{
        UINT16 ISOLASTSN           :     1;
        UINT16 ISOLASTNESN         :     1;
        UINT16 ISOLASTCIE          :     1;
        UINT16 ISOWAITACK          :     1;
        UINT16 reserved_27         :     4;
        UINT16 ISORSVD             :     1;
        UINT16 ISORETX             :     1;
        UINT16 reserved_28         :     1;
        UINT16 ISONESN             :     1;
        UINT16 ISOSN               :     1;
        UINT16 ISOCIE              :     1;
        UINT16 reserved_29         :     1;
        UINT16 ISORXBUFF_FULL      :     1;
    } field_ISOTXRXCNTL;
    //0x6C TXRXDESCCNT
    struct{
        UINT16 ACLTXDESCCNT        :     8;
        UINT16 ACLRXDESCCNT        :     8;
    } field_TXRXDESCCNT;
    //0x6E ISOTXRXPKCNT
    struct{
        UINT16 ISOTXPKTCNT         :     8;
        UINT16 ISORXPKTCNT         :     8;
    } field_ISOTXRXPKCNT;
    //0x70 COUNTER0LSB
    struct{
        UINT16 COUNTER0L           :    16;
    } field_COUNTER0LSB;
    //0x72 COUNTER0MSB   
    struct{ 
        UINT16 COUNTER0H           :    16;
    } field_COUNTER0MSB;
    //0x74 COUNTER1LSB
    struct{
        UINT16 COUNTER1L           :    16;
    } field_COUNTER1LSB;
    //0x76 COUNTER1MSB   
    struct{ 
        UINT16 COUNTER1H           :    16;
    } field_COUNTER1MSB;
    //0x78 COUNTER2LSB
    struct{
        UINT16 COUNTER2L           :    16;
    } field_COUNTER2LSB;
    //0x7A COUNTER2MSB  
    struct{  
        UINT16 COUNTER2H           :    16;
    } field_COUNTER2MSB;
    //0x7C COUNTER3LSB
    struct{
        UINT16 COUNTER3L           :    16;
    } field_COUNTER3LSB;
    //0x7E COUNTER3MSB   
    struct{ 
        UINT16 COUNTER3H           :    16;
    } field_COUNTER3MSB;
    //0x80 COUNTER4LSB
    struct{
        UINT16 COUNTER4L           :    16;
    } field_COUNTER4LSB;
    //0x82 COUNTER4MSB   
    struct{ 
        UINT16 COUNTER4H           :    16;
    } field_COUNTER4MSB;
    //0x84 COUNTER5LSB
    struct{
        UINT16 COUNTER5L           :    16;
    } field_COUNTER5LSB;
    //0x86 COUNTER5MSB   
    struct{ 
        UINT16 COUNTER5H           :    16;
    } field_COUNTER5MSB;
    //0x88 COUNTER6LSB
    struct{
        UINT16 COUNTER6L           :    16;
    } field_COUNTER6LSB;
    //0x8A COUNTER6MSB   
    struct{ 
        UINT16 COUNTER6H           :    16;
    } field_COUNTER6MSB;
    //0x8C HOPCS2CNTL0
    struct{
        UINT16 PRNRETX             :    16;
    } field_HOPCS2CNTL0;
    //0x8E HOPCS2CNTL1
    struct{
        UINT16 RETXMAPIDX          :    16;
    } field_HOPCS2CNTL1;
    //0x90 HOPPTR    
    struct{   
        UINT16 HOP_SEQ_PTR         :    14;
        UINT16 reserved_30         :     2;
    } field_HOPPTR;

} CtrlStruct;

typedef struct {
    UINT16 NEXTPTR             :    14;
    UINT16 reserved_0          :     1;
    UINT16 TXDONE              :     1;
} field_TXPTR;

typedef struct {
    UINT16 TXLLID              :     2;
    UINT16 TXNESN              :     1;
    UINT16 TXSN                :     1;
    UINT16 TXMD                :     1;
    UINT16 TXCP                :     1;
    UINT16 reserved_0          :     2;
    UINT16 TXLEN               :     8;
} field_TXPHCE;

typedef struct {
    UINT16 TXTYPE0             :     4;
    UINT16 reserved_0          :     1;
    UINT16 TXCHSEL             :     1;
    UINT16 TXTXADD             :     1;
    UINT16 TXRXADD             :     1;
    UINT16 TXADVLEN            :     8;
} field_TXPHADV;

typedef struct {
    UINT16 TXDATAPTR           :    16;
} field_TXDATAPTR;

typedef struct {
    UINT16 TXAELENGTH          :     6;
    UINT16 TXAEMODE            :     2;
    UINT16 TXADVA              :     1;
    UINT16 TXTGTA              :     1;
    UINT16 TXCTE               :     1;
    UINT16 TXADI               :     1;
    UINT16 TXAUXPTR            :     1;
    UINT16 TXSYNC              :     1;
    UINT16 TXPOW               :     1;
    UINT16 reserved_0          :     1;
} field_TXAEHEADER;

typedef struct {
    UINT16 TX_LL_CH            :     6;
    UINT16 TXAUX_CA            :     1;
    UINT16 TXAUXOFFSET_UNIT    :     1;
    UINT16 TXAUXOFFSET         :     8;
} field_TXAUXPTR0;

typedef struct {
    UINT16 TXAUXOFFSET         :     5;
    UINT16 TXAUX_PHY           :     3;
    UINT16 reserved_0          :     8;
} field_TXAUXPTR1;

typedef struct {
    UINT16 TXAEHEADER_DATAPTR  :    16;
} field_TXAEDATAPTR;

typedef struct {
    UINT16 TXCTETIME           :     5;
    UINT16 reserved_0          :     1;
    UINT16 TXCTETYPE           :     2;
    UINT16 reserved_1          :     8;
} field_TXPHCTE;

typedef struct {
    //0x00
    field_TXPTR             TXPTR;
    //0x02
    union {
        field_TXPHCE             TXPHCE;
        field_TXPHADV            TXPHADV;
    };
    //0x04
    field_TXDATAPTR         TXDATAPTR;
    //0x06
    field_TXAEHEADER        TXAEHEADER;
    //0x08
    field_TXAUXPTR0         TXAUXPTR0;
    //0x0A
    field_TXAUXPTR1         TXAUXPTR1;
    //0x0C
    field_TXAEDATAPTR       TXAEDATAPTR;
    //0x0E
    field_TXPHCTE           TXPHCTE;
} TxDescriptor;

typedef struct {
    //0x02
    UINT16 SYNCERR             :     1;
    UINT16 RXTIMEERR           :     1;
    UINT16 LENERR              :     1;
    UINT16 CRCERR              :     1;
    UINT16 MICERR              :     1;
    UINT16 LLIDERR             :     1;
    UINT16 SNERR               :     1;
    UINT16 NESNERR             :     1;
    UINT16 reserved_0          :     7;
    UINT16 RXCTEERR            :     1;
} field_RXSTATCE;

typedef struct {
    //0x02
    UINT16 SYNCERR             :     1;
    UINT16 RXTIMEERR           :     1;
    UINT16 LENERR              :     1;
    UINT16 CRCERR              :     1;
    UINT16 PRIVERR             :     1;
    UINT16 TYPEERR             :     1;
    UINT16 BDADDR_MATCH        :     1;
    UINT16 PEER_ADD_MATCH      :     1;
    UINT16 IN_PERADVL          :     1;
    UINT16 IN_WHL              :     1;
    UINT16 DEV_FILTERING_OK    :     1;
    UINT16 ADVMODEERR          :     1;
    UINT16 FOLLOWAUXPTR        :     1;
    UINT16 reserved_0          :     2;
    UINT16 RXCTEERR            :     1;
} field_RXSTATADV;

typedef struct {
    //0x02
    UINT16 SYNCERR             :     1;
    UINT16 RXTIMEERR           :     1;
    UINT16 LENERR              :     1;
    UINT16 CRCERR              :     1;
    UINT16 MICERR              :     1;
    UINT16 LLIDERR             :     1;
    UINT16 SNERR               :     1;
    UINT16 NESNERR             :     1;
    UINT16 RXGROUP_LBL         :     3;
    UINT16 RXSTREAM_LBL        :     5;
} field_RXSTATISOM0;

typedef struct {
    //0x02
    UINT16 SYNCERR             :     1;
    UINT16 RXTIMEERR           :     1;
    UINT16 LENERR              :     1;
    UINT16 CRCERR              :     1;
    UINT16 MICERR              :     1;
    UINT16 LLIDERR             :     1;
    UINT16 SNERR               :     1;
    UINT16 NESNERR             :     1;
    UINT16 RXGROUP_LBL         :     3;
    UINT16 RXSTREAM_LBL        :     5;
} field_RXSTATISOM1;

typedef struct {
    //0x02
    UINT16 SYNCERR             :     1;
    UINT16 RXTIMEERR           :     1;
    UINT16 LENERR              :     1;
    UINT16 CRCERR              :     1;
    UINT16 MICERR              :     1;
    UINT16 reserved_0          :     3;
    UINT16 RXGROUP_LBL         :     3;
    UINT16 RXSTREAM_LBL        :     5;
} field_RXSTATISOM2;

typedef struct {
    //0x04
    UINT16 RXLLID              :     2;
    UINT16 RXNESN              :     1;
    UINT16 RXSN                :     1;
    UINT16 RXMD                :     1;
    UINT16 RXCP                :     1;
    UINT16 reserved_0          :     2;
    UINT16 RXLEN            :     8;
} field_RXPHCE;

typedef struct {
    //0x04
    UINT16 RXTYPE              :     4;
    UINT16 reserved_0          :     1;
    UINT16 RXCHSEL             :     1;
    UINT16 RXTXADD             :     1;
    UINT16 RXRXADD             :     1;
    UINT16 RXADVLEN            :     8;
} field_RXPHADV;

typedef struct {
    //0x04
    UINT16 RXLLID              :     2;
    UINT16 RXNESN              :     1;
    UINT16 RXSN                :     1;
    UINT16 RXMD                :     1;
    UINT16 reserved_0          :     3;
    UINT16 RXLEN               :     8;
} field_RXPHISOM0;

typedef struct {
    //0x04
    UINT16 RXLLID              :     2;
    UINT16 RXNESN              :     1;
    UINT16 RXSN                :     1;
    UINT16 RXMD                :     1;
    UINT16 reserved_0          :     3;
    UINT16 RXLEN               :     8;
} field_RXPHCIS;

typedef struct {
    //0x04
    UINT16 RXLLID              :     2;
    UINT16 RXCSSN              :     3;
    UINT16 RXCSTF              :     1;
    UINT16 reserved_0          :     2;
    UINT16 RXLEN               :     8;
} field_RXPHBIS;

typedef struct {
    UINT16 NEXTPTR             :    14;
    UINT16 reserved_0          :     1;
    UINT16 RXDONE              :     1;
} field_RXPTR;

typedef struct {
    UINT16 RXRSSI              :     8;
    UINT16 USED_CH_IDX         :     6;
    UINT16 RATE                :     2;
} field_RXCHASS;

typedef struct {
    UINT16 RXCLKNSYNC          :    16;
} field_RXCLKNSYNC0;

typedef struct {
    UINT16 RXCLKNSYNC          :    12;
    UINT16 reserved_0          :     4;
} field_RXCLKNSYNC1;

typedef struct {
    UINT16 RXFCNTSYNC          :    10;
    UINT16 reserved_0          :     1;
    UINT16 RXLINKLBL           :     5;
} field_RXFCNTSYNC;

typedef struct {
    UINT16 RXWPALPTR           :    16;
} field_RXWPALPTR;

typedef struct {
    UINT16 RXRALPTR            :    16;
} field_RXRALPTR;

typedef struct {
    UINT16 RXAELENGTH          :     6;
    UINT16 RXAEMODE            :     2;
    UINT16 RXADVA              :     1;
    UINT16 RXTGTA              :     1;
    UINT16 RXCTE               :     1;
    UINT16 RXADI               :     1;
    UINT16 RXAUXPTR            :     1;
    UINT16 RXSYNC              :     1;
    UINT16 RXPOW               :     1;
    UINT16 RXRSVD              :     1;
} field_RXAEHEADER;

typedef struct {
    UINT16 RXDATAPTR           :    16;
} field_RXDATAPTR;

typedef struct {
    UINT16 RXCTETIME           :     5;
    UINT16 reserved_0          :     1;
    UINT16 RXCTETYPE           :     2;
    UINT16 NBRXIQSAMP          :     8;
} field_RXPHCTE;

typedef struct {
    UINT16 RXCTEPTR            :    16;
} field_RXCTEPTR;


typedef struct {
    //0x00 RXPTR
    field_RXPTR           RXPTR;
    //0x02
    union {
        field_RXSTATCE        RXSTATCE;
        field_RXSTATADV       RXSTATADV;
        field_RXSTATISOM0     RXSTATISOM0;
        field_RXSTATISOM1     RXSTATISOM1;
        field_RXSTATISOM2     RXSTATISOM2;
    };
    //0x4
    union {
        field_RXPHCE          RXPHCE;
        field_RXPHADV         RXPHADV;
        field_RXPHISOM0       RXPHISOM0;
        field_RXPHCIS         RXPHCIS;
        field_RXPHBIS         RXPHBIS;
    };
    //0x06
    field_RXCHASS         RXCHASS;
    //0x08
    field_RXCLKNSYNC0     RXCLKNSYNC0;
    //0x0A
    field_RXCLKNSYNC1     RXCLKNSYNC1;
    //0x0C
    field_RXFCNTSYNC      RXFCNTSYNC;
    //0x0E
    field_RXWPALPTR       RXWPALPTR;
    //0x10
    field_RXRALPTR        RXRALPTR;
    //0x12
    field_RXAEHEADER      RXAEHEADER;
    //0x14
    field_RXDATAPTR       RXDATAPTR;
    //0x16
    field_RXPHCTE         RXPHCTE;
    //0x18
    field_RXCTEPTR        RXCTEPTR;
} RxDescriptor;

#endif

#pragma once

#pragma pack(push, 1)

typedef struct EM_ET
{
	// EXTAB
	UINT16 MODE : 3;
	UINT16 STATUS : 3;
	UINT16 ISO : 1;
	UINT16 RSVD : 1;
	UINT16 AE_NPS : 1;
	UINT16 SIC : 1;
	UINT16 SPA : 1;
	UINT16 SCH_PRIO : 5;

	// RAWSTP
	UINT32 RAWSTP : 28;
	UINT32 TBD_RAWSTP : 4;

	// FINESTP
	UINT16 FINESTP : 10;
	UINT16 TBD_FINESTP : 4;

	// CSPTR
	UINT16 CSPTR : 14;
	UINT16 TBD_CSPTR : 2;

	// PRIOBW
	UINT16 PRIO1D : 15;
	UINT16 PRIO1D_UNIT : 1;

	// PRIOLVL
	UINT16 SCH_PRIO2: 5;
	UINT16 TBD_PRIOLVL : 3;
	UINT16 SCH_PRIO3 : 5;
	UINT16 TBD_PRIOLVL2 : 3;

	// PRIOPTI
	UINT16 TBD_PRIOPTI : 8;
	UINT16 PTI_PRI : 5;
	UINT16 TBD_PRIOPTI2 : 3;
} *PEM_ET;

typedef struct EM_CS
{
	// CNTL
	UINT16 FORMAT : 5;
	UINT16 TBD_CNTL : 3;
	UINT16 DNABORT : 1;
	UINT16 RXBSY_EN : 1;
	UINT16 TXBSY_EN : 1;
	UINT16 TBD_CNTL2 : 5;

	// LINKCNTL
	UINT16 PRIV_NPUB : 1;
	UINT16 RXCRYPT_EN : 1;
	UINT16 TXCRYPT_EN : 1;
	UINT16 CRYPT_MODE : 1;
	UINT16 MIC_MODE : 1;
	UINT16 NULLRXLLIDFLT : 1;
	UINT16 SAS : 1;
	UINT16 TBD_LINKCNTL : 1;
	UINT16 LINKLBL : 5;
	UINT16 TBD_LINKCNTL2 : 1;
	UINT16 HP_LP_MODE : 2;

	// ISOLINKCNTL
	UINT16 ISOTYPE : 2;
	UINT16 ISOSYNCEN : 1;
	UINT16 ISOSYNCMODE : 1;
	UINT16 TBD_ISOLINKCNTL : 4;
	UINT16 GROUP_LBL : 3;
	UINT16 STREAM_LBL : 5;

	// THRCNTL_RATECNTL
	UINT16 TXRATE : 2;
	UINT16 RXRATE : 2;
	UINT16 AUXRATE : 2;
	UINT16 TBD_THRCNTL_RATECNTL : 2;
	UINT16 TXTHR: 4;
	UINT16 RXTHR: 4;

	// BDADDRL/M/U
	UINT8 BDADDR[6];
	
	// SYNCWL/H
	UINT32 SYNCW;

	// CRCINIT0/1/RXMAXCTEBUF
	UINT32 CRCINIT : 24;
	UINT32 RXMAXCTEBUF : 8;

	// FILTPOL_RALCNTL
	UINT8 RAL_EN : 1;
	UINT8 RAL_MODE : 1;
	UINT8 LOCAL_RPA_SEL : 1;
	UINT8 PERADV_FILT_EN : 1;
	UINT8 RAL_RESOL_EN : 1;
	UINT8 TBD_RALCNTL : 1;
	UINT8 FILTER_POLICY;

	// HOPCNTL
	UINT16 CH_IDX: 6;
	UINT16 TBD_HOPCNTL : 2;
	UINT16 HOP_MODE : 2;
	UINT16 FH_EN : 1;

	// TXRXCNTL
	UINT8 TXPWR;
	UINT8 EXT_PA_EN : 1;
	UINT8 ENDS_ON_SAC : 1;
	UINT8 RXMAFSERR : 1;
	UINT8 NESN : 1;
	UINT8 SN : 1;
	UINT8 LASTEMPTY : 1;
	UINT8 RXBUFF_FULL : 1;

	// RXDFCNTL
	UINT16 DFEN : 1;
	UINT16 DFFILTEREN : 1;
	UINT16 DFTYPE : 2;
	UINT16 DFSAMPCNTL : 2;
	UINT16 DFSWCNTL : 2;
	UINT16 DFRSPEN : 1;
	UINT16 TBD_RXDFCNTL : 7;

	// RXWINCNTL
	UINT16 RXWINSZ : 15;
	UINT16 RXWIDE : 1;

	// ISOTXDESCPTR
	UINT16 ISOTXDESCPTR : 14;
	UINT16 TBD_ISOTXDESCPTR : 2;

	// ISORXDESCPTR
	UINT16 ISORXDESCPTR : 14;
	UINT16 TBD_ISORXDESCPTR : 2;

	// ACLTXDESCPTR
	UINT16 ACLTXDESCPTR : 14;
	UINT16 TBD_ACLTXDESCPTR : 2;

	// RXDFANTPATTCNTL
	UINT16 RX_ANT_PATT_LENGTH : 7;
	UINT16 TBD_RXDFANTPATTCNTL : 1;
	UINT16 MAX_SAMP_CTE : 5;
	UINT16 TBD_RXDFANTPATTCNTL2 : 2;
	UINT16 DFRSP : 1;

	// RXDFANTSWPTR
	UINT16 RX_ANTENNA_ID_PTR : 14;
	UINT16 TBD_RXDFANTSWPTR : 2;

	// TXDFANTPATTCNTL
	UINT16 TX_ANT_PATT_LENGTH : 7;
	UINT16 TBD_TXDFANTPATTCNTL : 9;

	// TXDFANTSWPTR
	UINT16 TX_ANTENNA_ID_PTR : 14;
	UINT16 TBD_TXDFANTSWPTR : 2;

	union 
	{
		UINT16 WINOFFSET;
		UINT16 MINEVTIME;
	};

	UINT16 MAXEVTIME;

	// CHMAP0/1/2
	union
	{
		UINT16 CONNINTERVAL;
		UINT16 LLCHMAP0;
	};
	UINT16 LLCHMAP1;
	UINT16 LLCHMAP2 : 5;
	UINT16 ADVCHMAP : 3;
	UINT16 TBD_CHMAP2 : 2;
	UINT16 CH_AUX : 6;

	union
	{
		struct
		{
			// RXMAXAUXCHAIN
			UINT16 MAXCHRXBYTE : 11;
			UINT16 MAXCHRXDESC : 5;
		};
		struct
		{
			UINT8 ACLRXMAXBUFF;
			UINT8 ISORXMAXBUF;
		};
	};

	// RXMAXBUF
	UINT16 RXMAXTIME : 13;
	UINT16 TBD_RXMAXBUF : 3;

	union
	{
		UINT16 SK[8];
		struct
		{
			// ADV_BD_ADDR
			UINT16 ADV_BD_ADDR[3];
			UINT16 ADV_BD_ADDR3 : 1;
			UINT16 TBD_ADV_BD_ADDR3 : 15;

			// AUXTXDESCPTR
			UINT16 AUXTXDESCPTR : 14;
			UINT16 TBD_AUXTXDESCPTR : 2;

			UINT16 WINOFFSET_2M;
			UINT16 CONNINTERVAL_2M;
			UINT16 WINOFFSET_LR;
		};
	};

	union
	{
		UINT16 IV[4];
		UINT16 CONNINTERVAL_LR;
	};

	union
	{
		struct
		{
			UINT16 TXWINOFFSET;

			// EXTADVSTAT
			UINT8 PREV_ADV_PKT_TYPE : 3;
			UINT8 PREV_ADV_MODE : 2;
			UINT8 PREV_LAM : 1;
			UINT8 PREV_PAM : 1;
			UINT8 PREV_CTE : 1;
			UINT8 TBD_EXTADVSTAT;

		};

		struct
		{
			UINT32 TXCCMPKTCNT0;
			UINT16 TXCCMPKTCNT2 : 6;
			UINT16 TBD_TXCCMPKTCNT2 : 2;

		};
	};

	UINT32 RXCCMPKTCNT;
	UINT16 RXCCMPKTCNT2 : 6;
	UINT16 TBD_RXCCMPKTCNT2 : 2;

	// EVTCNT
	UINT16 EVENT_CNT;

	union
	{
		struct
		{
			// EVTCNTOFFSET
			UINT32 EVENT_CNT_OFFSET;
			UINT16 EVENT_CNT_OFFSET2 : 7;
			UINT16 TBD_EVTCNTOFFSET2 : 9;
		};

		UINT8 NSE;
	};

	// ISOEVTCNTL
	UINT8 SUBEVTCNT;
	UINT8 FLUSHCNT;

	// TXRXDESCCNT
	UINT8 ACLTXDESCCNT;
	UINT8 ACLRXDESCCNT;

	// ISOTXRXPKTCNT
	UINT8 ISOTXPKTCNT;
	UINT8 ISORXPKTCNT;


	// ISOTXRXCNTL
	UINT16 ISOLASTSN : 1;
	UINT16 ISOLASTNESN : 1;
	UINT16 ISOLASTCIE : 1;
	UINT16 ISOWAITACK : 1;
	UINT16 TBD_ISOTXRXCNTL : 4;
	UINT16 ISORSVD : 1;
	UINT16 ISORETX : 1;
	UINT16 TBD_ISOTXRXCNTL2 : 1;
	UINT16 ISONESN : 1;
	UINT16 ISOSN : 1;
	UINT16 ISOCIE : 1;
	UINT16 TBD_ISOTXRXCNTL3 : 1;
	UINT16 ISORXBUFF_FULL : 1;

	UINT32 COUNTER[7];

	// HOPCS2CNTL0
	UINT16 PRNRETX;
	// HOPCS2CNTL1
	UINT16 RETXMAPIDX;

	// HOPPTR
	UINT16 HOP_SEQ_PTR : 14;
	UINT16 TBD_HOPPTR : 2;

	// TXHEADERCNTL
	UINT16 ISOM0_RSVD_SN : 2;
	UINT16 ISOM0_RETX_SN : 2;
	UINT16 ISOM0_RSVD_MD : 2;
	UINT16 ISOM0_RETX_MD : 2;
	UINT16 CS_NESN_DSB : 1;
	UINT16 CS_SN_DSB : 1;
	UINT16 CS_MD_DSB : 1;
	UINT16 CS_NPI_DSB : 1;
	UINT16 CS_CIE_DSB : 1;
	UINT16 TBD_TXHEADERCNTL : 3;

} *PEM_CS;

typedef struct EM_TXDESC
{
	// TXPTR
	UINT16 NEXTPTR : 14;
	UINT16 TBD_TXPTR : 1;
	UINT16 TXDONE : 1;

	union
	{
		struct
		{
			// TXPHCE
			UINT8 TXLLID : 2;
			UINT8 TXNESN : 1;
			UINT8 TXSN : 1;
			UINT8 TXMD : 1;
			UINT8 TXCP : 1;
			UINT8 TXACLRFU : 2;
			UINT8 TXLEN;
		};
		struct
		{
			// TXPHADV
			UINT8 TXTYPE0 : 4;
			UINT8 TXADVRFU : 1;
			UINT8 TXCHSEL : 1;
			UINT8 TXTXADD : 1;
			UINT8 TXRXADD : 1;
			UINT8 TXADVLEN;
		};
	};


	UINT16 TXDATAPTR;

	// TXAEHEADER
	UINT16 TXAELENGTH : 6;
	UINT16 TXAEMODE : 2;
	UINT16 TXADVA : 1;
	UINT16 TXTGTA : 1;
	UINT16 TXCTE : 1;
	UINT16 TXADI : 1;
	UINT16 TXAUXPTR : 1;
	UINT16 TXSYNC : 1;
	UINT16 TXPOW : 1;
	UINT16 TXRSVD : 1;

	// TXAUXPTR0/1
	UINT32 TX_LL_CH : 6;
	UINT32 TXAUX_CA : 1;
	UINT32 TXAUXOFFSET_UNIT : 1;
	UINT32 TXAUXOFFSET : 13;
	UINT32 TXAUX_PHY : 3;
	UINT32 TBD_TXAUXPTR : 8;

	// TXAEDATAPTR
	UINT16 TXAEHEADER_DATAPTR;

	// TXPHCTE
	UINT8 TXCTETIME : 5;
	UINT8 TXCTERFU : 1;
	UINT8 TXCTETYPE : 2;
	UINT8 TBD_TXPHCTE;
} *PEM_TXDESC;

typedef struct EM_RXDESC
{
	// RXPTR
	UINT16 NEXTPTR : 14;
	UINT16 TBD_TXPTR : 1;
	UINT16 RXDONE : 1;

	union
	{
		struct
		{
			UINT16 SYNCERR : 1;
			UINT16 RXTIMEERR : 1;
			UINT16 LENERR : 1;
			UINT16 CRCERR : 1;
			UINT16 MICERR : 1;
			UINT16 LLIDERR : 1;
			UINT16 SNERR : 1;
			UINT16 NESNERR : 1;
			UINT16 TBD_RXSTATCE : 7;
			UINT16 RXCTEERR : 1;
		} RXSTATCE;

		struct
		{
			UINT16 SYNCERR : 1;
			UINT16 RXTIMEERR : 1;
			UINT16 LENERR : 1;
			UINT16 CRCERR : 1;
			UINT16 PRIVERR : 1;
			UINT16 TYPEERR : 1;
			UINT16 BDADDR_MATCH : 1;
			UINT16 PEER_ADD_MATCH : 1;
			UINT16 IN_PERADVL : 1;
			UINT16 IN_WHL : 1;
			UINT16 DEV_FILTERING_OK : 1;
			UINT16 ADVMODEERR : 1;
			UINT16 FOLLOWAUXPTR : 1;
			UINT16 TBD_RXSTATADV : 2;
			UINT16 RXCTEERR : 1;
		} RXSTATADV;

		struct
		{
			UINT16 SYNCERR : 1;
			UINT16 RXTIMEERR : 1;
			UINT16 LENERR : 1;
			UINT16 CRCERR : 1;
			UINT16 MICERR : 1;
			UINT16 LLIDERR : 1;
			UINT16 SNERR : 1;
			UINT16 NESNERR : 1;
			UINT16 RXGROUP_LBL : 3;
			UINT16 RXSTREAM_LBL : 5;
		} RXSTATISOM; // 0/1/2
	};

	union
	{
		struct
		{
			UINT16 RXLLID : 2;
			UINT16 RXNESN : 1;
			UINT16 RXSN : 1;
			UINT16 RXMD : 1;
			UINT16 RXCP : 1;
			UINT16 RXACLRFU : 2;
			UINT16 RXLEN : 8;
		} RXPHCE;

		struct
		{
			UINT16 RXTYPE : 4;
			UINT16 RXADVRFU : 1;
			UINT16 RXCHSEL : 1;
			UINT16 RXTXADD : 1;
			UINT16 RXRXADD : 1;
			UINT16 RXADVLEN : 8;

		} RXPHADV;

		struct
		{
			UINT16 RXLLID : 2;
			UINT16 RXNESN : 1;
			UINT16 RXSN : 1;
			UINT16 RXMD : 1;
			UINT16 RXISOM0RFU : 3;
			UINT16 RXLEN : 8;

		} RXPHISOM0;

		struct
		{
			UINT16 RXLLID : 2;
			UINT16 RXNESN : 1;
			UINT16 RXSN : 1;
			UINT16 RXCIE : 1;
			UINT16 RXCISRFU0 : 1;
			UINT16 RXCISNPI : 1;
			UINT16 RXCISRFU1 : 1;
			UINT16 RXLEN : 8;

		} RXPHCIS;

		struct
		{
			UINT16 RXLLID : 2;
			UINT16 RXCSSN : 3;
			UINT16 RXCSTF : 1;
			UINT16 RXBISRFU : 2;
			UINT16 RXLEN : 8;

		} RXPHBIS;

	};

	// RXCHASS
	UINT16 RXRSS : 8;
	UINT16 USED_CH_IDX : 6;
	UINT16 RATE : 2;

	// RXCLKNSYNC
	UINT32 RXCLKNSYNC : 28;
	UINT32 TBD_RXCLKNSYNC : 4;

	// RXFCNTSYNC
	UINT16 RXFCNTSYNC : 10;
	UINT16 TBD_RXFCNTSYNC : 1;
	UINT16 RXLINKLBL : 5;

	UINT16 RXWPALPTR;

	UINT16 RXRALPTR;

	// RXAEHEADER
	UINT16 RXAELENGTH : 6;
	UINT16 RXAEMODE : 2;
	UINT16 RXADVA : 1;
	UINT16 RXTGTA : 1;
	UINT16 RXCTE : 1;
	UINT16 RXADI : 1;
	UINT16 RXAUXPTR : 1;
	UINT16 RXSYNC : 1;
	UINT16 RXPOW : 1;
	UINT16 RXRSVD : 1;

	UINT16 RXDATAPTR;

	// RXPHCTE
	UINT16 RXCTETIME : 5;
	UINT16 RXCTERFU : 1;
	UINT16 RXCTETYPE : 2;
	UINT16 NBRXIQSAMP : 8;

	UINT16 RXCTEPTR;
} *PEM_RXDESC;

#pragma pack(pop)


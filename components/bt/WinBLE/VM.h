#pragma once


void VM_Init();
void VM_Deinit();

#pragma pack(push, 1)
typedef struct PAS_HEAD
{
	UINT8 HeadLength; // 22h
	UINT8 Version; // 03
	UINT8 TBD2_0 : 1;
	UINT8 RfChannel : 7;
	UINT8 TBD2[2];

	UINT8 ReceiveStatus : 4;
	UINT8 TBD5_4 : 2;
	UINT8 DecryptionInitiated : 1;
	UINT8 MeetsPredefinedFilterCriteria : 1;

	UINT8 Encrypt : 1;
	UINT8 TBD6_1 : 3;
	UINT8 PhyType : 2; // 1M, 2M..
	UINT8 Encrypt2 : 1;
	UINT8 TBD6_7 : 1;

	UINT8 TBD7_0 : 6;
	UINT8 ConnectFrame : 1;

	UINT32 AccessAddress;
	INT16 RSSI;
	UINT8 TBD14[5];
	UINT32 AccessAddress2;

	UINT8 Address[6];
	UINT8 AuxType;
	UINT8 TBD30[2];

	UINT16 PduSize;
	// UINT8 Pdu[x];
	// UINT8 CRC[3];
} *PPAS_HEAD;
#pragma pack(pop)

typedef struct PAS_RTX
{
	INT32 nPos; // IN/OUT, -1 when tx 
	PVOID hEvent; // OUT, for RX Wait
	PVOID pHead;
	INT32 cbHead;
	PVOID pPdu;
	INT32 cbPdu;
	UINT32 nPduCRC;
	INT nDrf;
	UINT64 ts;
} *PPAS_RTX;

typedef void (PasRtxFrame)(PPAS_RTX pRtx);

// Frame description
typedef struct VM_FD
{
	UINT8 nType;
	UINT8 RSSI;
	UINT16 nState;
	UINT32 AA;
	UINT8 ch : 8;
	UINT32 nInitCRC : 24;
	UINT64 ts;
} *PVM_FD;


typedef struct REG_PAGEINFO
{
	UINT32 nStartAddr;
	INT nSize;
} *PREG_PAGEINFO;

typedef struct RW_VM
{
	UINT64 nSysTime0;
	UINT64 nTmrClkN;
	UINT64 aTimer[4];
	HANDLE hThread;
	HANDLE hWaitTimer;
	HMODULE hPasDLL;
	PasRtxFrame* pfnPasRtx;
	INT32 nPasPos;
	INT32 cntEtEvent;
	INT32 cntEtEvent2;
	INT32 nEtPos;
} *PRW_VM;

///////////////////////////////////////////////////////////////////////////

typedef struct BLE_ADV
{
	UINT8 PDU_Type : 4;
	UINT8 RFU : 1;
	UINT8 ChSel : 1;
	UINT8 TxAdd : 1;
	UINT8 RxAdd : 1;
	UINT8 Length;
} *PBLE_ADV;


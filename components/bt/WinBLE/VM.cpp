/*

对RWIP基带软件入门级的仿真，仅摸拟数字中必需部分
供学习RWIP使用
Author: PeterLee
MoooreSillicon, TCL

*/


#include "stdafx.h"
#include "VM.h"
#include "EM.h"

extern "C" {
#include "reg_ipcore.h"
#include "reg_ipcore_bts.h"
#include "reg_blecore.h"
#include "_reg_em_et.h"
#include "em_map.h"

extern void rwip_isr(void);
} // extern "C"

#define PAS_PATH _T("C:\\Program Files (x86)\\Frontline Test System II\\Frontline 15.17\\Executables\\Core\\LiveImportAPI.dll")
//#define PAS_PATH _T("LiveImportAPI.dll")

#define VM_BIGWIN				(5 * 1000 * 10) // unit 0.1us, 5ms
#define _aSize(a)				sizeof(a)/sizeof(a[0])
#define MemberOffset(Class, Member)  int(&(((Class *)0)->Member))
#define REG32(addr)				(*(volatile uint32_t *)(addr))
#define REG16(addr)				(*(volatile uint16_t *)(addr))
#define EM_BASE					REG_EM_ET_BASE_ADDR
#define EM_PTR(addr)			(EM_BASE + (addr))

#define GET_PAGEINFO(p1, p2, pi) \
	p1 = PUINT8((pi).nStartAddr & ~0xFFFF); \
	p2 = PUINT8(((pi).nStartAddr + (pi).nSize + 0xFFFF) & ~0xFFFF);


const REG_PAGEINFO m_pi = { 0x21010000, 64 * 1024 };


static RW_VM m_vm;

static void VM_RegRW(UINT32 nAddr, UINT32 nValue0, BOOL bWrite);
static UINT32 VM_Thread(void* p);
static BOOL VM_RunCS(PUINT64 ptCur, PUINT64 ptET, PEM_CS pcs);
static void VM_TX(PVM_FD pfd, PVOID pPdu, INT cbPdu);

static inline void GetTime100ns(PUINT64 pt)
{
	::GetSystemTimeAsFileTime(LPFILETIME(pt));
	*pt -= m_vm.nSysTime0;
}

static inline void PulseWaitTimer()
{
	UINT64 t2 = -10; //1 us后中断

	::SetWaitableTimer(m_vm.hWaitTimer, PLARGE_INTEGER(&t2), 0, NULL, NULL, FALSE);
}

static inline void BtClk2Time100ns(UINT32 clk, UINT32 ft, PUINT64 pt)
{
	*pt = UINT64(clk) * 3125 + (624 - ft % 625) * 5;
}


static INT32 GetRfChannel(UINT32 c)
{
	switch (c)
	{
	case 37:
		c = 0;
		break;
	case 38:
		c = 12;
		break;
	case 39:
		break;
	default:
		c += c <= 10 ? 1 : 2;
		break;
	}

	return c;
}

static UINT32 BleCRC(UINT32 nCRC, const BYTE* pBuf, UINT cbBuf)
{
	const UINT32 m_nBits = (1 << 0) | (1 << 1) | (1 << 3) | (1 << 4) | (1 << 6) | (1 << 9) | (1 << 10);
	UINT i, j, B, b, b24;

	for (i = 0; i < cbBuf; i++)
	{
		B = pBuf[i];
		for (j = 0; j < 8; j++)
		{
			b = (B >> j) & 1;
			b24 = (nCRC >> 23) & 1;
			nCRC <<= 1;
			b24 ^= b;

			if (b24)
				nCRC ^= m_nBits;
		}
	}

	for (i = 0, j = 0; i <= 23; i++)
	{
		j |= ((nCRC >> (23 - i)) & 1) << i;
	}

	nCRC = j & 0xFFFFFF;
	// nCRC = ((j >> 16) & 0x0000FF) | (j & 0x00FF00) | ((j << 16) & 0xFF0000);

	return nCRC;
}



BOOL g_bVmTrace = FALSE;

extern "C" uint32_t reg_vm_rw(const char* desc, uint32_t addr, uint32_t value, uint32_t size_f)
{
	BOOL bWrite;
	UINT32 nValue0;

	bWrite = (size_f & REG_VM_W) ? TRUE : FALSE;
	nValue0 = *PUINT32(addr);

	if (bWrite)
	{
		if (size_f & REG_VM_VP)
		{
			CopyMemory(PUINT8(addr), PUINT8(value), uint16_t(size_f));
		}
		else
		{
			switch (uint16_t(size_f))
			{
			case 1:
				*PUINT8(addr) = UINT8(value);
				break;
			case 2:
				*PUINT16(addr) = UINT16(value);
				break;
			case 3:
				*PUINT32(addr) &= ~0xFFFFFF;
				*PUINT32(addr) |= value & 0xFFFFFF;
				break;
			case 4:
				*PUINT32(addr) = value;
				break;
			default:
				ASSERT(0);
			}
		}
	}

	VM_RegRW(addr, nValue0, bWrite);

	if (!bWrite && (size_f & REG_VM_VP))
	{
		CopyMemory(PUINT8(value), PUINT8(addr), uint16_t(size_f));
	}

	switch (uint16_t(size_f))
	{
	case 1:
		value = *PUINT8(addr);
		break;
	case 2:
		value = *PUINT16(addr);
		break;
	case 3:
		value = *PUINT32(addr) & 0xFFFFFF;
		break;
	case 4:
		value = *PUINT32(addr);
		break;
	default:
		ASSERT(size_f & REG_VM_VP);
	}

	if (g_bVmTrace)
	{
		TRACE("reg_vm_rw(0x%08x, %08x->%08x, %08x)\t%s\r\n", addr, nValue0, value, size_f, desc);
	}

	return value;
}


void VM_Init()
{
	PUINT8 p1, p2;

	GET_PAGEINFO(p1, p2, m_pi);
	p1 = (PUINT8)::VirtualAlloc(p1, p2 - p1, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
	ASSERT(p1);
	if (p1)
	{
		ZeroMemory(p1, p2 - p1);
	}

	ZeroMemory(&m_vm, sizeof(m_vm));


	m_vm.hPasDLL = ::LoadLibrary(PAS_PATH);
	m_vm.pfnPasRtx = (PasRtxFrame*)GetProcAddress(m_vm.hPasDLL, "PasRtxFrame");

	::GetSystemTimeAsFileTime(LPFILETIME(&m_vm.nSysTime0));

	m_vm.hWaitTimer = ::CreateWaitableTimer(NULL, FALSE, NULL);

	m_vm.hThread = ::CreateThread(NULL, 0, LPTHREAD_START_ROUTINE(VM_Thread), NULL, CREATE_SUSPENDED, NULL);
	if (m_vm.hThread != INVALID_HANDLE_VALUE)
	{
		::SetThreadPriority(m_vm.hThread, THREAD_PRIORITY_HIGHEST);
		::ResumeThread(m_vm.hThread);
	}

}


void VM_Deinit()
{
	
	::TerminateThread(m_vm.hThread, 0);
	::CloseHandle(m_vm.hThread);

	::FreeLibrary(m_vm.hPasDLL);

	PUINT8 p1, p2;

	GET_PAGEINFO(p1, p2, m_pi);
	::VirtualFree(p1, 0, MEM_RELEASE);
}

static void RegRwTimer(BOOL bWrite, INT i, UINT32 a1, UINT32 a2)
{
	if (bWrite)
	{
		UINT64 t, t2 = UINT64(REG32(a1)) * 3125 + REG32(a2) * 5;

		GetTime100ns(&t);

		if (INT32(t2 - t) >= 0)
		{
			m_vm.aTimer[i] = t2;
			PulseWaitTimer();
		}
	}
}

static void CalcTimerClkN()
{
	if (REG32(IP_INTCNTL1_ADDR) & IP_CLKNINTSRMSK_MASK)
	{
		UINT64 t;

		GetTime100ns(&t);
		m_vm.nTmrClkN = (t / 3125) * 3125; // CLKN align

		m_vm.nTmrClkN += UINT64((REG32(IP_INTCNTL1_ADDR) & IP_CLKNINTSRVAL_MASK) >> IP_CLKNINTSRVAL_LSB) * 3125; // CLKNINTSRVAL
		if (INT32(m_vm.nTmrClkN - t) <= 0)
		{
			UINT32 i = 1 << ((REG32(IP_INTCNTL1_ADDR) & IP_CLKNINTSRMSK_MASK) >> IP_CLKNINTSRMSK_LSB);
			m_vm.nTmrClkN += UINT64(i) * 3125;
		}
	}
	else
	{
		m_vm.nTmrClkN = 0;
	}
}

void VM_RegRW(UINT32 nAddr, UINT32 nValue0, BOOL bWrite)
{
	switch (nAddr)
	{
	case IP_SLOTCLK_ADDR:
		if (bWrite)
		{
			UINT32 v = REG32(IP_SLOTCLK_ADDR);

			if (v & IP_SAMP_BIT)
			{
				UINT64 t;

				GetTime100ns(&t);

				v &= ~IP_SAMP_BIT;
				v &= ~IP_SCLK_MASK;
				v |= (UINT32(t / 3125)) & IP_SCLK_MASK;
				REG32(IP_SLOTCLK_ADDR) = v;

				REG32(IP_FINETIMECNT_ADDR) = 624 - (UINT32(t / 5) % 625); // IP_FINECNT_MASK

				REG32(IP_ISOCNTSAMP_ADDR) = UINT32(t / 10);
			}
		}
		break;
	case IP_RWDMCNTL_ADDR:
		if (bWrite)
		{
			UINT32 v = REG32(IP_RWDMCNTL_ADDR);

			REG32(IP_RWDMCNTL_ADDR) = v & ~(0xFFUL << 24);

			if (v & (1 << 27) && (REG32(IP_INTCNTL1_ADDR) & IP_SWINTMSK_BIT))
			{
				REG32(IP_INTSTAT1_ADDR) |= IP_SWINTSTAT_BIT;
				PulseWaitTimer();
			}
		}
		break;
	case IP_CLKNTGT1_ADDR:
	case IP_HMICROSECTGT1_ADDR:
		RegRwTimer(bWrite, 1, IP_CLKNTGT1_ADDR, IP_HMICROSECTGT1_ADDR);
		break;
	case IP_CLKNTGT2_ADDR:
	case IP_HMICROSECTGT2_ADDR:
		RegRwTimer(bWrite, 2, IP_CLKNTGT2_ADDR, IP_HMICROSECTGT2_ADDR);
		break;
	case IP_CLKNTGT3_ADDR:
	case IP_HMICROSECTGT3_ADDR:
		RegRwTimer(bWrite, 3, IP_CLKNTGT3_ADDR, IP_HMICROSECTGT3_ADDR);
		break;
	case IP_INTCNTL1_ADDR:
		CalcTimerClkN();
		break;
	case IP_INTACK1_ADDR:
		REG32(IP_INTSTAT1_ADDR) &= ~REG32(IP_INTACK1_ADDR);
		break;
	case IP_ACTSCHCNTL_ADDR:
		if (bWrite && (REG32(IP_ACTSCHCNTL_ADDR) & IP_START_ACT_BIT))
		{
			REG32(IP_ACTSCHCNTL_ADDR) &= ~IP_START_ACT_BIT;
			if (m_vm.cntEtEvent2 == m_vm.cntEtEvent)
			{
				m_vm.nEtPos = REG32(IP_ACTSCHCNTL_ADDR) & IP_ENTRY_IDX_MASK;
			}
			m_vm.cntEtEvent2++;
			PulseWaitTimer();
		}
		break;
	default:
		break;
	}
}


UINT32 VM_Thread(void* p)
{
	INT rc;

	for (;;)
	{
		rc = ::WaitForSingleObject(m_vm.hWaitTimer, INFINITE);
		
		switch (rc)
		{
		case WAIT_OBJECT_0:
			if (1)
			{
				INT i;
				UINT64 t, t2 = -1;
			
				GetTime100ns(&t);

				while (INT(m_vm.cntEtEvent2 - m_vm.cntEtEvent) > 0)
				{
					PEM_ET pet = (PEM_ET)EM_PTR(REG32(BLE_ETPTR_ADDR) & BLE_ETPTR_MASK);
					UINT64 tt;

					pet += m_vm.nEtPos;

					BtClk2Time100ns(pet->RAWSTP, pet->FINESTP, &tt);
					if (INT32(t - tt) >= 0)
					{
						BOOL bDo = FALSE;
						if (pet->CSPTR)
						{
							PEM_CS pcs = (PEM_CS)EM_PTR(pet->CSPTR << 2);
							bDo = VM_RunCS(&t, &tt, pcs);
						}

						pet->STATUS = EM_ET_STATUS_TERM_NORMAL;
						REG32(IP_INTSTAT1_ADDR) |= IP_FIFOINTSTAT_BIT;
						REG32(IP_ACTFIFOSTAT_ADDR) = (REG32(IP_ACTFIFOSTAT_ADDR) & ~IP_CURRENT_ET_IDX_MASK)
							| (m_vm.nEtPos << IP_CURRENT_ET_IDX_LSB) | IP_ENDACTINTSTAT_BIT;
						m_vm.cntEtEvent++;
						if (++m_vm.nEtPos >= 16)
							m_vm.nEtPos--;
					}
					else if (t2 > tt)
					{
						t2 = tt;
						break;
					}
				}

				for (i = 0; i < _aSize(m_vm.aTimer); i++)
				{
					if (m_vm.aTimer[i])
					{
						if (INT32(t - m_vm.aTimer[i]) >= 0)
						{
							m_vm.aTimer[i] = 0;
							if (REG32(IP_INTCNTL1_ADDR) & (IP_FINETGTINTMSK_BIT << i))
							{
								REG32(IP_INTSTAT1_ADDR) |= IP_FINETGTINTSTAT_BIT << i;
							}
						}
						else if (t2 > m_vm.aTimer[i])
						{
							t2 = m_vm.aTimer[i];
						}
					}
				}

				if (m_vm.nTmrClkN)
				{
					if (INT32(t - m_vm.nTmrClkN) >= 0)
					{
						CalcTimerClkN();
						if (REG32(IP_INTCNTL1_ADDR) & IP_CLKNINTMSK_BIT)
						{
							REG32(IP_INTSTAT1_ADDR) |= IP_CLKNINTSTAT_BIT;
						}

					}
					else if (t2 > m_vm.nTmrClkN)
					{
						t2 = m_vm.nTmrClkN;
					}
				}

			
				if (INT64(t2) != -1)
				{
					t2 += m_vm.nSysTime0;
					::SetWaitableTimer(m_vm.hWaitTimer, PLARGE_INTEGER(&t2), 0, NULL, NULL, FALSE);
				}

				rwip_isr();

			}
			break;
		}
	}

	return 0;
}


BOOL VM_RunCS(PUINT64 ptCur, PUINT64 ptET, PEM_CS pcs)
{
	UINT64 t2 = *ptET + UINT64(pcs->MINEVTIME) * 6250;

	if (INT32(*ptCur - t2) > VM_BIGWIN)
		return FALSE;

	switch (pcs->FORMAT)
	{
	case 4: // Low Duty Cycle Advertiser
		if (1)
		{
			PEM_TXDESC ptd = (PEM_TXDESC)EM_PTR(pcs->ACLTXDESCPTR << 2);
			PUINT8 pb = (PUINT8)EM_PTR(ptd->TXDATAPTR);
			VM_FD fd;
			UINT8 aBuf[260];
			PBLE_ADV pAdv = PBLE_ADV(aBuf);

			TRACE("VM_RunCS: adv\r\n");

			pAdv->PDU_Type = 0; // ADV_IND;
			pAdv->RFU = 0;
			pAdv->ChSel = 0;
			pAdv->TxAdd = 0;
			pAdv->RxAdd = 0;
			pAdv->Length = ptd->TXADVLEN;

			CopyMemory(pAdv + 1, pcs->BDADDR, 6);
			CopyMemory(PBYTE(pAdv + 1) + 6, pb, pAdv->Length);

			fd.nType = pcs->FORMAT;
			fd.RSSI = 0;
			fd.nState = 0;
			fd.AA = pcs->SYNCW;
			fd.ch = 37;
			fd.nInitCRC = pcs->CRCINIT;
			fd.ts = m_vm.nSysTime0 + *ptET;

			do
			{
				VM_TX(&fd, pAdv, sizeof(*pAdv) + pAdv->Length);
				fd.ts += 704 * 10;  // ns;
			} while (++fd.ch < 40);
		}
		break;
	}

	return TRUE;
}

void VM_TX(PVM_FD pfd, PVOID pPdu, INT cbPdu)
{
	const UINT8 m_aPas[] =
	{
		0x22, 0x03, 0x00, 0x00, 0x00, 0x80, 0x8a, 0x00, 0xd6, 0xbe, 0x89, 0x8e, 0xe2, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00,
		0xd6, 0xbe, 0x89, 0x8e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x0e, 0x00
	};

	PAS_HEAD pas;
	PAS_RTX tx;

	CopyMemory(&pas, m_aPas, sizeof(pas));
	pas.RfChannel = GetRfChannel(pfd->ch);
	pas.AccessAddress2 = pas.AccessAddress = pfd->AA;
	pas.RSSI = INT8(pfd->RSSI) * 8;
	if (pfd->nType & 0x80)
		pas.ConnectFrame = 1;
	pas.AuxType = 0;
	pas.PduSize = cbPdu;

	tx.nPos = -1;
	tx.pHead = &pas;
	tx.cbHead = sizeof(pas);
	tx.pPdu = pPdu;
	tx.cbPdu = cbPdu;
	tx.nPduCRC = BleCRC(pfd->nInitCRC, PBYTE(pPdu), cbPdu);
	tx.nDrf = 0x0100C000;
	tx.ts = pfd->ts;

	m_vm.pfnPasRtx(&tx);
	if (tx.nPos != -1)
		m_vm.nPasPos = tx.nPos;

}



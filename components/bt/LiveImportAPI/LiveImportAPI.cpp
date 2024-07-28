// LiveImportAPI.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"

#include "LiveImportAPI.h"

#define TRACE(psz, ...)
#define DllImport  extern "C" __declspec( dllimport )
#define DllExport  extern "C" __declspec( dllexport )

typedef struct PAS_RTX
{
	INT32 nPos; // IN/OUT
	PVOID hEvent; // OUT, for RX Wait
	PVOID pHead;
	INT32 cbHead;
	PVOID pPdu;
	INT32 cbPdu;
	UINT32 nPduCRC;
	INT32 nDrf;
	INT64 ts;
} *PPAS_RTX;

typedef struct PAS_AIRPKT
{
	UINT16 cbHead;
	UINT16 cbPdu;
	INT32 nDrf;
	INT64 ts;
	// UINT8 aHead[cbHead];
	// UINT8 aPDU[cbPdu];
	// UINT8 aCRC[3];
} *PPAS_AIRPKT;

#define AIRBUF_NUM			13
#pragma data_seg("Shared")
volatile LONG m_nTxPos = -1;
static LONG m_nRxPos = 0;
static INT64 m_tsOffset = 0;
BYTE g_aAirBuf[AIRBUF_NUM][312] = { 0 }; // max 311, +1B for CRC
#pragma data_seg()
#pragma comment(linker, "/Section:Shared,RWS")

static HANDLE m_hPasThread = NULL;
static HANDLE m_hRtxEvent = NULL;

static DWORD ThreadProcPAS(LPVOID p)
{
	INT rc;

	for (;;)
	{
		rc = WaitForSingleObject(m_hRtxEvent, INFINITE);

		switch (rc)
		{
		case WAIT_OBJECT_0:
			while (m_nRxPos != m_nTxPos)
			{
				PPAS_AIRPKT p = PPAS_AIRPKT(g_aAirBuf[m_nRxPos]);
				INT cb = p->cbHead + p->cbPdu + 3;

				TRACE("%s %d-%d, drf%08x,t%08x-%08x\r\n", __FUNCTION__,
					p->cbHead, p->cbPdu, p->nDrf, *PDWORD(&p->ts), *(PDWORD(&p->ts) + 1));

				rc = g_pSendFrame2(1,
					cb,				// The "real" length of a frame. Some frames may be truncated, so this may not be the same as included length
					cb,				// The size of the data passed in this call
					PBYTE(p + 1),	// The actual bytes of the frame
					p->nDrf,		// any errors or other data related flag
					0,				// Which side this data comes from
					p->ts + m_tsOffset);

				if (++m_nRxPos >= AIRBUF_NUM)
					m_nRxPos = 0;
			}
			break;
		default:
			TRACE(TEXT("ThreadProcPAS: Error %d\r\n"), GetLastError());
			break;
		}
	}
}

BOOL LoadLiveImport(HMODULE hModule)
{
	if (!m_hRtxEvent)
		m_hRtxEvent = CreateEvent(NULL, TRUE, FALSE, TEXT("$LiveImportAPI$"));

	if (m_nTxPos == -1)
	{
		SYSTEMTIME st;
		INT64 ts1, ts2;

		m_nTxPos = 0;

		GetLocalTime(&st);
		SystemTimeToFileTime(&st, LPFILETIME(&ts2));
		GetSystemTimeAsFileTime(LPFILETIME(&ts1));
		m_tsOffset = ts2 - ts1;

		m_hPasThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)ThreadProcPAS,
			hModule, CREATE_SUSPENDED, NULL);
		if (m_hPasThread)
		{
			SetThreadPriority(m_hPasThread, THREAD_PRIORITY_HIGHEST);
			ResumeThread(m_hPasThread);
		}
	}

	TRACE("LiveImportAPI %08x\r\n", m_hPasThread);

	return LoadLiveImportAPIFunctions(hModule);

}

void UnloadLiveImport(HMODULE hModule)
{
	NullLiveImportFunctionPointers();
}


DllExport void PasRtxFrame(PPAS_RTX pRtx)
{
	if (pRtx->nPos == -1) // tx
	{
		INT nPos = m_nTxPos;
		PPAS_AIRPKT p = PPAS_AIRPKT(g_aAirBuf[nPos]);
		PUINT32 pCRC = PUINT32(PBYTE(p + 1) + pRtx->cbHead + pRtx->cbPdu);

		p->cbHead = pRtx->cbHead;
		p->cbPdu = pRtx->cbPdu;
		p->nDrf = pRtx->nDrf;
		p->ts = pRtx->ts;
		CopyMemory(p + 1, pRtx->pHead, p->cbHead);
		CopyMemory(PBYTE(p + 1) + p->cbHead, pRtx->pPdu, p->cbPdu);
		*pCRC = pRtx->nPduCRC;

		pRtx->nPos = nPos + 1 < AIRBUF_NUM ? nPos + 1 : 0;
		if (::InterlockedCompareExchange(&m_nTxPos, pRtx->nPos, nPos) == nPos)
			::PulseEvent(m_hRtxEvent);
		else
			pRtx->nPos = -1;
	}
	else // rx
	{
		if (pRtx->nPos == m_nTxPos)
		{
			pRtx->hEvent = m_hRtxEvent;
			return;
		}

		PPAS_AIRPKT p = PPAS_AIRPKT(g_aAirBuf[pRtx->nPos]);
		PUINT32 pCRC = PUINT32(PBYTE(p + 1) + p->cbHead + p->cbPdu);

		pRtx->pHead = PBYTE(p + 1);
		pRtx->pPdu = PBYTE(p + 1) + p->cbHead;
		pRtx->cbHead = p->cbHead;
		pRtx->cbPdu = p->cbPdu;
		pRtx->nPduCRC = *pCRC;
		pRtx->nDrf = p->nDrf;
		pRtx->ts = p->ts;

		pRtx->nPos = pRtx->nPos + 1 >= AIRBUF_NUM ? 0 : pRtx->nPos + 1;
	}
}


//////////////////////////////////////////////////////////////////////////////////////////


DllExport HRESULT GetDllVersion(TCHAR** pszDllVersion, int* piSize)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pGetDllVersion(pszDllVersion, piSize);
}

DllExport HRESULT InitializeLiveImport(const TCHAR* szMemoryName, const TCHAR* szConfiguration, bool* pboolSuccess)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pInitializeLiveImport(szMemoryName, szConfiguration, pboolSuccess);
}

DllExport HRESULT InitializeLiveImportEx(const TCHAR* szMemoryName, const TCHAR* szConfiguration, bool* pboolSuccess, EQueueMode eQueueMode)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pInitializeLiveImportEx(szMemoryName, szConfiguration, pboolSuccess, eQueueMode);
}

DllExport HRESULT SetEnqueueMode(EQueueMode eQueueMode)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSetEnqueueMode(eQueueMode);
}

DllExport HRESULT ReleaseLiveImport(void)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pReleaseLiveImport();
}

DllExport HRESULT StillAlive(bool* pboolIsAppAlive)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pStillAlive(pboolIsAppAlive);
}

DllExport HRESULT IsAppReady(bool* pboolIsAppReady)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pIsAppReady(pboolIsAppReady);
}

DllExport HRESULT SendFrame(
	int iOriginalLength,				// The "real" length of a frame. Some frames may be truncated, so this may not be the same as included length
	int iIncludedLength,				// The size of the data passed in this call
	const BYTE* pbytFrame,				// The actual bytes of the frame
	int iDrf,							// any errors or other data related flag
	int iStream,						// Which side this data comes from
	__int64 i64Timestamp)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendFrame(
		iOriginalLength,				// The "real" length of a frame. Some frames may be truncated, so this may not be the same as included length
		iIncludedLength,				// The size of the data passed in this call
		pbytFrame,						// The actual bytes of the frame
		iDrf,							// any errors or other data related flag
		iStream,						// Which side this data comes from
		i64Timestamp);
}

DllExport HRESULT SendFrame2(
	int iDatasourceId,
	int iOriginalLength,				// The "real" length of a frame. Some frames may be truncated, so this may not be the same as included length
	int iIncludedLength,				// The size of the data passed in this call
	const BYTE* pbytFrame,				// The actual bytes of the frame
	int iDrf,							// any errors or other data related flag
	int iStream,						// Which side this data comes from
	__int64 i64Timestamp)
{
	TRACE("%s ds%d, %d-%d, drf%08x, s%08x t%08x-%08x\r\n", __FUNCTION__,
		iDatasourceId, iOriginalLength, iIncludedLength, iDrf, iStream,
		*PDWORD(&i64Timestamp), *(PDWORD(&i64Timestamp) + 1));

	return g_pSendFrame2(
		iDatasourceId,
		iOriginalLength,				// The "real" length of a frame. Some frames may be truncated, so this may not be the same as included length
		iIncludedLength,				// The size of the data passed in this call
		pbytFrame,						// The actual bytes of the frame
		iDrf,							// any errors or other data related flag
		iStream,						// Which side this data comes from
		i64Timestamp);
}


DllExport HRESULT SendFrameWithComment(
	int iOriginalLength,				// The "real" length of a frame. Some frames may be truncated, so this may not be the same as included length
	int iIncludedLength,				// The size of the data passed in this call
	const BYTE* pbytFrame,				// The actual bytes of the frame
	int iDrf,							// any errors or other data related flag
	int iStream,						// Which side this data comes from
	__int64 i64Timestamp,
	const TCHAR* ptcComment,  // Comment
	unsigned int uiCommentLength  // Length of comment
	)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendFrameWithComment(
		iOriginalLength,				// The "real" length of a frame. Some frames may be truncated, so this may not be the same as included length
		iIncludedLength,				// The size of the data passed in this call
		pbytFrame,						// The actual bytes of the frame
		iDrf,							// any errors or other data related flag
		iStream,						// Which side this data comes from
		i64Timestamp,
		ptcComment,  // Comment
		uiCommentLength);  // Length of comment
}

DllExport HRESULT SendFrameWithCommentFromDatasource(
	int iDatasourceId,
	int iOriginalLength,				// The "real" length of a frame. Some frames may be truncated, so this may not be the same as included length
	int iIncludedLength,				// The size of the data passed in this call
	const BYTE* pbytFrame,				// The actual bytes of the frame
	int iDrf,							// any errors or other data related flag
	int iStream,						// Which side this data comes from
	__int64 i64Timestamp,
	const TCHAR* ptcComment,  // Comment
	unsigned int uiCommentLength  // Length of comment
	)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendFrameWithCommentFromDatasource(
		iDatasourceId,
		iOriginalLength,				// The "real" length of a frame. Some frames may be truncated, so this may not be the same as included length
		iIncludedLength,				// The size of the data passed in this call
		pbytFrame,						// The actual bytes of the frame
		iDrf,							// any errors or other data related flag
		iStream,						// Which side this data comes from
		i64Timestamp,
		ptcComment,  // Comment
		uiCommentLength  // Length of comment
		);
}


DllExport HRESULT SendKeepAliveEvent(int iDatasourceId, eKeepAliveEventTypes eKaeType, __int64 i64Timestamp)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendKeepAliveEvent(iDatasourceId, eKaeType, i64Timestamp);
}

DllExport HRESULT SendEvent(BYTE bytData, int iDrf, int iStream, __int64 i64Timestamp)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendEvent(bytData, iDrf, iStream, i64Timestamp);
}

DllExport HRESULT SendControlSignalChange(int iNdrf, __int64 i64Timestamp)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendControlSignalChange(iNdrf, i64Timestamp);
}

DllExport HRESULT SendNondataFlagChange(int iNdrf, __int64 i64Timestamp)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendNondataFlagChange(iNdrf, i64Timestamp);
}

DllExport HRESULT SendBreak(int iStream, __int64 i64Timestamp)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendBreak(iStream, i64Timestamp);
}

DllExport HRESULT SendFlowControl(bool boolFlowControlIsOn, __int64 i64Timestamp)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendFlowControl(boolFlowControlIsOn, i64Timestamp);
}

DllExport HRESULT SendConnectionStatus(bool boolIsConnected, __int64 i64Timestamp)
{
	TRACE("%s\r\n", __FUNCTION__);
	return  g_pSendConnectionStatus(boolIsConnected, i64Timestamp);
}

DllExport HRESULT SendConfigurationString(const TCHAR* szConfiguration)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendConfigurationString(szConfiguration);
}

DllExport HRESULT SendConfigurationString2(int iDatasourceId, const TCHAR* szConfiguration)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendConfigurationString2(iDatasourceId, szConfiguration);
}

DllExport HRESULT SendSetIODialogHwnd (const HWND hwnd)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendSetIODialogHwnd(hwnd);
}

DllExport HRESULT SendXmitDialogHwnd (const HWND hwnd)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendXmitDialogHwnd(hwnd);
}

DllExport HRESULT SendSpecialEvent(int iStream, int iEventNumber, __int64 i64Timestamp)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendSpecialEvent(iStream, iEventNumber, i64Timestamp);
}

DllExport HRESULT SendStartOfFrame(int iStream, __int64 i64Timestamp)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendStartOfFrame(iStream, i64Timestamp);
}

DllExport HRESULT SendEndOfFrame(int iStream, __int64 i64Timestamp)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendEndOfFrame(iStream, i64Timestamp);
}

DllExport HRESULT SendAbortedFrame(int iStream, __int64 i64Timestamp)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendAbortedFrame(iStream, i64Timestamp);
}

DllExport HRESULT SendByteSimple(BYTE byData, __int64 i64Timestamp)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendByteSimple(byData, i64Timestamp);
}

DllExport HRESULT CheckForMessages()
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pCheckForMessages();
}

DllExport HRESULT GetAppVersionNumber(TCHAR** pszAppVersionNumber, int* piSize)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pGetAppVersionNumber(pszAppVersionNumber, piSize);
}

DllExport HRESULT GetAppSerialNumber(TCHAR** pszAppSerialNumber, int* piSize)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pGetAppSerialNumber(pszAppSerialNumber, piSize);
}

DllExport HRESULT GetAppDisplayedConfigurationName(TCHAR** pszAppConfigName, int* piSize)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pGetAppDisplayedConfigurationName(pszAppConfigName, piSize);
}

DllExport HRESULT GetSerialNumberSectionKeyValuePairs(TCHAR** pszKeyValuePairs, int* piSize)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pGetSerialNumberSectionKeyValuePairs(pszKeyValuePairs, piSize);
}

DllExport HRESULT GetDriverSavePath(TCHAR** pszDriverSavePath, int* piSize)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pGetDriverSavePath(pszDriverSavePath, piSize);
}

DllExport HRESULT GetDriverSaveName(TCHAR** pszDriverSaveName, int* piSize)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pGetDriverSaveName(pszDriverSaveName, piSize);
}

DllExport HRESULT RegisterNotification(eNotificationTypes eType, NotificationType pNotification, void* pThis)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pRegisterNotification(eType, pNotification, pThis);
}

DllExport HRESULT RegisterNotification2(eNotificationTypes eType, NotificationType2 pNotification, void* pThis)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pRegisterNotification2(eType, pNotification, pThis);
}

DllExport HRESULT SendNotification(eNotificationTypes eType)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendNotification(eType);
}

DllExport HRESULT SendNotification2(int iDatasourceId, eNotificationTypes eType)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendNotification2(iDatasourceId, eType);
}

DllExport HRESULT SendArraySimple(BYTE* pbytData, int iLength, __int64 i64Timestamp)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendArraySimple(pbytData, iLength, i64Timestamp);
}

DllExport HRESULT SendStringSimple(TCHAR* szData, __int64 i64Timestamp)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendStringSimple(szData, i64Timestamp);
}

DllExport HRESULT SendNumberOfLostMessages(const int iNumberOfLostMessages)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendNumberOfLostMessages(iNumberOfLostMessages);
}

DllExport HRESULT UpdateStat(int iStream, int iStatNumber, __int64 i64IncrementAmount)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pUpdateStat(iStream, iStatNumber, i64IncrementAmount);
}

DllExport HRESULT FramesLost(int iFramesLost)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pFramesLost(iFramesLost);
}

DllExport HRESULT FramesLost2(int iDatasourceId, int iFramesLost)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pFramesLost2(iDatasourceId, iFramesLost);
}

DllExport HRESULT SetExePath(TCHAR* szServerPath)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSetExePath(szServerPath);
}

DllExport HRESULT SendPostNotify(UINT uiId, WPARAM wParam, LPARAM lParam)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendPostNotify(uiId, wParam, lParam);
}

DllExport HRESULT SendComment(const TCHAR* szComment)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendComment(szComment);
}

DllExport HRESULT AddNamedData(const TCHAR* szName, const TCHAR* szValue)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pAddNamedData(szName, szValue);
}

DllExport HRESULT SetDataSourceFilterName(const TCHAR* szFilterName)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSetDataSourceFilterName(szFilterName);
}

DllExport HRESULT SendQueueStats(unsigned int uiProcessId, __int64 i64NumQueued, const TCHAR* szDataSrcName)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendQueueStats(uiProcessId, i64NumQueued, szDataSrcName);
}

DllExport HRESULT SaveAndClear(const TCHAR* szSavePath)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSaveAndClear(szSavePath);
}

DllExport HRESULT SaveCapture(const TCHAR* szSavePath)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSaveCapture(szSavePath);
}

DllExport HRESULT ClearCapture(void)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pClearCapture();
}

DllExport HRESULT ExportHtmlToPath(const TCHAR* htmlPath)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pExportHtmlToPath(htmlPath);
}

DllExport HRESULT GetLiveImportInitializationStatus(long* pliveImportInitializationStatus)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pGetLiveImportInitializationStatus(pliveImportInitializationStatus);
}

DllExport HRESULT SendSpectrumPathName(const TCHAR* szSpectrumPathName)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendSpectrumPathName(szSpectrumPathName);
}

DllExport HRESULT SendStartingTimestamp(__int64 i64Timestamp)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendStartingTimestamp(i64Timestamp);
}

DllExport HRESULT GetNumFreeFrameBytes(int *pAvailFrameBytes)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pGetNumFreeFrameBytes(pAvailFrameBytes);
}

DllExport HRESULT GetNumAppEventEnqueueBusyWaits(int *pNumBusyWaits)
{
	TRACE("%s\r\n", __FUNCTION__);
	return  g_pGetNumAppEventEnqueueBusyWaits(pNumBusyWaits);
}



//  add for pas 15.17

DllExport HRESULT GlobalInterfaceCheck(int p1, int p2, int p3, int p4)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pGlobalInterfaceCheck(p1, p2, p3, p4);
}

DllExport HRESULT SendPower(int iLevel, __int64 i64Timestamp)
{
	TRACE("%s\r\n", __FUNCTION__);
	return g_pSendPower(iLevel, i64Timestamp);
}


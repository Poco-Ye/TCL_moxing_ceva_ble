// LiveImportAPI.h : Defines the initialization routines for the DLL.

#include "tchar.h"
#include "DriverNotifications.h"
#include "QueueMode.h"
#include "stdlib.h"
#include "stdio.h"

// Type/value definitions:
#ifndef	__cplusplus
#define	bool BYTE
#define	true ((bool) 1)
#define	false ((bool) 0)
#endif

// API function pointer definitions:
typedef HRESULT (_GetDllVersion)(TCHAR** pszDllVersion, int* piSize);
typedef HRESULT (_InitializeLiveImport)(const TCHAR* szMemoryName, const TCHAR* szConfiguration, bool* pboolSuccess);
typedef HRESULT (_InitializeLiveImportEx)(const TCHAR* szMemoryName, const TCHAR* szConfiguration, bool* pboolSuccess, EQueueMode eQueueMode);
typedef HRESULT (_SetEnqueueMode)(EQueueMode eQueueMode);
typedef HRESULT (_ReleaseLiveImport)(void);
typedef HRESULT (_StillAlive)(bool* pboolIsAppAlive);
typedef HRESULT (_IsAppReady)(bool* pboolIsAppReady);

typedef HRESULT (_SendFrame)(
		int iOriginalLength,				// The "real" length of a frame. Some frames may be truncated, so this may not be the same as included length
		int iIncludedLength,				// The size of the data passed in this call
		const BYTE* pbytFrame,						// The actual bytes of the frame
		int iDrf,							// any errors or other data related flag
		int iStream,						// Which side this data comes from
		__int64 i64Timestamp);

typedef HRESULT (_SendFrame2)(
		int iDatasourceId,
		int iOriginalLength,				// The "real" length of a frame. Some frames may be truncated, so this may not be the same as included length
		int iIncludedLength,				// The size of the data passed in this call
		const BYTE* pbytFrame,						// The actual bytes of the frame
		int iDrf,							// any errors or other data related flag
		int iStream,						// Which side this data comes from
		__int64 i64Timestamp);

typedef HRESULT (_SendFrameWithComment)(
		int iOriginalLength,				// The "real" length of a frame. Some frames may be truncated, so this may not be the same as included length
		int iIncludedLength,				// The size of the data passed in this call
		const BYTE* pbytFrame,						// The actual bytes of the frame
		int iDrf,							// any errors or other data related flag
		int iStream,						// Which side this data comes from
		__int64 i64Timestamp,
		const TCHAR* ptcComment,  // Comment
		unsigned int uiCommentLength  // Length of comment
		);

typedef HRESULT (_SendFrameWithCommentFromDatasource)(
		int iDatasourceId,
		int iOriginalLength,				// The "real" length of a frame. Some frames may be truncated, so this may not be the same as included length
		int iIncludedLength,				// The size of the data passed in this call
		const BYTE* pbytFrame,						// The actual bytes of the frame
		int iDrf,							// any errors or other data related flag
		int iStream,						// Which side this data comes from
		__int64 i64Timestamp,
		const TCHAR* ptcComment,  // Comment
		unsigned int uiCommentLength  // Length of comment
		);

typedef HRESULT (_SendKeepAliveEvent)(int iDatasourceId, eKeepAliveEventTypes eKaeType, __int64 i64Timestamp);
typedef HRESULT (_SendEvent)(BYTE bytData, int iDrf, int iStream, __int64 i64Timestamp);
typedef HRESULT (_SendControlSignalChange)(int iNdrf, __int64 i64Timestamp);
typedef HRESULT (_SendNondataFlagChange)(int iNdrf, __int64 i64Timestamp);
typedef HRESULT (_SendBreak)(int iStream, __int64 i64Timestamp);
typedef HRESULT (_SendFlowControl)(bool boolFlowControlIsOn, __int64 i64Timestamp);
typedef HRESULT (_SendPower)(int iLevel, __int64 i64Timestamp);
typedef HRESULT (_SendConnectionStatus)(bool boolIsConnected, __int64 i64Timestamp);
typedef HRESULT (_SendConfigurationString)(const TCHAR* szConfiguration);
typedef HRESULT (_SendConfigurationString2)(int iDatasourceId, const TCHAR* szConfiguration);
typedef HRESULT (_SendSetIODialogHwnd) (const HWND hwnd);
typedef HRESULT (_SendXmitDialogHwnd) (const HWND hwnd);
typedef HRESULT (_SendSpecialEvent)(int iStream, int iEventNumber, __int64 i64Timestamp);
typedef HRESULT (_SendStartOfFrame)(int iStream, __int64 i64Timestamp);
typedef HRESULT (_SendEndOfFrame)(int iStream, __int64 i64Timestamp);
typedef HRESULT (_SendAbortedFrame)(int iStream, __int64 i64Timestamp);
typedef HRESULT (_SendByteSimple)(BYTE byData, __int64 i64Timestamp);
typedef HRESULT (_CheckForMessages)();
typedef HRESULT (_GetAppVersionNumber)(TCHAR** pszAppVersionNumber, int* piSize);
typedef HRESULT (_GetAppSerialNumber)(TCHAR** pszAppSerialNumber, int* piSize);
typedef HRESULT (_GetAppDisplayedConfigurationName)(TCHAR** pszAppConfigName, int* piSize);
typedef HRESULT (_GetSerialNumberSectionKeyValuePairs)(TCHAR** pszKeyValuePairs, int* piSize);
typedef HRESULT (_GetDriverSavePath)(TCHAR** pszDriverSavePath, int* piSize);
typedef HRESULT (_GetDriverSaveName)(TCHAR** pszDriverSaveName, int* piSize);
typedef HRESULT (_RegisterNotification)(eNotificationTypes eType, NotificationType pNotification, void* pThis);
typedef HRESULT (_RegisterNotification2)(eNotificationTypes eType, NotificationType2 pNotification, void* pThis);
typedef HRESULT (_SendNotification)(eNotificationTypes eType);
typedef HRESULT (_SendNotification2)(int iDatasourceId, eNotificationTypes eType);
typedef HRESULT (_SendArraySimple)(BYTE* pbytData, int iLength, __int64 i64Timestamp);
typedef HRESULT (_SendStringSimple)(TCHAR* szData, __int64 i64Timestamp);
typedef HRESULT (_SendNumberOfLostMessages)(const int iNumberOfLostMessages);
typedef HRESULT (_UpdateStat)(int iStream, int iStatNumber, __int64 i64IncrementAmount);
typedef HRESULT (_FramesLost)(int iFramesLost);
typedef HRESULT (_FramesLost2)(int iDatasourceId, int iFramesLost);
typedef HRESULT (_SetExePath)(TCHAR* szServerPath);
typedef HRESULT (_SendPostNotify)(UINT uiId, WPARAM wParam, LPARAM lParam);
typedef HRESULT (_SendComment)(const TCHAR* szComment);
typedef HRESULT (_AddNamedData)(const TCHAR* szName, const TCHAR* szValue);
typedef HRESULT (_SetDataSourceFilterName)(const TCHAR* szFilterName);
typedef HRESULT (_SendQueueStats)(unsigned int uiProcessId, __int64 i64NumQueued, const TCHAR* szDataSrcName);
typedef HRESULT (_SaveAndClear)(const TCHAR* szSavePath);
typedef HRESULT (_SaveCapture)(const TCHAR* szSavePath);
typedef HRESULT (_ClearCapture)(void);
typedef HRESULT (_ExportHtmlToPath)(const TCHAR* htmlPath);
typedef HRESULT (_GetLiveImportInitializationStatus)(long* pliveImportInitializationStatus);
typedef HRESULT (_SendSpectrumPathName)(const TCHAR* szSpectrumPathName);
typedef HRESULT (_SendStartingTimestamp)(__int64 i64Timestamp);
typedef HRESULT (_GetNumFreeFrameBytes)(int *pAvailFrameBytes);
typedef HRESULT (_GetNumAppEventEnqueueBusyWaits)(int *pNumBusyWaits);

#ifndef	SIMPLIFIED_LIVE_IMPORT
// API function pointer variable definitions:
_GetDllVersion* g_pGetDllVersion= NULL;
_InitializeLiveImport* g_pInitializeLiveImport= NULL;
_InitializeLiveImportEx* g_pInitializeLiveImportEx= NULL;
_SetEnqueueMode* g_pSetEnqueueMode= NULL;
_ReleaseLiveImport* g_pReleaseLiveImport= NULL;
_StillAlive* g_pStillAlive= NULL;
_IsAppReady* g_pIsAppReady= NULL;
_SendFrame* g_pSendFrame= NULL;
_SendFrame2* g_pSendFrame2= NULL;
_SendFrameWithComment* g_pSendFrameWithComment= NULL;
_SendFrameWithCommentFromDatasource* g_pSendFrameWithCommentFromDatasource= NULL;
_SendKeepAliveEvent* g_pSendKeepAliveEvent= NULL;
_SendEvent* g_pSendEvent= NULL;
_SendControlSignalChange* g_pSendControlSignalChange= NULL;
_SendNondataFlagChange* g_pSendNondataFlagChange= NULL;
_SendBreak* g_pSendBreak= NULL;
_SendFlowControl* g_pSendFlowControl= NULL;
_SendConnectionStatus* g_pSendConnectionStatus= NULL;
_SendConfigurationString* g_pSendConfigurationString= NULL;
_SendConfigurationString2* g_pSendConfigurationString2= NULL;
_SendSetIODialogHwnd* g_pSendSetIODialogHwnd= NULL;
_SendXmitDialogHwnd* g_pSendXmitDialogHwnd= NULL;
_SendSpecialEvent* g_pSendSpecialEvent= NULL;
_SendStartOfFrame* g_pSendStartOfFrame= NULL;
_SendEndOfFrame* g_pSendEndOfFrame= NULL;
_SendAbortedFrame* g_pSendAbortedFrame= NULL;
_SendByteSimple* g_pSendByteSimple= NULL;
_CheckForMessages* g_pCheckForMessages= NULL;
_GetAppVersionNumber* g_pGetAppVersionNumber= NULL;
_GetAppSerialNumber* g_pGetAppSerialNumber= NULL;
_GetAppDisplayedConfigurationName* g_pGetAppDisplayedConfigurationName= NULL;
_GetSerialNumberSectionKeyValuePairs* g_pGetSerialNumberSectionKeyValuePairs= NULL;
_GetDriverSavePath* g_pGetDriverSavePath= NULL;
_GetDriverSaveName* g_pGetDriverSaveName= NULL;
_RegisterNotification* g_pRegisterNotification= NULL;
_RegisterNotification2* g_pRegisterNotification2= NULL;
_SendNotification* g_pSendNotification= NULL;
_SendNotification2* g_pSendNotification2= NULL;
_SendArraySimple* g_pSendArraySimple= NULL;
_SendStringSimple* g_pSendStringSimple= NULL;
_SendNumberOfLostMessages* g_pSendNumberOfLostMessages= NULL;
_UpdateStat* g_pUpdateStat= NULL;
_FramesLost* g_pFramesLost= NULL;
_FramesLost2* g_pFramesLost2= NULL;
_SetExePath* g_pSetExePath= NULL;
_SendPostNotify* g_pSendPostNotify= NULL;
_SendComment* g_pSendComment= NULL;
_AddNamedData* g_pAddNamedData= NULL;
_SetDataSourceFilterName* g_pSetDataSourceFilterName= NULL;;
_SendQueueStats* g_pSendQueueStats= NULL;
_SaveAndClear* g_pSaveAndClear= NULL;
_SaveCapture* g_pSaveCapture= NULL;
_ClearCapture* g_pClearCapture= NULL;
_ExportHtmlToPath* g_pExportHtmlToPath= NULL;
_GetLiveImportInitializationStatus* g_pGetLiveImportInitializationStatus= NULL;
_SendSpectrumPathName* g_pSendSpectrumPathName= NULL;
_SendStartingTimestamp* g_pSendStartingTimestamp= NULL;
_GetNumFreeFrameBytes* g_pGetNumFreeFrameBytes= NULL;
_GetNumAppEventEnqueueBusyWaits* g_pGetNumAppEventEnqueueBusyWaits= NULL;

// add for pas 15.17
typedef HRESULT(_GeneralAPI)(int p1, int p2, int p3, int p4);
_GeneralAPI* g_pGlobalInterfaceCheck = NULL;
_SendPower* g_pSendPower = NULL;

// Library definitions:
TCHAR* g_pszLibraryName= _T("LiveImportAPI_pas.dll");
//TCHAR* g_pszLibraryName = _T("C:\\Program Files (x86)\\Frontline Test System II\\Frontline 15.17\\Executables\\Core\\LiveImportAPI.dll");
HMODULE g_hLiveImportAPI= NULL;

bool ShowFailMessage(char* pszProcName)
{
	TCHAR szError[1024]= {0};
	_stprintf_s(szError, _countof(szError), _T("Failed to get address of function \"%s\"."), (TCHAR*)pszProcName);
	MessageBox(NULL, szError, _T("Error"), MB_OK);
	FreeLibrary(g_hLiveImportAPI);
	return (false);
}

bool LoadAPIFunctions(void)
{
	char* pszProcName= NULL;
	
	pszProcName= "InitializeLiveImport";
	g_pInitializeLiveImport= (_InitializeLiveImport*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pInitializeLiveImport)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "InitializeLiveImportEx";
	g_pInitializeLiveImportEx= (_InitializeLiveImportEx*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pInitializeLiveImportEx)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SetEnqueueMode";
	g_pSetEnqueueMode= (_SetEnqueueMode*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSetEnqueueMode)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "ReleaseLiveImport";
	g_pReleaseLiveImport= (_ReleaseLiveImport*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pReleaseLiveImport)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "StillAlive";
	g_pStillAlive= (_StillAlive*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pStillAlive)
		return (ShowFailMessage(pszProcName));

	pszProcName= "IsAppReady";
	g_pIsAppReady= (_IsAppReady*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pIsAppReady)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SendFrame";
	g_pSendFrame= (_SendFrame*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendFrame)
		return (ShowFailMessage(pszProcName));

	pszProcName= "SendFrame2";
	g_pSendFrame2= (_SendFrame2*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendFrame2)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SendFrameWithComment";
	g_pSendFrameWithComment= (_SendFrameWithComment*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendFrameWithComment)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SendFrameWithCommentFromDatasource";
	g_pSendFrameWithCommentFromDatasource= (_SendFrameWithCommentFromDatasource*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendFrameWithCommentFromDatasource)
		return (ShowFailMessage(pszProcName));

	pszProcName= "SendKeepAliveEvent";
	g_pSendKeepAliveEvent= (_SendKeepAliveEvent*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendKeepAliveEvent)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SendEvent";
	g_pSendEvent= (_SendEvent*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendEvent)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SendControlSignalChange";
	g_pSendControlSignalChange= (_SendControlSignalChange*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendControlSignalChange)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SendNondataFlagChange";
	g_pSendNondataFlagChange= (_SendNondataFlagChange*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendNondataFlagChange)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SendBreak";
	g_pSendBreak= (_SendBreak*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendBreak)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SendFlowControl";
	g_pSendFlowControl= (_SendFlowControl*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendFlowControl)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SendConnectionStatus";
	g_pSendConnectionStatus= (_SendConnectionStatus*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendConnectionStatus)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SendConfigurationString";
	g_pSendConfigurationString= (_SendConfigurationString*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendConfigurationString)
		return (ShowFailMessage(pszProcName));

	pszProcName= "SendConfigurationString2";
	g_pSendConfigurationString2= (_SendConfigurationString2*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendConfigurationString2)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SendSetIODialogHwnd";
	g_pSendSetIODialogHwnd= (_SendSetIODialogHwnd*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendSetIODialogHwnd)
		return (ShowFailMessage(pszProcName));

	pszProcName= "SendXmitDialogHwnd";
	g_pSendXmitDialogHwnd= (_SendXmitDialogHwnd*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendXmitDialogHwnd)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SendSpecialEvent";
	g_pSendSpecialEvent= (_SendSpecialEvent*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendSpecialEvent)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SendControlSignalChange";
	g_pSendControlSignalChange= (_SendControlSignalChange*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendControlSignalChange)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SendStartOfFrame";
	g_pSendStartOfFrame= (_SendStartOfFrame*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendStartOfFrame)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SendEndOfFrame";
	g_pSendEndOfFrame= (_SendEndOfFrame*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendEndOfFrame)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SendAbortedFrame";
	g_pSendAbortedFrame= (_SendAbortedFrame*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendAbortedFrame)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SendByteSimple";
	g_pSendByteSimple= (_SendByteSimple*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendByteSimple)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "CheckForMessages";
	g_pCheckForMessages= (_CheckForMessages*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pCheckForMessages)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "GetAppVersionNumber";
	g_pGetAppVersionNumber= (_GetAppVersionNumber*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pGetAppVersionNumber)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "GetAppSerialNumber";
	g_pGetAppSerialNumber= (_GetAppSerialNumber*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pGetAppSerialNumber)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "GetAppDisplayedConfigurationName";
	g_pGetAppDisplayedConfigurationName= (_GetAppDisplayedConfigurationName*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pGetAppDisplayedConfigurationName)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "GetSerialNumberSectionKeyValuePairs";
	g_pGetSerialNumberSectionKeyValuePairs= (_GetSerialNumberSectionKeyValuePairs*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pGetSerialNumberSectionKeyValuePairs)
		return (ShowFailMessage(pszProcName));

    pszProcName= "GetDriverSavePath";
    g_pGetDriverSavePath= (_GetDriverSavePath*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pGetDriverSavePath)
		return (ShowFailMessage(pszProcName));

    pszProcName= "GetDriverSaveName";
    g_pGetDriverSaveName= (_GetDriverSaveName*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pGetDriverSaveName)
		return (ShowFailMessage(pszProcName));

	pszProcName= "RegisterNotification";
	g_pRegisterNotification= (_RegisterNotification*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pRegisterNotification)
		return (ShowFailMessage(pszProcName));

	pszProcName= "RegisterNotification2";
	g_pRegisterNotification2= (_RegisterNotification2*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pRegisterNotification2)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SendNotification";
	g_pSendNotification= (_SendNotification*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendNotification)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SendArraySimple";
	g_pSendArraySimple= (_SendArraySimple*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendArraySimple)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SendStringSimple";
	g_pSendStringSimple= (_SendStringSimple*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendStringSimple)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SendNumberOfLostMessages";
	g_pSendNumberOfLostMessages= (_SendNumberOfLostMessages*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendNumberOfLostMessages)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "UpdateStat";
	g_pUpdateStat= (_UpdateStat*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pUpdateStat)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "FramesLost";
	g_pFramesLost= (_FramesLost*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pFramesLost)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SetExePath";
	g_pSetExePath= (_SetExePath*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSetExePath)
		return (ShowFailMessage(pszProcName));

	pszProcName= "SendPostNotify";
	g_pSendPostNotify= (_SendPostNotify*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendPostNotify)
		return (ShowFailMessage(pszProcName));

	pszProcName= "SendComment";
	g_pSendComment= (_SendComment*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendComment)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "AddNamedData";
	g_pAddNamedData= (_AddNamedData*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pAddNamedData)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SetDataSourceFilterName";
	g_pSetDataSourceFilterName= (_SetDataSourceFilterName*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSetDataSourceFilterName)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SendQueueStats";
	g_pSendQueueStats= (_SendQueueStats*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendQueueStats)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SaveAndClear";
	g_pSaveAndClear= (_SaveAndClear*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSaveAndClear)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SaveCapture";
	g_pSaveCapture= (_SaveCapture*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSaveCapture)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "ClearCapture";
	g_pClearCapture= (_ClearCapture*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pClearCapture)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "ExportHtmlToPath";
	g_pExportHtmlToPath= (_ExportHtmlToPath*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pExportHtmlToPath)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "GetLiveImportInitializationStatus";
	g_pGetLiveImportInitializationStatus= (_GetLiveImportInitializationStatus*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pGetLiveImportInitializationStatus)
		return (ShowFailMessage(pszProcName));
	
	pszProcName= "SendSpectrumPathName";
	g_pSendSpectrumPathName= (_SendSpectrumPathName*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendSpectrumPathName)
		return (ShowFailMessage(pszProcName));

	pszProcName= "SendStartingTimestamp";
	g_pSendStartingTimestamp= (_SendStartingTimestamp*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pSendStartingTimestamp)
		return (ShowFailMessage(pszProcName));
    
  	pszProcName= "GetNumFreeFrameBytes";
	g_pGetNumFreeFrameBytes= (_GetNumFreeFrameBytes*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pGetNumFreeFrameBytes)
		return (ShowFailMessage(pszProcName));

   	pszProcName= "GetNumAppEventEnqueueBusyWaits";
	g_pGetNumAppEventEnqueueBusyWaits= (_GetNumAppEventEnqueueBusyWaits*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if(NULL == g_pGetNumAppEventEnqueueBusyWaits)
		return (ShowFailMessage(pszProcName));

	// add for pas 15.17

	pszProcName = "GlobalInterfaceCheck";
	g_pGlobalInterfaceCheck = (_GeneralAPI*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if (NULL == g_pGlobalInterfaceCheck)
		return (ShowFailMessage(pszProcName));

	pszProcName = "SendPower";
	g_pSendPower = (_SendPower*)GetProcAddress(g_hLiveImportAPI, pszProcName);
	if (NULL == g_pSendPower)
		return (ShowFailMessage(pszProcName));

	return (true);
}

void NullLiveImportFunctionPointers(void);
bool LoadLiveImportAPIFunctions(HMODULE hModule)
{
	TCHAR szBuf[1024];

	if (!_tcsrchr(g_pszLibraryName, '\\'))
	{
		if (GetModuleFileName(hModule, szBuf, _countof(szBuf)))
		{
			LPTSTR psz = _tcsrchr(szBuf, '\\') + 1;
			_tcscpy_s(psz, szBuf + _countof(szBuf) - psz, g_pszLibraryName);
			g_pszLibraryName = szBuf;
		}
	}

	g_hLiveImportAPI= LoadLibrary(g_pszLibraryName);
	if(NULL == g_hLiveImportAPI)
	{
		DWORD lastError = GetLastError();
		_stprintf_s(szBuf, _countof(szBuf), _T("Failed to load module \"%s\" - error %d"), g_pszLibraryName, lastError);
		MessageBox(NULL, szBuf, _T("Error"), MB_OK);
		return (false);
	}
	
	bool rc = LoadAPIFunctions();
	if (!rc)
	{
		NullLiveImportFunctionPointers();
	}
	return rc;
}

void NullLiveImportFunctionPointers(void)
{
	if (g_hLiveImportAPI)
	{
		FreeLibrary(g_hLiveImportAPI);
		g_hLiveImportAPI = NULL;
	}
	g_pGetDllVersion= NULL;
	g_pInitializeLiveImport= NULL;
	g_pReleaseLiveImport= NULL;
	g_pStillAlive= NULL;
	g_pIsAppReady= NULL;
	g_pSendFrame= NULL;
	g_pSendEvent= NULL;
	g_pSendControlSignalChange= NULL;
	g_pSendNondataFlagChange= NULL;
	g_pSendBreak= NULL;
	g_pSendFlowControl= NULL;
	g_pSendConnectionStatus= NULL;
	g_pSendConfigurationString= NULL;
	g_pSendSetIODialogHwnd= NULL;
	g_pSendXmitDialogHwnd= NULL;
	g_pSendSpecialEvent= NULL;
	g_pSendStartOfFrame= NULL;
	g_pSendEndOfFrame= NULL;
	g_pSendAbortedFrame= NULL;
	g_pSendByteSimple= NULL;
	g_pCheckForMessages= NULL;
	g_pGetAppVersionNumber= NULL;
	g_pGetAppSerialNumber= NULL;
	g_pGetAppDisplayedConfigurationName= NULL;
	g_pGetSerialNumberSectionKeyValuePairs= NULL;
	g_pGetDriverSavePath= NULL;
	g_pGetDriverSaveName= NULL;
	g_pRegisterNotification= NULL;
	g_pSendNotification= NULL;
	g_pSendArraySimple= NULL;
	g_pSendStringSimple= NULL;
	g_pSendNumberOfLostMessages= NULL;
	g_pUpdateStat= NULL;
	g_pFramesLost= NULL;
	g_pSetExePath= NULL;
	g_pSendPostNotify = NULL;

	g_pSendFrame2 = NULL;
	g_pSendFrameWithComment= NULL;
	g_pSendFrameWithCommentFromDatasource= NULL;
	g_pSendKeepAliveEvent= NULL;
	g_pFramesLost2= NULL;
	g_pSendConfigurationString2= NULL;
	g_pSendNotification2= NULL;
	g_pRegisterNotification2 = NULL;
	g_pSendQueueStats = NULL;
	g_pSaveAndClear = NULL;
	g_pSaveCapture= NULL;
	g_pClearCapture= NULL;
	g_pExportHtmlToPath= NULL;
	g_pGetLiveImportInitializationStatus= NULL;
	g_pSendSpectrumPathName= NULL;
	g_pSendStartingTimestamp= NULL;
	g_pGetNumFreeFrameBytes= NULL;
	g_pGetNumAppEventEnqueueBusyWaits= NULL;

	// add for pas 15.17
	g_pGlobalInterfaceCheck = NULL;
	g_pSendPower = NULL;
}

#endif

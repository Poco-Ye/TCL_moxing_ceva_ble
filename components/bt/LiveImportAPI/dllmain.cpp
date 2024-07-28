// dllmain.cpp : 定义 DLL 应用程序的入口点。
#include "stdafx.h"

BOOL LoadLiveImport(HMODULE hModule);
void UnloadLiveImport(HMODULE hModule);

BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
					 )
{
	BOOL rc = TRUE;
	
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
		rc = LoadLiveImport(hModule);
		break;
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
		break;
	case DLL_PROCESS_DETACH:
		UnloadLiveImport(hModule);
		break;
	}

	return rc;
}


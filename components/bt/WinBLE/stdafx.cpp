
// stdafx.cpp : ֻ������׼�����ļ���Դ�ļ�
// WinBLE.pch ����ΪԤ����ͷ
// stdafx.obj ������Ԥ����������Ϣ

#include "stdafx.h"

#include <crtdbg.h>

#define TRACE_BUF_SIZE 1024

extern "C" int win_trace(LPCTSTR psz, ...)
{
	TCHAR szBuf[1024];
	INT rc;
	va_list arglist;

	va_start(arglist, psz);
	rc = _vsnprintf_s(szBuf, TRACE_BUF_SIZE, TRACE_BUF_SIZE, psz, arglist);
	va_end(arglist);

	OutputDebugString(szBuf);

	return rc;

}

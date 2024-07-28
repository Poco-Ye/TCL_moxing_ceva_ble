
// stdafx.cpp : 只包括标准包含文件的源文件
// WinBLE.pch 将作为预编译头
// stdafx.obj 将包含预编译类型信息

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

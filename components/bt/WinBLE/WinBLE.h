
// WinBLE.h : WinBLE Ӧ�ó������ͷ�ļ�
//
#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"       // ������


// CWinBLEApp:
// �йش����ʵ�֣������ WinBLE.cpp
//

class CWinBLEApp : public CWinApp
{
public:
	CWinBLEApp();


// ��д
public:
	virtual BOOL InitInstance();
	virtual int ExitInstance();

// ʵ��

public:
	afx_msg void OnAppAbout();
	DECLARE_MESSAGE_MAP()
};

extern CWinBLEApp theApp;

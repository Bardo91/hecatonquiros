// ServoPC.h : main header file for the SERVOPC application
//

#pragma once

#ifndef __AFXWIN_H__
	#error include 'stdafx.h' before including this file for PCH
#endif

#include "resource.h"		// main symbols

/////////////////////////////////////////////////////////////////////////////
// CServoPCApp:
// See ServoPC.cpp for the implementation of this class
//

class CServoPCApp : public CWinApp
{
public:
	CServoPCApp();

	public:
	virtual BOOL InitInstance();

	DECLARE_MESSAGE_MAP()
};

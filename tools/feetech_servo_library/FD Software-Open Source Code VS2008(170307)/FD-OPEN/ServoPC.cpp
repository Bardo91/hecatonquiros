// ServoPC.cpp : Defines the class behaviors for the application.
//

#include "stdafx.h"
#include "ServoPC.h"
#include "ServoMain.h"
#include "SCServo/SCServo.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CServoPCApp

BEGIN_MESSAGE_MAP(CServoPCApp, CWinApp)
	ON_COMMAND(ID_HELP, CWinApp::OnHelp)
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CServoPCApp construction

CServoPCApp::CServoPCApp()
{
	// TODO: add construction code here,
	// Place all significant initialization in InitInstance
}

/////////////////////////////////////////////////////////////////////////////
// The one and only CServoPCApp object

CServoPCApp theApp;

/////////////////////////////////////////////////////////////////////////////
// CServoPCApp initialization
BOOL CServoPCApp::InitInstance()
{
	AfxEnableControlContainer();

	CServoMain * pServoMainDlg;
	pServoMainDlg = new CServoMain;
	m_pMainWnd = pServoMainDlg;
	SCServo	*pServo =  new SCServo;
	CSCComm *pCom = new CSCComm;
	pServo->pSerial = pCom;
	pServoMainDlg->pServo = pServo;
	pServoMainDlg->Create(IDD_SERVOMAIN_DIALOG);
	

	return TRUE;
}

// ServoPG.cpp : implementation file
//

#include "stdafx.h"
#include "resource.h"
#include "ServoTest.h"
#include "SCServo/SCServo.h"
#include "SCServo/DataCal.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// ServoTest dialog

ServoTest::ServoTest(SCServo *pServo)
{
	this->pServo = pServo;
	IsTranTest = 0 ;
	TraInfoStr = "";
	TranDatSize = 0;
	memset(TranErrNum, 0, sizeof(TranErrNum));
}

void ServoTest::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_BTN_UPGWRITE, m_BtnWrite);
	DDX_Control(pDX, IDC_PROGRESS_UPG, m_UpgradeCtl);
	DDX_Control(pDX, IDC_EDIT_FIRMWARE, m_FirmCtl);
	DDX_Control(pDX, IDC_EDIT_ID, m_VerID);
	DDX_Control(pDX, IDC_STATIC_TRANCOUNT, m_TranCount);

	DDX_Control(pDX, IDC_EDIT_TRANTOUT, m_TranTimeOut);
	DDX_Control(pDX, IDC_EDIT_TRAN_INFO, m_TranInfo);
	DDX_Control(pDX, IDC_EDIT_TRANID, m_TranID);
	DDX_Control(pDX, IDC_EDIT_IDTRANN, m_TranIDN);
	DDX_Control(pDX, IDC_BTN_TRANTEST, m_TranTest);
}


BEGIN_MESSAGE_MAP(ServoTest, CDialog)
	ON_BN_CLICKED(IDC_BTN_UPGOPEN, &ServoTest::OnBnClickedBtnUpgopen)
	ON_BN_CLICKED(IDC_BTN_UPGWRITE, &ServoTest::OnBnClickedBtnUpgwrite)
	ON_BN_CLICKED(IDC_BTN_READ, &ServoTest::OnBnClickedBtnRead)
	ON_BN_CLICKED(IDC_BTN_TRANTEST, &ServoTest::OnBnClickedBtnTrantest)
	ON_BN_CLICKED(IDC_BTN_TRANCLR, &ServoTest::OnBnClickedBtnTranclr)
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// ServoTest message handlers


BOOL ServoTest::OnInitDialog()
{
	CDialog::OnInitDialog();
	m_BtnWrite.EnableWindow(FALSE);
	m_FirmCtl.SetWindowText(_T(""));
	m_TranCount.SetWindowText("Transfer:0Byte");
	m_TranTimeOut.SetWindowText("50");
	m_TranInfo.SetWindowText("");
	m_TranID.SetWindowText("1");
	m_TranIDN.SetWindowText("17");
	SetCurID(0);
	return TRUE;
}


void ServoTest::OnOK()
{}
void ServoTest::OnCancel()
{}


//-----------------------upgrade-----------------

void ServoTest::OnBnClickedBtnUpgopen()
{
}

void ServoTest::OnBnClickedBtnUpgwrite()
{
}
//-----------------------upgrade-----------------


void ServoTest::OnBnClickedBtnRead()
{
	// TODO: 在此添加控件通知处理程序代码

	CString Str;
	m_VerID.GetWindowText(Str);
	UCHAR ID  = atoi(Str.GetBuffer());

	int FVM = pServo->readByte(ID, P_VERSION_L);
	int FVS = pServo->readByte(ID, P_VERSION_H);
	if(FVM==-1 || FVS==-1){
		goto error;
		return;
	}
	Str.Format("%d.%d", FVM, FVS);
	m_FirmCtl.SetWindowText(Str);
	return;
error:
	MessageBox("Read Firmware Error\n");
}

void ServoTest::SetCurID(UINT ID)
{
	CString EditStr;
	EditStr.Format("%d", ID);
	m_VerID.SetWindowText(EditStr);
}

UINT ThreadTranFunc(LPVOID lpParam)
{
	ServoTest *pDlg = (ServoTest *)lpParam;
	SCServo *pServo = pDlg->pServo;
	CSCComm *pSerial = pDlg->pServo->pSerial;

	int TimeOut;
	CString Str;
	
	int addST = 0;
	int addEND = P_PRESENT_GOALPOS_H;
	int len = addEND-addST+1;
	int res;
	int DatLen;
	UCHAR *Mem = new UCHAR[len];

	pDlg->m_TranTimeOut.GetWindowText(Str);
	TimeOut = atoi(Str.GetBuffer());
	int IOTimeOut = pSerial->IOTimeOut;
	pSerial->SetupTimeOut(TimeOut);
	int ID1,IDn;
	pDlg->m_TranID.GetWindowText(Str);
	ID1 = atoi(Str.GetBuffer());
	pDlg->m_TranIDN.GetWindowText(Str);
	IDn = atoi(Str.GetBuffer());

	//获得计时器的时钟频率
	LARGE_INTEGER litmp;
	LONGLONG QPart1,Qpart2;
	double dfMinus,dfFreq,dfTime;
	QueryPerformanceFrequency(&litmp);
	dfFreq = (double)litmp.QuadPart;
	while(1){
		int ID = ID1;
		while(ID<=IDn){
			QueryPerformanceCounter(&litmp);
			QPart1 = litmp.QuadPart; //开始计时
			res = pServo->Read(ID, addST, Mem, len);
			QueryPerformanceCounter(&litmp);
			Qpart2 = litmp.QuadPart; //终止计时
			dfMinus = (double)(Qpart2-QPart1);//计算计数器值
			dfTime = (dfMinus*1000000)/dfFreq;//获得对应时间us
			if(res){
				TRACE("Tran OK\n");
				DatLen = len+8+6;
				pDlg->WriteTranInfo(ID, (int)dfTime, DatLen);
				
			}else{
				TRACE("Tran Error\n");
				DatLen = 0;
				pDlg->WriteTranInfo(ID, (int)dfTime);
			}
			ID++;
		}
		pDlg->WriteTranInfo();
		pDlg->GetTranCount(DatLen);
		if(pDlg->IsTranTest==0){
			TRACE("Tran Break\n");
			break;
		}
	}
	delete[] Mem;
	pSerial->SetupTimeOut(IOTimeOut);
	return 0;
}

void ServoTest::WriteTranInfo(int ID, int TimeOut, int DatLen)
{
	CString Str;
	int Speed = (DatLen*8000000)/TimeOut;
	Str.Format("ID:%d,Time:%dus  Error:%d %.2fkb/s\r\n", ID, TimeOut, TranErrNum[ID], (float)Speed/1000);
	TraInfoStr += Str;
}

void ServoTest::GetTranCount(int DatLen)
{
	CString Str;
	TranDatSize += DatLen;
	Str.Format("Transfer:%dByte", TranDatSize);
	m_TranCount.SetWindowText(Str);
}

void ServoTest::WriteTranInfo()
{
	m_TranInfo.SetWindowText(TraInfoStr);
	TraInfoStr = "";
}

void ServoTest::WriteTranInfo(int ID, int TimeOut)
{
	CString Str;
	TranErrNum[ID]++;
	Str.Format("ID:%d,Time:%dus  Error:%d\r\n", ID, TimeOut, TranErrNum[ID]);
	TraInfoStr += Str;
}

void ServoTest::OnBnClickedBtnTrantest()
{
	// TODO: 在此添加控件通知处理程序代码
	CString Str;
	m_TranTest.GetWindowText(Str);
	if(Str=="Test"){
		m_TranTest.SetWindowText("Stop");
		IsTranTest = 1;
		AfxBeginThread(ThreadTranFunc,LPVOID(this));
	}else{
		m_TranTest.SetWindowText("Test");
		IsTranTest = 0;
	}
}

void ServoTest::OnBnClickedBtnTranclr()
{
	// TODO: 在此添加控件通知处理程序代码
	m_TranInfo.SetWindowText("");
	memset(TranErrNum, 0, sizeof(TranErrNum));
	TranDatSize = 0;
}

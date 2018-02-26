#include "stdafx.h"
#include "resource.h"
#include "RobotDlg.h"
#include "SCServo/SCServo.h"
#include "SCServo/DataCal.h"


#pragma pack(1)
struct
{
	UINT16  CurPos;//当前位置 //56 57
	UINT16	CurSpeed;//当前速度 //58 59
	UINT16	CurLoad;//当前负载 //60 61
	UINT8   CurVoltage;//当前电压 //62
	UINT8   CurHeat;//当前温度 //63
	UINT8   Registered;//
	UINT8   Error;
	UINT8   Moving;//66
	UINT16	CurGoalPos;//67 68
	UINT16	Current;//69 70
} Mem;
#pragma pack()

void WINAPI TimerCallback(UINT uTimerID, UINT uMsg, DWORD dwUser, DWORD dw1, DWORD dw2)
{
	CRobotDlg *pRobotDlg = (CRobotDlg *)dwUser;
	SCServo *pServo = pRobotDlg->pServo;
	int PosShit = pRobotDlg->PosShit;
	int addST = P_PRESENT_POSITION_L;
	int addEND;
	if(pServo->End){
		addEND = P_PRESENT_GOALPOS_H;
		Mem.Current = 0;
	}else{
		addEND = P_PRESENT_CURRENT_H;
	}
	int len = addEND-addST+1;
	pRobotDlg->Cs.Lock();
	int res = pServo->Read(pRobotDlg->SelID, addST, (UCHAR*)&Mem, len);
	pRobotDlg->Cs.Unlock();

	if(res==0){
		TRACE("Read Error\n");
		return;
	}

	CString str;
	INT dat;

	if(pServo->End){
		DataCal::SwapHex(Mem.CurPos);
	}
	str.Format("%d", Mem.CurPos);
	pRobotDlg->m_LabCurPos.SetWindowTextA(str);
	pRobotDlg->m_PIC.AddBlackLineDat(Mem.CurPos>>PosShit);

	if(pServo->End){
		DataCal::SwapHex(Mem.CurSpeed);
	}
	dat = DataCal::scs2host(Mem.CurSpeed, 15);
	str.Format("%d", dat);
	pRobotDlg->m_LabSpeed.SetWindowTextA(str);
	pRobotDlg->m_PIC.AddGreenLineDat(500+(dat>>1));

	if(pServo->End){
		DataCal::SwapHex(Mem.CurLoad);
	}
	dat = DataCal::scs2host(Mem.CurLoad);
	str.Format("%d", dat);
	pRobotDlg->m_LabLoad.SetWindowTextA(str);
	pRobotDlg->m_PIC.AddRedLineDat(500+(dat>>1));

	if(pServo->End){
		DataCal::SwapHex(Mem.CurGoalPos);
	}
	str.Format("%d", Mem.CurGoalPos);
	pRobotDlg->m_LabGoalPos.SetWindowTextA(str);
	pRobotDlg->m_PIC.AddBlueLineDat(Mem.CurGoalPos>>PosShit);

	dat = DataCal::scs2host(Mem.Current, 15);
	str.Format("%d", dat);
	pRobotDlg->m_LabCurrent.SetWindowTextA(str);
	pRobotDlg->m_PIC.AddYellowLineDat(500+(dat>>1));

	str.Format("%d.%d V", Mem.CurVoltage/10, Mem.CurVoltage%10);
	pRobotDlg->m_LabVol.SetWindowTextA(str);

	str.Format("%d Deg", Mem.CurHeat);
	pRobotDlg->m_LabTmp.SetWindowTextA(str);


	str.Format("%d", Mem.Moving);
	pRobotDlg->m_LabMoving.SetWindowTextA(str);

	str.Format("0x%x", Mem.Error);
	pRobotDlg->m_LabError.SetWindowTextA(str);

	pRobotDlg->m_PIC.Invalidate(FALSE);
}


CRobotDlg::CRobotDlg(SCServo *pServo)
{
	this->pServo = pServo;
	TimeID = -1;
	IDEND = 10;
	m_WriteFun = 0;
	PosShit = 0;
	AdcMax = 1023;
}

CRobotDlg::~CRobotDlg()
{
}

void CRobotDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_PIC_DRAW, m_PIC);
	DDX_Control(pDX, IDC_STATIC_VIRPOS, m_LabGoalPos);
	DDX_Control(pDX, IDC_STATIC_TEMP, m_LabTmp);
	DDX_Control(pDX, IDC_STATIC_VOL, m_LabVol);
	DDX_Control(pDX, IDC_STATIC_LOAD, m_LabLoad);
	DDX_Control(pDX, IDC_STATIC_SPEED, m_LabSpeed);
	DDX_Control(pDX, IDC_STATIC_POS, m_LabCurPos);
	DDX_Control(pDX, IDC_STATIC_CURRENT, m_LabCurrent);
	DDX_Control(pDX, IDC_STATIC_MOVING, m_LabMoving);
	DDX_Control(pDX, IDC_STATIC_ERROR, m_LabError);
	DDX_Control(pDX, IDC_SLIDER_POS, m_SlidPos);
	DDX_Control(pDX, IDC_EDIT_T, m_EditT);
	DDX_Control(pDX, IDC_EDIT_V, m_EditV);
	DDX_Control(pDX, IDC_EDIT_GOALPOS, m_EditPos);
	DDX_Radio(pDX, IDC_RADIO_WRITE, m_WriteFun);
	DDX_Control(pDX, IDC_SLIDER_ZOOMX, m_SlidZoomX);
	DDX_Control(pDX, IDC_SLIDER_HORIZ, m_SlidHoriz);
}


BEGIN_MESSAGE_MAP(CRobotDlg, CDialog)
	ON_WM_HSCROLL()
	ON_BN_CLICKED(IDC_CHECK_TORQUEEN, &CRobotDlg::OnBnClickedCheckTorqueen)
	ON_BN_CLICKED(IDC_BTN_DRAW, &CRobotDlg::OnBnClickedBtnDraw)
	ON_EN_CHANGE(IDC_EDIT_ID, &CRobotDlg::OnEnChangeEditId)
	ON_BN_CLICKED(IDC_RADIO_WRITE, &CRobotDlg::OnBnClickedRadioWrite)
	ON_BN_CLICKED(IDC_RADIO_SYSWRITE, &CRobotDlg::OnBnClickedRadioSyswrite)
	ON_BN_CLICKED(IDC_RADIO_REGWITE, &CRobotDlg::OnBnClickedRadioRegwite)
	ON_BN_CLICKED(IDC_BTN_ACTION, &CRobotDlg::OnBnClickedBtnAction)
	ON_EN_CHANGE(IDC_EDIT_IDN, &CRobotDlg::OnEnChangeEditIdn)
	ON_BN_CLICKED(IDC_CHECK_CURPOS, &CRobotDlg::OnBnClickedCheckCurpos)
	ON_BN_CLICKED(IDC_CHECK_TROQUE, &CRobotDlg::OnBnClickedCheckTroque)
	ON_BN_CLICKED(IDC_CHECK_SPEED, &CRobotDlg::OnBnClickedCheckSpeed)
	ON_BN_CLICKED(IDC_CHECK_VIRPOS, &CRobotDlg::OnBnClickedCheckVirpos)
	ON_BN_CLICKED(IDC_CHECK_CURRENT, &CRobotDlg::OnBnClickedCheckCurrent)
	ON_BN_CLICKED(IDC_BTN_POS_SET, &CRobotDlg::OnBnClickedBtnPosSet)
END_MESSAGE_MAP()



BOOL CRobotDlg::OnInitDialog()
{
	CDialog::OnInitDialog();


	m_SlidPos.SetRange(0, AdcMax);
	m_SlidPos.SetPos(AdcMax>>1);

	m_LabCurPos.SetWindowText(_T("0"));
	m_LabSpeed.SetWindowText(_T("0"));
	m_LabLoad.SetWindowText(_T("0"));
	m_LabVol.SetWindowText(_T("0.0 V"));
	m_LabTmp.SetWindowText(_T("0 Deg"));
	m_LabGoalPos.SetWindowText(_T("0"));
	m_LabCurrent.SetWindowText("0");
	m_LabMoving.SetWindowText("0");
	m_LabError.SetWindowText("0x0");

	CEdit *pEditID = (CEdit *)GetDlgItem(IDC_EDIT_ID);
	pEditID->SetLimitText(3);
	SetCurID(0);

	CEdit *pEdirCycle = (CEdit *)GetDlgItem(IDC_EDIT_CYCLE);
	pEdirCycle->SetLimitText(3);
	pEdirCycle->SetWindowText(_T("10"));

	CButton * BtnAct = (CButton *)GetDlgItem(IDC_BTN_ACTION);
	BtnAct->EnableWindow(FALSE);

	CEdit *pEditIDN = (CEdit *)GetDlgItem(IDC_EDIT_IDN);
	pEditIDN->SetLimitText(3);
	pEditIDN->SetWindowText(_T("10"));
	pEditIDN->EnableWindow(FALSE);

	m_SlidZoomX.SetRange(8, 64);
	m_SlidZoomX.SetPos(8);
	m_SlidHoriz.SetRange(0, 2000);
	m_SlidHoriz.SetPos(0);

	m_EditT.SetLimitText(6);
	m_EditV.SetLimitText(6);

	m_EditT.SetWindowText("0");
	m_EditV.SetWindowText("1000");


	m_EditPos.SetLimitText(4);
	CString tempStr;
	tempStr.Format("%d",AdcMax>>1);
	m_EditPos.SetWindowText(tempStr);

	CButton	* pCheckCtl = (CButton *)GetDlgItem(IDC_CHECK_CURPOS);
	pCheckCtl->SetCheck(TRUE);
	pCheckCtl = (CButton *)GetDlgItem(IDC_CHECK_TROQUE);
	pCheckCtl->SetCheck(TRUE);
	pCheckCtl = (CButton *)GetDlgItem(IDC_CHECK_SPEED);
	pCheckCtl->SetCheck(TRUE);
	pCheckCtl = (CButton *)GetDlgItem(IDC_CHECK_VIRPOS);
	pCheckCtl->SetCheck(TRUE);
	pCheckCtl = (CButton *)GetDlgItem(IDC_CHECK_CURRENT);
	pCheckCtl->SetCheck(TRUE);

	m_PIC.InitPic(1);
	return TRUE;
}

void CRobotDlg::TimeStart(UINT time)
{
	if(TimeID==-1){
		TimeID = timeSetEvent(time, 1, (LPTIMECALLBACK)TimerCallback, (DWORD)this, TIME_PERIODIC);
	}
}

void CRobotDlg::TimeStop()
{
	if(TimeID!=-1){
		timeKillEvent(TimeID);
		CButton	*pDrawBtn = (CButton *)GetDlgItem(IDC_BTN_DRAW);
		pDrawBtn->SetWindowText(_T("Draw"));
	}
	TimeID = -1;
}

void CRobotDlg::OnOK()
{
	//MessageBox("OK");
}

void CRobotDlg::OnCancel()
{
	//MessageBox("Cancel");
}


void CRobotDlg::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	UpdateData();
	int CtlID = pScrollBar->GetDlgCtrlID();
	if(CtlID==IDC_SLIDER_HORIZ){
		int Horiz = m_SlidHoriz.GetPos();
		m_PIC.SetHoriz(Horiz);
		m_PIC.Invalidate(FALSE);
	}
	if(CtlID==IDC_SLIDER_ZOOMX){
		int Vx = m_SlidZoomX.GetPos();
		m_PIC.SetVx(Vx);
		m_PIC.Invalidate(FALSE);
	}
	if(CtlID==IDC_SLIDER_POS){
		int Pos = m_SlidPos.GetPos();
		CString str;
		str.Format("%d", Pos);
		m_EditPos.SetWindowText(str);
		m_EditT.GetWindowText(str);
		int T = atoi(str.GetBuffer());
		m_EditV.GetWindowText(str);
		int V = atoi(str.GetBuffer());
		if(V<0){
			V = -V;
			V |= (1<<15);
		}
		if(T<0){
			T = -T;
			T |= (1<<10);
		}
		PostitonSet(Pos, V, T, m_WriteFun);

	}
	CDialog::OnHScroll(nSBCode, nPos, pScrollBar);
}
void CRobotDlg::OnBnClickedCheckTorqueen()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData();
	CButton	* pEnableCtl = (CButton *)GetDlgItem(IDC_CHECK_TORQUEEN);
	UCHAR En = pEnableCtl->GetCheck();
	
	ToruqeEn(En, m_WriteFun);
}

void CRobotDlg::OnBnClickedBtnDraw()
{
	// TODO: 在此添加控件通知处理程序代码
	CButton	*pDrawBtn = (CButton *)GetDlgItem(IDC_BTN_DRAW);
	CString DrawStr;
	pDrawBtn->GetWindowText(DrawStr);

	if(DrawStr=="Draw"){
		CEdit *pEdir = (CEdit *)GetDlgItem(IDC_EDIT_CYCLE);
		CString EditStr;
		pEdir->GetWindowText(EditStr);
		int DrawCycle = atoi(EditStr.GetBuffer());
		if(DrawCycle==0)
			return;
		TimeStart(DrawCycle);
		pDrawBtn->SetWindowText(_T("Stop"));
	}else{
		TimeStop();
	}
}

void CRobotDlg::SetCurID(UINT ID)
{
	CEdit *pEdir = (CEdit *)GetDlgItem(IDC_EDIT_ID);
	CString EditStr;
	EditStr.Format("%d", ID);
	pEdir->SetWindowText(EditStr);
	ID = 0;
}

void CRobotDlg::OnEnChangeEditId()
{
	// TODO:  在此添加控件通知处理程序代码
	CEdit *pEdir = (CEdit *)GetDlgItem(IDC_EDIT_ID);
	CString EditStr;
	pEdir->GetWindowText(EditStr);
	SelID = atoi(EditStr.GetBuffer());
	if(SelID>254){
		SelID = 254;
		pEdir->SetWindowText(_T("254"));
	}
}

void CRobotDlg::OnBnClickedRadioWrite()
{
	// TODO: 在此添加控件通知处理程序代码
	CButton * BtnAct = (CButton *)GetDlgItem(IDC_BTN_ACTION);
	BtnAct->EnableWindow(FALSE);
	CEdit *pEditIDN = (CEdit *)GetDlgItem(IDC_EDIT_IDN);
	pEditIDN->EnableWindow(FALSE);
}

void CRobotDlg::OnBnClickedRadioSyswrite()
{
	// TODO: 在此添加控件通知处理程序代码
	CButton * BtnAct = (CButton *)GetDlgItem(IDC_BTN_ACTION);
	BtnAct->EnableWindow(FALSE);
	CEdit *pEditIDN = (CEdit *)GetDlgItem(IDC_EDIT_IDN);
	pEditIDN->EnableWindow(TRUE);
}

void CRobotDlg::OnBnClickedRadioRegwite()
{
	// TODO: 在此添加控件通知处理程序代码
	CButton * BtnAct = (CButton *)GetDlgItem(IDC_BTN_ACTION);
	BtnAct->EnableWindow(TRUE);
	CEdit *pEditIDN = (CEdit *)GetDlgItem(IDC_EDIT_IDN);
	pEditIDN->EnableWindow(TRUE);
}

void CRobotDlg::OnBnClickedBtnAction()
{
	// TODO: 在此添加控件通知处理程序代码
	pServo->RegWriteAction();
}

void CRobotDlg::OnEnChangeEditIdn()
{
	// TODO:  在此添加控件通知处理程序代码
	CEdit *pEdir = (CEdit *)GetDlgItem(IDC_EDIT_IDN);
	CString EditStr;
	pEdir->GetWindowText(EditStr);
	int ID = atoi(EditStr.GetBuffer());
	if(ID>254){
		ID = 254;
		pEdir->SetWindowText(_T("10"));
	}
	IDEND = ID;
}

void CRobotDlg::OnBnClickedCheckCurpos()
{
	// TODO: 在此添加控件通知处理程序代码
	CButton	* pCheckCtl = (CButton *)GetDlgItem(IDC_CHECK_CURPOS);
	if(pCheckCtl->GetCheck()){
		m_PIC.LineSel |= BLACKSEL;
	}else{
		m_PIC.LineSel &= ~BLACKSEL;
	}
	m_PIC.Invalidate(FALSE);
}

void CRobotDlg::OnBnClickedCheckTroque()
{
	// TODO: 在此添加控件通知处理程序代码
	CButton	* pCheckCtl = (CButton *)GetDlgItem(IDC_CHECK_TROQUE);
	if(pCheckCtl->GetCheck()){
		m_PIC.LineSel |= REDSEL;
	}else{
		m_PIC.LineSel &= ~REDSEL;
	}
	m_PIC.Invalidate(FALSE);
}

void CRobotDlg::OnBnClickedCheckSpeed()
{
	// TODO: 在此添加控件通知处理程序代码
	CButton	* pCheckCtl = (CButton *)GetDlgItem(IDC_CHECK_SPEED);
	if(pCheckCtl->GetCheck()){
		m_PIC.LineSel |= GREENSEL;
	}else{
		m_PIC.LineSel &= ~GREENSEL;
	}
	m_PIC.Invalidate(FALSE);
}
void CRobotDlg::OnBnClickedCheckVirpos()
{
	// TODO: 在此添加控件通知处理程序代码
	CButton	* pCheckCtl = (CButton *)GetDlgItem(IDC_CHECK_VIRPOS);
	if(pCheckCtl->GetCheck()){
		m_PIC.LineSel |= BLUESEL;
	}else{
		m_PIC.LineSel &= ~BLUESEL;
	}
	m_PIC.Invalidate(FALSE);
}

void CRobotDlg::OnBnClickedCheckCurrent()
{
	// TODO: 在此添加控件通知处理程序代码
	CButton	* pCheckCtl = (CButton *)GetDlgItem(IDC_CHECK_CURRENT);
	if(pCheckCtl->GetCheck()){
		m_PIC.LineSel |= YELLOWSEL;
	}else{
		m_PIC.LineSel &= ~YELLOWSEL;
	}
	m_PIC.Invalidate(FALSE);
}

void CRobotDlg::OnBnClickedBtnPosSet()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData();
	CString str;
	m_EditPos.GetWindowText(str);
	int Pos = atoi(str.GetBuffer());
	m_EditT.GetWindowText(str);
	int T = atoi(str.GetBuffer());
	m_EditV.GetWindowText(str);
	int V = atoi(str.GetBuffer());
	if(Pos>AdcMax){
		Pos = AdcMax;
		str.Format("%d", Pos);
		m_EditPos.SetWindowText(str);
	}
	m_SlidPos.SetPos(Pos);

	if(V<0){
		V = -V;
		V |= (1<<15);
	}
	if(T<0){
		T = -T;
		T |= (1<<10);
	}
	PostitonSet(Pos, V, T, m_WriteFun);
}

void CRobotDlg::PostitonSet(UINT16 Pos, UINT16 V, UINT16 T, int Fun)
{
	// TODO: 在此添加控件通知处理程序代码
	int res;
	if(Fun==0){
		Cs.Lock();
		res = pServo->WritePos(SelID, Pos, T, V);
		Cs.Unlock();
		if(res==0)
			TRACE("Write %d:pos %d Error\n", SelID, Pos);
	}
	if(Fun==1){
		UCHAR IDST = SelID;
		UCHAR IDLen = IDEND-IDST+1;
		UCHAR *IDBuf = new UCHAR[IDLen];
		int i;
		for(i=0; i<IDLen; i++){
			IDBuf[i] = IDST+i;
		}
		Cs.Lock();
		pServo->SyncWritePos(IDBuf, IDLen, Pos, T, V);
		Cs.Unlock();
		delete[] IDBuf;
	}
	if(Fun==2){
		UCHAR IDST = SelID;
		UCHAR IDLen = IDEND-IDST+1;
		int i;
		Cs.Lock();
		for(i=0; i<IDLen; i++){
			res = pServo->RegWritePos(IDST+i, Pos, T, V);
			if(res==0)
				TRACE("RegWrite %d:pos %d Error\n", IDST+i, Pos);
		}
		Cs.Unlock();
	}
}

void CRobotDlg::ToruqeEn(UCHAR En, int Fun)
{
	if(Fun==0){	
		Cs.Lock();
		pServo->EnableTorque(SelID, En);
		Cs.Unlock();
	}
	if(Fun==1){
		UCHAR IDST = SelID;
		int IDLen = IDEND-IDST+1;
		UCHAR *IDBuf = new UCHAR[IDLen];
		int i;
		for(i=0; i<IDLen; i++){
			IDBuf[i] = IDST+i;
		}
		Cs.Lock();
		pServo->snycWrite(IDBuf, IDLen, P_TORQUE_ENABLE, &En, 1);
		Cs.Unlock();
		delete[] IDBuf;
	}
	if(Fun==2){
		UCHAR IDST = SelID;
		int IDLen = IDEND-IDST+1;
		int i;
		Cs.Lock();
		for(i=0; i<IDLen; i++){
			pServo->regWrite(IDST+i, P_TORQUE_ENABLE, &En, 1);
		}
		Cs.Unlock();
	}
}

void CRobotDlg::SetMode()
{
	Cs.Lock();
	int End = pServo->readByte(SelID, P_END);
	int Level = pServo->readByte(SelID, P_RETURN_LEVEL);
	Cs.Unlock();
	if(End!=-1 && Level!=-1){
		pServo->End = End;
		pServo->Level = Level;
		if(End){
			PosShit = 0;
			m_PIC.TextVP = 1;
			AdcMax = 1023;
		}else{
			PosShit = 2;
			m_PIC.TextVP = 4;
			AdcMax = 4095;
		}
		m_SlidPos.SetRange(0, AdcMax);
		m_PIC.Invalidate(FALSE);
	}else{
		TRACE("read ID:%d Servo info error!\n", SelID);
	}
}
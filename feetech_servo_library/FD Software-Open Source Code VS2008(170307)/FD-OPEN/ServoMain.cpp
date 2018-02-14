// ServoMain.cpp : 实现文件
//

#include "stdafx.h"
#include "resource.h"
#include "ServoMain.h"
#include "ServoPG.h"
#include "ServoTest.h"
#include "RobotDlg.h"
#include "SCServo/SCServo.h"
// CServoMain 对话框


CServoMain::CServoMain()
{
	m_hIcon = AfxGetApp()->LoadIcon(IDI_FT32);
	IsOpen = 0;
	IsSearch = 0;
}

CServoMain::~CServoMain()
{
	delete	pServoPGDlg;
	delete	pServoTestDlg;
	delete	pRobotDlg;
}

void CServoMain::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX,IDC_SERVO_TAB,m_tab);
}


BEGIN_MESSAGE_MAP(CServoMain, CDialog)
	ON_NOTIFY(TCN_SELCHANGE, IDC_SERVO_TAB, &CServoMain::OnTcnSelchangeServoTab)
	ON_WM_CLOSE()
	ON_BN_CLICKED(IDC_BTN_SEARCH, &CServoMain::OnBnClickedBtnSearch)
	ON_BN_CLICKED(IDC_BTN_CLEAN, &CServoMain::OnBnClickedBtnClean)
	ON_BN_CLICKED(IDC_BTN_COMOPEN, &CServoMain::OnBnClickedBtnComopen)
	ON_NOTIFY(NM_CLICK, IDC_LIST_ID, &CServoMain::OnNMClickListId)
END_MESSAGE_MAP()


// CServoMain 消息处理程序


BOOL CServoMain::OnInitDialog()
{
	CDialog::OnInitDialog();
	SetIcon(m_hIcon, FALSE);
	// TODO:  在此添加额外的初始化

	pServoPGDlg = new ServoPG(pServo);
	pServoTestDlg = new ServoTest(pServo);
	pRobotDlg = new CRobotDlg(pServo);

	m_tab.InsertItem(0, _T("ROBOT"));
	m_tab.InsertItem(1, _T("PROGRAM"));
	m_tab.InsertItem(2, _T("UPGRADE"));

	pServoPGDlg->Create(IDD_SERVOPG_DIALOG, &m_tab);
	pServoTestDlg->Create(IDD_TEST_DIALOG, &m_tab);
	pRobotDlg->Create(IDD_ROBOT_DIALOG, &m_tab);

	CRect rc;
	pServoTestDlg->GetClientRect(&rc);
	rc.top += 30;
	rc.bottom += 30;
	pServoTestDlg->MoveWindow(&rc);

	pServoPGDlg->GetClientRect(&rc);
	rc.top += 30;
	rc.bottom += 30;
	pServoPGDlg->MoveWindow(&rc);

	pRobotDlg->GetClientRect(&rc);
	rc.top += 30;
	rc.bottom += 30;
	pRobotDlg->MoveWindow(&rc);

	pDialog[0] = pRobotDlg;
	pDialog[1] = pServoPGDlg;
	pDialog[2] = pServoTestDlg;

	m_CurSelTab = 0;
	m_tab.SetCurSel(m_CurSelTab);
	pDialog[m_CurSelTab]->ShowWindow(SW_SHOW);

	CComboBox *pComBox = (CComboBox *)GetDlgItem(IDC_COMBO_BAUD);
	pComBox->InsertString(0, "1000000");
	pComBox->InsertString(1, "500000");
	pComBox->InsertString(2, "250000");
	pComBox->InsertString(3, "128000");
	pComBox->InsertString(4, "115200");
	pComBox->InsertString(5, "76800");
	pComBox->InsertString(6, "57600");
	pComBox->InsertString(7, "38400");
	pComBox->SetCurSel(0);

	CListCtrl * pList = (CListCtrl *)GetDlgItem(IDC_LIST_ID);
	LONG lStyle;
	lStyle = GetWindowLong(pList->m_hWnd, GWL_STYLE);// 获取当前窗口style 
	lStyle &= ~LVS_TYPEMASK; // 清除显示方式位 
	lStyle |= LVS_REPORT; // 设置style
	SetWindowLong(pList->m_hWnd, GWL_STYLE, lStyle);// 设置style 
	DWORD dwStyle = pList->GetExtendedStyle(); 
	dwStyle |= LVS_EX_FULLROWSELECT;// 选中某行使整行高亮（只适用与report 风格的listctrl ） 
	dwStyle |= LVS_EX_GRIDLINES;// 网格线（只适用与report 风格的listctrl ） 
	//dwStyle |= LVS_EX_CHECKBOXES;//item 前生成checkbox 控件 
	pList->SetExtendedStyle(dwStyle); // 设置扩展风格

	pList->InsertColumn(0, "ID", LVCFMT_LEFT, 60);// 插入列
	pList->InsertColumn(1, "Band", LVCFMT_LEFT, 100);

	pServo->pSerial->GetComList();
	pComBox = (CComboBox *)GetDlgItem(IDC_COMBO_COM);
	int i;
	for(i=0; i<pServo->pSerial->ComListNum; i++){
		pComBox->InsertString(0, pServo->pSerial->ComSerialList[i]);
	}
	pComBox->SetCurSel(0);
	return TRUE;
}

void CServoMain::OnTcnSelchangeServoTab(NMHDR *pNMHDR, LRESULT *pResult)
{
	// TODO: 在此添加控件通知处理程序代码
	CRect rc;
	int PerSelTab = m_CurSelTab;
	m_CurSelTab = m_tab.GetCurSel();
	if(PerSelTab==0)
	{
		pRobotDlg->TimeStop();
	}
	if(PerSelTab!=m_CurSelTab)
	{
		pDialog[PerSelTab]->ShowWindow(SW_HIDE);
		pDialog[m_CurSelTab]->ShowWindow(SW_SHOW);
	}
	*pResult = 0;
}

void CServoMain::OnOK()
{
	//MessageBox("OK");
}
void CServoMain::OnCancel()
{
	//MessageBox("Cancel");
}
void CServoMain::OnClose()
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	::PostQuitMessage(0);
	CDialog::OnClose();
}

void CServoMain::WriteComEdit(CString str)
{
	CEdit *pCOMEdit = (CEdit*)GetDlgItem(IDC_EDIT_PINGINFO);
	CString AllStr;
	int nlen=pCOMEdit->GetLineCount();
	int strlen;
	pCOMEdit->GetWindowTextA(AllStr);
	strlen = AllStr.GetLength();
	pCOMEdit->SetSel(strlen,strlen);
	if(strlen)
		str = "\r\n"+str;
	else
		str = str;
	pCOMEdit->ReplaceSel(str);
}

void CServoMain::InsetList(int ID, int Band)
{
	CString IDStr, ModeStr;
	IDStr.Format("%d", ID);
	ModeStr.Format("%d", Band);
	CListCtrl * pList = (CListCtrl *)GetDlgItem(IDC_LIST_ID);
	LVFINDINFO finfo;
	finfo.flags = LVFI_STRING;
	finfo.psz = IDStr.GetBuffer();
	int nStart = -1; //从头查找
	int index = pList->FindItem(&finfo,nStart);
	if(index==-1)
	{
		pList->InsertItem(pList->GetItemCount(), IDStr);
		pList->SetItemText(pList->GetItemCount()-1, 1, ModeStr);
	}
	else
	{
		pList->DeleteItem(index);
		pList->InsertItem(index+1, IDStr);
		pList->SetItemText(index, 1, ModeStr);
	}
}

UINT SearchThreadFunc(LPVOID lpParam)
{
	CServoMain *pDlg = (CServoMain *)lpParam;
	SCServo *pServo = pDlg->pServo;
	CButton* pSeachBtn = (CButton*)pDlg->GetDlgItem(IDC_BTN_SEARCH);
	int i;
	for(i=0; i<254; i++)
	{
		CString ComStr;
		if(pDlg->IsSearch==0)
			goto end;
		if(pDlg->IsOpen==0)
			goto end;
		ComStr.Format("Pinging ID:%d SERVO...", i);
		pDlg->WriteComEdit(ComStr);
		int ID = pServo->Ping(i);
		if(ID != -1)
		{
			pDlg->InsetList(ID ,pDlg->Baud);
		}
	}
end:
	pSeachBtn->SetWindowText(_T("Search"));
	return 0;
}

void CServoMain::OnBnClickedBtnSearch()
{
	// TODO: 在此添加控件通知处理程序代码
	if(IsOpen==0)
		return;
	CButton* pSeachBtn = (CButton*)GetDlgItem(IDC_BTN_SEARCH);
	CListCtrl * pIDList = (CListCtrl *)GetDlgItem(IDC_LIST_ID);
	CString SeachStr;
	pSeachBtn->GetWindowText(SeachStr);
	if(SeachStr==_T("Search")){
		pSeachBtn->SetWindowText(_T("Stop"));
		pIDList->DeleteAllItems();
		IsSearch = 1;
		AfxBeginThread(SearchThreadFunc,LPVOID(this));
	}else{
		IsSearch = 0;
	}
}

void CServoMain::OnBnClickedBtnClean()
{
	// TODO: 在此添加控件通知处理程序代码
	CEdit *pCOMEdit = (CEdit*)GetDlgItem(IDC_EDIT_PINGINFO);
	CListCtrl * pList = (CListCtrl *)GetDlgItem(IDC_LIST_ID);
	pCOMEdit->SetWindowText(_T(""));
	pList->DeleteAllItems();
}

void CServoMain::OnBnClickedBtnComopen()
{
	// TODO: 在此添加控件通知处理程序代码
	CString BaudStr;
	CString ComStr;
	CButton *pOpenBtn = (CButton *)GetDlgItem(IDC_BTN_COMOPEN);
	CComboBox *pBaudComBox = (CComboBox *)GetDlgItem(IDC_COMBO_BAUD);
	CComboBox *pStrComBox = (CComboBox *)GetDlgItem(IDC_COMBO_COM);
	pBaudComBox->GetWindowText(BaudStr);
	Baud = atoi(BaudStr.GetBuffer());
	pStrComBox->GetWindowText(ComStr);
	if(IsOpen==0)
	{
		IsOpen = pServo->pSerial->OpenDev(ComStr);
		if(IsOpen)
		{
			pServo->pSerial->SetupDev(pServo->pSerial->BaudRate);
			pServo->pSerial->SetupArduino(Baud);
			pServo->pSerial->SetupDev(Baud);
			pOpenBtn->SetWindowText(_T("Close"));
			pBaudComBox->EnableWindow(FALSE);
			pStrComBox->EnableWindow(FALSE);
		}
	}
	else
	{
		IsOpen = 0;
		pServo->pSerial->CloseDev();
		pOpenBtn->SetWindowText(_T("Open"));
		pBaudComBox->EnableWindow(TRUE);
		pStrComBox->EnableWindow(TRUE);
	}
}

void CServoMain::OnNMClickListId(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMITEMACTIVATE pNMItemActivate = reinterpret_cast<NMITEMACTIVATE*>(pNMHDR);
	// TODO: 在此添加控件通知处理程序代码
	*pResult = 0;
	CListCtrl * pList = (CListCtrl *)GetDlgItem(IDC_LIST_ID);
	if(pNMItemActivate->iItem==-1){
		return;
	}
	CString ItemStr;
	ItemStr = pList->GetItemText(pNMItemActivate->iItem, 0);
	int ID = atoi(ItemStr.GetBuffer());
	pRobotDlg->SetCurID(ID);
	pServoPGDlg->SetCurID(ID);
	pServoTestDlg->SetCurID(ID);
	pRobotDlg->SetMode();
}

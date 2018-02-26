// ServoPG.cpp : implementation file
//

#include "stdafx.h"
#include "resource.h"
#include "ServoPG.h"
#include "SCServo/SCServo.h"
#include "SCServo/DataCal.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// ServoPG dialog

CString	MemName[][4]={
	{"P_MODEL_NUMBER","2","","-1"},//0 1
	{"P_END","1", "1","-1"},//2
	{"P_VERSION","2","","-1"},//3 4
	{"P_ID","1","0","-1"},//5
	{"P_BAUD_RATE","1","0","-1"},//6
	{"P_RETURN_DELAY_TIME","1","0","-1"},//7
	{"P_RETURN_LEVEL","1","1","-1"},//8
	{"P_MIN_ANGLE_LIMIT","2","0","-1"},//9 10
	{"P_MAX_ANGLE_LIMIT","2","1023","-1"},//11 12
	{"P_LIMIT_TEMPERATURE","1","80","-1"},//13
	{"P_MAX_LIMIT_VOLTAGE","1","90","-1"},//14
	{"P_MIN_LIMIT_VOLTAGE","1","35","-1"},//15
	{"P_MAX_TORQUE","2","1023","-1"},//16 17
	{"P_H|L","1","0","-1"},//18
	{"P_ALARM_SHUTDOWN","1","37","-1"},//19
	{"P_ALARM_LED","1","37","-1"},//20
	{"P_COMPLIANCE_P","1","15","-1"},//21
	{"P_COMPLIANCE_D","1","0","-1"},//22
	{"P_COMPLIANCE_I","1","0","-1"},//23
	{"P_PUNCH","2","50","-1"},//24 25
	{"P_CW_DEAD","1","2","-1"},//26
	{"P_CCW_DEAD","1","2","-1"},//27
	{"P_IMAX","2","0","-1"},//28 29
	{"P_ADT","1","5","-1"},//30
	{"P_ACC","1","20","-1"},//31
	{"P_SETP","1","5","-1"},//32
	{"P_OFS","2","0","11"},//33 34
	{"P_MODE","1","0","-1"},//35
	{"P_MAX_CURRENT","2","2047","-1"},//36 37
	{"NULL", "NULL","NULL","NULL"}
};

ServoPG::ServoPG(SCServo *pServo)
{
	this->pServo = pServo;
	ID = 0;
	memLen = 0;
	int i = 0;
	while(MemName[i][0] != "NULL"){
		if(MemName[i][1]=="1"){
			memLen++;
		}else{
			memLen += 2;
		}
		i++;
	}
	nMemData = new UCHAR[memLen];
}

ServoPG::~ServoPG()
{
}
void ServoPG::ResetDefault()
{
	CListCtrl * pList = (CListCtrl *)GetDlgItem(IDC_LIST_MEM);
	int i = 0;
	while(MemName[i][2]!="NULL"){
		pList->SetItemText(i, 1, MemName[i][2]);
		i++;
	}
}

void ServoPG::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_EDIT_COMMAD, m_CommadCtl);
}


BEGIN_MESSAGE_MAP(ServoPG, CDialog)
	ON_WM_HSCROLL()
	ON_BN_CLICKED(IDC_BTN_RESET, &ServoPG::OnBnClickedBtnReset)
	ON_BN_CLICKED(IDC_BTN_READ, &ServoPG::OnBnClickedBtnRead)
	ON_BN_CLICKED(IDC_BTN_WRITE, &ServoPG::OnBnClickedBtnWrite)
	ON_BN_CLICKED(IDC_BTN_SAVE, &ServoPG::OnBnClickedBtnSave)
	ON_BN_CLICKED(IDC_BTN_OPEN, &ServoPG::OnBnClickedBtnOpen)
	ON_NOTIFY(NM_CLICK, IDC_LIST_MEM, &ServoPG::OnNMClickListMem)
	ON_BN_CLICKED(IDC_BTN_SET, &ServoPG::OnBnClickedBtnSet)
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// ServoPG message handlers

BOOL ServoPG::OnInitDialog()
{
	CDialog::OnInitDialog();
	// TODO: Add extra initialization here

	CListCtrl * pList = (CListCtrl *)GetDlgItem(IDC_LIST_MEM);
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

	pList->InsertColumn(0, "MEM", LVCFMT_LEFT, 160);// 插入列
	pList->InsertColumn(1, "Value", LVCFMT_LEFT, 150);

	int i = 0;
	while(MemName[i][0]!="NULL"){
		pList->InsertItem(i, MemName[i][0]);
		i++;
	}

	m_CommadCtl.ReplaceSel("Instructions:");
	ResetDefault();
	return TRUE;
}

void ServoPG::WriteComEdit(CString str)
{
	CString AllStr;
	int nlen=m_CommadCtl.GetLineCount();
	int strlen;
	m_CommadCtl.GetWindowTextA(AllStr);
	strlen = AllStr.GetLength();
	m_CommadCtl.SetSel(strlen,strlen);
	str="\r\n"+str;
	m_CommadCtl.ReplaceSel(str);
}


void ServoPG::OnBnClickedBtnRead()
{
	// TODO: 在此添加控件通知处理程序代码

	CString str;
	str.Format(_T("----------ID:%d----------"), ID);
	WriteComEdit(str);


	if(pServo->Read(ID, 0, nMemData, memLen)!=memLen){
		WriteComEdit(_T("Read Servo Fail!"));
		return;
	}
	writeMem(0, memLen, nMemData, nMemData[2]);
	pServo->End = nMemData[2];
	pServo->Level = nMemData[8];
	WriteComEdit(_T("Read All Servo Data"));
}


void ServoPG::OnBnClickedBtnWrite()
{
	// TODO: 在此添加控件通知处理程序代码
	BOOL res;;
	CButton *pUnlock = (CButton*)GetDlgItem(IDC_CHECK_UNLOCK);
	CString str;
	str.Format(_T("----------ID:%d----------"), ID);
	WriteComEdit(str);

	
	if(pServo->Read(ID, 0, nMemData, memLen)!=memLen){
		WriteComEdit(_T("Read Servo Fail!"));
		return;
	}
	pServo->End = nMemData[2];
	pServo->Level = nMemData[8];

	pServo->pSerial->SetupTimeOut(500);
	if(pUnlock->GetCheck()){
		res = pServo->writeByte(ID, P_LOCK, 0);
		if(!res){
			WriteComEdit(_T("Unlock Eprom Fail!"));
			goto error;
		}
		WriteComEdit(_T("Unlock Eprom"));
	}
	readMem(0, memLen, nMemData, pServo->End);
	pServo->Level = nMemData[8];
	res = pServo->genWrite(ID, 5, nMemData+5, memLen-5);
	if(!res){
		WriteComEdit(_T("Write Servo Fail!"));
		goto error;
	}
	WriteComEdit(_T("Write All Servo Data"));
	ID = nMemData[5];
	if(pUnlock->GetCheck()){
		if(!pServo->Level){
			Sleep(500);
		}
		res = pServo->writeByte(ID, P_LOCK, 1);
		if(!res){
			WriteComEdit(_T("lock Eprom Fail!"));
			goto error;
		}
		WriteComEdit(_T("lock Eprom"));
	}
error:
	pServo->pSerial->SetupTimeOut(50);
}


void ServoPG::OnBnClickedBtnOpen()
{
	// TODO: 在此添加控件通知处理程序代码
	CString path;
	CFile pServoFile;

	CFileDialog OpenDlg(TRUE,_T("dat"),0,OFN_HIDEREADONLY|OFN_OVERWRITEPROMPT
		,"Servo Files (*.dat)|*.dat||",this);
	if(OpenDlg.DoModal()==IDOK){
		path = OpenDlg.GetPathName();
		if(!pServoFile.Open(path,CFile::modeReadWrite)){
			MessageBox(_T("Cannot open file"));
			return;
		}
		pServoFile.Read(nMemData, memLen);
		writeMem(0, memLen, nMemData, 0);
		pServoFile.Close();
		return;
	}
	return;
}

void ServoPG::OnBnClickedBtnSave()
{
	// TODO: 在此添加控件通知处理程序代码
	CString path;
	CFile pServoFile;

	CFileDialog SaveDlg(FALSE,_T("dat"),0,OFN_HIDEREADONLY|OFN_OVERWRITEPROMPT
		,"Servo Files (*.dat)|*.dat||",this);
	if(SaveDlg.DoModal()==IDOK){
		path = SaveDlg.GetPathName();
		readMem(0, memLen, nMemData, 0);
		pServoFile.Open(path, CFile::modeReadWrite|CFile::modeCreate);
		pServoFile.Write(nMemData, memLen);
		pServoFile.Close();
	}
}

void ServoPG::OnBnClickedBtnReset()
{
	// TODO: 在此添加控件通知处理程序代码
	ResetDefault();
}



void ServoPG::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	CDialog::OnHScroll(nSBCode, nPos, pScrollBar);
}


void ServoPG::OnNMClickListMem(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMITEMACTIVATE pNMItemActivate = reinterpret_cast<NMITEMACTIVATE*>(pNMHDR);
	// TODO: 在此添加控件通知处理程序代码
	*pResult = 0;
	CListCtrl * pList = (CListCtrl *)GetDlgItem(IDC_LIST_MEM);
	CEdit *pSet = (CEdit *)GetDlgItem(IDC_EDIT_SET);
	if(pNMItemActivate->iItem==-1){
		return;
	}
	CString ItemStr;
	ItemStr = pList->GetItemText(pNMItemActivate->iItem, 1);
	pSet->SetWindowText(ItemStr);
}

void ServoPG::OnOK()
{
	//MessageBox("OK");
}

void ServoPG::OnCancel()
{
	//MessageBox("Cancel");
}

void ServoPG::OnBnClickedBtnSet()
{
	// TODO: 在此添加控件通知处理程序代码
	CListCtrl * pList = (CListCtrl *)GetDlgItem(IDC_LIST_MEM);
	CEdit *pSet = (CEdit *)GetDlgItem(IDC_EDIT_SET);
	CString ItemStr;
	pSet->GetWindowText(ItemStr);
	int ItemIndex = pList->GetSelectionMark();
	if(ItemIndex!=-1){
		pList->SetItemText(ItemIndex, 1, ItemStr);
		CString ComStr = pList->GetItemText(ItemIndex, 0);
		ComStr += ":" + ItemStr;
		WriteComEdit(ComStr);
	}
}

void ServoPG::writeMem(int St, int nLen, UCHAR *nData, int End)
{
	CListCtrl * pList = (CListCtrl *)GetDlgItem(IDC_LIST_MEM);
	int nDati = 0, Memi = St;
	int Size;
	CString MemStr;
	int Data;
	while(MemName[Memi][0]!="NULL"){
		Size = atoi(MemName[Memi][1].GetBuffer());
		if(Size==1){
			Data = nData[nDati];
			CString ngvStr = MemName[Memi][3];
			if(ngvStr!="-1"){
				INT16 ngvByte = atoi(ngvStr.GetBuffer());
				Data = DataCal::scs2host(Data, ngvByte);
			}
			MemStr.Format("%d", Data);
			pList->SetItemText(Memi, 1, MemStr);
			nDati++;
		}else{
			if((nDati+1)<nLen){
				if(!End){
					Data = ((UINT)nData[nDati+1]<<8) | nData[nDati];
				}else{
					Data = ((UINT)nData[nDati]<<8) | nData[nDati+1];
				}
				CString ngvStr = MemName[Memi][3];
				if(ngvStr!="-1"){
					INT16 ngvByte = atoi(ngvStr.GetBuffer());
					Data = DataCal::scs2host(Data, ngvByte);
				}
				MemStr.Format("%d", Data);
				pList->SetItemText(Memi, 1, MemStr);
			}
			nDati += 2;
		}
		if(nDati>=nLen){
			break;
		}
		Memi++;
	}
}

void ServoPG::readMem(int St, int nLen, UCHAR *nData, int End)
{
	CListCtrl * pList = (CListCtrl *)GetDlgItem(IDC_LIST_MEM);
	int nDati = 0, Memi = St;
	int Size;
	CString MemStr;
	int Data;
	while(MemName[Memi][0]!="NULL"){
		Size = atoi(MemName[Memi][1].GetBuffer());
		if(Size==1){
			MemStr = pList->GetItemText(Memi, 1);
			Data = atoi(MemStr.GetBuffer());
			CString ngvStr = MemName[Memi][3];
			if(ngvStr!="-1" && Data<0){
				INT16 ngvByte = atoi(ngvStr.GetBuffer());
				Data = DataCal::host2scs(Data, ngvByte);
			}
			nData[nDati] = Data;
			nDati++;
		}else{
			if((nDati+1)<nLen){
				MemStr = pList->GetItemText(Memi, 1);
				Data = atoi(MemStr.GetBuffer());
				CString ngvStr = MemName[Memi][3];
				if(ngvStr!="-1" && Data<0){
					INT16 ngvByte = atoi(ngvStr.GetBuffer());
					Data = DataCal::host2scs(Data, ngvByte);
				}
				if(!End){
					nData[nDati] = Data&0xff;
					nData[nDati+1] = (Data>>8);
				}else{
					nData[nDati] = (Data>>8);
					nData[nDati+1] = Data&0xff;
				}				
			}
			nDati += 2;
		}
		if(nDati>=nLen){
			break;
		}
		Memi++;
	}
}

void ServoPG::SetCurID(int ID)
{
	CListCtrl * pList = (CListCtrl *)GetDlgItem(IDC_LIST_MEM);
	this->ID = ID;
	CString MemStr;
	MemStr.Format("%d", ID);
	pList->SetItemText(3, 1, MemStr);
}
#pragma once
#include "TextProgressCtrl.h"

class	SCServo;

class ServoTest : public CDialog
{
// Construction
public:
	ServoTest(SCServo *pServo);

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
	virtual BOOL OnInitDialog();
	virtual void OnOK();
	virtual void OnCancel();
	afx_msg void OnBnClickedBtnUpgopen();
	afx_msg void OnBnClickedBtnUpgwrite();
	afx_msg void OnBnClickedBtnRead();
	afx_msg void OnBnClickedBtnTrantest();
	afx_msg void OnBnClickedBtnTranclr();
public:
	SCServo	*pServo;
	CButton		m_BtnWrite;
	CTextProgressCtrl m_UpgradeCtl;
	int			IsTranTest;
	CEdit       m_TranInfo;
	CEdit		m_TranID;
	CEdit       m_TranIDN;
	CEdit		m_TranTimeOut;
	CEdit		m_VerID;
public:
	void	SetCurID(UINT ID);
	void	WriteTranInfo(int ID, int TimeOut, int DatLen);
	void	WriteTranInfo(int ID, int TimeOut);
	void	WriteTranInfo();
	void	GetTranCount(int DatLen);
private:
	CEdit		m_FirmCtl;
	CStatic		m_TranCount;
	CButton		m_TranTest;
	CString		TraInfoStr;
	int			TranErrNum[255];
	long		TranDatSize;
};
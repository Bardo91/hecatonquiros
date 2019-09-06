#pragma once
#include "PIC/PIC.h"


// UpgradeDlg 对话框
class	SCServo;
class	Hex2Bin;
class	KAES;
class CRobotDlg : public CDialog
{

public:
	CRobotDlg(SCServo *pServo);
	virtual ~CRobotDlg();

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持
	virtual BOOL OnInitDialog();
	virtual void OnOK();
	virtual void OnCancel();
	DECLARE_MESSAGE_MAP()
	afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	afx_msg void OnBnClickedCheckTorqueen();
	afx_msg void OnBnClickedBtnDraw();
	afx_msg void OnEnChangeEditId();
	afx_msg void OnBnClickedRadioWrite();
	afx_msg void OnBnClickedRadioSyswrite();
	afx_msg void OnBnClickedRadioRegwite();
	afx_msg void OnBnClickedBtnAction();
	afx_msg void OnEnChangeEditIdn();
	afx_msg void OnBnClickedCheckCurpos();
	afx_msg void OnBnClickedCheckTroque();
	afx_msg void OnBnClickedCheckSpeed();
	afx_msg void OnBnClickedCheckVirpos();
	afx_msg void OnBnClickedCheckCurrent();
	afx_msg void OnBnClickedBtnPosSet();
public:
	SCServo	*pServo;
	int		TimeID;
	int		IDEND;
	int		SelID;
	CPIC	m_PIC;
	CStatic m_LabGoalPos;
	CStatic m_LabTmp;
	CStatic m_LabVol;
	CStatic m_LabLoad;
	CStatic m_LabSpeed;
	CStatic m_LabCurPos;
	CStatic m_LabCurrent;
	CStatic m_LabMoving;
	CStatic m_LabError;
	CSliderCtrl m_SlidPos;
	CEdit	m_EditT;
	CEdit	m_EditV;
	CEdit	m_EditPos;
	int		m_WriteFun;
	CSliderCtrl m_SlidZoomX;
	CSliderCtrl m_SlidHoriz;
	int		PosShit;
	int		AdcMax;
	CCriticalSection  Cs;
public:
	void TimeStart(UINT time);
	void TimeStop();
	void SetCurID(UINT ID);
	void SetMode();
private:
	void PostitonSet(UINT16 Pos, UINT16 V, UINT16 T, int Fun);
	void ToruqeEn(UCHAR En, int Fun);
};

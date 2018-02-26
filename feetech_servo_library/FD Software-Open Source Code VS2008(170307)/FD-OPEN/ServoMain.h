#pragma once
// CServoMain 对话框

class SCServo;
class ServoPG;
class ServoTest;
class CRobotDlg;

class CServoMain : public CDialog
{
public:
	CServoMain();   // 标准构造函数
	virtual ~CServoMain();

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持
	DECLARE_MESSAGE_MAP()
	
	virtual BOOL OnInitDialog();
	virtual void OnOK();
	virtual void OnCancel();
	afx_msg void OnTcnSelchangeServoTab(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnClose();
	afx_msg void OnBnClickedBtnSearch();
	afx_msg void OnBnClickedBtnClean();
	afx_msg void OnBnClickedBtnComopen();
	afx_msg void OnNMClickListId(NMHDR *pNMHDR, LRESULT *pResult);
private:
	HICON		m_hIcon;
	CDialog		*pDialog[3];
	int			m_CurSelTab;
public:
	void WriteComEdit(CString str);
	void InsetList(int ID, int Band);
public:
	ServoPG		*pServoPGDlg;
	ServoTest	*pServoTestDlg;
	CRobotDlg	*pRobotDlg;
	SCServo		*pServo;
	CTabCtrl	m_tab;
	BOOL		IsOpen;
	BOOL		IsSearch;
	int			Baud;
};

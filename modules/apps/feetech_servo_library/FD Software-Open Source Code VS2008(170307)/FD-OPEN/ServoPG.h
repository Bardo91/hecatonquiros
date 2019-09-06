#pragma once

// ServoPG dialog
class	SCServo;
class ServoPG : public CDialog
{
public:
	ServoPG(SCServo *pServo);
	~ServoPG();
private:
	CEdit m_CommadCtl;
	UINT8	ID;
	UCHAR *nMemData;
	int	memLen;
public:
	SCServo	*pServo;
protected:
	DECLARE_MESSAGE_MAP()
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	virtual BOOL OnInitDialog();
	virtual void OnOK();
	virtual void OnCancel();
	afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	afx_msg void OnBnClickedBtnReset();
	afx_msg void OnBnClickedBtnRead();
	afx_msg void OnBnClickedBtnWrite();
	afx_msg void OnBnClickedBtnSave();
	afx_msg void OnBnClickedBtnOpen();
	afx_msg void OnBnClickedBtnSet();
	afx_msg void OnNMClickListMem(NMHDR *pNMHDR, LRESULT *pResult);
private:
	void ResetDefault();
	void WriteComEdit(CString str);
	void writeMem(int St, int nLen, UCHAR *nData, int End);
	void readMem(int St, int nLen, UCHAR *nData, int End);
public:
	void SetCurID(int ID);
};
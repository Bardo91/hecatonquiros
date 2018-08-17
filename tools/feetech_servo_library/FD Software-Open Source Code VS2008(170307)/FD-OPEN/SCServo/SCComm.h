/*
 * SCComm.h
 * 基于MFC的COM口程序接口
 * 日期: 2016.8.26
 * 作者: 谭雄乐
 */

#ifndef _SCCOMM_H
#define _SCCOMM_H

class CSCComm
{
public:
	CSCComm(void);
	~CSCComm(void);
	virtual int Write(UCHAR *nDat, UINT nLen);
	virtual int Read(UCHAR *nDat, UINT nLen);
	void SetupTimeOut(UINT TimeOut);
	BOOL OpenDev(CString ComStrName);
	BOOL CloseDev(void);
	UINT SetupDev(UINT BaudRate);
	void FlushRXComm();
	void SetupArduino(UINT BaudRate);

	CString ComSerialList[256];
	int	GetComList(void);
public:
	int ComListNum;
	int	IOTimeOut;
	UINT BaudRate;
protected:
	HANDLE hCom;
	CString Com2DevCom(CString ComStrName);
	CString Com2DevCom(int ComNum);
};

#endif
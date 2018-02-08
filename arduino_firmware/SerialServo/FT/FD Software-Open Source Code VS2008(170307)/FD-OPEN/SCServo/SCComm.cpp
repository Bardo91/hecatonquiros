/*
 * SCComm.cpp
 * ����MFC��COM�ڳ���ӿ�
 * ����: 2016.8.26
 * ����: ̷����
 */
#include "stdafx.h"
#include <setupapi.h>
#include <devguid.h> 
#include "SCComm.h"


CSCComm::CSCComm(void)
{
	hCom = (HANDLE)-1;
	IOTimeOut = 50;
	ComListNum = -1;
	BaudRate = 1000000;
}

CSCComm::~CSCComm(void)
{
}

int CSCComm::Read(UCHAR *nDat, UINT nLen)
{
	if(hCom==(HANDLE)-1)
		return 0;
	int rlen = 0;
	ReadFile(hCom, nDat, nLen, (LPDWORD)&rlen, NULL);
	return rlen;
}

int CSCComm::Write(UCHAR *nDat, UINT nLen)
{
	if(hCom==(HANDLE)-1)
		return 0;
	int wlen = 0;
	WriteFile(hCom, nDat, nLen, (LPDWORD)&wlen, NULL);
	return wlen;
}

void CSCComm::FlushRXComm()
{
	PurgeComm(hCom, PURGE_RXCLEAR);//��ջ�����
}

int CSCComm::GetComList(void)
{
	CHAR Name[25]; 
	UCHAR szPortName[25];
	LONG Status; 
	DWORD dwIndex = 0; 
	DWORD dwName; 
	DWORD dwSizeofPortName; 
	DWORD Type;
	HKEY hKey; 
	LPCTSTR data_Set="HARDWARE\\DEVICEMAP\\SERIALCOMM\\";
	dwName = sizeof(Name); 
	dwSizeofPortName = sizeof(szPortName);

	long ret0 = RegOpenKeyEx(HKEY_LOCAL_MACHINE, data_Set, 0, KEY_READ, &hKey);//��һ���ƶ���ע����,�ɹ�����ERROR_SUCCESS����0��ֵ
	if(ret0 == ERROR_SUCCESS) 
	{
		ComListNum = 0;
		do 
		{
			Status = RegEnumValue(hKey, dwIndex++, Name, &dwName, NULL, &Type, szPortName, &dwSizeofPortName);//��ȡ��ֵ 
			if((Status == ERROR_SUCCESS)||(Status == ERROR_MORE_DATA)) 
			{ 
				ComSerialList[ComListNum] = CString(szPortName);// �����ַ������� 
				TRACE("serial:%s\n",ComSerialList[ComListNum]);
				ComListNum++;// ���ڼ��� 
			} 
			//ÿ��ȡһ��dwName��dwSizeofPortName���ᱻ�޸�
			dwName = sizeof(Name); 
			dwSizeofPortName = sizeof(szPortName); 
		} while((Status == ERROR_SUCCESS)||(Status == ERROR_MORE_DATA)); 

		RegCloseKey(hKey); 
	}
	return ComListNum;
}

CString CSCComm::Com2DevCom(CString ComStrName)
{
	int ComNum = (USHORT)_tcstoul(ComStrName.GetBuffer() + 3, NULL, 10);
	if(ComNum>9){
		ComStrName.Format("\\\\.\\COM%d", ComNum);
	}
	return ComStrName;
}

CString CSCComm::Com2DevCom(int ComNum)
{
	CString ComStrName;
	if(ComNum>9){
		ComStrName.Format("\\\\.\\COM%d", ComNum);
	}else{
		ComStrName.Format("COM%d", ComNum);
	}
	return ComStrName;
}

BOOL CSCComm::OpenDev(CString ComStrName)
{
	ComStrName = Com2DevCom(ComStrName);
	hCom = CreateFile(ComStrName, GENERIC_READ|GENERIC_WRITE,
		0, NULL, OPEN_EXISTING, 0, NULL);
	if(hCom==(HANDLE)-1)
		return FALSE;
	return TRUE;
}

BOOL CSCComm::CloseDev(void)
{
	BOOL res = CloseHandle(hCom);
	hCom = (HANDLE)-1;
	return res;
}

void CSCComm::SetupTimeOut(UINT TimeOut)
{
	COMMTIMEOUTS TimeOuts;
	TimeOuts.ReadIntervalTimeout = 0;
	TimeOuts.ReadTotalTimeoutMultiplier = 0;
	TimeOuts.ReadTotalTimeoutConstant = TimeOut;
	TimeOuts.WriteTotalTimeoutMultiplier = 0;
	TimeOuts.WriteTotalTimeoutConstant = TimeOut;
	SetCommTimeouts(hCom, &TimeOuts);//���ó�ʱ
}

UINT CSCComm::SetupDev(UINT BaudRate)
{
	int OldBaudRate;
	if(hCom==(HANDLE)-1)
		return FALSE;
	SetupComm(hCom,1024,1024);//���ö�д������

	SetupTimeOut(IOTimeOut);
	
	DCB dcb;
	GetCommState(hCom, &dcb);

	OldBaudRate = dcb.BaudRate;
	dcb.BaudRate = BaudRate;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	dcb.fRtsControl = dcb.fDtrControl = 0;
	SetCommState(hCom, &dcb);//���ô���

	PurgeComm(hCom, PURGE_TXCLEAR|PURGE_RXCLEAR);//��ջ�����
	this->BaudRate = BaudRate;
	return OldBaudRate;
}

void CSCComm::SetupArduino(UINT BaudRate)
{
	CString baudStr;
	baudStr.Format("#BAUD %d\r\n", BaudRate);
	Write((UCHAR *)baudStr.GetBuffer(), baudStr.GetLength());
}

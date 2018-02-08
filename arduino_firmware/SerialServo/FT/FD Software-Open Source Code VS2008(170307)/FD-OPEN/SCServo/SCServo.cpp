/*
 * SCServo.cpp
 * 硬件通信接口
 * 日期: 2016.8.25
 * 作者: 谭雄乐
 */
#include "stdafx.h"
#include "SCServo.h"

SCServo::SCServo()
{
	pSerial = NULL;
}

int SCServo::readSCS(unsigned char *nDat, int nLen)
{
	if(nDat==NULL){
		return 0;
	}
	return pSerial->Read(nDat, nLen);
}

int SCServo::writeSCS(unsigned char *nDat, int nLen)
{
	if(nDat==NULL){
		return 0;
	}
	return pSerial->Write(nDat, nLen);
}

int SCServo::writeSCS(unsigned char bDat)
{
	return pSerial->Write(&bDat, 1);
}

void SCServo::flushSCS()
{
	pSerial->FlushRXComm();
}
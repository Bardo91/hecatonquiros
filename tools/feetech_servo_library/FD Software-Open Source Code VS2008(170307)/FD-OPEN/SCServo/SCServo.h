/*
 * SCServo.h
 * Ӳ��ͨ�Žӿ�
 * ����: 2016.8.25
 * ����: ̷����
 */

#ifndef _SCSERVO_H
#define _SCSERVO_H

#include "SCSProtocol.h"
#include "SCComm.h"

class SCServo : public SCSProtocol
{
public:
	SCServo(void);
	virtual int writeSCS(unsigned char *nDat, int nLen);//���nLen�ֽ�
	virtual int readSCS(unsigned char *nDat, int nLen);//����nLen�ֽ�
	virtual int writeSCS(unsigned char bDat);//���1�ֽ�
	virtual void flushSCS();//ˢ�½ӿڻ�����
public:
	CSCComm *pSerial;//����ָ��
};

#endif
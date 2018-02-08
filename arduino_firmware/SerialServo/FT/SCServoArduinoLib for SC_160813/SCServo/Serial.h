/*
 * Serial.h
 * 串口通信接口
 * 日期: 2016.8.9
 * 作者: 谭雄乐
 */

#ifndef _SERILA_H
#define _SERILA_H


#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class CSerial
{
public:
	CSerial(void);
	virtual int writeSerial(unsigned char *nDat, int nLen);
	virtual int readSerial(unsigned char *nDat, int nLen);
	virtual int writeSerial(unsigned char bDat);
	virtual void flushSerial();
public:
	unsigned long int IOTimeOut;
	HardwareSerial *pSerial;
};

#endif
/*
 * SCServo.cpp
 * Series Control Servo
 * Created on: 2014.4.15
 * Author: Tony tan
 */
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#define printf(args) (Serial.write(args))
#else
#include "WProgram.h"
#define printf(args) (Serial.print(args,BYTE))
#endif
#include "SCServo.h"

SCServo::SCServo ()
{
}

u8 SCServo::EnableTorque(u8 ID, u8 Enable, u8 ReturnLevel)
{
	int messageLength = 4;

	printf(startByte);
	printf(startByte);
	printf(ID);
	printf(messageLength);
	printf(INST_WRITE);
	printf(P_TORQUE_ENABLE);
	printf(Enable);
	printf((~(ID + messageLength + INST_WRITE + Enable + P_TORQUE_ENABLE))&0xFF);
	if(ID != 0xfe && ReturnLevel==2)
		return ReadBuf(6);
	return 6;
}

u8 SCServo:: WritePos(u8 ID, int position, int velocity, u8 ReturnLevel)
{
	int messageLength = 7;
	byte posL =  position>>8;
	byte posH =  position&0xff;	
	byte velL =  velocity>>8;
	byte velH =  velocity&0xff;

	printf(startByte);
	printf(startByte);
	printf(ID);
	printf(messageLength);
	printf(INST_WRITE);
	printf(P_GOAL_POSITION_L);
	printf(posL);
	printf(posH);
	printf(velL);
	printf(velH);
	printf((~(ID + messageLength + INST_WRITE + P_GOAL_POSITION_L + posL + posH + velL + velH))&0xFF);
	if(ID != 0xfe && ReturnLevel==2)
		return ReadBuf(6);
	return 6;
}

u8 SCServo:: RegWritePos(u8 ID, int position, int velocity, u8 ReturnLevel)
{
	int messageLength = 7;
	byte posL =  position>>8;
	byte posH =  position&0xff;		
	
	byte velL =  velocity>>8;
	byte velH =  velocity&0xff;

	printf(startByte);
	printf(startByte);
	printf(ID);
	printf(messageLength);
	printf(INST_REG_WRITE);
	printf(P_GOAL_POSITION_L);
	printf(posL);
	printf(posH);
	printf(velL);
	printf(velH);
	printf((~(ID + messageLength + INST_REG_WRITE + P_GOAL_POSITION_L + posL + posH + velL + velH))&0xFF);
	if(ID != 0xfe && ReturnLevel==2)
		return ReadBuf(6);
	return 6;
}

void SCServo:: RegWriteAction()
{
	int messageLength = 2;
	byte ID =  0xFE; 
	printf(startByte);
	printf(startByte);
	printf(ID);
	printf(messageLength);
	printf(INST_ACTION);
	printf((~(ID + messageLength + INST_ACTION))&0xFF);
}

u8 SCServo:: ReadBuf(u8 len, u8 *buf)
{
	u16 n = 0;
	u8 size = 0;
	u8 ComData;
	while(n<TIMEOUT)
	{
		if(Serial.available())
		{
			if(buf)
				buf[size] = Serial.read();
			else
				ComData = Serial.read();
			size++;
			if(size>=len)
				break;
			n = 0;
		}
		n++;
	}
	return size;
}

s16 SCServo:: ReadPos(u8 ID)
{	
	u8 buf[8];
	u8 size;
	u16 pos;
	memset(buf,0,sizeof(buf));
	printf(startByte);
	printf(startByte);
	printf(ID);
	printf(4);
	printf(INST_READ);
	printf(P_PRESENT_POSITION_L);
	printf(2);
	printf((~(ID + 4 + INST_READ + P_PRESENT_POSITION_L + 2))&0xFF);
	size = ReadBuf(8, buf);
	if(size<8)
		return -1;
	pos = buf[5];
	pos <<= 8;
	pos |= buf[6];
	return (s16)pos;
}

void SCServo:: SyncWritePos(u8 ID[], u8 IDN, int position, int velocity)
{
	int messageLength = 5*IDN+4;
	u8 Sum = 0;
	byte posL =  position>>8;
	byte posH =  position&0xff;		
	
	byte velL =  velocity>>8;
	byte velH =  velocity&0xff;

	printf(startByte);
	printf(startByte);
	printf(0xfe);
	printf(messageLength);
	printf(INST_SYNC_WRITE);
	printf(P_GOAL_POSITION_L);
	printf(4);
	
	Sum = 0xfe + messageLength + INST_SYNC_WRITE + P_GOAL_POSITION_L + 4;
	int i;
	for(i=0; i<IDN; i++)
	{
		printf(ID[i]);
		printf(posL);
		printf(posH);
		printf(velL);
		printf(velH);
		Sum += ID[i] + posL + posH + velL + velH;
	}
	printf((~Sum)&0xFF);
}
/*
 * SCServo.h
 * Series Control Servo
 * Created on: 2014.4.15
 * Author: Tony tan
 */
#ifndef _SCSERVO_h_
#define _SCSERVO_h_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define		s8		char
#define		u8		unsigned char
#define		u16		unsigned short
#define		s16		short
#define		u32		unsigned long
#define		s32		long

class SCServo{
	public:
		SCServo();
		u8 EnableTorque(u8 ID, u8 Enable, u8 ReturnLevel=2);
		u8 WritePos(u8 ID, int position, int velocity, u8 ReturnLevel=2);
		u8 RegWritePos(u8 ID, int position, int velocity, u8 ReturnLevel=2);
		s16 ReadPos(u8 ID);
		void RegWriteAction();
		void SyncWritePos(u8 ID[], u8 IDN, int position, int velocity);
	private:
		u8 ReadBuf(u8 len, u8 *buf=NULL);
		#define		startByte	0xFF
		#define		TIMEOUT		600//TIMEOUT 600
	//register Address
		#define P_MODEL_NUMBER_L 0
		#define P_MODEL_NUMBER_H 1
		#define P_VERSION_L 3		
		#define P_VERSION_H 4
		#define P_ID 5
		#define P_BAUD_RATE 6
		#define P_RETURN_DELAY_TIME 7
		#define P_RETURN_LEVEL 8
		#define P_MIN_ANGLE_LIMIT_L 9
		#define P_MIN_ANGLE_LIMIT_H 10
		#define P_MAX_ANGLE_LIMIT_L 11
		#define P_MAX_ANGLE_LIMIT_H 12
		#define P_LIMIT_TEMPERATURE 13
		#define P_MAX_LIMIT_VOLTAGE 14
		#define P_MIN_LIMIT_VOLTAGE 15
		#define P_MAX_TORQUE_L 16
		#define P_MAX_TORQUE_H 17
		#define P_ALARM_LED 18
		#define P_ALARM_SHUTDOWN 19
		#define P_COMPLIANCE_P 21
		#define P_COMPLIANCE_D 22
		#define P_COMPLIANCE_I 23
		#define P_PUNCH_H 24
		#define P_PUNCH_L 25
		#define P_CW_COMPLIANCE_MARGIN 26
		#define P_CCW_COMPLIANCE_MARGIN 27

		#define P_TORQUE_ENABLE (31)
		#define P_LED (32)
		#define P_GOAL_POSITION_L (33)
		#define P_GOAL_POSITION_H (34)
		#define P_GOAL_SPEED_L (35)
		#define P_GOAL_SPEED_H (36)
		#define P_LOCK (37)

		#define P_PRESENT_POSITION_L (41)
		#define P_PRESENT_POSITION_H (42)
		#define P_PRESENT_SPEED_L (43)
		#define P_PRESENT_SPEED_H (44)
		#define P_PRESENT_LOAD_L (45)
		#define P_PRESENT_LOAD_H (46)
		#define P_PRESENT_VOLTAGE (47)
		#define P_PRESENT_TEMPERATURE (48)
		#define P_REGISTERED_INSTRUCTION (49)
		#define P_ERROR (50)
		#define P_MOVING (51)
	
	//Instruction:
		#define INST_PING 0x01
		#define INST_READ 0x02
		#define INST_WRITE 0x03
		#define INST_REG_WRITE 0x04
		#define INST_ACTION 0x05
		#define INST_RESET 0x06	
		#define INST_SYNC_WRITE 0x83
};
#endif

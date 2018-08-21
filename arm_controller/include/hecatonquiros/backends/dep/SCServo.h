// FEETECH DRIVER
//
//	Original code from FEETECH company. 2016.8.9
//
//
//	Adapted version for C++. 2018-04-19
//

#ifndef HECATONQUIROS_ARMCONTROLLER_BACKENDS_DEP_SCSERVO_h_
#define HECATONQUIROS_ARMCONTROLLER_BACKENDS_DEP_SCSERVO_h_

#include <string>
#include <mutex>
#include <map>
#include <iostream>
#include <stdio.h>

#include <serial/serial.h>

/// \brief Class for controlling SC servos from FEETECH.
/// The class takes control of the given serial port and it is thread safe
class SCServo{
	private: // Static methods
		class SafeSerial{	/// 666 ugly old unnecessary class, please remove asap
			public:
				bool init(std::string &_port){
					mSerial = new serial::Serial(_port, 1000000, serial::Timeout::simpleTimeout(30));
					return mSerial->isOpen();
				}
				std::string readline(){
					return mSerial->readline();
				}
				int read(unsigned char *_buffer, int _len){
					return mSerial->read(_buffer, _len);
				}
				void write(std::string &_cmd){
            		mSerial->write(_cmd);
				}
				void write(unsigned char *_buffer, int _len){
            		mSerial->write(_buffer, _len);
				}
				void flush(){
            		mSerial->flush();
				}
				bool isOpen(){
            		return mSerial->isOpen();
				}
			private:
				serial::Serial *mSerial;
		};

		static std::map<std::string, SafeSerial*> mSerialPorts;

		// Initialize or get serial port if exists
		static bool init(std::string &_portName){
			if(mSerialPorts.find(_portName) != mSerialPorts.end()){
				return true;
			}else{
				SafeSerial *serial = new SafeSerial;
				if(serial->init(_portName)){
					mSerialPorts[_portName] = serial;
					return true;
				}else{
					return false;
				}
			}
		}

	public:	// Custom types
		typedef		char			s8;
		typedef		unsigned char	u8;	
		typedef		unsigned short	u16;	
		typedef		short			s16;
		typedef		unsigned long	u32;	
		typedef		long			s32;

	public:
		/// Default constructor. Initializes internal variables and serial port
		/// \brief _port: given serial port
		SCServo(std::string _serialPort);
		bool isConnected();
		void SyncWritePos(u8 ID[], u8 IDN, u16 Position[], u16 Time[], u16 Speed[] = 0);//ͬ��дλ��ָ��
		int ReadPos(u8 ID);//��λ��
		int WritePos(u8 ID, u16 Position, u16 Time, u16 Speed = 0);//��ͨдλ��ָ��
		int ReadLoadH(u8 ID);
		int EnableTorque(u8 ID, u8 Enable);//Ť������ָ��



	private:

		int genWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen);//��ͨдָ��
		int regWrite(u8 ID, u8 MemAddr,u8 *nDat, u8 nLen);//�첽дָ��
		void syncWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 *nDat[], u8 nLen);//ͬ��дָ��
		int writeByte(u8 ID, u8 MemAddr, u8 bDat);//д1���ֽ�
		int writeWord(u8 ID, u8 MemAddr, u16 wDat);//д2���ֽ�
		int RegWritePos(u8 ID, u16 Position, u16 Time, u16 Speed = 0);//�첽дλ��ָ��
		void RegWriteAction();//ִ���첽дָ��
		int WriteSpe(u8 ID, s16 Speed);//��Ȧ����ָ��
		int Read(u8 ID, u8 MemAddr, u8 *nData, u8 nLen);//��ָ��
		int readByte(u8 ID, u8 MemAddr);//��1���ֽ�
		int readWord(u8 ID, u8 MemAddr);//��2���ֽ�
		int ReadVoltage(u8 ID);//����ѹ
		int ReadTemper(u8 ID);//���¶�
		int ReadLoadL(u8 ID);
		int Ping(u8 ID);//Pingָ��
		void reBoot(u8 ID);
		int wheelMode(u8 ID);//��Ȧ����ģʽ
		int joinMode(u8 ID, u16 minAngle=0, u16 maxAngle=1023);//��ͨ�ŷ�ģʽ
		int Reset(u8 ID);//��λ�������ΪĬ��ֵ
	public:
		u8	Level;//������صȼ�
		u8	End;//��������С�˽ṹ

	private:
		SafeSerial *mSerialConnection = nullptr;
		std::mutex mLocker;
		
	private:
		void writeBuf(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Fun);
		int writePos(u8 ID, u16 Position, u16 Time, u16 Speed, u8 Fun);
		void Host2SCS(u8 *DataL, u8* DataH, int Data);//1��16λ�����Ϊ2��8λ��
		int	SCS2Host(u8 DataL, u8 DataH);//2��8λ�����Ϊ1��16λ��
		int	Ack(u8 ID);//����Ӧ��

		#define		B_1M		0
		#define		B_0_5M		1
		#define		B_250K		2
		#define		B_128K		3
		#define		B_115200	4
		#define		B_76800		5
		#define		B_57600		6
		#define		B_38400		7

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
		#define P_PUNCH_L 24
		#define P_PUNCH_H 25
		#define P_CW_DEAD 26
		#define P_CCW_DEAD 27
		#define P_IMAX_L 28
		#define P_IMAX_H 29

		#define P_TORQUE_ENABLE 40
		#define P_LED 41
		#define P_GOAL_POSITION_L 42
		#define P_GOAL_POSITION_H 43
		#define P_GOAL_TIME_L 44
		#define P_GOAL_TIME_H 45
		#define P_GOAL_SPEED_L 46
		#define P_GOAL_SPEED_H 47
		#define P_LOCK 48

		#define P_PRESENT_POSITION_L 56
		#define P_PRESENT_POSITION_H 57
		#define P_PRESENT_SPEED_L 58
		#define P_PRESENT_SPEED_H 59
		#define P_PRESENT_LOAD_L 60
		#define P_PRESENT_LOAD_H 61
		#define P_PRESENT_VOLTAGE 62
		#define P_PRESENT_TEMPERATURE 63
		#define P_REGISTERED_INSTRUCTION 64
		#define P_ERROR 65
		#define P_MOVING 66
		#define P_VIR_POSITION_L 67
		#define P_VIR_POSITION_H 68

		//Instruction:
		#define INST_PING 0x01
		#define INST_READ 0x02
		#define INST_WRITE 0x03
		#define INST_REG_WRITE 0x04
		#define INST_ACTION 0x05
		#define INST_RESET 0x06
		#define	INST_REBOOT	0x08
		#define INST_SYNC_WRITE 0x83
};
#endif

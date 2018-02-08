/*
 * SCSProtocol.h
 * SCS串行舵机协议程序
 * 日期: 2016.8.25
 * 作者: 谭雄乐
 */

#ifndef _SCSPROTOCOL_H
#define _SCSPROTOCOL_H

typedef		char			s8;
typedef		unsigned char	u8;	
typedef		unsigned short	u16;	
typedef		short			s16;
typedef		unsigned long	u32;	
typedef		long			s32;

class SCSProtocol{
public:
	SCSProtocol();
	int genWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen);//普通写指令
	int regWrite(u8 ID, u8 MemAddr,u8 *nDat, u8 nLen);//异步写指令
	void snycWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 *nDat, u8 nLen);//同步写指令
	int writeByte(u8 ID, u8 MemAddr, u8 bDat);//写1个字节
	int writeWord(u8 ID, u8 MemAddr, u16 wDat);//写2个字节
	int EnableTorque(u8 ID, u8 Enable);//扭力控制指令
	int WritePos(u8 ID, u16 Position, u16 Time, u16 Speed = 0);//普通写位置指令
	int RegWritePos(u8 ID, u16 Position, u16 Time, u16 Speed = 0);//异步写位置指令
	void RegWriteAction();//执行异步写指令
	void SyncWritePos(u8 ID[], u8 IDN, u16 Position, u16 Time, u16 Speed = 0);//同步写位置指令
	int WriteSpe(u8 ID, s16 Speed);//多圈控制指令
	int Read(u8 ID, u8 MemAddr, u8 *nData, u8 nLen);//读指令
	int readByte(u8 ID, u8 MemAddr);//读1个字节
	int readWord(u8 ID, u8 MemAddr);//读2个字节
	int ReadPos(u8 ID);//读位置
	int ReadVoltage(u8 ID);//读电压
	int ReadTemper(u8 ID);//记温度
	int Ping(u8 ID);//Ping指令
	void reBoot(u8 ID);
	int wheelMode(u8 ID);//多圈轮子模式
	int joinMode(u8 ID, u16 minAngle=0, u16 maxAngle=1023);//普通伺服模式
	int Reset(u8 ID);//复位舵机参数为默认值
public:
	u8	Level;//舵机返回等级
	u8	End;//处理器大小端结构
protected:
	virtual int writeSCS(unsigned char *nDat, int nLen) = 0;
	virtual int readSCS(unsigned char *nDat, int nLen) = 0;
	virtual int writeSCS(unsigned char bDat) = 0;
	virtual void flushSCS() = 0;
private:
	void writeBuf(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Fun);
	int writePos(u8 ID, u16 Position, u16 Time, u16 Speed, u8 Fun);
	void Host2SCS(u8 *DataL, u8* DataH, int Data);//1个16位数拆分为2个8位数
	int	SCS2Host(u8 DataL, u8 DataH);//2个8位数组合为1个16位数
	int	Ack(u8 ID);//返回应答

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
	#define P_END 2
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
	#define P_H_L 18
	#define P_ALARM_SHUTDOWN 19
	#define P_ALARM_LED 20
	#define P_COMPLIANCE_P 21
	#define P_COMPLIANCE_D 22
	#define P_COMPLIANCE_I 23
	#define P_PUNCH_L 24
	#define P_PUNCH_H 25
	#define P_CW_DEAD 26
	#define P_CCW_DEAD 27
	#define P_IMAX_L 28
	#define P_IMAX_H 29
	#define P_ADT 30
	#define P_ACC 31
	#define P_SETP 32
	#define P_OFSL 33
	#define P_OFSH 34
	#define P_MODE 35
	#define P_MAX_CURRENT_L 36
	#define P_MAX_CURRENT_H 37

	#define P_TORQUE_ENABLE 40
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
	#define P_MOVING 66
	#define P_PRESENT_GOALPOS_L 67
	#define P_PRESENT_GOALPOS_H 68
	#define P_PRESENT_CURRENT_L 69
	#define P_PRESENT_CURRENT_H 70

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

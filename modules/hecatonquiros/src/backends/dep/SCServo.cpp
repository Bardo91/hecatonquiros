// FEETECH DRIVER
//
//	Original code from FEETECH company. 2016.8.9
//
//
//	Adapted version for C++. 2018-04-19
//


#include <hecatonquiros/backends/dep/SCServo.h>

SCServo::SCServo(std::string _serialPort) {
	Level = 1;
	End = 1;
	if(mSerial == nullptr){
		if(this->init(_serialPort)){
			mSerial = mSerialPorts[_serialPort];
		}
	}
	
}

//---------------------------------------------------------------------------------------------------------------------
bool SCServo::isConnected(){
	try {
		return mSerial != nullptr && mSerial->isOpen();
	}catch (std::exception& e){
		std::cerr << e.what() << std::endl;
	}
	return false;
}

//---------------------------------------------------------------------------------------------------------------------
int SCServo::EnableTorque(u8 ID, u8 Enable) {
	return writeByte(ID, P_TORQUE_ENABLE, Enable);
}

//---------------------------------------------------------------------------------------------------------------------
int SCServo::WritePos(u8 ID, u16 Position, u16 Time, u16 Speed) {
	return writePos(ID, Position, Time, Speed, INST_WRITE);
}

//---------------------------------------------------------------------------------------------------------------------
int SCServo::RegWritePos(u8 ID, u16 Position, u16 Time, u16 Speed) {
	return writePos(ID, Position, Time, Speed, INST_REG_WRITE);
}

//---------------------------------------------------------------------------------------------------------------------
void SCServo::SyncWritePos(u8 ID[], u8 IDN, u16 Position[], u16 Time[], u16 Speed[]) {
	try {
		mSerial->flush();		
		u8 *buf_send[IDN];					
		u8 buf[6];
		for (int i = 0; i<IDN; i++){
			Host2SCS(buf+0, buf+1, Position[i]);
			Host2SCS(buf+2, buf+3, Time[i]);
			Host2SCS(buf+4, buf+5, Speed[i]);
			buf_send[i] = new u8 [6];						// 666 TODO FREE MEMORY
			std::memcpy(buf_send[i], buf, sizeof(u8)*6);
		}
		syncWrite(ID, IDN, P_GOAL_POSITION_L, buf_send, 6);	
	}catch (std::exception& e){
		std::cerr << e.what() << std::endl;
	}
}

//---------------------------------------------------------------------------------------------------------------------
int SCServo::ReadPos(u8 ID) {
	return readWord(ID, P_PRESENT_POSITION_L);
}

//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
void SCServo::Host2SCS(u8 *DataL, u8* DataH, int Data) {
	if(End) {
		*DataL = (Data>>8);
		*DataH = (Data&0xff);
	} else {
		*DataH = (Data>>8);
		*DataL = (Data&0xff);
	}
}

//---------------------------------------------------------------------------------------------------------------------
int SCServo::SCS2Host(u8 DataL, u8 DataH) {
	int Data;
	if(End){
		Data = DataL;
		Data<<=8;
		Data |= DataH;
	}else{
		Data = DataH;
		Data<<=8;
		Data |= DataL;
	}
	return Data;
}

//---------------------------------------------------------------------------------------------------------------------
void SCServo::writeBuf(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Fun) {

	try {
		u8 msgLen = 2;
		u8 bBuf[6];
		u8 CheckSum = 0;
		bBuf[0] = 0xff;
		bBuf[1] = 0xff;
		bBuf[2] = ID;
		bBuf[4] = Fun;
		if(nDat){
			msgLen += nLen + 1;
			bBuf[3] = msgLen;
			bBuf[5] = MemAddr;
			mSerial->write(bBuf, 6);		
		}else{
			bBuf[3] = msgLen;
			mSerial->write(bBuf, 5);		
		}
		CheckSum = ID + msgLen + Fun + MemAddr;
		u8 i = 0;
		if(nDat){
			for(i=0; i<nLen; i++){
				CheckSum += nDat[i];
			}
		}
		mSerial->write(nDat, nLen);
		u8 negCheckSum = ~CheckSum;
		mSerial->write(&negCheckSum, 1);	
	}catch (std::exception& e){
		std::cerr << e.what() << std::endl;
	}		
}

//---------------------------------------------------------------------------------------------------------------------
int SCServo::genWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen) {
	try {
		mSerial->flush();		
	}catch (std::exception& e){
		std::cerr << e.what() << std::endl;
	}
	writeBuf(ID, MemAddr, nDat, nLen, INST_WRITE);
	return Ack(ID);
}

//---------------------------------------------------------------------------------------------------------------------
int SCServo::regWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen) {
	try {
		mSerial->flush();		
	}catch (std::exception& e){
		std::cerr << e.what() << std::endl;
	}
	writeBuf(ID, MemAddr, nDat, nLen, INST_REG_WRITE);
	return Ack(ID);
}

//---------------------------------------------------------------------------------------------------------------------
void SCServo::syncWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 *nDat[], u8 nLen) {
	try {
		u8 mesLen = ((nLen+1)*IDN+4);
		u8 Sum = 0;
		u8 bBuf[7];
		bBuf[0] = 0xff;
		bBuf[1] = 0xff;
		bBuf[2] = 0xfe;
		bBuf[3] = mesLen;
		bBuf[4] = INST_SYNC_WRITE;
		bBuf[5] = MemAddr;
		bBuf[6] = nLen;
		mSerial->write(bBuf, 7);

		Sum = 0xfe + mesLen + INST_SYNC_WRITE + MemAddr + nLen;
		u8 i, j, k;
		for(i=0; i<IDN; i++){
			mSerial->write(&ID[i],1);
			mSerial->write(nDat[i], nLen);
			//std::cout << "nDat: " << (int) *nDat[i] << std::endl;

			Sum += ID[i];
			for(j=0; j<nLen; j++){
					Sum += nDat[i][j];
			}
		}
		u8 negSum = ~Sum;
		mSerial->write(&negSum,1);		
	}catch (std::exception& e){
		std::cerr << e.what() << std::endl;
	}
}

//---------------------------------------------------------------------------------------------------------------------
int SCServo::writeByte(u8 ID, u8 MemAddr, u8 bDat) {
	try {
		mSerial->flush();		
	}catch (std::exception& e){
		std::cerr << e.what() << std::endl;
	}
	writeBuf(ID, MemAddr, &bDat, 1, INST_WRITE);
	return Ack(ID);
}

//---------------------------------------------------------------------------------------------------------------------
int SCServo::writeWord(u8 ID, u8 MemAddr, u16 wDat) {
	try {
		mSerial->flush();		
	}catch (std::exception& e){
		std::cerr << e.what() << std::endl;
	}
	u8 buf[2];
	Host2SCS(buf+0, buf+1, wDat);
	writeBuf(ID, MemAddr, buf, 2, INST_WRITE);
	return Ack(ID);
}


//---------------------------------------------------------------------------------------------------------------------
int SCServo::writePos(u8 ID, u16 Position, u16 Time, u16 Speed, u8 Fun) {
	try {
		mSerial->flush();		
	}catch (std::exception& e){
		std::cerr << e.what() << std::endl;
	}
	u8 buf[6];
	Host2SCS(buf+0, buf+1, Position);
	Host2SCS(buf+2, buf+3, Time);
	Host2SCS(buf+4, buf+5, Speed);
	writeBuf(ID, P_GOAL_POSITION_L, buf, 6, Fun);
	return Ack(ID);
	//return 1;
}



//---------------------------------------------------------------------------------------------------------------------
void SCServo::RegWriteAction() {
	writeBuf(0xfe, 0, NULL, 0, INST_ACTION);
}


//---------------------------------------------------------------------------------------------------------------------
int SCServo::Read(u8 ID, u8 MemAddr, u8 *nData, u8 nLen) {
	try {
		mSerial->flush();
		writeBuf(ID, MemAddr, &nLen, 1, INST_READ);
		u8 bBuf[5];
		if(mSerial->read(bBuf, 5)!=5){
			return 0;
		}
		int size = mSerial->read(nData, nLen);
		if(mSerial->read(bBuf, 1)){
			return size;
		}		
	}catch (std::exception& e){
		std::cerr << e.what() << std::endl;
	}
	return 0;
}

//---------------------------------------------------------------------------------------------------------------------
int SCServo::readByte(u8 ID, u8 MemAddr) {
	u8 bDat;
	int Size = Read(ID, MemAddr, &bDat, 1);
	if(Size!=1){
		return -1;
	}else{
		return bDat;
	}
}

//---------------------------------------------------------------------------------------------------------------------
int SCServo::readWord(u8 ID, u8 MemAddr) {	
	u8 nDat[2];
	int Size;
	Size = Read(ID, MemAddr, nDat, 2);
	if(Size!=2)
		return -1;
		
	u16 wDat;
	wDat = SCS2Host(nDat[0], nDat[1]);
	return wDat;
}

//---------------------------------------------------------------------------------------------------------------------
int SCServo::WriteSpe(u8 ID, s16 Speed) {
	if(Speed<0){
		Speed = -Speed;
		Speed |= (1<<10);
	}
	return writeWord(ID, P_GOAL_TIME_L, Speed);
}

//---------------------------------------------------------------------------------------------------------------------
int SCServo::ReadVoltage(u8 ID) {	
	return readByte(ID, P_PRESENT_VOLTAGE);
}

//---------------------------------------------------------------------------------------------------------------------
int SCServo::ReadTemper(u8 ID) {	
	return readByte(ID, P_PRESENT_TEMPERATURE);
}

//---------------------------------------------------------------------------------------------------------------------
int SCServo::ReadLoadL(u8 ID) {	
	return readByte(ID, P_PRESENT_LOAD_L);
}

//---------------------------------------------------------------------------------------------------------------------
int SCServo::ReadLoadH(u8 ID) {	
	return readByte(ID, P_PRESENT_LOAD_H);
}

//---------------------------------------------------------------------------------------------------------------------
int SCServo::Ping(u8 ID) {
	try {
		mSerial->flush();
		u8 bBuf[6];
		writeBuf(ID, 0, NULL, 0, INST_PING);
		int size = mSerial->read(bBuf, 6);
		if(size==6){
			return bBuf[2];
		}		
	}catch (std::exception& e){
		std::cerr << e.what() << std::endl;
	}
	return -1;
}

//---------------------------------------------------------------------------------------------------------------------
void SCServo::reBoot(u8 ID) {
	writeBuf(ID, 0, NULL, 0, INST_REBOOT);
}

//---------------------------------------------------------------------------------------------------------------------
int SCServo::wheelMode(u8 ID) {
	u8 bBuf[4];
	bBuf[0] = 0;
	bBuf[1] = 0;
	bBuf[2] = 0;
	bBuf[3] = 0;
	return genWrite(ID, P_MIN_ANGLE_LIMIT_L, bBuf, 4);
}

//---------------------------------------------------------------------------------------------------------------------
int SCServo::joinMode(u8 ID, u16 minAngle, u16 maxAngle) {
	u8 bBuf[4];
	Host2SCS(bBuf, bBuf+1, minAngle);
	Host2SCS(bBuf+2, bBuf+3, maxAngle);
	return genWrite(ID, P_MIN_ANGLE_LIMIT_L, bBuf, 4);
}

//---------------------------------------------------------------------------------------------------------------------
int SCServo::Reset(u8 ID) {
	try {
		mSerial->flush();		
	}catch (std::exception& e){
		std::cerr << e.what() << std::endl;
	}
	writeBuf(ID, 0, NULL, 0, INST_RESET);
	return Ack(ID);
}

//---------------------------------------------------------------------------------------------------------------------
int	SCServo::Ack(u8 ID) {
	if(ID != 0xfe && Level){
		u8 buf[6];
		try {
			u8 Size = mSerial->read(buf, 6);
			if(Size!=6){
				return 0;
			}		
		}catch (std::exception& e){
			std::cerr << e.what() << std::endl;
		}
	}
	return 1;
}


std::map<std::string, serial::Serial*> SCServo::mSerialPorts;
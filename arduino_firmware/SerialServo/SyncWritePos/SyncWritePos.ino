#include <SCServo.h>

SCServo SERVO;
u8 ID[2];
void setup()
{
	Serial.begin(1000000);//init Serial baudrate
        delay(500);
        SERVO.EnableTorque(1, 1);
        SERVO.EnableTorque(2, 1);
        ID[0] = 1;
        ID[1] = 2;
}

void loop()
{
	SERVO.SyncWritePos(ID, 2, 0, 50);
	delay(2000);
	SERVO.SyncWritePos(ID, 2, 0x02ff, 100);
	delay(2000);
}

#include <SCServo.h>

SCServo SERVO;

void setup()
{
	Serial.begin(1000000);//init Serial baudrate
	delay(500);
	SERVO.EnableTorque(1, 1);
	SERVO.EnableTorque(2, 1);
}

void loop()
{
	SERVO.RegWritePos(1, 1023, 50);
	SERVO.RegWritePos(2, 1023, 50);
	SERVO.RegWriteAction();
	delay(2000);
	
	SERVO.RegWritePos(1, 0, 100);
	SERVO.RegWritePos(2, 0, 100);
	SERVO.RegWriteAction();
	delay(2000);
}

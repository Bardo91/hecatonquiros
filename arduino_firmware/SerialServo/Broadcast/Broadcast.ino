#include <SCServo.h>

SCServo SERVO;

void setup()
{
	Serial.begin(1000000);//init Serial baudrate
        delay(500);
        SERVO.EnableTorque(0xfe,1);
}

void loop()
{
	u16 i;
	for(i = 0;i < 1024; i++)
	{
		SERVO.WritePos(0xfe,i,100);//All Servo(broadcast) rotate to the position:i
		delay(10);
	}
	for(i = 1024; i > 0; i--)
	{
		SERVO.WritePos(0xfe,i,100);//All Servo(broadcast) rotate to the position:i
		delay(10);
	}
}

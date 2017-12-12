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
	SERVO.WritePos(1, 0x02FF, 25);// Servo ID:1, rotate to the position:0x2FF
	SERVO.WritePos(2, 0x02FF, 25);// Servo ID:2, rotate to the position:0x2FF
	delay(2000); 
	SERVO.WritePos(1,0x0000, 50);// Servo ID:1, rotate to the position:0x000
	SERVO.WritePos(2, 0x0000, 50);// Servo ID:1, rotate to the position:0x000
	delay(2000);
	SERVO.WritePos(1, 0x02FF, 75);// Servo ID:1, rotate to the position:0x2FF
	SERVO.WritePos(2, 0x02FF, 75);// Servo ID:2, rotate to the position:0x2FF
	delay(2000);
	SERVO.WritePos(1, 0x0000, 100);// Servo ID:1, rotate to the position:0x000
	SERVO.WritePos(2, 0x0000, 100);// Servo ID:2, rotate to the position:0x000
	delay(2000);
}

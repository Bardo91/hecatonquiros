#include <SCServo.h>

SCServo SERVO;

void setup()
{
	Serial.begin(1000000);//init Serial baudrate
        delay(500);
        //SERVO.EnableTorque(1, 1);
        SERVO.EnableTorque(2, 1);
}

void loop()
{
        s16 pos = SERVO.ReadPos(1);//read Servo ID:1 position
	if(pos!=-1)
        {
        	SERVO.WritePos(2, pos, 100);//Servo ID:2, rotate to the position:Servo ID:1
        }
        delay(5);
}

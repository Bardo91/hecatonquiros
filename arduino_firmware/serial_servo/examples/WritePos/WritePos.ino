#include <SCServo.h>

SCServo SERVO;

void setup()
{
  Serial1.begin(1000000);
  SERVO.pSerial = &Serial1;
  delay(500);
  SERVO.EnableTorque(21, 1);
  SERVO.EnableTorque(22, 1);
}

void loop()
{
  SERVO.WritePos(21, 1023, 4000);// Servo ID:1, rotate to the position:0x2FF
  SERVO.WritePos(22, 1023, 4000);// Servo ID:2, rotate to the position:0x2FF
  delay(4000); 
  SERVO.WritePos(21,0x0000, 3000);// Servo ID:1, rotate to the position:0x000
  SERVO.WritePos(22, 0x0000, 3000);// Servo ID:1, rotate to the position:0x000
  delay(3000);
  SERVO.WritePos(21, 1023, 2000);// Servo ID:1, rotate to the position:0x2FF
  SERVO.WritePos(22, 1023, 2000);// Servo ID:2, rotate to the position:0x2FF
  delay(2000);
  SERVO.WritePos(21, 0x0000, 1000);// Servo ID:1, rotate to the position:0x000
  SERVO.WritePos(22, 0x0000, 1000);// Servo ID:2, rotate to the position:0x000
  delay(1000);
}

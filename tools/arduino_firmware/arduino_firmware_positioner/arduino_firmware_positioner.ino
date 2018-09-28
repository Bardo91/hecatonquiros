//---------------------------------------------------------------------------------------------------------------------
//  HECATONQUIROS
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 ViGUS University of Seville
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
//  and associated documentation files (the "Software"), to deal in the Software without restriction, 
//  including without limitation the rights to use, copy, modify, merge, publish, distribute, 
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial 
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES 
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------


#include "Positioner.h"
#include "LSM6.h"
#include <Wire.h>
#include <Servo.h>

// Arduino   LSM6 board
// -------   ----------
//      5V - VIN
//     GND - GND
//     SDA - SDA
//     SCL - SCL


Positioner tool(A0, A1, A2, A3, A4);
LSM6 imu;
Servo baseLocker;
bool isLocked = true;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!imu.init()) {
    while (1);
  }
  baseLocker.attach(8); 
  baseLocker.write(180);
  imu.enableDefault();
}

void exec             (String _cmd);
void execPositioner   (String _cmd);

String command;
void loop() {
  if(Serial.available()){
    delay(1);
    char c = Serial.read();  //gets one byte from serial buffer
    ////Serial.println(c);
    if(c == '\r'){
      ////Serial.println("Found r");
      delay(1);
      if(Serial.read() == '\n'){
        ////Serial.println("Found n. Executing command");
        exec(command);
        command = "";
      }else{
        // Brokend endline, empty it
        command = "";
      }
    }else{
      command +=c;
    }
  }
}



void exec(String _cmd){
  if(_cmd[0] == 'p'){
    //Serial.println("Command of type positioner");
    execPositioner(_cmd.substring(1));
  }else if(_cmd[0] == 'l'){
    if(isLocked){
      baseLocker.write(0);
      isLocked = false;  
    }else{
      baseLocker.write(180);
      isLocked = true;
    }
  }else{
    // Do nothing
  }
}

void execPositioner(String _cmd){
  float t0,t1,t2,t3, t4;
  tool.rawJoints(t0,t1,t2,t3, t4);
  imu.read();
  String strJoints = String(t0) + ", " +String(t1) + ", " +String(t2)+ ", " +String(t3)+ ", " +String(t4)+", "+
                     String(imu.a.x) + ", " +String(imu.a.y) + ", " +String(imu.a.z) ;
                     
  Serial.println(strJoints);
}



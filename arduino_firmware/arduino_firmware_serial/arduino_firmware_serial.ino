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
#include "Arm4DoF.h"

Positioner tool(A0, A1, A2, A3, A4);
Arm4DoF leftArm;
Arm4DoF rightArm;

void setup() {
  Serial.begin(115200);

  leftArm.setup(1);
  leftArm.offsets(90,90,90,90);

  rightArm.setup(2);
  rightArm.offsets(90,90,90,90);
  
  
}

void exec             (String _cmd);
void execArm          (String _cmd, Arm4DoF *_arm);
void execWrist        (String _cmd, Arm4DoF *_arm);
void execClaw         (char _cmd, Arm4DoF *_arm);
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
  if(_cmd[0] == 'a'){
    //Serial.println("command of type arm");
    if(_cmd[1] == '1'){
      //Serial.println("Left arm");
      execArm(_cmd.substring(2), &leftArm);
    }else if(_cmd[1] == '2'){
      //Serial.println("Right arm");
      execArm(_cmd.substring(2), &rightArm);
    }
  }else if(_cmd[0] == 'p'){
    //Serial.println("Command of type positioner");
    execPositioner(_cmd.substring(1));
  }else if(_cmd[0] == 'c'){
    //Serial.println("command of claw");
    if(_cmd[1] == '1'){
      execClaw(_cmd[2], &leftArm);
    }else if(_cmd[1] == '2'){
      execClaw(_cmd[2], &rightArm);
    }
    }else if(_cmd[0] == 'w'){
    //Serial.println("command of wrist");
    if(_cmd[1] == '1'){
      execWrist(_cmd.substring(2), &leftArm);
    }else if(_cmd[1] == '2'){
      execWrist(_cmd.substring(2), &rightArm);
    }
  }else{
    // Do nothing
  }
}

void execArm(String _cmd, Arm4DoF *_arm){
  float signals[6];
  int numberJoints = 0;
  for(unsigned i = 0; i < 6; i++){
    int idx = _cmd.indexOf(',');
    if( idx == -1 ){  // case for only 3 joints, we dont need wrist, so put it to 0
       signals[i] = atoi(_cmd.c_str());
        numberJoints++;
       break;
    }
    else{
      signals[i] = atoi(_cmd.substring(0,idx).c_str());
      _cmd = _cmd.substring(idx+1);
      numberJoints++;
    }
  }
   Serial.println(numberJoints); 
  _arm->joints(signals, numberJoints);
}

void execWrist(String _cmd, Arm4DoF *_arm){
    int AngleWrist;
    //Serial.println(_cmd);
    AngleWrist = atoi(_cmd.c_str());
    //Serial.println(AngleWrist);
    _arm->wrist(AngleWrist);
}

void execClaw(char _cmd, Arm4DoF *_arm){
  //Serial.println(_cmd);
  if(_cmd == 'o'){
    _arm->openGripper();
    //Serial.println("Open claw");
  }else if(_cmd == 'c'){
    _arm->closeGripper();
    //Serial.println("Close claw"); 
  }else{
    // Do nothing
  }
  
}

void execPositioner(String _cmd){
  float t0,t1,t2,t3, t4;
  tool.rawJoints(t0,t1,t2,t3, t4);
  String strJoints = String(t0) + ", " +String(t1) + ", " +String(t2)+ ", " +String(t3)+ ", " +String(t4);
  Serial.println(strJoints);
}



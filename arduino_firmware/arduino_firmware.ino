//
// 
//
//
//
//

#include "Positioner.h"
#include "Arm.h"

Positioner tool(A0, A1, A2, A3, A4);
Arm leftArm;
Arm rightArm;

void setup() {
  Serial.begin(115200);

  leftArm.setup(8,9,10,11,12);
  leftArm.offsets(75,100,80,40,110);
  
  rightArm.setup(2,3,4,5,6);
  rightArm.offsets(75,90,85,90,96);
  
  
  leftArm.speed(20);
  rightArm.speed(20);

  leftArm.joints(0,0,90,0);
  leftArm.wrist(0);
  rightArm.joints(0,0,90,0);
  rightArm.wrist(0);

  leftArm.stopGripper();
  rightArm.stopGripper();
  
}

void exec             (String _cmd);
void execArm          (String _cmd, Arm *_arm);
void execWrist        (String _cmd, Arm *_arm);
void execClaw         (char _cmd, Arm *_arm);
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
      execArm(_cmd.substring(2), &leftArm);
    }else if(_cmd[1] == '2'){
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

void execArm(String _cmd, Arm *_arm){
  int signals[4] = {0, 0, 0, 0};
  for(unsigned i = 0; i < 4; i++){
    int idx = _cmd.indexOf(',');
    if( idx == -1 ){  // case for only 3 joints, we dont need wrist, so put it to 0
       signals[i] = atoi(_cmd.c_str());
       break;
    }
    else{
    signals[i] = atoi(_cmd.substring(0,idx).c_str());
    _cmd = _cmd.substring(idx+1);
    }
  }
  
  _arm->joints(signals[0], signals[1], signals[2], signals[3]);
}

void execWrist(String _cmd, Arm *_arm){
    int AngleWrist;
    //Serial.println(_cmd);
    AngleWrist = atoi(_cmd.c_str());
    //Serial.println(AngleWrist);
    _arm->wrist(AngleWrist);
}

void execClaw(char _cmd, Arm *_arm){
  //Serial.println(_cmd);
  if(_cmd == 'o'){
    _arm->openGripper();
    //Serial.println("Open claw");
  }else if(_cmd == 'c'){
    _arm->closeGripper();
    //Serial.println("Close claw");
  }else if(_cmd == 's'){
    _arm->stopGripper();    
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



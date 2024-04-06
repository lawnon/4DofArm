// Roboter Arm Steuerung

/*===================================================
  Default Home Position
  JT1: 0
  JT2: 30
  JT3: 90
  JT4: -90
  ===================================================*/

#include <Servo.h>
#include "Types.hpp"
#include "Logger.hpp"
#include "Joint.hpp"
#include "Kinematic.hpp"
#include "Commands.hpp"

Kinematic World;
Commands Cmds;

Joint Jt1;
Joint Jt2;
Joint Jt3;
Joint Jt4;

Posture PtHome{
  0, 135, 90, -85
};

/* Transforem String Array in to Posture Structure */
Posture DecodePosture(String input, String filter = ","){

  int delimeter1 = input.indexOf(filter);
  int delimeter2 = input.indexOf(filter, delimeter1 + 1);
  int delimeter3 = input.indexOf(filter, delimeter2 + 1);

  Posture pt{
    input.substring(0, delimeter1).toFloat(),
    input.substring(delimeter1 + 1, delimeter2).toFloat(),
    input.substring(delimeter2 + 1, delimeter3).toFloat(),
    input.substring(delimeter3 + 1, input.length()).toFloat()
  };

  return pt;
}

/* Transform String Arry in to Position Sturcture  */
Position DecodePosition(String input, String filter = ","){

  int delimeter1 = input.indexOf(filter);
  int delimeter2 = input.indexOf(filter, delimeter1 + 1);
  int delimeter3 = input.indexOf(filter, delimeter2 + 1);
  int delimeter4 = input.indexOf(filter, delimeter4 + 1);
  int delimeter5 = input.indexOf(filter, delimeter5 + 1);

  Position pos{
    input.substring(0, delimeter1).toInt(),
    input.substring(delimeter1 + 1, delimeter2).toFloat(),
    input.substring(delimeter2 + 1, delimeter3).toFloat(),
    input.substring(delimeter3 + 1, delimeter4).toFloat(),
    input.substring(delimeter4 + 1, delimeter5).toFloat(),
    input.substring(delimeter5 + 1, input.length()).toFloat()
  };

  return pos;
}

void increment(Joint jt, int origin, int dest, int itr){
  if (origin <= dest) {
    if (origin + itr <= dest) {
      jt.write(origin + itr);
      //log(origin + itr, "Fwd JT" + String(jt.id));
    }
  }
  else{
    if (origin  - itr >= dest){
      jt.write(origin - itr);
      //log(origin - itr, "Bwd JT" + String(jt.id));
    }
  }
}

/* Drive all Axis Simultanously to given Posture */
void drive(Posture pt){
  log("Drive Data: " + String(pt.jt1) + " | " + String(pt.jt2) + " | " + String(pt.jt3) + " | " + String(pt.jt4));


  int origin1 = Jt1.read();
  int origin2 = Jt2.read();
  int origin3 = Jt3.read();
  int origin4 = Jt4.read();


  int delta1 = Jt1.delta(pt.jt1);
  int delta2 = Jt2.delta(pt.jt2);
  int delta3 = Jt3.delta(pt.jt3);
  int delta4 = Jt4.delta(pt.jt4);
  log("Drive Delta: " + String(delta1) + " | " + String(delta2) + " | " + String(delta3) + " | " + String(delta4));

  // Get max delta
  int delta = (abs(delta1) >= abs(delta2)) ? abs(delta1) : abs(delta2);
  delta = (abs(delta) >= abs(delta3)) ? abs(delta) : abs(delta3);
  delta = (abs(delta) >= abs(delta4)) ? abs(delta) : abs(delta4);

  // Iterate Through delta Range and Increment
  for(int itr = 1; itr <= delta; itr++){
    //log(itr, "iterator");
    increment(Jt1, origin1, pt.jt1, itr);
    increment(Jt2, origin2, pt.jt2, itr);
    increment(Jt3, origin3, pt.jt3, itr);
    increment(Jt4, origin4, pt.jt4, itr);
    // Delay with respect to Speed
    Cmds.Delay();
  }
}


/* Drive all Axis Simultanously to given position */
void drive(Position ps){
  Posture pt  = World.ivKinematic(ps);
  log(pt);
  //drive(pt);
}

void gotoPos(String xs, String ys, String zs) {
  Position ps;
  Posture pt;

  ps.x = xs.toInt();
  ps.y = ys.toInt();
  ps.z = zs.toInt();
  ps.a = 0;
  ps.b = 0;
  ps.c = 0;

  pt = World.ivKinematic(ps);
  drive(pt);
}
/*
 * Eingangs Daten aus Serialle Port lesen
 * Auswerten und entsprechend aus führen.
 */
void Decode(String data) {
  // Eingangs Daten lesen
  Command inputCmd = Cmds.Parse(data);

  switch (inputCmd.Id) {
  case Commands::World:
    log("World Command not yet Implemented");
    break;

  case Commands::Jt1:
    Cmds.GotoDeg(Jt1, inputCmd.Value);
    break;

  case Commands::Jt2:
    Cmds.GotoDeg(Jt2, inputCmd.Value);
    break;

  case Commands::Jt3:
    Cmds.GotoDeg(Jt3, inputCmd.Value);
    break;

  case Commands::Jt4:
    Cmds.GotoDeg(Jt4, inputCmd.Value);
    break;

  case Commands::Speed:
    Cmds.SetParam(Commands::Speed, inputCmd.Value);
    break;

  case Commands::Here:
    // Get Angular Positoin of each Joint
    log(Posture {Jt1.read(), Jt2.read(), Jt3.read(), Jt4.read()});
    break;

  case Commands::Pos:
    // Get Carthesian Koordinates
    log(World.fdKinematic(Posture {Jt1.read(), Jt2.read(), Jt3.read(), Jt4.read()}));
    break;

  case Commands::GoHome:
    drive(Posture {PtHome.jt1, PtHome.jt2, PtHome.jt3, PtHome.jt4});
    break;

  case Commands::GotoPos:
    drive(DecodePosition(inputCmd.Content));
    break;

  case Commands::GotoJoint:
    drive(DecodePosture(inputCmd.Content));
    break;
  }
}

void Decode2(){
  if(Serial.available() > 0 ){
    log("Outputing Data to Serial 2");

    int data0 = Serial.read();
    Serial.println(data0);
    Serial.println(String(data0));
    Serial2.write(data0);
  }

  if(Serial2.available() > 0){
    log("Input Data from Serial 2");

    String data2 = Serial2.readString();
    Serial.println(data2);
  }
}

// Load and Initialize Default Data
void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  /*
    while(!Serial){}
    Serial.println("C++ Version: " + String(__cplusplus));
  */
  // Define Joint Structures
  Jt1.id = 1;
  Jt2.id = 2;
  Jt3.id = 3;
  Jt4.id = 4;
  // Attach Joint Parameters
  Jt1.attach(2,0,180,93);
  Jt2.attach(3,0,180,7);
  Jt3.attach(4,0,180,90);
  Jt4.attach(5,0,180,85);
  // Set Up Initial Configurations
  Jt1.write(PtHome.jt1);
  Jt2.write(PtHome.jt2);
  Jt3.write(PtHome.jt3);
  Jt4.write(PtHome.jt4);
}

//Execute Program Here
void loop() {

  // Get Inputs from Serial Port and Process
  if (Serial.available() > 0) {
    Decode(Serial.readString());
  }
  if (Serial2.available() > 0){
    Decode(Serial2.readString());
  }
}

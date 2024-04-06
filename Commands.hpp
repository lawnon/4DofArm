#ifndef COMMANDS_H_
#define COMMANDS_H_

#include <Servo.h>
#include <wiring_private.h>
#include "Types.hpp"
#include "Joint.hpp"
#include "Kinematic.hpp"
#include "Logger.hpp"

class Commands
{
public:
  Commands();

  // Const Difinitions
  enum Tags{
    World = 0,
    // Read Set Joint Positions
    Jt1 = 1,
    Jt2 = 2,
    Jt3 = 3,
    Jt4 = 4,
    // Read Set Parameters
    Speed = 5,
    Here = 6,
    Pos = 7,
    // Drive to  Given Configurations
    GoHome = 8,
    GotoPos = 9,
    GotoJoint = 10,
    // Debug
    Debug = 11
  };

  Command Parse(String input);

  void GotoDeg(Joint jt, float dest);
  void SetParam(int cIndex, float val);
  void LogPosture();
  void LogPosition();
  void Delay();
private:
  int _range = 0;
  Command _list[12];

  void sweep(Joint jt, float origin, float dest);
};

#endif // COMMANDS_H_

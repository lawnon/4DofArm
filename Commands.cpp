#include "Commands.hpp"

/* Command Functions */
Commands::Commands(){
  _list[Commands::World] = {Commands::World, "world"};
  _list[Commands::Jt1] = {Commands::Jt1, "jt1"};
  _list[Commands::Jt2] = {Commands::Jt2, "jt2"};
  _list[Commands::Jt3] = {Commands::Jt3, "jt3"};
  _list[Commands::Jt4] = {Commands::Jt4, "jt4"};
  _list[Commands::Speed] = {Commands::Speed, "speed", 70}; // in Percent
  _list[Commands::Here] = {Commands::Here, "here"};
  _list[Commands::Pos] = {Commands::Pos, "pos"};
  _list[Commands::GoHome] = {Commands::GoHome, "home"};
  _list[Commands::GotoPos] = {Commands::GotoPos, "gotopos"};
  _list[Commands::GotoJoint] = {Commands::GotoJoint, "gotojt"};
  _list[Commands::Debug] = {Commands::Debug, "debug", -1};

  _range = sizeof(_list)/sizeof(_list[0]);
}

Command Commands::Parse(String input){
  input.trim();
  input.toLowerCase();

  log("=======================================>");
  log(input, "Decoding Incoming Data: ");
  log(_range, "Command Range: ");

  int    commandIndex = input.indexOf(" ");
  String commandName  = input.substring(0, commandIndex);
  String commandValue = input.substring(commandIndex, input.length());

  log(commandValue.length(), "Command Value lenght: ");

  for (int itr; itr < _range; itr++) {
    if(_list[itr].Name.equals(commandName)){
      if(commandValue.length() > 0){
        _list[itr].Content = commandValue;
        _list[itr].Value = commandValue.toFloat();
      }
      log(_list[itr]);
      return _list[itr];
    }
  }

  return {-1, "Invalid", 0};
}

void Commands::GotoDeg(Joint jt, float dest) {
  float origin = jt.read();
  Commands::sweep(jt, origin, dest);
}

void Commands::SetParam(int cIndex, float val){
  if(val < 0){
    log(val, "Invalid Parameter Value: ");
    return;
  }

  if (cIndex < 0 & cIndex >= _range){
    log(cIndex, "invalid Param Index: ");
    return;
  }

  _list[cIndex].Value = val;
}

void Commands::LogPosture(){
  // Todo : yet to be implemented
}

void Commands::LogPosition(){

}

/*
  Private Functions Discriptions
*/

/*Drive Joint to given Position*/
void Commands::sweep(Joint jt, float origin, float dest) {
  log("Sweep Started");
  log("Jt" + String(jt.id) + " at " + String(origin));
  log("Jt" + String(jt.id) + " goto " + String(dest));

  if (dest >= origin){
    for (int i = origin; i <= dest; i += 1) {
      // in steps of 1 degree
      jt.write(i);
      Commands::Delay();
    }
  }else{
    for (int i = origin; i >= dest; i -= 1) {
      jt.write(i);
      Commands::Delay();
    }
  }

  log("Sweep Done");
}

void Commands::Delay(){
  delay(10 - _list[Commands::Speed].Value/10);
}

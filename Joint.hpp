#ifndef JOINT_H_
#define JOINT_H_

#include <servo.h>

class Joint{

public:
  Joint();
  void write(int degree);
  void attach(int pin);
  void attach(int pin, int min, int max);
  void attach(int pin, int min, int max, int offset);
  void limit(int min, int max);
  int read();
  int read2();
  int readMicroseconds();
  int delta(int dest);

  int id;
  int offset;
  int min :1;
  int max :180;
  Servo servo;
private:
  int _offset;
};

#endif // JOINT_H_

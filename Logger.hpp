#ifndef LOGGER_H_
#define LOGGER_H_


#include <HardwareSerial.h>
#include <WString.h>
#include "Kinematic.hpp"
#include "Commands.hpp"

void log(String text);
void log(int digit);
void log(String text, String titel);
void log(int digit, String titel);
void log(Posture pt);
void log(Position ps);
void log(Command cmd);

#endif // LOGGER_H_

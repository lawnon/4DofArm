#include "Logger.hpp"

void encode(String data){
  Serial.println(data); // Daten in Lokalen Serial Monitor ausgeben
  Serial2.write(10); // Neuer Zeilen Anfang Kodieren
  for (int i = 0; i < data.length(); i++) {
    Serial2.write(data.charAt(i));
  }
}

void log(String text = ""){
  encode("==>" + text);
}

void log(int digit = -1){
  encode("==>" + String(digit));
}

void log(String text = "", String titel = ""){
  encode("==>" + titel + ": " + text);
}

void log(int digit = -1, String titel = ""){
  encode("==>" + titel + ": " + String(digit));
}

void log(Posture pt){
  encode("===================================================>");
  encode("==>Angular Position of Each Joint");
  encode("==>JT1: " + String(pt.jt1));
  encode("==>JT2: " + String(pt.jt2));
  encode("==>JT3: " + String(pt.jt3));
  encode("==>JT4: " + String(pt.jt4));
}

void log(Position ps){
  encode("===================================================>");
  encode("==>Location and Orientation of Endeffector");
  encode("==>X: " + String(ps.x));
  encode("==>Y: " + String(ps.y));
  encode("==>Z: " + String(ps.z));
  encode("==>A: " + String(ps.a));
  encode("==>B: " + String(ps.b));
  encode("==>C: " + String(ps.c));
}

void log(Command cmd){
  encode("==================================================>");
  encode("==>ID: " + String(cmd.Id)+
                 "| Name: " + cmd.Name +
                 "| Value: " + String(cmd.Value) +
                 "| Content: " + cmd.Content);
}

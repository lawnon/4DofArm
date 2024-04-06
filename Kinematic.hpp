// Kemematisches Modellierung des Roboter Arms

#ifndef KINEMATIC_H_
#define KINEMATIC_H_

#include <math.h>
#include <HardwareSerial.h>
#include <WString.h>
#include "Types.hpp"
#include "Commands.hpp"

// Misc
float getRad(float deg);
float getDeg(float rad);

class Kinematic{
public:
  Kinematic();
  Position fdKinematic(Posture posture);
  Posture ivKinematic(Position position);
  void printMatrix(Matrix4x4 mat, String heading);

  // Danavit Hartenberg Parameter
  DhParam dhPar1;
  DhParam dhPar2;
  DhParam dhPar3;
  DhParam dhPar4;

  // Achsen-Transformations Matrizen
  Matrix4x4 tMat01;
  Matrix4x4 tMat12;
  Matrix4x4 tMat23;
  Matrix4x4 tMat34;
  // Verkettet Transformations Matrizen
  Matrix4x4 tMat02;
  Matrix4x4 tMat03;
  Matrix4x4 tMat04;

private:
  Matrix4x4 jointTMatrix(DhParam parma);
  Matrix4x4 matrixMulp(Matrix4x4 A, Matrix4x4 B);
  Matrix4x4 armTMatrix(Posture posture);
};

#endif // KINEMATIC_H_

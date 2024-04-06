#ifndef TYPES_H_
#define TYPES_H_

/* Kinematik Struct */

/* Denavit Hartenberg-Parameter for all Joints */
struct DhParameters{
  float alpha[4] = {-90.000, 0.000, 0.000, 0.000};
  float link[4] = {62.081, 58.000, 58.000, 66.700};
  float disp[4] = {9.450, 0.000, 0.000, -9.450};
  float theta[4] = {0.000, 0.000, 0.000, 0.000};
};

/* Denavit Hartenberg_Parameter Discription */
struct DhParam{
  float alpha;
  float link;
  float disp;
  float theta;
};

/* Structs Containing a 4 by 4 Matrix */
struct Matrix4x4{
  float m11[4]; float m12[4]; float m13[4]; float m14[4];
};

/* Location and Orientation Struct */
struct Position{
  float x;
  float y;
  float z;
  float a;
  float b;
  float c;
};

/* Angular Constelation of Joints */
struct Posture{
  float jt1;
  float jt2;
  float jt3;
  float jt4;
};



/* Command Struct */
struct Command {
  int Id;
  String Name;
  float Value;
  String Content;
};

#endif // TYPES_H_

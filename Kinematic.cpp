#include "Kinematic.hpp"

Kinematic::Kinematic(){
  // D-H Parameter Zuweisen
  Kinematic::dhPar1 = { 90.000, 28.691, 0.000, 0.000};
  Kinematic::dhPar2 = {-180.000, 58.000, 0.000, 0.000};
  Kinematic::dhPar3 = { 180.000, 68.300, 0.000, 0.000};
  Kinematic::dhPar4 = {   0.000, 66.539, 0.000, 0.000};
}

float getRad(float deg){
  return deg * (3.14/180);
}

float getDeg(float rad){
  return rad * (180/3.14);
}

void Kinematic::printMatrix(Matrix4x4 mat, String heading){

  log("=============================");
  log(heading);

  for(int i = 0; i <= 3; i++){
    log("Mat" + String(i+1) + "1: " + String(mat.m11[i]) +
        "| Mat" + String(i+1) + "2: " + String(mat.m12[i]) +
        "| Mat" + String(i+1) + "3: " + String(mat.m13[i]) +
        "| Mat" + String(i+1) + "4: " + String(mat.m14[i]));
  }
}

Position Kinematic::fdKinematic(Posture pt){
  Matrix4x4 mat = Kinematic::armTMatrix(pt);

  printMatrix(mat, "Arm Transformation Matrix");

  return {mat.m14[0], mat.m14[1], mat.m14[2]};
}

Posture Kinematic::ivKinematic(Position ps){
  Posture pt; // Roboter Winkel Anordnung
  float pr4; // Vektor länge der Achse 4 in der x-y Ebene
  float pr3; // Vektor länge der Achse 3 in der x-y Ebene
  float pz4; // Z wert Achse 4
  float pz3; // Z wert Achse 3
  float pa24; // // Winkel Summe Asche 2 bis 4 (jt24 = jt2 + jt3 + jt4)

  float n3,n2; // Zähler
  float d3,d2; // Nenner

  // Theta Asche 1 Berechnen
  pt.jt1 = getDeg(atan(getRad(ps.y/ps.x)));

  // Thet a Asche 3 Berechnen;
  pa24 = atan(getRad(ps.y/ps.z));
  float t = atan2(ps.y,ps.z);
  pr4 = sqrt(pow(ps.x,2) + pow(ps.y,2));
  pz4 = ps.z;
  pr3 = pr4 - dhPar4.link*cos(pa24);
  pz3 = pz4 - dhPar4.link*sin(pa24);

  // Berechnung
  n3 = pow(pr3,2) + pow(pz3,2) + (pow(dhPar2.link,2) + pow(dhPar3.link,2));
  d3 = 2 * dhPar2.link * dhPar3.link;
  pt.jt3 = getDeg(acos(round(n3/d3)));

  // Theta Achse 2 Berechnen;
  n2 = pr3*(dhPar2.link + dhPar3.link*cos(getRad(pt.jt3))) + pz3*(dhPar3.link*sin(getRad(pt.jt3)));
  d2 = pow(pr3,2) + pow(pz3,2);
  pt.jt2 = getDeg(acos(round(n2/d2)));

  // Theta Achse 4 Berechnen;
  pt.jt4 = pa24 - (pt.jt2 + pt.jt3);

  // Daten mit Loggen
  log("pa24: " + String(pa24));
  log("pat: " + String(t));
  log("pr4: " + String(pr4));
  log("pz4: " + String(pz4));
  log("pr3: " + String(pr3));
  log("pz3: " + String(pz3));
  log("n2 " + String(n2));
  log("n3: " + String(n3));
  log("d2: " + String(d2));
  log("d3 " + String(d3));

  return pt;
}

Matrix4x4 Kinematic::matrixMulp(Matrix4x4 matA, Matrix4x4 matB){
  Matrix4x4 matAB;

  float Mat[4][4] = {};

  // Matrix m11 bis m41
  matAB.m11[0] = matA.m11[0]*matB.m11[0] + matA.m12[0]*matB.m11[1] + matA.m13[0]*matB.m11[2] + matA.m14[0]*matB.m11[3];
  matAB.m11[1] = matA.m11[1]*matB.m11[0] + matA.m12[1]*matB.m11[1] + matA.m13[1]*matB.m11[2] + matA.m14[1]*matB.m11[3];
  matAB.m11[2] = matA.m11[2]*matB.m11[0] + matA.m12[2]*matB.m11[1] + matA.m13[2]*matB.m11[2] + matA.m14[2]*matB.m11[3];
  matAB.m11[3] = matA.m11[3]*matB.m11[0] + matA.m12[3]*matB.m11[1] + matA.m13[3]*matB.m11[2] + matA.m14[3]*matB.m11[3];
  // Matrix m12 bis m42
  matAB.m12[0] = matA.m11[0]*matB.m12[0] + matA.m12[0]*matB.m12[1] + matA.m13[0]*matB.m12[2] + matA.m14[0]*matB.m12[3];
  matAB.m12[1] = matA.m11[1]*matB.m12[0] + matA.m12[1]*matB.m12[1] + matA.m13[1]*matB.m12[2] + matA.m14[1]*matB.m12[3];
  matAB.m12[2] = matA.m11[2]*matB.m12[0] + matA.m12[2]*matB.m12[1] + matA.m13[2]*matB.m12[2] + matA.m14[2]*matB.m12[3];
  matAB.m12[3] = matA.m11[3]*matB.m12[0] + matA.m12[3]*matB.m12[1] + matA.m13[3]*matB.m12[2] + matA.m14[3]*matB.m12[3];
  // Matrix m13 bis m43
  matAB.m13[0] = matA.m11[0]*matB.m13[0] + matA.m12[0]*matB.m13[1] + matA.m13[0]*matB.m13[2] + matA.m14[0]*matB.m13[3];
  matAB.m13[1] = matA.m11[1]*matB.m13[0] + matA.m12[1]*matB.m13[1] + matA.m13[1]*matB.m13[2] + matA.m14[1]*matB.m13[3];
  matAB.m13[2] = matA.m11[2]*matB.m13[0] + matA.m12[2]*matB.m13[1] + matA.m13[2]*matB.m13[2] + matA.m14[2]*matB.m13[3];
  matAB.m13[3] = matA.m11[3]*matB.m13[0] + matA.m12[3]*matB.m13[1] + matA.m13[3]*matB.m13[2] + matA.m14[3]*matB.m13[3];
  // Matrix m12 bis m42
  matAB.m14[0] = matA.m11[0]*matB.m14[0] + matA.m12[0]*matB.m14[1] + matA.m13[0]*matB.m14[2] + matA.m14[0]*matB.m14[3];
  matAB.m14[1] = matA.m11[1]*matB.m14[0] + matA.m12[1]*matB.m14[1] + matA.m13[1]*matB.m14[2] + matA.m14[1]*matB.m14[3];
  matAB.m14[2] = matA.m11[2]*matB.m14[0] + matA.m12[2]*matB.m14[1] + matA.m13[2]*matB.m14[2] + matA.m14[2]*matB.m14[3];
  matAB.m14[3] = matA.m11[3]*matB.m14[0] + matA.m12[3]*matB.m14[1] + matA.m13[3]*matB.m14[2] + matA.m14[3]*matB.m14[3];

  return matAB;
}

Matrix4x4 Kinematic::jointTMatrix(DhParam dhPar){
  Matrix4x4 mat;
  /*
   * Denavit-Hartenberg-Transformationmatrix (a alpha d theta)
   * T = [
   *      cos(the), -sin(the)*round(cos(alp)), sin(the)*sin(alp),  a*cos(the);
   *      sin(the), cos(the)*round(cos(alp)),  -cos(the)*sin(alp), a*sin(the);
   *      0,        sin(alp),                  round(cos(alp)),    d;
   *      0,        0,                         0,                  1
   *     ];
   */
  // Matrix m11 bis m41
  mat.m11[0] = cos(getRad(dhPar.theta));
  mat.m11[1] = sin(getRad(dhPar.theta));
  mat.m11[2] = 0;
  mat.m11[3] = 0;
  // Matrix m12 bis m42
  mat.m12[0] = -cos(getRad(dhPar.alpha))*sin(getRad(dhPar.theta));
  mat.m12[1] = cos(getRad(dhPar.alpha))*cos(getRad(dhPar.theta));
  mat.m12[2] = sin(getRad(dhPar.alpha));
  mat.m12[3] = 0;
  // Matrix m13 bis m43
  mat.m13[0] = sin(getRad(dhPar.alpha))*sin(getRad(dhPar.theta));
  mat.m13[1] = -sin(getRad(dhPar.alpha))*cos(getRad(dhPar.theta));
  mat.m13[2] = cos(getRad(dhPar.alpha));
  mat.m13[3] = 0;
  // Matrix m14 bis m44
  mat.m14[0] = dhPar.link*cos(getRad(dhPar.theta));
  mat.m14[1] = dhPar.link*sin(getRad(dhPar.theta));
  mat.m14[2] = dhPar.disp;
  mat.m14[3] = 1;

  return mat;
}

Matrix4x4 Kinematic::armTMatrix(Posture p){
  // D-H Parameter Zuweisen
  dhPar1.theta = p.jt1;
  dhPar2.theta = p.jt2;
  dhPar3.theta = p.jt3;
  dhPar4.theta = p.jt4;

  log("===================================================");
  log("D-H Paramter");
  log("dhPar1: " + String(dhPar1.alpha) + " ," + String(dhPar1.link) + ", " + String(dhPar1.disp) + ", " + String(dhPar1.theta));
  log("dhPar2: " + String(dhPar2.alpha) + " ," + String(dhPar2.link) + ", " + String(dhPar2.disp) + ", " + String(dhPar2.theta));
  log("dhPar3: " + String(dhPar3.alpha) + " ," + String(dhPar3.link) + ", " + String(dhPar3.disp) + ", " + String(dhPar3.theta));
  log("dhPar4: " + String(dhPar4.alpha) + " ," + String(dhPar4.link) + ", " + String(dhPar4.disp) + ", " + String(dhPar4.theta));
  log("===================================================");

  // Achsen-Transformations Matrizen Ermittlen
  tMat01 = jointTMatrix(dhPar1);
  tMat12 = jointTMatrix(dhPar2);
  tMat23 = jointTMatrix(dhPar3);
  tMat34 = jointTMatrix(dhPar4);

  // Achsen Transformationen Loggen
  //printMatrix(tMat01, "TMatrix01");
  //printMatrix(tMat12, "TMatrix12");
  //printMatrix(tMat23, "TMatrix23");
  //printMatrix(tMat34, "TMatrix34");

  // Verkettet-Transformations Matrizen Ermittlen
  tMat02 = matrixMulp(tMat01, tMat12);
  tMat03 = matrixMulp(tMat02, tMat23);
  tMat04 = matrixMulp(tMat03, tMat34);

  // Transformationen Loggen
  //printMatrix(tMat02, "TMatrix02");
  //printMatrix(tMat03, "TMatrix03");
  //printMatrix(tMat04, "TMatrix04");

  return tMat04;
}

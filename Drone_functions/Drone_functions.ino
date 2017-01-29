#include <Wire.h>
//#include <Servo.h>
#include <ServoTimer2.h>
#include <PID_v1.h>
#include <PinChangeInt.h>
#include "Drone_variables2.h"

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  Wire.begin();
  IMU_setup();
  ESC1.attach(ESC1pin);
  ESC2.attach(ESC2pin);
  ESC3.attach(ESC3pin);
  ESC4.attach(ESC4pin);
}

void loop() {
  delay(10);
//  IMU_values();
  Serial_read();
  ESC_write();
  Serial.print(pitch);
  Serial.print('\t');
  Serial.print(roll);
  Serial.print('\t');
  Serial.print(dtime, 8);
  Serial.print('\t');
  Serial.println(ESC1_val);
}

void ESC_write(){

  ESC1_val = value;
  ESC2_val = value;
  ESC3_val = value;
  ESC4_val = value;
  
  ESC1_val = constrain(ESC1_val,1000,2000);
  ESC2_val = constrain(ESC2_val,1000,2000);
  ESC3_val = constrain(ESC3_val,1000,2000);
  ESC4_val = constrain(ESC4_val,1000,2000);

  ESC1.write(ESC1_val);
//  ESC2.write(ESC2_val);
//  ESC3.write(ESC3_val);
//  ESC4.write(ESC4_val);

//  ESC1.write(1000);
  ESC2.write(1000);
  ESC3.write(1000);
  ESC4.write(1000);

//  ESC1.writeMicroseconds(ESC1_val);
////  ESC2.writeMicroseconds(ESC2_val);
////  ESC3.writeMicroseconds(ESC3_val);
////  ESC4.writeMicroseconds(ESC4_val);
//
////  ESC1.writeMicroseconds(1000);
//  ESC2.writeMicroseconds(1000);
//  ESC3.writeMicroseconds(1000);
//  ESC4.writeMicroseconds(1000);
}

void Serial_read(){
      if (Serial.available() > 0) {  
    incomingByte = Serial.readStringUntil(',');     
    spacelocation = incomingByte.indexOf(' ');
    command = incomingByte.substring(0,spacelocation);
    value = incomingByte.substring(spacelocation+1).toFloat();
    if ((value>2000 || value<1000) && command != "p"){
      value = 1000;
    }
  }
}

void IMU_values(){
  ////////////////GETTING IMU VALUES/////////////
  
  //////////////////////////////////////////////////////////////////////
  //gyro_x CALCULATION//////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////
  Wire.beginTransmission(gyro_add);
  Wire.write(gyro_xaddL);
  Wire.endTransmission();

  Wire.requestFrom(gyro_add, 1);
  while (Wire.available() == 0);
  gyro_xL = Wire.read();

  Wire.beginTransmission(gyro_add);
  Wire.write(gyro_xaddH);
  Wire.endTransmission();

  Wire.requestFrom(gyro_add, 1);
  while (Wire.available() == 0);
  gyro_xH = Wire.read();

  gyro_x = gyro_xH * 256 + gyro_xL;
  gyro_valx = gyro_x * 2000.000 / 32764.000;
  //Serial.println(gyro_valx);

  //////////////////////////////////////////////////////////////////
  //gyro_y calculation//////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////
  Wire.beginTransmission(gyro_add);
  Wire.write(gyro_yaddL);
  Wire.endTransmission();

  Wire.requestFrom(gyro_add, 1);
  while (Wire.available() == 0);
  gyro_yL = Wire.read();

  Wire.beginTransmission(gyro_add);
  Wire.write(gyro_yaddH);
  Wire.endTransmission();

  Wire.requestFrom(gyro_add, 1);
  while (Wire.available() == 0);
  gyro_yH = Wire.read();

  gyro_y = gyro_yH * 256 + gyro_yL;
  gyro_valy = gyro_y * 2000.000 / 32764.000;
  //Serial.println(gyro_valy);


  //////////////////////////////////////////////////////////////////
  //gyro_z calculation//////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////
  Wire.beginTransmission(gyro_add);
  Wire.write(gyro_zaddL);
  Wire.endTransmission();

  Wire.requestFrom(gyro_add, 1);
  while (Wire.available() == 0);
  gyro_zL = Wire.read();

  Wire.beginTransmission(gyro_add);
  Wire.write(gyro_zaddH);
  Wire.endTransmission();

  Wire.requestFrom(gyro_add, 1);
  while (Wire.available() == 0);
  gyro_zH = Wire.read();

  gyro_z = gyro_zH * 256 + gyro_zL;
  gyro_valz = gyro_z * 2000.000 / 32764.000;
  //Serial.println(gyro_valz);

  //////////////////////////////////////////////////////////////////////
  //accel_x CALCULATION//////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////
  Wire.beginTransmission(accel_add);
  Wire.write(accel_xaddL);
  Wire.endTransmission();

  Wire.requestFrom(accel_add, 1);
  while (Wire.available() == 0);
  accel_xL = Wire.read();

  Wire.beginTransmission(accel_add);
  Wire.write(accel_xaddH);
  Wire.endTransmission();

  Wire.requestFrom(accel_add, 1);
  while (Wire.available() == 0);
  accel_xH = Wire.read();

  accel_x = accel_xH * 256 + accel_xL;
  accel_valx = accel_x * 2.000 / 32768.000;
  //Serial.print(accel_valx);

  //////////////////////////////////////////////////////////////////
  //accel_y calculation//////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////
  Wire.beginTransmission(accel_add);
  Wire.write(accel_yaddL);
  Wire.endTransmission();

  Wire.requestFrom(accel_add, 1);
  while (Wire.available() == 0);
  accel_yL = Wire.read();

  Wire.beginTransmission(accel_add);
  Wire.write(accel_yaddH);
  Wire.endTransmission();

  Wire.requestFrom(accel_add, 1);
  while (Wire.available() == 0);
  accel_yH = Wire.read();

  accel_y = accel_yH * 256 + accel_yL;
  accel_valy = accel_y * 2.000 / 32764.000;
//  Serial.println(accel_valy);

  //////////////////////////////////////////////////////////////////
  //accel_z calculation//////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////
  Wire.beginTransmission(accel_add);
  Wire.write(accel_zaddL);
  Wire.endTransmission();

  Wire.requestFrom(accel_add, 1);
  while (Wire.available() == 0);
  accel_zL = Wire.read();

  Wire.beginTransmission(accel_add);
  Wire.write(accel_zaddH);
  Wire.endTransmission();

  Wire.requestFrom(accel_add, 1);
  while (Wire.available() == 0);
  accel_zH = Wire.read();

  accel_z = accel_zH * 256 + accel_zL;
  accel_valz = accel_z * 2.000 / 32764.000;

  //////////////////////////////////////////////////////////////////////
  //comp_x CALCULATION//////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////
  Wire.beginTransmission(comp_add);
  Wire.write(comp_xaddL);
  Wire.endTransmission();

  Wire.requestFrom(comp_add, 1);
  while (Wire.available() == 0);
  comp_xL = Wire.read();

  Wire.beginTransmission(comp_add);
  Wire.write(comp_xaddH);
  Wire.endTransmission();

  Wire.requestFrom(comp_add, 1);
  while (Wire.available() == 0);
  comp_xH = Wire.read();

  comp_x = comp_xH * 256 + comp_xL;

  //////////////////////////////////////////////////////////////////////
  //comp_y CALCULATION//////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////
  Wire.beginTransmission(comp_add);
  Wire.write(comp_yaddL);
  Wire.endTransmission();

  Wire.requestFrom(comp_add, 1);
  while (Wire.available() == 0);
  comp_yL = Wire.read();

  Wire.beginTransmission(comp_add);
  Wire.write(comp_yaddH);
  Wire.endTransmission();

  Wire.requestFrom(comp_add, 1);
  while (Wire.available() == 0);
  comp_yH = Wire.read();

  comp_y = comp_yH * 256 + comp_yL;

  //////////////////////////////////////////////////////////////////////
  //comp_z CALCULATION//////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////
  Wire.beginTransmission(comp_add);
  Wire.write(comp_zaddL);
  Wire.endTransmission();

  Wire.requestFrom(comp_add, 1);
  while (Wire.available() == 0);
  comp_zL = Wire.read();

  Wire.beginTransmission(comp_add);
  Wire.write(comp_zaddH);
  Wire.endTransmission();

  Wire.requestFrom(comp_add, 1);
  while (Wire.available() == 0);
  comp_zH = Wire.read();

  comp_z = comp_zH * 256 + comp_zL;

  /////////////COMPLEMENTARY FILTER/////////////

  T[0] = T[1];
  T[1] = micros();
  dtime = (T[1] - T[0]) / 1000000.000;

  accel_totforce = sqrt(accel_valx * accel_valx + accel_valy * accel_valy + accel_valz * accel_valz);
  accel_Rx = accel_valx/accel_totforce;
  accel_Ry = accel_valy/accel_totforce;
  accel_Rz = accel_valz/accel_totforce;

  Ax_accel = atan2(accel_Ry,accel_Rz);
  Ay_accel = atan2(accel_Rx,accel_Rz);

  Ax_gyro = Ax + gyro_valx*dtime*PI/180.000;
  Ay_gyro = Ay - gyro_valy*dtime*PI/180.000;
  
  Ax = 0.980*Ax_gyro + 0.020*Ax_accel;
  Ay = 0.980*Ay_gyro + 0.020*Ay_accel;
  
  roll = Ax*180.0000/PI;
  pitch = -Ay*180.0000/PI;
  }
  
void IMU_setup(){
  
  Wire.beginTransmission(gyro_add);   
  Wire.write(gyro_start_add);
  Wire.write(15);                 //turns gyro to normal mode, default is power down mode
  Wire.endTransmission();

  Wire.beginTransmission(gyro_add);
  Wire.write(gyro_start_add2);
  Wire.write(32);                 //turns gyro range to 2000 dps
  Wire.endTransmission();

  Wire.beginTransmission(accel_add);
  Wire.write(accel_start_add);
  Wire.write(151);                //turns accelerometer axis on and sets ODR
  Wire.endTransmission();

  Wire.beginTransmission(accel_add);
  Wire.write(accel_start_add2);
  Wire.write(0);                 //sets accelerometer range to +-2g
  Wire.endTransmission();

  Wire.beginTransmission(comp_add);
  Wire.write(comp_start_add);
  Wire.write(32);                 //sets compass range to +-1.3 gauss
  Wire.endTransmission();
  
  Wire.beginTransmission(comp_add);
  Wire.write(comp_start_add2);
  Wire.write(0);                 //sets compass to continuous conversion mode
  Wire.endTransmission();  

  Wire.beginTransmission(comp_add);
  Wire.write(comp_start_add3);
  Wire.write(24);                 
  Wire.endTransmission();    

  ////////////////////////////////////////////////////////////////barometer
  int AC1_add1 = 0xAA;
  int AC1_add2 = 0xAB;
  Wire.beginTransmission(alt_add);
  Wire.write(AC1_add1);
  Wire.endTransmission();

  Wire.requestFrom(alt_add, 1);
  while (Wire.available() == 0);
  int AC1A = Wire.read();

  Wire.beginTransmission(alt_add);
  Wire.write(AC1_add2);
  Wire.endTransmission();

  Wire.requestFrom(alt_add, 1);
  while (Wire.available() == 0);
  int AC1B = Wire.read();
  AC1 = AC1A*256+AC1B;
 
  //////////////////////////////////////////////////////////////
  int AC2_add1 = 0xAC;
  int AC2_add2 = 0xAD;
  Wire.beginTransmission(alt_add);
  Wire.write(AC2_add1);
  Wire.endTransmission();

  Wire.requestFrom(alt_add, 1);
  while (Wire.available() == 0);
  int AC2A = Wire.read();

  Wire.beginTransmission(alt_add);
  Wire.write(AC2_add2);
  Wire.endTransmission();

  Wire.requestFrom(alt_add, 1);
  while (Wire.available() == 0);
  int AC2B = Wire.read();
  AC2 = AC2A*256+AC2B;

  //////////////////////////////////////////////////////////////
  int AC3_add1 = 0xAE;
  int AC3_add2 = 0xAF;
  Wire.beginTransmission(alt_add);
  Wire.write(AC3_add1);
  Wire.endTransmission();

  Wire.requestFrom(alt_add, 1);
  while (Wire.available() == 0);
  int AC3A = Wire.read();

  Wire.beginTransmission(alt_add);
  Wire.write(AC3_add2);
  Wire.endTransmission();

  Wire.requestFrom(alt_add, 1);
  while (Wire.available() == 0);
  int AC3B = Wire.read();
  AC3 = AC3A*256+AC3B;


  //////////////////////////////////////////////////////////////
  int AC4_add1 = 0xB0;
  int AC4_add2 = 0xB1;
  Wire.beginTransmission(alt_add);
  Wire.write(AC4_add1);
  Wire.endTransmission();

  Wire.requestFrom(alt_add, 1);
  while (Wire.available() == 0);
  int AC4A = Wire.read();

  Wire.beginTransmission(alt_add);
  Wire.write(AC4_add2);
  Wire.endTransmission();

  Wire.requestFrom(alt_add, 1);
  while (Wire.available() == 0);
  int AC4B = Wire.read();
  AC4 = AC4A*256+AC4B;

  
  //////////////////////////////////////////////////////////////
  int AC5_add1 = 0xB2;
  int AC5_add2 = 0xB3;
  Wire.beginTransmission(alt_add);
  Wire.write(AC5_add1);
  Wire.endTransmission();

  Wire.requestFrom(alt_add, 1);
  while (Wire.available() == 0);
  int AC5A = Wire.read();

  Wire.beginTransmission(alt_add);
  Wire.write(AC5_add2);
  Wire.endTransmission();

  Wire.requestFrom(alt_add, 1);
  while (Wire.available() == 0);
  int AC5B = Wire.read();
  AC5 = AC5A*256+AC5B;
  

  //////////////////////////////////////////////////////////////
  int AC6_add1 = 0xB4;
  int AC6_add2 = 0xB5;
  Wire.beginTransmission(alt_add);
  Wire.write(AC6_add1);
  Wire.endTransmission();

  Wire.requestFrom(alt_add, 1);
  while (Wire.available() == 0);
  int AC6A = Wire.read();

  Wire.beginTransmission(alt_add);
  Wire.write(AC6_add2);
  Wire.endTransmission();

  Wire.requestFrom(alt_add, 1);
  while (Wire.available() == 0);
  int AC6B = Wire.read();
  AC6 = AC6A*256+AC6B;
   

  //////////////////////////////////////////////////////////////
  int BB1_add1 = 0xB6;
  int BB1_add2 = 0xB7;
  Wire.beginTransmission(alt_add);
  Wire.write(BB1_add1);
  Wire.endTransmission();

  Wire.requestFrom(alt_add, 1);
  while (Wire.available() == 0);
  int BB1A = Wire.read();

  Wire.beginTransmission(alt_add);
  Wire.write(BB1_add2);
  Wire.endTransmission();

  Wire.requestFrom(alt_add, 1);
  while (Wire.available() == 0);
  int BB1B = Wire.read();
  BB1 = BB1A*256+BB1B;

  //////////////////////////////////////////////////////////////
  int BB2_add1 = 0xB8;
  int BB2_add2 = 0xB9;
  Wire.beginTransmission(alt_add);
  Wire.write(BB2_add1);
  Wire.endTransmission();

  Wire.requestFrom(alt_add, 1);
  while (Wire.available() == 0);
  int BB2A = Wire.read();

  Wire.beginTransmission(alt_add);
  Wire.write(BB2_add2);
  Wire.endTransmission();

  Wire.requestFrom(alt_add, 1);
  while (Wire.available() == 0);
  int BB2B = Wire.read();
  BB2 = BB2A*256+BB2B;
  
  //////////////////////////////////////////////////////////////
  int MB_add1 = 0xBA;
  int MB_add2 = 0xBB;
  Wire.beginTransmission(alt_add);
  Wire.write(MB_add1);
  Wire.endTransmission();

  Wire.requestFrom(alt_add, 1);
  while (Wire.available() == 0);
  int MBA = Wire.read();

  Wire.beginTransmission(alt_add);
  Wire.write(MB_add2);
  Wire.endTransmission();

  Wire.requestFrom(alt_add, 1);
  while (Wire.available() == 0);
  int MBB = Wire.read();
  MB = MBA*256+MBB;

  //////////////////////////////////////////////////////////////
  int MC_add1 = 0xBC;
  int MC_add2 = 0xBD;
  Wire.beginTransmission(alt_add);
  Wire.write(MC_add1);
  Wire.endTransmission();

  Wire.requestFrom(alt_add, 1);
  while (Wire.available() == 0);
  int MCA = Wire.read();

  Wire.beginTransmission(alt_add);
  Wire.write(MC_add2);
  Wire.endTransmission();

  Wire.requestFrom(alt_add, 1);
  while (Wire.available() == 0);
  int MCB = Wire.read();
  MC = MCA*256+MCB;

  //////////////////////////////////////////////////////////////
  int MD_add1 = 0xBE;
  int MD_add2 = 0xBF;
  Wire.beginTransmission(alt_add);
  Wire.write(MD_add1);
  Wire.endTransmission();

  Wire.requestFrom(alt_add, 1);
  while (Wire.available() == 0);
  int MDA = Wire.read();

  Wire.beginTransmission(alt_add);
  Wire.write(MD_add2);
  Wire.endTransmission();

  Wire.requestFrom(alt_add, 1);
  while (Wire.available() == 0);
  int MDB = Wire.read();
  MD = MDA*256+MDB;  
}

void ricoprint(String Rico1, String Rico2, String Rico3, String Rico4, String Rico5, String Rico6){
  Serial.print(Rico1);
  Serial.print('\t');
  Serial.print(Rico2);
  Serial.print('\t');
  Serial.print(Rico3);
  Serial.print('\t');
  Serial.print(Rico4);
  Serial.print('\t');
  Serial.print(Rico5);
  Serial.print('\t');
  Serial.print(Rico6);
  Serial.print('\n');
}


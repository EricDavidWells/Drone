#include <Wire.h>
#include <ServoTimer2.h>
#include <PID_v1.h>
#include <PinChangeInt.h>
#include "Drone_variables2.h"

double PIDpitchout;
double PIDpitchset=0;
double PIDpitchkp=1;
double PIDpitchki=0;
double PIDpitchkd=0;
PID PIDpitch(&pitch,&PIDpitchout,&PIDpitchset,PIDpitchkp,PIDpitchki,PIDpitchkd,DIRECT);

double PIDrollout;
double PIDrollset=0;
PID PIDroll(&roll,&PIDrollout,&PIDrollset,PIDpitchkp,PIDpitchki,PIDpitchkd,DIRECT);

double PIDyawout;
double PIDyawset;
double PIDyawkp=0.5;
double PIDyawki=0;
double PIDyawkd=0;
double PIDgyro_valz;
PID PIDyaw(&PIDgyro_valz,&PIDyawout,&PIDyawset,PIDyawkp,PIDyawki,PIDyawkd,DIRECT);

int rec_pitchgain = 15;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2);
  Wire.begin();
  IMU_setup();
  pinMode(ESC1pin, OUTPUT);
  pinMode(ESC2pin, OUTPUT);
  pinMode(ESC3pin, OUTPUT);
  pinMode(ESC4pin, OUTPUT);
  pinMode(ch1, INPUT); digitalWrite(ch1, HIGH);
  pinMode(ch2, INPUT); digitalWrite(ch2, HIGH);
  pinMode(ch3, INPUT); digitalWrite(ch3, HIGH);
  pinMode(ch4, INPUT); digitalWrite(ch4, HIGH);

  PCintPort::attachInterrupt(ch1, &rising, RISING);
  Calibrate();

  PIDpitch.SetOutputLimits(-100,100);
  PIDpitch.SetMode(AUTOMATIC);
  PIDpitch.SetSampleTime(20);
  PIDroll.SetOutputLimits(-100,100);
  PIDroll.SetMode(AUTOMATIC);
  PIDroll.SetSampleTime(20);
  PIDyaw.SetOutputLimits(-100,100);
  PIDyaw.SetMode(AUTOMATIC);
  PIDyaw.SetSampleTime(20);
}

void loop() {
  delay(10); 
  Serial_read();
  IMU_values();
  ESC_write();
//  ESC_chill();
//  Receiver_filter();              //this is called within ESC_write function for speed
  Reset_detect();
  PID_calc();

//  Serial.print(" accel_x:  ");
//  Serial.print(accel_valx);
//  Serial.print(" accel_y:  ");
//  Serial.print(accel_valy);
//  Serial.print(" accel_z:  ");
//  Serial.print(accel_valz);
  Serial.print(" value:  ");
  Serial.print(value);
  Serial.print(" pitch:  ");
  Serial.print(pitch);
  Serial.print(" roll: ");
  Serial.print(roll);
  Serial.print(" PIDroll ");
  Serial.print(PIDrollout);
  Serial.print(" PIDpitch: ");
  Serial.print(PIDpitchout);
  Serial.print(" ch1: ");
  Serial.print(pwm_corr[0]);
  Serial.print(" ch2: ");
  Serial.print(pwm_corr[1]);
  Serial.print(" ch3: ");
  Serial.print(pwm_corr[2]);
  Serial.print(" ch4: ");
  Serial.print(pwm_corr[3]);
  Serial.print('\t');
  Serial.println();
  
}

void PID_calc(){
//  PIDpitchset = (pwm_value[1]-(rec_cal[1]-400.000))*(rec_pitchgain*2)/(800.000)-rec_pitchgain;
//  PIDrollset = (pwm_value[0]-(rec_cal[0]-400.000))*(rec_pitchgain*2)/(800.000)-rec_pitchgain;

  PIDroll.Compute();
  PIDpitch.Compute();
}

void rising(){
//  latest_interrupted_pin=PCintPort::arduinoPin;
//  PCintPort::detachInterrupt(latest_interrupted_pin);
//  PCintPort::attachInterrupt(latest_interrupted_pin, &falling, FALLING);
//  prev_time = micros();

  latest_interrupted_pin=PCintPort::arduinoPin;
  PCintPort::detachInterrupt(pins[i]);
  PCintPort::attachInterrupt(pins[i], &falling, FALLING);
  prev_time = micros();

}
 
void falling(){
//  latest_interrupted_pin=PCintPort::arduinoPin;
//  PCintPort::detachInterrupt(latest_interrupted_pin);
//  rec_speed[i] = pwm_value[i]-(micros()-prev_time);
//  pwm_value[i] = micros()-prev_time;
//  i = i+1;
//  i = i % pinlength;
//  PCintPort::attachInterrupt(pins[i], &rising, RISING);

//  latest_interrupted_pin=PCintPort::arduinoPin;
  
  rec_speed[i] = pwm_value[i]-(micros()-prev_time);
  pwm_value[i] = micros()-prev_time;
  PCintPort::detachInterrupt(pins[i]);
  i = i+1;
  i = i % pinlength;
  PCintPort::attachInterrupt(pins[i], &rising, RISING);
}

void ESC_write(){

  PCintPort::detachInterrupt(pins[i]);
  
  PORTB |= B00001100;        //turn on pins 10 and 11
  PORTD |= B00101000;        //turn on pins 3 and 5
  loop_timer = micros();     //start timer

  timer_ESC1 = loop_timer + value;   //say at what time the channel needs to shut off
  timer_ESC2 = loop_timer + value;
  timer_ESC3 = loop_timer + value;
  timer_ESC4 = loop_timer + value;

//  timer_ESC1 = loop_timer + value + PIDpitchout - PIDrollout;   //say at what time the channel needs to shut off
//  timer_ESC2 = loop_timer + value + PIDpitchout + PIDrollout;
//  timer_ESC3 = loop_timer + value - PIDpitchout - PIDrollout;
//  timer_ESC4 = loop_timer + value - PIDpitchout + PIDrollout;

//  timer_ESC1 = loop_timer + pwm_corr[2] + PIDpitchout - PIDrollout;   //say at what time the channel needs to shut off
//  timer_ESC2 = loop_timer + pwm_corr[2] + PIDpitchout + PIDrollout;
//  timer_ESC3 = loop_timer + pwm_corr[2] - PIDpitchout - PIDrollout;
//  timer_ESC4 = loop_timer + pwm_corr[2] - PIDpitchout + PIDrollout;

  Receiver_filter();

  while((PORTB - 3) >= 4 || (PORTD-192) >= 8){
    esc_timer = micros();

    if(timer_ESC1 <= esc_timer){PORTB &= B11111011;}                //Set digital output 10 to low if the time is expired.
    if(timer_ESC2 <= esc_timer){PORTD &= B11110111;}                //Set digital output 11 to low if the time is expired.
    if(timer_ESC3 <= esc_timer){PORTB &= B11110111;}                //Set digital output 3 to low if the time is expired.
    if(timer_ESC4 <= esc_timer){PORTD &= B11011111;}                //Set digital output 5 to low if the time is expired

//    if((loop_timer + 1000) <= esc_timer){PORTB &= B11111011;}
//    if((loop_timer + 1000) <= esc_timer){PORTD &= B11110111;}               //for only writing to one motor 
//    if((loop_timer + 1000) <= esc_timer){PORTB &= B11110111;}              
//    if((loop_timer + 1000) <= esc_timer){PORTD &= B11011111;} 

  }

  PCintPort::attachInterrupt(pins[i], &rising, RISING);
}

void Serial_read(){
      if (Serial.available() > 0) {  
    incomingByte = Serial.readStringUntil(',');     
    spacelocation = incomingByte.indexOf(' ');
    command = incomingByte.substring(0,spacelocation);
    value = incomingByte.substring(spacelocation+1).toFloat();
    if ((value>2000 || value<1000)){
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
  gyro_valx = gyro_x * 250.000 / 32764.000;
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
  gyro_valy = gyro_y * 250.000 / 32764.000;
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
  gyro_valz = gyro_z * 250.000 / 32764.000;
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
  
  Ax = 0.990*Ax_gyro + 0.010*Ax_accel;
  Ay = 0.990*Ay_gyro + 0.010*Ay_accel;
  
  roll = Ax*180.0000/PI;
  pitch = -Ay*180.0000/PI;
  }

void Reset_detect(){
//reset the IMU if a nan error starts happenning
if ( pitch > 500 || pitch < -500 || roll > 500 || roll < -500 ){
  Serial.println("error");
  resetcount += 1;
}
else{
  resetcount = 0;
}

if (resetcount >= 3){
  IMU_setup();
}
}
  
void IMU_setup(){
  
  Wire.beginTransmission(gyro_add);   
  Wire.write(gyro_start_add);
  Wire.write(15);                 //turns gyro to normal mode, default is power down mode
  Wire.endTransmission();

  Wire.beginTransmission(gyro_add);
  Wire.write(gyro_start_add2);
  Wire.write(0);                 //32 sets gyro range to +-500dps; 0 sets gyro range to +-250dps; 
  Wire.endTransmission();

  Wire.beginTransmission(accel_add);
  Wire.write(accel_start_add);
  Wire.write(151);                //turns accelerometer axis on and sets ODR
  Wire.endTransmission();

  Wire.beginTransmission(accel_add);
  Wire.write(accel_start_add2);
  Wire.write(0);                 //32 sets accelerometer range to +-8g; 8 sets to +-4g; 0 sets to +-2g;
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

void Calibrate(){
  Serial.println("Calibrating");
  delay(1000);            //delay to allow receiver to start functioning properly
  
  cal_timerstart = micros();      //locate timer starting position for reference
  long r = 0;                          //initiate random local variable to count
  long m = 0;                          //initiate random local variable to count modulo style to pulse ESC's every once in a while so they don't freak out while calibrating
  
  
   while((cal_timer - cal_timerstart) < 3000000){   //record pwm values for 5 seconds
      for (int j = 0; j<4; j++){
        rec_cal[j] += pwm_value[j];
      }     
      r += 1;         //increment how many values are recorded
      m += 1;         //increment how many cycles since last update
      cal_timer = micros();   //update timer
      if (m % 1000 == 0){      //if weve done 1000 cycles pulse ESC
        PORTB |= B00001100;        //turn on pins 10 and 11
        PORTD |= B00101000;        //turn on pins 3 and 5
        delayMicroseconds(1000);
        PORTB &= B11110011;
        PORTD &= B11010111;
        m = 0;
      } 

      IMU_values();
      gyro_cal[0] += gyro_valx;
      gyro_cal[1] += gyro_valy;
      gyro_cal[2] += gyro_valz;
   }
   
      for (int j = 0; j<4; j++){    //get average receiver value for each channel
        rec_cal[j] /= r;
      }

      gyro_cal[0] /= r;
      gyro_cal[1] /= r;
      gyro_cal[2] /= r;
}

void Receiver_filter(){
        //Sends all receiver values to between 1000 and 2000, except limits throttle to 1500 to make more sensitive
        pwm_corr[0] = (pwm_value[0]-(rec_cal[0]-450))*(1000)/(900) + 1000;
        pwm_corr[0] = constrain(pwm_corr[0],1000,2000);
        pwm_corr[1] = (pwm_value[1]-(rec_cal[1]-450))*(1000)/(900) + 1000;
        pwm_corr[1] = constrain(pwm_corr[1],1000,2000);
        pwm_corr[2] = (pwm_value[2]-rec_cal[2])*(400)/(900) + 1000;
        pwm_corr[2] = constrain(pwm_corr[2],1000,1500);
        pwm_corr[3] = (pwm_value[3]-(rec_cal[3]-450))*(1000)/(900) + 1000;
        pwm_corr[3] = constrain(pwm_corr[3],1000,2000);
       
}

void ESC_chill(){
//pulse ESC to 1000 to stop annoying noise while checking other things

  PCintPort::detachInterrupt(pins[i]);
  
  PORTB |= B00001100;        //turn on pins 10 and 11
  PORTD |= B00101000;        //turn on pins 3 and 5
  loop_timer = micros();     //start timer

  timer_ESC1 = loop_timer + 1000;   //say at what time the channel needs to shut off
  timer_ESC2 = loop_timer + 1000;
  timer_ESC3 = loop_timer + 1000;
  timer_ESC4 = loop_timer + 1000;

  Receiver_filter();

  while((PORTB - 3) >= 4 || (PORTD-192) >= 8){
    esc_timer = micros();          //update timer
    if(timer_ESC1 <= esc_timer)PORTB &= B11111011;                //Set digital output 10 to low if the time is expired.
    if(timer_ESC2 <= esc_timer)PORTD &= B11110111;                //Set digital output 11 to low if the time is expired.
    if(timer_ESC3 <= esc_timer)PORTB &= B11110111;                //Set digital output 3 to low if the time is expired.
    if(timer_ESC4 <= esc_timer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired
  }

   PCintPort::attachInterrupt(pins[i], &rising, RISING);
}


#include <SoftwareSerial.h>
#include<Wire.h>
#include "TimerOne.h"

int state = -9;

const int MPU_addr=0x68; int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int minVal=265; 
int maxVal=402;

unsigned long prevTime, loopTime;
int receiveLoopTime = 0;

float coef = 0.9 * 0.545;

float Kp = 5.5; //7
float Kd = 13; //9.7 9.5
float Ki = 0.95; //1.3

float a;

float gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

int dir1 = 2;
int stp1 = 3;
int dir2 = 4;
int stp2 = 5;

int dirRobot = 0;
int rotRobot = 1; //positive counterclockwise

float pos = 0;
float currPos = 0;
float Lpos = 0;
float LcurrPos = 0;
float Rpos = 0;
float RcurrPos = 0;

float angle;
float accAngle;
float currentAngle;
float angleOffset = 3.00; //higher if leans forward
float targetAngle = 0;
float variableOffset = 0;
float accelerantOffset = 0;
float acc = 0.0003; //0.0003
float velOffset = 0;
float maxVelOffset = 0.25;
float maxRotVelOffset = 2;
float rotVelOffset = 0;

float error;
float dAngle;
float iError = 0;

float vel;
float Lvel;
float Rvel;

void setup() {
  pinMode(dir1, OUTPUT);
  pinMode(stp1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(stp2, OUTPUT);
  digitalWrite(dir1, HIGH);
  digitalWrite(dir2, LOW);
  Wire.begin(); 
  Wire.beginTransmission(MPU_addr); 
  Wire.write(0x6B); 
  Wire.write(0); 
  Wire.endTransmission(true); 
  Serial.begin(9600); 
  record();
  currentAngle = accAngle;
  Timer1.initialize(1500);
  Timer1.attachInterrupt(velocityLoop);
}

void loop() {
  unsigned long currTime = millis();
  loopTime = currTime - prevTime;
  if (loopTime >= 10) {
    prevTime = currTime;
    record();
    PID();
    receiveLoopTime++;
  }
  if (receiveLoopTime >= 30) {
    if(Serial.available() > 0){ // Checks whether data is comming from the serial port
      state = Serial.read(); // Reads the data from the serial port
    }
    if (state == 51) { //49 left 50 right 51 up 52 down
      dirRobot = -1;
      rotRobot = 0;
//      rotVelOffset = 0;
//      state = 0;
    }
    else if (state == 52) {
      dirRobot = 1;
      rotRobot = 0;
//      rotVelOffset = 0;
//      state = 0;
    } 
    else if (state == 49) {
      rotRobot = 1;
      dirRobot = 0;
    }
    else if ( state == 50) {
      rotRobot = -1;
      dirRobot = 0;
    }
    else {
      dirRobot = 0;
      rotRobot = 0;
//      rotVelOffset = 0;
    }
    receiveLoopTime -= 30;
  }
}

void PID() {
  vel = coef * (Kp * error - Kd * dAngle + Ki * iError);
  if (vel > 12) {
    variableOffset = 0.7;
  }
  else if (vel < -12) {
    variableOffset = -0.7;
  }
  else {
    variableOffset = 0;
  }
//  if (dirRobot = 1 && vel > -12) {
//    accelerantOffset = 0.3;
//  }
//  else if (dirRobot = -1 && vel < 12) {
//    accelerantOffset = -0.3;
//  }
//  else {
//    accelerantOffset = 0;
//  }
}

void velocityLoop() {
  pos += vel / 100.0 + velOffset;
  Rpos += Rvel / 100.0 + velOffset;
  Lpos += Lvel / 100.0 + velOffset;
  if (vel > 100) {
    vel = 100;
  }
  else if (vel < -100) {
    vel = -100;
  }
  if (velOffset > maxVelOffset) { //makes sure velocity doesn't exceed max velocity
    velOffset = maxVelOffset;
  }
  else if (velOffset < -1 * maxVelOffset) {
    velOffset = -1 * maxVelOffset;
  }
  if (angle > 50 || angle < -50) {
    vel = 0;
  }
//  if (rotRobot != 0) {
//    rotVelOffset += rotRobot * 0.001;
//  }
//  else if (rotVelOffset != 0) {
//    rotVelOffset -= rotVelOffset * 0.001 / maxRotVelOffset;
//    if (abs(rotVelOffset) <= abs(rotVelOffset * 0.001 / maxRotVelOffset)) {
//      rotVelOffset = 0;
//    }
//  }
//  if (rotVelOffset > maxRotVelOffset) { //makes sure rotational velocity doesn't exceed maximum
//    rotVelOffset = maxRotVelOffset;
//  }
//  else if (rotVelOffset < -1 * maxRotVelOffset) {
//    rotVelOffset = -1 * maxRotVelOffset;
//  }
  Lvel = vel - rotRobot * 2;
  Rvel = vel + rotRobot * 2;
  if (rotRobot == 0) {
    if (currPos + 1 <= pos) {
      currPos += 1;
      digitalWrite(dir1, HIGH);
      digitalWrite(dir2, LOW);
      Step();
    }
    else if (currPos - 1 >= pos) {
      currPos -= 1;
      digitalWrite(dir1, LOW);
      digitalWrite(dir2, HIGH);
      Step();
    }
  }
  else  {
    if (RcurrPos + 1 <= Rpos) {
      RcurrPos += 1;
      digitalWrite(dir1, HIGH);
      digitalWrite(stp1, HIGH);
      digitalWrite(stp1, LOW);
    }
    else if (RcurrPos - 1 >= Rpos) {
      RcurrPos -= 1;
      digitalWrite(dir1, LOW);
      digitalWrite(stp1, HIGH);
      digitalWrite(stp1, LOW);
    }
    if (LcurrPos + 1 <= Lpos) {
      LcurrPos += 1;
      digitalWrite(dir2, LOW);
      digitalWrite(stp2, HIGH);
      digitalWrite(stp2, LOW);
    }
    else if (LcurrPos - 1 >= Lpos) {
      LcurrPos -= 1;
      digitalWrite(dir2, HIGH);
      digitalWrite(stp2, HIGH);
      digitalWrite(stp2, LOW);
    }
  }
  accelerantOffset = dirRobot * 0.7;
}

void Step() {
  digitalWrite(stp1, HIGH);
  digitalWrite(stp2, HIGH);
  digitalWrite(stp1, LOW);
  digitalWrite(stp2, LOW);
}

void record() {
  recordGyroRegisters();
  getAngle();
  updateAngle();
}

void updateAngle() {
  float prevAngle = angle;
  a = 0.75/(0.75 + loopTime * 0.001);
  angle = a * (currentAngle - rotY * loopTime * 0.065) + (1 - a) * accAngle;
//  gyroSum += gyroY;
//  count++;
//  Serial.println("angle: " + (String) angle + " Acc angle: " + (String) accAngle);
  error = angle + variableOffset + accelerantOffset - targetAngle;
  iError += error * loopTime * 0.065;
  dAngle = (prevAngle - angle) / (0.065 * loopTime);
  currentAngle = angle;
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read() - 22; //Store middle two bytes into accelY
  processGyroData();
}

void getAngle() {
  Wire.beginTransmission(MPU_addr); 
  Wire.write(0x3B); 
  Wire.endTransmission(false); 
  Wire.requestFrom(MPU_addr,14,true); 
  AcX=Wire.read()<<8|Wire.read(); 
  AcY=Wire.read()<<8|Wire.read(); 
  AcZ=Wire.read()<<8|Wire.read(); 
  int xAng = map(AcX,minVal,maxVal,-90,90); 
  int yAng = map(AcY,minVal,maxVal,-90,90); 
  int zAng = map(AcZ,minVal,maxVal,-90,90);
  float rawAngle = RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  accAngle = rawAngle + angleOffset;
  if (accAngle > 180) {
    accAngle -= 360;
  }
  recordGyroRegisters();
}

void processGyroData() {
  rotX = gyroX / 131.0 * PI /180.0;
  rotY = gyroY / 131.0 * PI / 180.0; 
}

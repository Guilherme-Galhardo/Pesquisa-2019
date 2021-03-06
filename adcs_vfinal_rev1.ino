//Determinação de atitude rev1

//Bias em Z de 0.0099768 por amostra
#include <Wire.h>
#include <dummy.h>
#include <MPU6050.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

#define SDA1 21
#define SCL1 22
#define SDA2 5
#define SCL2 4

// Variáveis MPU6050
int intPin = 8;
float aRes, gRes;
#define blinkPin 2
boolean blinkOn = false;
int16_t accelCount[3];
float ax, ay, az;
int16_t gyroCount[3];
float gx, gy, gz;
float gyroBias[3] = {0,0,0.0068}, accelBias[3] = {0,0,0};
int16_t tempCount;
float temp;
float SelfTest[6];
float q[4] = {1.0f,0.0f,0.0f,0.0f};
uint32_t delt_t = 0;
uint32_t count = 0;
float pitch = 0, yaw = 0, roll = 0;
float GyroMeasError = PI * (5.0f/180.0f);
float beta = sqrt(3.0f/4.0f) * GyroMeasError;
float GyroMeasDrift = PI * (6.0f/180.0f);
float zeta = sqrt(1.89f/4.0f) * GyroMeasDrift;
float deltat = 0.001f;
uint32_t lastUpdate = 0, firstUpdate = 0;
uint32_t Now = 0;
double angRef = 50;

MPU6050lib mpu;

//Variáveis Motor
#define DAC1 25
const int currentPin = 13;
const int dirPin = 12;

void setup() {
  Wire.setClock(800000);
  Wire.begin();
  Serial.begin(115200);
  SerialBT.begin("ADCS");

  dacWrite(DAC1, 0);

  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(dirPin, INPUT);
  digitalWrite(dirPin, LOW);
  pinMode(blinkPin, OUTPUT);
  digitalWrite(blinkPin, HIGH);

  STest();

}

void loop() {
  
  AttDetermination ();
}

void STest () {
  uint8_t c = mpu.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);
  if(c == 0x68) {
    mpu.MPU6050SelfTest(SelfTest);
    if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
      mpu.calibrateMPU6050(gyroBias, accelBias);
      mpu.initMPU6050();
    }
    else {
      while(1);
    }
  }
}

void SatAttDetQuat(float ax, float ay, float az, float gx, float gy, float gz){
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float f1, f2, f3;
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;
  float qDot1, qDot2, qDot3, qDot4;
  float hatDot1, hatDot2, hatDot3, hatDot4;
  float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;

  float _halfq1 = 0.5f * q1;
  float _halfq2 = 0.5f * q2;
  float _halfq3 = 0.5f * q3;
  float _halfq4 = 0.5f * q4;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;

  norm = sqrt(ax*ax + ay*ay + az*az);
  if (norm==0.0f) return;
  norm = 1.0f/norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  f1 = _2q2 * q4 - _2q1 * q3 - ax;
  f2 = _2q1 * q2 + _2q3 * q4 - ay;
  f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
  J_11or24 = _2q3;
  J_12or23 = _2q4;
  J_13or22 = _2q1;
  J_14or21 = _2q2;
  J_32 = 2.0f * J_14or21;
  J_33 = 2.0f * J_11or24;
  
  hatDot1 = J_14or21 * f2 - J_11or24 * f1;
  hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
  hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
  hatDot4 = J_14or21 * f1 + J_11or24 * f2;
  
  norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
  hatDot1 /= norm;
  hatDot2 /= norm;
  hatDot3 /= norm;
  hatDot4 /= norm;

  gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
  gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
  gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

  gbiasx += gerrx * deltat * zeta;
  gbiasy += gerry * deltat * zeta;
  gbiasz += gerrz * deltat * zeta;
  gx -= gbiasx;
  gy -= gbiasy;
  gz -= gbiasz;

  qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
  qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
  qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
  qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

  q1 += (qDot1 -(beta * hatDot1)) * deltat;
  q2 += (qDot2 -(beta * hatDot2)) * deltat;
  q3 += (qDot3 -(beta * hatDot3)) * deltat;
  q4 += (qDot4 -(beta * hatDot4)) * deltat;

  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);   
  norm = 1.0f/norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

void AttDetermination () {
    if(mpu.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {
      mpu.readAccelData(accelCount);
      aRes = mpu.getAres();
      ax = (float)accelCount[0] * aRes;
      ay = (float)accelCount[1] * aRes;
      az = (float)accelCount[2] * aRes;
      mpu.readGyroData(gyroCount);
      gRes = mpu.getGres();
      gx = (float)gyroCount[0] * gRes;
      gy = (float)gyroCount[1] * gRes;
      gz = (float)gyroCount[2] * gRes;
      tempCount = mpu.readTempData();
      temp = ((float)tempCount)/340. + 36.53;
    }
    
    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f);
    lastUpdate = Now;
    if(lastUpdate - firstUpdate > 10000000uL) {
      beta = 0.041;
      zeta = 0.015;
    }

    SatAttDetQuat(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f);
    
    delt_t = millis() - count;
    if(delt_t > 500) {
      digitalWrite(blinkPin, blinkOn);
      
      yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
      pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2])); 
      roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

      pitch *= 180.0f / PI;
      yaw   *= 180.0f / PI;
      roll  *= 180.0f / PI;

      blinkOn = ~blinkOn;
      count = millis();

      PIDAttControlSimple(angRef, yaw);
      int corrente = analogRead(currentPin);
      
      char telemetria[100];
      sprintf(telemetria, "%f,%f,%d\n", yaw, angRef, corrente);
      SerialBT.println(telemetria);
  }
}

void PIDdetumbleSimple (double ws) {
  double kpd = 3.5;
  double kdd = 1.3;
  double kid = 1;

  unsigned long t_now, t_last;
  double t_ep;
  double error;
  double lastError;
  double cumError, rateError;
  double preoutput;
  double setPoint = 0;

  t_now = millis();
  t_ep = (double)(t_now - t_last);

  error = setPoint - ws;
  cumError += error * t_ep;
  rateError = (error - lastError) / t_ep;

  preoutput = kpd*error + kdd*rateError + kid*cumError;
  int scale = min(1.0,abs(preoutput));
  int output = (int)(255*scale);

  if (preoutput < 0)
  {
    digitalWrite(dirPin, LOW);
    dacWrite(DAC1, output);
  }

  else
  {
    digitalWrite(dirPin,HIGH);
    dacWrite(DAC1, output);
  }
}

void PIDAttControlSimple (double angleRef, float yaw) {

  double kpd = 3.5;
  double kdd = 1.3;
  double kid = 1;

  unsigned long t_now, t_last;
  double t_ep;
  double error;
  double lastError;
  double cumError, rateError;
  double preoutput;

  t_now = millis();
  t_ep = (double)(t_now - t_last);

  error = angleRef - yaw;
  cumError += error * t_ep;
  rateError = (error - lastError) / t_ep;

  preoutput = kpd*error + kdd*rateError + kid*cumError;
  int scale = min(1.0,abs(preoutput));
  int output = (int)(255*scale);

  if (preoutput < 0)
  {
    digitalWrite(dirPin, LOW);
    dacWrite(DAC1, output);
  }

  else
  {
    digitalWrite(dirPin,HIGH);
    dacWrite(DAC1, output);
  }
}

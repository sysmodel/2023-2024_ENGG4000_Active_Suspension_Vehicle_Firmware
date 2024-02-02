#include "TestHeader.h"



static double currentVal;
static int offsetWindow = 1000;

ADS1115 ADS(0x48);
static float f = ADS.toVoltage(1);  // voltage factor
static long int nowtime = 0;
static long int lasttime = 0;
static double fcurrentOffset = 10.0;
static double fcurrent; // filtered current
static float timeConstant = 0.04; // time constant in s

static int sensVVal;
static int scaledVolt;
static float realVolt;

static double setIPID, currentPID, outPID; // define PID variable
static double Kp=2, Ki=0, Kd=0;  // specify initial tuning parameters
PID myPID(&currentPID, &outPID, &setIPID, Kp, Ki, Kd, DIRECT);



void InitStuff() {
  Wire.begin();
  ADS.begin();
  ADS.setGain(0);
}

int CCUpdatePWM(double setCurr, double curr, float setVolt, float volt) {
  setIPID = setCurr;
  currentPID = curr;
  myPID.Compute();
  int pwmOffset = int(setVolt / volt * 255.0);
  int pwm = pwmOffset + int(outPID);
  return pwm;
}

float GetVoltage() {
  sensVVal = analogRead(sensV);
  scaledVolt = map(sensVVal, 0, 1023, 0, 5000);
  realVolt = float(scaledVolt * 0.005);  // in volts
  return realVolt;
}

double GetCurrent() {
  int16_t val = ADS.readADC(ADCCurrent);
  double current = (int(val * f * 1000.0) - 2491)/(0.066);
  lasttime = nowtime;
  nowtime = millis();
  fcurrent = fcurrent + (current - fcurrent + fcurrentOffset) * double(nowtime - lasttime) / double(1000 * timeConstant);
  fcurrent = fcurrent / 1000.0;
  return fcurrent;
}

double CalibrateCurrent() {
  float currentSum = 0;
  for (int i = 0; i < offsetWindow; i++)
  {
    currentVal = GetCurrent();
    currentSum += currentVal;
  }
  double offset = currentSum / float(offsetWindow-1);
  return offset;
}


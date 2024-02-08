#include "CurrentTestHeader.h"



static double currentVal;
static int offsetWindow = 500;


static float f = ADS.toVoltage(1);  // voltage factor
static long int nowtime = 0;
static long int lasttime = 0;
// static double fcurrentOffset = 200.0;

// static double current; // non-filtered current
// static double fcurrent; // filtered current
static float timeConstant = 0.01; // time constant in s

static int sensVVal;
static int scaledVolt;
static float realVolt;

static double setIPID, currentPID, outPID; // define PID variable
static double Kp=2, Ki=0, Kd=0;  // specify initial tuning parameters


ADS1115 ADS(0x48);
PID myPID(&currentPID, &outPID, &setIPID, Kp, Ki, Kd, DIRECT);



void InitStuff() {
  Serial.begin(115200);
  Wire.begin();
  ADS.begin();
  ADS.setDataRate(7);
  ADS.setGain(0);
}






// double CalibrateCurrent() {
//   float currentSum = 0;

//   for (int i = 0; i < offsetWindow; i++)
//   {
//     currentVal = GetFilteredCurrent(0);
//     currentSum += currentVal;
//   }
//   double offset = currentSum / float(offsetWindow-1);
//   return offset;
// }
double CalibrateCurrent() {
  float currentSum = 0;
  for (int i = 0; i < offsetWindow; i++)
  {
    double a = GetCurrent(0);
    currentVal = csAnalog;
    currentSum += currentVal;
  }
  double offset = currentSum / float(offsetWindow);
  return offset;
}








int CCUpdatePWM(double setCurr, double curr, float setVolt, float volt) {
  setIPID = setCurr;
  currentPID = curr;
  myPID.Compute();
  int pwmOffset = int(setVolt / volt * 255.0);
  int pwm = pwmOffset + int(outPID);
  if (pwm > pwmCeiling) {pwm = pwmCeiling;} else if (pwm < 0) {pwm = 0;}
  return pwm;
}








float GetVoltage() {
  sensVVal = analogRead(sensV);
  scaledVolt = map(sensVVal, 0, 1023, 0, 5000);
  realVolt = float(scaledVolt * 0.005);  // in volts
  return realVolt;
}









double GetFilteredCurrent(double fcurrentOffset) {
  double current = GetCurrent(fcurrentOffset);
  lasttime = nowtime;
  nowtime = millis();
  fcurrent = fcurrent + (current - fcurrent) * double(nowtime - lasttime) / double(1000 * timeConstant);
  // fcurrent = fcurrent / 1000.0;
  return fcurrent;
}






double GetCurrent(int currOff) {
  int16_t val = ADS.readADC(ADCCurrent);
  csAnalog = int(val * f * 1000.0);
  current = double(csAnalog - int(currOff))/(0.066);
  return current;
}
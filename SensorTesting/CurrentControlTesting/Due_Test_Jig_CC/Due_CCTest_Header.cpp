#include "Due_CCTest_Header.h"


// //---------------------------------------------------------------------

// // *** Global Variables ***

// // Misc.
// extern int sensVVal = 0;
// extern int sensCVal = 0;
// extern unsigned long scaledVolt;
// extern float realVolt = 0;
// extern float realAmp = 0;
// extern double currentSensorSensitivity = 66; // mv/A
// extern const int pwmCeiling;

// // Strain gauge pins and object
// extern const int LOADCELL_DOUT_PIN = 5;
// extern const int LOADCELL_SCK_PIN = 6;
// extern long reading;
// extern long lastReading;
// extern unsigned long dReading;
// extern float weight;
// extern HX711 scale;  // object for strain gauge

// // Current sensing
// extern unsigned int total; // holds <= 64 analogReads
// extern byte numReadings = 64;
// extern float offset = 512.1; // calibrate zero current
// extern float span = 0.066; // calibrate max current | ~0.07315 is for 30A sensor
// extern double current; // holds final current, in A

// // Current control
// extern int pwm = 25;
// extern double k_IT = 0.1282; // in A/(N*m)
// extern double setI = 4.00; // in A
// extern float deltaI;
// extern float torque; // in N*m

// // PID
// extern double setIPID, currentPID, outPID; // define PID variable
// extern double Kp=2, Ki=0, Kd=0;  // specify initial tuning parameters
// extern int errPWM;
// // PID myPID(&currentPID, &outPID, &setIPID, Kp, Ki, Kd, DIRECT);
// PID myPID(&current, &outPID, &setI, Kp, Ki, Kd, DIRECT);
// extern double maxCorrect = pwmCeiling;

// // Current sensor offset correction
// extern float currentOffset = 0;  // in A
// extern int offsetWindow = 1000;
// extern float currentSum;

// // Set up PWM offset for PID use
// extern int pwmOffset = 0;
// extern float setV = 0;


// //---------------------------------------------------------------------


void GetCurrentValue() {
  byte numReadings = 64;
  float offset = 512.1; // calibrate zero current
  float span = 0.066; // calibrate max current | ~0.07315 is for 30A sensor
  unsigned int total = 0; // reset
  for (int i = 0; i < numReadings; i++) total += analogRead(sensC);
  current = (total / numReadings - offset) * span;
}

void ReadSensors() {
  // // Check strain gauge value & calculate weight
  // if (scale.is_ready()) {
  //   lastReading = reading;
  //   reading = scale.read();
  //   reading = -reading - 1500000;
  // } else {
  //   // Serial.println("HX711 not found.");
  // }
  // dReading = reading - lastReading;
  // if (dReading > 0) {
  //   weight = dReading / 45 / 1000;  // 45 units measured are approx 1g, weight is in kg
  // }

  // Check voltage sensor
  int sensVVal = analogRead(sensV);
  unsigned long scaledVolt = map(sensVVal, 0, 1023, 0, 5000);
  realVolt = scaledVolt * 0.005;  // in volts

  // Check current sensor
  GetCurrentValue();
  current -= (currentOffset-7.24);

  CCUpdatePWM();
}

void CCUpdatePWM() {
  // Current control (PID library)
  currentPID = current;
  setIPID = setI;
  myPID.Compute();
  pwmOffset = int(setV / realVolt * 255.0);
  // pwm = int((double(pwm)*100.0 + outPID*100.0)) / 100;
  pwm = pwmOffset + int(outPID);
  if (pwm > pwmCeiling) {pwm = pwmCeiling;} else if (pwm < 0) {pwm = 0;}
  analogWrite(mdEn, pwm);
}

void CalibrateCurrent() {
  float currentSum;
  for (int i = 0; i < offsetWindow; i++)
  {
    GetCurrentValue();
    currentSum += current;
  }
  currentOffset = currentSum / float(offsetWindow-1);
}
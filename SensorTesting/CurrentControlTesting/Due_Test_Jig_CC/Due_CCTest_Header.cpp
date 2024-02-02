#include "Due_CCTest_Header.h"




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
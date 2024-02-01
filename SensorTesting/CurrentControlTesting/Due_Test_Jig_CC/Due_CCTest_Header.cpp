#include "Due_CCTest_Header.h"

void GetCurrentValue() {
  total = 0; // reset
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
  sensVVal = analogRead(sensV);
  scaledVolt = map(sensVVal, 0, 1023, 0, 5000);
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

  // // Current control (not PID)
  // deltaI = setI - current;
  // pwm = pwm + deltaI*6;
  // if (pwm > pwmCeiling) {pwm = pwmCeiling;} else if (pwm < 0) {pwm = 0;}
  // analogWrite(mdEn, pwm);
}

void CalibrateCurrent() {
  for (int i = 0; i < offsetWindow; i++)
  {
    GetCurrentValue();
    currentSum += current;
    if (i == (offsetWindow-1)) {
      currentOffset = currentSum / float(offsetWindow-1);
    }
  }
}
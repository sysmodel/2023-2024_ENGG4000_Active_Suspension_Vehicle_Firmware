/* 
* Rack and Pinion Test Jig Firmware (w/ current control)
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Jan 22, 2024
* Last Update: Jan 26, 2024
* Board: Uno R4 Minima
* Version: 3
*
I/O:
- force sensor: digital inputs
- enable: pwm digital output
- voltage sensor: analog input
- current sensor: analog input
- direction: digital outputs
- potentiometer: manual control of current setpoint

Description:
- This code is written to test the rack and pinion mechanism with a 390 motor driven to a set current while measuring a strain gauge. 
- There are force (strain gauge), voltage and current sensors.
- There is PID control from the current reading to produce a corrective voltage.

References:
- Link: https://forum.arduino.cc/t/is-there-a-good-replacement-for-timerone/1182956/10
*/

//------------------------------------------------------------------

#include "CurrentTestHeader.h"

//------------------------------------------------------------------

// *** Setting variables ***
#define pwmCeiling 50
String runMode = "run"; // can be 'run' or 'stop' or 'poten'
double setI = 4.00; // in A
float resistance = 0.2234;  // in ohm
double Kp=2, Ki=0, Kd=0;  // specify PID tuning parameters
double maxCorrect = 50;

// *** Variable declarations ***
float setV = resistance * setI;
int potVal; // for potentiometer manual PWM control
double deltaI; // in A

//------------------------------------------------------------------

void ReadSensors() {
  GetFilteredCurrent();
  GetVoltage();
}

void CCUpdatePWM() {
  setIPID = setI;
  currentPID = fcurrent;
  myPID.Compute();
  pwmOffset = int(setV / voltage * 255.0);
  pwm = pwmOffset + int(outPID);
  if (pwm > pwmCeiling) {pwm = pwmCeiling;} else if (pwm < 0) {pwm = 0;}
  analogWrite(mdEn, pwm);
}

void ReadSG() {
  // Check strain gauge value
  if (scale.is_ready()) {
    readingSG = scale.read();
    readingSG = -readingSG - 1500000;
  } else {
    Serial.println("HX711 not found.");
  }
}

//------------------------------------------------------------------

void setup() {
  myPID.SetOutputLimits(-maxCorrect, maxCorrect);
  InitStuff();
  digitalWrite(mdIn1, LOW);
  digitalWrite(mdIn2, HIGH);
  Timer1.attachInterrupt(ReadSensors).start(3000);
  Timer2.attachInterrupt(CCUpdatePWM).start(1500);
  Timer3.attachInterrupt(ReadSG).start(2000);
  Serial.println("Starting in 1s.");
  delay(1000);
}

void loop() {

  if (runMode == "poten") {
    setI = double(map(analogRead(potPin), 0, 1023, 0, 5000)) / 1000.0;
    setV = resistance * setI;
  }

  // Print values here, then record using Realterm and process using Excel
  deltaI = setI - fcurrent;
  Serial.print(millis());
  Serial.print(",");
  Serial.print(deltaI);       // in A
  Serial.print(",");
  Serial.print(setI);       // in A
  Serial.print(",");  
  Serial.print(readingSG);     // in units (45 gram/unit)
  Serial.print(",");
  Serial.print(currentOffset);
  Serial.print(",");
  Serial.print(voltage);   // in V
  Serial.print(",");
  Serial.print(fcurrent);    // in A
  Serial.print(",");
  Serial.print(pwm);     // 0-255
  Serial.print(",");
  Serial.println(pwmOffset);

  // delay(20);

}
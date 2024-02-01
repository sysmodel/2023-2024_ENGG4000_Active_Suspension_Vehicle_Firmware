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
- potentiometer: manual control of PWM or current setpoint

Description:
- This code is written to test the rack and pinion mechanism with a 390 motor driven back and forth. 
- There are force (strain gauge), voltage and current sensors.

Functions & Descriptions:
- GetCurrentValue(): Runs code to read the current sensor and translate its output into amps.
- ReadSensors(): Runs the current value acquisition function and also obtains the strain gauge value and the current.
- CCUpdatePWM(): Uses a controller to output an appropriate correction to the PWM based on current value error.
- CalibrateCurrent(): In the setup, takes a few iterations to record the current input at 0A so that it may be used for correction.

References:
- Link: https://forum.arduino.cc/t/is-there-a-good-replacement-for-timerone/1182956/10
*/



//---------------------------------------------------------------------


// Firmware main header containing functions to decrease clutter
#include "Due_CCTest_Header.h"


//---------------------------------------------------------------------


void setup() {
  Serial.begin(115200);
  pinMode(sensV, INPUT);
  pinMode(sensC, INPUT);
  pinMode(mdEn, OUTPUT);
  pinMode(mdIn1, OUTPUT);
  pinMode(mdIn2, OUTPUT);
  pinMode(led, OUTPUT);
  Timer3.attachInterrupt(ReadSensors).start(1000);
  digitalWrite(led, LOW);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN, 128);
  myPID.SetOutputLimits(-maxCorrect, maxCorrect);
  myPID.SetMode(AUTOMATIC);
  CalibrateCurrent();
  delay(1500);
  Serial.println("Starting now.");
  digitalWrite(mdIn1, LOW);
  digitalWrite(mdIn2, HIGH);
  setV = resistance * setI;
}

void loop() {
  if(runMode == "run") {
    if (current < 15) {
      // Read sensors and update PWM for current control accordingly
      digitalWrite(led, HIGH);

      // Print values here, then record using Realterm and process using Excel
      deltaI = setI - current;
      Serial.print(millis());
      Serial.print(",");
      Serial.print(deltaI);     // in A
      Serial.print(",");
      Serial.print(setI);       // in A
      Serial.print(",");  
      Serial.print(reading);    // in kg
      Serial.print(",");
      Serial.print(realVolt);   // in V
      Serial.print(",");
      Serial.print(current);    // in A
      Serial.print(",");
      Serial.print(pwm);        // 0-255
      Serial.print(",");
      Serial.print(pwmOffset);
      Serial.print(",");
      Serial.println(outPID);
    } else {
      analogWrite(mdEn, 0);
    }
  } else if (runMode == "stop") {
    analogWrite(mdEn, 0);
  } else if (runMode == "poten") {
    digitalWrite(led, HIGH);
    potVal = map(analogRead(potPin), 0, 1023, 0, pwmCeiling);  // setting PWM ceiling at 200 (max 255)
    analogWrite(mdEn, potVal);
    ReadSensors();
  }
}


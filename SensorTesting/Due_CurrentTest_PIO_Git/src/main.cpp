/* 
* Rack and Pinion Test Jig Firmware (w/ current control)
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Jan 22, 2024
* Last Update: Jan 26, 2024
* Board: Due
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
int pwmCeiling = 225;
String runMode = "run"; // can be 'run' or 'stop' or 'poten'
double setI = 1; // in A
float resistance = 0.663; //0.663;  // in ohm; original value was 0.2234 ohm, but this was not reflected in the current control
double Kp=10, Ki=200, Kd=0;  // specify PID tuning parameters
double maxCorrect = 255;

// *** Variable declarations ***
float setV = resistance * setI;
int potVal; // for potentiometer manual PWM control
double deltaI; // in A
int pwmStep;
float setIStep = 0;
int cnt = 1;
long int funcTime = 0;

//------------------------------------------------------------------

void CCUpdatePWM() {
  setIPID = setI;
  currentPID = currentINA;
  myPID.Compute();
  setV = resistance * setI;
  pwmOffset = int(setV / voltage * 255.0);
  pwm = pwmOffset + int(outPID);
  if (pwm > pwmCeiling) {pwm = pwmCeiling;} else if (pwm < 0) {pwm = 0;}
  analogWrite(mdEn, pwm);
}

void ReadSensors() {
  funcTime = micros();
  // GetCurrent(currentOffset);
  // GetFilteredCurrent();
  GetCurrentINA(currentOffsetINA);


  CCUpdatePWM();
  funcTime = micros() - funcTime;
}

void ReadSG() {
  // Check strain gauge value
  if (scale.is_ready()) {
    readingSG = scale.read();
    readingSG = -readingSG - 1500000;
  } else {
    // Serial.println("HX711 not found.");
  }
}

void StepPotPWM() {
  if (runMode == "poten") {
    pwmStep = map(analogRead(potPin), 0, 1023, -2, 2);
    pwm += pwmStep;
    if (pwm > pwmCeiling) {pwm = pwmCeiling;} else if (pwm < 0) {pwm = 0;}
    analogWrite(mdEn, pwm);
  }
}

void StepSetI() {
  int mySets[] = {2, 4, 8, 3, 6, 4};
  setI = cnt * 1.0;
  cnt++;
  if (cnt > 9) {cnt = 0;}
}

//------------------------------------------------------------------

void setup() {
  SetDirec("CW");
  InitStuff();
  myPID.SetOutputLimits(-maxCorrect, maxCorrect);
  pwm = 0;
  CalibrateCurrentINA();
  Timer1.attachInterrupt(ReadSensors).start(10000);
  // Timer2.attachInterrupt(CCUpdatePWM).start(4000);
  // analogWrite(mdEn, 25);
  Timer3.attachInterrupt(ReadSG).start(40000);
  // Timer4.attachInterrupt(StepPotPWM).start(500000);
  Timer5.attachInterrupt(StepSetI).start(5000000);
  Serial.println("Starting in 1s.");
  delay(1000);
}

void loop() {

  GetVoltage();

  if (runMode == "poten") {
    setI = double(map(analogRead(potPin), 0, 1023, 0, 6000)) / 1000.0;
    setV = resistance * setI;
    // pwm = map(analogRead(potPin), 0, 1023, 0, pwmCeiling);
    // if (pwm > pwmCeiling) {pwm = pwmCeiling;} else if (pwm < 0) {pwm = 0;}
    // analogWrite(mdEn, pwm);
  }

  // Print values here, then record using Realterm and process using Excel
  deltaI = setI - currentINA;
  Serial.print(millis());
  Serial.print(",");
  Serial.print(deltaI);       // in A
  Serial.print(",");
  Serial.print(setI);       // in A
  Serial.print(",");
  Serial.print(readingSG);     // in units (45 gram/unit)
  // Serial.print(",");
  // Serial.print(funcTime);
  // Serial.print(",");  
  // Serial.print(csAnalog);
  // Serial.print(",");
  // Serial.print(currentOffsetINA);
  Serial.print(",");
  Serial.print(voltage);   // in V
  Serial.print(",");
  Serial.print(currentINA);    // in A
  // Serial.print(",");
  // Serial.print(outPID);
  Serial.print(",");
  Serial.println(pwm);     // 0-255
  // Serial.print(",");
  // Serial.println(pwmOffset);

  delay(10);

}
#include "TestHeader.h"



// *** Setting variables ***
String runMode = "run"; // can be 'run' or 'stop' or 'poten'
double setI = 4.00; // in A
float resistance = 0.2234;  // in ohm
int pwmCeiling = 50;

// *** Variable declarations ***
double readCurrent;
float readVoltage;
double currentOffset;
float setV = resistance * setI;
int potVal; // for potentiometer manual PWM control
double deltaI; // in A



void ReadSensors() {
  readCurrent = GetCurrent();
  readVoltage = GetVoltage();
}

void setup() {
  Serial.begin(115200);
  currentOffset = CalibrateCurrent();
  Timer3.attachInterrupt(ReadSensors).start(2000);
  Serial.println("Starting in 1s.");
  delay(1000);
}

void loop() {

  ReadSensors();

  if (runMode == "poten") {
    setI = double(map(analogRead(potPin), 0, 1023, 0, 5000)) / 1000.0;
    setV = resistance * setI;
  }

  // Print values here, then record using Realterm and process using Excel
  deltaI = setI - readCurrent;
  Serial.print(millis());
  Serial.print(",");
  Serial.print(deltaI);       // in A
  Serial.print(",");
  Serial.print(setI);       // in A
  Serial.print(",");  
  // Serial.print(reading);     // in kg
  // Serial.print(",");
  Serial.print(readVoltage);   // in V
  Serial.print(",");
  Serial.print(readCurrent);    // in A
  Serial.print(",");
  Serial.print(pwm);     // 0-255
  Serial.print(",");
  Serial.print(pwmOffset);
  Serial.print(",");
  Serial.println(outPID);

}

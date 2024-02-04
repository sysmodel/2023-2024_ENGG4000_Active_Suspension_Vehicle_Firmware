#include "TestHeader.h"



// *** Setting variables ***
String runMode = "run"; // can be 'run' or 'stop' or 'poten'
double setI = 4.00; // in A
float resistance = 0.2234;  // in ohm

// *** Variable declarations ***
double readCurrent;
float readVoltage;
double currentOffset;
float setV = resistance * setI;
int potVal; // for potentiometer manual PWM control
double deltaI; // in A
double outOffset = 0; // in A
double sum;
int cnt = 0;



void ReadSensors() {
  readCurrent = GetFilteredCurrent(currentOffset) - outOffset;
  readVoltage = GetVoltage();
}

void setup() {
  InitStuff();
  currentOffset = CalibrateCurrent();
  Serial.print("Current offset: ");Serial.println(currentOffset,3);
  Timer1.attachInterrupt(ReadSensors).start(3000);
  Serial.println("Starting in 1s.");
  delay(1000);
}

void loop() {

  // if ((abs(readCurrent) < 50) && (cnt > 40)) {cnt = 0; sum = 0;}
  for (;cnt < 50; cnt++) {
    sum += readCurrent;
  }
  outOffset = sum/(cnt+1);

  if (runMode == "poten") {
    setI = double(map(analogRead(potPin), 0, 1023, 0, 5000)) / 1000.0;
    setV = resistance * setI;
  }

  // Serial.println(readCurrent);

  // // Print values here, then record using Realterm and process using Excel
  // deltaI = setI - readCurrent;
  // Serial.print(millis());
  // Serial.print(",");
  // Serial.print(deltaI);       // in A
  // Serial.print(",");
  // Serial.print(setI);       // in A
  // Serial.print(",");  
  // // Serial.print(reading);     // in kg
  // // Serial.print(",");
  Serial.print(outOffset);
  Serial.print(",");
  Serial.print(TestOutput() - int(outOffset));
  Serial.print(",");
  // Serial.print(currentOffset);
  // Serial.print(",");
  // Serial.print(readVoltage);   // in V
  // Serial.print(",");
  Serial.println(readCurrent);    // in A
  // Serial.print(",");
  // Serial.print(pwm);     // 0-255
  // Serial.print(",");
  // Serial.print(pwmOffset);
  // Serial.print(",");
  // Serial.println(outPID);

  // delay(20);

}

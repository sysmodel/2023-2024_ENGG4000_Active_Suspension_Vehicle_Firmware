#include <Arduino.h>
#include "DueTimer.h"
#include "QuadEncoder.h"
#include "AbsEncoders.h"
#include "Adafruit_INA260.h"
#include "PID_v1.h"
#include "Waveforms.h"

//------------------------------------------------------------------

#define led LED_BUILTIN
#define steerPotPin A4
#define battCell1Pin A3
#define battCell2Pin A2

//------------------------------------------------------------------

// Variable definitions
float current[4] = {0,0,0,0}; // array of measured current values
float offset[4] = {0,0,0,0};  // array of current offset values
int i = 0; // used for indexing
float voltArray[2] = {0,0};
float battVoltage = 0;
long timeCount = millis();
int sineCount = 0;
int funcTime = 0;
int currentFlag = 0;

// Motor driver
uint32_t mdEnPins[4] = {4,5,2,3};
uint32_t mdIn1Pins[4] = {51,53,47,49};
uint32_t mdIn2Pins[4] = {50,52,46,48};
int pwm[4] = {0,0,0,0};
bool direc[4] = {0,0,0,0}; // motor action directions; 1 is up, 0 is down
bool desDirec[4] = {0,0,0,0}; // desired motor action directions; 1 is up, 0 is down
bool direcChange[4] = {0,0,0,0};

// Jetson communication
float setI[4] = {0,0,0,0}; // array of stored current setpoints

// FF-PI controller
float resistance[4] = {0.372,0.301,0.310,0.325}; // in ohm; original value was 0.2234 ohm, but this was not reflected in the current control
double Kp=0, Ki=400, Kd=0;  // specify PID tuning parameters
double maxCorrect = 255; // used in piFR.SetOutputLimits() function
double currentPI[4] = {0,0,0,0}; // array of current values to be used by the PI objects
double outPI[4] = {0,0,0,0}; // array to store the outputs of the PI objects
double setIPI[4] = {0,0,0,0}; // array of current setpoint values to be used by the PI objects
float setV[4] = {0,0,0,0};
int pwmOffset[4] = {0,0,0,0};
int pwmCeiling = 180;

// Encoders
uint8_t quadEncoderFlag = 0;
uint8_t absEncoderFlag = 0;
uint8_t resolution = 12;
// - Define global variable to store abs encoder positions and speeds
uint16_t absEncCurrentPositionFR; // Front Right
uint16_t absEncCurrentPositionFL; // Front Left
uint16_t absEncCurrentPositionBR; // Back Right
uint16_t absEncCurrentPositionBL; // Back Left
double absEncCurrentVelocityFR; // Front Right
double absEncCurrentVelocityFL; // Front Left
double absEncCurrentVelocityBR; // Back Right
double absEncCurrentVelocityBL; // Back Left
// - Define pins for each encoder; structure of arrays: {FR, FL, BR, BL}; indices: {0,1,2,3}
uint8_t sdoPin[4] = {11, 13, 7, 9};
uint8_t sckPin[4] = {10, 12, 6, 8};
uint8_t csPin[4] = {25, 24, 27, 26};

//------------------------------------------------------------------

// Creation of absolute encoder objects; structure of arrays: {FR, FL, BR, BL}; indices: {0,1,2,3}
AbsEnc absEncoderFR(sckPin[0], csPin[0], sdoPin[0], resolution);
AbsEnc absEncoderFL(sckPin[1], csPin[1], sdoPin[1], resolution);
AbsEnc absEncoderBR(sckPin[2], csPin[2], sdoPin[2], resolution);
AbsEnc absEncoderBL(sckPin[3], csPin[3], sdoPin[3], resolution);

// Creation of INA260 current sensor objects
Adafruit_INA260 ina260FR = Adafruit_INA260();
Adafruit_INA260 ina260FL = Adafruit_INA260();
Adafruit_INA260 ina260BR = Adafruit_INA260();
Adafruit_INA260 ina260BL = Adafruit_INA260();

// Creating of PID objects
PID piFR(&currentPI[0], &outPI[0], &setIPI[0], Kp, Ki, Kd, DIRECT);
PID piFL(&currentPI[1], &outPI[1], &setIPI[1], Kp, Ki, Kd, DIRECT);
PID piBR(&currentPI[2], &outPI[2], &setIPI[2], Kp, Ki, Kd, DIRECT);
PID piBL(&currentPI[3], &outPI[3], &setIPI[3], Kp, Ki, Kd, DIRECT);

//------------------------------------------------------------------

void SetDirec(int wheel, bool dir) {
  digitalWrite(mdIn1Pins[wheel], HIGH);
  digitalWrite(mdIn2Pins[wheel], HIGH);
  if (dir == 1) {
    digitalWrite(mdIn2Pins[wheel], HIGH);
    digitalWrite(mdIn1Pins[wheel], LOW);
  } else if (dir == 0) {
    digitalWrite(mdIn1Pins[wheel], HIGH);
    digitalWrite(mdIn2Pins[wheel], LOW);
  }
}

void GetCurrent() {

  for(int i=0;i<4;i++) {
    switch(i) {
      case 0:
        current[i] = float(ina260FR.readCurrent())/1000.0 - offset[i];
        break;
      case 1:
        current[i] = float(ina260FL.readCurrent())/1000.0 - offset[i];
        break;
      case 2:
        current[i] = float(ina260BR.readCurrent())/1000.0 - offset[i];
        break;
      case 3:
        current[i] = float(ina260BL.readCurrent())/1000.0 - offset[i];
        break;
    }
    
    current[i] = 1.0887*current[i];

    // if (current[i] < 0) {
    //   direc[i] = 0;
      
    // } else {
    //   direc[i] = 1;
    // }
  }
  currentFlag = 1;
}

void GetVoltage() {
  voltArray[0] = float(map(analogRead(battCell1Pin), 0, 1023, 0, 3300)) * 0.005;
  battVoltage = float(map(analogRead(battCell2Pin), 0, 1023, 0, 3300)) * 0.005;
  voltArray[1] = battVoltage - voltArray[0];
}

void GetQuadEncoderData() {
  // Get the vehicle speed in rad/s
  quadEncoderVel = GetQuadEncoderSpeed();

  // Set a flag to print this value in the loop()
  quadEncoderFlag = 1;
}

void GetAbsEncoderData() {
  // Get the positions from each abs encoder
  absEncCurrentPositionFR = absEncoderFR.AbsEncPos();
  absEncCurrentPositionFL = absEncoderFL.AbsEncPos();
  absEncCurrentPositionBR = absEncoderBR.AbsEncPos();
  absEncCurrentPositionBL = absEncoderBL.AbsEncPos();

  // Get the velocities from each abs encoder
  absEncCurrentVelocityFR = absEncoderFR.AbsEncVel();
  absEncCurrentVelocityFL = absEncoderFL.AbsEncVel();
  absEncCurrentVelocityBR = absEncoderBR.AbsEncVel();
  absEncCurrentVelocityBL = absEncoderBL.AbsEncVel();

  // Set flag to print this value in the loop()
  absEncoderFlag = 1;
}

void InitStuff() {
  // Initialize serial communication at 115200 bps
  Serial.begin(115200);
  
  // Setting pin modes
  pinMode(led, OUTPUT); // for built-in LED, HIGH is on and vice versa
  for(i=0;i<4;i++) {  // pins of motor drivers
    pinMode(mdEnPins[i], OUTPUT);
    pinMode(mdIn1Pins[i], OUTPUT);
    pinMode(mdIn2Pins[i], OUTPUT);
  }
  pinMode(steerPotPin, INPUT);  // steering potentiometer pin
  pinMode(battCell1Pin, INPUT); // pin of voltage sensor measuring battery cell #1
  pinMode(battCell2Pin, INPUT); // pin of voltage sensor measuring battery cell #2

  // Begin current sensor reading and set averaging count
  if (!ina260FR.begin(0x41)) {
    Serial.println("Couldn't find INA260 chip (FR)");
    while (1);
  }
  if (!ina260FL.begin(0x44)) {
    Serial.println("Couldn't find INA260 chip (FL)");
    while (1);
  }
  if (!ina260BR.begin(0x45)) {
    Serial.println("Couldn't find INA260 chip (BR)");
    // while (1);
  }
  if (!ina260BL.begin(0x40)) {
    Serial.println("Couldn't find INA260 chip (BL)");
    // while (1);
  }
  Serial.println("Found INA260 chip");
  ina260FR.setMode(INA260_MODE_CURRENT_CONTINUOUS);
  ina260FL.setMode(INA260_MODE_CURRENT_CONTINUOUS);
  ina260BR.setMode(INA260_MODE_CURRENT_CONTINUOUS);
  ina260BL.setMode(INA260_MODE_CURRENT_CONTINUOUS);
  // ina260FR.setAveragingCount(INA260_COUNT_16);
  // ina260FL.setAveragingCount(INA260_COUNT_16);
  // ina260BR.setAveragingCount(INA260_COUNT_16);
  // ina260BL.setAveragingCount(INA260_COUNT_16);
  ina260FR.setCurrentConversionTime(INA260_TIME_2_116_ms);
  ina260FL.setCurrentConversionTime(INA260_TIME_2_116_ms);
  ina260BR.setCurrentConversionTime(INA260_TIME_2_116_ms);
  ina260BL.setCurrentConversionTime(INA260_TIME_2_116_ms);

  // Set PID mode, sampling time, and output limits
  piFR.SetMode(AUTOMATIC);
  piFL.SetMode(AUTOMATIC);
  piBR.SetMode(AUTOMATIC);
  piBL.SetMode(AUTOMATIC);
  piFR.SetSampleTime(3);
  piFL.SetSampleTime(3);
  piBR.SetSampleTime(3);
  piBL.SetSampleTime(3);
  piFR.SetOutputLimits(-maxCorrect, maxCorrect);
  piFL.SetOutputLimits(-maxCorrect, maxCorrect);
  piBR.SetOutputLimits(-maxCorrect, maxCorrect);
  piBL.SetOutputLimits(-maxCorrect, maxCorrect);
}

void CCUpdatePWM() {
  // Changed for test from 4
  for(int i=0;i<4;i++) {
    setIPI[i] = setI[i];
    currentPI[i] = current[i];
    setV[i] = resistance[i] * setI[i];
    pwmOffset[i] = int(setV[i] / battVoltage * 255.0); // Feedforward (FF) control
  }

  piFR.Compute();
  piFL.Compute();
  piBR.Compute();
  piBL.Compute();
  // Changed for test was 4
  for(int i=0;i<4;i++) {
    pwm[i] = pwmOffset[i] + int(outPI[i]); // FF + PI control
    if(pwm[i] < 0) {
      pwm[i] = -pwm[i];
      desDirec[i] = 0;
    } else {
      desDirec[i] = 1;
    }
    if (direc[i] != desDirec[i]) {SetDirec(i,desDirec[i]); direc[i] = desDirec[i];}
    // SetDirec(i,desDirec[i]);
    if (pwm[i] > pwmCeiling) {pwm[i] = pwmCeiling;} else if (pwm[i] < 0) {pwm[i] = 0;}
    analogWrite(mdEnPins[i], pwm[i]);
  }
}

// Frequency testing
uint8_t frequency = 15;
bool printToFile = false; 
double last_time = 0;
int amplitude = 4;
float phase[4] = {0,0,0,0};
int mod;

void ActuateAction() {
  // This function calls on independent functions to read the current and ...
  // ... compute the FF-PI controller to actuate a PWM accordingly.
  funcTime = micros();

  mod = funcTime % (2.0E6);
  
  for(i=0;i<4;i++) {
    // setI[i] = amplitude * sin(3 * 2.0 * PI * funcTime/1.0E6 + phase[i]*2*PI);
    if (mod > 1.6E6) {
      setI[i] = amplitude;
    } else {
      setI[i] = -amplitude;
    }
  }

  GetCurrent();
  CCUpdatePWM();
  funcTime = micros() - funcTime;
  last_time = micros()/1.0E6;
  printToFile = true;
}

int timedInputCount = 0;
int freqs[3] = {5, 0, 15};
int amps[3] = {7, -9, 4};
// int mod[4] = {0,0,0,0};

void TimedInput() {
  // frequency = 5 * timedInputCount;
  // if (timedInputCount > 12) {timedInputCount = 0;}


  // frequency = freqs[timedInputCount];
  // amplitude = amps[timedInputCount];
  // frequency = 15;
  // amplitude = 5;
  // timedInputCount++;
  // if (timedInputCount > 2) {timedInputCount = 0;}

  // mod = funcTime%

  // for(i=0;i<4;i++) {
  //   setI[i] = amplitude * sin(3 * 2.0 * PI * funcTime/1.0E6 + phase[i]*2*PI);
  // }
  

  // frequency = 30;

  // amplitude += 2;
  // if (amplitude > 7) {amplitude = 5;}
}


//------------------------------------------------------------------

void setup() {

  InitStuff();

  // Wait until serial port is opened
  while (!Serial) {delay(1);}

  // setI[4] = {-8.0,-8.0,1.0,1.0};
  for(int i=0;i<4;i++) {Serial.println(setI[i]);}
  for(int i=0;i<4;i++) {desDirec[i] = 0;}
  for(int i=0;i<4;i++) {SetDirec(i,0);}

  // Initialize Timmer Interupts for 33Hz
  // Timer1.attachInterrupt(GetQuadEncoderData).start(30303); // Timer for Quad Encoder (33Hz)
  // Timer2.attachInterrupt(GetAbsEncoderData).start(30303);  // Timer for Abs Encoder (33Hz)
  Timer3.attachInterrupt(ActuateAction).start(5000); // Timer for ActuateAction function
  Timer4.attachInterrupt(TimedInput).start(2000000); // Timer for input to actuators

}

double time_capture; 
float current_capture;
void loop() {

  GetVoltage();

  // analogWrite(mdEnPins[0], 60);

  // timeCount = millis();

  /*
  // Print out quadrature encoder data (Validation)
  if (quadEncoderFlag == 1) {
    Serial.print("Quad Encoder Velocity (rad/s): ");
    Serial.println(quadEncoderVel);
    quadEncoderFlag = 0;
  }
  */

  // Print out absolute encoder data (Validation)
  if (absEncoderFlag == 1) {
    Serial.print("FR Abs Encoder Position: ");
    Serial.print(absEncCurrentPositionFR);
    Serial.print(" FR Abs Encoder Velocity: ");
    Serial.println(absEncCurrentVelocityFR);
    absEncoderFlag = 0;


    // Independent of the abs. encoders but should print at the same time
    Serial.print("Currents: ");
    for(int i=0;i<4;i++) {Serial.print(current[i]); Serial.print(",");}
    Serial.println("");
  }

  // Serial.print("Currents: ");
  // for(i=0;i<4;i++) {Serial.print(current[i],3); Serial.print(",");}
  // Serial.println("");
  // for(i=0;i<4;i++) {Serial.print(setI[i],3); Serial.print(",");}
  // Serial.println("");
  // Serial.print("Voltage: "); Serial.println(battVoltage,2);

  // if(currentFlag == 1) {
  //   Serial.println("Currents: ");
  //   for(int i=0;i<4;i++) {Serial.print(current[i],3); Serial.print(",");}
  //   // Serial.println("");
  //   // for(i=0;i<4;i++) {Serial.print(setI[i],3); Serial.print(",");}
  //   for(int i=0;i<4;i++) {Serial.print(pwm[i]); Serial.print(",");}
  //   Serial.print("--");
  //   Serial.print("Voltage: "); Serial.print(battVoltage,2);
  //   Serial.print(", "); Serial.println(funcTime);
  //   currentFlag = 0;
  // }

  // Serial.println("Currents: ");
  // for(int i=0;i<4;i++) {Serial.print(current[i],3); Serial.print(",");}
  // // Serial.println("");
  // // for(i=0;i<4;i++) {Serial.print(setI[i],3); Serial.print(",");}
  // for(int i=0;i<4;i++) {Serial.print(pwm[i]); Serial.print(",");}
  // Serial.print("--");
  // Serial.print("Voltage: "); Serial.print(battVoltage,2);
  // Serial.print(", "); Serial.print(funcTime);
  // Serial.print(", "); for(int i=0;i<4;i++) {Serial.print(direc[i]); Serial.print(",");}
  // Serial.println("");
  // currentFlag = 0;


  if(printToFile){
    time_capture = last_time;
    current_capture = current[3];

    // Serial.print(time_capture, 3);
    // Serial.print(",");
    // Serial.print(setI[0], 6);
    // Serial.print(",");
    Serial.print(current_capture, 4);
    // Serial.print(",");
    // Serial.print(battVoltage, 2);
    Serial.print(",");
    Serial.print(battVoltage);
    Serial.print(",");
    Serial.print(funcTime);
    Serial.print(",");
    Serial.print(setI[3]);
    Serial.println();
    printToFile = false;
  }
  // delay(10);
}


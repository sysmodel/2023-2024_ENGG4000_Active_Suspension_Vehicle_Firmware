#include "FFIControl.h"

//------------------------------------------------------------------

// FF-I controller
float resistance[4] = {0.663*(3/6.2)*(3/2.35)*(3/3.3),0.663,0.663,0.663}; // in ohm; original value was 0.2234 ohm, but this was not reflected in the current control
double Kp=0, Ki=400, Kd=0;  // specify PID tuning parameters
double maxCorrect = 255; // used in piFR.SetOutputLimits() function
double currentPI[4] = {0,0,0,0}; // array of current values to be used by the PI objects
double outPI[4] = {0,0,0,0}; // array to store the outputs of the PI objects
double setIPI[4] = {0,0,0,0}; // array of current setpoint values to be used by the PI objects
float setV[4] = {0,0,0,0};
int pwmOffset[4] = {0,0,0,0};
int pwmCeiling = 120;

// Motor driver
uint32_t mdEnPins[4] = {4,5,2,3};
uint32_t mdIn1Pins[4] = {51,53,47,49};
uint32_t mdIn2Pins[4] = {50,52,46,48};
int pwmH[4] = {0,0,0,0};
bool direc[4] = {0,0,0,0}; // motor action directions; 1 is up, 0 is down
bool desDirec[4] = {0,0,0,0}; // desired motor action directions; 1 is up, 0 is down

// Variable definitions
float curr[4] = {0,0,0,0}; // array of measured current values
float offset[4] = {0,0,0,0};  // array of current offset values
long tSample = 10; // sampling time in ms

// General
int iH; // for indexing

//------------------------------------------------------------------

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

// InitFFIC
void InitFFIC() {
    // Setting pin modes
    for(iH=0;iH<4;iH++) {  // pins of motor drivers
        pinMode(mdEnPins[iH], OUTPUT);
        pinMode(mdIn1Pins[iH], OUTPUT);
        pinMode(mdIn2Pins[iH], OUTPUT);
    }

    // Setting initial motor direction
    for(int iH=0;iH<4;iH++) {desDirec[iH] = 0;}
    for(int iH=0;iH<4;iH++) {SetDirec(iH,0);}

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
        while (1);
    }
    if (!ina260BL.begin(0x40)) {
        Serial.println("Couldn't find INA260 chip (BL)");
        while (1);
    }
    Serial.println("Found INA260 chip");
    ina260FR.setAveragingCount(INA260_COUNT_4);
    ina260FL.setAveragingCount(INA260_COUNT_4);
    ina260BR.setAveragingCount(INA260_COUNT_4);
    ina260BL.setAveragingCount(INA260_COUNT_4);

    // Set PID mode, sampling time, and output limits
    piFR.SetMode(AUTOMATIC);
    piFL.SetMode(AUTOMATIC);
    piBR.SetMode(AUTOMATIC);
    piBL.SetMode(AUTOMATIC);
    piFR.SetSampleTime(tSample);
    piFL.SetSampleTime(tSample);
    piBR.SetSampleTime(tSample);
    piBL.SetSampleTime(tSample);
    piFR.SetOutputLimits(-maxCorrect, maxCorrect);
    piFL.SetOutputLimits(-maxCorrect, maxCorrect);
    piBR.SetOutputLimits(-maxCorrect, maxCorrect);
    piBL.SetOutputLimits(-maxCorrect, maxCorrect);
}

// SetDirec
void SetDirec(int wheel, bool dir) {
  if (dir == 1) {
    digitalWrite(mdIn1Pins[wheel], LOW);
    digitalWrite(mdIn2Pins[wheel], HIGH);
  } else if (dir == 0) {
    digitalWrite(mdIn1Pins[wheel], HIGH);
    digitalWrite(mdIn2Pins[wheel], LOW);
  }
}

// GetCurrent
float GetCurrent(int wheel) {
    switch(wheel) {
        case 0:
        curr[wheel] = (ina260FR.readCurrent() - offset[wheel])/1000.0;
        break;
        case 1:
        curr[wheel] = (ina260FL.readCurrent() - offset[wheel])/1000.0;
        break;
        case 2:
        curr[wheel] = (ina260BR.readCurrent() - offset[wheel])/1000.0;
        break;
        case 3:
        curr[wheel] = (ina260BL.readCurrent() - offset[wheel])/1000.0;
        break;
    }
    if (curr[wheel] < 0) {
        // current[i] = -current[i];
        direc[wheel] = 0;
        curr[wheel] = 1.0637*curr[wheel] - 0.1416;
    } else {
        direc[wheel] = 1;
        curr[wheel] = 1.0637*curr[wheel] + 0.1416;
    }
    return curr[wheel];
    // current[i] = 1.0637*current[i] + 0.1416;
    //   currentFlag = 1;
}

// CCUpdatePWM
int CCUpdatePWM(int wheel, float setI, float crnt, float batteryVolt) {

    // Setup for FFI controller
    setIPI[wheel] = setI;
    currentPI[wheel] = crnt;
    setV[wheel] = resistance[wheel] * setI;
    pwmOffset[wheel] = int(setV[wheel] / batteryVolt * 255.0); // Feedforward (FF) control

    // I controller output calculation
    switch(wheel) {
        case 0:
            piFR.Compute();
            break;
        case 1:
            piFL.Compute();
            break;
        case 2:
            piBR.Compute();
            break;
        case 3:
            piBL.Compute();
            break;
    }
    
    pwmH[wheel] = pwmOffset[wheel] + int(outPI[wheel]); // FF + PI control
    if(pwmH[wheel] < 0) {
      pwmH[wheel] = -pwmH[wheel];
      desDirec[wheel] = 0;
    } else {
      desDirec[wheel] = 1;
    }

    // Actuation
    SetDirec(wheel,desDirec[wheel]);
    if (pwmH[wheel] > pwmCeiling) {pwmH[wheel] = pwmCeiling;} else if (pwmH[wheel] < 0) {pwmH[wheel] = 0;}
    analogWrite(mdEnPins[wheel], pwmH[wheel]);

    // Return PWM value for observation outside this function
    return pwmH[wheel];
}

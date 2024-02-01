#ifndef Due_CCTest_H

#define Due_CCTest_H


//---------------------------------------------------------------------


// Libraries
#include <Arduino.h>
#include "DueTimer.h"
#include "HX711.h"
#include <PID_v1.h>


//---------------------------------------------------------------------


// Pins
#define potPin A0
#define sensV A1
#define sensC A2  //current is analog signal via 100mV/A
#define mdEn 3 //motor driver enable, PWM signal
#define mdIn1 8 //high or low signal for direction, complements mdIn2
#define mdIn2 9 //low or high signal for direction, complements mdIn1
#define led LED_BUILTIN


//---------------------------------------------------------------------


// *** Global Variables ***

// Misc.
String runMode = "run"; // can be 'run' or 'stop' or 'poten'
int potVal; // for potentiometer manual PWM control
int sensVVal = 0;
int sensCVal = 0;
volatile int cnt = 0;
unsigned long scaledVolt;
float realVolt = 0;
float realAmp = 0;
double currentSensorSensitivity = 66; // mv/A
int pwmCeiling = 80;

// LED control (for PWM state visualization)
int in1State = LOW;
int in2State = LOW;
int ledState = LOW;

// Strain gauge pins and object
const int LOADCELL_DOUT_PIN = 5;
const int LOADCELL_SCK_PIN = 6;
long reading;
long lastReading;
unsigned long dReading;
float weight;
HX711 scale;  // object for strain gauge

// Current sensing
unsigned int total; // holds <= 64 analogReads
byte numReadings = 64;
float offset = 512.1; // calibrate zero current
float span = 0.066; // calibrate max current | ~0.07315 is for 30A sensor
double current; // holds final current, in A
float rawData;

// Current control
int pwm = 25;
int duty;
int setT = 10; // 23 // in N*m
double k_IT = 0.1282; // in A/(N*m)
double setI = 4.00; // in A
float deltaI;
float torque; // in N*m

// PID
double setIPID, currentPID, outPID; // define PID variable
double Kp=2, Ki=0, Kd=0;  // specify initial tuning parameters
int errPWM;
// PID myPID(&currentPID, &outPID, &setIPID, Kp, Ki, Kd, DIRECT);
PID myPID(&current, &outPID, &setI, Kp, Ki, Kd, DIRECT);
double maxCorrect = pwmCeiling;

// Current sensor offset correction
float currentOffset = 0;  // in A
int offsetWindow = 1000;
float currentSum;

// Set up PWM offset for PID use
int pwmOffset = 0;
float resistance = 0.2234;  // in ohm
float setV = 0;


//---------------------------------------------------------------------


// *** Function Declarations ***

void GetCurrentValue();

void ReadSensors();

void CCUpdatePWM();

void CalibrateCurrent();


//---------------------------------------------------------------------


#endif // Due_CCTest_H


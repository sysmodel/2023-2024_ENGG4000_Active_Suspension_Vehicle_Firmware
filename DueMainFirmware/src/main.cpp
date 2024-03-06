/* 
* Sys-MoDEL Active Suspension Main.ino file
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Jan 6, 2024
* Last Update: March 6, 2024
*
* Version 1.0
*
* Description: 
* This code is the Main.ino file that is used to retrieve data from the array of sensors. 
* Each retrieval of data is done by executing an internal ISR on the Arduino DUE. 
*
* Functions & Descriptions: 
* Name: 
* Description: 
*
* References:
*
*/

#include <Arduino.h>
#include "DueTimer.h"
#include "QuadEncoder.h"
#include "AbsEncoders.h"


// Variable definitions
uint8_t quadEncoderFlag = 0;
uint8_t absEncoderFlag = 0;
uint8_t resolution = 12;

// Define global variable to store abs encoder positions and speeds
uint16_t absEncCurrentPositionFR; // Front Right
uint16_t absEncCurrentPositionFL; // Front Left
uint16_t absEncCurrentPositionBR; // Back Right
uint16_t absEncCurrentPositionBL; // Back Left

double absEncCurrentVelocityFR; // Front Right
double absEncCurrentVelocityFL; // Front Left
double absEncCurrentVelocityBR; // Back Right
double absEncCurrentVelocityBL; // Back Left

// Define pins for each encoder
uint8_t sdoPinFR = 5;
uint8_t sckPinFR = 6;
uint8_t csPinFR = 7;

uint8_t sdoPinFL = 2;
uint8_t sckPinFL = 3;
uint8_t csPinFL = 4;

uint8_t sdoPinBR = 11;
uint8_t sckPinBR = 12;
uint8_t csPinBR = 13;

uint8_t sdoPinBL = 8;
uint8_t sckPinBL = 9;
uint8_t csPinBL = 10;

// Creation of absolute encoder objects
AbsEnc absEncoderFR(sckPinFR, csPinFR, sdoPinFR, resolution);
AbsEnc absEncoderFL(sckPinFL, csPinFL, sdoPinFL, resolution);
AbsEnc absEncoderBR(sckPinBR, csPinBR, sdoPinBR, resolution);
AbsEnc absEncoderBL(sckPinBL, csPinBL, sdoPinBL, resolution);

void GetQuadEncoderData()
{
  // Get the vehicle speed in rad/s
  quadEncoderVel = GetQuadEncoderSpeed();

  // Set a flag to print this value in the loop()
  quadEncoderFlag = 1;
}

void GetAbsEncoderData()
{
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

void setup() 
{
  Serial.begin(115200); // Initialize serial communication at 115200 bps

  // Wait until serial port is opened
  while (!Serial) { delay(1); }

  // Initialize Timmer Interupts for 33Hz
  Timer1.attachInterrupt(GetQuadEncoderData).start(30303); // Timer for Quad Encoder (33Hz)
  Timer2.attachInterrupt(GetAbsEncoderData).start(30303); // Timer for Abs Encoder (33Hz)
}

void loop() 
{
  /*
  // Print out quadrature encoder data (Validation)
  if (quadEncoderFlag == 1)
  {
    Serial.print("Quad Encoder Velocity (rad/s): ");
    Serial.println(quadEncoderVel);
    quadEncoderFlag = 0;
  }
  */

  // Print out absolute encoder data (Validation)
  if (absEncoderFlag == 1)
  {
    Serial.print("FR Abs Encoder Position: ");
    Serial.print(absEncCurrentPositionFR);
    Serial.print(" FR Abs Encoder Velocity: ");
    Serial.println(absEncCurrentVelocityFR);
    absEncoderFlag = 0;
  }
  delay(500);
}

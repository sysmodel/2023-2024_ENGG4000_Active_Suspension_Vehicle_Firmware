/* 
* Sys-MoDEL Active Suspension QuadEncoder_M.cpp file
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Jan 6, 2024
* Last Update: Mar 6, 2024
*
* Version 1.0
*
* Description: 
* This code is the Quad_Encoder.cpp file that is used to get data from the quadrature encoder. 
* Each retrieval of data is done by executing an internal ISR on the Arduino DUE. 
*
* Functions & Descriptions: 
* Name: GetQuadEncoderSpeed()
* Description: Gets the angular speed of the quad encoder (rad/s)
*
* References:
*
*/

#include "QuadEncoder.h"
#include "Encoder.h"

// Define Variables
int encoderPinA = 23; // Channel A on quadrature encoder
int encoderPinB = 22; // Channel B on quadrature encoder
int32_t quadEncoderLastCount = 0;
int32_t quadEncoderCount = 0;
unsigned long quadEncoderLastTime = micros();
unsigned long quadEncoderTime = 0;
volatile double quadEncoderVel = 0.0;
int ppr = 2048;

// Define Encoder Object
Encoder QuadEncoder(encoderPinA, encoderPinB);

// Function to calculate the speed of the quadrature encoder
double GetQuadEncoderSpeed() {  
   // Calculate encoder speed
   quadEncoderCount = QuadEncoder.read();
   quadEncoderTime = micros();

   double quadEncoderVel = (double(quadEncoderCount - quadEncoderLastCount) * 2 * PI) / (double((quadEncoderTime - quadEncoderLastTime) * ppr * 1e-6)); 

   // Update lastCount and lastTime for the next calculation
   quadEncoderLastCount = quadEncoderCount;
   quadEncoderLastTime = quadEncoderTime;

   return quadEncoderVel;  
}


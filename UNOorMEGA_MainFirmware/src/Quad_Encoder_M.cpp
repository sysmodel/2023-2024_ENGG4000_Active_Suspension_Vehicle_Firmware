/* 
* Sys-MoDEL Active Suspension Quad_Encoder.cpp file
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Jan 6, 2024
* Last Update: Jan 11, 2024
*
* Version 1.0
*
* Description: 
* This code is the Quad_Encoder.cpp file that is used to get data from the quadrature encoder. 
* Each retrieval of data is done by executing an internal ISR on the Arduino DUE. 
*
* Functions & Descriptions: 
* Name: 
* Description: 
*
* References:
*
*/

#include "Quad_Encoder_M.h"
#include "Encoder.h"

// Define Variables
int pinA = 2; // Channel A on quadrature encoder
int pinB = 3; // Channel B on quadrature encoder
int32_t quadEncoderLastCount = 0;
int32_t quadEncoderCount = 0;
unsigned long quadEncoderLastTime = micros();
unsigned long quadEncoderTime = 0;
double quadEncoderVel = 0;
int ppr = 2048;

// Define Encoder Object
Encoder encoder(pinA, pinB);

// Function to calculate the speed of the quadrature encoder
double GetQuadEncoderSpeed() {  // Change return type to double
   // Calculate encoder speed
   quadEncoderCount = encoder.read();
   quadEncoderTime = micros();

   double quadEncoderVel = (double(quadEncoderCount - quadEncoderLastCount) * 2 * PI) / (double((quadEncoderTime - quadEncoderLastTime) * ppr * 1e-6)); 

   // Update lastCount and lastTime for the next calculation
   quadEncoderLastCount = quadEncoderCount;
   quadEncoderLastTime = quadEncoderTime;

   return quadEncoderVel;  // Return the calculated value
}



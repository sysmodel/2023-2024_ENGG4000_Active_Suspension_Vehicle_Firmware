/* 
* Sys-MoDEL Active Suspension QuadEncoder_M.h file
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Jan 6, 2024
* Last Update: Mar 6, 2024
*
* Version 1.0
*
* Description: 
* This code is the Quad_Encoder.h file that is used to get data from the quadrature encoder. 
* Each retrieval of data is done by executing an internal ISR on the Arduino DUE. 
*
* Functions & Descriptions: 
* Name: GetQuadEncoderSpeed()
* Description: Gets the angular speed of the quad encoder (rad/s)
*
* References:
*
*/

#ifndef QuadEncoder_H
#define QuadEncoder_H

#include <Arduino.h>
#include "Encoder.h"
// Define global variable to store quad encode speed
extern volatile double quadEncoderVel;

// Declare Function Prototypes
double GetQuadEncoderSpeed();

#endif
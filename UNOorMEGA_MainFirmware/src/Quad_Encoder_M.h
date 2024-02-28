/* 
* Sys-MoDEL Active Suspension Quad_Encoder_M.h file
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Jan 6, 2024
* Last Update: Jan 11, 2024
*
* Version 1.0
*
* Description: 
* This code is the Quad_Encoder.h file that is used to get data from the quadrature encoder. 
* Each retrieval of data is done by executing an internal ISR on the Arduino DUE. 
*
* Functions & Descriptions: 
* Name: 
* Description: 
*
* References:
*
*/
#ifndef Quad_Encoder_M_H
#define Quad_Encoder_M_H

#include <Arduino.h>
#include "Encoder.h"

// Declare Function Prototypes
double GetQuadEncoderSpeed();

#endif
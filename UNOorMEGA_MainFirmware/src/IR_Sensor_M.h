/* 
* Sys-MoDEL Active Suspension IR_Sensor.h file
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Jan 6, 2024
* Last Update: Jan 11, 2024
*
* Version 1.0
*
* Description: 
* This code is the IR_Sensor.h file that is used to calculate the vertical distance of the vehicle.  
*
* Functions & Descriptions: 
* Name: 
* Description: 
*
* References:
*
*/

#ifndef IR_Sensor_M_H
#define IR_Sensor_M_H

#include <Arduino.h>
#include "Adafruit_VCNL4040.h"

// Declare global sensor object
extern Adafruit_VCNL4040 VCNL4040;

// Declare function prototypes
void InitializeVCNL4040();
uint16_t GetProximity();

#endif
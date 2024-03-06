/* 
* Sys-MoDEL Active Suspension Abs_Encoder.h file
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Jan 6, 2024
* Last Update: March 6, 2024
*
* Version 1.0
*
* Description: 
* This code is the Abs_Encoder.h file that is used to get data from the absolute encoder. 
* Each retrieval of data is done by executing an internal ISR on the Arduino DUE. 
*
* Functions & Descriptions: 
* Name: AbsEncPosition()
* Description: Gets the absolute encoder's position **Need to determine if we need angular or linear position**
*
* Name: AbsEncoderVelocity()
* Description: Gets the absolute encoder's velocity in rad/s
*
* References:
*
*/

#ifndef ABSOLUTEENCODER_H
#define ABSOLUTEENCODER_H

#include <Arduino.h>

class AbsoluteEncoder {
public:
    AbsoluteEncoder(uint8_t sckPin, uint8_t csPin, uint8_t sdoPin, uint8_t resolution);
    uint16_t AbsEncPosition();
    double AbsEncoderVelocity();

private:
    uint8_t sckPin;
    uint8_t csPin;
    uint8_t sdoPin;
    uint8_t resolution;
    int16_t absEncoderPosition;
    int16_t absEncoderCurrentPosition;
    unsigned long absEncoderTime;
    unsigned long absEncoderPreviousTime;
};

#endif

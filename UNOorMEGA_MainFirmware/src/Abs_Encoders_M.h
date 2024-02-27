/* 
* Sys-MoDEL Active Suspension Abs_Encoder.h file
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Jan 6, 2024
* Last Update: Jan 11, 2024
*
* Version 1.0
*
* Description: 
* This code is the Abs_Encoder.h file that is used to get data from the absolute encoder. 
* Each retrieval of data is done by executing an internal ISR on the Arduino DUE. 
*
* Functions & Descriptions: 
* Name: 
* Description: 
*
* References:
*
*/

#ifndef Abs_Encoders_M_H
#define Abs_Encoders_M_H

#include <Arduino.h>

class AMT232Encoder {
public:
    AMT232Encoder(int dataPin, int clockPin, int csPin);

    int getAbsEncoderPosition();  // Changed name
    double getAbsEncoderVelocity();  // Changed name

private:
    int _dataPin;
    int _clockPin;
    int _csPin;

    // Private methods for lower-level communication
    uint16_t getPositionSSI(uint8_t resolution);
    // ... other private methods as needed
};

#endif

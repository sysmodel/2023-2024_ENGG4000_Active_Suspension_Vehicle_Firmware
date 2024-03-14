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

class AbsEnc
{
    public: 
        // Default constructor 
        AbsEnc() {} // do not use 

        // Actual constructor
        AbsEnc(uint8_t sckPin, uint8_t csPin, uint8_t sdoPin, uint8_t resolution);

        // Initialize pins
        void initAbsEnc();

        // Get position 
        uint16_t AbsEncPos();

        //Get velocity
        double AbsEncVel();
        

    private:
        uint8_t _sckPin;
        uint8_t _csPin;
        uint8_t _sdoPin;
        uint8_t _resolution;
        uint32_t _varNOP = 0;

        // NOP delay function
        void DelayNOP();
        
};

#endif 

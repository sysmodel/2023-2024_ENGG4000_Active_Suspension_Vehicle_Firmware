/* 
* Sys-MoDEL Active Suspension VCNL4040.h file
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Jan 6, 2024
* Last Update: March 6, 2024
*
* Version 1.0
*
* Description: 
*
* Functions & Descriptions: 
* Name: 
* Description: 
*
* Name: 
* Description: 
* References:
*
*/

#include <Arduino.h>
#include <Adafruit_VCNL4040.h>

class VCNL4040IR
{
    public: 
        // Cosntructor 
        VCNL4040IR(); 

        // Destructor 
        ~VCNL4040IR();

        // Begin
        bool Begin();

        // Get distance function
        double GetDistance();

    private:
        // Create object
        Adafruit_VCNL4040 *_vcnl4040;

        // Variables
        double _factor = 78.122;
        double _exponent = -0.563;
        double _rawReading;
};
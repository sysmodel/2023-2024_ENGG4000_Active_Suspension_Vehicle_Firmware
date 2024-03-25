/* 
* Sys-MoDEL Active Suspension Abs_Encoder.cpp file
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Jan 6, 2024
* Last Update: March 6, 2024
*
* Version 1.0
*
* Description: 
* This code is the Abs_Encoder.cpp file that is used to get data from the absolute encoder. 
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

#include "AbsEncoders.h"

// Define Variables
int32_t absEncoderPreviousPosition = 0;
int32_t absEncoderPosition = 0;
unsigned long absEncoderLastTime = micros();
unsigned long absEncoderTime = 0;
volatile double absEncoderVel = 0.0;

// Actual constructor
AbsEnc::AbsEnc(uint8_t sckPin, uint8_t csPin, uint8_t sdoPin, uint8_t resolution)
{
    _sckPin  = sckPin;
    _csPin = csPin;
    _sdoPin = sdoPin;
    _resolution = resolution;

    initAbsEnc();
};

// Initialize pins
void AbsEnc::initAbsEnc()
{
    pinMode(_sckPin, OUTPUT);
    pinMode(_csPin, OUTPUT);
    pinMode(_sdoPin, INPUT);
}

// Set encoder positions 
void AbsEnc::SetEncPositions(int encMinReading, int encMaxReading, bool CW)
{
    _encMinReading = encMinReading;
    _encMaxReading = encMaxReading; 
    _CW = CW;
    if (_CW)
    {
        if(_encMinReading > encMaxReading)
        {
            _switchPoint = (_encMinReading - _encMaxReading) / 2;
        }
        else
        {
            _switchPoint = (_encMaxReading - 4095 + _encMinReading) / 2;
        }
    }
    else 
    {
        if (_encMinReading > encMaxReading)
        {
            _switchPoint = (_encMaxReading - 4095 + _encMinReading) / 2;
        }
        else 
        {
           _switchPoint = (_encMaxReading + _encMinReading) / 2; 
        }
    }
    
}
uint16_t AbsEnc::AbsEncPos()
{
    noInterrupts();
    uint8_t i, j; //we'll use these incrementers
    uint16_t currentPosition = 0;
    uint8_t _clockCounts = _resolution + 2; //the AMT23 includes 2 additional bits in the response that are used as checkbits
    bool binaryArray[_clockCounts]; //we'll read each bit one at a time and put in array. SSI comes out reversed so this helps reorder
    bool bitHolder; //this variable holds the current bit in our read loop
    bool checkBit0, checkBit1; //the frist two bits in the position response are checkbits used to check the validity of the position response
    _varNOP = 0; 

    //drop cs low and wait the minimum required time. This is done with NOPs
    digitalWrite(_csPin, LOW);
    for (i = 0; i < 26; i++) DelayNOP();

    //We will clock the encoder the number of times (resolution + 2), incrementing with 'j'
    //note that this method of bit-banging doesn't give a reliable clock speed.
    //in applications where datarate is important, the Arduino is not the best solution unless you
    //can find a way to make the SPI interface work for this protocol format.
    for (j = 0; j < _clockCounts; j++)
    {
        //first we lower the clock line and wait until the pin state has fully changed
        digitalWrite(_sckPin, LOW);
        for (i = 0; i < 26; i++) DelayNOP();

        //now we go high with the clock. no need to wait with NOPs because the pin read we'll do next times sufficient time
        digitalWrite(_sckPin, HIGH);
        for (i = 0; i < 26; i++) DelayNOP();

        //Grab the data off of the SDO line and place it into the binary array
        binaryArray[j] = digitalRead(_sdoPin);
    }
    //release cs line, position has been fully received
    digitalWrite(_csPin, HIGH);

    //now we'll reverse the order of the binary array so that the bit ordering matches binary
    for (i = 0, j = _clockCounts - 1; i < (_clockCounts / 2); i++, j--)
    {
        bitHolder = binaryArray[i];
        binaryArray[i] = binaryArray[j];
        binaryArray[j] = bitHolder;
    }

    //create uint16_t from binary array by masking and bit shifting
    for (i = 0; i < _clockCounts - 2; i++) currentPosition |= binaryArray[i] << i;

    //grab check bits in highest bit slots
    checkBit1 = binaryArray[_clockCounts - 1];
    checkBit0 = binaryArray[_clockCounts - 2];

    //use the checkbit equation from the ATM23 datasheet
    if (!(checkBit1 == !(binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
        && (checkBit0 == !(binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
    {
        currentPosition = 0xFFFF; //bad pos, return 0xFFFF which is not a valid value
    }

    interrupts();
    return currentPosition;
    
}

double AbsEnc::GetRackPosition()
{
    if (_encMinReading == NULL)
    {
        Serial.println("SetEncPositions Not Called!");
        exit(0);
    }

    _currentPosition = AbsEncPos();

    if (_CW)
    {
        if (_encMaxReading > _encMinReading)
        {
            return double(_currentPosition - _encMinReading) * _pulseToDistance;
        }
        else 
        {
            if (_currentPosition < _switchPoint)
            {
                return double((4095.0 - _encMinReading) + _currentPosition) * _pulseToDistance;
            }
            else 
            {
                return double(_currentPosition - _encMinReading) * _pulseToDistance;
            }
        }
    }
    else 
    {
        if (_encMinReading > _encMaxReading)
        {
            return double(_encMinReading - _currentPosition) * _pulseToDistance;
        }
        else
        {
            if (_currentPosition < _switchPoint)
            {
                return double( _encMinReading - _currentPosition) * _pulseToDistance;
            }
            else 
            {
                return double(_encMinReading + (4095 - _currentPosition)) * _pulseToDistance;
            }
        }
    }
} 


double AbsEnc::AbsEncVel() 
{
    absEncoderPosition = AbsEncPos();
    absEncoderTime = micros();

    double deltaPosition = absEncoderPosition - absEncoderPreviousPosition;

    /* Need to convert position to anglular or linear result */

    double deltaTime = (absEncoderTime - absEncoderLastTime) / 1000;
    double velocity = deltaPosition / deltaTime; 

    absEncoderPreviousPosition = absEncoderPosition;
    absEncoderLastTime = absEncoderTime;

    return velocity;
}

void AbsEnc::DelayNOP()
{
    _varNOP ++;
}


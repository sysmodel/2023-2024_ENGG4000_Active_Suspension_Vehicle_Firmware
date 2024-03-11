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

#define NOP __asm__ __volatile__ ("nop\n\t")

// Define Variables
int32_t absEncoderPreviousPosition = 0;
int32_t absEncoderPosition = 0;
unsigned long absEncoderLastTime = micros();
unsigned long absEncoderTime = 0;
volatile double absEncoderVel = 0.0;

// Object constructor
AbsEnc::AbsEnc(uint8_t sckPin, uint8_t csPin, uint8_t sdoPin, uint8_t resolution)
    : sckPin(sckPin), csPin(csPin), sdoPin(sdoPin), resolution(resolution) {
}

uint16_t AbsEnc::AbsEncPos() 
{
    uint8_t i, j; //we'll use these incrementers
    uint16_t currentPosition;
    uint8_t _clockCounts = resolution + 2; //the AMT23 includes 2 additional bits in the response that are used as checkbits
    bool binaryArray[_clockCounts]; //we'll read each bit one at a time and put in array. SSI comes out reversed so this helps reorder
    bool bitHolder; //this variable holds the current bit in our read loop
    bool checkBit0, checkBit1; //the frist two bits in the position response are checkbits used to check the validity of the position response

    //drop cs low and wait the minimum required time. This is done with NOPs
    digitalWrite(csPin, LOW);
    for (i = 0; i < 5; i++) NOP;

    //We will clock the encoder the number of times (resolution + 2), incrementing with 'j'
    //note that this method of bit-banging doesn't give a reliable clock speed.
    //in applications where datarate is important, the Arduino is not the best solution unless you
    //can find a way to make the SPI interface work for this protocol format.
    for (j = 0; j < _clockCounts; j++)
    {
        //first we lower the clock line and wait until the pin state has fully changed
        digitalWrite(sckPin, LOW);
        for (i = 0; i < 10; i++) NOP;

        //now we go high with the clock. no need to wait with NOPs because the pin read we'll do next times sufficient time
        digitalWrite(sckPin, HIGH);

        //Grab the data off of the SDO line and place it into the binary array
        binaryArray[j] = digitalRead(sdoPin);
    }
    //release cs line, position has been fully received
    digitalWrite(csPin, HIGH);

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

    return currentPosition;
}

double AbsEnc::AbsEncVel() 
{
    absEncoderPosition = AbsEncPos();
    absEncoderTime = micros();

    double deltaPosition = absEncoderPosition - absEncoderPreviousPosition;
    double deltaTime = (absEncoderTime - absEncoderLastTime) / 1000.0;
    double velocity = deltaPosition / deltaTime; 

    absEncoderPreviousPosition = absEncoderPosition;
    absEncoderLastTime = absEncoderTime;

    return velocity;
}


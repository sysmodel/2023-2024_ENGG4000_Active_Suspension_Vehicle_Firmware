/* 
* Sys-MoDEL Active Suspension Abs_Encoder.cpp file
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Jan 6, 2024
* Last Update: Jan 11, 2024
*
* Version 1.0
*
* Description: 
* This code is the Abs_Encoder.cpp file that is used to get data from the absolute encoder. 
* Each retrieval of data is done by executing an internal ISR on the Arduino DUE. 
*
* Functions & Descriptions: 
* Name: 
* Description: 
*
* References:
*
*/

#include "Abs_Encoders_M.h"

#define carriageReturn  0x0D
#define newLine         0x0A
#define tab             0x09

#define OFF             0
#define ON              1

#define SSI_CS          7
#define SSI_SCK         10
#define SSI_SDO         11

#define res12           12
#define res14           14

AMT232Encoder::AMT232Encoder(int dataPin, int clockPin, int csPin)
    : _dataPin(dataPin), _clockPin(clockPin), _csPin(csPin) {
    pinMode(_dataPin, INPUT);
    pinMode(_clockPin, OUTPUT);
    pinMode(_csPin, OUTPUT);
}

int AMT232Encoder::getAbsEncoderPosition() {  // Changed name
    uint8_t i, j; // we'll use these incrementers
    uint16_t currentPosition;
    uint8_t _clockCounts = 12 + 2; //the AMT232 includes 2 additional bits in the response that are used as checkbits
    bool binaryArray[_clockCounts]; //we'll read each bit one at a time and put in array. SSI comes out reversed so this helps reorder
    bool bitHolder; //this variable holds the current bit in our read loop
    bool checkBit0, checkBit1; //the frist two bits in the position response are checkbits used to check the validity of the position response

    //drop cs low and wait the minimum required time. This is done with NOPs
    digitalWrite(SSI_CS, LOW);
    for (i = 0; i < 5; i++) {
        // NOP; // Original code used NOPs, you can replace with a delay if preferred
        delayMicroseconds(1);  // Delay for 1 microsecond
    }

    //We will clock the encoder the number of times (resolution + 2), incrementing with 'j'
    //note that this method of bit-banging doesn't give a reliable clock speed.
    //in applications where datarate is important, the Arduino is not the best solution unless you
    //can find a way to make the SPI interface work for this protocol format.
    for (j = 0; j < _clockCounts; j++) {
        //lower the clock line and wait until the pin state has fully changed
        digitalWrite(SSI_SCK, LOW);
        delayMicroseconds(1);  // Delay for 1 microsecond

        //now we go high with the clock. no need to wait with NOPs because the pin read we'll do next times sufficient time
        digitalWrite(SSI_SCK, HIGH);
        
        //Grab the data off of the SDO line and place it into the binary array
        binaryArray[j] = digitalRead(SSI_SDO);
    }
    //release cs line, position has been fully received
    digitalWrite(SSI_CS, HIGH);

    //now we'll reverse the order of the binary array so that the bit ordering matches binary
    for (i = 0, j = _clockCounts - 1; i < (_clockCounts / 2); i++, j--) {
        bitHolder = binaryArray[i];
        binaryArray[i] = binaryArray[j];
        binaryArray[j] = bitHolder;
    }

    //create uint16_t from binary array by masking and bit shifting
    for (i = 0; i < _clockCounts - 2; i++) currentPosition |= binaryArray[i] << i;

    //grab the highest two bits and put them into the checkbit holders
    if (!(checkBit1 == !(binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
        && (checkBit0 == !(binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
      {
        currentPosition = 0xFFFF; //bad pos, return 0xFFFF which is not a valid value
      }
    return getPositionSSI(12);  // Assuming 12-bit resolution
}

double AMT232Encoder::getAbsEncoderVelocity() {  // Changed name
    // Implement logic to calculate angular velocity based on position changes and time
    // (Replace with your specific algorithm)
    // Example:
    int currentPosition = getAbsEncoderPosition();  // Changed name
    static int previousPosition = 0;
    unsigned long currentTime = millis();
    static unsigned long previousTime = 0;

    double deltaPosition = currentPosition - previousPosition;
    double deltaTime = (currentTime - previousTime) / 1000.0;
    double velocity = deltaPosition / deltaTime; // Assuming revolutions per second

    previousPosition = currentPosition;
    previousTime = currentTime;

    return velocity;
}

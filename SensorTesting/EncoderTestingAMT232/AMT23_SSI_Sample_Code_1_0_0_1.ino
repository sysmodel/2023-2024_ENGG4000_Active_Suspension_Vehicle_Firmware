/*
 * AMT23_SSI_Sample_Code.ino
 * Company: CUI Inc.
 * Author: Jason Kelly
 * Version: 1.0.0.1
 * Date: February 14, 2020
 * 
 * This sample code can be used with the Arduino Mega to control the AMT22 encoder.
 * It uses SSI to control the encoder and the the Arduino UART to report back to the PC
 * via the Arduino Serial Monitor. Code can be modified for any Arduino including the UNO.
 * The pins are the same on the Arduino UNO but you will need to make sure to change the board
 * type in the Tools Menu.
 * For more information or assistance contact CUI Inc for support.
 * 
 * After uploading code to Arduino Mego open the open the Serial Monitor under the Tools 
 * menu and set the baud rate to 115200 to view the serial stream the position from the AMT23.
 * 
 * Arduino Pin Connections
 * SSI Chip Select: Pin 7
 * SSI Clock (SCK): Pin 10
 * SSI Data (SDO):  Pin 11
 * 
 * 
 * AMT23 Pin Connections
 * Vdd (5V):              Pin 1
 * SSI DATA (SDO):        Pin 2
 * SSI CLOCK (SCK):       Pin 3
 * GND:                   Pin 4
 * Mode (unconnnected:    Pin 5
 * SSI Chip Select:       Pin 6
 * 
 * 
 * This is free and unencumbered software released into the public domain.
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 * 
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

/* NOP */
#define NOP __asm__ __volatile__ ("nop\n\t")

/* Define special ascii characters */
#define carriageReturn  0x0D
#define newLine         0x0A
#define tab             0x09

/* boolean */
#define OFF             0
#define ON              1

/* pins */
#define SSI_CS          7
#define SSI_SCK         10
#define SSI_SDO         11

/* available resolutions */
#define res12           12
#define res14           14

/* Serial */
#define baudRate        115200

void setup() 
{
  //pin modes for AMT23 SSI
  pinMode(SSI_SCK, OUTPUT);
  pinMode(SSI_CS, OUTPUT);
  pinMode(SSI_SDO, INPUT);

  //serial port for debugging
  Serial.begin(baudRate);
}

void loop() 
{
  //we don't need to redeclare variables so they can go here
  //and then we can have our own infinite loop below
  uint16_t encoderPosition; //holder for encoder position
  uint8_t attempts; //we can use this for making sure position is valid
  
  while(1)
  {
    //set attemps counter at 0 so we can try again if we get bad position    
    attempts = 0;

    //this function gets the encoder position and returns it as a uint16_t
    //send the function either res12 or res14 for your encoders current resolution
    encoderPosition = getPositionSSI(res12); 
    while (encoderPosition == 0xFFFF && attempts++ < 3)
    {
      delay(1);
      encoderPosition = getPositionSSI(res12); //try again
    }

    Serial.print(encoderPosition, DEC); //print the position
    Serial.write(newLine);
    delay(500);
  }
}

/*
 * Use this function to understand how the SSI protocol works first, then you can use the function
 * below that has better efficiency.
 */
uint16_t getPositionSSI(uint8_t resolution)
{
  uint8_t i, j; //we'll use these incrementers
  uint16_t currentPosition;
  uint8_t _clockCounts = resolution + 2; //the AMT23 includes 2 additional bits in the response that are used as checkbits
  bool binaryArray[_clockCounts]; //we'll read each bit one at a time and put in array. SSI comes out reversed so this helps reorder
  bool bitHolder; //this variable holds the current bit in our read loop
  bool checkBit0, checkBit1; //the frist two bits in the position response are checkbits used to check the validity of the position response

  //drop cs low and wait the minimum required time. This is done with NOPs
  digitalWrite(SSI_CS, LOW);
  for (i = 0; i < 5; i++) NOP;

  //We will clock the encoder the number of times (resolution + 2), incrementing with 'j'
  //note that this method of bit-banging doesn't give a reliable clock speed.
  //in applications where datarate is important, the Arduino is not the best solution unless you
  //can find a way to make the SPI interface work for this protocol format.
  for (j = 0; j < _clockCounts; j++)
  {
    //first we lower the clock line and wait until the pin state has fully changed
    digitalWrite(SSI_SCK, LOW);
    for (i = 0; i < 10; i++) NOP;

    //now we go high with the clock. no need to wait with NOPs because the pin read we'll do next times sufficient time
    digitalWrite(SSI_SCK, HIGH);
    
    //Grab the data off of the SDO line and place it into the binary array
    binaryArray[j] = digitalRead(SSI_SDO);
  }
  //release cs line, position has been fully received
  digitalWrite(SSI_CS, HIGH);

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
  if (resolution == res12) //if we're in 12-bit
  {
    if (!(checkBit1 == !(binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
        && (checkBit0 == !(binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
    {
      currentPosition = 0xFFFF; //bad pos, return 0xFFFF which is not a valid value
    }
  }
  else if (resolution == res14) //if we're in 14-bit
  {
    if (!(checkBit1 == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
        && (checkBit0 == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
    {
      currentPosition = 0xFFFF; //bad pos, return 0xFFFF which is not a valid value
    }
  }
  
  return currentPosition;
}

/*
 * This function is a more efficient version of the getPositionSSI() function
 * that strips out some of the bit reordering and improves the speed of calculating the checkbit.
 * Once the SSI protocol is fully understood, this function will be more useful in an end application.
 */
uint16_t getPositionSSI_efficient(uint8_t resolution)
{
  uint8_t i, j; //we'll use these incrementers
  uint8_t odd, even; //bit parity counters
  uint16_t currentPosition = 0;
  uint8_t _clockCounts = resolution + 2; //the AMT23 includes 2 additional bits in the response that are used as checkbits
  uint8_t checkBit1, checkBit0; //the frist two bits in the position response are checkbits used to check the validity of the position response

  //drop cs low and wait the minimum required time. This is done with NOPs
  digitalWrite(SSI_CS, LOW);
  for (i = 0; i < 5; i++) NOP;

  //We will clock the encoder the number of times (resolution + 2), incrementing with 'j'
  //note that this method of bit-banging doesn't give a reliable clock speed.
  //in applications where datarate is important, the Arduino is not the best solution unless you
  //can find a way to make the SPI interface work for this protocol format.
  for (j = 0; j < _clockCounts; j++)
  {
    //first we lower the clock line and wait until the pin state has fully changed
    digitalWrite(SSI_SCK, LOW);
    for (i = 0; i < 10; i++) NOP;

    //now we go high with the clock. no need to wait with NOPs because the pin read we'll do next times sufficient time
    digitalWrite(SSI_SCK, HIGH);
    
    //throw the pin value into the position, note that it's reversing it as well
    currentPosition |= (digitalRead(SSI_SDO) << (_clockCounts - j - 1));
  }
  //release cs line, position has been fully received
  digitalWrite(SSI_CS, HIGH);


  //grab the highest two bits and put them into the checkbit holders
  checkBit1 = (currentPosition >> (_clockCounts - 1)) & 0x01;
  checkBit0 = (currentPosition >> (_clockCounts - 2)) & 0x01;

  //at this point currentPosition still holds the checkbits. So if we're in 14 bit mode, there's 16 bits
  //we only move up 14 bits here (resolution) because we're only tallying up the 1's in the position
  //we're counting the bits in even slots and the ones in odd slots
  for (uint8_t i = 0; i < resolution; i++) (i % 2 == 0) ? even += ((currentPosition >> i) & 0x01) : odd += ((currentPosition >> i) & 0x01);

  //check the counts against the checkbits
  if ((checkBit1 == odd % 2) || (checkBit0 == even % 2)) currentPosition = 0xFFFF;
  else 
  {
    //this isn't the 'fastest' since we're introducting an if/else but doing
    // currentPosition &= 2^resolution; doesn't work because arduino has a problem with
    // powers.
    if (resolution == res12) currentPosition &= 0xFFF;
    else if (resolution == res14) currentPosition &= 0x3FFF;
  }

  return currentPosition;
}

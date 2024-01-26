// Drok Motor Testing

int enA = 9;
int in1 = 53;
int in2 = 52;
 
void setup()
 
{
  Serial.begin(19200);
  // Set all the motor control pins to outputs
 
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

 
}
 
void demoOne()
 
{
 
  // This function will run the motors in both directions at a fixed speed
 
  // Turn on motor A
 
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
 
  // Set speed to 200 out of possible range 0~255
 
  analogWrite(enA, 200);
 
  delay(2000);
 
  // Now change motor directions
 
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);  
 
  delay(2000);
 
  // Now turn off motors
 
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
 
}
 
void demoTwo()
 
{
 
  // This function will run the motors across the range of possible speeds
  // Note that maximum speed is determined by the motor itself and the operating voltage
 
  // Turn on motors
 
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);  
 
  // Accelerate from zero to maximum speed
 
  for (int i = 0; i < 256; i++)
 
  {
 
    analogWrite(enA, i);
 
    delay(20);
 
  } 
 
  // Decelerate from maximum speed to zero
 
  for (int i = 255; i >= 0; --i)
 
  {
 
    analogWrite(enA, i);
 
    delay(20);
 
  } 
 
  // Now turn off motors
 
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  

 
}
 
void loop()
 
{
 
  Serial.println("Demo One");
  demoOne();
 
  delay(1000);
 
  Serial.println("Demo Two");
  demoTwo();
 
  delay(1000);
 
}
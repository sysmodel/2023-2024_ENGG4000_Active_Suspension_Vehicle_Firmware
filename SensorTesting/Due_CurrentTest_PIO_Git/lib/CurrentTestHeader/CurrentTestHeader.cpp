#include "CurrentTestHeader.h"

//------------------------------------------------------------------

ADS1115 ADS(0x48);
Adafruit_INA260 ina260 = Adafruit_INA260();

double currentOffset;
double current; // non-filtered current
int csAnalog; // current sensor analog
static float f = ADS.toVoltage(1);  // voltage factor
static float currentSum = 0;
static int offsetWindow = 1000;
static int sensVVal;
static int scaledVolt;
float voltage;
static long int nowtime = 0;
static long int lasttime = 0;
double fcurrent; // filtered current
long readingSG;
HX711 scale;  // object for strain gauge
const int LOADCELL_DOUT_PIN = 12;
const int LOADCELL_SCK_PIN = 13;

int pwmOffset;
int pwm;
float timeConstant = 0.06; // time constant in s
double setIPID, currentPID, outPID; // define PID variable
PID myPID(&currentPID, &outPID, &setIPID, Kp, Ki, Kd, DIRECT);

String direction = "CW"; // can be CW or CCW

float csVolt;
static float shuntResis = 0.013333333;
static int16_t val,val0,val2;
static float csAn2,csAn0;
// static float dividerRatio = 0.3125;
static float dividerRatio = 0.066;

float currentINA;
float currentOffsetINA;

//------------------------------------------------------------------

void InitStuff() {
  Serial.begin(115200);

  pinMode(sensV, INPUT);
  pinMode(mdEn, OUTPUT);
  pinMode(mdIn1, OUTPUT);
  pinMode(mdIn2, OUTPUT);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  Wire.begin();
  // ADS.begin();
  // ADS.setDataRate(7);
  // ADS.setGain(0);

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  myPID.SetMode(AUTOMATIC);

  if (!ina260.begin()) {
    Serial.println("Couldn't find INA260 chip");
    while (1);
  }
  Serial.println("Found INA260 chip");
  ina260.setAveragingCount(INA260_COUNT_4);
  ina260.setCurrentConversionTime(INA260_TIME_8_244_ms);

  // CalibrateCurrent();
}

void GetVoltage() {
  sensVVal = analogRead(sensV);
  scaledVolt = map(sensVVal, 0, 1023, 0, 3300);
  voltage = float(scaledVolt) * 0.005;  // in volts
}

void GetCurrent(double currOff) {
  // val0 = ADS.readADC(0);
  // val2 = ADS.readADC(2);
  // val2 = ADS.readADC_Differential_2_3();
  val = ADS.readADC(ADCCurrent);
  csAnalog = ADS.toVoltage(val) * 1000.0;
  // csAn2 = ADS.toVoltage(val2) * 1000.0;
  // csVolt = csAn2/dividerRatio;
  current = csAnalog/dividerRatio/1000 - currOff;
  // if (direction == "CW") {
  //   current = ((float(csAnalog)/1000.0)/dividerRatio)/shuntResis;
  // } else if (direction == "CCW") {
  //   current = (voltage*float(pwm)/255.0-float(csAnalog)/1000.0)/dividerRatio/shuntResis;
  // }
  // current = double(csAnalog) - currentOffset;
}

void CalibrateCurrent() {
  analogWrite(mdEn, 0);
  currentSum = 0;
  for (int i = 0; i < offsetWindow; i++) {
    GetCurrent(0);
    currentSum += current;
  }
  currentOffset = currentSum / double(offsetWindow);
  Serial.print("Current offset: ");Serial.println(currentOffset,3);
}

void GetFilteredCurrent() {
  // double current = GetCurrent(fcurrentOffset);
  GetCurrent(currentOffset);
  lasttime = nowtime;
  nowtime = millis();
  fcurrent = fcurrent + (current - fcurrent) * double(nowtime - lasttime) / double(1000 * timeConstant);
  // fcurrent = fcurrent / 1000.0;
}

void SetDirec(String dir) {
  if (dir == "CW") {
    digitalWrite(mdIn1, LOW);
    digitalWrite(mdIn2, HIGH);
  } else if (dir == "CCW") {
    digitalWrite(mdIn1, HIGH);
    digitalWrite(mdIn2, LOW);
  }
}

void GetCurrentINA(float offset) {
  currentINA = (float(ina260.readCurrent())/1000.0 - offset)*1.14 + 0.02;
}

void CalibrateCurrentINA() {
  analogWrite(mdEn, 0);
  currentSum = 0;
  for (int j = 0; j < offsetWindow; j++) {
    GetCurrentINA(0.0);
    currentSum += float(currentINA);
    delay(5);
  }
  currentOffsetINA = currentSum / float(offsetWindow);
  Serial.print("Current offset: ");Serial.println(currentOffsetINA,3);
}
#include <Protocentral_FDC1004.h>
#include "RTClib.h"
#include <Timer.h>
#define SETLEVEL 20
#define LOWTIME 10
#define HIGHTIME 15
Timer timer;
RTC_DS3231 rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

#include "Wire.h"
#define UPPER_BOUND  0X4000                 // max readout capacitance 
#define LOWER_BOUND  (-1 * UPPER_BOUND)
#define CHANNEL 0                          // channel to be read
#define MEASURMENT 0
FDC1004 FDC;

int capdac = 0;
int outletpin1 = 12;//31 Higher value pins for Arduino MEGA
int outletpin2 = 11; //33
int outletpin3 = 10;//35
int inletpin1 = 9;//37
int inletpin2 = 8;//39
int pumpPin = 7;//41
int trigPin = 2;//51,  violet
int echoPin = 4;//53,  blue
int buzzerPin = 6;//13

float h_cap = 0;
float h_cond = 0;
float h_ultra = 0;
float h = 0;
boolean pumpStatus = LOW;
void setup() {
  timer.every(3 * 1000, setRandom);
  pinMode(inletpin1, OUTPUT);
  pinMode(inletpin2, OUTPUT);
  pinMode(outletpin3, OUTPUT);
  pinMode(outletpin1, OUTPUT);
  pinMode(outletpin2, OUTPUT);
  pinMode(pumpPin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(outletpin1, HIGH);
  digitalWrite(outletpin2, HIGH);
  digitalWrite(outletpin3, HIGH);
  Serial.begin(115200);
  Wire.begin();
#ifndef ESP8266
  while (!Serial); // for Leonardo/Micro/Zero
#endif
  delay(3000); // wait for console opening


  if (rtc.lostPower()) {
    //Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

}


void loop()
{
  timer.update();
  digitalWrite(inletpin1, LOW);
  digitalWrite(inletpin2, LOW);
  DateTime now = rtc.now();
  h_ultra = ultrasonic();
  //h_cond = conducSensorValue();
  h_cap = CapDist();

  h = 0.7*h_ultra + 0.1*h_cap + 0.2*h_cond; //Sensor Fusion
  
  Serial.println(h);
  
  if(h>=25)
    digitalWrite(buzzerPin,HIGH);
   else digitalWrite(buzzerPin,LOW);
  if(h<=10){
    digitalWrite(pumpPin,!HIGH);
    digitalWrite(inletpin1,!HIGH);
    digitalWrite(inletpin2,!HIGH);
    pumpStatus = HIGH;
  }
  else if(h>10 && pumpStatus == HIGH){
    digitalWrite(pumpPin,!HIGH);
    digitalWrite(inletpin1,!HIGH);
    digitalWrite(inletpin2,!HIGH);
  }
  else if(h>SETLEVEL){
    if(pumpStatus == HIGH){
      delay(2000);
    }
    digitalWrite(pumpPin,!LOW);
    digitalWrite(inletpin1,!LOW);
    digitalWrite(inletpin2,!LOW);
    pumpStatus = LOW;
  }
  else if(h>10 && h<(SETLEVEL-2) && pumpStatus == LOW){
    if(now.hour() >0 && now.hour()<4){
      digitalWrite(pumpPin,!HIGH);
    digitalWrite(inletpin1,!HIGH);
    digitalWrite(inletpin2,!HIGH);
    }
  }
}

int conducSensorValue(void) {
  byte inputA = 0;
  byte inputB = 0;
  Wire.beginTransmission(32);
  Wire.write(0x12); // set MCP23017 memory pointer to GPIOA address
  Wire.endTransmission();
  Wire.requestFrom(0x20, 1); // request one byte of data from MCP20317
  inputA = Wire.read(); // store the incoming byte into "inputA"
  Wire.beginTransmission(32);

  Wire.write(0x13); // set MCP23017 memory pointer to GPIOB address
  Wire.endTransmission();
  Wire.requestFrom(0x20, 1); // request one byte of data from MCP20317
  inputB = Wire.read(); // store the incoming byte into "inputB"

  boolean level[10];
  level[9] = inputB & 0b00000010; //Get level status for 9
  level[8] = inputB & 0b00000001; //Get level status for 8
  for (byte i = 0; i < 8; i++) { //Get level status for 0-7
    level[i] = inputA & (1 << i);
  }

  int height = -3;
  for (int i = 9; i >= 0; i--) { //Calulate height in cm
    if (level[i] == 0)
      height += 3;
  }
  delay(20);
  return height;
}

int CapDist()
{
  FDC.configureMeasurementSingle(MEASURMENT, CHANNEL, capdac);
  FDC.triggerSingleMeasurement(MEASURMENT, FDC1004_100HZ);

  //wait for completion
  delay(15);
  uint16_t value[2];
  if (! FDC.readMeasurement(MEASURMENT, value))
  {
    int16_t msb = (int16_t) value[0];
    int32_t capacitance = ((int32_t)457) * ((int32_t)msb); //in attofarads
    capacitance /= 1000;   //in femtofarads
    capacitance += ((int32_t)3028) * ((int32_t)capdac);
    int c1 = capacitance / 1000;
    c1 = (c1 - 25.68) / 2.282;    //Calculate distance
    if (msb > UPPER_BOUND)               // adjust capdac accordingly
    {
      if (capdac < FDC1004_CAPDAC_MAX)
        capdac++;
    }
    else if (msb < LOWER_BOUND)
    {
      if (capdac > 0)
        capdac--;
    }
    delay(20);
    return (c1);
  }

}

float ultrasonic() {
  long t = 0;
  float d = 0;
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(20);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(20);
  digitalWrite(trigPin, LOW);

  pinMode(echoPin, INPUT);
  t = pulseIn(echoPin, HIGH);

  d = t / 58.0;
  d = 31.5 - d;
  delay(100);
  return d;
}

void setRandom() {
  int i;
  byte state;
  byte d1, d2, d3;
  state = random(0, 255);
  d1 = bitRead(state, 1);
  d2 = bitRead(state, 2);
  d3 = bitRead(state, 3);
  digitalWrite(outletpin1, d1);
  digitalWrite(outletpin2, d2);
  digitalWrite(outletpin3, d2);
}



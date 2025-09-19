
// Basic syntax: setup() runs once, loop() repeats. Use pinMode, digitalWrite, delay, etc.
// Libraries: 16_-18_ (LCD: LiquidCrystal_I2C.h), 19_-20_ (DHT: DHT.h), 23_/25_ (DS18B20: OneWire.h, DallasTemperature.h), 27_ (Servo: Servo.h). Others use core functions.

// 01_.ino - Simple LED Blink on Pin 2
short led = 2;
void setup() {
  pinMode(led, OUTPUT);
}
void loop() {
  digitalWrite(led, HIGH);
  delay(1000);
  digitalWrite(led, LOW);
  delay(1000);
}
// End 01_.ino

// =============================================================================
// 02_.ino - Two LEDs Blinking with Function
int L1=4, L2=2;
void setup() {
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
}
void loop() {
  pin=8; // Note: 'pin' undeclared; declare 'int pin;' if needed
  LED_BLINK(L2, 5, 50);
  LED_BLINK(L1, 5, 50);
}
void LED_BLINK(int pin, int turn, int Delay) {
  for (int i=0; i<turn; i++) {
    digitalWrite(pin, HIGH);
    delay(Delay);
    digitalWrite(pin, LOW);
    delay(Delay);
  }
}
// End 02_.ino

// =============================================================================
// 03_.ino - 8 LEDs Forward and Backward
byte L1=2, L2=3, L3=4, L4=5, L5=6, L6=7, L7=8, L8=9;
int Delay = 50;
void setup() {
  for (int i=0; i<8; i++) {
    pinMode((i+2), OUTPUT);
  }
}
void loop() {
  forward();
  backward();
}
void forward() {
  for (byte i=0; i<8; i++) {
    digitalWrite((i+2), HIGH);
    delay(Delay);
    digitalWrite((i+2), LOW);
  }
}
void backward() {
  for (byte i=6; i>0; i--) {
    digitalWrite((i+2), HIGH);
    delay(Delay);
    digitalWrite((i+2), LOW);
  }
}
// End 03_.ino

// =============================================================================
// 04_.ino - Switch Control LED (No Debounce)
byte SW=11, LED=13;
bool State=0;
void setup() {
  pinMode(SW, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
}
void loop() {
  State = digitalRead(SW);
  if (State == 0) {
    digitalWrite(LED, HIGH);
    delay(1000);
  } else {
    digitalWrite(LED, LOW);
  }
}
// End 04_.ino

// =============================================================================
// 05_.ino - Switch with Debounce
byte SW=11, LED=13;
bool f1=0, State=0;
void setup() {
  pinMode(SW, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
}
void loop() {
  if (!digitalRead(SW)) {
    delay(100);
    if (!digitalRead(SW)) {
      while (!digitalRead(SW)) {}
      f1 = !f1;
    }
  }
  digitalWrite(LED, f1);
}
// End 05_.ino

// =============================================================================
// 06_.ino - Binary Value on 8 LEDs (Incrementing)
byte value = 0;
void setup() {
  for (int i=0; i<8; i++) {
    pinMode(i, OUTPUT);
  }
  Serial.begin(9600);
  value = 0;
}
void loop() {
  if (value==255) { // Note: 'vlaue' typo in original
    value=0;
  }
  for (short j=0; j<8; j++) {
    digitalWrite(j, value & (1 << j));
  }
  delay(500);
  value++;
}
// End 06_.ino

// =============================================================================
// 07_.ino - Display Fixed Value on 8 Pins
unsigned char pins[]={0,1,2,3,4,5,6,7};
int num=65;
void diplayValue(int Value) {
  for (short j=0; j<8; j++) {
    digitalWrite(pins[j], Value & (1 << j));
  }
}
void setup() {
  for (int i=0; i<8; i++) {
    pinMode(pins[i], OUTPUT);
  }
}
void loop() {
  diplayValue(num);
  delay(1000);
}
// End 07_.ino

// =============================================================================
// 08_.ino - Count Up/Down on 8 LEDs
unsigned char pins[]={0,1,2,3,4,5,6,7};
unsigned char num=0;
bool up_d_ch=0;
void diplayValue() {
  for (short j=0; j<8; j++) {
    digitalWrite(pins[j], num & (1 << j));
  }
  if (up_d_ch==0) {
    num++;
  } else {
    num--;
  }
  if (num==0 || num==15) {
    up_d_ch = !up_d_ch;
  }
}
void setup() {
  for (int i=0; i<8; i++) {
    pinMode(pins[i], OUTPUT);
  }
}
void loop() {
  diplayValue();
  delay(500);
}
// End 08_.ino

// =============================================================================
// 09_.ino - 7-Segment Display Fixed Value
unsigned char pins[]={2,3,4,5,6,7,8,9};
unsigned char digits[]={10,11,12,13};
unsigned char num=65;
bool dir=0;
void diplayValue(byte val) {
  for (short j=0; j<8; j++) {
    digitalWrite(pins[j], val & (1 << j));
  }
}
void setup() {
  for (int i=0; i<8; i++) {
    pinMode(pins[i], OUTPUT);
  }
  for (int i=0; i<4; i++) {
    pinMode(digits[i], OUTPUT);
  }
  digitalWrite(10, HIGH);
}
void loop() {
  diplayValue(num);
  delay(500);
}
// End 09_.ino

// =============================================================================
// 10_.ino - 4-Digit 7-Segment Counter with Button
unsigned char SSDpins[]={2,3,4,5,6,7,8,9};
unsigned char digits[]={10,11,12,13};
int unum=0;
byte SW=A0;
unsigned char num[]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f};
bool dir=0;
byte digi_val[4]={};
void diplayValue(byte val) {
  for (short j=0; j<8; j++) {
    digitalWrite(SSDpins[j], val & (1 << j));
  }
}
void seg_val(int val) {
  digi_val[0] = val % 10;
  digi_val[1] = val / 10 % 10;
  digi_val[2] = val / 100 % 10;
  digi_val[3] = val / 1000;
}
void diplay_on_SSD(int dnum) {
  seg_val(dnum);
  for (byte z=0; z<4; z++) {
    digitalWrite(digits[z], HIGH);
    diplayValue(num[digi_val[z]]);
    delay(5);
    digitalWrite(digits[z], LOW);
  }
}
void setup() {
  for (int i=0; i<8; i++) {
    pinMode(SSDpins[i], OUTPUT);
  }
  for (int i=0; i<4; i++) {
    pinMode(digits[i], OUTPUT);
  }
  for (byte k=10; k<14; k++) {
    digitalWrite(k, LOW);
  }
  pinMode(SW, INPUT);
}
void loop() {
  diplay_on_SSD(unum);
  if (!digitalRead(SW)) {
    delay(100);
    while (!digitalRead(SW)) {}
    unum++;
  }
}
// End 10_.ino

// =============================================================================
// 11_.ino - Serial Control LED (Buggy)
byte number=0;
char recievedChar;
String IncomingData;
void setup() {
  Serial.begin(19200);
  pinMode(13, OUTPUT);
}
void loop() {
  if (Serial.available() > 0) {
    recievedChar = Serial.read();
    if (IncomingData == 49) { // Bug: IncomingData not updated; use recievedChar == '1'
      digitalWrite(13, HIGH);
      Serial.println("LED is ON");
    }
    if (IncomingData == '0') { // Bug: Same issue
      digitalWrite(13, LOW);
      Serial.println("LED is OFF");
    }
  }
}
// End 11_.ino

// =============================================================================
// 12_.ino - Switch State to Serial
byte SW=7;
void setup() {
  pinMode(SW, INPUT_PULLUP);
  Serial.begin(9600);
}
void loop() {
  if (!digitalRead(SW)) {
    delay(50);
    if (!digitalRead(SW)) {
      Serial.println("Device is ON");
    }
  } else {
    Serial.println("Device is OFF");
  }
  delay(500);
}
// End 12_.ino

// =============================================================================
// 13_.ino - Serial Command LED Control
byte SW=7;
String CMD;
void setup() {
  pinMode(SW, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  Serial.begin(9600);
}
void loop() {
  if (Serial.available()) {
    CMD = Serial.readString();
    if (CMD == "ON") {
      digitalWrite(13, HIGH);
    }
    if (CMD == "OFF") {
      digitalWrite(13, LOW);
    }
  }
}
// End 13_.ino

// =============================================================================
// 13_T.ino - Switch State to Serial (Simplified)
byte SW=7;
void setup() {
  Serial.begin(9600);
  pinMode(SW, INPUT_PULLUP);
}
void loop() {
  if (!digitalRead(SW)) {
    delay(50);
    if (!digitalRead(SW)) {
      Serial.print("ON");
    }
  } else {
    Serial.print("OFF");
  }
  delay(100);
}
// End 13_T.ino

// =============================================================================
// 14_.ino - Serial PWM Control
String u_in;
void setup() {
  pinMode(9, OUTPUT);
  Serial.begin(9600);
}
void loop() {
  if (Serial.available() > 0) {
    analogWrite(9, Serial.parseInt());
  }
  delay(500);
}
// End 14_.ino

// =============================================================================
// 15_.ino - Analog Input to PWM LED
unsigned char analogpin=A0;
int ADCval=0;
float Voltage=0.0;
byte LED_pin=9;
short Brightness=0;
void setup() {
  Serial.begin(9600);
  pinMode(LED_pin, OUTPUT);
}
void loop() {
  ADCval = analogRead(analogpin);
  delay(10);
  Brightness = (ADCval/1023.0)*255.0;
  Serial.println(Brightness);
  analogWrite(LED_pin, Brightness);
  delay(2);
}
// End 15_.ino

// =============================================================================
// 16_.ino - LCD Scroll Text (Needs LiquidCrystal_I2C.h)
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2);
String msg = "PH 2024 - Electronics and Computing Laboratory";
void setup() {
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Ayubowan..!!");
}
void loop() {
  lcd.setCursor(0,0);
  lcd.print("Ayubowan..!!");
  for (byte pos=0; pos<15; pos++) {
    lcd.scrollDisplayLeft();
    delay(250);
  }
  for (byte pos=0; pos<15; pos++) {
    lcd.scrollDisplayRight();
    delay(250);
  }
  lcd.noAutoscroll();
  delay(2000);
  lcd.clear();
}
// End 16_.ino

// =============================================================================
// 17_.ino - LCD Counter (Needs LiquidCrystal_I2C.h)
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2);
void setup() {
  lcd.init();
  lcd.backlight();
  lcd.setCursor(6,0);
  lcd.print("Count!");
}
void loop() {
  for (byte i=0; i<11; i++) {
    lcd.clear();
    lcd.setCursor(6,0);
    lcd.print("Count!");
    lcd.setCursor(8,1);
    lcd.print(i);
    delay(1000);
  }
}
// End 17_.ino

// =============================================================================
// 18_.ino - LCD Voltage Monitor (Needs LiquidCrystal_I2C.h)
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2);
unsigned char analogpin=A0;
short ADCval=0;
double Voltage=0.0;
double thv=2.5;
void setup() {
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
}
void loop() {
  ADCval = analogRead(analogpin);
  delay(10);
  Voltage = ADCval * (4.8 * 0.001);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Voltage: ");
  lcd.print(Voltage);
  lcd.print("V");
  lcd.setCursor(0,1);
  if (Voltage > thv) {
    lcd.print("ALERT:Over Voltage");
  } else {
    lcd.print("Threshold: ");
    lcd.print(thv);
  }
  delay(3000);
}
// End 18_.ino

// =============================================================================
// 19_.ino - DHT11 Sensor with LCD (Needs DHT.h, LiquidCrystal_I2C.h)
#include "DHT.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2);
#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
void setup() {
  Serial.begin(9600);
  dht.begin();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
}
void loop() {
  delay(2000);
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float f = dht.readTemperature(true);
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  lcd.clear();
  lcd.print("Temp:");
  lcd.print(t);
  lcd.print((char)223);
  lcd.print("C");
  lcd.setCursor(0,1);
  lcd.print("Humidity:");
  lcd.print(h);
  lcd.print("%");
}
// End 19_.ino

// =============================================================================
// 20_.ino - DHT11, Ultrasonic, LCD Switcher (Needs DHT.h, LiquidCrystal_I2C.h)
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
LiquidCrystal_I2C lcd(0x27,16,2);
#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
byte status=0;
byte button=4;
byte LED=5;
const byte Trig=9;
const byte Echo=10;
long duration=0;
int distance=0;
short LED_W=0;
void setup() {
  Serial.begin(9600);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(button, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  dht.begin();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
}
void loop() {
  if (!digitalRead(button)) {
    delay(50);
    while (!digitalRead(button)) {}
    if (status==2) {
      status=0;
    } else {
      status++;
    }
    lcd.clear();
  }
  switch (status) {
    case 0: lcd.setCursor(0,0); Temp(); break;
    case 1: lcd.setCursor(0,0); Hum(); break;
    case 2: lcd.setCursor(0,0); DIS(); break;
  }
  delay(100);
}
void DIS() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  duration = pulseIn(Echo, HIGH);
  distance = duration*0.034/2;
  lcd.clear();
  lcd.print("Distance: ");
  lcd.print(distance);
  lcd.print("cm");
  lcd.setCursor(0,1);
  lcd.print("                ");
  LED_W=map(distance,5,50,0,255);
  if (distance<5) {
    analogWrite(LED,0);
  } else {
    analogWrite(LED,LED_W);
  }
}
void Temp() {
  lcd.print("Temp:");
  lcd.print(dht.readTemperature());
  lcd.print((char)223);
  lcd.print("C");
}
void Hum() {
  lcd.setCursor(0,0);
  lcd.print("Humidity:");
  lcd.print(dht.readHumidity());
  lcd.print("%");
}
// End 20_.ino

// =============================================================================
// 21_.ino - Interrupt-Based LED Control (Buggy)
byte PL1=2, PL2=3, St_L=4, PL1_LED=5, PL2_LED=6;
byte led=5;
bool PL1_ST=0, PL2_ST=0;
byte SensorD=0;
void setup() {
  pinMode(PL1, INPUT_PULLUP);
  pinMode(PL2, INPUT_PULLUP);
  pinMode(St_L, OUTPUT);
  pinMode(PL1_LED, OUTPUT);
  pinMode(PL2_LED, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PL1), PL1_Win, LOW);
  attachInterrupt(digitalPinToInterrupt(PL2), PL2_Win, LOW);
  digitalWrite(St_L, HIGH); // Note: 'digitalWriite' typo in original
  delay(5000);
  digitalWrite(St_L, LOW);
}
void loop() {
  // Wait for interrupts
}
void LED_Con() {
  digitalWrite(led, !digitalRead(button)); // Note: 'button' undeclared
}
void PL1_Win() {
  digitalWrite(PL1_LED, HIGH);
  noInterrupts();
}
void PL2_Win() {
  digitalWrite(PL2_LED, HIGH);
  noInterrupts();
}
// End 21_.ino

// =============================================================================
// 22_.ino - EEPROM Read
#include <EEPROM.h>
int value=0;
void setup() {
  Serial.begin(9600);
}
void loop() {
  value = EEPROM.read(0);
  Serial.println(value);
  delay(2000);
}
// End 22_.ino

// =============================================================================
// 23_Dallas_18B20.ino - DS18B20 Temp Sensor (Needs OneWire.h, DallasTemperature.h)
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
void setup() {
  Serial.begin(9600);
  sensors.begin();
}
void loop() {
  sensors.requestTemperatures();
  delay(1500);
  float tempC = sensors.getTempCByIndex(0);
  if (tempC != DEVICE_DISCONNECTED_C) {
    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.println(" °C");
  } else {
    Serial.println("Error: Could not read temperature data");
  }
}
// End 23_Dallas_18B20.ino

// =============================================================================
// 24_millis.ino - Non-Blocking LED Blink with millis()
byte L1=4, L2=5;
bool st1=0, st2=0;
int interval1=1000, interval2=300;
unsigned long T1=0, T2=0, T3=0, T4=0;
void setup() {
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  Serial.begin(9600);
}
void loop() {
  T2=T4=millis();
  if ((T2-T1) >= interval1) {
    st1 = !st1;
    digitalWrite(L1, st1);
    T1 = millis();
  }
  if ((T4-T3) >= interval2) {
    st2 = !st2;
    digitalWrite(L2, st2);
    T3 = millis();
  }
}
// End 24_millis.ino

// =============================================================================
// 25_.ino - DS18B20 and Ultrasonic (Needs OneWire.h, DallasTemperature.h)
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
const byte Trig=9;
const byte Echo=10;
long duration=0;
int distance=0;
int int1=1000, int2=5000;
unsigned long T1=0, T2=0, T3=0;
void setup() {
  Serial.begin(9600);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  sensors.begin();
  T1=T2=millis();
}
void loop() {
  T3=millis();
  if ((T3-T1) >= int1) {
    DISTANCE();
    T1=millis();
  }
  if ((T3-T2) >= int2) {
    temp();
    T2=millis();
  }
}
void DISTANCE() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  duration = pulseIn(Echo, HIGH);
  distance = duration*0.034/2;
  Serial.print(distance);
  Serial.println("cm");
}
void temp() {
  sensors.requestTemperatures();
  delay(100);
  float tempC = sensors.getTempCByIndex(0);
  if (tempC != DEVICE_DISCONNECTED_C) {
    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.println(" °C");
  } else {
    Serial.println("Error: Could not read temperature data");
  }
}
// End 25_.ino

// =============================================================================
// 26_DC_Motor_L298N.ino - DC Motor Control with L298N
#define F1 5
#define B1 6
#define F2 9
#define B2 10
byte ENA=3, ENB=11;
byte POT=A0;
unsigned char speed1=70, speed2=200;
unsigned long T1=0, T2=0;
byte dir=4;
bool st=0;
void setup() {
  pinMode(F1, OUTPUT);
  pinMode(B1, OUTPUT);
  pinMode(F2, OUTPUT);
  pinMode(B2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(POT, INPUT);
  pinMode(dir, INPUT_PULLUP);
  T1=T2=millis();
}
void loop() {
  if (!digitalRead(dir)) {
    delay(50);
    while (!digitalRead(dir)) {}
    st = !st;
  }
  speed1 = map(analogRead(POT),0,1024,0,255);
  if (st==0) {
    FWD(speed1);
  } else {
    BWD(speed1);
  }
}
byte FWD(byte sp) {
  digitalWrite(F1, HIGH);
  digitalWrite(B1, LOW);
  digitalWrite(F2, HIGH);
  digitalWrite(B2, LOW);
  analogWrite(ENA, sp);
  analogWrite(ENB, sp);
}
byte BWD(byte sp) {
  digitalWrite(F1, LOW);
  digitalWrite(B1, HIGH);
  digitalWrite(F2, LOW);
  digitalWrite(B2, HIGH);
  analogWrite(ENA, sp);
  analogWrite(ENB, sp);
}
// End 26_DC_Motor_L298N.ino

// =============================================================================
// 27_Servo_with_POT.ino - Servo Control with Potentiometer (Needs Servo.h)
#include <Servo.h>
Servo myservo;
int pos=0;
byte POT=A0;
void setup() {
  myservo.attach(9,500,2400);
  pinMode(POT, INPUT);
}
void loop() {
  pos = map(analogRead(POT),0,1024,0,180);
  myservo.write(pos);
  delay(100);
}
// End 27_Servo_with_POT.ino

// End of Cheat Sheet - Good Luck!
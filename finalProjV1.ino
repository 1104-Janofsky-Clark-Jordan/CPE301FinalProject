//Author: Rhea Janofsky-Clark
//NSHE: 8001589797
//Assignment: CPE301 Final Project

#include <LiquidCrystal.h>
#include <Stepper.h>
#include <RTClib.h>
#include <DHT.h> 

RTC_DS1307 rtc;

#define DHT11_PIN 50
#define DHTTYPE DHT11

DHT dht(DHT11_PIN, DHTTYPE);

#define RDA 0x80
#define TBE 0x20  
#define INT1S 16000000/1024

const int steps = 150;
Stepper stepMotor = Stepper(steps, 17, 18, 20, 21);
LiquidCrystal lcd(2, 3, 4, 5, 6, 7);

const int signal = A2;

const int onbutton = 8;
const int offbutton = 9;
const int onPin = 10;
const int offPin = 11;
const int power = 12;
const int motorOn = 31;
const int motorIn1= 32;
const int motorIn2 = 33;
const int idle = 35;
const int running = 37;
const int disable = 39;
const int error = 41;
const int waterMinimum = 130;
bool waterLow = false;
bool prevWater = true;

// these variables are to stop the idle/error messages and leds from blinking on and off
bool error1 = false;
bool idle1 = false;

bool systemEnabled = false;
bool prevSys = false;

unsigned long lastPrint = 0;

unsigned long currentTime = 0;

void displayTime() {
  DateTime CT = rtc.now();
  U0putchar('C');U0putchar('u');U0putchar('r');U0putchar('r');U0putchar('e');U0putchar('n');U0putchar('t');
  U0putchar(' ');
  U0putchar('D');U0putchar('a');U0putchar('t');U0putchar('e');U0putchar('/');U0putchar('T');U0putchar('i');U0putchar('m');U0putchar('e');
  U0putchar(' ');
  
  int dayNum[2];
  int day = CT.day();
  dayNum[0] = day/10;
  dayNum[1] = day%10;
  for (int i=0; i<2; i++){
    U0putchar(dayNum[i]);
  }

  int monthNum[2];
  int month = CT.month();
  monthNum[0] = month/10;
  monthNum[1] = month%10;
  for (int i=0; i<2; i++){
    U0putchar(monthNum[i]);
  }

  int yearNum[4];
  int year = CT.year();
  yearNum[0] = year/1000;
  yearNum[1] = 10%(year/100);
  yearNum[2] = 10%(year/10);
  yearNum[3] = year%1000;
  for (int i=0; i<4; i++){
    U0putchar(yearNum[i]);
  }
  
  int hourNum[2];
  int hour = CT.hour();
  hourNum[0] = hour/10;
  hourNum[1] = hour%10;
  for (int i=0; i<2; i++){
    U0putchar(hourNum[i]);
  }

  int minNum[2];
  int min = CT.minute();
  minNum[0] = min/10;
  minNum[1] = min%10;
  for (int i=0; i<2; i++){
    U0putchar(minNum[i]);
  }

  int secNum[2];
  int sec = CT.second();
  secNum[0] = sec/10;
  secNum[1] = sec%10;
  for (int i=0; i<2; i++){
    U0putchar(secNum[i]);
  }
}

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
 
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

volatile unsigned char *myTCCR1B  = 0x81;
volatile unsigned int  *myTCNT1   = 0x84;
volatile unsigned long currentTicks = 0; // Global variable to track timer ticks
volatile unsigned int* portB = 0x25;

void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}
unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void U0init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}
unsigned char U0getchar()
{
  return *myUDR0;
}
void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

ISR(myInterrupt)
{
  *myTCCR1B &= ~0x07;  // Disable the timer (clear CS12, CS11, CS10)
  *myTCNT1 = (unsigned int)(65535 - (unsigned long)(currentTicks));  // Set counter value
  *myTCCR1B |= 0x03;  // Enable the timer with a prescaler of 64 (CS11 and CS10)

  if (currentTicks != 65535)
  {
    *portB ^= 0x40;  // Toggle pin connected to PB6
  }
}

void printTH(){
  U0putchar('T');U0putchar('e');U0putchar('m');U0putchar('p');U0putchar('e');U0putchar('r');U0putchar('a');U0putchar('t');U0putchar('u');U0putchar('r');U0putchar('e');U0putchar(':');
  U0putchar(' ');
  char temp[6];
  char hum[6]; 
  sprintf(temp, "%dC\n", dht.readTemperature()); 
  sprintf(hum, "%d%%\n", dht.readHumidity()); 
  int tempLen = strlen(temp);
  for (int i = 0; i < tempLen; i++) {
    U0putchar(temp[i]);
  }
  U0putchar('H');U0putchar('u');U0putchar('m');U0putchar('i');U0putchar('d');U0putchar('i');U0putchar('t');U0putchar('y');U0putchar(':');
  U0putchar(' ');
  int humLen = strlen(hum);
  for (int i = 0; i < humLen; i++) {
    U0putchar(hum[i]);
  }
}


void setup() {
  rtc.begin();
  lcd.begin(16, 2);
  lcd.clear();
  U0init(9600);
  pinMode(motorOn, OUTPUT);
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(error, OUTPUT);
  pinMode(idle, OUTPUT);
  pinMode(running, OUTPUT);
  pinMode(disable, OUTPUT);
  pinMode(onPin, INPUT);
  pinMode(offPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(onPin), myInterrupt, RISING);
  pinMode(onbutton, INPUT_PULLUP);
  pinMode(offbutton, INPUT_PULLUP);
  pinMode(power, OUTPUT);
  digitalWrite(power, LOW);
}

void loop() {
  int onState = digitalRead(onPin);
  int offState = digitalRead(offPin);
  if (onState == HIGH){
    systemEnabled = HIGH;
    prevSys = LOW;
  } else if (offState == HIGH){
    systemEnabled = LOW;
    prevSys = HIGH;
  } else {
    systemEnabled = LOW;
    prevSys = LOW;
  }
  if (systemEnabled){
    digitalWrite(disable, LOW);
    int onChk = digitalRead(onbutton);
    int offChk = digitalRead(offbutton);
    digitalWrite(power, HIGH);
    int sensVal = analogRead(signal);
    digitalWrite(power, LOW);
    if (sensVal < waterMinimum) {
      waterLow = true;
    } else {
      waterLow = false;
    }

    if (waterLow){
      currentTime = millis();
      if (currentTime - lastPrint >= 60000) {
        printTH();
        displayTime(); 
        lastPrint = currentTime;
      }
      prevWater = waterLow;
      digitalWrite(idle, LOW);
      digitalWrite(error, HIGH);
      if (!error1){
        const char waterLowMess [] = "Water Level is low.";
        for (int i = 0; waterLowMess[i] != '\0'; i++) {
          U0putchar(waterLowMess[i]); 
        }
        U0putchar('\n');
        displayTime();
        error1 = true;
      }
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Error: Low");
      lcd.setCursor(0, 1);
      lcd.print("Water Level");
      idle1 = false;
    } else if (waterLow != prevWater) {
      if (waterLow) {
        const char waterLowMess [] = "Water Level is low";
        for (int i = 0; waterLowMess[i] != '\0'; i++) {
          U0putchar(waterLowMess[i]); 
        }
        U0putchar('\n');
        displayTime();
      } else {
        const char waterNormMess [] = "Water Level is normal";
        for (int i = 0; waterNormMess[i] != '\0'; i++) {
          U0putchar(waterNormMess[i]); 
        }
        U0putchar('\n');
        displayTime();
      }
      prevWater = waterLow;
    } else {
      digitalWrite(error, LOW);
      currentTime = millis();
      if (currentTime - lastPrint >= 60000) {
        printTH();
        displayTime(); 
        lastPrint = currentTime;
      }
      digitalWrite(idle, HIGH);
      int onChk = digitalRead(onbutton);
      int offChk = digitalRead(offbutton);
      if (onChk == LOW && offChk == HIGH) {
        stepMotor.setSpeed(50);
        stepMotor.step(-steps);
        const char ventMess1 [] = "Rotated vent counterclockwise.";
        for (int i = 0; ventMess1[i] != '\0'; i++) {
          U0putchar(ventMess1[i]); 
        }
        U0putchar('\n');
        displayTime();
      } else if (onChk == HIGH && offChk == LOW) {
        stepMotor.setSpeed(50);
        stepMotor.step(steps);
        const char ventMess2 [] = "Rotated vent clockwise.";
        for (int i = 0; ventMess2[i] != '\0'; i++) {
          U0putchar(ventMess2[i]); 
        }
        U0putchar('\n');
        displayTime();
      }
      int runBefore = 0;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(dht.readTemperature());
      lcd.print(" C");
      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(dht.readHumidity());
      lcd.print("%");
      while(!waterLow && (dht.readTemperature() >= 20 || dht.readHumidity() >= 60)){ 
        currentTime = millis();
        if (currentTime - lastPrint >= 60000){
          printTH();
          displayTime(); 
          lastPrint = currentTime;
        }
        idle1 = false;
        if (!runBefore){
          const char coolMess [] = "Cooling started";
          for (int i = 0; coolMess[i] != '\0'; i++) {
            U0putchar(coolMess[i]); 
          }
          U0putchar('\n');
          displayTime();
          runBefore = true;
        }

        digitalWrite(running, HIGH);
        digitalWrite(idle, LOW);
        digitalWrite(motorIn1, HIGH);
        digitalWrite(motorIn2, LOW);
        digitalWrite(motorOn, HIGH);
    
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Temp: ");
        lcd.print(dht.readTemperature());
        lcd.print(" C");

        lcd.setCursor(0, 1);
        lcd.print("Humidity: ");
        lcd.print(dht.readHumidity());
        lcd.print("%");

        digitalWrite(power, HIGH);
        int sensVal = analogRead(signal);
        digitalWrite(power, LOW);
        if (sensVal < waterMinimum) {
            waterLow = true;
        } else {
            waterLow = false;
        }
        if (waterLow || (dht.readTemperature() <= 20 && dht.readHumidity() <= 60)) {
          digitalWrite(running, LOW);
          digitalWrite(motorIn1, LOW);
          digitalWrite(motorIn2, LOW);
          digitalWrite(motorOn, LOW);
        }
      }
      runBefore = false;
    }
  }
  else {
    lcd.clear();
    digitalWrite(disable, HIGH);
  }
}
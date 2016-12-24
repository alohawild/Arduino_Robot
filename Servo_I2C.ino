
/*****************************************************************

  //************************************************************
  // Servo Control using Adafruit i2C contoller                *
  // 6 Dec 2016                                                *
  // Use Serial LCD for display                                *
  // See http://www.arduino.cc/playground/Learning/SparkFunSerLCD
  // Michael R. Wild                                           *
  //************************************************************

Hardware is using AdaFruit PCA9685 Servo Controller in A4 and A5.
Sparkfun Serial Display using softserial is on D10.
Pot is on A0 for adusting sweep.

Serial is to report status as is the LCD screen.
Put four servos in PCA 9685.

*****************************************************************/

/* Based on sketch from Ladyada so note below kept */
/* This is based on servo setch                    */

/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/*****************************************************************/
// Use software serial to allow use of two or more connections.
// This is transmit only so should work.

#include <SoftwareSerial.h>

//************************************************************************
// Controlling values for Servo

Adafruit_PWMServoDriver pwmServo = Adafruit_PWMServoDriver(); // use default x40 setting

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
uint8_t servonum = 0;

//************************************************************************
// House keeping

char  version_code[ ]   = "1.0";
char  project_name[ ]   = "Servo Testing";

char printBuffer[50];

//************************************************************************
// Used to track nob setting

int nobPin = 0;  // A0 for reading
int nobValue = 0;  // Value for read
int nobLast = -1; // Previous value
int nobRange = 0; // Mapped for range of sweep

//************************************************************************
// Pins 3 for display using Sparkfun serial display using only one pin

#define txPin 3
#define LCDMax 16

SoftwareSerial LCD = SoftwareSerial(0, txPin);
const int LCDdelay = 10;

#define RefreshRate 1024

//for printing
char message0_Buffer[LCDMax + 1] = "                ";
char message1_Buffer[LCDMax + 1] = "                ";
//                                  1234567890123456

//********************************
//* Serial LCD                   *
//********************************

void lcdPosition(int col, int row) {
  LCD.write(0xFE);   //command flag
  LCD.write((col + row * 64 + 128));  //position
  delay(LCDdelay);
}
void clearLCD() {
  LCD.write(0xFE);   //command flag
  LCD.write(0x01);   //clear command.
  delay(LCDdelay);
}
void backlightOn() {  //turns on the backlight
  LCD.write(0x7C);   //command flag for backlight stuff
  LCD.write(157);    //light level.
  delay(LCDdelay);
}
void backlightOff() { //turns off the backlight
  LCD.write(0x7C);   //command flag for backlight stuff
  LCD.write(128);     //light level for off.
  delay(LCDdelay);
}
void serCommand() {  //a general function to call the command flag for issuing all other commands
  LCD.write(0xFE);
}

void setup() {
  Serial.begin(9600);
  Serial.println(project_name);

  //*************************************************************************
  // set up the LCD's number of columns and rows:
  pinMode(txPin, OUTPUT);
  LCD.begin(9600);
  clearLCD();
  backlightOn();
  lcdPosition(0, 0);

  // Print a message to the LCD. This lets us know about a reset
  lcdPosition(0, 0);
  LCD.print(project_name);

  lcdPosition(0, 1);
  LCD.print("Version ");
  LCD.print(version_code);
  delay(5000);

  pwmServo.begin();
  
  pwmServo.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}


void loop() {

  nobValue = analogRead(nobPin);

  if (nobValue != nobLast) {
    nobLast = nobValue;
    nobRange = map(nobValue, 0, 1023, SERVOMIN, SERVOMAX);
  }

  int nobPercent = round((nobValue / 1023.0)*100);
  sprintf(printBuffer, "Setting: %03d %04d", nobPercent, nobRange);
  Serial.println(printBuffer);
  
  // Drive each servo one at a time
  sprintf(printBuffer, "Servo: %03d", servonum);
  Serial.println(printBuffer);

  clearLCD();
  lcdPosition(0, 0);
  sprintf(message0_Buffer,"Servo: %03d", servonum);
  LCD.print(message0_Buffer);

  lcdPosition(0, 1);
  sprintf(message1_Buffer,"Nob: %03d", nobPercent);
  LCD.print(message1_Buffer);

  for (uint16_t pulselen = SERVOMIN; pulselen < nobRange; pulselen++) {
    pwmServo.setPWM(servonum, 0, pulselen);
    }
  //pwmServo.setPWM(servonum, 0, SERVOMAX);
  delay(700);
  for (uint16_t pulselen = nobRange; pulselen > SERVOMIN; pulselen--) {
    pwmServo.setPWM(servonum, 0, pulselen);  }

  delay(700);
  pwmServo.setPWM(servonum, 0, 0);
  
    servonum ++;
    if (servonum > 3) servonum = 0; // Just four as that is enough.
}

/*****************************************************************

  //************************************************************
  // XBee Robot                                                *
  // 10 Dec 2016                                               *
  // Use Serial LCD for display                                *
  // See http://www.arduino.cc/playground/Learning/SparkFunSerLCD
  // 
  // Michael R. Wild                                           *
  //************************************************************

*****************************************************************/
// Use software serial to allow use of display as it is read only

#include <SoftwareSerial.h>

/*****************************************************************/

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

//************************************************************************
// House keeping

char  version_code[ ]   = "0.5";
char  project_name[ ]   = "XBee Mega Motor";
//************************************************************************
// Pins for XBee

#define XBee Serial2

//************************************************************************
// Pins for Motor Contoller

#define MotorCon Serial1

//************************************************************************
// Pins 3 for display using Sparkfun serial display using only one pin

#define txPin 3
#define LCDMax 16

SoftwareSerial LCD = SoftwareSerial(2, txPin);
const int LCDdelay = 10;

#define RefreshRate 1024

//for printing
char message0_Buffer[LCDMax + 1] = "                ";
char message1_Buffer[LCDMax + 1] = "                ";
//                                  1234567890123456

char printBuffer[50];
//************************************************************************
// Controlling values for Servo

Adafruit_PWMServoDriver pwmServo = Adafruit_PWMServoDriver(); // use default x40 setting

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
uint8_t servonum = 0;

//************************************************************************
// Used to track nob setting for servo

#define nobPin 0       // A0 for reading
int nobValue = 0;  // Value for read
int nobLast = 0;  // Previous value
int nobRange = 0;      // Mapped for range of sweep

//************************************************************************
// Commands

#define commandMax 40
char command_string[commandMax + 1] = "123456789012345678902345678901234567890";
int command_len;

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

//********************************
//* CLR Buffer 1                 *
//********************************

void clrBuffer() {
  for (int i=0;i<LCDMax;i++) {
    message1_Buffer[i] = ' ';
  }
}


//********************************
//* Command                      *
//********************************

void clrCommand() {
  command_len = 0;
  for (int i=0;i<commandMax;i++) {
    command_string[i] = ' ';
  }
}

void addCommand(char newChar) {
  if (command_len < commandMax) {
    command_string[command_len] = newChar;
    command_len++;
  }
}

int validCommand() {
// ATR1100
// ATS0100

  
  if ((command_string[0] == 'A') && // old school attention!
      (command_string[1] == 'T')) {

        if ((command_string[2] == 'F') || // Motor Forward
            (command_string[2] == 'S') || // Servo
            (command_string[2] == 'R')  // Motor Reverse

            ) {

          if ((command_string[3] >= '0') && // Hex digit
              (command_string[3] <= '9') ||
              (command_string[3] >= 'A') &&
              (command_string[3] <= 'F')        
              ) {

            if (((command_string[4]>='0') &&
                (command_string[4]<='9')) &&
                ((command_string[5]>='0') &&
                (command_string[5]<='9')) &&
                ((command_string[6]>='0') &&
                (command_string[6]<='9')) 
                ) {
                  return 1;
             }
             else {
              return -4; //bad value
             }
            
          }
          else {
            return -3; //bad parm
          }
               
        }
        else {
          return -2; //bad request
        }
   }
   else {
    return -1; // bad command
   }

}

void motorF(int motorNum, int speedValue) // Motor
{ 

  char commandBuffer[4];
  int speedRange = map(speedValue, 0, 100, 0, 9);
  int motorRange = map(motorNum, 0, 15, 1, 2);
  sprintf(commandBuffer, "%1dF%1d\r", motorRange, speedRange);
  MotorCon.write(commandBuffer[0]);
  MotorCon.write(commandBuffer[1]);
  MotorCon.write(commandBuffer[2]);
  MotorCon.write(commandBuffer[3]);
  motorEcho();

}

void motorR(int motorNum, int speedValue) // Motor
{ 

  char commandBuffer[4];
  int speedRange = map(speedValue, 0, 100, 0, 9);
  int motorRange = map(motorNum, 0, 15, 1, 2);
  sprintf(commandBuffer, "%1dR%1d\r", motorRange, speedRange);
  MotorCon.write(commandBuffer[0]);
  MotorCon.write(commandBuffer[1]);
  MotorCon.write(commandBuffer[2]);
  MotorCon.write(commandBuffer[3]);
  motorEcho();

}

void servo(int ServoNum, int speedValue) // Servo
{ 
  
  int servoRevised = round(SERVOMAX * 0.01 * nobRange); // Revised to range from nob
  int servoRange = map(speedValue, 0, 1023, SERVOMIN, servoRevised); // Set proportional value
 
  sprintf(printBuffer, "Servo %02d Range %3d\r", ServoNum, servoRange); 
  Serial.println(printBuffer);
  
  if (servoRevised <= SERVOMIN) // clear servo
  {
    pwmServo.setPWM(servonum, 0, SERVOMIN);
  } else                      // use servo setting
  {
    for (uint16_t pulselen = SERVOMIN; pulselen < servoRange; pulselen++) {
    pwmServo.setPWM(servonum, 0, pulselen);
    }
  }
 

}




void doCommand() {

  char commandRequest = command_string[2];
  int commandParm = command_string[3]-'0';
  if ( 
    (command_string[3] >= 'A') &&
    (command_string[3] <= 'F')
    )
  {
    commandParm = command_string[3]-'A' + 10;

  }
  int commandValue = (command_string[4]-'0')*100 + (command_string[5]- '0')*10 + 
                   (command_string[6]-'0');
  XBee.println(commandValue);
  XBee.println(commandRequest);
  

  switch(commandRequest){
      
      case 'F': // Move Forward

        motorF(commandParm, commandValue);

        break;

      case 'R': // Move Forward

        motorR(commandParm, commandValue);

        break;


      case 'S': // Move Forward

        servo(commandParm, commandValue);

        break;
     
      default:
        
        break;
    
  }
  
}

void checkNob()

{
  char message_Buffer[LCDMax + 1] = "                ";
  char printBuffer[50];
  
  nobValue = (analogRead(nobPin)) / 8;
  
  if (!(nobLast == nobValue)) {
    
    nobLast = nobValue;
   
    nobRange = map(nobValue, 0, 1023, SERVOMIN, SERVOMAX);
    
    // debounce and turn in to a % 
    int nobPercent = round((nobValue / 1023.0)*100*8);
    sprintf(printBuffer, "Setting: %03d", nobPercent);
    Serial.println(printBuffer);

    // Send message to LCD
    clearLCD();
    lcdPosition(0, 0);
    sprintf(message_Buffer,"Nob: %03d", nobPercent, " ");
    LCD.print(message_Buffer);
  }
}

void readXBee()

{

  char readChar = ' ';
  
  while (XBee.available()>0)
    { // If data comes in from XBee, treat as command

      readChar = XBee.read();
    
      if ((readChar>='0' && readChar<='9') ||
          (readChar>='A' && readChar<='Z') ||
          (readChar>='a' && readChar<='z') ||
          (readChar=='-') ||
          (readChar=='.') ||
          (readChar==':') ||
          (readChar==' ') ||
          (readChar==',')
          )
      {
        for (int i=0;i<LCDMax; i++) {
          message1_Buffer[i]=message1_Buffer[i+1];
        }
        //Xbee.write(readChar);
        message1_Buffer[LCDMax-2] = readChar;
      }
      else {
        readChar = ' '; //force to blank
      }

      if (readChar!=' ') {
        addCommand(readChar);

      }
      else {
        if (command_len>0) {
          int errorCode = validCommand();
          if (errorCode>0) {
            XBee.println("OK");
            lcdPosition(0,0);
            LCD.print("OK              ");
//                     1234567890123456

          doCommand();
          }
          clrCommand();
        
        }
      }
    }


}

void readSerial()

{

  char readChar = ' ';

  while (Serial.available()>0)
    { // If data comes in from XBee, treat as command

      readChar = Serial.read();
    
      if ((readChar>='0' && readChar<='9') ||
          (readChar>='A' && readChar<='Z') ||
          (readChar>='a' && readChar<='z') ||
          (readChar=='-') ||
          (readChar=='.') ||
          (readChar==':') ||
          (readChar==' ') ||
          (readChar==',')
          )
      {
        for (int i=0;i<LCDMax; i++) {
          message1_Buffer[i]=message1_Buffer[i+1];
        }
        //Xbee.write(readChar);
        message1_Buffer[LCDMax-2] = readChar;
      }
      else {
        readChar = ' '; //force to blank
      }

      if (readChar!=' ') {
        addCommand(readChar);

      }
      else {
        if (command_len>0) {
          int errorCode = validCommand();
          if (errorCode>0) {
            Serial.println("OK");
            lcdPosition(0,0);
            LCD.print("OK              ");
//                     1234567890123456

          doCommand();
          }
          clrCommand();
        
        }
      }
    }


}

void motorEcho()

{
  while (MotorCon.available()>0)
  { // If data comes in from Serial1, send it out to serial monitor
      char readChar = MotorCon.read();
      Serial.write(readChar);
  } 
}

void setup()
{
  // confgured each XBee to 19200
  // for the XBee. Make sure the baud rate matches the config
  // where xxxx is a four digit number to define the little network
  // ATIDxxxx,DH0,DL1,MY0,BD4,WR,CN
  // ATIDxxxx,DH0,DL0,MY1,BD4,WR,CN
  // setting of your XBee.

  XBee.begin(19200);

  //*************************************************************************
  // Talking to Serial  
  Serial.begin(9600);

  Serial.println(project_name);
  Serial.print("Version: ");
  Serial.println(version_code);

  //*************************************************************************
  // Motor Control
  MotorCon.begin(115200);
  motorEcho();

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
  delay(1000);
  clearLCD();

  //*************************************************************************
  clrCommand();
  clrBuffer();
  
  //*************************************************************************
  // set up pwm servo control
  pwmServo.begin();
  
  pwmServo.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

}

void loop()
{

  // Check Nob and report changes
  checkNob();

  // Motor controller might be talking
  motorEcho();

  if (XBee.available()>0)
  { // If data comes in from XBee, send it out to serial monitor
    readXBee();
  } else if (Serial.available()>0)
  {
    readSerial();
  }
  lcdPosition(0,1);
  LCD.print(message1_Buffer);
  delay(10);
}


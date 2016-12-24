/*****************************************************************

  //************************************************************
  // Serial Echo Mega                                          *
  // 7 Dec 2016                                                *
  // Michael R. Wild                                           *
  //************************************************************

  Attach serial item to Serial 1 and talk to via Serial


*****************************************************************/
// Use software serial to allow use of two or more connections.

//************************************************************************
// House keeping

char  project_name[ ]   = "Serial Echo Mega 0.1";

void setup()
{

  Serial.begin(9600);

  Serial.write(project_name);

  Serial1.begin(115200);
  
}

void loop()
{

  while (Serial1.available()>0)
  { // If data comes in from Serial1, send it out to serial monitor
    char readChar = Serial1.read();
    Serial.write(readChar);
  } 
  while (Serial.available()>0)
  { // If data comes in from serial, send it out to serial1 
    char readChar = Serial.read();
    Serial1.write(readChar);
  }
}


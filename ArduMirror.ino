/*
***************
* ArduMirror  *
***************

By zoomx

 Control of a mirror with pan and tilt, two relays and one servo.
 Installed at La Montagnola station to control
 FTIR mirror (pan and tilt) and two power sokets.
 A servo controls the camera position.

 was Stepper05SerialControl2
 Added serial control
 two Stepper Motor Control
 Move the motor depending on commands received
 Using two EasyDriver stepper module

 Added Relay control

 Need optimisation to remove doubled functions

 2015 02 25

 2015 05 20

 2015 06 11
 Added Servo control

 2015 09 16
 Added LimitSwitch NOT WORKING!!!

 2016 01 20
 New Limits implementation

 2016 02 16
 Added ResetSteppersLimits and

 SerialInputNewline functions
 http://forum.arduino.cc/index.php?topic=288234.0

2016 02 24
Pan limit now is a real limit to avoid that mirror impact structures
and damage pan limits
Reset take care of all possible start position (I hope!)

2016 02 25
Name changed in ArduMirror, finally
Other few updates
 */

#define LEDPIN 13  //Led is on pin 13 in Arduino UNO boards.

//Tilt
#define STEP1PULSE 9
#define STEP1DIR 8 //High=Down Low=Up
#define STEP1HALF 7
//Pan
#define STEP2PULSE 6
#define STEP2DIR 5 //High=Right Low=Left
#define STEP2HALF 4

#define RELAY1 2
#define RELAY2 10

#define SERVO 11	//PWM works on pins 3, 5, 6, 9, 10, and 11

#define PAN_START_PIN  A4 //front mirror
#define PAN_END_PIN  A3   //behind mirror
#define TILT_START_PIN A1 //front mirror
#define TILT_END_PIN A2   //behind mirror

#define DELAY 20000
#define DELAYM 10

#define SERVOCAMERA_POINT_POSITION 90   //need calibration!!!!
#define SERVOCAMERA_PARK_POSITION 50  //need calibration!!!!

#include <Servo.h>
Servo Servocamera;

//Serial input with termination section
#define INLENGTH 6          //Max string lenght over serial. Needed for input with termination
#define INTERMINATOR 13     //GetCharFromSerial routine cycle waiting for this character. Needed for input with termination

char inString[INLENGTH + 1]; //GetCharFromSerial returns this char array. Needed for input with termination
char comm;  //First character received. Needed for input with termination


//***********
int Distance = 0;  // Record the number of steps we've taken
int angle;
int FrontPosition;		//steps from zero to arrive in front position

//**********
int Pan_Actual_Position;
int Tilt_Actual_Position;

int Pan_End_Position;
int Tilt_End_Position;

int Tilt_Future_Position;
int Tilt_steps;

int Pan_Future_Position;
int Pan_steps;

int Pan_increment = 1;  //Used to increase or decrease the counter
int Tilt_increment = 1;

//SerialInputNewline
const byte numChars = 32;
char receivedChars[numChars];  // an array to store the received data
boolean newData = false;

//***********************************************************************************************
void PrintVersion() {
  Serial.println("ArduMirror");
  Serial.print(F("Version 0.9 "));
  Serial.print(__DATE__);  //this is the compiling date
  Serial.print(F(" "));
  Serial.println(__TIME__); //this is the compiling time
}

//***********************************************************************************************
void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = 13;   //'\r';
  char rc;

  // if (Serial.available() > 0) {
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

//***********************************************************************************************
void GetCharFromSerial() {
  //Get string from serial and put in inString
  //first letter in comm
  //http://forums.adafruit.com/viewtopic.php?f=8&t=9918

  Serial.flush(); //flush all previous received and transmitted data
  byte inCount = 0;
  do
  {
    while (!Serial.available());             // wait for input
    inString[inCount] = Serial.read();       // get it
    if (inString [inCount] == INTERMINATOR) break;  //it is the terminator?  //MANCA IL CONTROLLO SULLA LUNGHEZZA!!
    // CI VUOLE UN OR tipo "or inCount == INLENGTH"
  }
  while (++inCount < INLENGTH);
  inString[inCount] = 0;                     // null terminate the string
  //Serial.print(F("R1 "));  //Only for debug
  //Serial.println(inString);
  comm = inString[0];
}

//***********************************************************************************************
void Blink() {
  digitalWrite(LEDPIN, HIGH);
  delay(100);
  digitalWrite(LEDPIN, LOW);
}

//***********************************************************************************************
void PrintMenu() {
  /*
  b=right1 dir=1
  a=left1 dir=0

  letters->pan

  1=up dir=0
  2=down dir=1
  numbers->tilt
  */

  Serial.println(F("1 1 One step | 2 1 -One step"));
  Serial.println(F("3 1 10 step  | 4 1 -10 step"));
  Serial.println(F("5 1 50 step  | 6 1 -50 step"));

  Serial.println(F("a 2 One step | b 2 -One step"));
  Serial.println(F("c 2 10 step  | d 2 -10 step"));
  Serial.println(F("e 2 50 step  | f 2 -50 step"));

  Serial.println(F("r relay1 on  | s relay1 off"));
  Serial.println(F("t relay2 on  | u relay2 off"));
  Serial.println(F("l Rst limits | p print actual position"));
  Serial.println(F("w Move Servo | n Rst limits2"));
  Serial.println(F("v Print version"));
  Serial.println(F("B Blink LED 13"));
  Serial.println(F("m print menu"));

  Serial.println(F("--------------------"));
  Serial.println(F("Type number and press enter"));


}

//***********************************************************************************************
void Step() {
  //Tilt
  digitalWrite(STEP1PULSE, HIGH);
  //delayMicroseconds(DELAY);
  delay(DELAYM);
  digitalWrite(STEP1PULSE, LOW);
  //delayMicroseconds(DELAY);
  delay(DELAYM);
  Tilt_Actual_Position = Tilt_Actual_Position + Tilt_increment;
}

//***********************************************************************************************
void Step2() {
  //Pan
  digitalWrite(STEP2PULSE, HIGH);
  //delayMicroseconds(DELAY);
  delay(DELAYM);
  digitalWrite(STEP2PULSE, LOW);
  //delayMicroseconds(DELAY);
  delay(DELAYM);
  Pan_Actual_Position = Pan_Actual_Position + Pan_increment;
}

//***********************************************************************************************
//Tilt
void Steps(int passi, int dir) {
  if (dir == 1)  //Right
  {
    digitalWrite(STEP1DIR, HIGH); //Down
    Tilt_increment = 1;
  }
  else
  {
    digitalWrite(STEP1DIR, LOW); //Up
    Tilt_increment = -1;
  }
  for (int i = 0; i < passi; i++) {
    Step();
  }
}

//***********************************************************************************************
//Pan
void Steps2(int passi, int dir) {
  if (dir == 1) //Down
  {
    digitalWrite(STEP2DIR, HIGH); //Right
    Pan_increment = 1;
    //If we are already on End Limit do nothing
    if (digitalRead(PAN_END_PIN) == 1) {
      return;
    }
  }
  else
  {
    digitalWrite(STEP2DIR, LOW); //Left
    Pan_increment = -1;
    //If we are already on Start Limit do nothing
    if (digitalRead(PAN_START_PIN) == 1) {
      return;
    }

  }
  for (int i = 0; i < passi; i++) {
    Step2();
  }
}

//***********************************************************************************************
void ParseMenu(char Stringa) {
  boolean IsKnownCommand = true;
  switch (Stringa) {
    case '1': //Up
      Steps(1, 0);
      break;
    case '2': //Down
      Steps(1, 1);
      break;
    case '3':
      Steps(10, 0);
      break;
    case '4':
      Steps(10, 1);
      break;
    case '5':
      Steps(50, 0);
      break;
    case '6':
      Steps(50, 1);
      break;
    case 'a': //left
      Steps2(1, 0);
      break;
    case 'b': //right
      Steps2(1, 1);
      break;
    case 'c':
      Steps2(10, 0);
      break;
    case 'd':
      Steps2(10, 1);
      break;
    case 'e':
      Steps2(50, 0);
      break;
    case 'f':
      Steps2(50, 1);
      break;

    case 'g':
      //goto NOT working because there is not a check
      //uncomment steps and steps2 to have it working.

      /*Get pan tilt and directions
       *Pan_increment, Pan_steps, Tilt_increment, Tilt_steps;
       *
       * ex g11001100
       * mean
       * pan
       * direction 1
       * 100 steps
       * tilt
       * direction1
       * 100 steps
       *
       * Lenght must be checked!
       * No checks at this time!
       */

      inString[0] = receivedChars[1];
      inString[1] = '\0';
      Pan_increment = atoi(inString);

      inString[0] = receivedChars[2];
      inString[1] = receivedChars[3];
      inString[2] = receivedChars[4];
      inString[3] = '\0';
      Pan_steps = atoi(inString);

      inString[0] = receivedChars[5];
      inString[1] = '\0';
      Tilt_increment = atoi(inString);

      inString[0] = receivedChars[6];
      inString[1] = receivedChars[7];
      inString[2] = receivedChars[8];
      inString[3] = '\0';
      Tilt_steps = atoi(inString);
      /*
            Serial.println(Pan_increment);
            Serial.println(Pan_steps);
            Serial.println(Tilt_increment);
            Serial.println(Tilt_steps);
      */

      //must check them before goto execution
      /*
       * increment must be 0 or 1
       * steps must be < Max
       * Pan steps must be <Max
       * Pan stesps must be <Max-Pan_Actual_Position + 10 as tolerance.
       *
      */
      if (Pan_increment < 0 || Pan_increment > 1) {
        break;
      }
      if (Tilt_increment < 0 || Tilt_increment > 1) {
        break;
      }
      /*
      int diff = Pan_End_Position - Pan_Actual_Position - Pan_steps;
      if (diff<0 ) {
        break;
      }

      steps2(Pan_steps,Pan_increment);
      steps(Tilt_steps,Tilt_increment);
      */
      break;

    case 'l':
      ResetSteppersLimits();
      break;
    case 'r':
      digitalWrite(RELAY1, HIGH);
      break;
    case 's':
      digitalWrite(RELAY1, LOW);
      break;
    case 't':
      digitalWrite(RELAY2, HIGH);
      break;
    case 'u':
      digitalWrite(RELAY2, LOW);
      break;
    case 'p':
      break;
    case 'm':
      PrintMenu();
      //IsKnownCommand = false;  //because we don't need OK..
      break;
    case 'B':
      Blink();
      break;
    case 'v':
      PrintVersion();
      break;
    case 'w':

      inString[0] = receivedChars[1];
      inString[1] = receivedChars[2];
      inString[2] = receivedChars[3];
      inString[4] = '\0';
      /*
      Serial.write(inString[1]);
      Serial.write(inString[2]);
      Serial.write(inString[3]);
      */
      //inString[0] = '0';
      //Serial.println();
      //Serial.println(atoi(inString));
      //sscanf(inString, "%d", &angle);
      //angle=100*atoi(inString[1])+10*atoi(inString[2])+atoi(inString[3]);

      angle = atoi(inString);
      //Serial.println(angle);
      //Check angle
      if (angle > 180)
      {
        angle = 180;
      }
      if (angle < 0)
      {
        angle = 0;
      }

      //Serial.println(angle);
      Servocamera.write(angle);
      break;
    default:
      IsKnownCommand = false;
  }
  if (IsKnownCommand == true)
  {
    Serial.println(F("OK"));
    EndCommand();
    Blink();
  } else {
    Serial.println(F("ERROR!"));
    Serial.println(Stringa, HEX);
    Serial.println(Stringa, HEX); //I need this because PC expects three lines
    Blink();
  }
}

//***********************************************************************************************
void EndCommand() {
  Serial.println(Pan_Actual_Position);
  Serial.println(Tilt_Actual_Position);
}


//***********************************************************************************************
void ResetSteppersLimits() {
  // Go to the Up end switch
  //#define TILT_START_PIN A1 //front mirror

  //Serial.print(digitalRead(TILT_START_PIN));
  //Serial.print(" ");
  //Serial.println(digitalRead(TILT_END_PIN));

  //Check if position is TILT_START_PIN or near it
  //if yes move a little
  if (digitalRead(TILT_START_PIN) == 1) {
    Steps(150, 1);
  }

  Tilt_Actual_Position = 0;
  //Move until find TILT_START_PIN
  while (digitalRead(TILT_START_PIN) == 0) {
    Steps(1, 0);
  }
  //Serial.println(Tilt_Actual_Position);
  delay(500);

  Tilt_Actual_Position = 0;
  //Move until find TILT_END_PIN
  while (digitalRead(TILT_END_PIN) == 0) {
    Steps(1, 1);
  }
  //Serial.print("Total steps ->");
  //Serial.println(Tilt_Actual_Position);
  delay(500);

  Tilt_End_Position = Tilt_Actual_Position;
  Tilt_Future_Position = Tilt_End_Position / 2;
  //Move to the middle
  Steps(Tilt_Future_Position, 0);
  //Serial.println(Tilt_Actual_Position);

  //Repeat for Pan
  //Serial.print(digitalRead(PAN_START_PIN));
  //Serial.print(" ");
  //Serial.println(digitalRead(PAN_END_PIN));

  Pan_Actual_Position = 0;
  while (digitalRead(PAN_START_PIN) == 0) {
    Steps2(1, 0);
  }
  //Serial.println(Pan_Actual_Position);
  delay(500);

  Pan_Actual_Position = 0;
  while (digitalRead(PAN_END_PIN) == 0) {
    Steps2(1, 1);
  }
  //Serial.print("Total steps ->");
  //Serial.println(Pan_Actual_Position);
  delay(500);

  Pan_End_Position = Pan_Actual_Position;
  Pan_Future_Position = Pan_End_Position / 2;
  //Move to the middle
  Steps2(Pan_Future_Position, 0);
  //Serial.println(Pan_Actual_Position);

  //Print all
  //Serial.println(Pan_End_Position);
  //Serial.println(Tilt_End_Position);
  //Serial.println(Pan_Actual_Position);
  //Serial.println(Tilt_Actual_Position);
}

//***********************************************************************************************
void ShowMovements() {
  Serial.println("UP");
  Steps(50, 0); //UP
  delay(1000);
  Serial.println("DOWN");
  Steps(50, 1); //DOWN
  delay(1000);
  Serial.println("RIGHT");
  Steps2(50, 1);//RIGHT
  delay(1000);
  Serial.println("LEFT");
  Steps2(50, 0); //LEFT
}

/***********************************************************************************************
 *
 * *********************************************************************************************
 */

void setup() {
  Serial.begin(115200);
  Serial.println("ArduMirror");

  pinMode(STEP1DIR, OUTPUT);
  pinMode(STEP1PULSE, OUTPUT);

  pinMode(STEP2DIR, OUTPUT);
  pinMode(STEP2PULSE, OUTPUT);

  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);

  pinMode(LEDPIN, OUTPUT);

  pinMode(PAN_START_PIN, INPUT_PULLUP);
  pinMode(PAN_END_PIN, INPUT_PULLUP);
  pinMode(TILT_START_PIN, INPUT);
  pinMode(TILT_END_PIN, INPUT);

  digitalWrite(STEP1DIR, LOW);
  digitalWrite(STEP1PULSE, LOW);

  digitalWrite(STEP2DIR, LOW);
  digitalWrite(STEP2PULSE, LOW);

  digitalWrite(RELAY1, LOW);
  digitalWrite(RELAY2, LOW);

  digitalWrite(LEDPIN, LOW);

  Servocamera.attach(SERVO);
  Servocamera.write(SERVOCAMERA_POINT_POSITION);

  //ResetSteppersLimits();
  /*
  Serial.flush();
  char rc;
  while (Serial.available() > 0 ) {
    rc = Serial.read();
  }
  */
}

/***********************************************************************************************
 *
 * *********************************************************************************************
 */

void loop() {

  //New loop routines
  recvWithEndMarker();
  if (newData == true) {
    comm = receivedChars[0];
    ParseMenu(comm);
    newData = false;
  }

  //Old loop routines
  //GetCharFromSerial();
  //Serial.println(inString);
  //ParseMenu(comm);
}





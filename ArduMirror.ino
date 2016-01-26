/*
********************
* MirrorStepper04  *
********************

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
#define RELAY2 3 // was 10  //Ripristinare il 3!!!!

#define SERVO 10// was 11	//PWM works on pins 3, 5, 6, 9, 10, and 11

#define PAN_START_PIN  11
#define PAN_END_PIN  12
#define TILT_START_PIN A0
#define TILT_END_PIN A1

#define DELAY 2000

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

int Pan_increment = 1;  //Da correggere nel codice per vedere se va incrementato o decrementato a seconda del movimento.
int Tilt_increment = 1;




void PrintVersion() {
  Serial.println("MirrorStepperServo04");
  Serial.print(F("Version 0.1"));
  Serial.print(__DATE__);  //this is the compiling date
  Serial.print(F(" "));
  Serial.println(__TIME__); //this is the compiling time
  Serial.println();
}

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

void Blink() {
  // Serial.println(F("Blink LED"));  //Only for debug
  digitalWrite(LEDPIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);               // wait for a second
  digitalWrite(LEDPIN, LOW);    // turn the LED off by making the voltage LOW
}


void PrintMenu() {
  /*
    Serial.println(F("1 1 One step"));
    Serial.println(F("2 1 -One step"));
    Serial.println(F("3 1 10 step"));
    Serial.println(F("4 1 -10 step"));
    Serial.println(F("5 1 50 step"));
    Serial.println(F("6 1 -50 step"));

    Serial.println(F("a 2 One step"));
    Serial.println(F("b 2 -One step"));
    Serial.println(F("c 2 10 step"));
    Serial.println(F("d 2 -10 step"));
    Serial.println(F("e 2 50 step"));
    Serial.println(F("f 2 -50 step"));

    Serial.println(F("r relay1 on"));
    Serial.println(F("s relay1 off"));
    Serial.println(F("t relay2 on"));
    Serial.println(F("u relay2 off"));

    Serial.println(F("v Print version"));
    Serial.println(F("B Blink LED 13"));
    Serial.println(F("m print menu"));
    Serial.println(F("--------------------"));
    Serial.println(F("Type number and press enter"));


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

  Serial.println(F("w Move Servo"));
  Serial.println(F("v Print version"));
  Serial.println(F("B Blink LED 13"));
  Serial.println(F("m print menu"));
  Serial.println(F("--------------------"));
  Serial.println(F("Type number and press enter"));

  // AGGIUNGERE
  //GOTO
  //RESET FINECORSA


}

void Step() {
  //Tilt
  //Controlla se non siamo ad un fine corsa
  digitalWrite(STEP1PULSE, HIGH);
  delayMicroseconds(DELAY);
  digitalWrite(STEP1PULSE, LOW);
  delayMicroseconds(DELAY);
  Tilt_Actual_Position = Tilt_Actual_Position + Tilt_increment;
}

void Step2() {
  //Pan
  //Controlla se non siamo ad un fine corsa
  digitalWrite(STEP2PULSE, HIGH);
  delayMicroseconds(DELAY);
  digitalWrite(STEP2PULSE, LOW);
  delayMicroseconds(DELAY);
  Pan_Actual_Position = Pan_Actual_Position + Pan_increment;
}

//Tilt
void Steps(int passi, int dir) {
  if (dir == 1)  //Right
  {
    digitalWrite(STEP1DIR, HIGH); //Right
    Tilt_increment = 1;
    //si decide anche quale finecorsa usare
  }
  else
  {
    digitalWrite(STEP1DIR, LOW); //Left
    Tilt_increment = -1;
    //si decide anche quale finecorsa usare
  }
  for (int i = 0; i < passi; i++) {
    Step();
    //delay(10);
  }
}

//Pan
void Steps2(int passi, int dir) {
  if (dir == 1) //Down
  {
    digitalWrite(STEP2DIR, HIGH); //Down
    Pan_increment = 1;
    //si decide anche quale finecorsa usare
  }
  else
  {
    digitalWrite(STEP2DIR, LOW); //Up
    Pan_increment = -1;
    //si decide anche quale finecorsa usare
  }
  for (int i = 0; i < passi; i++) {
    Step2();
    //delay(10);
    // stampa la posizione attuale

  }
}

void ParseMenu(char Stringa) {
  //Serial.println(F("Parsing")); //Only for debug
  boolean IsKnownCommand = true;
  switch (Stringa) {
    case '1': //Up
      Steps(1, 0);    //Aggiungerei alla fine anche una stampa di un OK per stabilire che ha finito. Oppure su richiesta? No, la richiesta potrebbe essere fatta prima della fine del comando.
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


    case 'm':
      PrintMenu();
      IsKnownCommand = false;  //because we don't need OK
      break;
    case 'B':
      Blink();
      break;
    case 'v':
      PrintVersion();
      break;
    case 'w':
      /*
      Serial.write(inString[1]);
      Serial.write(inString[2]);
      Serial.write(inString[3]);
      */
      inString[4] = '\0';
      inString[0] = '0';
      //Serial.println();
      //Serial.println(atoi(inString));
      //sscanf(inString, "%d", &angle);
      //angle=100*atoi(inString[1])+10*atoi(inString[2])+atoi(inString[3]);
      angle = atoi(inString);
      Serial.println(angle);
      if (angle > 180)
      {
        angle = 180;
      }
      if (angle < 0)
      {
        angle = 0;
      }
      Servocamera.write(angle);
      break;
    default:
      //Serial.print(F("Command Unknown! ->"));
      //Serial.println(Stringa, HEX);
      IsKnownCommand = false;
  }
  if (IsKnownCommand == true)
  {
    Serial.println(F("OK"));
    EndCommand();
    // Serial.println(F("OK"));
    Blink();
  } else {
    Serial.println(F("ERROR!"));
    Serial.println(Stringa, HEX);
    Serial.println(Stringa, HEX);
    Blink();
  }
}

void EndCommand() {

  Serial.println(Pan_Actual_Position);
  Serial.println(Tilt_Actual_Position);
}

void setup() {
  Serial.begin(115200);
  Serial.println("MirrorStepperServo04");

  pinMode(STEP1DIR, OUTPUT);
  pinMode(STEP1PULSE, OUTPUT);

  pinMode(STEP2DIR, OUTPUT);
  pinMode(STEP2PULSE, OUTPUT);

  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);

  pinMode(LEDPIN, OUTPUT);

  digitalWrite(STEP1DIR, LOW);
  digitalWrite(STEP1PULSE, LOW);

  digitalWrite(STEP2DIR, LOW);
  digitalWrite(STEP2PULSE, LOW);

  digitalWrite(RELAY1, LOW);
  digitalWrite(RELAY2, LOW);

  digitalWrite(LEDPIN, LOW);

  Servocamera.attach(SERVO);
  Servocamera.write(SERVOCAMERA_POINT_POSITION);

  Pan_Actual_Position = 500;
  Tilt_Actual_Position = 500;
}

void loop() {
  GetCharFromSerial();
  //Serial.println(inString);
  ParseMenu(comm);
}




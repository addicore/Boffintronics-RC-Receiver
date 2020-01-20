//**********************************************************************************************************************
//
//                                       Steve Koci's DIY Remote Receiver
//                                        by Addicore.com & Boffintronics
//
// Revision History:
//       01-03-2020   ARM     Pre Release Version 0.5
//       01-18-2020   ARM     Initial Release Version 1.0
//
//**********************************************************************************************************************
// Copyright (C) 2020  Boffintronics, LLC
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//**********************************************************************************************************************

#include <SPI.h>
#include <EEPROM.h>
#include <Servo.h>
#include <nRF24L01.h>    // NRF24L01 library by TMRh20 https://github.com/TMRh20/RF24
#include <RF24.h>

// Digital outputs
const int OUT1 = 1;       // Outer left and shoulder button (S4 & S8)
const int OUT2 = 2;       // Left Toggle Switch (S2)
const int OUT3 = 3;       // Left Joystick button (A1)
const int OUT4 = 4;       // Center left button (S5)
const int OUT5 = 5;       // Center right button (S6)
const int OUT6 = 6;       // Right Joystick button (A2)
const int OUT7 = 7;       // Right Toggle Switch (S3)
const int OUT8 = 10;      // Outer left and shoulder button (S7 & S9)

const int PROG_BTN = 7;   // Program mode button
const int PROG_LED = 10;  // Program mode status LED
const int LED_ON = 1;
const int LED_OFF = 0;

// Servo outputs
Servo myServo1;           // Left Slide pot (PCB R1)
Servo myServo2;           // Left Joystick X pot (PCB A1)
Servo myServo3;           // Left Joystick Y pot (PCB A1)
Servo myServo4;           // Right Joystick X pot (PCB A2)
Servo myServo5;           // Right Joystick Y pot (PCB A2)
Servo myServo6;           // Right Slide pot (PCB R2)

// NRF24L01 select pins
RF24 radio(8, 9);         // NRF24L01 CE,CSN pins

struct Data_Package {     // NRF24L01 Data payload package
  int LsPot;
  int LjPotX;
  int LjPotY;
  int RjPotX;
  int RjPotY;
  int RsPot;
  byte Switches;
};
Data_Package data;

int Program = false;

//**********************************************************************************************************************
void setup() {

  pinMode(PROG_BTN, INPUT_PULLUP); // set program button input
  pinMode(PROG_LED, OUTPUT);       // set program LED output

  Serial.begin(19200);  // Initialize serial communications with PC
  Serial.println("----------------------------------");
  Serial.println("Steve Koci's DIY Remote Controller");
  Serial.println("         by Addicore.com");
  Serial.println("       Receiver Version 1.0");
  Serial.println("----------------------------------");

  CheckEeprom();    // Check eeprom data and initilize if necessary

  digitalWrite(PROG_LED, LED_ON); // Indicate we are in program mode
  if (digitalRead(PROG_BTN) == LOW) {    // check for program button pressed, enter program mode if so
    ProgramMode();
  }

  pinMode(OUT1, OUTPUT);
  pinMode(OUT2, OUTPUT);
  pinMode(OUT3, OUTPUT);
  pinMode(OUT4, OUTPUT);
  pinMode(OUT5, OUTPUT);
  pinMode(OUT6, OUTPUT);
  pinMode(OUT7, OUTPUT);
  pinMode(OUT8, OUTPUT);

  myServo1.attach(A0);
  myServo2.attach(A1);
  myServo3.attach(A2);
  myServo4.attach(A3);
  myServo5.attach(A4);
  myServo6.attach(A5);

  SetUpRadio();  // Setup Radio for receive

}

//**********************************************************************************************************************
void loop() {
  //--------------------------------------------------------------------------------------------------------------------
  //
  //                                                      ADC low (always 0)
  //                                                     /   ADC high (always 1023)
  //                                                    /   /    Servo end point 1 in degrees (0 to 180)
  //                                                   /   /    /   Servo end point 2 in degrees (0 to 180)
  //                                                  /   /    /   /
  // myServoX.write(AdjustServoData((map(data.LsPot, 0, 1023, 0, 180)), 0, 0));
  //                                                                     \  \
  //                                                                      \  Deadzone, 0=off, 1=on (currently unimplemented)
  //                                                                       Offset in degrees (0 to 180)
  //
  //--------------------------------------------------------------------------------------------------------------------

  if (radio.available()) {
    // get new data package from radio
    radio.read(&data, sizeof(Data_Package)); 

    // Write to servos with adjustments (see above)
    myServo1.write(AdjustServoData((map(data.LsPot, 0, 1023, 0, 180)), 0, 0));    // left slide pot
    myServo2.write(AdjustServoData((map(data.LjPotX, 0, 1023, 0, 180)), 0, 0));   // left joystick x pot
    myServo3.write(AdjustServoData((map(data.LjPotY, 0, 1023, 0, 180)), 0, 0));   // left joystick y pot
    myServo4.write(AdjustServoData((map(data.RjPotX, 0, 1023, 180, 0)), 0, 0)); // right joystick x pot
    myServo5.write(AdjustServoData((map(data.RjPotY, 0, 1023, 180, 0)), 0, 0));   // right joystick y pot
    myServo6.write(AdjustServoData((map(data.RsPot, 0, 1023, 0, 180)), 0, 0));    // right slide pot

    // Write digital data (buttons) to outputs
    digitalWrite(OUT1, !bitRead(data.Switches, 7));
    digitalWrite(OUT2, !bitRead(data.Switches, 6));
    digitalWrite(OUT3, !bitRead(data.Switches, 5));
    digitalWrite(OUT4, !bitRead(data.Switches, 4));
    digitalWrite(OUT5, !bitRead(data.Switches, 3));
    digitalWrite(OUT6, !bitRead(data.Switches, 2));
    digitalWrite(OUT7, !bitRead(data.Switches, 1));
    digitalWrite(OUT8, !bitRead(data.Switches, 0));
  }
}
//**********************************************************************************************************************


//--------------------------------------------------------------------------------------------------------------------
// Adjust servo data for trim and deadzone
//--------------------------------------------------------------------------------------------------------------------

int AdjustServoData (int Data, int Trim, int DeadZone) {
  int AdjustedData;
  int Center;

  AdjustedData = Data + Trim;
  AdjustedData = constrain(AdjustedData, 0, 180);

  Center = 90 + Trim;

  return AdjustedData;
}

// --------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------------------
//   NRF24L01 radio setup
//--------------------------------------------------------------------------------------------------------------------


// EEPROM locations used to store radio parameters
const int EEP_FREQ_TABLE_INDEX = 2;   // EEPROM address of table index
const int EEP_PIPE_TABLE_INDEX = 3;   // EEPROM address of pipe table index
const int EEP_MULTI_RX = 4;           // EEPROM address of multi rx flag

// NRF24L01 frequency table
// this table contains 11 selected frequencies out of the 125 possible for the NRF24
// [0] = the master frequency used only in program mode to sync with the receiver
// [1] through [10] are the available operating frequencies that can be set in program mode
// valid NRF24 frequencies are numbered 0 to 124, frequencies 100-124 are reccomended as are they above wifi
const int NRFfrequencyTable[12] = {100, 102, 104, 106, 108, 110, 112, 114, 116, 120, 124};

// NRF24L01 pipe address table
const byte pipes[][6] = {"Pipe0", "Pipe1", "Pipe2", "Pipe3", "Pipe4", "Pipe5"};

void SetUpRadio(void) {
  int NRFfrequencyIndex = 0;
  int NRFpipeIndex = 1;

  if (Program == false) {
    NRFfrequencyIndex = EEPROM.read(EEP_FREQ_TABLE_INDEX); // read the table index from EEPROM
    NRFpipeIndex = EEPROM.read(EEP_PIPE_TABLE_INDEX); // read the pipe table index from EEPROM
  } else {
    NRFfrequencyIndex = 0;  // program mode, set for master frequency and pipe
    NRFpipeIndex = 1;       //
  }

  radio.begin();
  radio.setChannel(NRFfrequencyTable[NRFfrequencyIndex]);  // use the index to look up the frequency and set the NRF24l01
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);   // Set data rate to 250kbps
  radio.startListening();
  radio.openReadingPipe(1, pipes[NRFpipeIndex]);

  Serial.print ("Frequency = ");
  Serial.print (NRFfrequencyIndex);
  Serial.print ("  ");
  Serial.println (NRFfrequencyTable[NRFfrequencyIndex]);
  Serial.print ("Pipe = ");
  Serial.print (NRFpipeIndex);
  Serial.print ("  ");
  String pipe = pipes[NRFpipeIndex];
  Serial.println (pipe);
  return;
}


// --------------------------------------------------------------------------------------------------------------------

/*
   --------------------------------------------------------------------------------------------------------------------
   Program mode
   --------------------------------------------------------------------------------------------------------------------
*/
const unsigned long PROG_TIMOUT = 30000;     // program timeout (in ms)
const unsigned long LED_ON_TIME = 100;       // led on time (in ms)
const unsigned long LED_OFF_TIME = 300;      // led off time (in ms)
const unsigned long LED_DELAY_TIME = 1000;   // time between flash sequences (in ms)
const unsigned long SHORT_PRESS = 100;       // short button press (in ms), inc fequency number
const unsigned long LONG_PRESS = 5000;       // long button press (in ms), lock in new frequency number, exit program mode

struct Program_Data_Package {
  int pHead = 0;
  int pFreq = 0;
  int pPipe = 0;
  int pTail = 0;
};
Program_Data_Package pdata;

void ProgramMode(void) {

  int FlashCount = EEPROM.read(EEP_FREQ_TABLE_INDEX);
  int NewCount = FlashCount;
  unsigned long ButtonUpCount = 1;
  unsigned long ButtonDownCount = 0;
  unsigned long LedOnCount = 0;
  unsigned long LedOffCount = 0;
  unsigned long ProgramTimeout = 0;
  Program = true;

  digitalWrite(PROG_LED, LED_ON); // Indicate we are in program mode

  while (digitalRead(PROG_BTN) == LOW) {  // wait for program button to be released
  }

  digitalWrite(PROG_LED, LED_OFF);

  Serial.println("Program mode");


  LedOffCount = millis();
  ProgramTimeout = millis();

  SetUpRadio();  // set radio to receive on master frequency and pipe (Program flag is true)

  while (1) {    //

    //--------------------------------------------------------------------------------------------------------------------
    // flash the current frequency number on program LED

    if ((digitalRead(PROG_LED) == LED_ON)
        && (millis() >= (LedOnCount + LED_ON_TIME))) {
      digitalWrite(PROG_LED, LED_OFF);
      LedOffCount = millis();
      FlashCount--;
    }

    if ((FlashCount > 0)
        && (digitalRead(PROG_LED) == LED_OFF)
        && (millis() >= (LedOffCount + LED_OFF_TIME))) {
      digitalWrite(PROG_LED, LED_ON);
      LedOnCount = millis();
    }

    if ((FlashCount == 0)
        && (digitalRead(PROG_LED) == LED_OFF)
        && (millis() >= (LedOffCount + LED_DELAY_TIME))) {
      digitalWrite(PROG_LED, LED_ON);
      LedOnCount = millis();
      FlashCount = NewCount;
    }

    //--------------------------------------------------------------------------------------------------------------------

    if (radio.available()) {
      radio.read(&pdata, sizeof(Program_Data_Package));

      if ((pdata.pHead == 0xaa) && (pdata.pTail == 0x55)) {

        Serial.print("Program EEPROM: ");
        Serial.print (pdata.pHead, HEX);
        Serial.print (" ");
        Serial.print (pdata.pFreq, HEX);
        Serial.print (" ");
        Serial.print (pdata.pPipe, HEX);
        Serial.print (" ");
        Serial.println (pdata.pTail, HEX);

        EEPROM.write(EEP_FREQ_TABLE_INDEX, pdata.pFreq);
        EEPROM.write(EEP_PIPE_TABLE_INDEX, pdata.pPipe);
        NewCount = pdata.pFreq;

        pdata.pHead = 0;
        pdata.pTail = 0;

      }
    }
  }
}

//--------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------------------
// Check eeprom for valid head, tail, and that the frequency and pipe table indexes are in the correct range
// if any are incorrect, rewrite the entire block with default values
//--------------------------------------------------------------------------------------------------------------------

void CheckEeprom(void) {

  if ((EEPROM.read(1) != 0xAA)
      || ((EEPROM.read(2) < 0) || (EEPROM.read(2) > 10))
      || ((EEPROM.read(3) < 0) || (EEPROM.read(3) > 5))
      || (EEPROM.read(4) > 4)
      || (EEPROM.read(5) != 0x55)) {

    Serial.println ("EEPROM Corrupt - Rewriting with defaults ");
    EEPROM.write(1, 0xAA);  // block head
    EEPROM.write(2, 5);     // default frequency
    EEPROM.write(3, 1);     // default pipe
    EEPROM.write(4, 0);     // default Multi Rx (unused on receiver)
    EEPROM.write(5, 0x55);  // block tail
  }

  Serial.print ("EEPROM Valid: ");
  Serial.print(EEPROM.read(1), HEX);
  Serial.print("  ");
  Serial.print(EEPROM.read(2), HEX);
  Serial.print("  ");
  Serial.print(EEPROM.read(3), HEX);
  Serial.print("  ");
  Serial.print(EEPROM.read(4), HEX);
  Serial.print("  ");
  Serial.println(EEPROM.read(5), HEX);

  return;
}

//--------------------------------------------------------------------------------------------------------------------










/*
       myServo1.write(map(data.LsPot, 0, 1023, 0, 180));
       myServo2.write(map(data.LjPotX, 0, 1023, 180, 0));
       myServo3.write(map(data.LjPotY, 0, 1023, 180, 0));
       myServo4.write(map(data.RjPotX, 0, 1023, 0, 180));
       myServo5.write(map(data.RjPotY, 0, 1023, 0, 180));
       myServo6.write(map(data.RsPot, 0, 1023, 0, 180));
*/

//int LeftPotX = map(data.LjPotX, 0, 1023, 0, 180);
//int LeftPotY = map(data.LjPotY, 0, 1023, 0, 180);
//int RightPotX = map(data.RjPotX, 0, 1023, 180, 0);
//int RightPotY = map(data.RjPotY, 0, 1023, 180, 0);
//int LeftSlide = map(data.LsPot, 0, 1023, 0, 180);
//int RightSlide = map(data.RsPot, 0, 1023, 0, 180);

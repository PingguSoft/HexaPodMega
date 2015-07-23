

//=============================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix software
//Software version: V2.0
//Date: 29-10-2009
//Programmer: Jeroen Janssen [aka Xan]
//         Kurt Eckhardt(KurtE) converted to C and Arduino
//   KÃ¥re Halvorsen aka Zenta - Makes everything work correctly!
//
// This version of the Phoenix code was ported over to the Arduino Environement
// and is specifically configured for the Lynxmotion BotBoarduino
//
//=============================================================================
//
//KNOWN BUGS:
//    - Lots ;)
//
//=============================================================================
// Header Files
//=============================================================================

#define DEFINE_HEX_GLOBALS
#if ARDUINO>99
#include <Arduino.h>
#else
#endif
#include <Wire.h>
#include <EEPROM.h>

#include "common.h"
#include "ServoEx.h"
#include "config.h"
#include "Phoenix.h"
//#include "Phoenix_Input_Serial.h"
//#include "Phoenix_Code.h"

void setup()
{
#ifdef DBG_SERIAL
    DBG_SERIAL.begin(BAUD_DEBUG);
#endif    
}

void loop()
{
}

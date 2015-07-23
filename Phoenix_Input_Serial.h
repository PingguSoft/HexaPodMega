//====================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix, control file.
//The control input subroutine for the phoenix software is placed in this file.
//Can be used with V2.0 and above
//Configuration version: V1.0
//Date: 25-10-2009
//Programmer: Jeroen Janssen (aka Xan)
//             Kurt Eckhardt (aka KurtE) - converted to c ported to Arduino...
//
//Hardware setup: Serial version - This contoll input, uses the same format of input as the old Powerpod serial test program.
// Obviously it can be hacked up to almost any format
//
//NEW IN V1.1
//    - added speaker constant
//    - added variable for number of gaits in code
//    - Changed BodyRot to 1 decimal percision
//    - Added variable Center Point of Rotation for the body
//
//	Walk method 1:
//    - Left Stick	Walk/Strafe
//    - Right Stick	Rotate
//
//	Walk method 2:
//    - Left Stick	Disable
//    - Right Stick	Walk/Rotate
//
//
//
// Packet format:
// DualShock(0) : Checksum of other u8
// DualShock(1)
//   bit7 - Left Button test
//   bit6 - Down Button test
//   bit5 - Right Button test
//   bit4 - Up Button test
//   bit3 - Start Button test
//   bit2 - R3 Button test (Horn)
//   bit1 - L3 Button test
//   bit0 - Select Button test
// DualShock(2)
//	bit7 - Square Button test
//	bit6 - Cross Button test
//	bit5 - Circle Button test
//	bit4 - Triangle Button test
//	bit3 - R1 Button test
//	bit2 - L1 Button test
//	bit1 - R2 Button test
//	bit0 - L2 Button test
// DualShock(3) - Right stick Left/right
// DualShock(4) - Right Stick Up/Down
// DualShock(5) - Left Stick Left/right
// DualShock(6) - Left Stick Up/Down
// Note: The actual usages are from PS2 control
//PS2 CONTROLS:
//    [Common Controls]
//    - Start	    	Turn on/off the bot
//    - L1	    	Toggle Shift mode
//    - L2	    	Toggle Rotate mode
//    - Circle		Toggle Single leg mode
//   - Square        Toggle Balance mode
//    - Triangle		Move body to 35 mm from the ground (walk pos)
//                	and back to the ground
//    - D-Pad up		Body up 10 mm
//    - D-Pad down	Body down 10 mm
//    - D-Pad left	decrease speed with 50mS
//    - D-Pad right	increase speed with 50mS
//
//    [Walk Controls]
//    - select		Switch gaits
//    - Left Stick	(Walk mode 1) Walk/Strafe
//                     (Walk mode 2) Disable
//    - Right Stick	(Walk mode 1) Rotate,
//                    (Walk mode 2) Walk/Rotate
//    - R1	    	Toggle Double gait travel speed
//    - R2	    	Toggle Double gait travel length
//
//    [Shift Controls]
//    - Left Stick	Shift body X/Z
//    - Right Stick	Shift body Y and rotate body Y
//
//    [Rotate Controls]
//    - Left Stick	Rotate body X/Z
//    - Right Stick	Rotate body Y
//
//    [Single leg Controls]
//    - select		Switch legs
//    - Left Stick	Move Leg X/Z (relative)
//    - Right Stick	Move Leg Y (absolute)
//    - R2	    	Hold/release leg position
//
//    [GP Player Controls]
//    - select		Switch Sequences
//    - R2	    	Start Sequence
//
//====================================================================
// [Include files]
#if ARDUINO>99
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif
#include <avr/pgmspace.h>
#include "utils.h"


//[CONSTANTS]
// Default to Serial but allow to be defined to something else
#ifndef BT_SERIAL
#define BT_SERIAL Serial
#endif

#ifndef BAUD_BT
#define BAUD_BT 38400
#endif

#define WALKMODE          0
#define TRANSLATEMODE     1
#define ROTATEMODE        2
#define SINGLELEGMODE     3
#define GPPLAYERMODE      4

#define SERB_PAD_LEFT    0x8000    //   bit7 - Left Button test
#define SERB_PAD_DOWN    0x4000    //   bit6 - Down Button test
#define SERB_PAD_RIGHT   0x2000    //   bit5 - Right Button test
#define SERB_PAD_UP      0x1000    //   bit4 - Up Button test
#define SERB_START       0x800      //   bit3 - Start Button test
#define SERB_R3          0x400    //   bit2 - R3 Button test (Horn)
#define SERB_L3          0x200    //   bit1 - L3 Button test
#define SERB_SELECT      0x100    //   bit0 - Select Button test
// DualShock(2)
#define SERB_SQUARE      0x80    //	bit7 - Square Button test
#define SERB_CROSS       0x40    //	bit6 - Cross Button test
#define SERB_CIRCLE      0x20    //	bit5 - Circle Button test
#define SERB_TRIANGLE    0x10    //	bit4 - Triangle Button test
#define SERB_R1          0x8    //	bit3 - R1 Button test
#define SERB_L1          0x4    //	bit2 - L1 Button test
#define SERB_R2          0x2    //	bit1 - R2 Button test
#define SERB_L2          0x1    //	bit0 - L2 Button test

#define  SER_RX          3             // DualShock(3) - Right stick Left/right
#define  SER_RY          4            // DualShock(4) - Right Stick Up/Down
#define  SER_LX          5            // DualShock(5) - Left Stick Left/right
#define  SER_LY          6            // DualShock(6) - Left Stick Up/Down


#define TRAVEL_DEAD_ZONE 4      //The deadzone for the analog input from the remote
#define  MAXPS2ERRORCNT  5     // How many times through the loop will we go before shutting off robot?

#ifndef MAX_BODY_Y
#define MAX_BODY_Y 100
#endif

//=============================================================================
// Global - Local to this file only...
//=============================================================================

// Define an instance of the Input Controller...
InputController  g_InputController;       // Our Input controller

static s16       g_BodyYOffset;
static word        g_wSerialErrorCnt;
static s16       g_BodyYShift;
static u8        ControlMode;
static word        g_wButtonsPrev;
static bool        DoubleHeightOn;
static bool        DoubleTravelOn;
static bool        WalkMethod;
u8               GPSeq;             //Number of the sequence
s16              g_sGPSMController;    // What GPSM value have we calculated. 0xff - Not used yet

// some external or forward function references.
extern void SerTurnRobotOff(void);

//==============================================================================
// This is The function that is called by the Main program to initialize
//the input controller, which in this case is the PS2 controller
//process any commands.
//==============================================================================
// If both PS2 and XBee are defined then we will become secondary to the xbee

u8 abDualShock[7];  // we will to receive 7 u8s of data with the first u8 being the checksum

void InputController::Init(void)
{
  int error;

  // May need to init the Serial port here...
  BT_SERIAL.begin(BAUD_BT);

  g_BodyYOffset = 0;
  g_BodyYShift = 0;
  g_wSerialErrorCnt = 0;  // error count

  ControlMode = SINGLELEGMODE;
  DoubleHeightOn = false;
  DoubleTravelOn = false;
  WalkMethod = false;

  mControlState.wSpeedControl = 100;    // Sort of migrate stuff in from Devon.

  abDualShock[SER_LX] = 128;
  abDualShock[SER_LY] = 128;
  abDualShock[SER_RX] = 128;
  abDualShock[SER_RY] = 128;
}

//==============================================================================
// This function is called by the main code to tell us when it is about to
// do a lot of bit-bang outputs and it would like us to minimize any interrupts
// that we do while it is active...
//==============================================================================
void InputController::AllowControllerInterrupts(bool fAllow)
{
  // We don't need to do anything...

}

//#define ButtonPressed(wMask) (((wButtons & wMask) == 0) && ((g_wButtonsPrev & wMask) != 0))
#define ButtonPressed(wMask) (wButtons & wMask)


//==============================================================================
// This is The main code to input function to read inputs from the PS2 and then
//process any commands.
//==============================================================================
void InputController::ControlInput(void)
{
  //u8 abDualShock[7];  // we will to receive 7 u8s of data with the first u8 being the checksum
  unsigned long ulLastChar;
  bool fAdjustLegPositions = false;
  word wButtons;

#if 0
  BT_SERIAL.print("Rd");

  // we will loop through reading our 7 u8s
  ulLastChar = millis();
  for (u8 i=0; i < 7; i++) {
    while  (BT_SERIAL.available() == 0) {
      if ((millis() - ulLastChar) > 200) {
        // We may have lost the serial communications
        if (g_wSerialErrorCnt < MAXPS2ERRORCNT)
          g_wSerialErrorCnt++;    // Increment the error count and if to many errors, turn off the robot.
        else if (mControlState.fHexCurOn)
          SerTurnRobotOff();
        return;  //
      }
    }

    abDualShock[i] = BT_SERIAL.read();
    ulLastChar = millis();
  }
#endif

  int cmd = -1;

  if (BT_SERIAL.available() == 0)
    return;

  cmd = BT_SERIAL.read();
  printf(F("cmd:%c\n"), cmd);

  wButtons = 0;

  switch(cmd) {
    case 'w':
        wButtons |= SERB_PAD_UP;
        break;

    case 'a':
        wButtons |= SERB_PAD_LEFT;
        break;

    case 's':
        wButtons |= SERB_PAD_DOWN;
        break;

    case 'd':
        wButtons |= SERB_PAD_RIGHT;
        break;

    case ' ':
        wButtons |= SERB_START;
          abDualShock[SER_LX] = 128;
          abDualShock[SER_LY] = 128;
          abDualShock[SER_RX] = 128;
          abDualShock[SER_RY] = 128;
        break;

    case 'z':
        wButtons |= SERB_SELECT;
        break;

    case '1':
        wButtons |= SERB_L1;
        break;

    case 'q':
        wButtons |= SERB_L2;
        break;

    case '3':
        wButtons |= SERB_R1;
        break;

    case 'e':
        wButtons |= SERB_R2;
        break;

    // Left Joystick
    case 't':
        if (abDualShock[SER_LY] > 10)
            abDualShock[SER_LY] -= 10;
        break;

    case 'f':
        if (abDualShock[SER_LX] > 10)
            abDualShock[SER_LX] -= 10;
        break;

    case 'g':
        if (abDualShock[SER_LY] < 245)
            abDualShock[SER_LY] += 10;
        break;

    case 'h':
        if (abDualShock[SER_LX] < 245)
            abDualShock[SER_LX] += 10;
        break;

    // Right Joystick
    case 'i':
        if (abDualShock[SER_RY] > 10)
            abDualShock[SER_RY] -= 10;
        break;

    case 'j':
        if (abDualShock[SER_RX] > 10)
            abDualShock[SER_RX] -= 10;
        break;

    case 'k':
        if (abDualShock[SER_RY] < 245)
            abDualShock[SER_RY] += 10;
        break;

    case 'l':
        if (abDualShock[SER_RX] < 245)
            abDualShock[SER_RX] += 10;
        break;

    // buttons
    case 'm':
        wButtons |= SERB_CROSS;
        break;

    case ',':
        wButtons |= SERB_CIRCLE;
        break;

    case '.':
        wButtons |= SERB_SQUARE;
        break;

    case '/':
        wButtons |= SERB_TRIANGLE;
        break;
  }

  switch(cmd) {
    case 't':
    case 'f':
    case 'g':
    case 'h':
    case 'i':
    case 'j':
    case 'k':
    case 'l':
        printf(F("LX:%3d, LY:%3d, RX:%3d, RY:%3d\n"), abDualShock[SER_LX], abDualShock[SER_LY],
            abDualShock[SER_RX], abDualShock[SER_RY]);
        break;
  }

  if (1) {

//    wButtons = (abDualShock[1] << 8) | abDualShock[2];

    // In an analog mode so should be OK...
    g_wSerialErrorCnt = 0;    // clear out error count...

    if (ButtonPressed(SERB_START)) {// OK lets try "0" button for Start.
      if (mControlState.fHexCurOn) {
        SerTurnRobotOff();
      }
      else {
        //Turn on
        mControlState.fHexCurOn = 1;
        fAdjustLegPositions = true;
        printf(F("ON\n"));
      }
    }

    if (mControlState.fHexCurOn) {
      // [SWITCH MODES]

      //Translate mode
      if (ButtonPressed(SERB_L1)) {// L1 Button Test
        Utils::sound( 1, 50, 2000);
        if (ControlMode != TRANSLATEMODE )
          ControlMode = TRANSLATEMODE;
        else {
          if (mControlState.bSingleLegCurSel==255)
            ControlMode = WALKMODE;
          else
            ControlMode = SINGLELEGMODE;
        }
        printf(F("MODE:%d\n"), ControlMode);
      }

      //Rotate mode
      if (ButtonPressed(SERB_L2)) {    // L2 Button Test
        Utils::sound( 1, 50, 2000);
        if (ControlMode != ROTATEMODE)
          ControlMode = ROTATEMODE;
        else {
          if (mControlState.bSingleLegCurSel == 255)
            ControlMode = WALKMODE;
          else
            ControlMode = SINGLELEGMODE;
        }
        printf(F("MODE:%d\n"), ControlMode);
      }

      //Single leg mode fNO
      if (ButtonPressed(SERB_CIRCLE)) {// O - Circle Button Test
        if (abs(mControlState.c3dTravelLen.x)<TRAVEL_DEAD_ZONE && abs(mControlState.c3dTravelLen.z)<TRAVEL_DEAD_ZONE
          && abs(mControlState.c3dTravelLen.y*2)<TRAVEL_DEAD_ZONE )   {
          if (ControlMode != SINGLELEGMODE) {
            ControlMode = SINGLELEGMODE;
            if (mControlState.bSingleLegCurSel == 255)  //Select leg if none is selected
              mControlState.bSingleLegCurSel=IDX_RF; //Startleg
          }
          else {
            ControlMode = WALKMODE;
            mControlState.bSingleLegCurSel=255;
          }
        }
        printf(F("MODE:%d\n"), ControlMode);
      }

#ifdef OPT_GPPLAYER
      // GP Player Mode X
      if (ButtonPressed(SERB_CROSS)) { // X - Cross Button Test
        Utils::sound(1, 50, 2000);
        if (ControlMode != GPPLAYERMODE) {
          ControlMode = GPPLAYERMODE;
          GPSeq=0;
        }
        else
          ControlMode = WALKMODE;
      }
#endif // OPT_GPPLAYER

      //[Common functions]
      //Switch Balance mode on/off
      if (ButtonPressed(SERB_SQUARE)) { // Square Button Test
        mControlState.fBalanceMode = !mControlState.fBalanceMode;
        if (mControlState.fBalanceMode) {
          Utils::sound(1, 250, 1500);
        }
        else {
          Utils::sound( 2, 100, 2000, 50, 4000);
        }
      }

      //Stand up, sit down
      if (ButtonPressed(SERB_TRIANGLE)) { // Triangle - Button Test
        if (g_BodyYOffset>0)
          g_BodyYOffset = 0;
        else
          g_BodyYOffset = 35;
        fAdjustLegPositions = true;
      }

      if (ButtonPressed(SERB_PAD_UP)) {// D-Up - Button Test
        g_BodyYOffset += 10;

        // And see if the legs should adjust...
        fAdjustLegPositions = true;
        if (g_BodyYOffset > MAX_BODY_Y)
          g_BodyYOffset = MAX_BODY_Y;
      }

      if (ButtonPressed(SERB_PAD_DOWN) && g_BodyYOffset) {// D-Down - Button Test
        if (g_BodyYOffset > 10)
          g_BodyYOffset -= 10;
        else
          g_BodyYOffset = 0;      // constrain don't go less than zero.

        // And see if the legs should adjust...
        fAdjustLegPositions = true;
      }

      if (ButtonPressed(SERB_PAD_RIGHT)) { // D-Right - Button Test
        if (mControlState.wSpeedControl>0) {
          mControlState.wSpeedControl = mControlState.wSpeedControl - 50;
          Utils::sound( 1, 50, 2000);
        }
      }

      if (ButtonPressed(SERB_PAD_LEFT)) { // D-Left - Button Test
        if (mControlState.wSpeedControl<2000 ) {
          mControlState.wSpeedControl = mControlState.wSpeedControl + 50;
          Utils::sound( 1, 50, 2000);
        }
      }

      //[Walk functions]
      if (ControlMode == WALKMODE) {
        //Switch gates
        if (ButtonPressed(SERB_SELECT)            // Select Button Test
        && abs(mControlState.c3dTravelLen.x)<TRAVEL_DEAD_ZONE //No movement
        && abs(mControlState.c3dTravelLen.z)<TRAVEL_DEAD_ZONE
          && abs(mControlState.c3dTravelLen.y*2)<TRAVEL_DEAD_ZONE  ) {
          mControlState.bGaitType = mControlState.bGaitType+1;                    // Go to the next gait...
          if (mControlState.bGaitType<CONFIG_NUM_GAITS) {                 // Make sure we did not exceed number of gaits...
            Utils::sound( 1, 50, 2000);
          }
          else {
            Utils::sound(2, 50, 2000, 50, 2250);
            mControlState.bGaitType = 0;
          }
          selectGait();
        }

        //Double leg lift height
        if (ButtonPressed(SERB_R1)) { // R1 Button Test
          Utils::sound( 1, 50, 2000);
          DoubleHeightOn = !DoubleHeightOn;
          if (DoubleHeightOn)
            mControlState.sLegLiftHeight = 80;
          else
            mControlState.sLegLiftHeight = 50;
        }

        //Double Travel Length
        if (ButtonPressed(SERB_R2)) {// R2 Button Test
          Utils::sound(1, 50, 2000);
          DoubleTravelOn = !DoubleTravelOn;
        }

        // Switch between Walk method 1 && Walk method 2
        if (ButtonPressed(SERB_R3)) { // R3 Button Test
          Utils::sound(1, 50, 2000);
          WalkMethod = !WalkMethod;
        }

        //Walking
        if (WalkMethod)  //(Walk Methode)
          mControlState.c3dTravelLen.z = (abDualShock[SER_RY]-128); //Right Stick Up/Down

        else {
          mControlState.c3dTravelLen.x = -(abDualShock[SER_LX] - 128);
          mControlState.c3dTravelLen.z = (abDualShock[SER_LY] - 128);
        }

        if (!DoubleTravelOn) {  //(Double travel length)
          mControlState.c3dTravelLen.x = mControlState.c3dTravelLen.x/2;
          mControlState.c3dTravelLen.z = mControlState.c3dTravelLen.z/2;
        }

        mControlState.c3dTravelLen.y = -(abDualShock[SER_RX] - 128)/4; //Right Stick Left/Right
      }

      //[Translate functions]
      g_BodyYShift = 0;
      if (ControlMode == TRANSLATEMODE) {
        mControlState.c3dBodyPos.x = (abDualShock[SER_LX] - 128)/2;
        mControlState.c3dBodyPos.z = -(abDualShock[SER_LY] - 128)/3;
        mControlState.c3dBodyRot.y = (abDualShock[SER_RX] - 128)*2;
        g_BodyYShift = (-(abDualShock[SER_RY] - 128)/2);
      }

      //[Rotate functions]
      if (ControlMode == ROTATEMODE) {
        mControlState.c3dBodyRot.x = (abDualShock[SER_LY] - 128);
        mControlState.c3dBodyRot.y = (abDualShock[SER_RX] - 128)*2;
        mControlState.c3dBodyRot.z = (abDualShock[SER_LX] - 128);
        g_BodyYShift = (-(abDualShock[SER_RY] - 128)/2);
      }

      //[Single leg functions]
      if (ControlMode == SINGLELEGMODE) {
        //Switch leg for single leg control
        if (ButtonPressed(SERB_SELECT)) { // Select Button Test
          Utils::sound(1, 50, 2000);
          if (mControlState.bSingleLegCurSel<5)
            mControlState.bSingleLegCurSel = mControlState.bSingleLegCurSel+1;
          else
            mControlState.bSingleLegCurSel=0;
        }

        mControlState.c3dSingleLeg.x= (abDualShock[SER_LX] - 128)/2; //Left Stick Right/Left
        mControlState.c3dSingleLeg.y= (abDualShock[SER_RY] - 128)/10; //Right Stick Up/Down
        mControlState.c3dSingleLeg.z = (abDualShock[SER_LY] - 128)/2; //Left Stick Up/Down

        // Hold single leg in place
        if (ButtonPressed(SERB_R2)) { // R2 Button Test
          Utils::sound(1, 50, 2000);
          mControlState.fSingleLegHold = !mControlState.fSingleLegHold;
        }
      }

#ifdef OPT_GPPLAYER
      //[GPPlayer functions]
      if (ControlMode == GPPLAYERMODE) {

        // Lets try some speed control... Map all values if we have mapped some before
        // or start mapping if we exceed some minimum delta from center
        // Have to keep reminding myself that commander library already subtracted 128...
        if (g_ServoDriver.FIsGPSeqActive() ) {
          if ((g_sGPSMController != 32767)
            || (abDualShock[SER_RY] > (128+16)) || (abDualShock[SER_RY] < (128-16)))
          {
            // We are in speed modify mode...
            s16 sNewGPSM = map(abDualShock[SER_RY], 0, 255, -200, 200);
            if (sNewGPSM != g_sGPSMController) {
              g_sGPSMController = sNewGPSM;
              g_ServoDriver.GPSetSpeedMultiplyer(g_sGPSMController);
            }

          }
        }

        //Switch between sequences
        if (ButtonPressed(SERB_SELECT)) { // Select Button Test
          if (!g_ServoDriver.FIsGPSeqActive() ) {
            if (GPSeq < 5) {  //Max sequence
              Utils::sound(1, 50, 1500);
              GPSeq = GPSeq+1;
            }
            else {
              Utils::sound(2, 50, 2000, 50, 2250);
              GPSeq=0;
            }
          }
        }
        //Start Sequence
        if (ButtonPressed(SERB_R2))// R2 Button Test
          if (!g_ServoDriver.FIsGPSeqActive() ) {
            g_ServoDriver.GPStartSeq(GPSeq);
            g_sGPSMController = 32767;  // Say that we are not in Speed modify mode yet... valid ranges are 50-200 (both postive and negative...
          }
          else {
            g_ServoDriver.GPStartSeq(0xff);    // tell the GP system to abort if possible...
            Utils::sound (2, 50, 2000, 50, 2000);
          }


      }
#endif // OPT_GPPLAYER

      //Calculate walking time delay
      mControlState.bInputTimeDelay = 128 - max(max(abs(abDualShock[SER_LX] - 128), abs(abDualShock[SER_LY] - 128)), abs(abDualShock[SER_RX] - 128));
    }

    //Calculate mControlState.c3dBodyPos.y
    mControlState.c3dBodyPos.y = min(max(g_BodyYOffset + g_BodyYShift,  0), MAX_BODY_Y);
    if (fAdjustLegPositions)
      adjustLegPosToBodyHeight();    // Put main workings into main program file

  // remember which buttons were set here
  g_wButtonsPrev = wButtons;

  }
  else {
    // We may have lost the PS2... See what we can do to recover...
    if (g_wSerialErrorCnt < MAXPS2ERRORCNT)
      g_wSerialErrorCnt++;    // Increment the error count and if to many errors, turn off the robot.
    else if (mControlState.fHexCurOn)
      SerTurnRobotOff();
  }
}

//==============================================================================
// SerTurnRobotOff - code used couple of places so save a little room...
//==============================================================================
void SerTurnRobotOff(void)
{
  //Turn off
  mControlState.c3dBodyPos.x = 0;
  mControlState.c3dBodyPos.y = 0;
  mControlState.c3dBodyPos.z = 0;
  mControlState.c3dBodyRot.x = 0;
  mControlState.c3dBodyRot.y = 0;
  mControlState.c3dBodyRot.z = 0;
  mControlState.c3dTravelLen.x = 0;
  mControlState.c3dTravelLen.z = 0;
  mControlState.c3dTravelLen.y = 0;
  g_BodyYOffset = 0;
  g_BodyYShift = 0;
  mControlState.bSingleLegCurSel = 255;
  mControlState.fHexCurOn = 0;
  adjustLegPosToBodyHeight();    // Put main workings into main program file
}








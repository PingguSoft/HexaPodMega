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
//Hardware setup: PS2 version
// 
//NEW IN V1.0
//- First Release
//
//Walk method 1:
//- Left StickWalk/Strafe
//- Right StickRotate
//
//Walk method 2:
//- Left StickDisable
//- Right StickWalk/Rotate
//
//
//PS2 CONTROLS:
//[Common Controls]
//- StartTurn on/off the bot
//- L1Toggle Shift mode
//- L2Toggle Rotate mode
//- CircleToggle Single leg mode
//   - Square        Toggle Balance mode
//- TriangleMove body to 35 mm from the ground (walk pos) 
//and back to the ground
#define D_PAD_STEP 5
//- D-Pad upBody up D_PAD_STEP mm
//- D-Pad downBody down D_PAD_STEP mm
//- D-Pad leftdecrease speed with 50mS
//- D-Pad rightincrease speed with 50mS
//
//[Walk Controls]
//- selectSwitch gaits
//- Left Stick(Walk mode 1) Walk/Strafe
// (Walk mode 2) Disable
//- Right Stick(Walk mode 1) Rotate, 
//(Walk mode 2) Walk/Rotate
//- R1Toggle Double gait travel speed
//- R2Toggle Double gait travel length
//
//[Shift Controls]
//- Left StickShift body X/Z
//- Right StickShift body Y and rotate body Y
//
//[Rotate Controls]
//- Left StickRotate body X/Z
//- Right StickRotate body Y
//
//[Single leg Controls]
//- selectSwitch legs
//- Left StickMove Leg X/Z (relative)
//- Right StickMove Leg Y (absolute)
//- R2Hold/release leg position
//
//[GP Player Controls]
//- selectSwitch Sequences
//- R2Start Sequence
//
//====================================================================
// [Include files]
#if ARDUINO>99
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif
#include <PS2X_lib.h>

//[CONSTANTS]
#define WALKMODE          0
#define TRANSLATEMODE     1
#define ROTATEMODE        2
#define SINGLELEGMODE     3
#define GPPLAYERMODE      4


#define cTravelDeadZone 4      //The deadzone for the analog input from the remote
#define  MAXPS2ERRORCNT  5     // How many times through the loop will we go before shutting off robot?

#ifndef MAX_BODY_Y
#define MAX_BODY_Y 100
#endif

//=============================================================================
// Global - Local to this file only...
//=============================================================================
/* Assign a unique ID to this sensor at the same time */
#ifdef STABILISATOR
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
#endif
PS2X ps2x; // create PS2 Controller Class


// Define an instance of the Input Controller...
InputController  g_InputController;       // Our Input controller 


static short      g_BodyYOffset; 
static short      g_sPS2ErrorCnt;
static short       g_BodyYShift;
static byte        ControlMode;
static bool        DoubleHeightOn;
static bool        DoubleTravelOn;
static bool        WalkMethod;
#ifdef STABILISATOR
static bool        StalilisatorOn;
static float       ref_StabilisationX;
static float       ref_StabilisationY;
static float       ref_StabilisationZ;
#endif
byte            GPSeq;             //Number of the sequence
short              g_sGPSMController;    // What GPSM value have we calculated. 0xff - Not used yet
#if LED_EYE_PWM_PIN!=0
static bool EyeOn =false;
#endif
// some external or forward function references.
extern void PS2TurnRobotOff(void);

#if LED_EYE_PWM_PIN!=0
void SetEyes()
{
   SSCSerial.print("#");
   SSCSerial.print(LED_EYE_PWM_PIN);
   if(EyeOn)
      SSCSerial.print("P2500");
    else
      SSCSerial.print("P500");
    SSCSerial.println("T250");
}
#endif
//==============================================================================
// This is The function that is called by the Main program to initialize
//the input controller, which in this case is the PS2 controller
//process any commands.
//==============================================================================

// If both PS2 and XBee are defined then we will become secondary to the xbee
void InputController::Init(void)
{
#ifdef STABILISATOR
digitalWrite(FEEDBACK_LED, LOW);  
mag.begin();
digitalWrite(FEEDBACK_LED, HIGH);  
#endif

  int error;

  //error = ps2x.config_gamepad(57, 55, 56, 54);  // Setup gamepad (clock, command, attention, data) pins
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT);  // Setup gamepad (clock, command, attention, data) pins

#ifdef DBGSerial
	DBGSerial.print("PS2 Init: ");
	DBGSerial.println(error, DEC);
#endif
  g_BodyYOffset = MAX_BODY_Y/8;    
  g_BodyYShift = 0;
  g_sPS2ErrorCnt = 0;  // error count

  ControlMode = WALKMODE;
  DoubleHeightOn = false;
  DoubleTravelOn = false;
  WalkMethod = false;
#ifdef STABILISATOR
  StalilisatorOn=false;
#endif
#if LED_EYE_PWM_PIN!=0
 EyeOn =false;
#endif
  g_InControlState.SpeedControl = 100;    // Sort of migrate stuff in from Devon.
}

//==============================================================================
// This function is called by the main code to tell us when it is about to
// do a lot of bit-bang outputs and it would like us to minimize any interrupts
// that we do while it is active...
//==============================================================================
void InputController::AllowControllerInterrupts(boolean fAllow)
{
  // We don't need to do anything...
}

//==============================================================================
// This is The main code to input function to read inputs from the PS2 and then
//process any commands.
//==============================================================================
void InputController::ControlInput(void)
{
#ifdef STABILISATOR
  
digitalWrite(FEEDBACK_LED, LOW);  
  sensors_event_t event; 
  mag.getEvent(&event);
digitalWrite(FEEDBACK_LED, HIGH);  
#endif

  boolean fAdjustLegPositions = false;
  // Then try to receive a packet of information from the PS2.
  // Then try to receive a packet of information from the PS2.
  ps2x.read_gamepad();          //read controller and set large motor to spin at 'vibrate' speed

    // Wish the library had a valid way to verify that the read_gamepad succeeded... Will hack for now
  if ((ps2x.Analog(1) & 0xf0) == 0x70) {
#ifdef DBGSerial
#ifdef DEBUG_PS2_INPUT
	if (g_fDebugOutput) {
		DBGSerial.print("PS2 Input: ");
		DBGSerial.print(ps2x.ButtonDataByte(), HEX);
		DBGSerial.print(":");
		DBGSerial.print(ps2x.Analog(PSS_LX), DEC);
		DBGSerial.print(" ");
		DBGSerial.print(ps2x.Analog(PSS_LY), DEC);
		DBGSerial.print(" ");
		DBGSerial.print(ps2x.Analog(PSS_RX), DEC);
		DBGSerial.print(" ");
		DBGSerial.println(ps2x.Analog(PSS_RY), DEC);
	}
#endif
#endif
    // In an analog mode so should be OK...
    g_sPS2ErrorCnt = 0;    // clear out error count...

    if (ps2x.ButtonPressed(PSB_START)) {// OK lets try "0" button for Start. 
      if (g_InControlState.fHexOn) {
        PS2TurnRobotOff();
      } 
      else {
        //Turn on
        g_InControlState.fHexOn = 1;
        fAdjustLegPositions = true;
      }
    }

    if (g_InControlState.fHexOn) {
      // [SWITCH MODES]

      //Translate mode
      if (ps2x.ButtonPressed(PSB_L1)) {// L1 Button Test
        MSound( 1, 50, 2000);  
        if (ControlMode != TRANSLATEMODE )
          ControlMode = TRANSLATEMODE;
        else {
          if (g_InControlState.SelectedLeg==255) 
            ControlMode = WALKMODE;
          else
            ControlMode = SINGLELEGMODE;
        }
      }

      //Rotate mode
      if (ps2x.ButtonPressed(PSB_L2)) {    // L2 Button Test
        MSound( 1, 50, 2000);
        if (ControlMode != ROTATEMODE)
          ControlMode = ROTATEMODE;
        else {
          if (g_InControlState.SelectedLeg == 255) 
            ControlMode = WALKMODE;
          else
            ControlMode = SINGLELEGMODE;
        }
      }
#if LED_EYE_PWM_PIN!=0

      if (ps2x.ButtonPressed(PSB_CIRCLE)) 
      {
        EyeOn=!EyeOn;
        MSound(1, 50, 2000);  
        SetEyes();
      }
#endif
      
#ifdef OPT_SINGLELEG
      //Single leg mode fNO
      if (ps2x.ButtonPressed(PSB_CIRCLE)) {// O - Circle Button Test
        if (abs(g_InControlState.TravelLength.x)<cTravelDeadZone && abs(g_InControlState.TravelLength.z)<cTravelDeadZone 
          && abs(g_InControlState.TravelLength.y*2)<cTravelDeadZone )   {
          if (ControlMode != SINGLELEGMODE) {
            ControlMode = SINGLELEGMODE;
            if (g_InControlState.SelectedLeg == 255)  //Select leg if none is selected
              g_InControlState.SelectedLeg=cRF; //Startleg
          } 
          else {
            ControlMode = WALKMODE;
            g_InControlState.SelectedLeg=255;
          }
        }
      }      
#endif // OPT_SINGLELEG

#ifdef STABILISATOR
      //[Common functions]
      //Switch Balance mode on/off 
      if (ps2x.ButtonPressed(PSB_CROSS)) { // CROSS Button Test
        StalilisatorOn = !StalilisatorOn;
        if (StalilisatorOn) {
          MSound(1, 250, 1500); 
          ref_StabilisationX=  event.magnetic.x;
          ref_StabilisationY=  event.magnetic.y;
          ref_StabilisationZ=  event.magnetic.z;
        } 
        else {
          MSound( 2, 100, 2000, 50, 4000);
        }
      }

#endif// STABILISATOR


/*
#ifdef OPT_GPPLAYER
      // GP Player Mode X
      if (ps2x.ButtonPressed(PSB_CROSS)) { // X - Cross Button Test
        MSound(1, 50, 2000);  
        if (ControlMode != GPPLAYERMODE) {
          ControlMode = GPPLAYERMODE;
          GPSeq=0;
        } 
        else
          ControlMode = WALKMODE;
      }
#endif // OPT_GPPLAYER
*/
      //[Common functions]
      //Switch Balance mode on/off 
      if (ps2x.ButtonPressed(PSB_SQUARE)) { // Square Button Test
        g_InControlState.BalanceMode = !g_InControlState.BalanceMode;
        if (g_InControlState.BalanceMode) {
          MSound(1, 250, 1500); 
        } 
        else {
          MSound( 2, 100, 2000, 50, 4000);
        }
      }

      //Stand up, sit down  
      if (ps2x.ButtonPressed(PSB_TRIANGLE)) { // Triangle - Button Test
        if (g_BodyYOffset==0) 
          g_BodyYOffset = CHexInitY/2;
        else if(g_BodyYOffset==CHexInitY/2)
          g_BodyYOffset = CHexInitY;
        else
          g_BodyYOffset = 0;
        fAdjustLegPositions = true;
      }

      if (ps2x.ButtonPressed(PSB_PAD_UP)) {// D-Up - Button Test
        g_BodyYOffset += D_PAD_STEP;

        // And see if the legs should adjust...
        fAdjustLegPositions = true;
        if (g_BodyYOffset > MAX_BODY_Y)
          g_BodyYOffset = MAX_BODY_Y;
      }

      if (ps2x.ButtonPressed(PSB_PAD_DOWN) && g_BodyYOffset) {// D-Down - Button Test
        if (g_BodyYOffset > D_PAD_STEP)
          g_BodyYOffset -= D_PAD_STEP;
        else
          g_BodyYOffset = 0;      // constrain don't go less than zero.

        // And see if the legs should adjust...
        fAdjustLegPositions = true;
      }

      if (ps2x.ButtonPressed(PSB_PAD_RIGHT)) { // D-Right - Button Test
        if (g_InControlState.SpeedControl>0) {
          g_InControlState.SpeedControl = g_InControlState.SpeedControl - 50;
          MSound( 1, 50, 2000);  
        }
      }

      if (ps2x.ButtonPressed(PSB_PAD_LEFT)) { // D-Left - Button Test
        if (g_InControlState.SpeedControl<2000 ) {
          g_InControlState.SpeedControl = g_InControlState.SpeedControl + 50;
          MSound( 1, 50, 2000); 
        }
      }

      //[Walk functions]
      if (ControlMode == WALKMODE) {
        //Switch gates
        if (ps2x.ButtonPressed(PSB_SELECT)            // Select Button Test
        && abs(g_InControlState.TravelLength.x)<cTravelDeadZone //No movement
        && abs(g_InControlState.TravelLength.z)<cTravelDeadZone 
          && abs(g_InControlState.TravelLength.y*2)<cTravelDeadZone  ) {
          g_InControlState.GaitType = g_InControlState.GaitType+1;                    // Go to the next gait...
          if (g_InControlState.GaitType<NUM_GAITS) {                 // Make sure we did not exceed number of gaits...
            MSound( 1, 50, 2000); 
          } 
          else {
            MSound(2, 50, 2000, 50, 2250); 
            g_InControlState.GaitType = 0;
          }
          GaitSelect();
        }

        //Double leg lift height
        if (ps2x.ButtonPressed(PSB_R1)) { // R1 Button Test
          MSound( 1, 50, 2000); 
          DoubleHeightOn = !DoubleHeightOn;
          if (DoubleHeightOn)
            g_InControlState.LegLiftHeight = 50;
          else
            g_InControlState.LegLiftHeight = 25;
        }

        //Double Travel Length
        if (ps2x.ButtonPressed(PSB_R2)) {// R2 Button Test
          MSound(1, 50, 2000); 
          DoubleTravelOn = !DoubleTravelOn;
        }

        // Switch between Walk method 1 && Walk method 2
        if (ps2x.ButtonPressed(PSB_R3)) { // R3 Button Test
          MSound(1, 50, 2000); 
          WalkMethod = !WalkMethod;
        }

        //Walking
        if (WalkMethod)  //(Walk Methode) 
          g_InControlState.TravelLength.z = (ps2x.Analog(PSS_RY)-128)/2; //Right Stick Up/Down  

        else {
          g_InControlState.TravelLength.x = -(ps2x.Analog(PSS_LX) - 128)*3/5;
          g_InControlState.TravelLength.z = (ps2x.Analog(PSS_LY) - 128)*3/5;
        }

        if (!DoubleTravelOn) {  //(Double travel length)
          g_InControlState.TravelLength.x = g_InControlState.TravelLength.x/2;
          g_InControlState.TravelLength.z = g_InControlState.TravelLength.z/2;
        }

        g_InControlState.TravelLength.y = -(ps2x.Analog(PSS_RX) - 128)/4; //Right Stick Left/Right 
      }

      //[Translate functions]
      g_BodyYShift = 0;
      if (ControlMode == TRANSLATEMODE) {
        g_InControlState.BodyPos.x = (ps2x.Analog(PSS_LX) - 128)/3;
        g_InControlState.BodyPos.z = -(ps2x.Analog(PSS_LY) - 128)/3;
        g_InControlState.BodyRot1.y = (ps2x.Analog(PSS_RX) - 128)*2;
        g_BodyYShift = (-(ps2x.Analog(PSS_RY) - 128)/2);
      }

      //[Rotate functions]
      if (ControlMode == ROTATEMODE) {
        g_InControlState.BodyRot1.x = (ps2x.Analog(PSS_LY) - 128);
        g_InControlState.BodyRot1.y = (ps2x.Analog(PSS_RX) - 128)*2;
        g_InControlState.BodyRot1.z = (ps2x.Analog(PSS_LX) - 128);
        g_BodyYShift = (-(ps2x.Analog(PSS_RY) - 128)/2);
      }

      //[Single leg functions]
      if (ControlMode == SINGLELEGMODE) {
        //Switch leg for single leg control
        if (ps2x.ButtonPressed(PSB_SELECT)) { // Select Button Test
          MSound(1, 50, 2000); 
          if (g_InControlState.SelectedLeg<5)
            g_InControlState.SelectedLeg = g_InControlState.SelectedLeg+1;
          else
            g_InControlState.SelectedLeg=0;
        }

        g_InControlState.SLLeg.x= (ps2x.Analog(PSS_LX) - 128)/2; //Left Stick Right/Left
        g_InControlState.SLLeg.y= (ps2x.Analog(PSS_RY) - 128)/10; //Right Stick Up/Down
        g_InControlState.SLLeg.z = (ps2x.Analog(PSS_LY) - 128)/2; //Left Stick Up/Down

        // Hold single leg in place
        if (ps2x.ButtonPressed(PSB_R2)) { // R2 Button Test
          MSound(1, 50, 2000);  
          g_InControlState.fSLHold = !g_InControlState.fSLHold;
        }
      }


#ifdef STABILISATOR
        if (StalilisatorOn) {
          float dx = ref_StabilisationX-event.magnetic.x;
          float dy = ref_StabilisationY-event.magnetic.y;
          float dz = ref_StabilisationZ-event.magnetic.z;
          if(dx>STABILISATOR_TRIGGERVALUE||dx<-STABILISATOR_TRIGGERVALUE)
            g_InControlState.BodyRot1.x -= dx;
          if(dz>STABILISATOR_TRIGGERVALUE||dz<-STABILISATOR_TRIGGERVALUE)
            g_InControlState.BodyRot1.y -= dz;
          if(dy>STABILISATOR_TRIGGERVALUE||dy<-STABILISATOR_TRIGGERVALUE)
            g_InControlState.BodyRot1.z -= dy;
          //g_InControlState.BodyRot1.z = (ps2x.Analog(PSS_LX) - 128);
          
          }
#endif // STABILISATOR
#ifdef OPT_GPPLAYER
      //[GPPlayer functions]
      if (ControlMode == GPPLAYERMODE) {

        // Lets try some speed control... Map all values if we have mapped some before
        // or start mapping if we exceed some minimum delta from center
        // Have to keep reminding myself that commander library already subtracted 128...
        if (g_ServoDriver.FIsGPSeqActive() ) {
          if ((g_sGPSMController != 32767)  
            || (ps2x.Analog(PSS_RY) > (128+16)) || (ps2x.Analog(PSS_RY) < (128-16)))
          {
            // We are in speed modify mode...
            short sNewGPSM = map(ps2x.Analog(PSS_RY), 0, 255, -200, 200);
            if (sNewGPSM != g_sGPSMController) {
              g_sGPSMController = sNewGPSM;
              g_ServoDriver.GPSetSpeedMultiplyer(g_sGPSMController);
            }

          }
        }

        //Switch between sequences
        if (ps2x.ButtonPressed(PSB_SELECT)) { // Select Button Test
          if (!g_ServoDriver.FIsGPSeqActive() ) {
            if (GPSeq < 5) {  //Max sequence
              MSound(1, 50, 1500);
              GPSeq = GPSeq+1;
            } 
            else {
              MSound(2, 50, 2000, 50, 2250);
              GPSeq=0;
            }
          }
        }
        //Start Sequence
        if (ps2x.ButtonPressed(PSB_R2))// R2 Button Test
          if (!g_ServoDriver.FIsGPSeqActive() ) {
          g_ServoDriver.GPStartSeq(GPSeq);
            g_sGPSMController = 32767;  // Say that we are not in Speed modify mode yet... valid ranges are 50-200 (both postive and negative... 
          }
          else {
            g_ServoDriver.GPStartSeq(0xff);    // tell the GP system to abort if possible...
            MSound (2, 50, 2000, 50, 2000);
          }


      }
#endif // OPT_GPPLAYER

      //Calculate walking time delay
      g_InControlState.InputTimeDelay = 128 - max(max(abs(ps2x.Analog(PSS_LX) - 128), abs(ps2x.Analog(PSS_LY) - 128)), abs(ps2x.Analog(PSS_RX) - 128));
    }

    //Calculate g_InControlState.BodyPos.y
    g_InControlState.BodyPos.y = min(max(g_BodyYOffset + g_BodyYShift,  0), MAX_BODY_Y);
    if (fAdjustLegPositions)
      AdjustLegPositionsToBodyHeight();    // Put main workings into main program file
  } 
  else {
    // We may have lost the PS2... See what we can do to recover...
    if (g_sPS2ErrorCnt < MAXPS2ERRORCNT)
      g_sPS2ErrorCnt++;    // Increment the error count and if to many errors, turn off the robot.
    else if (g_InControlState.fHexOn)
      PS2TurnRobotOff();
    ps2x.reconfig_gamepad();
  }
}

//==============================================================================
// PS2TurnRobotOff - code used couple of places so save a little room...
//==============================================================================
void PS2TurnRobotOff(void)
{
  //Turn off
  g_InControlState.BodyPos.x = 0;
  g_InControlState.BodyPos.y = 0;
  g_InControlState.BodyPos.z = 0;
  g_InControlState.BodyRot1.x = 0;
  g_InControlState.BodyRot1.y = 0;
  g_InControlState.BodyRot1.z = 0;
  g_InControlState.TravelLength.x = 0;
  g_InControlState.TravelLength.z = 0;
  g_InControlState.TravelLength.y = 0;
  g_BodyYOffset = 0;
  g_BodyYShift = 0;
  g_InControlState.SelectedLeg = 255;
  g_InControlState.fHexOn = 0;
  AdjustLegPositionsToBodyHeight();    // Put main workings into main program file
#if LED_EYE_PWM_PIN!=0
if(EyeOn)
{
 EyeOn =false;
 SetEyes();
}
#endif
  setup();
}





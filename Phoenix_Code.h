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
// Phoenix_Code.h
//
//     This contains the main code for the Phoenix project.  It is included in
//     all of the different configurations of the phoenix code.
//
//NEW IN V2.X
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

#include <EEPROM.h>
#include <pins_arduino.h>
#include <avr/pgmspace.h>

#define BALANCE_DIV_FACTOR  6    //;Other values than 6 can be used, testing...CAUTION!! At your own risk ;)


//--------------------------------------------------------------------
//[TABLES]
//ArcCosinus Table
//Table build in to 3 part to get higher accuracy near cos = 1.
//The biggest error is near cos = 1 and has a biggest value of 3*0.012098rad = 0.521 deg.
//-    Cos 0 to 0.9 is done by steps of 0.0079 rad. [1/127]
//-    Cos 0.9 to 0.99 is done by steps of 0.0008 rad [0.1/127]
//-    Cos 0.99 to 1 is done by step of 0.0002 rad [0.01/64]
//Since the tables are overlapping the full range of 127+127+64 is not necessary. Total u8s: 277

static const u8 TBL_ACOS[] PROGMEM = {
  255,254,252,251,250,249,247,246,245,243,242,241,240,238,237,236,234,233,232,231,229,228,227,225,
  224,223,221,220,219,217,216,215,214,212,211,210,208,207,206,204,203,201,200,199,197,196,195,193,
  192,190,189,188,186,185,183,182,181,179,178,176,175,173,172,170,169,167,166,164,163,161,160,158,
  157,155,154,152,150,149,147,146,144,142,141,139,137,135,134,132,130,128,127,125,123,121,119,117,
  115,113,111,109,107,105,103,101,98,96,94,92,89,87,84,81,79,76,73,73,73,72,72,72,71,71,71,70,70,
  70,70,69,69,69,68,68,68,67,67,67,66,66,66,65,65,65,64,64,64,63,63,63,62,62,62,61,61,61,60,60,59,
  59,59,58,58,58,57,57,57,56,56,55,55,55,54,54,53,53,53,52,52,51,51,51,50,50,49,49,48,48,47,47,47,
  46,46,45,45,44,44,43,43,42,42,41,41,40,40,39,39,38,37,37,36,36,35,34,34,33,33,32,31,31,30,29,28,
  28,27,26,25,24,23,23,23,23,22,22,22,22,21,21,21,21,20,20,20,19,19,19,19,18,18,18,17,17,17,17,16,
  16,16,15,15,15,14,14,13,13,13,12,12,11,11,10,10,9,9,8,7,6,6,5,3,0 };//

//Sin table 90 deg, persision 0.5 deg [180 values]
static const u16 TBL_SIN[] PROGMEM = {
  0, 87, 174, 261, 348, 436, 523, 610, 697, 784, 871, 958, 1045, 1132, 1218, 1305, 1391, 1478, 1564,
  1650, 1736, 1822, 1908, 1993, 2079, 2164, 2249, 2334, 2419, 2503, 2588, 2672, 2756, 2840, 2923, 3007,
  3090, 3173, 3255, 3338, 3420, 3502, 3583, 3665, 3746, 3826, 3907, 3987, 4067, 4146, 4226, 4305, 4383,
  4461, 4539, 4617, 4694, 4771, 4848, 4924, 4999, 5075, 5150, 5224, 5299, 5372, 5446, 5519, 5591, 5664,
  5735, 5807, 5877, 5948, 6018, 6087, 6156, 6225, 6293, 6360, 6427, 6494, 6560, 6626, 6691, 6755, 6819,
  6883, 6946, 7009, 7071, 7132, 7193, 7253, 7313, 7372, 7431, 7489, 7547, 7604, 7660, 7716, 7771, 7826,
  7880, 7933, 7986, 8038, 8090, 8141, 8191, 8241, 8290, 8338, 8386, 8433, 8480, 8526, 8571, 8616, 8660,
  8703, 8746, 8788, 8829, 8870, 8910, 8949, 8987, 9025, 9063, 9099, 9135, 9170, 9205, 9238, 9271, 9304,
  9335, 9366, 9396, 9426, 9455, 9483, 9510, 9537, 9563, 9588, 9612, 9636, 9659, 9681, 9702, 9723, 9743,
  9762, 9781, 9799, 9816, 9832, 9848, 9862, 9876, 9890, 9902, 9914, 9925, 9935, 9945, 9953, 9961, 9969,
  9975, 9981, 9986, 9990, 9993, 9996, 9998, 9999, 10000 };//


//Build tables for Leg configuration like I/O and MIN/imax values to easy access values using a FOR loop
//Constants are still defined as single values in the cfg file to make it easy to read/configure

// Servo Horn offsets
#ifdef OFFSET_RR_FEMUR_HORN   // per leg configuration
    static const short TBL_OFFSET_FEMUR_HORN[] PROGMEM = {
        OFFSET_RR_FEMUR_HORN,
        OFFSET_RM_FEMUR_HORN,
        OFFSET_RF_FEMUR_HORN,
        OFFSET_LR_FEMUR_HORN,
        OFFSET_LM_FEMUR_HORN,
        OFFSET_LF_FEMUR_HORN
    };
    #define OFFSET_FEMUR_HORN(LEGI) ((short)pgm_read_word(&TBL_OFFSET_FEMUR_HORN[LEGI]))
#else   // Fixed per leg, if not defined 0
    #define OFFSET_FEMUR_HORN(LEGI)  (0)
#endif

// Servo TARS offsets
#ifdef CONFIG_4DOF
#ifdef OFFSET_RR_TARS_HORN   // per leg configuration
    static const short TBL_OFFSET_TARS_HORN[] PROGMEM = {
        OFFSET_RR_TARS_HORN,
        cRMTarsHornOffset1,
        cRFTarsHornOffset1,
        cLRTarsHornOffset1,
        cLMTarsHornOffset1,
        cLFTarsHornOffset1
    };
    #define OFFSET_TARS_HORN(LEGI) ((short)pgm_read_word(&TBL_OFFSET_TARS_HORN[LEGI]))
#else
    #define OFFSET_TARS_HORN(LEGI)  (0)
#endif
#endif

// min max tables
static const short TBL_COXA_MIN[] PROGMEM = {
  cRRCoxaMin1,  cRMCoxaMin1,  cRFCoxaMin1,  cLRCoxaMin1,  cLMCoxaMin1,  cLFCoxaMin1};
static const short TBL_COXA_MAX[] PROGMEM = {
  cRRCoxaMax1,  cRMCoxaMax1,  cRFCoxaMax1,  cLRCoxaMax1,  cLMCoxaMax1,  cLFCoxaMax1};
static const short TBL_FEMUR_MIN[] PROGMEM ={
  cRRFemurMin1, cRMFemurMin1, cRFFemurMin1, cLRFemurMin1, cLMFemurMin1, cLFFemurMin1};
static const short TBL_FEMUR_MAX[] PROGMEM ={
  cRRFemurMax1, cRMFemurMax1, cRFFemurMax1, cLRFemurMax1, cLMFemurMax1, cLFFemurMax1};
static const short TBL_TIBIA_MIN[] PROGMEM ={
  cRRTibiaMin1, cRMTibiaMin1, cRFTibiaMin1, cLRTibiaMin1, cLMTibiaMin1, cLFTibiaMin1};
static const short TBL_TIBIA_MAX[] PROGMEM = {
  cRRTibiaMax1, cRMTibiaMax1, cRFTibiaMax1, cLRTibiaMax1, cLMTibiaMax1, cLFTibiaMax1};

#ifdef CONFIG_4DOF
    static const short TBL_TARS_MIN[] PROGMEM = {
      cRRTarsMin1, cRMTarsMin1, cRFTarsMin1, cLRTarsMin1, cLMTarsMin1, cLFTarsMin1};
    static const short TBL_TARS_MAX[] PROGMEM = {
      cRRTarsMax1, cRMTarsMax1, cRFTarsMax1, cLRTarsMax1, cLMTarsMax1, cLFTarsMax1};
#endif


// leg length tables
static const u8 TBL_COXA_LENGTH[] PROGMEM = {
  cRRCoxaLength,  cRMCoxaLength,  cRFCoxaLength,  cLRCoxaLength,  cLMCoxaLength,  cLFCoxaLength};
static const u8 TBL_FEMUR_LENGTH[] PROGMEM = {
  cRRFemurLength, cRMFemurLength, cRFFemurLength, cLRFemurLength, cLMFemurLength, cLFFemurLength};
static const u8 TBL_TIBIA_LENGTH[] PROGMEM = {
  cRRTibiaLength, cRMTibiaLength, cRFTibiaLength, cLRTibiaLength, cLMTibiaLength, cLFTibiaLength};
#ifdef CONFIG_4DOF
static const u8 TBL_TARS_LENGTH[] PROGMEM = {
  cRRTarsLength, cRMTarsLength, cRFTarsLength, cLRTarsLength, cLMTarsLength, cLFTarsLength};
#endif


// Body Offsets [distance between the center of the body and the center of the coxa]
static const short TBL_OFFSET_X[] PROGMEM = {
  cRROffsetX, cRMOffsetX, cRFOffsetX, cLROffsetX, cLMOffsetX, cLFOffsetX};
static const short TBL_OFFSET_Z[] PROGMEM = {
  cRROffsetZ, cRMOffsetZ, cRFOffsetZ, cLROffsetZ, cLMOffsetZ, cLFOffsetZ};

// Default leg angle
static const short TBL_COXA_ANGLE[] PROGMEM = {
  cRRCoxaAngle1, cRMCoxaAngle1, cRFCoxaAngle1, cLRCoxaAngle1, cLMCoxaAngle1, cLFCoxaAngle1};

// Start positions for the leg
static const short TBL_INT_POS_X[] PROGMEM = {
  cRRInitPosX, cRMInitPosX, cRFInitPosX, cLRInitPosX, cLMInitPosX, cLFInitPosX};
static const short TBL_INT_POS_Y[] PROGMEM = {
  cRRInitPosY, cRMInitPosY, cRFInitPosY, cLRInitPosY, cLMInitPosY, cLFInitPosY};
static const short TBL_INT_POS_Z[] PROGMEM = {
  cRRInitPosZ, cRMInitPosZ, cRFInitPosZ, cLRInitPosZ, cLMInitPosZ, cLFInitPosZ};


// Define some globals for debug information
boolean g_fShowDebugPrompt;
boolean g_fDebugOutput = true;
boolean g_fEnableServos = true;

//--------------------------------------------------------------------
//[REMOTE]
#define TRAVEL_DEAD_ZONE    4    //The deadzone for the analog input from the remote

//====================================================================
//[ANGLES]
short           CoxaAngle1[6];    //Actual Angle of the horizontal hip, decimals = 1
short           FemurAngle1[6];   //Actual Angle of the vertical hip, decimals = 1
short           TibiaAngle1[6];   //Actual Angle of the knee, decimals = 1
#ifdef CONFIG_4DOF
short           TarsAngle1[6];	  //Actual Angle of the knee, decimals = 1
#endif

//--------------------------------------------------------------------
//[POSITIONS SINGLE LEG CONTROL]

short           LegPosX[6];    //Actual X Posion of the Leg
short           LegPosY[6];    //Actual Y Posion of the Leg
short           LegPosZ[6];    //Actual Z Posion of the Leg
//--------------------------------------------------------------------
//[INPUTS]

//--------------------------------------------------------------------
//[GP PLAYER]
//--------------------------------------------------------------------
//[OUTPUTS]
boolean         LedA;    //Red
boolean         LedB;    //Green
boolean         LedC;    //Orange
boolean         Eyes;    //Eyes output
//--------------------------------------------------------------------
//[VARIABLES]
u8            Index;                    //Index universal used
u8            LegIndex;                //Index used for leg Index Number

//GetSinCos / ArcCos
short           AngleDeg1;        //Input Angle in degrees, decimals = 1
short           sin4;             //Output Sinus of the given Angle, decimals = 4
short           cos4;            //Output Cosinus of the given Angle, decimals = 4
short           AngleRad4;        //Output Angle in radials, decimals = 4

//GetAtan2
short           AtanX;            //Input X
short           AtanY;            //Input Y
short           Atan4;            //ArcTan2 output
long            XYhyp2;            //Output presenting Hypotenuse of X and Y

//Body Inverse Kinematics
short           PosX;            //Input position of the feet X
short           PosZ;            //Input position of the feet Z
short           PosY;            //Input position of the feet Y
long            BodyFKPosX;        //Output Position X of feet with Rotation
long            BodyFKPosY;        //Output Position Y of feet with Rotation
long            BodyFKPosZ;        //Output Position Z of feet with Rotation


//Leg Inverse Kinematics
long            IKFeetPosX;        //Input position of the Feet X
long            IKFeetPosY;        //Input position of the Feet Y
long            IKFeetPosZ;        //Input Position of the Feet Z
boolean         IKSolution;        //Output true if the solution is possible
boolean         IKSolutionWarning;    //Output true if the solution is NEARLY possible
boolean         IKSolutionError;    //Output true if the solution is NOT possible
//--------------------------------------------------------------------
//[TIMING]
unsigned long   lTimerStart;    //Start time of the calculation cycles
unsigned long   lTimerEnd;        //End time of the calculation cycles
u8            CycleTime;        //Total Cycle time

word            ServoMoveTime;        //Time for servo updates
word            PrevServoMoveTime;    //Previous time for the servo updates

//--------------------------------------------------------------------
//[GLOABAL]
//--------------------------------------------------------------------

// Define our global Input Control State object
INCONTROLSTATE   g_InControlState;      // This is our global Input control state object...

// Define our ServoWriter class
ServoDriver  g_ServoDriver;      // our global servo driver class

boolean         g_fLowVoltageShutdown;    // If set the bot shuts down because the input voltage is to low
word            Voltage;


//--boolean         g_InControlState.fHexOn;            //Switch to turn on Phoenix
//--boolean         g_InControlState.fPrev_HexOn;        //Previous loop state
//--------------------------------------------------------------------
//[Balance]
long            TotalTransX;
long            TotalTransZ;
long            TotalTransY;
long            TotalYBal1;
long            TotalXBal1;
long            TotalZBal1;

//[Single Leg Control]
u8            PrevSelectedLeg;
boolean         AllDown;

//[gait]

short		NomGaitSpeed;		//Nominal speed of the gait
short           TLDivFactor;         //Number of steps that a leg is on the floor while walking
short           NrLiftedPos;         //Number of positions that a single leg is lifted [1-3]
u8            LiftDivFactor;       //Normaly: 2, when NrLiftedPos=5: 4
u8            FrontDownPos;        //Where the leg should be put down to ground

boolean         HalfLiftHeigth;      //If TRUE the outer positions of the ligted legs will be half height

boolean         TravelRequest;        //Temp to check if the gait is in motion
u8            StepsInGait;         //Number of steps in gait

boolean         LastLeg;             //TRUE when the current leg is the last leg of the sequence
u8            GaitStep;            //Actual Gait step

u8            GaitLegNr[6];        //Init position of the leg

u8            GaitLegNrIn;         //Input Number of the leg

long            GaitPosX[6];         //Array containing Relative X position corresponding to the Gait
long            GaitPosY[6];         //Array containing Relative Y position corresponding to the Gait
long            GaitPosZ[6];         //Array containing Relative Z position corresponding to the Gait
long            GaitRotY[6];         //Array containing Relative Y rotation corresponding to the Gait


boolean         fWalking;            //  True if the robot are walking
u8            bExtraCycle;          // Forcing some extra timed cycles for avoiding "end of gait bug"
#define         cGPlimit 2           // GP=GaitPos testing different limits

boolean        g_fRobotUpsideDown;    // Is the robot upside down?
boolean        fRobotUpsideDownPrev;
//=============================================================================
// Function prototypes
//=============================================================================
extern void GaitSelect(void);
extern void  WriteOutputs(void);
extern void SingleLegControl(void);
extern void GaitSeq(void);
extern void BalanceBody(void);
extern void CheckAngles();

extern void    PrintSystemStuff(void);            // Try to see why we fault...


extern void BalCalcOneLeg (long PosX, long PosZ, long PosY, u8 BalLegNr);
extern void BodyFK (short PosX, short PosZ, short PosY, short RotationY, u8 BodyIKLeg) ;
extern void LegIK (short IKFeetPosX, short IKFeetPosY, short IKFeetPosZ, u8 LegIKLegNr);
extern void Gait (u8 GaitCurrentLegNr);
extern short GetATan2 (short AtanX, short AtanY);

extern void StartUpdateServos(void);
extern boolean TerminalMonitor(void);

//--------------------------------------------------------------------------
// SETUP: the main arduino setup function.
//--------------------------------------------------------------------------
void setup(){
#ifdef OPT_SKETCHSETUP
  SketchSetup();
#endif
  g_fShowDebugPrompt = true;
  g_fDebugOutput = true;
#ifdef DBG_SERIAL
  DBG_SERIAL.begin(BAUD_DEBUG);
#endif
  // Init our ServoDriver
  g_ServoDriver.Init();

  //Checks to see if our Servo Driver support a GP Player
  //    DBG_SERIAL.write("Program Start\n\r");
  // debug stuff
  delay(10);


  //Turning off all the leds
  LedA = 0;
  LedB = 0;
  LedC = 0;
  Eyes = 0;

  //Tars Init Positions
  for (LegIndex= 0; LegIndex <= 5; LegIndex++ )
  {
    LegPosX[LegIndex] = (short)pgm_read_word(&TBL_INT_POS_X[LegIndex]);    //Set start positions for each leg
    LegPosY[LegIndex] = (short)pgm_read_word(&TBL_INT_POS_Y[LegIndex]);
    LegPosZ[LegIndex] = (short)pgm_read_word(&TBL_INT_POS_Z[LegIndex]);
  }

  //Single leg control. Make sure no leg is selected
  g_InControlState.SelectedLeg = 255; // No Leg selected
  PrevSelectedLeg = 255;

  //Body Positions
  g_InControlState.BodyPos.x = 0;
  g_InControlState.BodyPos.y = 0;
  g_InControlState.BodyPos.z = 0;

  //Body Rotations
  g_InControlState.BodyRot1.x = 0;
  g_InControlState.BodyRot1.y = 0;
  g_InControlState.BodyRot1.z = 0;
  g_InControlState.BodyRotOffset.x = 0;
  g_InControlState.BodyRotOffset.y = 0;        //Input Y offset value to adjust centerpoint of rotation
  g_InControlState.BodyRotOffset.z = 0;


  //Gait
  g_InControlState.GaitType = 1;  // 0; Devon wanted
  g_InControlState.BalanceMode = 0;
  g_InControlState.LegLiftHeight = 50;
  g_InControlState.ForceGaitStepCnt = 0;    // added to try to adjust starting positions depending on height...
  GaitStep = 1;
  GaitSelect();

  g_InputController.Init();

  // Servo Driver
  ServoMoveTime = 150;
  g_InControlState.fHexOn = 0;
  g_fLowVoltageShutdown = false;
#ifdef DEBUG_IOPINS
  //  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
#endif

#ifdef OPT_WALK_UPSIDE_DOWN
  g_fRobotUpsideDown = false; //Assume off...
  printf(F("up_side_down:%d\n"), IsRobotUpsideDown);
#endif


  pinMode(PIN_STATUS_LED, OUTPUT);
}


//=============================================================================
// Loop: the main arduino main Loop function
//=============================================================================


void loop(void)
{
  //Start time
  lTimerStart = millis();
  DoBackgroundProcess();
  //Read input
  CheckVoltage();        // check our voltages...
  if (!g_fLowVoltageShutdown) {
    //    WRITE_PIN(A0, HIGH);
    g_InputController.ControlInput();
    //    WRITE_PIN(A0, LOW);
  }
  WriteOutputs();        // Write Outputs

#ifdef IsRobotUpsideDown
    if (!fWalking){// dont do this while walking
    g_fRobotUpsideDown = IsRobotUpsideDown;    // Grab the current state of the robot...
    if (g_fRobotUpsideDown != fRobotUpsideDownPrev) {
      // Double check to make sure that it was not a one shot error
      g_fRobotUpsideDown = IsRobotUpsideDown;    // Grab the current state of the robot...
      if (g_fRobotUpsideDown != fRobotUpsideDownPrev) {
        fRobotUpsideDownPrev = g_fRobotUpsideDown;
        printf(F("up_side_down_prev:%d"), fRobotUpsideDownPrev);
      }
    }
  }
  //  DBG_SERIAL.println(analogRead(0), DEC);
#endif
#ifdef OPT_WALK_UPSIDE_DOWN
  if (g_fRobotUpsideDown){
    g_InControlState.TravelLength.x = -g_InControlState.TravelLength.x;
    g_InControlState.BodyPos.x = -g_InControlState.BodyPos.x;
    g_InControlState.SLLeg.x = -g_InControlState.SLLeg.x;
    g_InControlState.BodyRot1.z = -g_InControlState.BodyRot1.z;
  }
#endif

#ifdef OPT_GPPLAYER
    //GP Player
  g_ServoDriver.GPPlayer();
  if (g_ServoDriver.FIsGPSeqActive())
    return;  // go back to process the next message
#endif

  //Single leg control
  SingleLegControl ();
  DoBackgroundProcess();

  //Gait
  GaitSeq();

  DoBackgroundProcess();

  //Balance calculations
  TotalTransX = 0;     //reset values used for calculation of balance
  TotalTransZ = 0;
  TotalTransY = 0;
  TotalXBal1 = 0;
  TotalYBal1 = 0;
  TotalZBal1 = 0;
  if (g_InControlState.BalanceMode) {
    for (LegIndex = 0; LegIndex <= 2; LegIndex++) {    // balance calculations for all Right legs

      DoBackgroundProcess();
      BalCalcOneLeg (-LegPosX[LegIndex]+GaitPosX[LegIndex],
      LegPosZ[LegIndex]+GaitPosZ[LegIndex],
      (LegPosY[LegIndex]-(short)pgm_read_word(&TBL_INT_POS_Y[LegIndex]))+GaitPosY[LegIndex], LegIndex);
    }

    for (LegIndex = 3; LegIndex <= 5; LegIndex++) {    // balance calculations for all Right legs
      DoBackgroundProcess();
      BalCalcOneLeg(LegPosX[LegIndex]+GaitPosX[LegIndex],
      LegPosZ[LegIndex]+GaitPosZ[LegIndex],
      (LegPosY[LegIndex]-(short)pgm_read_word(&TBL_INT_POS_Y[LegIndex]))+GaitPosY[LegIndex], LegIndex);
    }
    BalanceBody();
  }


  //Reset IKsolution indicators
  IKSolution = 0 ;
  IKSolutionWarning = 0;
  IKSolutionError = 0 ;

  //Do IK for all Right legs
  for (LegIndex = 0; LegIndex <=2; LegIndex++) {
    DoBackgroundProcess();
    BodyFK(-LegPosX[LegIndex]+g_InControlState.BodyPos.x+GaitPosX[LegIndex] - TotalTransX,
    LegPosZ[LegIndex]+g_InControlState.BodyPos.z+GaitPosZ[LegIndex] - TotalTransZ,
    LegPosY[LegIndex]+g_InControlState.BodyPos.y+GaitPosY[LegIndex] - TotalTransY,
    GaitRotY[LegIndex], LegIndex);

    LegIK (LegPosX[LegIndex]-g_InControlState.BodyPos.x+BodyFKPosX-(GaitPosX[LegIndex] - TotalTransX),
    LegPosY[LegIndex]+g_InControlState.BodyPos.y-BodyFKPosY+GaitPosY[LegIndex] - TotalTransY,
    LegPosZ[LegIndex]+g_InControlState.BodyPos.z-BodyFKPosZ+GaitPosZ[LegIndex] - TotalTransZ, LegIndex);
  }

  //Do IK for all Left legs
  for (LegIndex = 3; LegIndex <=5; LegIndex++) {
    DoBackgroundProcess();
    BodyFK(LegPosX[LegIndex]-g_InControlState.BodyPos.x+GaitPosX[LegIndex] - TotalTransX,
    LegPosZ[LegIndex]+g_InControlState.BodyPos.z+GaitPosZ[LegIndex] - TotalTransZ,
    LegPosY[LegIndex]+g_InControlState.BodyPos.y+GaitPosY[LegIndex] - TotalTransY,
    GaitRotY[LegIndex], LegIndex);
    LegIK (LegPosX[LegIndex]+g_InControlState.BodyPos.x-BodyFKPosX+GaitPosX[LegIndex] - TotalTransX,
    LegPosY[LegIndex]+g_InControlState.BodyPos.y-BodyFKPosY+GaitPosY[LegIndex] - TotalTransY,
    LegPosZ[LegIndex]+g_InControlState.BodyPos.z-BodyFKPosZ+GaitPosZ[LegIndex] - TotalTransZ, LegIndex);
  }
#ifdef OPT_WALK_UPSIDE_DOWN
  if (g_fRobotUpsideDown){ //Need to set them back for not messing with the SmoothControl
    g_InControlState.BodyPos.x = -g_InControlState.BodyPos.x;
    g_InControlState.SLLeg.x = -g_InControlState.SLLeg.x;
    g_InControlState.BodyRot1.z = -g_InControlState.BodyRot1.z;
  }
#endif
  //Check mechanical limits
  CheckAngles();

  //Write IK errors to leds
  LedC = IKSolutionWarning;
  LedA = IKSolutionError;

  // TJ
  //g_InControlState.fHexOn = 1;

  //Drive Servos
  if (g_InControlState.fHexOn) {
    if (g_InControlState.fHexOn && !g_InControlState.fPrev_HexOn) {
      digitalWrite(PIN_STATUS_LED, 1);
      Utils::sound(3, 60, 2000, 80, 2250, 100, 2500);
#ifdef USEXBEE
      XBeePlaySounds(3, 60, 2000, 80, 2250, 100, 2500);
#endif

      Eyes = 1;
    }

    //Calculate Servo Move time
    if ((abs(g_InControlState.TravelLength.x)>TRAVEL_DEAD_ZONE) || (abs(g_InControlState.TravelLength.z)>TRAVEL_DEAD_ZONE) ||
      (abs(g_InControlState.TravelLength.y*2)>TRAVEL_DEAD_ZONE)) {
      ServoMoveTime = NomGaitSpeed + (g_InControlState.InputTimeDelay*2) + g_InControlState.SpeedControl;

      //Add aditional delay when Balance mode is on
      if (g_InControlState.BalanceMode)
        ServoMoveTime = ServoMoveTime + 100;
    }
    else //Movement speed excl. Walking
    ServoMoveTime = 200 + g_InControlState.SpeedControl;

    // note we broke up the servo driver into start/commit that way we can output all of the servo information
    // before we wait and only have the termination information to output after the wait.  That way we hopefully
    // be more accurate with our timings...
    DoBackgroundProcess();
    StartUpdateServos();

    // See if we need to sync our processor with the servo driver while walking to ensure the prev is completed
    //before sending the next one


    // Finding any incident of GaitPos/Rot <>0:
    for (LegIndex = 0; LegIndex <= 5; LegIndex++) {
      if ( (GaitPosX[LegIndex] > cGPlimit) || (GaitPosX[LegIndex] < -cGPlimit)
        || (GaitPosZ[LegIndex] > cGPlimit) || (GaitPosZ[LegIndex] < -cGPlimit)
        || (GaitRotY[LegIndex] > cGPlimit) || (GaitRotY[LegIndex] < -cGPlimit))    {

        bExtraCycle = NrLiftedPos + 1;//For making sure that we are using timed move until all legs are down
        break;
      }
    }
    if (bExtraCycle>0){
      unsigned long lTimeWaitEnd;
      bExtraCycle--;
      fWalking = !(bExtraCycle==0);

      //Get endtime and calculate wait time
      lTimeWaitEnd = lTimerStart + PrevServoMoveTime;

      WRITE_PIN(A1, HIGH);
      do {
        // Wait the appropriate time, call any background process while waiting...
        DoBackgroundProcess();
      }
      while (millis() < lTimeWaitEnd);
      WRITE_PIN(A1, LOW);
#ifdef DEBUG
      if (g_fDebugOutput) {
        printf(F("BRX:%d, Walk:%d, GS:%d\n"), g_InControlState.BodyRot1.x, fWalking, GaitStep);
        printf(F("LEFT  GPX:%5d, GPY:%5d, GPZ:%5d\n"), GaitPosX[IDX_LF], GaitPosY[IDX_LF], GaitPosZ[IDX_LF]);
        printf(F("RIGHT GPX:%5d, GPY:%5d, GPZ:%5d\n"), GaitPosX[IDX_RF], GaitPosY[IDX_RF], GaitPosZ[IDX_RF]);
      }
#endif
    }
#ifdef DEBUG
    if (g_fDebugOutput) {
//      printf(F("TY:%5d, LFZ:%5d\n"), TotalYBal1, LegPosZ[IDX_LF]);
    }
#endif
    // Only do commit if we are actually doing something...
    TOGGLE_PIN(A2);
    g_ServoDriver.CommitServoDriver(ServoMoveTime);

  }
  else {
    //Turn the bot off - May need to add ajust here...
    if (g_InControlState.fPrev_HexOn || (AllDown= 0)) {
      ServoMoveTime = 600;
      StartUpdateServos();
      g_ServoDriver.CommitServoDriver(ServoMoveTime);
      Utils::sound(3, 100, 2500, 80, 2250, 60, 2000);
#ifdef USEXBEE
      XBeePlaySounds(3, 100, 2500, 80, 2250, 60, 2000);
#endif
      delay(600);
      digitalWrite(PIN_STATUS_LED, 0);
    }
    else {
      g_ServoDriver.FreeServos();
      Eyes = 0;
    }
    // We also have a simple debug monitor that allows us to
    // check things. call it here..
#ifdef OPT_TERMINAL_MONITOR
    if (TerminalMonitor())
      return;
#endif
    delay(20);  // give a pause between times we call if nothing is happening
  }

  PrevServoMoveTime = ServoMoveTime;

  //Store previous g_InControlState.fHexOn State
  if (g_InControlState.fHexOn)
    g_InControlState.fPrev_HexOn = 1;
  else
    g_InControlState.fPrev_HexOn = 0;
}


void StartUpdateServos()
{
  u8    LegIndex;

  // First call off to the init...
  g_ServoDriver.BeginServoUpdate();    // Start the update

    for (LegIndex = 0; LegIndex <= 5; LegIndex++) {
#ifdef CONFIG_4DOF
    g_ServoDriver.OutputServoInfoForLeg(LegIndex, CoxaAngle1[LegIndex], FemurAngle1[LegIndex], TibiaAngle1[LegIndex], TarsAngle1[LegIndex]);
#else
    g_ServoDriver.OutputServoInfoForLeg(LegIndex, CoxaAngle1[LegIndex], FemurAngle1[LegIndex], TibiaAngle1[LegIndex]);
//    g_ServoDriver.OutputServoInfoForLeg(LegIndex, 90, 90, 90);
#endif
  }
}




//--------------------------------------------------------------------
//[WriteOutputs] Updates the state of the leds
//--------------------------------------------------------------------
void WriteOutputs(void)
{
#ifdef cEyesPin
  digitalWrite(cEyesPin, Eyes);
#endif
}
//--------------------------------------------------------------------
//[CHECK VOLTAGE]
//Reads the input voltage and shuts down the bot when the power drops
u8 s_bLVBeepCnt;
boolean CheckVoltage() {
#ifdef VOLT_TURN_OFF
  // Moved to Servo Driver - BUGBUG: Need to do when I merge back...
  //    Voltage = analogRead(PIN_ANALOG_VOLT); // Battery voltage
  //    Voltage = ((long)Voltage*1955)/1000;
  Voltage = g_ServoDriver.GetBatteryVoltage();

  // BUGBUG:: if voltage is 0 it failed to retrieve don't hang program...
  //if (!Voltage)
  //  return 0;

  return 0;

  printf(F("VOLT:%d\n"), Voltage);
  if (!g_fLowVoltageShutdown) {
    if ((Voltage < VOLT_TURN_OFF) || (Voltage >= 1999)) {
      printf(F("Voltage went low, turn off robot :%d\n"), Voltage);

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
      g_InControlState.SelectedLeg = 255;
      g_fLowVoltageShutdown = 1;
      s_bLVBeepCnt = 0;    // how many times we beeped...
      g_InControlState.fHexOn = false;
    }
#ifdef VOLT_TURN_ON
  }
  else if ((Voltage > VOLT_TURN_ON) && (Voltage < 1999)) {
    printf(F("Voltage restored:%d"), Voltage);
    g_fLowVoltageShutdown = 0;
#endif
  }
  else {
    if (s_bLVBeepCnt < 5) {
      s_bLVBeepCnt++;
      printf(F("VOLT:%d\n"), Voltage);
      Utils::sound( 1, 45, 2000);
    }
    delay(2000);
  }
#endif

  return g_fLowVoltageShutdown;
}

//--------------------------------------------------------------------
//[SINGLE LEG CONTROL]
void SingleLegControl(void)
{

  //Check if all legs are down
  AllDown = (LegPosY[IDX_RF]==(short)pgm_read_word(&TBL_INT_POS_Y[IDX_RF])) &&
    (LegPosY[IDX_RM]==(short)pgm_read_word(&TBL_INT_POS_Y[IDX_RM])) &&
    (LegPosY[IDX_RR]==(short)pgm_read_word(&TBL_INT_POS_Y[IDX_RR])) &&
    (LegPosY[IDX_LR]==(short)pgm_read_word(&TBL_INT_POS_Y[IDX_LR])) &&
    (LegPosY[IDX_LM]==(short)pgm_read_word(&TBL_INT_POS_Y[IDX_LM])) &&
    (LegPosY[IDX_LF]==(short)pgm_read_word(&TBL_INT_POS_Y[IDX_LF]));

  if (g_InControlState.SelectedLeg<=5) {
    if (g_InControlState.SelectedLeg!=PrevSelectedLeg) {
      if (AllDown) { //Lift leg a bit when it got selected
        LegPosY[g_InControlState.SelectedLeg] = (short)pgm_read_word(&TBL_INT_POS_Y[g_InControlState.SelectedLeg])-20;

        //Store current status
        PrevSelectedLeg = g_InControlState.SelectedLeg;
      }
      else {//Return prev leg back to the init position
        LegPosX[PrevSelectedLeg] = (short)pgm_read_word(&TBL_INT_POS_X[PrevSelectedLeg]);
        LegPosY[PrevSelectedLeg] = (short)pgm_read_word(&TBL_INT_POS_Y[PrevSelectedLeg]);
        LegPosZ[PrevSelectedLeg] = (short)pgm_read_word(&TBL_INT_POS_Z[PrevSelectedLeg]);
      }
    }
    else if (!g_InControlState.fSLHold) {
      //LegPosY[g_InControlState.SelectedLeg] = LegPosY[g_InControlState.SelectedLeg]+g_InControlState.SLLeg.y;
      LegPosY[g_InControlState.SelectedLeg] = (short)pgm_read_word(&TBL_INT_POS_Y[g_InControlState.SelectedLeg])+g_InControlState.SLLeg.y;// Using DIY remote Zenta prefer it this way
      LegPosX[g_InControlState.SelectedLeg] = (short)pgm_read_word(&TBL_INT_POS_X[g_InControlState.SelectedLeg])+g_InControlState.SLLeg.x;
      LegPosZ[g_InControlState.SelectedLeg] = (short)pgm_read_word(&TBL_INT_POS_Z[g_InControlState.SelectedLeg])+g_InControlState.SLLeg.z;
    }
  }
  else {//All legs to init position
    if (!AllDown) {
      for(LegIndex = 0; LegIndex <= 5;LegIndex++) {
        LegPosX[LegIndex] = (short)pgm_read_word(&TBL_INT_POS_X[LegIndex]);
        LegPosY[LegIndex] = (short)pgm_read_word(&TBL_INT_POS_Y[LegIndex]);
        LegPosZ[LegIndex] = (short)pgm_read_word(&TBL_INT_POS_Z[LegIndex]);
      }
    }
    if (PrevSelectedLeg!=255)
      PrevSelectedLeg = 255;
  }
}

#ifndef DEFAULT_GAIT_SPEED
#define DEFAULT_GAIT_SPEED 60
#define DEFAULT_SLOW_GAIT 70
#endif
//--------------------------------------------------------------------
void GaitSelect(void)
{
  //Gait selector
  switch (g_InControlState.GaitType)  {
  case 0:
    //Ripple Gait 12 steps
    GaitLegNr[IDX_LR] = 1;
    GaitLegNr[IDX_RF] = 3;
    GaitLegNr[IDX_LM] = 5;
    GaitLegNr[IDX_RR] = 7;
    GaitLegNr[IDX_LF] = 9;
    GaitLegNr[IDX_RM] = 11;

    NrLiftedPos = 3;
    FrontDownPos = 2;
    LiftDivFactor = 2;
    HalfLiftHeigth = 3;
    TLDivFactor = 8;
    StepsInGait = 12;
    NomGaitSpeed = DEFAULT_SLOW_GAIT;
    break;
  case 1:
    //Tripod 8 steps
    GaitLegNr[IDX_LR] = 5;
    GaitLegNr[IDX_RF] = 1;
    GaitLegNr[IDX_LM] = 1;
    GaitLegNr[IDX_RR] = 1;
    GaitLegNr[IDX_LF] = 5;
    GaitLegNr[IDX_RM] = 5;

    NrLiftedPos = 3;
    FrontDownPos = 2;
    LiftDivFactor = 2;
    HalfLiftHeigth = 3;
    TLDivFactor = 4;
    StepsInGait = 8;
    NomGaitSpeed = DEFAULT_SLOW_GAIT;
    break;
  case 2:
    //Triple Tripod 12 step
    GaitLegNr[IDX_RF] = 3;
    GaitLegNr[IDX_LM] = 4;
    GaitLegNr[IDX_RR] = 5;
    GaitLegNr[IDX_LF] = 9;
    GaitLegNr[IDX_RM] = 10;
    GaitLegNr[IDX_LR] = 11;

    NrLiftedPos = 3;
    FrontDownPos = 2;
    LiftDivFactor = 2;
    HalfLiftHeigth = 3;
    TLDivFactor = 8;
    StepsInGait = 12;
    NomGaitSpeed = DEFAULT_GAIT_SPEED;
    break;
  case 3:
    // Triple Tripod 16 steps, use 5 lifted positions
    GaitLegNr[IDX_RF] = 4;
    GaitLegNr[IDX_LM] = 5;
    GaitLegNr[IDX_RR] = 6;
    GaitLegNr[IDX_LF] = 12;
    GaitLegNr[IDX_RM] = 13;
    GaitLegNr[IDX_LR] = 14;

    NrLiftedPos = 5;
    FrontDownPos = 3;
    LiftDivFactor = 4;
    HalfLiftHeigth = 1;
    TLDivFactor = 10;
    StepsInGait = 16;
    NomGaitSpeed = DEFAULT_GAIT_SPEED;
    break;
  case 4:
    //Wave 24 steps
    GaitLegNr[IDX_LR] = 1;
    GaitLegNr[IDX_RF] = 21;
    GaitLegNr[IDX_LM] = 5;

    GaitLegNr[IDX_RR] = 13;
    GaitLegNr[IDX_LF] = 9;
    GaitLegNr[IDX_RM] = 17;

    NrLiftedPos = 3;
    FrontDownPos = 2;
    LiftDivFactor = 2;
    HalfLiftHeigth = 3;
    TLDivFactor = 20;
    StepsInGait = 24;
    NomGaitSpeed = DEFAULT_SLOW_GAIT;
    break;
  case 5:
    //Tripod 6 steps
    GaitLegNr[IDX_LR] = 4;
    GaitLegNr[IDX_RF] = 1;
    GaitLegNr[IDX_LM] = 1;

    GaitLegNr[IDX_RR] = 1;
    GaitLegNr[IDX_LF] = 4;
    GaitLegNr[IDX_RM] = 4;

    NrLiftedPos = 2;
    FrontDownPos = 1;
    LiftDivFactor = 2;
    HalfLiftHeigth = 1;
    TLDivFactor = 4;
    StepsInGait = 6;
    NomGaitSpeed = DEFAULT_GAIT_SPEED;
    break;
  }
}

//--------------------------------------------------------------------
//[GAIT Sequence]
void GaitSeq(void)
{
  //Check if the Gait is in motion
  TravelRequest = (abs(g_InControlState.TravelLength.x)>TRAVEL_DEAD_ZONE) || (abs(g_InControlState.TravelLength.z)>TRAVEL_DEAD_ZONE)
    || (abs(g_InControlState.TravelLength.y)>TRAVEL_DEAD_ZONE) || (g_InControlState.ForceGaitStepCnt != 0) || fWalking;

  //Calculate Gait sequence
  LastLeg = 0;
  for (LegIndex = 0; LegIndex <= 5; LegIndex++) { // for all legs
    if (LegIndex == 5) // last leg
      LastLeg = 1 ;

    Gait(LegIndex);
  }    // next leg

  // If we have a force count decrement it now...
  if (g_InControlState.ForceGaitStepCnt)
    g_InControlState.ForceGaitStepCnt--;
}


//--------------------------------------------------------------------
//[GAIT]
void Gait (u8 GaitCurrentLegNr)
{


  //Clear values under the TRAVEL_DEAD_ZONE
  if (!TravelRequest) {
    g_InControlState.TravelLength.x=0;
    g_InControlState.TravelLength.z=0;
    g_InControlState.TravelLength.y=0;//Gait NOT in motion, return to home position
  }
  //Leg middle up position OK
  //Gait in motion

  if ((TravelRequest && (NrLiftedPos==1 || NrLiftedPos==3 || NrLiftedPos==5) &&
    GaitStep==GaitLegNr[GaitCurrentLegNr]) || (!TravelRequest && GaitStep==GaitLegNr[GaitCurrentLegNr] && ((abs(GaitPosX[GaitCurrentLegNr])>2) ||
    (abs(GaitPosZ[GaitCurrentLegNr])>2) || (abs(GaitRotY[GaitCurrentLegNr])>2)))) { //Up
    GaitPosX[GaitCurrentLegNr] = 0;
    GaitPosY[GaitCurrentLegNr] = -g_InControlState.LegLiftHeight;
    GaitPosZ[GaitCurrentLegNr] = 0;
    GaitRotY[GaitCurrentLegNr] = 0;
  }
  //Optional Half heigth Rear (2, 3, 5 lifted positions)
  else if (((NrLiftedPos==2 && GaitStep==GaitLegNr[GaitCurrentLegNr]) || (NrLiftedPos>=3 &&
    (GaitStep==GaitLegNr[GaitCurrentLegNr]-1 || GaitStep==GaitLegNr[GaitCurrentLegNr]+(StepsInGait-1))))
    && TravelRequest) {
    GaitPosX[GaitCurrentLegNr] = -g_InControlState.TravelLength.x/LiftDivFactor;
    GaitPosY[GaitCurrentLegNr] = -3*g_InControlState.LegLiftHeight/(3+HalfLiftHeigth);     //Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
    GaitPosZ[GaitCurrentLegNr] = -g_InControlState.TravelLength.z/LiftDivFactor;
    GaitRotY[GaitCurrentLegNr] = -g_InControlState.TravelLength.y/LiftDivFactor;
  }
  // _A_
  // Optional Half heigth front (2, 3, 5 lifted positions)
  else if ((NrLiftedPos>=2) && (GaitStep==GaitLegNr[GaitCurrentLegNr]+1 || GaitStep==GaitLegNr[GaitCurrentLegNr]-(StepsInGait-1)) && TravelRequest) {
    GaitPosX[GaitCurrentLegNr] = g_InControlState.TravelLength.x/LiftDivFactor;
    GaitPosY[GaitCurrentLegNr] = -3*g_InControlState.LegLiftHeight/(3+HalfLiftHeigth); // Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
    GaitPosZ[GaitCurrentLegNr] = g_InControlState.TravelLength.z/LiftDivFactor;
    GaitRotY[GaitCurrentLegNr] = g_InControlState.TravelLength.y/LiftDivFactor;
  }

  //Optional Half heigth Rear 5 LiftedPos (5 lifted positions)
  else if (((NrLiftedPos==5 && (GaitStep==GaitLegNr[GaitCurrentLegNr]-2 ))) && TravelRequest) {
    GaitPosX[GaitCurrentLegNr] = -g_InControlState.TravelLength.x/2;
    GaitPosY[GaitCurrentLegNr] = -g_InControlState.LegLiftHeight/2;
    GaitPosZ[GaitCurrentLegNr] = -g_InControlState.TravelLength.z/2;
    GaitRotY[GaitCurrentLegNr] = -g_InControlState.TravelLength.y/2;
  }

  //Optional Half heigth Front 5 LiftedPos (5 lifted positions)
  else if ((NrLiftedPos==5) && (GaitStep==GaitLegNr[GaitCurrentLegNr]+2 || GaitStep==GaitLegNr[GaitCurrentLegNr]-(StepsInGait-2)) && TravelRequest) {
    GaitPosX[GaitCurrentLegNr] = g_InControlState.TravelLength.x/2;
    GaitPosY[GaitCurrentLegNr] = -g_InControlState.LegLiftHeight/2;
    GaitPosZ[GaitCurrentLegNr] = g_InControlState.TravelLength.z/2;
    GaitRotY[GaitCurrentLegNr] = g_InControlState.TravelLength.y/2;
  }
  //_B_
  //Leg front down position //bug here?  From _A_ to _B_ there should only be one gaitstep, not 2!
  //For example, where is the case of GaitStep==GaitLegNr[GaitCurrentLegNr]+2 executed when NRLiftedPos=3?
  else if ((GaitStep==GaitLegNr[GaitCurrentLegNr]+FrontDownPos || GaitStep==GaitLegNr[GaitCurrentLegNr]-(StepsInGait-FrontDownPos))
    && GaitPosY[GaitCurrentLegNr]<0) {
    GaitPosX[GaitCurrentLegNr] = g_InControlState.TravelLength.x/2;
    GaitPosZ[GaitCurrentLegNr] = g_InControlState.TravelLength.z/2;
    GaitRotY[GaitCurrentLegNr] = g_InControlState.TravelLength.y/2;
    GaitPosY[GaitCurrentLegNr] = 0;

  }

  //Move body forward
  else {
    GaitPosX[GaitCurrentLegNr] = GaitPosX[GaitCurrentLegNr] - (g_InControlState.TravelLength.x/TLDivFactor);
    GaitPosY[GaitCurrentLegNr] = 0;
    GaitPosZ[GaitCurrentLegNr] = GaitPosZ[GaitCurrentLegNr] - (g_InControlState.TravelLength.z/TLDivFactor);
    GaitRotY[GaitCurrentLegNr] = GaitRotY[GaitCurrentLegNr] - (g_InControlState.TravelLength.y/TLDivFactor);
  }


  //Advance to the next step
  if (LastLeg)  {  //The last leg in this step
    GaitStep++;
    if (GaitStep>StepsInGait)
      GaitStep = 1;
  }
}


//--------------------------------------------------------------------
//[BalCalcOneLeg]
void BalCalcOneLeg (long PosX, long PosZ, long PosY, u8 BalLegNr)
{
  long            CPR_X;            //Final X value for centerpoint of rotation
  long            CPR_Y;            //Final Y value for centerpoint of rotation
  long            CPR_Z;            //Final Z value for centerpoint of rotation
  long             lAtan;

  //Calculating totals from center of the body to the feet
  CPR_Z = (short)pgm_read_word(&TBL_OFFSET_Z[BalLegNr]) + PosZ;
  CPR_X = (short)pgm_read_word(&TBL_OFFSET_X[BalLegNr]) + PosX;
  CPR_Y = 150 + PosY;        // using the value 150 to lower the centerpoint of rotation 'g_InControlState.BodyPos.y +

  TotalTransY += (long)PosY;
  TotalTransZ += (long)CPR_Z;
  TotalTransX += (long)CPR_X;

  lAtan = GetATan2(CPR_X, CPR_Z);
  TotalYBal1 += (lAtan*1800) / 31415;

  lAtan = GetATan2 (CPR_X, CPR_Y);
  TotalZBal1 += ((lAtan*1800) / 31415) -900; //Rotate balance circle 90 deg

  lAtan = GetATan2 (CPR_Z, CPR_Y);
  TotalXBal1 += ((lAtan*1800) / 31415) - 900; //Rotate balance circle 90 deg

}
//--------------------------------------------------------------------
//[BalanceBody]
void BalanceBody(void)
{
  TotalTransZ = TotalTransZ/BALANCE_DIV_FACTOR ;
  TotalTransX = TotalTransX/BALANCE_DIV_FACTOR;
  TotalTransY = TotalTransY/BALANCE_DIV_FACTOR;

  if (TotalYBal1 > 0)        //Rotate balance circle by +/- 180 deg
    TotalYBal1 -=  1800;
  else
    TotalYBal1 += 1800;

  if (TotalZBal1 < -1800)    //Compensate for extreme balance positions that causes owerflow
    TotalZBal1 += 3600;

  if (TotalXBal1 < -1800)    //Compensate for extreme balance positions that causes owerflow
    TotalXBal1 += 3600;

  //Balance rotation
  TotalYBal1 = -TotalYBal1/BALANCE_DIV_FACTOR;
  TotalXBal1 = -TotalXBal1/BALANCE_DIV_FACTOR;
  TotalZBal1 = TotalZBal1/BALANCE_DIV_FACTOR;
}


//--------------------------------------------------------------------
//[GETSINCOS] Get the sinus and cosinus from the angle +/- multiple circles
//AngleDeg1     - Input Angle in degrees
//sin4        - Output Sinus of AngleDeg
//cos4          - Output Cosinus of AngleDeg
void GetSinCos(short AngleDeg1)
{
  short        ABSAngleDeg1;    //Absolute value of the Angle in Degrees, decimals = 1
  //Get the absolute value of AngleDeg
  if (AngleDeg1 < 0)
    ABSAngleDeg1 = AngleDeg1 *-1;
  else
    ABSAngleDeg1 = AngleDeg1;

  //Shift rotation to a full circle of 360 deg -> AngleDeg // 360
  if (AngleDeg1 < 0)    //Negative values
    AngleDeg1 = 3600-(ABSAngleDeg1-(3600*(ABSAngleDeg1/3600)));
  else                //Positive values
  AngleDeg1 = ABSAngleDeg1-(3600*(ABSAngleDeg1/3600));

  if (AngleDeg1>=0 && AngleDeg1<=900)     // 0 to 90 deg
  {
    sin4 = pgm_read_word(&TBL_SIN[AngleDeg1/5]);             // 5 is the presision (0.5) of the table
    cos4 = pgm_read_word(&TBL_SIN[(900-(AngleDeg1))/5]);
  }

  else if (AngleDeg1>900 && AngleDeg1<=1800)     // 90 to 180 deg
  {
    sin4 = pgm_read_word(&TBL_SIN[(900-(AngleDeg1-900))/5]); // 5 is the presision (0.5) of the table
    cos4 = -pgm_read_word(&TBL_SIN[(AngleDeg1-900)/5]);
  }
  else if (AngleDeg1>1800 && AngleDeg1<=2700) // 180 to 270 deg
  {
    sin4 = -pgm_read_word(&TBL_SIN[(AngleDeg1-1800)/5]);     // 5 is the presision (0.5) of the table
    cos4 = -pgm_read_word(&TBL_SIN[(2700-AngleDeg1)/5]);
  }

  else if(AngleDeg1>2700 && AngleDeg1<=3600) // 270 to 360 deg
  {
    sin4 = -pgm_read_word(&TBL_SIN[(3600-AngleDeg1)/5]); // 5 is the presision (0.5) of the table
    cos4 = pgm_read_word(&TBL_SIN[(AngleDeg1-2700)/5]);
  }
}


//--------------------------------------------------------------------
//(GETARCCOS) Get the sinus and cosinus from the angle +/- multiple circles
//cos4        - Input Cosinus
//AngleRad4     - Output Angle in AngleRad4
long GetArcCos(short cos4)
{
  boolean NegativeValue/*:1*/;    //If the the value is Negative
  //Check for negative value
  if (cos4<0)
  {
    cos4 = -cos4;
    NegativeValue = 1;
  }
  else
    NegativeValue = 0;

  //Limit cos4 to his maximal value
  cos4 = min(cos4,DEC_EXP_4);

  if ((cos4>=0) && (cos4<9000))
  {
    AngleRad4 = (u8)pgm_read_byte(&TBL_ACOS[cos4/79]);
    AngleRad4 = ((long)AngleRad4*616)/DEC_EXP_1;            //616=acos resolution (pi/2/255) ;
  }
  else if ((cos4>=9000) && (cos4<9900))
  {
    AngleRad4 = (u8)pgm_read_byte(&TBL_ACOS[(cos4-9000)/8+114]);
    AngleRad4 = (long)((long)AngleRad4*616)/DEC_EXP_1;             //616=acos resolution (pi/2/255)
  }
  else if ((cos4>=9900) && (cos4<=10000))
  {
    AngleRad4 = (u8)pgm_read_byte(&TBL_ACOS[(cos4-9900)/2+227]);
    AngleRad4 = (long)((long)AngleRad4*616)/DEC_EXP_1;             //616=acos resolution (pi/2/255)
  }

  //Add negative sign
  if (NegativeValue)
    AngleRad4 = 31416 - AngleRad4;

  return AngleRad4;
}

unsigned long isqrt32 (unsigned long n) //
{
  unsigned long root;
  unsigned long remainder;
  unsigned long  place;

  root = 0;
  remainder = n;
  place = 0x40000000; // OR place = 0x4000; OR place = 0x40; - respectively

  while (place > remainder)
    place = place >> 2;
  while (place)
  {
    if (remainder >= root + place)
    {
      remainder = remainder - root - place;
      root = root + (place << 1);
    }
    root = root >> 1;
    place = place >> 2;
  }
  return root;
}


//--------------------------------------------------------------------
//(GETATAN2) Simplyfied ArcTan2 function based on fixed point ArcCos
//ArcTanX         - Input X
//ArcTanY         - Input Y
//ArcTan4          - Output ARCTAN2(X/Y)
//XYhyp2            - Output presenting Hypotenuse of X and Y
short GetATan2 (short AtanX, short AtanY)
{
  XYhyp2 = isqrt32(((long)AtanX*AtanX*DEC_EXP_4) + ((long)AtanY*AtanY*DEC_EXP_4));
  GetArcCos (((long)AtanX*(long)DEC_EXP_6) /(long) XYhyp2);

  if (AtanY < 0)                // removed overhead... Atan4 = AngleRad4 * (AtanY/abs(AtanY));
    Atan4 = -AngleRad4;
  else
    Atan4 = AngleRad4;
  return Atan4;
}

//--------------------------------------------------------------------
//(BODY INVERSE KINEMATICS)
//BodyRotX         - Global Input pitch of the body
//BodyRotY         - Global Input rotation of the body
//BodyRotZ         - Global Input roll of the body
//RotationY         - Input Rotation for the gait
//PosX            - Input position of the feet X
//PosZ            - Input position of the feet Z
//SinB                  - Sin buffer for BodyRotX
//CosB               - Cos buffer for BodyRotX
//SinG                  - Sin buffer for BodyRotZ
//CosG               - Cos buffer for BodyRotZ
//BodyFKPosX         - Output Position X of feet with Rotation
//BodyFKPosY         - Output Position Y of feet with Rotation
//BodyFKPosZ         - Output Position Z of feet with Rotation
void BodyFK (short PosX, short PosZ, short PosY, short RotationY, u8 BodyIKLeg)
{
  short            SinA4;          //Sin buffer for BodyRotX calculations
  short            CosA4;          //Cos buffer for BodyRotX calculations
  short            SinB4;          //Sin buffer for BodyRotX calculations
  short            CosB4;          //Cos buffer for BodyRotX calculations
  short            SinG4;          //Sin buffer for BodyRotZ calculations
  short            CosG4;          //Cos buffer for BodyRotZ calculations
  short             CPR_X;            //Final X value for centerpoint of rotation
  short            CPR_Y;            //Final Y value for centerpoint of rotation
  short            CPR_Z;            //Final Z value for centerpoint of rotation

  //Calculating totals from center of the body to the feet
  CPR_X = (short)pgm_read_word(&TBL_OFFSET_X[BodyIKLeg])+PosX + g_InControlState.BodyRotOffset.x;
  CPR_Y = PosY + g_InControlState.BodyRotOffset.y;         //Define centerpoint for rotation along the Y-axis
  CPR_Z = (short)pgm_read_word(&TBL_OFFSET_Z[BodyIKLeg]) + PosZ + g_InControlState.BodyRotOffset.z;

  //Successive global rotation matrix:
  //Math shorts for rotation: Alfa [A] = Xrotate, Beta [B] = Zrotate, Gamma [G] = Yrotate
  //Sinus Alfa = SinA, cosinus Alfa = cosA. and so on...

  //First calculate sinus and cosinus for each rotation:
  GetSinCos (g_InControlState.BodyRot1.x+TotalXBal1);
  SinG4 = sin4;
  CosG4 = cos4;

  GetSinCos (g_InControlState.BodyRot1.z+TotalZBal1);
  SinB4 = sin4;
  CosB4 = cos4;

#ifdef OPT_WALK_UPSIDE_DOWN
  if (g_fRobotUpsideDown)
    GetSinCos (-g_InControlState.BodyRot1.y+(-RotationY*DEC_EXP_1)+TotalYBal1) ;
  else
    GetSinCos (g_InControlState.BodyRot1.y+(RotationY*DEC_EXP_1)+TotalYBal1) ;
#else
  GetSinCos (g_InControlState.BodyRot1.y+(RotationY*DEC_EXP_1)+TotalYBal1) ;
#endif
  SinA4 = sin4;
  CosA4 = cos4;

  //Calcualtion of rotation matrix:
  BodyFKPosX = ((long)CPR_X*DEC_EXP_2 - ((long)CPR_X*DEC_EXP_2*CosA4/DEC_EXP_4*CosB4/DEC_EXP_4 - (long)CPR_Z*DEC_EXP_2*CosB4/DEC_EXP_4*SinA4/DEC_EXP_4
    + (long)CPR_Y*DEC_EXP_2*SinB4/DEC_EXP_4 ))/DEC_EXP_2;
  BodyFKPosZ = ((long)CPR_Z*DEC_EXP_2 - ( (long)CPR_X*DEC_EXP_2*CosG4/DEC_EXP_4*SinA4/DEC_EXP_4 + (long)CPR_X*DEC_EXP_2*CosA4/DEC_EXP_4*SinB4/DEC_EXP_4*SinG4/DEC_EXP_4
    + (long)CPR_Z*DEC_EXP_2*CosA4/DEC_EXP_4*CosG4/DEC_EXP_4 - (long)CPR_Z*DEC_EXP_2*SinA4/DEC_EXP_4*SinB4/DEC_EXP_4*SinG4/DEC_EXP_4
    - (long)CPR_Y*DEC_EXP_2*CosB4/DEC_EXP_4*SinG4/DEC_EXP_4 ))/DEC_EXP_2;
  BodyFKPosY = ((long)CPR_Y  *DEC_EXP_2 - ( (long)CPR_X*DEC_EXP_2*SinA4/DEC_EXP_4*SinG4/DEC_EXP_4 - (long)CPR_X*DEC_EXP_2*CosA4/DEC_EXP_4*CosG4/DEC_EXP_4*SinB4/DEC_EXP_4
    + (long)CPR_Z*DEC_EXP_2*CosA4/DEC_EXP_4*SinG4/DEC_EXP_4 + (long)CPR_Z*DEC_EXP_2*CosG4/DEC_EXP_4*SinA4/DEC_EXP_4*SinB4/DEC_EXP_4
    + (long)CPR_Y*DEC_EXP_2*CosB4/DEC_EXP_4*CosG4/DEC_EXP_4 ))/DEC_EXP_2;
}



//--------------------------------------------------------------------
//[LEG INVERSE KINEMATICS] Calculates the angles of the coxa, femur and tibia for the given position of the feet
//IKFeetPosX            - Input position of the Feet X
//IKFeetPosY            - Input position of the Feet Y
//IKFeetPosZ            - Input Position of the Feet Z
//IKSolution            - Output true if the solution is possible
//IKSolutionWarning     - Output true if the solution is NEARLY possible
//IKSolutionError    - Output true if the solution is NOT possible
//FemurAngle1           - Output Angle of Femur in degrees
//TibiaAngle1           - Output Angle of Tibia in degrees
//CoxaAngle1            - Output Angle of Coxa in degrees
//--------------------------------------------------------------------
void LegIK (short IKFeetPosX, short IKFeetPosY, short IKFeetPosZ, u8 LegIKLegNr)
{
  unsigned long    IKSW2;            //Length between Shoulder and Wrist, decimals = 2
  unsigned long    IKA14;            //Angle of the line S>W with respect to the ground in radians, decimals = 4
  unsigned long    IKA24;            //Angle of the line S>W with respect to the femur in radians, decimals = 4
  short            IKFeetPosXZ;    //Diagonal direction from Input X and Z
#ifdef CONFIG_4DOF
  // these were shorts...
  long            TarsOffsetXZ;    //Vector value \ ;
  long            TarsOffsetY;     //Vector value / The 2 DOF IK calcs (femur and tibia) are based upon these vectors
  long            TarsToGroundAngle1;    //Angle between tars and ground. Note: the angle are 0 when the tars are perpendicular to the ground
  long            TGA_A_H4;
  long            TGA_B_H3;
#else
#define TarsOffsetXZ 0		// Vector value
#define TarsOffsetY  0		//Vector value / The 2 DOF IK calcs (femur and tibia) are based upon these vectors
#endif


  long            Temp1;
  long            Temp2;
  long            T3;

  //Calculate IKCoxaAngle and IKFeetPosXZ
  GetATan2 (IKFeetPosX, IKFeetPosZ);
  CoxaAngle1[LegIKLegNr] = (((long)Atan4*180) / 3141) + (short)pgm_read_word(&TBL_COXA_ANGLE[LegIKLegNr]);

  //Length between the Coxa and tars [foot]
  IKFeetPosXZ = XYhyp2/DEC_EXP_2;
#ifdef CONFIG_4DOF
  // Some legs may have the 4th DOF and some may not, so handle this here...
  //Calc the TarsToGroundAngle1:
  if ((u8)pgm_read_byte(&TBL_TARS_LENGTH[LegIKLegNr])) {    // We allow mix of 3 and 4 DOF legs...
    TarsToGroundAngle1 = -cTarsConst + cTarsMulti*IKFeetPosY + ((long)(IKFeetPosXZ*cTarsFactorA))/DEC_EXP_1 - ((long)(IKFeetPosXZ*IKFeetPosY)/(cTarsFactorB));
    if (IKFeetPosY < 0)     //Always compensate TarsToGroundAngle1 when IKFeetPosY it goes below zero
      TarsToGroundAngle1 = TarsToGroundAngle1 - ((long)(IKFeetPosY*cTarsFactorC)/DEC_EXP_1);     //TGA base, overall rule
    if (TarsToGroundAngle1 > 400)
      TGA_B_H3 = 200 + (TarsToGroundAngle1/2);
    else
      TGA_B_H3 = TarsToGroundAngle1;

    if (TarsToGroundAngle1 > 300)
      TGA_A_H4 = 240 + (TarsToGroundAngle1/5);
    else
      TGA_A_H4 = TarsToGroundAngle1;

    if (IKFeetPosY > 0)    //Only compensate the TarsToGroundAngle1 when it exceed 30 deg (A, H4 PEP note)
      TarsToGroundAngle1 = TGA_A_H4;
    else if (((IKFeetPosY <= 0) & (IKFeetPosY > -10))) // linear transition between case H3 and H4 (from PEP: H4-K5*(H3-H4))
      TarsToGroundAngle1 = (TGA_A_H4 -(((long)IKFeetPosY*(TGA_B_H3-TGA_A_H4))/DEC_EXP_1));
    else                //IKFeetPosY <= -10, Only compensate TGA1 when it exceed 40 deg
    TarsToGroundAngle1 = TGA_B_H3;

    //Calc Tars Offsets:
    GetSinCos(TarsToGroundAngle1);
    TarsOffsetXZ = ((long)sin4*(u8)pgm_read_byte(&TBL_TARS_LENGTH[LegIKLegNr]))/DEC_EXP_4;
    TarsOffsetY = ((long)cos4*(u8)pgm_read_byte(&TBL_TARS_LENGTH[LegIKLegNr]))/DEC_EXP_4;
  }
  else {
    TarsOffsetXZ = 0;
    TarsOffsetY = 0;
  }
#endif

  //Using GetAtan2 for solving IKA1 and IKSW
  //IKA14 - Angle between SW line and the ground in radians
  IKA14 = GetATan2 (IKFeetPosY-TarsOffsetY, IKFeetPosXZ-(u8)pgm_read_byte(&TBL_COXA_LENGTH[LegIKLegNr])-TarsOffsetXZ);

  //IKSW2 - Length between femur axis and tars
  IKSW2 = XYhyp2;

  //IKA2 - Angle of the line S>W with respect to the femur in radians
  Temp1 = ((((long)(u8)pgm_read_byte(&TBL_FEMUR_LENGTH[LegIKLegNr])*(u8)pgm_read_byte(&TBL_FEMUR_LENGTH[LegIKLegNr])) - ((long)(u8)pgm_read_byte(&TBL_TIBIA_LENGTH[LegIKLegNr])*(u8)pgm_read_byte(&TBL_TIBIA_LENGTH[LegIKLegNr])))*DEC_EXP_4 + ((long)IKSW2*IKSW2));
  Temp2 = (long)(2*(u8)pgm_read_byte(&TBL_FEMUR_LENGTH[LegIKLegNr]))*DEC_EXP_2 * (unsigned long)IKSW2;
  T3 = Temp1 / (Temp2/DEC_EXP_4);
  IKA24 = GetArcCos (T3 );
  //IKFemurAngle
#ifdef OPT_WALK_UPSIDE_DOWN
  if (g_fRobotUpsideDown)
    FemurAngle1[LegIKLegNr] = (long)(IKA14 + IKA24) * 180 / 3141 - 900 + OFFSET_FEMUR_HORN(LegIKLegNr);//Inverted, up side down
  else
    FemurAngle1[LegIKLegNr] = -(long)(IKA14 + IKA24) * 180 / 3141 + 900 + OFFSET_FEMUR_HORN(LegIKLegNr);//Normal
#else
  FemurAngle1[LegIKLegNr] = -(long)(IKA14 + IKA24) * 180 / 3141 + 900 + OFFSET_FEMUR_HORN(LegIKLegNr);//Normal
#endif

  //IKTibiaAngle
  Temp1 = ((((long)(u8)pgm_read_byte(&TBL_FEMUR_LENGTH[LegIKLegNr])*(u8)pgm_read_byte(&TBL_FEMUR_LENGTH[LegIKLegNr])) + ((long)(u8)pgm_read_byte(&TBL_TIBIA_LENGTH[LegIKLegNr])*(u8)pgm_read_byte(&TBL_TIBIA_LENGTH[LegIKLegNr])))*DEC_EXP_4 - ((long)IKSW2*IKSW2));
  Temp2 = (2*(u8)pgm_read_byte(&TBL_FEMUR_LENGTH[LegIKLegNr])*(u8)pgm_read_byte(&TBL_TIBIA_LENGTH[LegIKLegNr]));
  GetArcCos (Temp1 / Temp2);
#ifdef OPT_WALK_UPSIDE_DOWN
  if (g_fRobotUpsideDown)
    TibiaAngle1[LegIKLegNr] = (1800-(long)AngleRad4*180/3141);//Full range tibia, wrong side (up side down)
  else
    TibiaAngle1[LegIKLegNr] = -(1800-(long)AngleRad4*180/3141);//Full range tibia, right side (up side up)
#else
#ifdef PHANTOMX_V2     // BugBug:: cleaner way?
  TibiaAngle1[LegIKLegNr] = -(1450-(long)AngleRad4*180/3141); //!!!!!!!!!!!!145 instead of 1800
#else
  TibiaAngle1[LegIKLegNr] = -(900-(long)AngleRad4*180/3141);
#endif
#endif

#ifdef CONFIG_4DOF
  //Tars angle
  if ((u8)pgm_read_byte(&TBL_TARS_LENGTH[LegIKLegNr])) {    // We allow mix of 3 and 4 DOF legs...
    TarsAngle1[LegIKLegNr] = (TarsToGroundAngle1 + FemurAngle1[LegIKLegNr] - TibiaAngle1[LegIKLegNr])
      + OFFSET_TARS_HORN(LegIKLegNr);
  }
#endif

  //Set the Solution quality
  if(IKSW2 < ((word)((u8)pgm_read_byte(&TBL_FEMUR_LENGTH[LegIKLegNr])+(u8)pgm_read_byte(&TBL_TIBIA_LENGTH[LegIKLegNr])-30)*DEC_EXP_2))
    IKSolution = 1;
  else
  {
    if(IKSW2 < ((word)((u8)pgm_read_byte(&TBL_FEMUR_LENGTH[LegIKLegNr])+(u8)pgm_read_byte(&TBL_TIBIA_LENGTH[LegIKLegNr]))*DEC_EXP_2))
      IKSolutionWarning = 1;
    else
      IKSolutionError = 1    ;
  }
}


//--------------------------------------------------------------------
//[CHECK ANGLES] Checks the mechanical limits of the servos
//--------------------------------------------------------------------
void CheckAngles(void)
{

  for (LegIndex = 0; LegIndex <=5; LegIndex++)
  {
    CoxaAngle1[LegIndex]  = min(max(CoxaAngle1[LegIndex], (short)pgm_read_word(&TBL_COXA_MIN[LegIndex])),
    (short)pgm_read_word(&TBL_COXA_MAX[LegIndex]));
    FemurAngle1[LegIndex] = min(max(FemurAngle1[LegIndex], (short)pgm_read_word(&TBL_FEMUR_MIN[LegIndex])),
    (short)pgm_read_word(&TBL_FEMUR_MAX[LegIndex]));
    TibiaAngle1[LegIndex] = min(max(TibiaAngle1[LegIndex], (short)pgm_read_word(&TBL_TIBIA_MIN[LegIndex])),
    (short)pgm_read_word(&TBL_TIBIA_MAX[LegIndex]));
#ifdef CONFIG_4DOF
    if ((u8)pgm_read_byte(&TBL_TARS_LENGTH[LegIndex])) {    // We allow mix of 3 and 4 DOF legs...
      TarsAngle1[LegIndex] = min(max(TarsAngle1[LegIndex], (short)pgm_read_word(&TBL_TARS_MIN[LegIndex])),
      (short)pgm_read_word(&TBL_TARS_MAX[LegIndex]));
    }
#endif
  }
}


//--------------------------------------------------------------------
// SmoothControl (From Zenta) -  This function makes the body
//            rotation and translation much smoother
//--------------------------------------------------------------------
short SmoothControl (short CtrlMoveInp, short CtrlMoveOut, u8 CtrlDivider)
{

  if (CtrlMoveOut < (CtrlMoveInp - 4))
    return CtrlMoveOut + abs((CtrlMoveOut - CtrlMoveInp)/CtrlDivider);
  else if (CtrlMoveOut > (CtrlMoveInp + 4))
    return CtrlMoveOut - abs((CtrlMoveOut - CtrlMoveInp)/CtrlDivider);

  return CtrlMoveInp;
}

//--------------------------------------------------------------------
// AdjustLegPositionsToBodyHeight() - Will try to adjust the position of the legs
//     to be appropriate for the current y location of the body...
//--------------------------------------------------------------------

uint8_t g_iLegInitIndex = 0x00;    // remember which index we are currently using...

void AdjustLegPositionsToBodyHeight(void)
{
#ifdef CNT_HEX_INITS
  // Lets see which of our units we should use...
  // Note: We will also limit our body height here...
  if (g_InControlState.BodyPos.y > (short)pgm_read_byte(&TBL_MAX_HEX_BODY_Y[CNT_HEX_INITS-1]))
    g_InControlState.BodyPos.y =  (short)pgm_read_byte(&TBL_MAX_HEX_BODY_Y[CNT_HEX_INITS-1]);

  uint8_t i;
  word XZLength1 = pgm_read_byte(&TBL_INIT_HEX_XZ[CNT_HEX_INITS-1]);
  for(i = 0; i < (CNT_HEX_INITS-1); i++) {    // Don't need to look at last entry as we already init to assume this one...
    if (g_InControlState.BodyPos.y <= (short)pgm_read_byte(&TBL_MAX_HEX_BODY_Y[i])) {
      XZLength1 = pgm_read_byte(&TBL_INIT_HEX_XZ[i]);
      break;
    }
  }
  if (i != g_iLegInitIndex) {
    g_iLegInitIndex = i;  // remember the current index...
    //now lets see what happens when we change the leg positions...
    for (uint8_t LegIndex = 0; LegIndex <= 5; LegIndex++) {
      printf(F("leg:%d (%d, %d) -> "), LegIndex, LegPosX[LegIndex], LegPosZ[LegIndex]);
      GetSinCos((short)pgm_read_word(&TBL_COXA_ANGLE[LegIndex]));
      LegPosX[LegIndex] = ((long)((long)cos4 * XZLength1))/DEC_EXP_4;  //Set start positions for each leg
      LegPosZ[LegIndex] = -((long)((long)sin4 * XZLength1))/DEC_EXP_4;
      printf(F("(%d, %d)\n"), LegPosX[LegIndex], LegPosZ[LegIndex]);
    }
    // Make sure we cycle through one gait to have the legs all move into their new locations...
    g_InControlState.ForceGaitStepCnt = StepsInGait;
  }
#endif // CNT_HEX_INITS

}

#ifdef OPT_TERMINAL_MONITOR
extern void DumpEEPROMCmd(u8 *pszCmdLine);

//==============================================================================
// TerminalMonitor - Simple background task checks to see if the user is asking
//    us to do anything, like update debug levels ore the like.
//==============================================================================
boolean TerminalMonitor(void)
{
  u8 szCmdLine[20];  // currently pretty simple command lines...
  u8 ich;
  int ch;
  // See if we need to output a prompt.
  if (g_fShowDebugPrompt) {
    printf(F("Arduino Phoenix Monitor\n"));
    printf(F("D - Toggle debug on or off\n"));
    printf(F("E - Dump EEPROM\n"));

    // Let the Servo driver show it's own set of commands...
    g_ServoDriver.ShowTerminalCommandList();
    g_fShowDebugPrompt = false;
  }

  // First check to see if there is any characters to process.
  if ((ich = DBG_SERIAL.available())) {
    ich = 0;
    // For now assume we receive a packet of data from serial monitor, as the user has
    // to click the send button...
    for (ich=0; ich < sizeof(szCmdLine); ich++) {
      ch = DBG_SERIAL.read();        // get the next character
      if ((ch == -1) || ((ch >= 10) && (ch <= 15)))
        break;
      szCmdLine[ich] = ch;
    }
    szCmdLine[ich] = '\0';    // go ahead and null terminate it...
    printf(F("Serial Cmd Line:%s !!\n"), szCmdLine);

    // So see what are command is.
    if (ich == 0) {
      g_fShowDebugPrompt = true;
    }
    else if ((ich == 1) && ((szCmdLine[0] == 'd') || (szCmdLine[0] == 'D'))) {
      g_fDebugOutput = !g_fDebugOutput;
      if (g_fDebugOutput)
        printf(F("Debug is on\n"));
      else
        printf(F("Debug is off\n"));
    }
    else if (((szCmdLine[0] == 'e') || (szCmdLine[0] == 'E'))) {
      DumpEEPROMCmd(szCmdLine);
    }
    else
    {
      g_ServoDriver.ProcessTerminalCommand(szCmdLine, ich);
    }

    return true;
  }
  return false;
}

//--------------------------------------------------------------------
// DumpEEPROM
//--------------------------------------------------------------------
u8 g_bEEPromDumpMode = 0;  // assume mode 0 - hex dump
word g_wEEPromDumpStart = 0;  // where to start dumps from
u8 g_bEEPromDumpCnt = 16;  // how much to dump at a time

void DumpEEPROM() {
  u8 i;
  word wDumpCnt = g_bEEPromDumpCnt;

  while (wDumpCnt) {
    printf(F("%04x - "), g_wEEPromDumpStart);

    // First in Hex
    for (i = 0; (i < 16) && (i < wDumpCnt); i ++) {
      u8 b;
      b = EEPROM.read(g_wEEPromDumpStart+i);
      printf(F("%02x ", b);
    }
    // Next in Ascii
    printf(F(" : "));
    for (i = 0; (i < 16) && (i < wDumpCnt); i ++) {
      u8 b;
      b = EEPROM.read(g_wEEPromDumpStart+i);
      if ((b > 0x1f) && (b < 0x7f))
        printf(F("%c", b);
      else
        printf(F("."));
    }
    printf(F("\n"));
    g_wEEPromDumpStart += i;  // how many u8s we output
    wDumpCnt -= i;            // How many more to go...
  }

}
//--------------------------------------------------------------------
// GetCmdLineNum - passed pointer to pointer so we can update...
//--------------------------------------------------------------------
word GetCmdLineNum(u8 **ppszCmdLine) {
  u8 *psz = *ppszCmdLine;
  word w = 0;

  // Ignore any blanks
  while (*psz == ' ')
    psz++;

  // See if Hex value passed in
  if ((*psz == '0') && ((*(psz+1) == 'x') || (*(psz+1) == 'X'))) {
    // Hex mode
    psz += 2;  // get over 0x
    for (;;) {
      if ((*psz >= '0') && (*psz <= '9'))
        w = w * 16 + *psz++ - '0';
      else if ((*psz >= 'a') && (*psz <= 'f'))
        w = w * 16 + *psz++ - 'a' + 10;
      else if ((*psz >= 'A') && (*psz <= 'F'))
        w = w * 16 + *psz++ - 'A' + 10;
      else
        break;
    }

  }
  else {
    // decimal mode
    while ((*psz >= '0') && (*psz <= '9'))
      w = w * 10 + *psz++ - '0';
  }
  *ppszCmdLine = psz;    // update command line pointer
  return w;

}

//--------------------------------------------------------------------
// DumpEEPROMCmd
//--------------------------------------------------------------------
void DumpEEPROMCmd(u8 *pszCmdLine) {
  // first u8 can be H for hex or W for words...
  if (!*++pszCmdLine)  // Need to get past the command letter first...
    DumpEEPROM();
  else if ((*pszCmdLine == 'h') || (*pszCmdLine == 'H'))
    g_bEEPromDumpMode = 0;
  else if ((*pszCmdLine == 'w') || (*pszCmdLine == 'W'))
    g_bEEPromDumpMode = 0;

  else {
    // First argument should be the start location to dump
    g_wEEPromDumpStart = GetCmdLineNum(&pszCmdLine);

    // If the next u8 is an "=" may try to do updates...
    if (*pszCmdLine == '=') {
      // make sure we don't get stuck in a loop...
      u8 *psz = pszCmdLine;
      word w;
      while (*psz) {
        w = GetCmdLineNum(&psz);
        if (psz == pszCmdLine)
          break;  // not valid stuff so bail!
        pszCmdLine = psz;  // remember how far we got...

        EEPROM.write(g_wEEPromDumpStart++, w & 0xff);
    }
  }
    else {
      if (*pszCmdLine == ' ') { // A blank assume we have a count...
        g_bEEPromDumpCnt = GetCmdLineNum(&pszCmdLine);
      }
      }
    DumpEEPROM();
    }
    }
#endif


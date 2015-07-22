//=============================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix software
//
//Programmer: Jeroen Janssen [aka Xan]
//         Kurt Eckhardt(KurtE) converted to C and Arduino
//   Kåre Halvorsen aka Zenta - Makes everything work correctly!
//
// This version of the Phoenix code was ported over to the Arduino Environement
//
//
// Phoenix.h - This is the first header file that is needed to build
//			a Phoenix program for a specific Hex Robot.
//
//
// This file assumes that the main source file either directly or through include
// file has defined all of the configuration information for the specific robot.
// Each robot will also need to include:
//
//=============================================================================
//
//KNOWN BUGS:
//    - Lots ;)
//
//=============================================================================
//==============================================================================
#ifndef _PHOENIX_CORE_H_
#define _PHOENIX_CORE_H_
#include <stdarg.h>
#include <EEPROM.h>

//=============================================================================
//[CONSTANTS]
//=============================================================================
#define BUTTON_DOWN 0
#define BUTTON_UP 	1

#define	DEC_EXP_1		10
#define	DEC_EXP_2		100
#define	DEC_EXP_4		10000
#define	DEC_EXP_6		1000000

enum {
    IDX_RR = 0,
    IDX_RM,
    IDX_RF,
    IDX_LR,
    IDX_LM,
    IDX_LF
};

#define NUM_GAITS    6
#define SMOOTH_DIV    4  //"Smooth division" factor for the smooth control function, a value of 3 to 5 is most suitable
extern void GaitSelect(void);
extern short SmoothControl (short CtrlMoveInp, short CtrlMoveOut, u8 CtrlDivider);



//-----------------------------------------------------------------------------
// Define Global variables
//-----------------------------------------------------------------------------
extern boolean          g_fDebugOutput;
extern boolean          g_fEnableServos;      // Hack to allow me to turn servo processing off...
extern boolean          g_fRobotUpsideDown;    // Is the robot upside down?


extern boolean CheckVoltage(void);

void AdjustLegPositionsToBodyHeight(void);

// debug handler...
extern boolean g_fDBGHandleError;


#ifdef OPT_BACKGROUND_PROCESS
#define DoBackgroundProcess()   g_ServoDriver.BackgroundProcess()
#else
#define DoBackgroundProcess()
#endif

#ifdef DEBUG_IOPINS
#define TOGGLE_PIN(pin)         {digitalWrite(pin, !digitalRead(pin));}
#define WRITE_PIN(pin, state)   {digitalWrite(pin, state);}
#else
#define TOGGLE_PIN(pin)
#define WRITE_PIN(pin, state)
#endif


//=============================================================================
//=============================================================================
// Define the class(s) for our Input controllers.
//=============================================================================
//=============================================================================
class InputController {
public:
    virtual void     Init(void);
    virtual void     ControlInput(void);
    virtual void     AllowControllerInterrupts(boolean fAllow);

private:
} ;

// Define a function that allows us to define which controllers are to be used.
extern void  RegisterInputController(InputController *pic);



typedef struct _Coord3D {
  long      x;
  long      y;
  long      z;
} COORD3D;


//==============================================================================
// class ControlState: This is the main structure of data that the Control
//      manipulates and is used by the main Phoenix Code to make it do what is
//      requested.
//==============================================================================
typedef struct _InControlState {
  boolean		fHexOn;				//Switch to turn on Phoenix
  boolean		fPrev_HexOn;		//Previous loop state

  //Body position
  COORD3D        BodyPos;
  COORD3D        BodyRotOffset;      // Body rotation offset;

  //Body Inverse Kinematics
  COORD3D        BodyRot1;               // X -Pitch, Y-Rotation, Z-Roll

  //[gait]
  u8			GaitType;			//Gait type
  short			LegLiftHeight;		//Current Travel height
  COORD3D        TravelLength;            // X-Z or Length, Y is rotation.

  //[Single Leg Control]
  u8			SelectedLeg;
  COORD3D       SLLeg;                //
  boolean		fSLHold;		 	//Single leg control mode


  //[Balance]
  boolean        BalanceMode;

  //[TIMING]
  u8			InputTimeDelay;	//Delay that depends on the input to get the "sneaking" effect
  word			SpeedControl;	//Adjustible Delay
  u8       ForceGaitStepCnt;          // new to allow us to force a step even when not moving
} INCONTROLSTATE;

//==============================================================================
//==============================================================================
// Define the class(s) for Servo Drivers.
//==============================================================================
//==============================================================================
class ServoDriver {
public:
  void Init(void);

    word GetBatteryVoltage(void);

#ifdef OPT_GPPLAYER
    inline boolean  FIsGPEnabled(void) {return _fGPEnabled;};
  boolean         FIsGPSeqDefined(uint8_t iSeq);
    inline boolean  FIsGPSeqActive(void) {return _fGPActive;};
    void            GPStartSeq(uint8_t iSeq);  // 0xff - says to abort...
  void            GPPlayer(void);
    uint8_t         GPNumSteps(void);          // How many steps does the current sequence have
    uint8_t         GPCurStep(void);           // Return which step currently on...
    void            GPSetSpeedMultiplyer(short sm) ;      // Set the Speed multiplier (100 is default)
#endif
  void BeginServoUpdate(void);    // Start the update
#ifdef CONFIG_4DOF
  void OutputServoInfoForLeg(u8 LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1, short sTarsAngle1);
#else
  void OutputServoInfoForLeg(u8 LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1);
#endif
  void CommitServoDriver(word wMoveTime);
  void FreeServos(void);

    // Allow for background process to happen...
#ifdef OPT_BACKGROUND_PROCESS
    void BackgroundProcess(void);
#endif

#ifdef OPT_TERMINAL_MONITOR
    void ShowTerminalCommandList(void);
    boolean ProcessTerminalCommand(u8 *psz, u8 bLen);
#endif

private:

#ifdef OPT_GPPLAYER
  boolean _fGPEnabled;     // IS GP defined for this servo driver?
  boolean _fGPActive;      // Is a sequence currently active - May change later when we integrate in sequence timing adjustment code
  uint8_t    _iSeq;        // current sequence we are running
    short    _sGPSM;        // Speed multiplier +-200
#endif

} ;

//==============================================================================
//==============================================================================
// Define global class objects
//==============================================================================
//==============================================================================
extern ServoDriver      g_ServoDriver;           // our global servo driver class
extern InputController  g_InputController;       // Our Input controller
extern INCONTROLSTATE   g_InControlState;		 // State information that controller changes


#endif



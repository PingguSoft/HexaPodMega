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
//        	a Phoenix program for a specific Hex Robot.
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



//-----------------------------------------------------------------------------
// Define Global variables
//-----------------------------------------------------------------------------


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
    virtual void     AllowControllerInterrupts(bool fAllow);

private:
} ;

// Define a function that allows us to define which controllers are to be used.
extern void  RegisterInputController(InputController *pic);

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
    inline bool  FIsGPEnabled(void) {return _fGPEnabled;};
  bool         FIsGPSeqDefined(uint8_t iSeq);
    inline bool  FIsGPSeqActive(void) {return _fGPActive;};
    void            GPStartSeq(uint8_t iSeq);  // 0xff - says to abort...
  void            GPPlayer(void);
    uint8_t         GPNumSteps(void);          // How many steps does the current sequence have
    uint8_t         GPCurStep(void);           // Return which step currently on...
    void            GPSetSpeedMultiplyer(s16 sm) ;      // Set the Speed multiplier (100 is default)
#endif
  void BeginServoUpdate(void);    // Start the update
#ifdef CONFIG_4DOF
  void OutputServoInfoForLeg(u8 LegIndex, s16 sCoxaAngle1, s16 sFemurAngle1, s16 sTibiaAngle1, s16 sTarsAngle1);
#else
  void OutputServoInfoForLeg(u8 LegIndex, s16 sCoxaAngle1, s16 sFemurAngle1, s16 sTibiaAngle1);
#endif
  void CommitServoDriver(word wMoveTime);
  void FreeServos(void);

    // Allow for background process to happen...
#ifdef OPT_BACKGROUND_PROCESS
    void BackgroundProcess(void);
#endif

#ifdef OPT_TERMINAL_MONITOR
    void ShowTerminalCommandList(void);
    bool ProcessTerminalCommand(u8 *psz, u8 bLen);
#endif

private:

#ifdef OPT_GPPLAYER
  bool _fGPEnabled;     // IS GP defined for this servo driver?
  bool _fGPActive;      // Is a sequence currently active - May change later when we integrate in sequence timing adjustment code
  uint8_t    _iSeq;        // current sequence we are running
    s16    _sGPSM;        // Speed multiplier +-200
#endif

} ;

#endif


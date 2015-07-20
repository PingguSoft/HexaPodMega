//====================================================================
//Project Lynxmotion Phoenix
//Description:
//    This is the hardware configuration file for the Hex Robot.
//
//    This version of the Configuration file is set up to run on the
//    Lynxmotion BotboardDuino board, which is similar  to the Arduino Duemilanove
//
//    This version of configuration file assumes that the servos will be controlled
//    by a Lynxmotion Servo controller SSC-32 and the user is using a Lynxmotion
//    PS2 to control the robot.
//
//Date: March 18, 2012
//Programmer: Kurt (aka KurtE)
//
//
//NEW IN V1.0
//   - First Release
//
//====================================================================
#ifndef HEX_CFG_CHR3_H
#define HEX_CFG_CHR3_H


// Which type of control(s) do you want to compile in

#define DEBUG

//==================================================================================================================================
// Define which input classes we will use. If we wish to use more than one we need to define USEMULTI - This will define a forwarder
//    type implementation, that the Inputcontroller will need to call.  There will be some negotion for which one is in contol.
//
//  If this is not defined, The included Controller should simply implement the InputController Class...
//==================================================================================================================================
//#define USEMULTI
//#define USEXBEE            // only allow to be defined on Megas...
//#define USEPS2
//#define USECOMMANDER
#define USESERIAL

// Do we want Debug Serial Output?
#define DBGSerial Serial

// Some configurations will not allow this so if one of them undefine it
#if (defined USEXBEE) || (defined USECOMMANDER)
#ifdef DontAllowDebug
#undef DBGSerial
#endif
#endif

#ifdef USESERIAL
#define SerSerial Serial
#endif

#ifdef DBGSerial
//#define OPT_TERMINAL_MONITOR  // Only allow this to be defined if we have a debug serial port
#endif

#ifdef OPT_TERMINAL_MONITOR
//#define OPT_SSC_FORWARDER  // only useful if terminal monitor is enabled
#define OPT_FIND_SERVO_OFFSETS    // Only useful if terminal monitor is enabled
#endif

//#define OPT_GPPLAYER

//#define USE_SSC32
//#define	cSSC_BINARYMODE	1			// Define if your SSC-32 card supports binary mode.

// Debug options
//#define DEBUG_IOPINS    // used to control if we are going to use IO pins for debug support

//==================================================================================================================================
//==================================================================================================================================
//==================================================================================================================================
// CHR-3
//==================================================================================================================================

//[We assume that this has to be a Mega to run the servos dirctly...
#ifdef __AVR__
//[Kurt's Mega Shield]
#define SOUND_PIN    37
// XBee was defined to use a hardware Serial port
#define SERIAL_BAUD    115200
#define DEBUG_BAUD     115200

// Define Analog pin and minimum voltage that we will allow the servos to run
#define cVoltagePin  0      // Use our Analog pin jumper here...
#define cTurnOffVol  470     // 4.7v
#define cTurnOnVol   550     // 5.5V - optional part to say if voltage goes back up, turn it back on...


// SSC32 is using Hardware Serial Port
#else
//[Kurt's Mega shield on Chipkit Max32]
#define SOUND_PIN    44        //
#define PS2_DAT      46
#define PS2_CMD      47
#define PS2_SEL      48
#define PS2_CLK      49

// XBee was defined to use a hardware Serial port
#define XBEE_BAUD        38400
#define SERIAL_BAUD    38400

#endif

//====================================================================
//[IO Pins On Kurt's Mega shield]
// Kurts new Mega Shield for Mega pin numbers.
// Note: Pins 24, 26, 27 can not work on Max32
// Having problem with 32 and 34
#define cRRCoxaPin      35  //Rear Right leg Hip Horizontal
#define cRRFemurPin     36  //Rear Right leg Hip Vertical
#define cRRTibiaPin     19  //Rear Right leg Knee
#define cRRTarsPin      1   // Tar

#define cRMCoxaPin      25  //Middle Right leg Hip Horizontal
#define cRMFemurPin     33  //Middle Right leg Hip Vertical
#define cRMTibiaPin     34  //Middle Right leg Knee
#define cRMTarsPin       1  // Tar

#define cRFCoxaPin      22  //Front Right leg Hip Horizontal
#define cRFFemurPin     23  //Front Right leg Hip Vertical
#define cRFTibiaPin     24   //Front Right leg Knee
#define cRFTarsPin       1   // Tar

#define cLRCoxaPin       9   //Rear Left leg Hip Horizontal
#define cLRFemurPin     10   //Rear Left leg Hip Vertical
#define cLRTibiaPin     11   //Rear Left leg Knee
#define cLRTarsPin       1   // Tar

#define cLMCoxaPin       6  //Middle Left leg Hip Horizontal
#define cLMFemurPin      7   //Middle Left leg Hip Vertical
#define cLMTibiaPin      8  //Middle Left leg Knee
#define cLMTarsPin       1  // Tar = Not working...

#define cLFCoxaPin       2   //Front Left leg Hip Horizontal
#define cLFFemurPin      3   //Front Left leg Hip Vertical
#define cLFTibiaPin      5   //Front Left leg Knee
#define cLFTarsPin       1   // Tar

#define COXA_MinANG -800
#define COXA_MaxANG 800
#define FEMUR_MinANG -800
#define FEMUR_MaxANG 800
#define TIBIA_MinANG -800
#define TIBIA_MaxANG 800
//--------------------------------------------------------------------
//[MIN/MAX ANGLES]
#define cRRCoxaMin1	  COXA_MinANG	//Mechanical limits of the Right Rear Leg, decimals = 1
#define cRRCoxaMax1	  COXA_MaxANG
#define cRRFemurMin1	FEMUR_MinANG
#define cRRFemurMax1	FEMUR_MaxANG
#define cRRTibiaMin1	TIBIA_MinANG
#define cRRTibiaMax1	TIBIA_MaxANG

#define cRMCoxaMin1	  COXA_MinANG	//Mechanical limits of the Right Middle Leg, decimals = 1
#define cRMCoxaMax1	  COXA_MaxANG
#define cRMFemurMin1	FEMUR_MinANG
#define cRMFemurMax1	FEMUR_MaxANG
#define cRMTibiaMin1	TIBIA_MinANG
#define cRMTibiaMax1	TIBIA_MaxANG

#define cRFCoxaMin1	  COXA_MinANG	//Mechanical limits of the Right Front Leg, decimals = 1
#define cRFCoxaMax1	  COXA_MaxANG
#define cRFFemurMin1	FEMUR_MinANG
#define cRFFemurMax1	FEMUR_MaxANG
#define cRFTibiaMin1	TIBIA_MinANG
#define cRFTibiaMax1	TIBIA_MaxANG

#define cLRCoxaMin1	  COXA_MinANG	//Mechanical limits of the Left Rear Leg, decimals = 1
#define cLRCoxaMax1	  COXA_MaxANG
#define cLRFemurMin1	FEMUR_MinANG
#define cLRFemurMax1	FEMUR_MaxANG
#define cLRTibiaMin1	TIBIA_MinANG
#define cLRTibiaMax1	TIBIA_MaxANG

#define cLMCoxaMin1	  COXA_MinANG	//Mechanical limits of the Left Middle Leg, decimals = 1
#define cLMCoxaMax1	  COXA_MaxANG
#define cLMFemurMin1	FEMUR_MinANG
#define cLMFemurMax1	FEMUR_MaxANG
#define cLMTibiaMin1	TIBIA_MinANG
#define cLMTibiaMax1	TIBIA_MaxANG

#define cLFCoxaMin1	  COXA_MinANG	//Mechanical limits of the Left Front Leg, decimals = 1
#define cLFCoxaMax1	  COXA_MaxANG
#define cLFFemurMin1	FEMUR_MinANG
#define cLFFemurMax1	FEMUR_MaxANG
#define cLFTibiaMin1	TIBIA_MinANG
#define cLFTibiaMax1	TIBIA_MaxANG


//--------------------------------------------------------------------
//[LEG DIMENSIONS]
//Universal dimensions for each leg in mm
#define cXXCoxaLength     45    // This is for CH3-R with Type 3 legs
#define cXXFemurLength    35
#define cXXTibiaLength    80
#define cXXTarsLength     1    // 4DOF only...

#define cRRCoxaLength     cXXCoxaLength	    //Right Rear leg
#define cRRFemurLength    cXXFemurLength
#define cRRTibiaLength    cXXTibiaLength
#define cRRTarsLength	  cXXTarsLength	    //4DOF ONLY

#define cRMCoxaLength     cXXCoxaLength	    //Right middle leg
#define cRMFemurLength    cXXFemurLength
#define cRMTibiaLength    cXXTibiaLength
#define cRMTarsLength	  cXXTarsLength	    //4DOF ONLY

#define cRFCoxaLength     cXXCoxaLength	    //Rigth front leg
#define cRFFemurLength    cXXFemurLength
#define cRFTibiaLength    cXXTibiaLength
#define cRFTarsLength	  cXXTarsLength    //4DOF ONLY

#define cLRCoxaLength     cXXCoxaLength	    //Left Rear leg
#define cLRFemurLength    cXXFemurLength
#define cLRTibiaLength    cXXTibiaLength
#define cLRTarsLength	  cXXTarsLength    //4DOF ONLY

#define cLMCoxaLength     cXXCoxaLength	    //Left middle leg
#define cLMFemurLength    cXXFemurLength
#define cLMTibiaLength    cXXTibiaLength
#define cLMTarsLength	  cXXTarsLength	    //4DOF ONLY

#define cLFCoxaLength     cXXCoxaLength	    //Left front leg
#define cLFFemurLength    cXXFemurLength
#define cLFTibiaLength    cXXTibiaLength
#define cLFTarsLength	  cXXTarsLength	    //4DOF ONLY


//--------------------------------------------------------------------
//[BODY DIMENSIONS]
#define cRRCoxaAngle1    (-450)       //Default Coxa setup angle, decimals = 1
#define cRMCoxaAngle1    (0   )       //Default Coxa setup angle, decimals = 1
#define cRFCoxaAngle1    (450)       //Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle1    (-450)       //Default Coxa setup angle, decimals = 1
#define cLMCoxaAngle1    (0   )       //Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle1    (450)       //Default Coxa setup angle, decimals = 1


#define HALF_BODY_LENGHT 63
#define HALF_BODY_MIDDLE_WIDTH 50
#define HALF_BODY_FB_WIDTH 45

#define cRROffsetX 	-HALF_BODY_FB_WIDTH     //Distance X from center of the body to the Right Rear coxa
#define cRROffsetZ 	HALF_BODY_LENGHT	      //Distance Z from center of the body to the Right Rear coxa
#define cRMOffsetX 	-HALF_BODY_MIDDLE_WIDTH //Distance X from center of the body to the Right Middle coxa
#define cRMOffsetZ 	0	                      //Distance Z from center of the body to the Right Middle coxa
#define cRFOffsetX 	-HALF_BODY_FB_WIDTH	    //Distance X from center of the body to the Right Front coxa
#define cRFOffsetZ 	-HALF_BODY_LENGHT	      //Distance Z from center of the body to the Right Front coxa

#define cLROffsetX 	HALF_BODY_FB_WIDTH	    //Distance X from center of the body to the Left Rear coxa
#define cLROffsetZ 	HALF_BODY_LENGHT	      //Distance Z from center of the body to the Left Rear coxa
#define cLMOffsetX 	HALF_BODY_MIDDLE_WIDTH	//Distance X from center of the body to the Left Middle coxa
#define cLMOffsetZ 	0	                      //Distance Z from center of the body to the Left Middle coxa
#define cLFOffsetX 	HALF_BODY_FB_WIDTH	    //Distance X from center of the body to the Left Front coxa
#define cLFOffsetZ 	-HALF_BODY_LENGHT	      //Distance Z from center of the body to the Left Front coxa

//--------------------------------------------------------------------
//[START POSITIONS FEET]
#define cHexInitXZ	 100
#define CHexInitXZCos60  ((long)(cHexInitXZ*0.7))        // COS(60) = .5
#define CHexInitXZSin60  ((long)(cHexInitXZ*0.7))        // sin(60) = .866
#define CHexInitY	 40


// Lets try some multi leg positions depending on height settings.

#if 1 // Start first without...
#define CNT_HEX_INITS 3
#define MAX_BODY_Y  90
#ifdef DEFINE_HEX_GLOBALS
  const byte g_abHexIntXZ[] PROGMEM = {cHexInitXZ, 99, 86};
  const byte g_abHexMaxBodyY[] PROGMEM = { 20, 50, MAX_BODY_Y};
#else
  extern const byte g_abHexIntXZ[] PROGMEM;
  extern const byte g_abHexMaxBodyY[] PROGMEM;
#endif
#endif // if 0

#define cRRInitPosX     CHexInitXZCos60      //Start positions of the Right Rear leg
#define cRRInitPosY     CHexInitY
#define cRRInitPosZ     CHexInitXZSin60

#define cRMInitPosX     cHexInitXZ      //Start positions of the Right Middle leg
#define cRMInitPosY     CHexInitY
#define cRMInitPosZ     0

#define cRFInitPosX     CHexInitXZCos60      //Start positions of the Right Front leg
#define cRFInitPosY     CHexInitY
#define cRFInitPosZ     -CHexInitXZSin60

#define cLRInitPosX     CHexInitXZCos60      //Start positions of the Left Rear leg
#define cLRInitPosY     CHexInitY
#define cLRInitPosZ     CHexInitXZSin60

#define cLMInitPosX     cHexInitXZ      //Start positions of the Left Middle leg
#define cLMInitPosY     CHexInitY
#define cLMInitPosZ     0

#define cLFInitPosX     CHexInitXZCos60      //Start positions of the Left Front leg
#define cLFInitPosY     CHexInitY
#define cLFInitPosZ     -CHexInitXZSin60
//--------------------------------------------------------------------
//[Tars factors used in formula to calc Tarsus angle relative to the ground]
#define cTarsConst	720	//4DOF ONLY
#define cTarsMulti	2	//4DOF ONLY
#define cTarsFactorA	70	//4DOF ONLY
#define cTarsFactorB	60	//4DOF ONLY
#define cTarsFactorC	50	//4DOF ONLY

#endif CFG_HEX_H

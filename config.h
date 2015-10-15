/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 see <http://www.gnu.org/licenses/>
*/

#ifndef __CONFIG_H__
#define __CONFIG_H__
#include "common.h"

#define CONFIG_ORIGINAL         1
#define CONFIG_NASSPOP_MEGA     2
#define CONFIG_NASSPOP_MINI     3
#define CONFIG_BOARD            CONFIG_NASSPOP_MEGA


#define CONFIG_DOF_PER_LEG      3
#define CONFIG_NUM_LEGS         6
#define CONFIG_TRAVEL_DEAD_ZONE 4

#if (CONFIG_BOARD == CONFIG_ORIGINAL) || (CONFIG_BOARD == CONFIG_NASSPOP_MEGA)
#define CONFIG_DBG_SERIAL   Serial
#define CONFIG_DEBUG_BAUD   115200

#define CONFIG_CTRL_SERIAL  Serial1
#define CONFIG_CTRL_BAUD    115200
#elif (CONFIG_BOARD == CONFIG_NASSPOP_MINI)
//#define CONFIG_DBG_SERIAL   Serial
//#define CONFIG_DEBUG_BAUD   115200

#define CONFIG_CTRL_SERIAL  Serial
#define CONFIG_CTRL_BAUD    115200
#endif

#define CONFIG_CTRL_TYPE_SERIAL 0
#define CONFIG_CTRL_TYPE_BTCON  1
#define CONFIG_CTRL_TYPE    CONFIG_CTRL_TYPE_BTCON

#ifdef CONFIG_DBG_SERIAL
    #define CONFIG_TERMINAL
#endif

#define CONFIG_VBAT_SMOOTH   16
#define CONFIG_VBAT_SCALE    48
#define CONFIG_VBAT_OFFSET   -1   // 0.1v unit

#define CONFIG_VOLT_OFF 111 // 11.1v
#define CONFIG_VOLT_ON  115 // 11.5V - optional part to say if voltage goes back up, turn it back on...


//====================================================================
//[IO Pins On 2560]

#define PIN_ANALOG_VOLT A0

#if (CONFIG_BOARD == CONFIG_ORIGINAL)
    #define PIN_STATUS_RED      31
    #define PIN_STATUS_GREEN    30
    #define PIN_STATUS_BLUE     32
    #define PIN_SOUND           37

    #define PIN_RR_COXA         35  //Rear Right leg Hip Horizontal
    #define PIN_RR_FEMUR        36  //Rear Right leg Hip Vertical
    #define PIN_RR_TIBIA        19  //Rear Right leg Knee
    #define PIN_RR_TARS          1  //Tar

    #define PIN_RM_COXA         25  //Middle Right leg Hip Horizontal
    #define PIN_RM_FEMUR        33  //Middle Right leg Hip Vertical
    #define PIN_RM_TIBIA        34  //Middle Right leg Knee
    #define PIN_RM_TARS         1   //Tar

    #define PIN_RF_COXA         22  //Front Right leg Hip Horizontal
    #define PIN_RF_FEMUR        23  //Front Right leg Hip Vertical
    #define PIN_RF_TIBIA        24  //Front Right leg Knee
    #define PIN_RF_TARS         1   //Tar
#elif (CONFIG_BOARD == CONFIG_NASSPOP_MEGA)
    #define PIN_STATUS_RED      A7
    #define PIN_STATUS_GREEN    A6
    #define PIN_STATUS_BLUE     A5
    #define PIN_SOUND           A14

    #define PIN_RR_COXA         44  //Rear Right leg Hip Horizontal
    #define PIN_RR_FEMUR        45  //Rear Right leg Hip Vertical
    #define PIN_RR_TIBIA        46  //Rear Right leg Knee
    #define PIN_RR_TARS          1  //Tar

    #define PIN_RM_COXA         35  //Middle Right leg Hip Horizontal
    #define PIN_RM_FEMUR        33  //Middle Right leg Hip Vertical
    #define PIN_RM_TIBIA        36  //Middle Right leg Knee
    #define PIN_RM_TARS          1  //Tar

    #define PIN_RF_COXA         A15 //Front Right leg Hip Horizontal
    #define PIN_RF_FEMUR        32  //Front Right leg Hip Vertical
    #define PIN_RF_TIBIA        34  //Front Right leg Knee
    #define PIN_RF_TARS          1  //Tar
#elif (CONFIG_BOARD == CONFIG_NASSPOP_MINI)
    #define PIN_STATUS_RED      A7
    #define PIN_STATUS_GREEN    A6
    #define PIN_STATUS_BLUE     A5
    #define PIN_SOUND            1

    #define PIN_RR_COXA         44  //Rear Right leg Hip Horizontal
    #define PIN_RR_FEMUR        45  //Rear Right leg Hip Vertical
    #define PIN_RR_TIBIA        46  //Rear Right leg Knee
    #define PIN_RR_TARS          1  //Tar

    #define PIN_RM_COXA         35  //Middle Right leg Hip Horizontal
    #define PIN_RM_FEMUR        33  //Middle Right leg Hip Vertical
    #define PIN_RM_TIBIA        36  //Middle Right leg Knee
    #define PIN_RM_TARS          1  //Tar

    #define PIN_RF_COXA          0 //Front Right leg Hip Horizontal
    #define PIN_RF_FEMUR        32  //Front Right leg Hip Vertical
    #define PIN_RF_TIBIA        34  //Front Right leg Knee
    #define PIN_RF_TARS          1  //Tar
#endif

#define PIN_LR_COXA       9  //Rear Left leg Hip Horizontal
#define PIN_LR_FEMUR     10  //Rear Left leg Hip Vertical
#define PIN_LR_TIBIA     11  //Rear Left leg Knee
#define PIN_LR_TARS       1  //Tar

#define PIN_LM_COXA       6  //Middle Left leg Hip Horizontal
#define PIN_LM_FEMUR      7  //Middle Left leg Hip Vertical
#define PIN_LM_TIBIA      8  //Middle Left leg Knee
#define PIN_LM_TARS       1  //Tar = Not working...

#define PIN_LF_COXA       2  //Front Left leg Hip Horizontal
#define PIN_LF_FEMUR      3  //Front Left leg Hip Vertical
#define PIN_LF_TIBIA      5  //Front Left leg Knee
#define PIN_LF_TARS       1  //Tar

//--------------------------------------------------------------------
//[MIN/MAX ANGLES]
#define COXA_MIN_ANG    -800
#define COXA_MAX_ANG     800
#define FEMUR_MIN_ANG   -800
#define FEMUR_MAX_ANG    800
#define TIBIA_MIN_ANG   -800
#define TIBIA_MAX_ANG    800

#define cRRCoxaMin1	    COXA_MIN_ANG	//Mechanical limits of the Right Rear Leg, decimals = 1
#define cRRCoxaMax1	    COXA_MAX_ANG
#define cRRFemurMin1	FEMUR_MIN_ANG
#define cRRFemurMax1	FEMUR_MAX_ANG
#define cRRTibiaMin1	TIBIA_MIN_ANG
#define cRRTibiaMax1	TIBIA_MAX_ANG

#define cRMCoxaMin1	    COXA_MIN_ANG	//Mechanical limits of the Right Middle Leg, decimals = 1
#define cRMCoxaMax1	    COXA_MAX_ANG
#define cRMFemurMin1	FEMUR_MIN_ANG
#define cRMFemurMax1	FEMUR_MAX_ANG
#define cRMTibiaMin1	TIBIA_MIN_ANG
#define cRMTibiaMax1	TIBIA_MAX_ANG

#define cRFCoxaMin1	    COXA_MIN_ANG	//Mechanical limits of the Right Front Leg, decimals = 1
#define cRFCoxaMax1	    COXA_MAX_ANG
#define cRFFemurMin1	FEMUR_MIN_ANG
#define cRFFemurMax1	FEMUR_MAX_ANG
#define cRFTibiaMin1	TIBIA_MIN_ANG
#define cRFTibiaMax1	TIBIA_MAX_ANG

#define cLRCoxaMin1	    COXA_MIN_ANG	//Mechanical limits of the Left Rear Leg, decimals = 1
#define cLRCoxaMax1	    COXA_MAX_ANG
#define cLRFemurMin1	FEMUR_MIN_ANG
#define cLRFemurMax1	FEMUR_MAX_ANG
#define cLRTibiaMin1	TIBIA_MIN_ANG
#define cLRTibiaMax1	TIBIA_MAX_ANG

#define cLMCoxaMin1	    COXA_MIN_ANG	//Mechanical limits of the Left Middle Leg, decimals = 1
#define cLMCoxaMax1	    COXA_MAX_ANG
#define cLMFemurMin1	FEMUR_MIN_ANG
#define cLMFemurMax1	FEMUR_MAX_ANG
#define cLMTibiaMin1	TIBIA_MIN_ANG
#define cLMTibiaMax1	TIBIA_MAX_ANG

#define cLFCoxaMin1	    COXA_MIN_ANG	//Mechanical limits of the Left Front Leg, decimals = 1
#define cLFCoxaMax1	    COXA_MAX_ANG
#define cLFFemurMin1	FEMUR_MIN_ANG
#define cLFFemurMax1	FEMUR_MAX_ANG
#define cLFTibiaMin1	TIBIA_MIN_ANG
#define cLFTibiaMax1	TIBIA_MAX_ANG


//--------------------------------------------------------------------
//[LEG DIMENSIONS]
//Universal dimensions for each leg in mm
#define cXXCoxaLength     45    // This is for CH3-R with Type 3 legs
#define cXXFemurLength    35
#define cXXTibiaLength    70
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
#define cRRCoxaAngle1    (-450)         //Default Coxa setup angle, decimals = 1
#define cRMCoxaAngle1    (0   )         //Default Coxa setup angle, decimals = 1
#define cRFCoxaAngle1    (450)          //Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle1    (-450)         //Default Coxa setup angle, decimals = 1
#define cLMCoxaAngle1    (0   )         //Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle1    (450)          //Default Coxa setup angle, decimals = 1


#define HALF_BODY_LENGHT        63
#define HALF_BODY_MIDDLE_WIDTH  50
#define HALF_BODY_FB_WIDTH      45

#define cRROffsetX     -HALF_BODY_FB_WIDTH     //Distance X from center of the body to the Right Rear coxa
#define cRROffsetZ      HALF_BODY_LENGHT	    //Distance Z from center of the body to the Right Rear coxa
#define cRMOffsetX     -HALF_BODY_MIDDLE_WIDTH //Distance X from center of the body to the Right Middle coxa
#define cRMOffsetZ      0                       //Distance Z from center of the body to the Right Middle coxa
#define cRFOffsetX     -HALF_BODY_FB_WIDTH	    //Distance X from center of the body to the Right Front coxa
#define cRFOffsetZ     -HALF_BODY_LENGHT	    //Distance Z from center of the body to the Right Front coxa

#define cLROffsetX      HALF_BODY_FB_WIDTH	    //Distance X from center of the body to the Left Rear coxa
#define cLROffsetZ      HALF_BODY_LENGHT	    //Distance Z from center of the body to the Left Rear coxa
#define cLMOffsetX      HALF_BODY_MIDDLE_WIDTH	//Distance X from center of the body to the Left Middle coxa
#define cLMOffsetZ      0	                    //Distance Z from center of the body to the Left Middle coxa
#define cLFOffsetX      HALF_BODY_FB_WIDTH	    //Distance X from center of the body to the Left Front coxa
#define cLFOffsetZ     -HALF_BODY_LENGHT	    //Distance Z from center of the body to the Left Front coxa

//--------------------------------------------------------------------
//[START POSITIONS FEET]
#define cHexInitXZ	        100
#define CHexInitXZCos60     ((long)(cHexInitXZ*0.7))        // COS(60) = .5
#define CHexInitXZSin60     ((long)(cHexInitXZ*0.7))        // sin(60) = .866
#define CHexInitY	        40

// Lets try some multi leg positions depending on height settings.
#define CNT_HEX_INITS       3
#define MAX_BODY_Y          90

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
#define cTarsConst	    720	//4DOF ONLY
#define cTarsMulti	    2	//4DOF ONLY
#define cTarsFactorA	70	//4DOF ONLY
#define cTarsFactorB	60	//4DOF ONLY
#define cTarsFactorC	50	//4DOF ONLY

#define DEFAULT_GAIT_SPEED  60
#define DEFAULT_SLOW_GAIT   70


#endif

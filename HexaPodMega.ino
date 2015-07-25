

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
#include "PhoenixCore.h"
#include "PhoenixInput.h"
#include "PhoenixInputSerial.h"

enum {
    MODE_WALK = 0,
    MODE_TRANSLATE,
    MODE_ROTATE,
    MODE_SINGLE_LEG
};

PhoenixCore  *core;
PhoenixInput *input;

s16       mBodyYOffset;
u16       mErrorCnt;
s16       mBodyYShift;
u8        mModeControl;
bool      mBoolDoubleHeight;
bool      mBoolDblTravel;
bool      mBoolWalk;

void setup()
{
    mBodyYOffset = 0;
    mBodyYShift  = 0;
    mErrorCnt    = 0;

    mModeControl      = MODE_WALK;
    mBoolDoubleHeight = FALSE;
    mBoolDblTravel    = FALSE;
    mBoolWalk         = FALSE;

#ifdef DBG_SERIAL
    DBG_SERIAL.begin(CONFIG_DEBUG_BAUD);
#endif

    core  = new PhoenixCore();
    input = new PhoenixInputSerial();

	core->init();
	input->init();

    printf(F("FREE RAM : %d\n"), freeRam());
}

#define BUTTON_PRESSED(stat, mask) (stat & (mask))

void loop()
{
	u32  dwButton;
    u8   lx, ly, rx, ry;
    bool fAdjustLegPositions = FALSE;

	dwButton = input->get(&lx, &ly, &rx, &ry);

    if (BUTTON_PRESSED(dwButton, INPUT_LEFT_ANALOG | INPUT_RIGHT_ANALOG))
        printf(F("LX:%3d, LY:%3d, RX:%3d, RY:%3d\n"), lx, ly, rx, ry);

    if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_ON_OFF)) {
    	if (core->getCtrl()->fHexOn) {
            mBodyYOffset = 0;
            mBodyYShift  = 0;
            core->initCtrl();
            fAdjustLegPositions = TRUE;
            printf(F("OFF\n"));
            Utils::sound(3, 100, 2500, 80, 2250, 60, 2000);
            digitalWrite(PIN_STATUS_LED, 0);
        } else {
            core->getCtrl()->fHexOn = TRUE;
        	fAdjustLegPositions = TRUE;
            printf(F("ON\n"));
            Utils::sound(3, 60, 2000, 80, 2250, 100, 2500);
            digitalWrite(PIN_STATUS_LED, 1);
        }
    }

    if (!core->getCtrl()->fHexOn)
        goto loop_exit;

    if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_SHIFT)) {
        Utils::sound( 1, 50, 2000);
        if (mModeControl != MODE_TRANSLATE )
            mModeControl = MODE_TRANSLATE;
        else {
            if (core->getCtrl()->bSingleLegCurSel == 255)
                mModeControl = MODE_WALK;
            else
                mModeControl = MODE_SINGLE_LEG;
        }
        printf(F("MODE:%d\n"), mModeControl);
    }

    if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_ROTATE)) {
        Utils::sound( 1, 50, 2000);
        if (mModeControl != MODE_ROTATE)
            mModeControl = MODE_ROTATE;
        else {
            if (core->getCtrl()->bSingleLegCurSel == 255)
                mModeControl = MODE_WALK;
            else
                mModeControl = MODE_SINGLE_LEG;
        }
        printf(F("MODE:%d\n"), mModeControl);
    }

    if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_SINGLE_LEG)) {
        if (abs(core->getCtrl()->c3dTravelLen.x) < CONFIG_TRAVEL_DEAD_ZONE && abs(core->getCtrl()->c3dTravelLen.z) < CONFIG_TRAVEL_DEAD_ZONE &&
            abs(core->getCtrl()->c3dTravelLen.y*2) < CONFIG_TRAVEL_DEAD_ZONE )   {
            if (mModeControl != MODE_SINGLE_LEG) {
                mModeControl = MODE_SINGLE_LEG;
            if (core->getCtrl()->bSingleLegCurSel == 255)
                core->getCtrl()->bSingleLegCurSel = PhoenixCore::IDX_RF;
            }
            else {
                mModeControl = MODE_WALK;
                core->getCtrl()->bSingleLegCurSel=255;
            }
        }
        printf(F("MODE:%d\n"), mModeControl);
    }

    if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_BALANCE)) {
        core->getCtrl()->fBalanceMode = !core->getCtrl()->fBalanceMode;
        if (core->getCtrl()->fBalanceMode) {
            Utils::sound(1, 250, 1500);
        } else {
            Utils::sound( 2, 100, 2000, 50, 4000);
        }
    }

    if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_BODY_HEIGHT)) {
        if (mBodyYOffset>0)
            mBodyYOffset = 0;
        else
            mBodyYOffset = 35;
        fAdjustLegPositions = TRUE;
    }

    if (BUTTON_PRESSED(dwButton, INPUT_BODY_UP)) {
        mBodyYOffset += 10;
        if (mBodyYOffset > MAX_BODY_Y)
            mBodyYOffset = MAX_BODY_Y;
        fAdjustLegPositions = TRUE;
    }

    if (BUTTON_PRESSED(dwButton, INPUT_BODY_DOWN) && mBodyYOffset) {
        if (mBodyYOffset > 10)
            mBodyYOffset -= 10;
        else
            mBodyYOffset = 0;      // constrain don't go less than zero.

        fAdjustLegPositions = TRUE;
    }

    if (BUTTON_PRESSED(dwButton, INPUT_SPEED_UP)) {
        if (core->getCtrl()->wSpeedControl > 0) {
            core->getCtrl()->wSpeedControl = core->getCtrl()->wSpeedControl - 50;
            Utils::sound( 1, 50, 2000);
        }
    }

    if (BUTTON_PRESSED(dwButton, INPUT_SPEED_DOWN)) {
        if (core->getCtrl()->wSpeedControl < 2000 ) {
            core->getCtrl()->wSpeedControl = core->getCtrl()->wSpeedControl + 50;
            Utils::sound( 1, 50, 2000);
        }
    }


    //[Walk functions]
    if (mModeControl == MODE_WALK) {
        //Switch gates
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_SEL) &&
          abs(core->getCtrl()->c3dTravelLen.x) < CONFIG_TRAVEL_DEAD_ZONE &&
          abs(core->getCtrl()->c3dTravelLen.z) < CONFIG_TRAVEL_DEAD_ZONE &&
          abs(core->getCtrl()->c3dTravelLen.y*2) < CONFIG_TRAVEL_DEAD_ZONE) {
            core->getCtrl()->bGaitType = core->getCtrl()->bGaitType + 1;    // Go to the next gait...
            if (core->getCtrl()->bGaitType < CONFIG_NUM_GAITS) {            // Make sure we did not exceed number of gaits...
                Utils::sound( 1, 50, 2000);
            } else {
                Utils::sound(2, 50, 2000, 50, 2250);
                core->getCtrl()->bGaitType = 0;
            }
            core->selectGait();
        }

        // Double leg lift height
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_R1)) {
            Utils::sound( 1, 50, 2000);
            mBoolDoubleHeight = !mBoolDoubleHeight;
            if (mBoolDoubleHeight)
                core->getCtrl()->sLegLiftHeight = 80;
            else
                core->getCtrl()->sLegLiftHeight = 50;
        }

        // Double Travel Length
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_R2)) {
            Utils::sound(1, 50, 2000);
            mBoolDblTravel = !mBoolDblTravel;
        }

        // Switch between Walk method 1 && Walk method 2
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_R3)) { // R3 Button Test
            Utils::sound(1, 50, 2000);
            mBoolWalk = !mBoolWalk;
        }

        //Walking
        if (mBoolWalk)  //(Walk Methode)
            core->getCtrl()->c3dTravelLen.z = (ry - 128); //Right Stick Up/Down
        else {
            core->getCtrl()->c3dTravelLen.x = -(lx - 128);
            core->getCtrl()->c3dTravelLen.z = (ly - 128);
        }

        if (!mBoolDblTravel) {  //(Double travel length)
            core->getCtrl()->c3dTravelLen.x = core->getCtrl()->c3dTravelLen.x/2;
            core->getCtrl()->c3dTravelLen.z = core->getCtrl()->c3dTravelLen.z/2;
        }
        core->getCtrl()->c3dTravelLen.y = -(rx - 128)/4; //Right Stick Left/Right
    }

    //[Translate functions]
    mBodyYShift = 0;
    if (mModeControl == MODE_TRANSLATE) {
        core->getCtrl()->c3dBodyPos.x = (ly - 128)/2;
        core->getCtrl()->c3dBodyPos.z = -(ly - 128)/3;
        core->getCtrl()->c3dBodyRot.y = (rx - 128)*2;
        mBodyYShift = (-(ry - 128)/2);
    } else if (mModeControl == MODE_ROTATE) {
        core->getCtrl()->c3dBodyRot.x = (ly - 128);
        core->getCtrl()->c3dBodyRot.y = (rx - 128)*2;
        core->getCtrl()->c3dBodyRot.z = (lx - 128);
        mBodyYShift = (-(ry - 128)/2);
    } else if (mModeControl == MODE_SINGLE_LEG) {
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_SEL)) { // Select Button Test
            Utils::sound(1, 50, 2000);
            if (core->getCtrl()->bSingleLegCurSel < 5)
                core->getCtrl()->bSingleLegCurSel = core->getCtrl()->bSingleLegCurSel + 1;
            else
                core->getCtrl()->bSingleLegCurSel = 0;
        }
        core->getCtrl()->c3dSingleLeg.x = (lx - 128) / 2;     // Left Stick Right/Left
        core->getCtrl()->c3dSingleLeg.y = (ry - 128) / 10;    // Right Stick Up/Down
        core->getCtrl()->c3dSingleLeg.z = (ly - 128) / 2;     // Left Stick Up/Down

        // Hold single leg in place
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_R2)) {
            Utils::sound(1, 50, 2000);
            core->getCtrl()->fSingleLegHold = !core->getCtrl()->fSingleLegHold;
        }
    }
    core->getCtrl()->bInputTimeDelay = 128 - max( max(abs(lx - 128), abs(ly - 128)),
                                             abs(rx - 128));

loop_exit:
    core->getCtrl()->c3dBodyPos.y = min(max(mBodyYOffset + mBodyYShift,  0), MAX_BODY_Y);

    if (fAdjustLegPositions)
        core->adjustLegPosToBodyHeight();    // Put main workings into main program file

    core->loop();
}

int freeRam() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}


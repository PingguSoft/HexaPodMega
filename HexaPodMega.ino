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
// Date : 12-07-2015
// Programmer : PingguSoft (pinggusoft@gmail.com)
//              Code rework for smartphone control and readability
//              Android App : BTCon4Drone
//               https://play.google.com/store/apps/details?id=com.pinggusoft.btcon
//=============================================================================

#define DEFINE_HEX_GLOBALS
#if ARDUINO > 99
#include <Arduino.h>
#endif
#include <Wire.h>
#include <EEPROM.h>

#include "common.h"
#include "ServoEx.h"
#include "config.h"
#include "PhoenixCore.h"
#include "PhoenixInput.h"
#include "PhoenixInputSerial.h"
#include "PhoenixInputBTCon.h"

enum {
    MODE_WALK = 0,
    MODE_TRANSLATE,
    MODE_ROTATE,
    MODE_SINGLE_LEG
};

PhoenixCore  *core;
PhoenixInput *input;
CTRL_STATE   ctrlState;

s16       mBodyYOffset;
u16       mErrorCnt;
s16       mBodyYShift;
u8        mModeControl;
bool      mBoolDoubleHeight;
bool      mBoolDblTravel;
bool      mBoolWalkMode2;


#if (CONFIG_CTRL_TYPE == CONFIG_CTRL_TYPE_BTCON)
u8 inputCallback(u8 cmd, u8 *data, u8 size, u8 *res)
{
    u8 ret = 0;

    switch (cmd) {
        case PhoenixInputBTCon::MSP_ANALOG:
            if (core) {
                u16 *ptr = (u16*)res;

                *ptr = core->getBattLevel();
                ret = 7;
            }
            break;
    }
    return ret;
}
#endif

void setup()
{
    mBodyYOffset = 0;
    mBodyYShift  = 0;
    mErrorCnt    = 0;

    mModeControl      = MODE_WALK;
    mBoolDoubleHeight = FALSE;
    mBoolDblTravel    = FALSE;
    mBoolWalkMode2    = FALSE;

#ifdef CONFIG_DBG_SERIAL
    CONFIG_DBG_SERIAL.begin(CONFIG_DEBUG_BAUD);
#endif

#if (CONFIG_CTRL_TYPE == CONFIG_CTRL_TYPE_SERIAL)
    input = new PhoenixInputSerial();
	input->init(NULL);
#elif (CONFIG_CTRL_TYPE == CONFIG_CTRL_TYPE_BTCON)
    input = new PhoenixInputBTCon();
	input->init(inputCallback);
#else
    #error No Controller !!
#endif

    core  = new PhoenixCore(&ctrlState);
	core->init();

    printf(F("FREE RAM : %d\n"), freeRam());
}

#define BUTTON_PRESSED(stat, mask) (stat & (mask))

void loop()
{
	u32  dwButton;
    u8   lx, ly, rx, ry;
    bool fAdjustLegPositions = FALSE;

	dwButton = input->get(&lx, &ly, &rx, &ry);

//    if (BUTTON_PRESSED(dwButton, INPUT_LEFT_ANALOG | INPUT_RIGHT_ANALOG))
//        printf(F("LX:%3d, LY:%3d, RX:%3d, RY:%3d\n"), lx, ly, rx, ry);

    if (!dwButton)
        goto loop_exit;

    if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_ON_OFF)) {
    	if (ctrlState.fHexOn) {
            mBodyYOffset = 0;
            mBodyYShift  = 0;
            core->initCtrl();
            printf(F("OFF\n"));
            Utils::sound(3, 100, 2500, 80, 2250, 60, 2000);
            digitalWrite(PIN_STATUS_LED, 0);
        } else {
            ctrlState.fHexOn = TRUE;
            printf(F("ON\n"));
            Utils::sound(3, 60, 2000, 80, 2250, 100, 2500);
            digitalWrite(PIN_STATUS_LED, 1);
        }
        fAdjustLegPositions = TRUE;
    }

    if (!ctrlState.fHexOn)
        goto loop_exit;

    // Switch between Walk method 1 && Walk method 2
    if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_WALK)) { // R3 Button Test
        Utils::sound(1, 50, 2000);
        mBoolWalkMode2 = !mBoolWalkMode2;
    }

    if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_SHIFT)) {
        Utils::sound( 1, 50, 2000);
        if (mModeControl != MODE_TRANSLATE )
            mModeControl = MODE_TRANSLATE;
        else {
            if (ctrlState.bSingleLegCurSel == 255)
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
            if (ctrlState.bSingleLegCurSel == 255)
                mModeControl = MODE_WALK;
            else
                mModeControl = MODE_SINGLE_LEG;
        }
        printf(F("MODE:%d\n"), mModeControl);
    }

    if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_SINGLE_LEG)) {
        if (abs(ctrlState.c3dTravelLen.x) < CONFIG_TRAVEL_DEAD_ZONE && abs(ctrlState.c3dTravelLen.z) < CONFIG_TRAVEL_DEAD_ZONE &&
            abs(ctrlState.c3dTravelLen.y * 2) < CONFIG_TRAVEL_DEAD_ZONE )   {
            if (mModeControl != MODE_SINGLE_LEG) {
                mModeControl = MODE_SINGLE_LEG;
            if (ctrlState.bSingleLegCurSel == 255)
                ctrlState.bSingleLegCurSel = PhoenixCore::IDX_RF;
            }
            else {
                mModeControl = MODE_WALK;
                ctrlState.bSingleLegCurSel=255;
            }
        }
        printf(F("MODE:%d\n"), mModeControl);
    }

    if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_BALANCE)) {
        ctrlState.fBalanceMode = !ctrlState.fBalanceMode;
        if (ctrlState.fBalanceMode) {
            Utils::sound(1, 250, 1500);
        } else {
            Utils::sound(2, 100, 2000, 50, 4000);
        }
    }

    if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_BODY_HEIGHT)) {
        if (mBodyYOffset > 0)
            mBodyYOffset = 0;
        else
            mBodyYOffset = 35;
        fAdjustLegPositions = TRUE;
    }

    if (input->getBodyHeight() & INPUT_HEIGHT_SUPPORTED) {
        mBodyYOffset = input->getBodyHeight() & INPUT_HEIGHT_MASK;
        if (mBodyYOffset > MAX_BODY_Y)
                mBodyYOffset = MAX_BODY_Y;
        fAdjustLegPositions = TRUE;
    } else {
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
                mBodyYOffset = 0;
            fAdjustLegPositions = TRUE;
        }
    }

    if (BUTTON_PRESSED(dwButton, INPUT_SPEED_UP)) {
        if (ctrlState.wSpeedControl > 0) {
            ctrlState.wSpeedControl = ctrlState.wSpeedControl - 50;
            Utils::sound( 1, 50, 2000);
        }
    }

    if (BUTTON_PRESSED(dwButton, INPUT_SPEED_DOWN)) {
        if (ctrlState.wSpeedControl < 2000 ) {
            ctrlState.wSpeedControl = ctrlState.wSpeedControl + 50;
            Utils::sound( 1, 50, 2000);
        }
    }


    if (mModeControl == MODE_WALK) {
        //Switch gaits
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_SEL)) {
            printf(F("x:%d, y:%d, z:%d\n"), ctrlState.c3dTravelLen.x, ctrlState.c3dTravelLen.y, ctrlState.c3dTravelLen.z);
            if (abs(ctrlState.c3dTravelLen.x) < CONFIG_TRAVEL_DEAD_ZONE &&
                abs(ctrlState.c3dTravelLen.z) < CONFIG_TRAVEL_DEAD_ZONE &&
                abs(ctrlState.c3dTravelLen.y*2) < CONFIG_TRAVEL_DEAD_ZONE) {
                ctrlState.bGaitType = ctrlState.bGaitType + 1;    // Go to the next gait...
                if (ctrlState.bGaitType < CONFIG_NUM_GAITS) {            // Make sure we did not exceed number of gaits...
                    Utils::sound( 1, 50, 2000);
                } else {
                    Utils::sound(2, 50, 2000, 50, 2250);
                    ctrlState.bGaitType = 0;
                }
                core->selectGait(ctrlState.bGaitType);
                printf(F("GAIT:%d\n"), ctrlState.bGaitType);
            } else {
                printf(F("GAIT can not be changed:%d\n"), ctrlState.bGaitType);
            }
        }

        // Double leg lift height
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_R1)) {
            Utils::sound( 1, 50, 2000);
            mBoolDoubleHeight = !mBoolDoubleHeight;
            if (mBoolDoubleHeight)
                ctrlState.sLegLiftHeight = 80;
            else
                ctrlState.sLegLiftHeight = 50;
        }

        // Double Travel Length
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_R2)) {
            Utils::sound(1, 50, 2000);
            mBoolDblTravel = !mBoolDblTravel;
        }

        if (mBoolWalkMode2)
            ctrlState.c3dTravelLen.z = (ry - 128); //Right Stick Up/Down
        else {
            ctrlState.c3dTravelLen.x = -(lx - 128);
            ctrlState.c3dTravelLen.z = (ly - 128);
        }

        if (!mBoolDblTravel) {
            ctrlState.c3dTravelLen.x = ctrlState.c3dTravelLen.x / 2;
            ctrlState.c3dTravelLen.z = ctrlState.c3dTravelLen.z / 2;
        }

        ctrlState.c3dTravelLen.y = -(rx - 128)/4; //Right Stick Left/Right
    }

    //[Translate functions]
    mBodyYShift = 0;
    if (mModeControl == MODE_TRANSLATE) {
        ctrlState.c3dBodyPos.x = (ly - 128)/2;
        ctrlState.c3dBodyPos.z = -(ly - 128)/3;
        ctrlState.c3dBodyRot.y = (rx - 128)*2;
        mBodyYShift = (-(ry - 128)/2);
    } else if (mModeControl == MODE_ROTATE) {
        ctrlState.c3dBodyRot.x = (ly - 128);
        ctrlState.c3dBodyRot.y = (rx - 128)*2;
        ctrlState.c3dBodyRot.z = (lx - 128);
        mBodyYShift = (-(ry - 128)/2);
    } else if (mModeControl == MODE_SINGLE_LEG) {
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_SEL)) { // Select Button Test
            Utils::sound(1, 50, 2000);
            if (ctrlState.bSingleLegCurSel < 5)
                ctrlState.bSingleLegCurSel = ctrlState.bSingleLegCurSel + 1;
            else
                ctrlState.bSingleLegCurSel = 0;
        }
        ctrlState.c3dSingleLeg.x = (lx - 128) / 2;     // Left Stick Right/Left
        ctrlState.c3dSingleLeg.y = (ry - 128) / 10;    // Right Stick Up/Down
        ctrlState.c3dSingleLeg.z = (ly - 128) / 2;     // Left Stick Up/Down

        // Hold single leg in place
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_R2)) {
            Utils::sound(1, 50, 2000);
            ctrlState.fSingleLegHold = !ctrlState.fSingleLegHold;
        }
    }
    ctrlState.bInputTimeDelay = 128 - max( max(abs(lx - 128), abs(ly - 128)),
                                             abs(rx - 128));
    ctrlState.c3dBodyPos.y = min(max(mBodyYOffset + mBodyYShift,  0), MAX_BODY_Y);

loop_exit:
    if (fAdjustLegPositions)
        core->adjustLegPosToBodyHeight();    // Put main workings into main program file

    core->loop();

    ctrlState.fHexOnOld = ctrlState.fHexOn;
}

int freeRam() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}


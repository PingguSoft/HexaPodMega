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
#include <SoftwareSerial.h>

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

u8        mColor;
s16       mBodyYOffset;
u16       mErrorCnt;
s16       mBodyYShift;
u8        mModeControl;
bool      mBoolDoubleHeight;
bool      mBoolDblTravel;
bool      mBoolWalkMode2;



int freeRam() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}


#if (CONFIG_CTRL_TYPE == CONFIG_CTRL_TYPE_BTCON)
u8 scale = 30;
s8 inputCallback(u8 cmd, u8 *data, u8 size, u8 *res)
{
    s8 ret = -1;

    switch (cmd) {
        case PhoenixInputBTCon::MSP_ANALOG:
            if (core) {
                u8 *ptr = (u8*)res;

                *ptr = core->getBattLevel(scale);
                *(ptr + 3) = core->getBattLevel();
                ret = 7;
            }
            break;

        case PhoenixInputBTCon::MSP_SET_MISC:
            scale = *(data + 18);
            ret = 0;
            break;
    }
    return ret;
}
#endif

enum {
    COLOR_BLACK  = 0,
    COLOR_RED    = 1,
    COLOR_GREEN  = 2,
    COLOR_YELLOW = 3,
    COLOR_BLUE   = 4,
    COLOR_PURPLE = 5,
    COLOR_CYAN   = 6,
    COLOR_WHITE  = 7
};

void showLED(u8 color)
{
    digitalWrite(PIN_STATUS_RED,   color & 0x01);
    digitalWrite(PIN_STATUS_GREEN, color & 0x02);
    digitalWrite(PIN_STATUS_BLUE,  color & 0x04);
}

void setup()
{
    pinMode(PIN_STATUS_RED,   OUTPUT);
    pinMode(PIN_STATUS_GREEN, OUTPUT);
    pinMode(PIN_STATUS_BLUE,  OUTPUT);
    pinMode(PIN_SOUND, OUTPUT);

    mColor       = 0;
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

void turnOff(void)
{
    mBodyYOffset = 0;
    mBodyYShift  = 0;
    core->initCtrl();
    Utils::sound(400, 0, 0, 100, 300);
    showLED(COLOR_BLACK);
    printf(F("OFF\n"));
}

void loop()
{
    u32  dwButton;
    u8   lx, ly, rx, ry;
    u8   ret;
    bool fAdjustLegPositions = FALSE;

	dwButton = input->get(&lx, &ly, &rx, &ry);

    if (!dwButton)
        goto loop_exit;

    if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_ON_OFF)) {
    	if (ctrlState.fHexOn) {
            turnOff();
        } else {
            ctrlState.fHexOn = TRUE;
            printf(F("ON\n"));
            Utils::sound(200, 200, 0, 100, 300);
            showLED(1);
        }
        fAdjustLegPositions = TRUE;
    }

    if (!ctrlState.fHexOn)
        goto loop_exit;

    // Switch between Walk method 1 && Walk method 2
    if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_WALK)) { // R3 Button Test
        Utils::sound(100, 100, 100, 50, 300);
        mBoolWalkMode2 = !mBoolWalkMode2;
    }

    if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_SHIFT)) {
        Utils::sound(100, 0, 0, 50, 300);
        if (mModeControl != MODE_TRANSLATE )
            mModeControl = MODE_TRANSLATE;
        else
            mModeControl = MODE_WALK;
        printf(F("MODE:%d\n"), mModeControl);
    }

    if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_ROTATE)) {
        Utils::sound(300, 0, 0, 50, 300);
        if (mModeControl != MODE_ROTATE)
            mModeControl = MODE_ROTATE;
        else
            mModeControl = MODE_WALK;
        printf(F("MODE:%d\n"), mModeControl);
    }

    if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_SINGLE_LEG)) {
        if (abs(ctrlState.c3dTravelLen.x) < CONFIG_TRAVEL_DEAD_ZONE && abs(ctrlState.c3dTravelLen.z) < CONFIG_TRAVEL_DEAD_ZONE &&
            abs(ctrlState.c3dTravelLen.y * 2) < CONFIG_TRAVEL_DEAD_ZONE )   {
            Utils::sound(300, 0, 0, 50, 300);

            if (mModeControl != MODE_SINGLE_LEG) {
                mModeControl = MODE_SINGLE_LEG;

            if (ctrlState.bSingleLegCurSel == 255)
                ctrlState.bSingleLegCurSel = PhoenixCore::IDX_RF;
            } else {
                mModeControl = MODE_WALK;
                ctrlState.bSingleLegCurSel = 255;
            }
        }
        printf(F("MODE:%d\n"), mModeControl);
    }

    if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_BALANCE)) {
        ctrlState.fBalanceMode = !ctrlState.fBalanceMode;
        if (ctrlState.fBalanceMode) {
            Utils::sound(30, 30, 0, 50, 300);
        } else {
            Utils::sound(300, 0, 0, 50, 300);
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
            Utils::sound(300, 0, 0, 50, 300);
        }
    }

    if (BUTTON_PRESSED(dwButton, INPUT_SPEED_DOWN)) {
        if (ctrlState.wSpeedControl < 2000 ) {
            ctrlState.wSpeedControl = ctrlState.wSpeedControl + 50;
            Utils::sound(300, 0, 0, 50, 300);
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
                if (ctrlState.bGaitType < NUM_GAITS) {            // Make sure we did not exceed number of gaits...
                    Utils::sound(300, 0, 0, 50, 300);
                } else {
                    Utils::sound(100, 100, 0, 50, 300);
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
            Utils::sound(300, 0, 0, 50, 300);
            mBoolDoubleHeight = !mBoolDoubleHeight;
            if (mBoolDoubleHeight)
                ctrlState.sLegLiftHeight = 80;
            else
                ctrlState.sLegLiftHeight = 50;
        }

        // Double Travel Length
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_R2)) {
            Utils::sound(300, 0, 0, 50, 300);
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

    mColor = 1 + mModeControl + (mBoolDblTravel ? 4 : 0);

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
            Utils::sound(300, 0, 0, 50, 300);
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
            Utils::sound(300, 0, 0, 50, 300);
            ctrlState.fSingleLegHold = !ctrlState.fSingleLegHold;
        }
    }

    ctrlState.bInputTimeDelay = 128 - max( max(abs(lx - 128), abs(ly - 128)),
                                             abs(rx - 128));
    ctrlState.c3dBodyPos.y = min(max(mBodyYOffset + mBodyYShift,  0), MAX_BODY_Y);

loop_exit:
    if (fAdjustLegPositions)
        core->adjustLegPosToBodyHeight();

    showLED(mColor);

    ret = core->loop();
#if 0
    if (ctrlState.fHexOn && (ret & PhoenixCore::STATUS_BATT_FAIL)) {
        turnOff();
        ctrlState.c3dBodyPos.y = 0;
        core->adjustLegPosToBodyHeight();
        core->loop();
    } else if (ret & PhoenixCore::STATUS_BATT_WARN) {
        Utils::sound(50, 50, 50, 50, 300);
    }
#endif
    Utils::handleSound();

    ctrlState.fHexOnOld = ctrlState.fHexOn;
}


#include <Arduino.h> // Arduino 1.0
#include <avr/pgmspace.h>
#include "utils.h"
#include "PhoenixInputSerial.h"

PhoenixInputSerial::PhoenixInputSerial(void)
{
    mLX = 128;
    mLY = 128;
    mRX = 128;
    mRY = 128;
}

void PhoenixInputSerial::init(void)
{
    //BT_SERIAL.begin(CONFIG_BT_BAUD);
    printf(F("%s\n"), __PRETTY_FUNCTION__);
}

//==============================================================================
// This is The main code to input function to read inputs from the PS2 and then
//process any commands.
//==============================================================================
u32 PhoenixInputSerial::get(u8 *lx, u8 *ly, u8 *rx, u8 *ry)
{
    int cmd;

    *lx = mLX;
    *ly = mLY;
    *rx = mRX;
    *ry = mRY;

    if (BT_SERIAL.available() == 0)
        return 0;

    cmd = BT_SERIAL.read();
    printf(F("cmd:%c\n"), cmd);

    switch(cmd) {
        case 'w':
            return INPUT_BODY_UP;

        case 's':
            return INPUT_BODY_DOWN;

        case 'a':
            return INPUT_SPEED_DOWN;

        case 'd':
            return INPUT_SPEED_UP;

        case ' ':
            *lx = mLX = 128;
            *ly = mLY = 128;
            *rx = mRX = 128;
            *ry = mRY = 128;
            return INPUT_TOGGLE_ON_OFF;

        case 'z':
            return INPUT_OPT_SEL;

        case '1':
            return INPUT_TOGGLE_SHIFT;

        case 'q':
            return INPUT_TOGGLE_ROTATE;

        case '3':
            return INPUT_OPT_R1;

        case 'e':
            return INPUT_OPT_R2;

        // Left Joystick
        case 't':
            if (mLY > 10)
                mLY -= 10;
            *ly = mLY;
            return INPUT_LEFT_ANALOG;

        case 'g':
            if (mLY < 245)
                mLY += 10;
            *ly = mLY;
            return INPUT_LEFT_ANALOG;

        case 'f':
            if (mLX > 10)
                mLX -= 10;
            *lx = mLX;
            return INPUT_LEFT_ANALOG;

        case 'h':
            if (mLX < 245)
                mLX += 10;
            *lx = mLX;
            return INPUT_LEFT_ANALOG;

        // Right Joystick
        case 'i':
            if (mRY > 10)
                mRY -= 10;
            *ry = mRY;
            return INPUT_RIGHT_ANALOG;

        case 'k':
            if (mRY < 245)
                mRY += 10;
            *ry = mRY;
            return INPUT_RIGHT_ANALOG;

        case 'j':
            if (mRX > 10)
                mRX -= 10;
            *rx = mRX;
            return INPUT_RIGHT_ANALOG;

        case 'l':
            if (mRX < 245)
                mRX += 10;
            *rx = mRX;
            return INPUT_RIGHT_ANALOG;

        // buttons
        case ',':
            return INPUT_TOGGLE_SINGLE_LEG;

        case '.':
            return INPUT_TOGGLE_BALANCE;

        case '/':
            return INPUT_TOGGLE_BODY_HEIGHT;
    }

    return 0;
}


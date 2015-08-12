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

#include <Arduino.h> // Arduino 1.0
#include <avr/pgmspace.h>
#include "utils.h"
#include "PhoenixInputBTCon.h"

PhoenixInputBTCon::PhoenixInputBTCon(void)
{
    mLX = 128;
    mLY = 128;
    mRX = 128;
    mRY = 128;
    mState = STATE_IDLE;
    mOldButtons = 0;
}

void PhoenixInputBTCon::init(s8 (*callback)(u8 cmd, u8 *data, u8 size, u8 *res))
{
    printf(F("%s\n"), __PRETTY_FUNCTION__);
    CONFIG_CTRL_SERIAL.begin(CONFIG_CTRL_BAUD);
    mCallback = callback;
}

static u8 chkSumTX;
void putChar2TX(u8 data)
{
    chkSumTX ^= data;
    CONFIG_CTRL_SERIAL.write(data);
}

void PhoenixInputBTCon::sendResponse(bool ok, u8 cmd, u8 *data, u8 size)
{
    putChar2TX('$');
    putChar2TX('M');
    putChar2TX((ok ? '>' : '!'));
    chkSumTX = 0;
    putChar2TX(size);
    putChar2TX(cmd);
    for (u8 i = 0; i < size; i++)
        putChar2TX(*data++);
    putChar2TX(chkSumTX);
}

void PhoenixInputBTCon::evalCommand(u8 cmd, u8 *data, u8 size)
{
    static  u8 batt = 0;
    static u16 wmCycleTime = 0;

    u8  buf[22];
    u16 *rc;

    memset(&buf, 0, sizeof(buf));
    switch (cmd) {
        case MSP_IDENT:
            buf[0] = 240;
            buf[1] = 3;
            sendResponse(TRUE, cmd, buf, 7);
            break;

        case MSP_STATUS:
            *((u16*)&buf[0]) = wmCycleTime++;
            sendResponse(TRUE, cmd, buf, 11);
            break;

        case MSP_MISC:
            rc = (u16*)buf;
            rc[2] = 2000;
            rc[3] = 1000;
            rc[4] = 1000;
            buf[18] = 100;
            buf[19] = 110;
            buf[20] = 105;
            buf[21] = 100;
            sendResponse(TRUE, cmd, buf, 22);
            break;

        case MSP_SET_RAW_RC:
            rc = (u16*)data;

            mLX = min(255, (*rc - 1000) * 10 / 39);   // 1000 -> 255 range, roll
            rc++;
            mLY = min(255, (*rc - 1000) * 10 / 39);   // 1000 -> 255 range, pitch
            mLY = 255 - mLY;
            rc++;
            mRX = min(255, (*rc - 1000) * 10 / 39);   // 1000 -> 255 range, yaw
            rc++;
            mRY = min(255, (*rc - 1000) * 10 / 39);   // 1000 -> 255 range, throttle
            rc++;

            mButtons &= 0xfff0;
            for (u8 i = 0; i < 4; i++) {                // AUX1 - AUX4
                if (*rc++ > 1700) {
                    mButtons |= (1 << i);
                }
            }
            break;

        case MSP_SET_USER_BUTTON:
            printf(F("SW:%02x\n"), *data);
            mButtons &= 0x000f;
            mButtons |= (*data << 4);                   // SW BUTTON 5 - 10
            sendResponse(TRUE, cmd, buf, 0);
            break;

        default:
            if (mCallback) {
                s8 ret = (*mCallback)(cmd, data, size, buf);
                if (ret >= 0)
                    sendResponse(TRUE, cmd, buf, ret);
            }
            break;
    }
}

u8 PhoenixInputBTCon::handleRX(void)
{
    u8 ret = 0;
    u8 rxSize = CONFIG_CTRL_SERIAL.available();

    if (rxSize == 0)
        return ret;

    while (rxSize--) {
        u8 ch = CONFIG_CTRL_SERIAL.read();

        switch (mState) {
            case STATE_IDLE:
                if (ch == '$')
                    mState = STATE_HEADER_START;
                break;

            case STATE_HEADER_START:
                mState = (ch == 'M') ? STATE_HEADER_M : STATE_IDLE;
                break;

            case STATE_HEADER_M:
                mState = (ch == '<') ? STATE_HEADER_ARROW : STATE_IDLE;
                break;

            case STATE_HEADER_ARROW:
                if (ch > MAX_PACKET_SIZE) { // now we are expecting the payload size
                    mState = STATE_IDLE;
                    continue;
                }
                mDataSize = ch;
                mCheckSum = ch;
                mOffset   = 0;
                mState    = STATE_HEADER_SIZE;
                break;

            case STATE_HEADER_SIZE:
                mCmd       = ch;
                mCheckSum ^= ch;
                mState     = STATE_HEADER_CMD;
                break;

            case STATE_HEADER_CMD:
                if (mOffset < mDataSize) {
                    mCheckSum           ^= ch;
                    mRxPacket[mOffset++] = ch;
                } else {
                    if (mCheckSum == ch) {
                        ret = mCmd;
                        evalCommand(ret, mRxPacket, mDataSize);
                    }
                    mState = STATE_IDLE;
                    //rxSize = 0;             // no more than one command per cycle
                }
                break;
        }
    }
    return ret;
}

u32 PhoenixInputBTCon::get(u8 *lx, u8 *ly, u8 *rx, u8 *ry)
{
    u8  cmd;
    u32 diff = 0;

    cmd = handleRX();
    if (cmd == 0)
        return diff;

    *lx = mLX;
    *ly = mLY;
    *rx = mRX;
    *ry = 128;

    switch (cmd) {
        case MSP_SET_RAW_RC:
        case MSP_SET_USER_BUTTON:
            diff = INPUT_LEFT_ANALOG | INPUT_RIGHT_ANALOG;
            diff |= (mButtons ^ mOldButtons);
            if (diff & INPUT_BUTTON_MASK)
                printf(F("BUTTON:%04x => %04x [%04x]\n"), mOldButtons, mButtons, diff);
            mOldButtons = mButtons;
            break;
    }
    return diff;
}

u8 PhoenixInputBTCon::getBodyHeight(void)
{
    u16 temp = MAX_BODY_Y * mRY / 255;
    return (INPUT_HEIGHT_SUPPORTED | min(127, temp));
}

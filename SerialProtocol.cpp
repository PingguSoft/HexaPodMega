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

// For Arduino 1.0 and earlier
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "config.h"
#include "SerialProtocol.h"
#include "utils.h"


SerialProtocol::SerialProtocol()
{
    mState = STATE_IDLE;
}

SerialProtocol::~SerialProtocol()
{
}

void SerialProtocol::begin(u32 baud)
{
    BT_SERIAL.begin(baud);
}

void SerialProtocol::sendString_P(const char *fmt, ...)
{
    char buf[128]; // resulting string limited to 128 chars

    va_list args;
    va_start (args, fmt);

#ifdef __AVR__
    vsnprintf_P(buf, sizeof(buf), (const char *)fmt, args); // progmem for AVR
#else
    vsnprintf(buf, sizeof(buf), (const char *)fmt, args); // for the rest of the world
#endif
    va_end(args);

    BT_SERIAL.write(buf);
}

void SerialProtocol::sendString(char *fmt, ...)
{
    char buf[128]; // resulting string limited to 128 chars
    va_list args;

    va_start (args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    BT_SERIAL.write(buf);
}


u8 SerialProtocol::getString(u8 *buf)
{
    u8 size = BT_SERIAL.available();

    for (u8 i = 0; i < size; i++)
        *buf++ = BT_SERIAL.read();

    return size;
}

void SerialProtocol::setCallback(u32 (*callback)(u8 cmd, u8 *data, u8 size))
{
    mCallback = callback;
}

static u8 chkSumTX;

void putChar2TX(u8 data)
{
    chkSumTX ^= data;
    BT_SERIAL.write(data);
}

void SerialProtocol::sendResponse(bool ok, u8 cmd, u8 *data, u8 size)
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

void SerialProtocol::evalCommand(u8 cmd, u8 *data, u8 size)
{
    static u8 batt = 0;

    switch (cmd) {
        case CMD_TEST:
            u8 buf[7];
            buf[0] = batt++;
            sendResponse(true, cmd, buf, 7);
            break;

        default:
            if (mCallback)
                (*mCallback)(cmd, data, size);
            break;
    }
}

u8 SerialProtocol::handleRX(void)
{
    u8 ret = 0;
    u8 rxSize = BT_SERIAL.available();

    if (rxSize == 0)
        return ret;

    while (rxSize--) {
        u8 ch = BT_SERIAL.read();

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
                        evalCommand(mCmd, mRxPacket, mDataSize);
                    }
                    mState = STATE_IDLE;
                    rxSize = 0;             // no more than one command per cycle
                }
                break;
        }
    }
    return ret;
}

u8 *SerialProtocol::getPacket(u8 *size)
{
    *size = mDataSize;
    return mRxPacket;
}

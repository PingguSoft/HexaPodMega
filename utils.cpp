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

#include <stdarg.h>
#include <EEPROM.h>
#include "config.h"
#include "common.h"
#include "utils.h"

#ifdef CONFIG_DBG_SERIAL
void printf(char *fmt, ... )
{
    char buf[128]; // resulting string limited to 128 chars
    va_list args;
    va_start (args, fmt );
    vsnprintf(buf, 128, fmt, args);
    va_end (args);
    CONFIG_DBG_SERIAL.print(buf);
}

void printf(const __FlashStringHelper *fmt, ... )
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
    CONFIG_DBG_SERIAL.print(buf);
}
#else
void printf(char *fmt, ... )
{
}
void printf(const __FlashStringHelper *fmt, ... )
{
}
#endif

static bool mBoolSeqActive = FALSE;
static bool mBoolCycleDone = FALSE;
static bool mBoolOn = FALSE;
static u16  mPatterns[5];
static u8   mIdx = 0;
static u32  mLastToggleTime = 0;

void soundOff(void){
    if (mBoolOn)
        digitalWrite(PIN_SOUND, LOW);
}

void setTiming(u16 pulse, u16 pause)
{
    if (!mBoolOn && (millis() >= (mLastToggleTime + pause)) && pulse != 0) {
        mBoolOn = TRUE;
        digitalWrite(PIN_SOUND, HIGH);
        mLastToggleTime = millis();
    } else if (mBoolOn && (pulse == 0 || (millis() >= mLastToggleTime + pulse))) {
        mBoolOn         = FALSE;
        mBoolCycleDone  = TRUE;
        digitalWrite(PIN_SOUND, LOW);
        mLastToggleTime = millis();
    }
}

void Utils::handleSound(void) {
    if (mBoolSeqActive) {
        if (mIdx < 3) {
            if (mPatterns[mIdx] != 0)
                setTiming(mPatterns[mIdx], mPatterns[4]);
        } else if (mLastToggleTime < (millis() - mPatterns[3]))  {  //sequence is over: reset everything
            mIdx = 0;
            mBoolSeqActive = FALSE;                               //sequence is now done, mBoolCycleDone sequence may begin
            soundOff();
            return;
        }

        if (mBoolCycleDone || mPatterns[mIdx] == 0) {            //single on off cycle is done
            if (mIdx < 3) {
                mIdx++;
            }
            mBoolCycleDone = FALSE;
            soundOff();
        }
    } else {
        soundOff();
    }
}

void Utils::sound(u16 first,u16 second,u16 third,u16 cyclepause, u16 endpause)
{
    if (!mBoolSeqActive) {
        mBoolSeqActive = TRUE;
        mPatterns[0] = first;
        mPatterns[1] = second;
        mPatterns[2] = third;
        mPatterns[3] = endpause;
        mPatterns[4] = cyclepause;
        handleSound();
    }
}

void Utils::dumpEEPROM(u16 addr, u16 cnt)
{
    u8  i;
    u8  b;

    while (cnt) {
        printf(F("%08x - "), addr);

        for (i = 0; (i < 16) && (i < cnt); i ++) {
            b = EEPROM.read(addr + i);
            printf(F("%02x "), b);
        }

        printf(F(" : "));
        for (i = 0; (i < 16) && (i < cnt); i ++) {
            b = EEPROM.read(addr + i);
            if ((b > 0x1f) && (b < 0x7f))
                printf(F("%c"), b);
            else
                printf(F("."));
        }
        printf(F("\n"));
        addr += i;
        cnt  -= i;
    }
}

u16 Utils::getCmdLineNum(byte **ppszCmdLine)
{
    u8  *psz = *ppszCmdLine;
    u16 w = 0;

    while (*psz == ' ')
        psz++;

    if ((*psz == '0') && ((*(psz+1) == 'x') || (*(psz+1) == 'X'))) {
        // Hex mode
        psz += 2;
        for (;;) {
            if ((*psz >= '0') && (*psz <= '9'))
                w = w * 16 + *psz++ - '0';
            else if ((*psz >= 'a') && (*psz <= 'f'))
                w = w * 16 + *psz++ - 'a' + 10;
            else if ((*psz >= 'A') && (*psz <= 'F'))
                w = w * 16 + *psz++ - 'A' + 10;
            else
                break;
        }
    } else {
        // decimal mode
        while ((*psz >= '0') && (*psz <= '9'))
            w = w * 10 + *psz++ - '0';
    }
    *ppszCmdLine = psz;

    return w;
}

/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is derived from deviationTx project for Arduino.

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
#endif


//==============================================================================
//    makeSound - Quick and dirty tone function to try to output a frequency
//            to a speaker for some simple sounds.
//==============================================================================
#ifdef PIN_SOUND
void Utils::makeSound(unsigned long duration,  unsigned int frequency)
{
    volatile uint8_t *pin_port;
    volatile uint8_t pin_mask;

    long toggle_count = 0;
    long lusDelayPerHalfCycle;

    // Set the pinMode as OUTPUT
    pinMode(PIN_SOUND, OUTPUT);

    pin_port = portOutputRegister(digitalPinToPort(PIN_SOUND));
    pin_mask = digitalPinToBitMask(PIN_SOUND);

    toggle_count = 2 * frequency * duration / 1000;
    lusDelayPerHalfCycle = 1000000L/(frequency * 2);

    // if we are using an 8 bit timer, scan through prescalars to find the best fit
    while (toggle_count--) {
        // toggle the pin
        *pin_port ^= pin_mask;

        // delay a half cycle
        delayMicroseconds(lusDelayPerHalfCycle);
    }
  *pin_port &= ~(pin_mask);  // keep pin low after stop
}

void Utils::sound(u8 notes, ...)
{
    va_list ap;
    unsigned int uDur;
    unsigned int uFreq;
    va_start(ap, notes);

    while (notes > 0) {
        uDur = va_arg(ap, unsigned int);
        uFreq = va_arg(ap, unsigned int);
        Utils::makeSound(uDur, uFreq);
        notes--;
    }
    va_end(ap);
}
#else
void Utils::sound(u8 cNotes, ...)
{
};
#endif


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
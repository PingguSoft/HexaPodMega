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

#ifndef _UTILS_H_
#define _UTILS_H_
#include <Arduino.h>
#include <avr/pgmspace.h>
#include "config.h"

// Bit vector from bit position
#define BV(bit) (1 << (bit))

void printf(char *fmt, ... );
void printf(const __FlashStringHelper *fmt, ... );

class Utils {
private:
public:
    static void handleSound(void);
    static void sound(u16 first,u16 second,u16 third,u16 cyclepause, u16 endpause);
    static void dumpEEPROM(u16 addr, u16 cnt);
    static u16  getCmdLineNum(byte **ppszCmdLine);
};

#endif

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

#ifndef _PHOENIX_SERVO_H_
#define _PHOENIX_SERVO_H_

#include "config.h"
#include "utils.h"

class PhoenixServo
{
private:


public:
    virtual void init(void);
    virtual void start(void);
    virtual void commit(word wMoveTime);
    virtual void release(void);
#if (CONFIG_DOF_PER_LEG == 4)
    virtual void write(u8 LegIndex, s16 sCoxaAngle1, s16 sFemurAngle1, s16 sTibiaAngle1, s16 sTarsAngle1);
#else
    virtual void write(u8 LegIndex, s16 sCoxaAngle1, s16 sFemurAngle1, s16 sTibiaAngle1);
#endif

    virtual u16  getBattVolt(void);
    virtual bool checkVoltage(void);
    virtual void processBackground(void);

#ifdef CONFIG_TERMINAL
    virtual void showTerminal(void);
    virtual bool handleTerminal(u8 *psz, u8 bLen);
#endif
};

#endif

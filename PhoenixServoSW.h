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

#ifndef _PHOENIX_SERVO_SW_H_
#define _PHOENIX_SERVO_SW_H_

#include "config.h"
#include "utils.h"
#include "ServoEx.h"
#include "PhoenixServo.h"

class PhoenixServoSW : public PhoenixServo
{
private:
    ServoEx         mServoLegs[CONFIG_NUM_LEGS * CONFIG_DOF_PER_LEG];        // Define the servo objects...
    s16             mServoOffsets[CONFIG_NUM_LEGS * CONFIG_DOF_PER_LEG];
    ServoGroupMove  mSGM;
    bool            mBoolServosAttached;

    u16             mVoltBuf[8];
    u16             mVoltSum;
    u8              mVoltIdx;

    void loadServosConfig(void);
    void attachServos(void);
    void setLegs1500ms(void);
    void handleServoOffsets(void);


public:
    PhoenixServoSW(void);

    virtual void init(void);
    virtual void start(void);
    virtual void commit(u16 wMoveTime);
    virtual void release(void);
#if (CONFIG_DOF_PER_LEG == 4)
    virtual void write(u8 LegIndex, s16 sCoxaAngle1, s16 sFemurAngle1, s16 sTibiaAngle1, s16 sTarsAngle1);
#else
    virtual void write(u8 LegIndex, s16 sCoxaAngle1, s16 sFemurAngle1, s16 sTibiaAngle1);
#endif
    virtual u16  getBattVolt(void);

#ifdef CONFIG_TERMINAL
    virtual void showTerminal(void);
    virtual bool handleTerminal(u8 *psz, u8 bLen);
    void move(int servo, int val, unsigned int time);
#endif

};

#endif

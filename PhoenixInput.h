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

#ifndef _PHOENIX_INPUT_H_
#define _PHOENIX_INPUT_H_

#include "config.h"
#include "utils.h"

#define INPUT_TOGGLE_ON_OFF         0x00001
#define INPUT_TOGGLE_SHIFT          0x00002
#define INPUT_TOGGLE_ROTATE         0x00004
#define INPUT_TOGGLE_SINGLE_LEG     0x00008
#define INPUT_TOGGLE_BALANCE        0x00010
#define INPUT_SPEED_DOWN            0x00020
#define INPUT_SPEED_UP              0x00040
#define INPUT_OPT_SEL               0x00080
#define INPUT_OPT_R2                0x00100
#define INPUT_OPT_R1                0x00200
#define INPUT_TOGGLE_WALK           0x00400
#define INPUT_BODY_UP               0x01000
#define INPUT_BODY_DOWN             0x02000
#define INPUT_TOGGLE_BODY_HEIGHT    0x04000
#define INPUT_LEFT_ANALOG           0x10000
#define INPUT_RIGHT_ANALOG          0x20000
#define INPUT_BUTTON_MASK           0x0ffff

#define INPUT_HEIGHT_SUPPORTED      0x80
#define INPUT_HEIGHT_MASK           0x7f

class PhoenixInput
{
private:

protected:
    u8  (*mCallback)(u8 cmd, u8 *data, u8 size, u8 *res);

public:
    virtual void init(u8 (*callback)(u8 cmd, u8 *data, u8 size, u8 *res));
    virtual u32  get(u8 *lx, u8 *ly, u8 *rx, u8 *ry);
    virtual u8   getBodyHeight(void);
};

#endif

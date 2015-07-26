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

#ifndef _PHOENIX_INPUT_MSP_H_
#define _PHOENIX_INPUT_MSP_H_

#include "config.h"
#include "utils.h"
#include "PhoenixInput.h"
#include "SerialProtocol.h"

class PhoenixInputMSP : public PhoenixInput
{
private:
    // stick
    u8        mLX;
    u8        mLY;
    u8        mRX;
    u8        mRY;
    SerialProtocol  mMSP;
public:

    PhoenixInputMSP(void);

    virtual void init(void);
    virtual u32  get(u8 *lx, u8 *ly, u8 *rx, u8 *ry);
};

#endif

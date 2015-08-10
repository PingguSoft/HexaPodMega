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

#ifndef _PHOENIX_INPUT_MSP_H_
#define _PHOENIX_INPUT_MSP_H_

#include "config.h"
#include "utils.h"
#include "PhoenixInput.h"

#define MAX_PACKET_SIZE 32

class PhoenixInputBTCon : public PhoenixInput
{
private:
    // stick
    u8        mLX;
    u8        mLY;
    u8        mRX;
    u8        mRY;
    u16       mButtons;
    u16       mOldButtons;

    typedef enum
    {
        STATE_IDLE,
        STATE_HEADER_START,
        STATE_HEADER_M,
        STATE_HEADER_ARROW,
        STATE_HEADER_SIZE,
        STATE_HEADER_CMD
    } STATE_T;
    //

    // variables
    u8   mRxPacket[MAX_PACKET_SIZE];

    u8   mState;
    u8   mOffset;
    u8   mDataSize;
    u8   mCheckSum;
    u8   mCmd;

    u8   handleRX(void);
    void sendResponse(bool ok, u8 cmd, u8 *data, u8 size);
    void evalCommand(u8 cmd, u8 *data, u8 size);

public:
    typedef enum {
        MSP_SET_USER_BUTTON = 51,
        MSP_IDENT  = 100,
        MSP_STATUS = 101,
        MSP_ANALOG = 110,
        MSP_MISC   = 114,
        MSP_SET_RAW_RC = 200,
    } MSP_T;

    PhoenixInputBTCon(void);

    virtual void init(u8 (*callback)(u8 cmd, u8 *data, u8 size, u8 *res));
    virtual u32  get(u8 *lx, u8 *ly, u8 *rx, u8 *ry);
    virtual u8   getBodyHeight(void);
};

#endif

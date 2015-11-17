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

#ifndef _PHOENIX_CORE_H_
#define _PHOENIX_CORE_H_

#include "config.h"
#include "utils.h"
#include "PhoenixServo.h"

#define	DEC_EXP_1		10
#define	DEC_EXP_2		100
#define	DEC_EXP_4		10000
#define	DEC_EXP_6		1000000
#define SMOOTH_DIV      4  //"Smooth division" factor for the smooth control function, a value of 3 to 5 is most suitable

#define NUM_GAITS       6

typedef struct {
    s32 x;
    s32 y;
    s32 z;
} COORD3D;


typedef struct {
    bool        fHexOn;             // Switch to turn on Phoenix
    bool        fHexOnOld;

    //Body position
    COORD3D     c3dBodyPos;
    COORD3D     c3dBodyRotOff;      // Body rotation offset;

    //Body Inverse Kinematics
    COORD3D     c3dBodyRot;         // X -Pitch, Y-Rotation, Z-Roll

    //[gait]
    u8          bGaitType;          // doGait type
    s16	        sLegLiftHeight;     // Current Travel height
    COORD3D     c3dTravelLen;       // X-Z or Length, Y is rotation.

    //[Single Leg Control]
    u8	    	bSingleLegCurSel;
    u8          bSingleLegOldSel;
    COORD3D     c3dSingleLeg;
    bool		fSingleLegHold;     // Single leg control mode

    //[Balance]
    bool        fBalanceMode;

    //[TIMING]
    u8	    	bInputTimeDelay;    // Delay that depends on the input to get the "sneaking" effect
    u16	        wSpeedControl;      // Adjustible Delay
    u8          bForcemGaitStepCnt;  // new to allow us to force a step even when not moving
} CTRL_STATE;

class PhoenixCore
{
private:
    static bool mBoolShowDbgPrompt;
    static bool mBoolDbgOutput;
    static bool mBoolUpsideDown;

    // ANGLES
    s16         mCoxaAngles[CONFIG_NUM_LEGS];    //Actual Angle of the horizontal hip, decimals = 1
    s16         mFemurAngles[CONFIG_NUM_LEGS];   //Actual Angle of the vertical hip, decimals = 1
    s16         mTibiaAngles[CONFIG_NUM_LEGS];   //Actual Angle of the knee, decimals = 1
#if (CONFIG_DOF_PER_LEG == 4)
    s16         mTarsAngles[CONFIG_NUM_LEGS];      //Actual Angle of the knee, decimals = 1
#endif

    // POSITIONS SINGLE LEG CONTROL
    s16         mLegPosXs[CONFIG_NUM_LEGS];    //Actual X Posion of the Leg
    s16         mLegPosYs[CONFIG_NUM_LEGS];    //Actual Y Posion of the Leg
    s16         mLegPosZs[CONFIG_NUM_LEGS];    //Actual Z Posion of the Leg

    u16         mCurServoMoveTime; // Time for servo updates
    u16         mOldServoMoveTime; // Previous time for the servo updates

    u32         mCommitTime;

    bool        mBoolWalking;            //  True if the robot are walking

    // [Balance]
    s32         mTotalTransX;
    s32         mTotalTransZ;
    s32         mTotalTransY;
    s32         mTotalYBal1;
    s32         mTotalXBal1;
    s32         mTotalZBal1;

    //[gait]
    s16		    mNormGaitSpeed;        //Nominal speed of the gait
    s16         mTLDivFactor;         //Number of steps that a leg is on the floor while walking
    s16         mNrLiftedPos;         //Number of positions that a single leg is lifted [1-3]
    u8          mLiftDivFactor;       //Normaly: 2, when mNrLiftedPos=5: 4
    u8          mFrontDownPos;        //Where the leg should be put down to ground
    u8          mHalfLiftHeight;      //If TRUE the outer positions of the ligted legs will be half height
    u8          mStepsInGait;         //Number of steps in gait
    u8          mGaitStep;            //Actual doGait step
    u8          mGaitLegInits[CONFIG_NUM_LEGS];        //init position of the leg

    s32         mGaitPosXs[CONFIG_NUM_LEGS];         //Array containing Relative X position corresponding to the Gait
    s32         mGaitPosYs[CONFIG_NUM_LEGS];         //Array containing Relative Y position corresponding to the Gait
    s32         mGaitPosZs[CONFIG_NUM_LEGS];         //Array containing Relative Z position corresponding to the Gait
    s32         mGaitRotYs[CONFIG_NUM_LEGS];         //Array containing Relative Y rotation corresponding to the Gait

    u8          mExtraCycle;          // Forcing some extra timed cycles for avoiding "end of gait bug"

    //[TIMING]
    u32         mTimerStart;    //Start time of the calculation cycles
    u32         mTimerLastCheck;

    PhoenixServo  *mServo;
    CTRL_STATE    *mPtrCtrlState;

    u8          mCurVolt;
    u8          mVoltWarnBeepCnt;

    void        updateServos(void);
    void        updateLEDs(void);
    bool        ctrlSingleLeg(void);
    void        doGaitSeq(void);
    void        doGait(u8 leg, bool fTravelReq);
    void        calcBalOneLeg (u8 leg, long posX, long posZ, long posY);
    void        balanceBody(void);
    void        getBodyIK(u8 leg, s16 posX, s16 posZ, s16 posY, s16 RotationY, long *x, long *y, long *z);
    u8          getLegIK(u8 leg, s16 IKFeetPosX, s16 IKFeetPosY, s16 IKFeetPosZ);
    void        validateAngles(void);
    s16         smoothControl (s16 CtrlMoveInp, s16 CtrlMoveOut, u8 CtrlDivider);
    bool        showTerminal(void);
    void        handleEEPROM(u8 *pszCmdLine);


public:
    enum {
        IDX_RR = 0,
        IDX_RM,
        IDX_RF,
        IDX_LR,
        IDX_LM,
        IDX_LF
    };

    enum {
        STATUS_OK,
        STATUS_WARNING,
        STATUS_ERROR,
        STATUS_BATT_WARN = 0x10,
        STATUS_BATT_FAIL = 0x20,
    };

    PhoenixCore(PhoenixServo *servo, CTRL_STATE *state);

    void        init(void);
    u8          loop(void);
    void        initCtrl(void);
    void        selectGait(u8 bGaitType);
    void        adjustLegPosToBodyHeight(void);
    u8          getBattLevel(void)     { return mCurVolt; }
    u8          getBattLevel(u8 scale) { return mCurVolt; }
};

#endif

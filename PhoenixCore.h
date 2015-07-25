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

typedef struct {
    s32 x;
    s32 y;
    s32 z;
} COORD3D;


typedef struct {
    bool        fHexOn;          // Switch to turn on Phoenix

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
    u8          bForceGaitStepCnt;  // new to allow us to force a step even when not moving
} CTRL_STATE;

class PhoenixCore
{
private:

    enum {
        LED_RED   = 0,
        LED_GREEN,
        LED_ORANGE,
        LED_EYES
    };

    enum {
        SOLUTION_OK,
        SOLUTION_WARNING,
        SOLUTION_ERROR,
    };

    static bool mBoolShowDbgPrompt;
    static bool mBoolDbgOutput;
    static bool mBoolUpsideDown;

    // ANGLES
    s16         mCoxaAngle[CONFIG_NUM_LEGS];    //Actual Angle of the horizontal hip, decimals = 1
    s16         mFemurAngle[CONFIG_NUM_LEGS];   //Actual Angle of the vertical hip, decimals = 1
    s16         mTibiaAngle[CONFIG_NUM_LEGS];   //Actual Angle of the knee, decimals = 1
#if (CONFIG_DOF_PER_LEG == 4)
    s16         mTarsAngle[CONFIG_NUM_LEGS];      //Actual Angle of the knee, decimals = 1
#endif

    // POSITIONS SINGLE LEG CONTROL
    s16         mLegPosX[CONFIG_NUM_LEGS];    //Actual X Posion of the Leg
    s16         mLegPosY[CONFIG_NUM_LEGS];    //Actual Y Posion of the Leg
    s16         mLegPosZ[CONFIG_NUM_LEGS];    //Actual Z Posion of the Leg

    // OUTPUTS
    u8          mLedOutput;

    CTRL_STATE  mControlState;

    u16         mCurServoMoveTime; // Time for servo updates
    u16         mOldServoMoveTime; // Previous time for the servo updates

    bool        mBoolWalking;            //  True if the robot are walking

    // [Balance]
    s32         mTotalTransX;
    s32         mTotalTransZ;
    s32         mTotalTransY;
    s32         mTotalYBal1;
    s32         mTotalXBal1;
    s32         mTotalZBal1;

    //[gait]
    s16		    NomGaitSpeed;        //Nominal speed of the gait
    s16         TLDivFactor;         //Number of steps that a leg is on the floor while walking
    s16         NrLiftedPos;         //Number of positions that a single leg is lifted [1-3]
    u8          LiftDivFactor;       //Normaly: 2, when NrLiftedPos=5: 4
    u8          FrontDownPos;        //Where the leg should be put down to ground
    bool        HalfLiftHeigth;      //If TRUE the outer positions of the ligted legs will be half height
    u8          StepsInGait;         //Number of steps in gait
    u8          GaitStep;            //Actual doGait step
    u8          GaitLegNr[CONFIG_NUM_LEGS];        //init position of the leg

    s32         lGaitPosX[CONFIG_NUM_LEGS];         //Array containing Relative X position corresponding to the Gait
    s32         lGaitPosY[CONFIG_NUM_LEGS];         //Array containing Relative Y position corresponding to the Gait
    s32         lGaitPosZ[CONFIG_NUM_LEGS];         //Array containing Relative Z position corresponding to the Gait
    s32         lGaitRotY[CONFIG_NUM_LEGS];         //Array containing Relative Y rotation corresponding to the Gait

    u8          bExtraCycle;          // Forcing some extra timed cycles for avoiding "end of gait bug"

    //[TIMING]
    u32         lTimerStart;    //Start time of the calculation cycles
    u32         lTimerEnd;        //End time of the calculation cycles
    u8          CycleTime;        //Total Cycle time

    PhoenixServo  *mServo;

    void        updateServos(void);
    void        updateLEDs(void);
    bool        checkVoltage(void);
    bool        ctrlSingleLeg(void);
    void        doGaitSeq(void);
    void        doGait(u8 leg, bool fTravelReq);
    void        calcBalOneLeg (u8 leg, long posX, long posZ, long posY);
    void        balanceBody(void);
    void        getBodyIK (s16 posX, s16 posZ, s16 posY, s16 RotationY, u8 leg, long *x, long *y, long *z);
    u8          getLegIK (s16 IKFeetPosX, s16 IKFeetPosY, s16 IKFeetPosZ, u8 leg);
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

    PhoenixCore(void);

    void        init(void);
    void        loop(void);
    CTRL_STATE  getCtrl(void) { return mControlState; }
    void        initCtrl(void);
    void        selectGait(void);
    void        adjustLegPosToBodyHeight(void);
};

#endif

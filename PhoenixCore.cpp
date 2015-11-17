//=============================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix software
//Software version: V2.0
//Date: 29-10-2009
//Programmer: Jeroen Janssen [aka Xan]
//         Kurt Eckhardt(KurtE) converted to C and Arduino
//   KÃ¥re Halvorsen aka Zenta - Makes everything work correctly!
//
// This version of the Phoenix code was ported over to the Arduino Environement
// and is specifically configured for the Lynxmotion BotBoarduino
//
// Date : 12-07-2015
// Programmer : PingguSoft (pinggusoft@gmail.com)
//              Code rework for smartphone control and readability
//              Android App : BTCon4Drone
//               https://play.google.com/store/apps/details?id=com.pinggusoft.btcon
//=============================================================================

#if ARDUINO>99
#include <Arduino.h>
#endif

#include <EEPROM.h>
#include <pins_arduino.h>
#include <avr/pgmspace.h>
#include "PhoenixCore.h"
#if (CONFIG_SERVO == CONFIG_SERVO_SW_PWM)
#include "PhoenixServoSW.h"
#else
#include "PhoenixServoUSC.h"
#endif

#define BALANCE_DIV_FACTOR  6    //;Other values than 6 can be used, testing...CAUTION!! At your own risk ;)

//--------------------------------------------------------------------
//[TABLES]
//ArcCosinus Table
//Table build in to 3 part to get higher accuracy near cos = 1.
//The biggest error is near cos = 1 and has a biggest value of 3*0.012098rad = 0.521 deg.
//-    Cos 0 to 0.9 is done by steps of 0.0079 rad. [1/127]
//-    Cos 0.9 to 0.99 is done by steps of 0.0008 rad [0.1/127]
//-    Cos 0.99 to 1 is done by step of 0.0002 rad [0.01/64]
//Since the tables are overlapping the full range of 127+127+64 is not necessary. Total u8s: 277

static const u8 TBL_ACOS[] PROGMEM = {
    255,254,252,251,250,249,247,246,245,243,242,241,240,238,237,236,234,233,232,231,229,228,227,225,
    224,223,221,220,219,217,216,215,214,212,211,210,208,207,206,204,203,201,200,199,197,196,195,193,
    192,190,189,188,186,185,183,182,181,179,178,176,175,173,172,170,169,167,166,164,163,161,160,158,
    157,155,154,152,150,149,147,146,144,142,141,139,137,135,134,132,130,128,127,125,123,121,119,117,
    115,113,111,109,107,105,103,101,98,96,94,92,89,87,84,81,79,76,73,73,73,72,72,72,71,71,71,70,70,
    70,70,69,69,69,68,68,68,67,67,67,66,66,66,65,65,65,64,64,64,63,63,63,62,62,62,61,61,61,60,60,59,
    59,59,58,58,58,57,57,57,56,56,55,55,55,54,54,53,53,53,52,52,51,51,51,50,50,49,49,48,48,47,47,47,
    46,46,45,45,44,44,43,43,42,42,41,41,40,40,39,39,38,37,37,36,36,35,34,34,33,33,32,31,31,30,29,28,
    28,27,26,25,24,23,23,23,23,22,22,22,22,21,21,21,21,20,20,20,19,19,19,19,18,18,18,17,17,17,17,16,
    16,16,15,15,15,14,14,13,13,13,12,12,11,11,10,10,9,9,8,7,6,6,5,3,0
};

//Sin table 90 deg, persision 0.5 deg [180 values]
static const u16 TBL_SIN[] PROGMEM = {
    0, 87, 174, 261, 348, 436, 523, 610, 697, 784, 871, 958, 1045, 1132, 1218, 1305, 1391, 1478, 1564,
    1650, 1736, 1822, 1908, 1993, 2079, 2164, 2249, 2334, 2419, 2503, 2588, 2672, 2756, 2840, 2923, 3007,
    3090, 3173, 3255, 3338, 3420, 3502, 3583, 3665, 3746, 3826, 3907, 3987, 4067, 4146, 4226, 4305, 4383,
    4461, 4539, 4617, 4694, 4771, 4848, 4924, 4999, 5075, 5150, 5224, 5299, 5372, 5446, 5519, 5591, 5664,
    5735, 5807, 5877, 5948, 6018, 6087, 6156, 6225, 6293, 6360, 6427, 6494, 6560, 6626, 6691, 6755, 6819,
    6883, 6946, 7009, 7071, 7132, 7193, 7253, 7313, 7372, 7431, 7489, 7547, 7604, 7660, 7716, 7771, 7826,
    7880, 7933, 7986, 8038, 8090, 8141, 8191, 8241, 8290, 8338, 8386, 8433, 8480, 8526, 8571, 8616, 8660,
    8703, 8746, 8788, 8829, 8870, 8910, 8949, 8987, 9025, 9063, 9099, 9135, 9170, 9205, 9238, 9271, 9304,
    9335, 9366, 9396, 9426, 9455, 9483, 9510, 9537, 9563, 9588, 9612, 9636, 9659, 9681, 9702, 9723, 9743,
    9762, 9781, 9799, 9816, 9832, 9848, 9862, 9876, 9890, 9902, 9914, 9925, 9935, 9945, 9953, 9961, 9969,
    9975, 9981, 9986, 9990, 9993, 9996, 9998, 9999, 10000
};


//Build tables for Leg configuration like I/O and MIN/imax values to easy access values using a FOR loop
//Constants are still defined as single values in the cfg file to make it easy to read/configure

// Servo Horn offsets
#ifdef OFFSET_RR_FEMUR_HORN   // per leg configuration
    static const s16 TBL_OFFSET_FEMUR_HORN[] PROGMEM = {
        OFFSET_RR_FEMUR_HORN,
        OFFSET_RM_FEMUR_HORN,
        OFFSET_RF_FEMUR_HORN,
        OFFSET_LR_FEMUR_HORN,
        OFFSET_LM_FEMUR_HORN,
        OFFSET_LF_FEMUR_HORN
    };
    #define OFFSET_FEMUR_HORN(LEGI) ((s16)pgm_read_word(&TBL_OFFSET_FEMUR_HORN[LEGI]))
#else   // Fixed per leg, if not defined 0
    #define OFFSET_FEMUR_HORN(LEGI)  (0)
#endif

// Servo TARS offsets
#if (CONFIG_DOF_PER_LEG == 4)
#ifdef OFFSET_RR_TARS_HORN   // per leg configuration
    static const s16 TBL_OFFSET_TARS_HORN[] PROGMEM = {
        OFFSET_RR_TARS_HORN,
        OFFSET_RM_TARS_HORN,
        OFFSET_RF_TARS_HORN,
        OFFSET_LR_TARS_HORN,
        OFFSET_LM_TARS_HORN,
        OFFSET_LF_TARS_HORN
    };
    #define OFFSET_TARS_HORN(LEGI) ((s16)pgm_read_word(&TBL_OFFSET_TARS_HORN[LEGI]))
#else
    #define OFFSET_TARS_HORN(LEGI)  (0)
#endif
#endif

// min max tables
static const s16 TBL_COXA_MIN[] PROGMEM = {
    cRRCoxaMin1,  cRMCoxaMin1,  cRFCoxaMin1,  cLRCoxaMin1,  cLMCoxaMin1,  cLFCoxaMin1};
static const s16 TBL_COXA_MAX[] PROGMEM = {
    cRRCoxaMax1,  cRMCoxaMax1,  cRFCoxaMax1,  cLRCoxaMax1,  cLMCoxaMax1,  cLFCoxaMax1};
static const s16 TBL_FEMUR_MIN[] PROGMEM ={
    cRRFemurMin1, cRMFemurMin1, cRFFemurMin1, cLRFemurMin1, cLMFemurMin1, cLFFemurMin1};
static const s16 TBL_FEMUR_MAX[] PROGMEM ={
    cRRFemurMax1, cRMFemurMax1, cRFFemurMax1, cLRFemurMax1, cLMFemurMax1, cLFFemurMax1};
static const s16 TBL_TIBIA_MIN[] PROGMEM ={
    cRRTibiaMin1, cRMTibiaMin1, cRFTibiaMin1, cLRTibiaMin1, cLMTibiaMin1, cLFTibiaMin1};
static const s16 TBL_TIBIA_MAX[] PROGMEM = {
    cRRTibiaMax1, cRMTibiaMax1, cRFTibiaMax1, cLRTibiaMax1, cLMTibiaMax1, cLFTibiaMax1};

#if (CONFIG_DOF_PER_LEG == 4)
static const s16 TBL_TARS_MIN[] PROGMEM = {
    cRRTarsMin1, cRMTarsMin1, cRFTarsMin1, cLRTarsMin1, cLMTarsMin1, cLFTarsMin1};
static const s16 TBL_TARS_MAX[] PROGMEM = {
    cRRTarsMax1, cRMTarsMax1, cRFTarsMax1, cLRTarsMax1, cLMTarsMax1, cLFTarsMax1};
#endif


// leg length tables
static const u8 TBL_COXA_LENGTH[] PROGMEM = {
    cRRCoxaLength,  cRMCoxaLength,  cRFCoxaLength,  cLRCoxaLength,  cLMCoxaLength,  cLFCoxaLength};
static const u8 TBL_FEMUR_LENGTH[] PROGMEM = {
    cRRFemurLength, cRMFemurLength, cRFFemurLength, cLRFemurLength, cLMFemurLength, cLFFemurLength};
static const u8 TBL_TIBIA_LENGTH[] PROGMEM = {
    cRRTibiaLength, cRMTibiaLength, cRFTibiaLength, cLRTibiaLength, cLMTibiaLength, cLFTibiaLength};
#if (CONFIG_DOF_PER_LEG == 4)
static const u8 TBL_TARS_LENGTH[] PROGMEM = {
    cRRTarsLength, cRMTarsLength, cRFTarsLength, cLRTarsLength, cLMTarsLength, cLFTarsLength};
#endif


// Body Offsets [distance between the center of the body and the center of the coxa]
static const s16 TBL_OFFSET_X[] PROGMEM = {
    cRROffsetX, cRMOffsetX, cRFOffsetX, cLROffsetX, cLMOffsetX, cLFOffsetX};
static const s16 TBL_OFFSET_Z[] PROGMEM = {
    cRROffsetZ, cRMOffsetZ, cRFOffsetZ, cLROffsetZ, cLMOffsetZ, cLFOffsetZ};

// Default leg angle
static const s16 TBL_COXA_ANGLE[] PROGMEM = {
    cRRCoxaAngle1, cRMCoxaAngle1, cRFCoxaAngle1, cLRCoxaAngle1, cLMCoxaAngle1, cLFCoxaAngle1};

// Start positions for the leg
static const s16 TBL_INT_POS_X[] PROGMEM = {
    cRRInitPosX, cRMInitPosX, cRFInitPosX, cLRInitPosX, cLMInitPosX, cLFInitPosX};
static const s16 TBL_INT_POS_Y[] PROGMEM = {
    cRRInitPosY, cRMInitPosY, cRFInitPosY, cLRInitPosY, cLMInitPosY, cLFInitPosY};
static const s16 TBL_INT_POS_Z[] PROGMEM = {
    cRRInitPosZ, cRMInitPosZ, cRFInitPosZ, cLRInitPosZ, cLMInitPosZ, cLFInitPosZ};


// Define some globals for debug information
bool PhoenixCore::mBoolShowDbgPrompt = FALSE;
bool PhoenixCore::mBoolDbgOutput     = FALSE;
bool PhoenixCore::mBoolUpsideDown    = FALSE;

//--------------------------------------------------------------------
//[REMOTE]

#define GP_DIFF_LIMIT       2       // GP=GaitPos testing different limits

// Define our ServoWriter class
//ServoDriver  g_ServoDriver;      // our global servo driver class

//--------------------------------------------------------------------
//[GETSINCOS] Get the sinus and cosinus from the angle +/- multiple circles
//angleDeg1     - Input Angle in degrees
//sin4        - Output Sinus of AngleDeg
//cos4          - Output Cosinus of AngleDeg
void sincos(s16 angleDeg1, s16 *sin4, s16 *cos4)
{
    s16        sAbsAngleDeg1;    //Absolute value of the Angle in Degrees, decimals = 1

    //Get the absolute value of AngleDeg
    if (angleDeg1 < 0)
        sAbsAngleDeg1 = -angleDeg1;
    else
        sAbsAngleDeg1 = angleDeg1;

    //Shift rotation to a full circle of 360 deg -> AngleDeg // 360
    if (angleDeg1 < 0)    //Negative values
        angleDeg1 = 3600 - (sAbsAngleDeg1 - (3600 * (sAbsAngleDeg1 / 3600)));
    else                //Positive values
        angleDeg1 = sAbsAngleDeg1 - (3600 * (sAbsAngleDeg1 / 3600));

    if (angleDeg1 >= 0 && angleDeg1 <= 900) {
        *sin4 = pgm_read_word(&TBL_SIN[angleDeg1 / 5]);                 // 5 is the presision (0.5) of the table
        *cos4 = pgm_read_word(&TBL_SIN[(900 - (angleDeg1)) / 5]);
    } else if (angleDeg1 > 900 && angleDeg1 <= 1800) {
        *sin4 = pgm_read_word(&TBL_SIN[(900 - (angleDeg1 - 900)) / 5]); // 5 is the presision (0.5) of the table
        *cos4 = -pgm_read_word(&TBL_SIN[(angleDeg1 - 900) / 5]);
    } else if (angleDeg1 > 1800 && angleDeg1 <= 2700) {
        *sin4 = -pgm_read_word(&TBL_SIN[(angleDeg1 - 1800) / 5]);       // 5 is the presision (0.5) of the table
        *cos4 = -pgm_read_word(&TBL_SIN[(2700 - angleDeg1) / 5]);
    } else if(angleDeg1 > 2700 && angleDeg1 <= 3600) {
        *sin4 = -pgm_read_word(&TBL_SIN[(3600 - angleDeg1) / 5]);       // 5 is the presision (0.5) of the table
        *cos4 = pgm_read_word(&TBL_SIN[(angleDeg1 - 2700) / 5]);
    }
}


//--------------------------------------------------------------------
//(GETARCCOS) Get the sinus and cosinus from the angle +/- multiple circles
//cos4        - Input Cosinus
//angleRad4     - Output Angle in angleRad4
long arccos(s16 cos4)
{
    bool fNegVal;
    s16  angleRad4 = 0;

    //Check for negative value
    if (cos4 < 0) {
        cos4 = -cos4;
        fNegVal = TRUE;
    } else
        fNegVal = FALSE;

    //Limit cos4 to his maximal value
    cos4 = min(cos4,DEC_EXP_4);

    if ((cos4 >= 0) && (cos4 < 9000)) {
        angleRad4 = (u8)pgm_read_byte(&TBL_ACOS[cos4 / 79]);
        angleRad4 = ((long)angleRad4 * 616) / DEC_EXP_1;            //616=arccos resolution (pi/2/255) ;
    } else if ((cos4 >= 9000) && (cos4 < 9900)) {
        angleRad4 = (u8)pgm_read_byte(&TBL_ACOS[(cos4 - 9000) / 8 + 114]);
        angleRad4 = (long)((long)angleRad4 * 616) / DEC_EXP_1;      //616=arccos resolution (pi/2/255)
    } else if ((cos4 >= 9900) && (cos4<= 10000)) {
        angleRad4 = (u8)pgm_read_byte(&TBL_ACOS[(cos4-9900)/2+227]);
        angleRad4 = (long)((long)angleRad4*616)/DEC_EXP_1;          //616=arccos resolution (pi/2/255)
    }

    //Add negative sign
    if (fNegVal)
        angleRad4 = 31416 - angleRad4;

    return angleRad4;
}

u32 isqrt32 (u32 n)
{
    u32 root;
    u32 remainder;
    u32  place;

    root = 0;
    remainder = n;
    place = 0x40000000; // OR place = 0x4000; OR place = 0x40; - respectively

    while (place > remainder)
        place = place >> 2;

    while (place) {
        if (remainder >= root + place) {
            remainder = remainder - root - place;
            root = root + (place << 1);
        }
        root  = root  >> 1;
        place = place >> 2;
    }
    return root;
}

//--------------------------------------------------------------------
//(GETATAN2) Simplyfied ArcTan2 function based on fixed point ArcCos
//ArcTanX         - Input X
//ArcTanY         - Input Y
//ArcTan4          - Output ARCTAN2(X/Y)
//XYhyp2            - Output presenting Hypotenuse of X and Y
s16 arctan2(s16 atanX, s16 atanY, long *hyp2XY)
{
    s16   angleRad4;
    s16   atan4;
    long  temp;

    temp = isqrt32(((long)atanX*atanX*DEC_EXP_4) + ((long)atanY*atanY*DEC_EXP_4));
    angleRad4 = arccos (((long)atanX*(long)DEC_EXP_6) /(long) temp);

    if (atanY < 0)                // removed overhead... Atan4 = angleRad4 * (atanY/abs(atanY));
        atan4 = -angleRad4;
    else
        atan4 = angleRad4;

    if (hyp2XY)
        *hyp2XY = temp;

    return atan4;
}

PhoenixCore::PhoenixCore(PhoenixServo *servo, CTRL_STATE *state)
{
    mServo = servo;
    mCommitTime = 0;
    mOldServoMoveTime = 0;
    mPtrCtrlState     = state;
}

void PhoenixCore::initCtrl(void)
{
    bool temp = mPtrCtrlState->fHexOnOld;

    memset(mPtrCtrlState, 0, sizeof(CTRL_STATE));
    mPtrCtrlState->fHexOnOld = temp;

    //Single leg control. Make sure no leg is selected
    mPtrCtrlState->bSingleLegCurSel = 255; // No Leg selected
    mPtrCtrlState->bSingleLegOldSel = 255;
}

void PhoenixCore::init(void)
{
    printf(F("%s\n"), __PRETTY_FUNCTION__);

    mServo->init();
    mCurVolt = mServo->getBattVolt();
    mTimerLastCheck = millis();

    //Tars init Positions
    for (u8 i = 0; i < CONFIG_NUM_LEGS; i++ ) {
        mLegPosXs[i] = (s16)pgm_read_word(&TBL_INT_POS_X[i]);    //Set start positions for each leg
        mLegPosYs[i] = (s16)pgm_read_word(&TBL_INT_POS_Y[i]);
        mLegPosZs[i] = (s16)pgm_read_word(&TBL_INT_POS_Z[i]);
    }

    initCtrl();

    //doGait
    mPtrCtrlState->bGaitType = 1;  // 0; Devon wanted
    mPtrCtrlState->sLegLiftHeight = 50;
    mGaitStep = 1;
    selectGait(mPtrCtrlState->bGaitType);

    // Servo Driver
    mCurServoMoveTime = 150;
    printf(F("up_side_down:%d\n"), mBoolUpsideDown);
}

u8 PhoenixCore::loop(void)
{
    bool            allDown;
    long            lBodyX;         //Output Position X of feet with Rotation
    long            lBodyY;         //Output Position Y of feet with Rotation
    long            lBodyZ;         //Output Position Z of feet with Rotation
    u8              ret = STATUS_OK;

    //Start time
    mTimerStart = millis();

    if (mCommitTime != 0) {
        if (mTimerStart >= mCommitTime) {
            mServo->commit(mCurServoMoveTime);
            mCommitTime = 0;
            mTimerStart = millis();
        } else {
            return ret;
        }
    }

    // every 500ms
    if (mTimerStart - mTimerLastCheck > 500) {
        mCurVolt = mServo->getBattVolt();
        mTimerLastCheck = mTimerStart;

        if (mCurVolt < CONFIG_VOLT_OFF) {
            mVoltWarnBeepCnt++;
            if (mVoltWarnBeepCnt > 10) {

                return STATUS_BATT_FAIL;
            }
            else
                ret |= STATUS_BATT_WARN;
        } else {
            mVoltWarnBeepCnt = 0;
        }
    }

    if (mBoolUpsideDown) {
        mPtrCtrlState->c3dTravelLen.x = -mPtrCtrlState->c3dTravelLen.x;
        mPtrCtrlState->c3dBodyPos.x = -mPtrCtrlState->c3dBodyPos.x;
        mPtrCtrlState->c3dSingleLeg.x = -mPtrCtrlState->c3dSingleLeg.x;
        mPtrCtrlState->c3dBodyRot.z = -mPtrCtrlState->c3dBodyRot.z;
    }

    //Single leg control
    allDown = ctrlSingleLeg();

    //doGait
    doGaitSeq();

    //Balance calculations
    mTotalTransX = 0;     //reset values used for calculation of balance
    mTotalTransZ = 0;
    mTotalTransY = 0;
    mTotalXBal1  = 0;
    mTotalYBal1  = 0;
    mTotalZBal1  = 0;

    if (mPtrCtrlState->fBalanceMode) {
        for (u8 i = 0; i < CONFIG_NUM_LEGS / 2; i++) {    // balance calculations for all Right legs
            calcBalOneLeg(i, -mLegPosXs[i]+mGaitPosXs[i],
                          mLegPosZs[i]+mGaitPosZs[i],
                          (mLegPosYs[i]-(s16)pgm_read_word(&TBL_INT_POS_Y[i]))+mGaitPosYs[i]);
        }

        for (u8 i = CONFIG_NUM_LEGS / 2; i < CONFIG_NUM_LEGS; i++) {    // balance calculations for all Right legs
            calcBalOneLeg(i, mLegPosXs[i]+mGaitPosXs[i],
                          mLegPosZs[i]+mGaitPosZs[i],
                          (mLegPosYs[i]-(s16)pgm_read_word(&TBL_INT_POS_Y[i]))+mGaitPosYs[i]);
        }
        balanceBody();
    }

    //Do IK for all Right legs
    for (u8 i = 0; i < CONFIG_NUM_LEGS / 2; i++) {
        getBodyIK(i,
                  -mLegPosXs[i]+mPtrCtrlState->c3dBodyPos.x+mGaitPosXs[i] - mTotalTransX,
                  mLegPosZs[i]+mPtrCtrlState->c3dBodyPos.z+mGaitPosZs[i] - mTotalTransZ,
                  mLegPosYs[i]+mPtrCtrlState->c3dBodyPos.y+mGaitPosYs[i] - mTotalTransY,
                  mGaitRotYs[i],
                  &lBodyX, &lBodyY, &lBodyZ);

        ret |= getLegIK(i, mLegPosXs[i]-mPtrCtrlState->c3dBodyPos.x+lBodyX-(mGaitPosXs[i] - mTotalTransX),
                        mLegPosYs[i]+mPtrCtrlState->c3dBodyPos.y-lBodyY+mGaitPosYs[i] - mTotalTransY,
                        mLegPosZs[i]+mPtrCtrlState->c3dBodyPos.z-lBodyZ+mGaitPosZs[i] - mTotalTransZ);
    }

    //Do IK for all Left legs
    for (u8 i = CONFIG_NUM_LEGS / 2; i < CONFIG_NUM_LEGS; i++) {
        getBodyIK(i,
                  mLegPosXs[i]-mPtrCtrlState->c3dBodyPos.x+mGaitPosXs[i] - mTotalTransX,
                  mLegPosZs[i]+mPtrCtrlState->c3dBodyPos.z+mGaitPosZs[i] - mTotalTransZ,
                  mLegPosYs[i]+mPtrCtrlState->c3dBodyPos.y+mGaitPosYs[i] - mTotalTransY,
                  mGaitRotYs[i],
                  &lBodyX, &lBodyY, &lBodyZ);

        ret |= getLegIK(i, mLegPosXs[i]+mPtrCtrlState->c3dBodyPos.x-lBodyX+mGaitPosXs[i] - mTotalTransX,
                       mLegPosYs[i]+mPtrCtrlState->c3dBodyPos.y-lBodyY+mGaitPosYs[i] - mTotalTransY,
                       mLegPosZs[i]+mPtrCtrlState->c3dBodyPos.z-lBodyZ+mGaitPosZs[i] - mTotalTransZ);
    }

    if (mBoolUpsideDown) { //Need to set them back for not messing with the smoothControl
        mPtrCtrlState->c3dBodyPos.x = -mPtrCtrlState->c3dBodyPos.x;
        mPtrCtrlState->c3dSingleLeg.x = -mPtrCtrlState->c3dSingleLeg.x;
        mPtrCtrlState->c3dBodyRot.z = -mPtrCtrlState->c3dBodyRot.z;
    }

    //Check mechanical limits
    validateAngles();

    //Drive Servos
    if (mPtrCtrlState->fHexOn) {
        //Calculate Servo Move time
        if ((abs(mPtrCtrlState->c3dTravelLen.x) > CONFIG_TRAVEL_DEAD_ZONE) ||
            (abs(mPtrCtrlState->c3dTravelLen.z) > CONFIG_TRAVEL_DEAD_ZONE) ||
            (abs(mPtrCtrlState->c3dTravelLen.y * 2) > CONFIG_TRAVEL_DEAD_ZONE)) {
            mCurServoMoveTime = mNormGaitSpeed + (mPtrCtrlState->bInputTimeDelay * 2) + mPtrCtrlState->wSpeedControl;
            //Add aditional delay when Balance mode is on
            if (mPtrCtrlState->fBalanceMode)
                mCurServoMoveTime = mCurServoMoveTime + 100;
        } else { //Movement speed excl. Walking
            mCurServoMoveTime = 200 + mPtrCtrlState->wSpeedControl;
        }

        // note we broke up the servo driver into start/commit that way we can output all of the servo information
        // before we wait and only have the termination information to output after the wait.  That way we hopefully
        // be more accurate with our timings...
        updateServos();

        // See if we need to sync our processor with the servo driver while walking to ensure the prev is completed
        // before sending the next one
        // Finding any incident of GaitPos/Rot <>0:
        for (u8 i = 0; i < CONFIG_NUM_LEGS; i++) {
            if ( (mGaitPosXs[i] > GP_DIFF_LIMIT) || (mGaitPosXs[i] < -GP_DIFF_LIMIT) ||
                 (mGaitPosZs[i] > GP_DIFF_LIMIT) || (mGaitPosZs[i] < -GP_DIFF_LIMIT) ||
                 (mGaitRotYs[i] > GP_DIFF_LIMIT) || (mGaitRotYs[i] < -GP_DIFF_LIMIT)) {
                mExtraCycle = mNrLiftedPos + 1;//For making sure that we are using timed move until all legs are down
                break;
            }
        }

        //printf(F("ExtraCycle:%d\n"), mExtraCycle);
        if (mExtraCycle > 0) {
            mExtraCycle--;
            mBoolWalking = (mExtraCycle != 0);
            mCommitTime  = mTimerStart + mOldServoMoveTime;
            //printf(F("Next1:%ld\n"), mCommitTime);

            if (mBoolDbgOutput) {
                printf(F("BRX:%d, Walk:%d, GS:%d\n"), mPtrCtrlState->c3dBodyRot.x, mBoolWalking, mGaitStep);
                printf(F("LEFT  GPX:%5d, GPY:%5d, GPZ:%5d\n"), mGaitPosXs[IDX_LF], mGaitPosYs[IDX_LF], mGaitPosZs[IDX_LF]);
                printf(F("RIGHT GPX:%5d, GPY:%5d, GPZ:%5d\n"), mGaitPosXs[IDX_RF], mGaitPosYs[IDX_RF], mGaitPosZs[IDX_RF]);
            }
        } else {
            // commit immediately
            mCommitTime = mTimerStart + mOldServoMoveTime;
            //printf(F("Next2:%ld\n"), mCommitTime);
        }

        if (mBoolDbgOutput) {
            printf(F("TY:%5d, LFZ:%5d\n"), mTotalYBal1, mLegPosZs[IDX_LF]);
        }


    } else {
        //Turn the bot off - May need to add ajust here...
        if (mPtrCtrlState->fHexOnOld) {
            printf(F("RESET LEGS !!!\n"));
            mCurServoMoveTime = 600;
            updateServos();
            mServo->commit(mCurServoMoveTime);
            delay(600);
        } else {
            mServo->release();
        }
        // We also have a simple debug monitor that allows us to
        // check things. call it here..
#ifdef CONFIG_TERMINAL
        if (showTerminal())
            return ret;
#endif
    }
    mOldServoMoveTime = mCurServoMoveTime;

    return ret;
}

void PhoenixCore::updateServos(void)
{
    // First call off to the init...
    mServo->start();

    for (u8 i = 0; i < CONFIG_NUM_LEGS; i++) {
#if (CONFIG_DOF_PER_LEG == 4)
        mServo->write(i, mCoxaAngles[i], mFemurAngles[i], mTibiaAngles[i], mTarsAngles[i]);
#else
        mServo->write(i, mCoxaAngles[i], mFemurAngles[i], mTibiaAngles[i]);
#endif
  }
}

bool PhoenixCore::ctrlSingleLeg(void)
{
    bool allDown = TRUE;

    for (u8 i = 0; i < CONFIG_NUM_LEGS; i++) {
        if (mLegPosYs[i] != (s16)pgm_read_word(&TBL_INT_POS_Y[i])) {
            allDown = FALSE;
            break;
        }
    }

    if (mPtrCtrlState->bSingleLegCurSel < CONFIG_NUM_LEGS) {
        if (mPtrCtrlState->bSingleLegCurSel != mPtrCtrlState->bSingleLegOldSel) {
            if (allDown) { //Lift leg a bit when it got selected
                mLegPosYs[mPtrCtrlState->bSingleLegCurSel] = (s16)pgm_read_word(&TBL_INT_POS_Y[mPtrCtrlState->bSingleLegCurSel]) - 20;
                //Store current status
                mPtrCtrlState->bSingleLegOldSel = mPtrCtrlState->bSingleLegCurSel;
            } else {//Return prev leg back to the init position
                mLegPosXs[mPtrCtrlState->bSingleLegOldSel] = (s16)pgm_read_word(&TBL_INT_POS_X[mPtrCtrlState->bSingleLegOldSel]);
                mLegPosYs[mPtrCtrlState->bSingleLegOldSel] = (s16)pgm_read_word(&TBL_INT_POS_Y[mPtrCtrlState->bSingleLegOldSel]);
                mLegPosZs[mPtrCtrlState->bSingleLegOldSel] = (s16)pgm_read_word(&TBL_INT_POS_Z[mPtrCtrlState->bSingleLegOldSel]);
            }
        }
        else if (!mPtrCtrlState->fSingleLegHold) {
            //mLegPosYs[mPtrCtrlState->bSingleLegCurSel] = mLegPosYs[mPtrCtrlState->bSingleLegCurSel]+mPtrCtrlState->c3dSingleLeg.y;
            mLegPosYs[mPtrCtrlState->bSingleLegCurSel] = (s16)pgm_read_word(&TBL_INT_POS_Y[mPtrCtrlState->bSingleLegCurSel])+mPtrCtrlState->c3dSingleLeg.y;// Using DIY remote Zenta prefer it this way
            mLegPosXs[mPtrCtrlState->bSingleLegCurSel] = (s16)pgm_read_word(&TBL_INT_POS_X[mPtrCtrlState->bSingleLegCurSel])+mPtrCtrlState->c3dSingleLeg.x;
            mLegPosZs[mPtrCtrlState->bSingleLegCurSel] = (s16)pgm_read_word(&TBL_INT_POS_Z[mPtrCtrlState->bSingleLegCurSel])+mPtrCtrlState->c3dSingleLeg.z;
        }
    } else {//All legs to init position
        if (!allDown) {
            for(u8 i = 0; i < CONFIG_NUM_LEGS; i++) {
                mLegPosXs[i] = (s16)pgm_read_word(&TBL_INT_POS_X[i]);
                mLegPosYs[i] = (s16)pgm_read_word(&TBL_INT_POS_Y[i]);
                mLegPosZs[i] = (s16)pgm_read_word(&TBL_INT_POS_Z[i]);
            }
        }
        if (mPtrCtrlState->bSingleLegOldSel != 255)
            mPtrCtrlState->bSingleLegOldSel = 255;
    }
    return allDown;
}

void PhoenixCore::selectGait(u8 bGaitType)
{
  //doGait selector
  switch (bGaitType)  {
  case 0:
    //Ripple doGait 12 steps
    mGaitLegInits[IDX_LR] = 1;
    mGaitLegInits[IDX_RF] = 3;
    mGaitLegInits[IDX_LM] = 5;
    mGaitLegInits[IDX_RR] = 7;
    mGaitLegInits[IDX_LF] = 9;
    mGaitLegInits[IDX_RM] = 11;

    mNrLiftedPos = 3;
    mFrontDownPos = 2;
    mLiftDivFactor = 2;
    mHalfLiftHeight = 3;
    mTLDivFactor = 8;
    mStepsInGait = 12;
    mNormGaitSpeed = DEFAULT_SLOW_GAIT;
    break;

  case 1:
    //Tripod 8 steps
    mGaitLegInits[IDX_LR] = 5;
    mGaitLegInits[IDX_RF] = 1;
    mGaitLegInits[IDX_LM] = 1;
    mGaitLegInits[IDX_RR] = 1;
    mGaitLegInits[IDX_LF] = 5;
    mGaitLegInits[IDX_RM] = 5;

    mNrLiftedPos = 3;
    mFrontDownPos = 2;
    mLiftDivFactor = 2;
    mHalfLiftHeight = 3;
    mTLDivFactor = 4;
    mStepsInGait = 8;
    mNormGaitSpeed = DEFAULT_SLOW_GAIT;
    break;

  case 2:
    //Triple Tripod 12 step
    mGaitLegInits[IDX_RF] = 3;
    mGaitLegInits[IDX_LM] = 4;
    mGaitLegInits[IDX_RR] = 5;
    mGaitLegInits[IDX_LF] = 9;
    mGaitLegInits[IDX_RM] = 10;
    mGaitLegInits[IDX_LR] = 11;

    mNrLiftedPos = 3;
    mFrontDownPos = 2;
    mLiftDivFactor = 2;
    mHalfLiftHeight = 3;
    mTLDivFactor = 8;
    mStepsInGait = 12;
    mNormGaitSpeed = DEFAULT_GAIT_SPEED;
    break;

  case 3:
    // Triple Tripod 16 steps, use 5 lifted positions
    mGaitLegInits[IDX_RF] = 4;
    mGaitLegInits[IDX_LM] = 5;
    mGaitLegInits[IDX_RR] = 6;
    mGaitLegInits[IDX_LF] = 12;
    mGaitLegInits[IDX_RM] = 13;
    mGaitLegInits[IDX_LR] = 14;

    mNrLiftedPos = 5;
    mFrontDownPos = 3;
    mLiftDivFactor = 4;
    mHalfLiftHeight = 1;
    mTLDivFactor = 10;
    mStepsInGait = 16;
    mNormGaitSpeed = DEFAULT_GAIT_SPEED;
    break;

  case 4:
    //Wave 24 steps
    mGaitLegInits[IDX_LR] = 1;
    mGaitLegInits[IDX_RF] = 21;
    mGaitLegInits[IDX_LM] = 5;

    mGaitLegInits[IDX_RR] = 13;
    mGaitLegInits[IDX_LF] = 9;
    mGaitLegInits[IDX_RM] = 17;

    mNrLiftedPos = 3;
    mFrontDownPos = 2;
    mLiftDivFactor = 2;
    mHalfLiftHeight = 3;
    mTLDivFactor = 20;
    mStepsInGait = 24;
    mNormGaitSpeed = DEFAULT_SLOW_GAIT;
    break;

  case 5:
    //Tripod 6 steps
    mGaitLegInits[IDX_LR] = 4;
    mGaitLegInits[IDX_RF] = 1;
    mGaitLegInits[IDX_LM] = 1;

    mGaitLegInits[IDX_RR] = 1;
    mGaitLegInits[IDX_LF] = 4;
    mGaitLegInits[IDX_RM] = 4;

    mNrLiftedPos = 2;
    mFrontDownPos = 1;
    mLiftDivFactor = 2;
    mHalfLiftHeight = 1;
    mTLDivFactor = 4;
    mStepsInGait = 6;
    mNormGaitSpeed = DEFAULT_GAIT_SPEED;
    break;
  }
}

//--------------------------------------------------------------------
//[GAIT Sequence]
void PhoenixCore::doGaitSeq(void)
{
    bool fTravelReq;

    //Check if the doGait is in motion
    fTravelReq = (abs(mPtrCtrlState->c3dTravelLen.x) > CONFIG_TRAVEL_DEAD_ZONE) ||
        (abs(mPtrCtrlState->c3dTravelLen.z) > CONFIG_TRAVEL_DEAD_ZONE) ||
        (abs(mPtrCtrlState->c3dTravelLen.y) > CONFIG_TRAVEL_DEAD_ZONE) ||
        (mPtrCtrlState->bForcemGaitStepCnt != 0) || mBoolWalking;

    //Calculate doGait sequence
    for (u8 i = 0; i < CONFIG_NUM_LEGS; i++) { // for all legs
        doGait(i, fTravelReq);
    }

    // If we have a force count decrement it now...
    if (mPtrCtrlState->bForcemGaitStepCnt)
        mPtrCtrlState->bForcemGaitStepCnt--;
}

//--------------------------------------------------------------------
//[GAIT]
void PhoenixCore::doGait(u8 leg, bool fTravelReq)
{
    //Clear values under the CONFIG_TRAVEL_DEAD_ZONE
    if (!fTravelReq) {
        mPtrCtrlState->c3dTravelLen.x = 0;
        mPtrCtrlState->c3dTravelLen.z = 0;
        mPtrCtrlState->c3dTravelLen.y = 0;//doGait NOT in motion, return to home position
    }

    //Leg middle up position OK
    //doGait in motion

    if ( (fTravelReq && (mNrLiftedPos == 1 || mNrLiftedPos == 3 || mNrLiftedPos == 5) && mGaitStep == mGaitLegInits[leg]) ||
         (!fTravelReq && mGaitStep == mGaitLegInits[leg] &&
          ((abs(mGaitPosXs[leg] )> 2) || (abs(mGaitPosZs[leg] )> 2) || (abs(mGaitRotYs[leg]) > 2)) ) ) { //Up
        mGaitPosXs[leg] = 0;
        mGaitPosYs[leg] = -mPtrCtrlState->sLegLiftHeight;
        mGaitPosZs[leg] = 0;
        mGaitRotYs[leg] = 0;
    }

    //Optional Half heigth Rear (2, 3, 5 lifted positions)
    else if (fTravelReq &&
              ( (mNrLiftedPos == 2 && mGaitStep == mGaitLegInits[leg]) ||
                (mNrLiftedPos >= 3 && (mGaitStep == mGaitLegInits[leg] - 1 || mGaitStep == mGaitLegInits[leg] + (mStepsInGait - 1))) ) ) {
        mGaitPosXs[leg] = -mPtrCtrlState->c3dTravelLen.x/mLiftDivFactor;
        mGaitPosYs[leg] = -3*mPtrCtrlState->sLegLiftHeight/(3+mHalfLiftHeight);     //Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
        mGaitPosZs[leg] = -mPtrCtrlState->c3dTravelLen.z/mLiftDivFactor;
        mGaitRotYs[leg] = -mPtrCtrlState->c3dTravelLen.y/mLiftDivFactor;
    }

    // _A_
    // Optional Half heigth front (2, 3, 5 lifted positions)
    else if (fTravelReq && (mNrLiftedPos >= 2) &&
             (mGaitStep == mGaitLegInits[leg] + 1 || mGaitStep == mGaitLegInits[leg] - (mStepsInGait - 1))) {
        mGaitPosXs[leg] = mPtrCtrlState->c3dTravelLen.x/mLiftDivFactor;
        mGaitPosYs[leg] = -3*mPtrCtrlState->sLegLiftHeight/(3+mHalfLiftHeight); // Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
        mGaitPosZs[leg] = mPtrCtrlState->c3dTravelLen.z/mLiftDivFactor;
        mGaitRotYs[leg] = mPtrCtrlState->c3dTravelLen.y/mLiftDivFactor;
    }

    //Optional Half heigth Rear 5 LiftedPos (5 lifted positions)
    else if (fTravelReq && (mNrLiftedPos == 5 && (mGaitStep == mGaitLegInits[leg] - 2))) {
        mGaitPosXs[leg] = -mPtrCtrlState->c3dTravelLen.x/2;
        mGaitPosYs[leg] = -mPtrCtrlState->sLegLiftHeight/2;
        mGaitPosZs[leg] = -mPtrCtrlState->c3dTravelLen.z/2;
        mGaitRotYs[leg] = -mPtrCtrlState->c3dTravelLen.y/2;
    }

    //Optional Half heigth Front 5 LiftedPos (5 lifted positions)
    else if (fTravelReq && (mNrLiftedPos == 5) &&
             (mGaitStep == mGaitLegInits[leg] + 2 || mGaitStep == mGaitLegInits[leg] - (mStepsInGait - 2))) {
        mGaitPosXs[leg] = mPtrCtrlState->c3dTravelLen.x/2;
        mGaitPosYs[leg] = -mPtrCtrlState->sLegLiftHeight/2;
        mGaitPosZs[leg] = mPtrCtrlState->c3dTravelLen.z/2;
        mGaitRotYs[leg] = mPtrCtrlState->c3dTravelLen.y/2;
    }
    //_B_
    //Leg front down position //bug here?  From _A_ to _B_ there should only be one gaitstep, not 2!
    //For example, where is the case of mGaitStep==mGaitLegInits[leg]+2 executed when NRLiftedPos=3?
    else if ((mGaitStep == mGaitLegInits[leg] + mFrontDownPos || mGaitStep == mGaitLegInits[leg] - (mStepsInGait-mFrontDownPos)) &&
        mGaitPosYs[leg] < 0) {
        mGaitPosXs[leg] = mPtrCtrlState->c3dTravelLen.x/2;
        mGaitPosZs[leg] = mPtrCtrlState->c3dTravelLen.z/2;
        mGaitRotYs[leg] = mPtrCtrlState->c3dTravelLen.y/2;
        mGaitPosYs[leg] = 0;
    }

    //Move body forward
    else {
        mGaitPosXs[leg] = mGaitPosXs[leg] - (mPtrCtrlState->c3dTravelLen.x/mTLDivFactor);
        mGaitPosYs[leg] = 0;
        mGaitPosZs[leg] = mGaitPosZs[leg] - (mPtrCtrlState->c3dTravelLen.z/mTLDivFactor);
        mGaitRotYs[leg] = mGaitRotYs[leg] - (mPtrCtrlState->c3dTravelLen.y/mTLDivFactor);
    }

    //Advance to the next step
    if (leg == 5)  {  //The last leg in this step
        mGaitStep++;
        if (mGaitStep > mStepsInGait)
            mGaitStep = 1;
    }
}


//--------------------------------------------------------------------
//[calcBalOneLeg]
void PhoenixCore::calcBalOneLeg (u8 leg, long posX, long posZ, long posY)
{
    long            CPR_X;            //Final X value for centerpoint of rotation
    long            CPR_Y;            //Final Y value for centerpoint of rotation
    long            CPR_Z;            //Final Z value for centerpoint of rotation
    long            lAtan;

    //Calculating totals from center of the body to the feet
    CPR_Z = (s16)pgm_read_word(&TBL_OFFSET_Z[leg]) + posZ;
    CPR_X = (s16)pgm_read_word(&TBL_OFFSET_X[leg]) + posX;
    CPR_Y = 150 + posY;        // using the value 150 to lower the centerpoint of rotation 'mPtrCtrlState->c3dBodyPos.y +

    mTotalTransY += (long)posY;
    mTotalTransZ += (long)CPR_Z;
    mTotalTransX += (long)CPR_X;

    lAtan = arctan2(CPR_X, CPR_Z, NULL);
    mTotalYBal1 += (lAtan * 1800) / 31415;

    lAtan = arctan2 (CPR_X, CPR_Y, NULL);
    mTotalZBal1 += ((lAtan * 1800) / 31415) -900; //Rotate balance circle 90 deg

    lAtan = arctan2 (CPR_Z, CPR_Y, NULL);
    mTotalXBal1 += ((lAtan * 1800) / 31415) - 900; //Rotate balance circle 90 deg
}

//--------------------------------------------------------------------
//[balanceBody]
void PhoenixCore::balanceBody(void)
{
    mTotalTransZ = mTotalTransZ / BALANCE_DIV_FACTOR ;
    mTotalTransX = mTotalTransX / BALANCE_DIV_FACTOR;
    mTotalTransY = mTotalTransY / BALANCE_DIV_FACTOR;

    if (mTotalYBal1 > 0)        //Rotate balance circle by +/- 180 deg
        mTotalYBal1 -=  1800;
    else
        mTotalYBal1 += 1800;

    if (mTotalZBal1 < -1800)    //Compensate for extreme balance positions that causes owerflow
        mTotalZBal1 += 3600;

    if (mTotalXBal1 < -1800)    //Compensate for extreme balance positions that causes owerflow
        mTotalXBal1 += 3600;

    //Balance rotation
    mTotalYBal1 = -mTotalYBal1 / BALANCE_DIV_FACTOR;
    mTotalXBal1 = -mTotalXBal1 / BALANCE_DIV_FACTOR;
    mTotalZBal1 =  mTotalZBal1 / BALANCE_DIV_FACTOR;
}

//--------------------------------------------------------------------
//(BODY INVERSE KINEMATICS)
//BodyRotX         - Global Input pitch of the body
//BodyRotY         - Global Input rotation of the body
//BodyRotZ         - Global Input roll of the body
//RotationY         - Input Rotation for the gait
//posX            - Input position of the feet X
//posZ            - Input position of the feet Z
//SinB                  - Sin buffer for BodyRotX
//CosB               - Cos buffer for BodyRotX
//SinG                  - Sin buffer for BodyRotZ
//CosG               - Cos buffer for BodyRotZ
//lBodyX         - Output Position X of feet with Rotation
//lBodyY         - Output Position Y of feet with Rotation
//lBodyZ         - Output Position Z of feet with Rotation
void PhoenixCore::getBodyIK(u8 leg, s16 posX, s16 posZ, s16 posY, s16 RotationY, long *x, long *y, long *z)
{
    s16            sinA4;          //Sin buffer for BodyRotX calculations
    s16            cosA4;          //Cos buffer for BodyRotX calculations
    s16            sinB4;          //Sin buffer for BodyRotX calculations
    s16            cosB4;          //Cos buffer for BodyRotX calculations
    s16            sinG4;          //Sin buffer for BodyRotZ calculations
    s16            cosG4;          //Cos buffer for BodyRotZ calculations
    s16            CPR_X;          //Final X value for centerpoint of rotation
    s16            CPR_Y;          //Final Y value for centerpoint of rotation
    s16            CPR_Z;          //Final Z value for centerpoint of rotation

    //Calculating totals from center of the body to the feet
    CPR_X = (s16)pgm_read_word(&TBL_OFFSET_X[leg])+posX + mPtrCtrlState->c3dBodyRotOff.x;
    CPR_Y = posY + mPtrCtrlState->c3dBodyRotOff.y;         //Define centerpoint for rotation along the Y-axis
    CPR_Z = (s16)pgm_read_word(&TBL_OFFSET_Z[leg]) + posZ + mPtrCtrlState->c3dBodyRotOff.z;

    //Successive global rotation matrix:
    //Math shorts for rotation: Alfa [A] = Xrotate, Beta [B] = Zrotate, Gamma [G] = Yrotate
    //Sinus Alfa = SinA, cosinus Alfa = cosA. and so on...

    //First calculate sinus and cosinus for each rotation:
    sincos(mPtrCtrlState->c3dBodyRot.x+mTotalXBal1, &sinG4, &cosG4);
    sincos(mPtrCtrlState->c3dBodyRot.z+mTotalZBal1, &sinB4, &cosB4);

    if (mBoolUpsideDown)
        sincos(-mPtrCtrlState->c3dBodyRot.y + (-RotationY * DEC_EXP_1) + mTotalYBal1, &sinA4, &cosA4) ;
    else
        sincos(mPtrCtrlState->c3dBodyRot.y + (RotationY * DEC_EXP_1) + mTotalYBal1, &sinA4, &cosA4) ;

    //Calcualtion of rotation matrix:
    *x = ((long)CPR_X*DEC_EXP_2 -
            ((long)CPR_X*DEC_EXP_2*cosA4/DEC_EXP_4*cosB4/DEC_EXP_4 -
             (long)CPR_Z*DEC_EXP_2*cosB4/DEC_EXP_4*sinA4/DEC_EXP_4 +
             (long)CPR_Y*DEC_EXP_2*sinB4/DEC_EXP_4)) / DEC_EXP_2;

    *z = ((long)CPR_Z*DEC_EXP_2 -
            ( (long)CPR_X*DEC_EXP_2*cosG4/DEC_EXP_4*sinA4/DEC_EXP_4 +
              (long)CPR_X*DEC_EXP_2*cosA4/DEC_EXP_4*sinB4/DEC_EXP_4*sinG4/DEC_EXP_4 +
              (long)CPR_Z*DEC_EXP_2*cosA4/DEC_EXP_4*cosG4/DEC_EXP_4 -
              (long)CPR_Z*DEC_EXP_2*sinA4/DEC_EXP_4*sinB4/DEC_EXP_4*sinG4/DEC_EXP_4 -
              (long)CPR_Y*DEC_EXP_2*cosB4/DEC_EXP_4*sinG4/DEC_EXP_4 )) / DEC_EXP_2;

    *y = ((long)CPR_Y  *DEC_EXP_2 -
            ( (long)CPR_X*DEC_EXP_2*sinA4/DEC_EXP_4*sinG4/DEC_EXP_4 -
              (long)CPR_X*DEC_EXP_2*cosA4/DEC_EXP_4*cosG4/DEC_EXP_4*sinB4/DEC_EXP_4 +
              (long)CPR_Z*DEC_EXP_2*cosA4/DEC_EXP_4*sinG4/DEC_EXP_4 +
              (long)CPR_Z*DEC_EXP_2*cosG4/DEC_EXP_4*sinA4/DEC_EXP_4*sinB4/DEC_EXP_4 +
              (long)CPR_Y*DEC_EXP_2*cosB4/DEC_EXP_4*cosG4/DEC_EXP_4 )) / DEC_EXP_2;
}

//--------------------------------------------------------------------
//[LEG INVERSE KINEMATICS] Calculates the angles of the coxa, femur and tibia for the given position of the feet
//IKFeetPosX            - Input position of the Feet X
//IKFeetPosY            - Input position of the Feet Y
//IKFeetPosZ            - Input Position of the Feet Z
//IKSolution            - Output TRUE if the solution is possible
//IKSolutionWarning     - Output TRUE if the solution is NEARLY possible
//IKSolutionError    - Output TRUE if the solution is NOT possible
//mFemurAngles           - Output Angle of Femur in degrees
//mTibiaAngles           - Output Angle of Tibia in degrees
//mCoxaAngles            - Output Angle of Coxa in degrees
//--------------------------------------------------------------------
u8 PhoenixCore::getLegIK(u8 leg, s16 IKFeetPosX, s16 IKFeetPosY, s16 IKFeetPosZ)
{
    u32    IKSW2;            //Length between Shoulder and Wrist, decimals = 2
    u32    IKA14;            //Angle of the line S>W with respect to the ground in radians, decimals = 4
    u32    IKA24;            //Angle of the line S>W with respect to the femur in radians, decimals = 4
    s16            IKFeetPosXZ;    //Diagonal direction from Input X and Z
#if (CONFIG_DOF_PER_LEG == 4)
    // these were shorts...
    long            TarsOffsetXZ;    //Vector value \ ;
    long            TarsOffsetY;     //Vector value / The 2 DOF IK calcs (femur and tibia) are based upon these vectors
    long            TarsToGroundAngle1;    //Angle between tars and ground. Note: the angle are 0 when the tars are perpendicular to the ground
    long            TGA_A_H4;
    long            TGA_B_H3;
#else
    #define TarsOffsetXZ 0	    // Vector value
    #define TarsOffsetY  0	    //Vector value / The 2 DOF IK calcs (femur and tibia) are based upon these vectors
#endif

    long            Temp1;
    long            Temp2;
    long            T3;
    long            hyp2XY;
    u8              ret;
#if (CONFIG_DOF_PER_LEG == 4)
    s16             sin4;
    s16             cos4;
#endif

    //Calculate IKCoxaAngle and IKFeetPosXZ
    s16 atan4 = arctan2 (IKFeetPosX, IKFeetPosZ, &hyp2XY);
    mCoxaAngles[leg] = (((long)atan4*180) / 3141) + (s16)pgm_read_word(&TBL_COXA_ANGLE[leg]);

    //Length between the Coxa and tars [foot]
    IKFeetPosXZ = hyp2XY / DEC_EXP_2;

#if (CONFIG_DOF_PER_LEG == 4)
    // Some legs may have the 4th DOF and some may not, so handle this here...
    //Calc the TarsToGroundAngle1:
    if ((u8)pgm_read_byte(&TBL_TARS_LENGTH[leg])) {    // We allow mix of 3 and 4 DOF legs...
        TarsToGroundAngle1 = -cTarsConst + cTarsMulti*IKFeetPosY + ((long)(IKFeetPosXZ*cTarsFactorA))/DEC_EXP_1 - ((long)(IKFeetPosXZ*IKFeetPosY)/(cTarsFactorB));
        if (IKFeetPosY < 0)     //Always compensate TarsToGroundAngle1 when IKFeetPosY it goes below zero
            TarsToGroundAngle1 = TarsToGroundAngle1 - ((long)(IKFeetPosY*cTarsFactorC)/DEC_EXP_1);     //TGA base, overall rule
        if (TarsToGroundAngle1 > 400)
            TGA_B_H3 = 200 + (TarsToGroundAngle1/2);
        else
            TGA_B_H3 = TarsToGroundAngle1;

        if (TarsToGroundAngle1 > 300)
            TGA_A_H4 = 240 + (TarsToGroundAngle1/5);
        else
            TGA_A_H4 = TarsToGroundAngle1;

        if (IKFeetPosY > 0)    //Only compensate the TarsToGroundAngle1 when it exceed 30 deg (A, H4 PEP note)
            TarsToGroundAngle1 = TGA_A_H4;
        else if (((IKFeetPosY <= 0) & (IKFeetPosY > -10))) // linear transition between case H3 and H4 (from PEP: H4-K5*(H3-H4))
            TarsToGroundAngle1 = (TGA_A_H4 -(((long)IKFeetPosY*(TGA_B_H3-TGA_A_H4))/DEC_EXP_1));
        else                //IKFeetPosY <= -10, Only compensate TGA1 when it exceed 40 deg
            TarsToGroundAngle1 = TGA_B_H3;

        //Calc Tars Offsets:
        sincos(TarsToGroundAngle1, &sin4, &cos4);
        TarsOffsetXZ = ((long)sin4*(u8)pgm_read_byte(&TBL_TARS_LENGTH[leg]))/DEC_EXP_4;
        TarsOffsetY = ((long)cos4*(u8)pgm_read_byte(&TBL_TARS_LENGTH[leg]))/DEC_EXP_4;
    } else {
        TarsOffsetXZ = 0;
        TarsOffsetY = 0;
    }
#endif

    //Using GetAtan2 for solving IKA1 and IKSW
    //IKA14 - Angle between SW line and the ground in radians
    IKA14 = arctan2(IKFeetPosY-TarsOffsetY, IKFeetPosXZ-(u8)pgm_read_byte(&TBL_COXA_LENGTH[leg])-TarsOffsetXZ, &hyp2XY);

    //IKSW2 - Length between femur axis and tars
    IKSW2 = hyp2XY;

    //IKA2 - Angle of the line S>W with respect to the femur in radians
    Temp1 = (( ((long)(u8)pgm_read_byte(&TBL_FEMUR_LENGTH[leg])*(u8)pgm_read_byte(&TBL_FEMUR_LENGTH[leg])) -
                ((long)(u8)pgm_read_byte(&TBL_TIBIA_LENGTH[leg])*(u8)pgm_read_byte(&TBL_TIBIA_LENGTH[leg])) )*DEC_EXP_4 + ((long)IKSW2*IKSW2));
    Temp2 = (long)(2*(u8)pgm_read_byte(&TBL_FEMUR_LENGTH[leg]))*DEC_EXP_2 * (u32)IKSW2;
    T3 = Temp1 / (Temp2/DEC_EXP_4);
    IKA24 = arccos (T3 );

    //IKFemurAngle
    if (mBoolUpsideDown)
        mFemurAngles[leg] = (long)(IKA14 + IKA24) * 180 / 3141 - 900 + OFFSET_FEMUR_HORN(leg);//Inverted, up side down
    else
        mFemurAngles[leg] = -(long)(IKA14 + IKA24) * 180 / 3141 + 900 + OFFSET_FEMUR_HORN(leg);//Normal

    //IKTibiaAngle
    Temp1 = ((((long)(u8)pgm_read_byte(&TBL_FEMUR_LENGTH[leg])*(u8)pgm_read_byte(&TBL_FEMUR_LENGTH[leg])) +
            ((long)(u8)pgm_read_byte(&TBL_TIBIA_LENGTH[leg])*(u8)pgm_read_byte(&TBL_TIBIA_LENGTH[leg])))*DEC_EXP_4 - ((long)IKSW2*IKSW2));
    Temp2 = (2*(u8)pgm_read_byte(&TBL_FEMUR_LENGTH[leg])*(u8)pgm_read_byte(&TBL_TIBIA_LENGTH[leg]));
    long angleRad4 = arccos(Temp1 / Temp2);

#ifdef OPT_WALK_UPSIDE_DOWN
    if (mBoolUpsideDown)
        mTibiaAngles[leg] = (1800-(long)angleRad4*180/3141);//Full range tibia, wrong side (up side down)
    else
        mTibiaAngles[leg] = -(1800-(long)angleRad4*180/3141);//Full range tibia, right side (up side up)
#else
#ifdef PHANTOMX_V2     // BugBug:: cleaner way?
    mTibiaAngles[leg] = -(1450-(long)angleRad4*180/3141); //!!!!!!!!!!!!145 instead of 1800
#else
    mTibiaAngles[leg] = -(900-(long)angleRad4*180/3141);
#endif
#endif

#if (CONFIG_DOF_PER_LEG == 4)
    //Tars angle
    if ((u8)pgm_read_byte(&TBL_TARS_LENGTH[leg])) {    // We allow mix of 3 and 4 DOF legs...
        mTarsAngles[leg] = (TarsToGroundAngle1 + mFemurAngles[leg] - mTibiaAngles[leg]) +
                           OFFSET_TARS_HORN(leg);
    }
#endif

    //Set the Solution quality
    if(IKSW2 < ((word)((u8)pgm_read_byte(&TBL_FEMUR_LENGTH[leg])+(u8)pgm_read_byte(&TBL_TIBIA_LENGTH[leg])-30)*DEC_EXP_2))
        ret = STATUS_OK;
    else {
        if(IKSW2 < ((word)((u8)pgm_read_byte(&TBL_FEMUR_LENGTH[leg])+(u8)pgm_read_byte(&TBL_TIBIA_LENGTH[leg]))*DEC_EXP_2))
            ret = STATUS_WARNING;
        else
            ret = STATUS_ERROR;
    }

  return ret;
}


//--------------------------------------------------------------------
//[CHECK ANGLES] Checks the mechanical limits of the servos
//--------------------------------------------------------------------
void PhoenixCore::validateAngles(void)
{
    for (u8 i = 0; i < CONFIG_NUM_LEGS; i++) {
        mCoxaAngles[i]  = min(max(mCoxaAngles[i], (s16)pgm_read_word(&TBL_COXA_MIN[i])),
            (s16)pgm_read_word(&TBL_COXA_MAX[i]));
        mFemurAngles[i] = min(max(mFemurAngles[i], (s16)pgm_read_word(&TBL_FEMUR_MIN[i])),
            (s16)pgm_read_word(&TBL_FEMUR_MAX[i]));
        mTibiaAngles[i] = min(max(mTibiaAngles[i], (s16)pgm_read_word(&TBL_TIBIA_MIN[i])),
            (s16)pgm_read_word(&TBL_TIBIA_MAX[i]));
#if (CONFIG_DOF_PER_LEG == 4)
        if ((u8)pgm_read_byte(&TBL_TARS_LENGTH[i])) {    // We allow mix of 3 and 4 DOF legs...
            mTarsAngles[i] = min(max(mTarsAngles[i], (s16)pgm_read_word(&TBL_TARS_MIN[i])),
                (s16)pgm_read_word(&TBL_TARS_MAX[i]));
        }
#endif
    }
}


//--------------------------------------------------------------------
// smoothControl (From Zenta) -  This function makes the body
//            rotation and translation much smoother
//--------------------------------------------------------------------
s16 PhoenixCore::smoothControl (s16 CtrlMoveInp, s16 CtrlMoveOut, u8 CtrlDivider)
{
    if (CtrlMoveOut < (CtrlMoveInp - 4))
        return CtrlMoveOut + abs((CtrlMoveOut - CtrlMoveInp)/CtrlDivider);
    else if (CtrlMoveOut > (CtrlMoveInp + 4))
        return CtrlMoveOut - abs((CtrlMoveOut - CtrlMoveInp)/CtrlDivider);

    return CtrlMoveInp;
}

//--------------------------------------------------------------------
// adjustLegPosToBodyHeight() - Will try to adjust the position of the legs
//     to be appropriate for the current y location of the body...
//--------------------------------------------------------------------

u8 oldLegInitIdx = 0x00;    // remember which index we are currently using...


static const u8 TBL_INIT_HEX_XZ[]    PROGMEM = {cHexInitXZ, 99, 86};
static const u8 TBL_MAX_HEX_BODY_Y[] PROGMEM = { 20, 50, MAX_BODY_Y};

void PhoenixCore::adjustLegPosToBodyHeight(void)
{
#ifdef CNT_HEX_INITS
    s16 sin4;
    s16 cos4;

    // Lets see which of our units we should use...
    // Note: We will also limit our body height here...
    if (mPtrCtrlState->c3dBodyPos.y > (s16)pgm_read_byte(&TBL_MAX_HEX_BODY_Y[CNT_HEX_INITS - 1]))
        mPtrCtrlState->c3dBodyPos.y =  (s16)pgm_read_byte(&TBL_MAX_HEX_BODY_Y[CNT_HEX_INITS - 1]);

    u8    i;
    u16   XZLength1 = pgm_read_byte(&TBL_INIT_HEX_XZ[CNT_HEX_INITS - 1]);
    for (i = 0; i < (CNT_HEX_INITS-1); i++) {    // Don't need to look at last entry as we already init to assume this one...
        if (mPtrCtrlState->c3dBodyPos.y <= (s16)pgm_read_byte(&TBL_MAX_HEX_BODY_Y[i])) {
            XZLength1 = pgm_read_byte(&TBL_INIT_HEX_XZ[i]);
            break;
        }
    }

    if (i != oldLegInitIdx) {
        oldLegInitIdx = i;  // remember the current index...
        //now lets see what happens when we change the leg positions...
        for (u8 i = 0; i < CONFIG_NUM_LEGS; i++) {
            printf(F("leg:%d (%d, %d) -> "), i, mLegPosXs[i], mLegPosZs[i]);
            sincos((s16)pgm_read_word(&TBL_COXA_ANGLE[i]), &sin4, &cos4);
            mLegPosXs[i] = ((long)((long)cos4 * XZLength1))/DEC_EXP_4;  //Set start positions for each leg
            mLegPosZs[i] = -((long)((long)sin4 * XZLength1))/DEC_EXP_4;
            printf(F("(%d, %d)\n"), mLegPosXs[i], mLegPosZs[i]);
        }
        // Make sure we cycle through one gait to have the legs all move into their new locations...
        mPtrCtrlState->bForcemGaitStepCnt = mStepsInGait;
    }
#endif // CNT_HEX_INITS

}

#ifdef CONFIG_TERMINAL
//==============================================================================
// showTerminal - Simple background task checks to see if the user is asking
//    us to do anything, like update debug levels ore the like.
//==============================================================================
bool PhoenixCore::showTerminal(void)
{
    u8  szCmdLine[20];
    u8  ich;
    int ch;

    // See if we need to output a prompt.
    if (mBoolShowDbgPrompt) {
        printf(F("Arduino Phoenix Monitor\n"));
        printf(F("D - Toggle debug on or off\n"));
        printf(F("E - Dump EEPROM\n"));

        // Let the Servo driver show it's own set of commands...
        mServo->showTerminal();
        mBoolShowDbgPrompt = FALSE;
    }

    ich = CONFIG_DBG_SERIAL.available();
    if (ich == 0)
        return FALSE;

    for (ich = 0; ich < sizeof(szCmdLine) - 1; ich++) {
        ch = CONFIG_DBG_SERIAL.read();
        if ((ch == -1) || ((ch >= 10) && (ch <= 15)))
            break;
        szCmdLine[ich] = ch;
    }
    szCmdLine[ich] = '\0';    // go ahead and null terminate it...
    printf(F("Terminal Cmd Line:%s !!\n"), szCmdLine);

    if (ich == 0) {
        mBoolShowDbgPrompt = TRUE;
    } else {
        switch(szCmdLine[0]) {
            case 'd':
            case 'D':
                 mBoolDbgOutput = !mBoolDbgOutput;
                if (mBoolDbgOutput)
                    printf(F("Debug is on\n"));
                else
                    printf(F("Debug is off\n"));
                break;

            case 'e':
            case 'E':
                handleEEPROM(szCmdLine);
                break;

            default:
                mServo->handleTerminal(szCmdLine, ich);
                break;
        }
    }
    return TRUE;
}

//--------------------------------------------------------------------
// handleEEPROM
//--------------------------------------------------------------------
void PhoenixCore::handleEEPROM(u8 *pszCmdLine)
{
    static u16 mAddrEEPROM = 0;
    u16 cnt = 16;

    // first u8 can be H for hex or W for words...
    if (!*++pszCmdLine)  // Need to get past the command letter first...
        Utils::dumpEEPROM(mAddrEEPROM, cnt);
    else {
        // First argument should be the start location to dump
        mAddrEEPROM = Utils::getCmdLineNum(&pszCmdLine);

        // If the next u8 is an "=" may try to do updates...
        if (*pszCmdLine == '=') {
            // make sure we don't get stuck in a loop...
            u8 *psz = pszCmdLine;
            word w;
            while (*psz) {
                w = Utils::getCmdLineNum(&psz);
                if (psz == pszCmdLine)
                break;  // not valid stuff so bail!
                pszCmdLine = psz;  // remember how far we got...

                EEPROM.write(mAddrEEPROM++, w & 0xff);
            }
        }
        else {
            if (*pszCmdLine == ' ') { // A blank assume we have a count...
                cnt = Utils::getCmdLineNum(&pszCmdLine);
            }
        }
        Utils::dumpEEPROM(mAddrEEPROM, cnt);
    }
}
#endif


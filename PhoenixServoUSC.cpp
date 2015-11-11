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


#include <EEPROM.h>
#include "PhoenixServoUSC.h"

static const u8 TBL_COXA_PIN[] PROGMEM = {
    PIN_RR_COXA,  PIN_RM_COXA,  PIN_RF_COXA,  PIN_LR_COXA,  PIN_LM_COXA,  PIN_LF_COXA
};

static const u8 TBL_FEMUR_PIN[] PROGMEM = {
    PIN_RR_FEMUR, PIN_RM_FEMUR, PIN_RF_FEMUR, PIN_LR_FEMUR, PIN_LM_FEMUR, PIN_LF_FEMUR
};

static const u8 TBL_TIBIA_PIN[] PROGMEM = {
    PIN_RR_TIBIA, PIN_RM_TIBIA, PIN_RF_TIBIA, PIN_LR_TIBIA, PIN_LM_TIBIA, PIN_LF_TIBIA
};

#if (CONFIG_DOF_PER_LEG == 4)
static const u8 TBL_TARS_PIN[] PROGMEM = {
    PIN_RR_TARS, PIN_RM_TARS, PIN_RF_TARS, PIN_LR_TARS, PIN_LM_TARS, PIN_LF_TARS
};
#endif


static const s16 TBL_LEGS_OFFSET[] PROGMEM = {
    0, -150,  -70,
    0, -115,  160,
    0,  -55,   80,
    0,   45,    0,
    0, -115,  165,
    0, -135,   60,
};

PhoenixServoUSC::PhoenixServoUSC(void)
{
#if defined(CONFIG_SERVO_USC_TX)
    mSerial = new SoftwareSerial(CONFIG_SERVO_USC_RX, CONFIG_SERVO_USC_TX);
    mSerial->begin(CONFIG_SERVO_USC_BAUD);
#endif
}

PhoenixServoUSC::PhoenixServoUSC(HardwareSerial *serial)
{
#if !defined(CONFIG_SERVO_USC_TX) && defined(CONFIG_CPU_PROMINI)
    mSerial = serial;
#endif
}

//-----------------------------------------------------------------------------
// load the servo offsets from the EEPROM
//-----------------------------------------------------------------------------
void PhoenixServoUSC::loadServosConfig(void)
{
    u8    *pb = (u8*)&mServoOffsets;
    u8    bChkSum = 0;
    int   i;

    memset(mServoOffsets, 0, sizeof(mServoOffsets));

    if (EEPROM.read(0) == CONFIG_NUM_LEGS * CONFIG_DOF_PER_LEG) {
        printf(F("Load from EEPROM !!\n"));
        for (i=0; i < sizeof(mServoOffsets); i++) {
            *pb = EEPROM.read(i+2);
            bChkSum += *pb++;
        }
        if (bChkSum == EEPROM.read(1)) {
            printf(F("Offset checksum is okay !!\n"));
            return;
        }
    }

    printf(F("Load default !!\n"));
    for (i = 0; i < CONFIG_NUM_LEGS * CONFIG_DOF_PER_LEG; i++) {
        mServoOffsets[i] = (s16)pgm_read_word(&TBL_LEGS_OFFSET[i]);
    }
}


//-----------------------------------------------------------------------------
// init
//-----------------------------------------------------------------------------
void PhoenixServoUSC::init(void)
{
    printf(F("%s\n"), __PRETTY_FUNCTION__);

    mBoolServosAttached = FALSE;
    loadServosConfig();

    mVoltSum = 0;
    mVoltIdx = 0;
    memset(mVoltBuf, 0, sizeof(mVoltBuf));

    for (u8 i = 0; i < CONFIG_VBAT_SMOOTH; i++)
        getBattVolt();


/*
    char buf[80];
    attachServos();
    for (;;) {
        for (int i = 0; i < 6; i++) {
            writeServo(i, 1000, 1000, 1000);
        }
        commit(500);
        delay(700);

        for (int i = 0; i < 6; i++) {
            writeServo(i, 2500, 2500, 2500);
        }
        commit(500);
        delay(700);
    }
*/
}

//-----------------------------------------------------------------------------
// getBattVolt
//-----------------------------------------------------------------------------
u8 PhoenixServoUSC::getBattVolt(void) {
    u16 v;
#ifdef PIN_ANALOG_VOLT
    v = analogRead(PIN_ANALOG_VOLT);
#else
    return CONFIG_VOLT_ON;
#endif

    mVoltSum += v;
    mVoltSum -= mVoltBuf[mVoltIdx];
    mVoltBuf[mVoltIdx++] = v;
    mVoltIdx %= CONFIG_VBAT_SMOOTH;

    u8 t;
#if CONFIG_VBAT_SMOOTH == 16
    t = mVoltSum / CONFIG_VBAT_SCALE + CONFIG_VBAT_OFFSET;
#elif CONFIG_VBAT_SMOOTH < 16
    t = (mVoltSum * (16 / CONFIG_VBAT_SMOOTH)) / CONFIG_VBAT_SCALE + CONFIG_VBAT_OFFSET;
#else
    t = ((mVoltSum / CONFIG_VBAT_SMOOTH) * 16) / CONFIG_VBAT_SCALE + CONFIG_VBAT_OFFSET;
#endif
    printf(F("analog:%d voltsum:%d, v:%d\n"), v, mVoltSum, t);

    return t;
}

//-----------------------------------------------------------------------------
// attachServos
//-----------------------------------------------------------------------------
void PhoenixServoUSC::attachServos(void) {
    u8 tot = 0;

    if (!mBoolServosAttached) {
        for (u8 j = 0; j < CONFIG_NUM_LEGS; j++) {
            mServos[tot++] = pgm_read_byte(&TBL_COXA_PIN[j]);
            mServos[tot++] = pgm_read_byte(&TBL_FEMUR_PIN[j]);
            mServos[tot++] = pgm_read_byte(&TBL_TIBIA_PIN[j]);
#if (CONFIG_DOF_PER_LEG == 4)
            mServos[tot++] = pgm_read_byte(&TBL_TARS_PIN[j]);
#endif
        }
        mBoolServosAttached = TRUE;
    }
}

//-----------------------------------------------------------------------------
// start
//-----------------------------------------------------------------------------
void PhoenixServoUSC::start(void)    // Start the update
{
    attachServos();
}

//-----------------------------------------------------------------------------
// write
//-----------------------------------------------------------------------------
#define PWM_DIV       991  //old 1059;
#define PF_CONST      592  //old 650 ; 900*(1000/PWM_DIV)+PF_CONST must always be 1500

// A PWM/deg factor of 10,09 give PWM_DIV = 991 and PF_CONST = 592
// For a modified 5645 (to 180 deg travel): PWM_DIV = 1500 and PF_CONST = 900.
#if (CONFIG_DOF_PER_LEG == 4)
void PhoenixServoUSC::write(u8 leg, s16 sCoxaAngle1, s16 sFemurAngle1, s16 sTibiaAngle1, s16 sTarsAngle1)
#else
void PhoenixServoUSC::write(u8 leg, s16 sCoxaAngle1, s16 sFemurAngle1, s16 sTibiaAngle1)
#endif
{
    u16    wCoxaSSCV;           // Coxa value in SSC units
    u16    wFemurSSCV;
    u16    wTibiaSSCV;
#if (CONFIG_DOF_PER_LEG == 4)
    u16    wTarsSSCV;
#endif

    //Update Right Legs
    if (leg < 3) {
        wCoxaSSCV  = ((long)(sCoxaAngle1   + 900)) * 1000 / PWM_DIV + PF_CONST;
        wFemurSSCV = ((long)(-sFemurAngle1 + 900)  * 1000 / PWM_DIV + PF_CONST);
        wTibiaSSCV = ((long)(-sTibiaAngle1 + 900)) * 1000 / PWM_DIV + PF_CONST;
#if (CONFIG_DOF_PER_LEG == 4)
        wTarsSSCV  = ((long)(-sTarsAngle1 + 900)) * 1000 / PWM_DIV + PF_CONST;
#endif
    } else {
        wCoxaSSCV  = ((long)(-sCoxaAngle1 + 900)) * 1000 / PWM_DIV + PF_CONST;
        wFemurSSCV = ((long)((long)(-sFemurAngle1 + 900)) * 1000 / PWM_DIV + PF_CONST);
        wTibiaSSCV = ((long)(-sTibiaAngle1 + 900)) * 1000 / PWM_DIV + PF_CONST;
#if (CONFIG_DOF_PER_LEG == 4)
        wTarsSSCV  = ((long)(sTarsAngle1 + 900)) * 1000 / PWM_DIV + PF_CONST;
#endif
    }
    mServoValues[leg][0] = wCoxaSSCV;
    mServoValues[leg][1] = wFemurSSCV;
    mServoValues[leg][2] = wTibiaSSCV;
#if (CONFIG_DOF_PER_LEG == 4)
    mServoValues[leg][3] = wTarsSSCV;
#endif
}



#if (CONFIG_DOF_PER_LEG == 4)
void PhoenixServoUSC::writeServo(u8 leg, u16 wCoxa, u16 wFemur, u16 wTibia, u16 wTars)
#else
void PhoenixServoUSC::writeServo(u8 leg, u16 wCoxa, u16 wFemur, u16 wTibia)
#endif
{
    char    fmt[7];
    char    buf[10];
    u8      i = leg * CONFIG_DOF_PER_LEG;

    memcpy_P(fmt, PSTR("#%dP%d\x00"), 7);

    memset(buf, 0, sizeof(buf));
    sprintf(buf, fmt, mServos[i], wCoxa + mServoOffsets[i]);
    i++;
    mSerial->write(buf);
    //printf(buf);

    memset(buf, 0, sizeof(buf));
    sprintf(buf, fmt, mServos[i], wFemur + mServoOffsets[i]);
    i++;
    mSerial->write(buf);
    //printf(buf);

    memset(buf, 0, sizeof(buf));
    sprintf(buf, fmt, mServos[i++], wTibia + mServoOffsets[i]);
    i++;
    mSerial->write(buf);
    //printf(buf);

#if (CONFIG_DOF_PER_LEG == 4)
    memset(buf, 0, sizeof(buf));
    sprintf(buf, fmt, mServos[i], wTars + mServoOffsets[i]);
    i++;
    mSerial->write(buf);
#endif
}

//-----------------------------------------------------------------------------
// commit
//-----------------------------------------------------------------------------
void PhoenixServoUSC::commit(u16 wMoveTime)
{
    char buf[20];

    for (u8 i = 0; i < CONFIG_NUM_LEGS; i++) {
#if (CONFIG_DOF_PER_LEG == 4)
        writeServo(i, mServoValues[i][0], mServoValues[i][1], mServoValues[i][2], mServoValues[i][3]);
#else
        writeServo(i, mServoValues[i][0], mServoValues[i][1], mServoValues[i][2]);
#endif
    }

    sprintf(buf, "T%d\r\n", wMoveTime);
    mSerial->write(buf);
    printf(buf);

    printf(F("--- TS:%ld \r\n"), millis());
}

//-----------------------------------------------------------------------------
// release
//-----------------------------------------------------------------------------
void PhoenixServoUSC::release(void)
{
    for (u8 leg = 0; leg < CONFIG_NUM_LEGS; leg++) {
#if (CONFIG_DOF_PER_LEG == 4)
        writeServo(leg, 1500, 1500, 1500, 1500);
#else
        writeServo(leg, 1500, 1500, 1500);
#endif
    }
    commit(500);
    mBoolServosAttached = FALSE;
}

#ifdef CONFIG_TERMINAL
//-----------------------------------------------------------------------------
// showTerminal
//-----------------------------------------------------------------------------
void PhoenixServoUSC::showTerminal(void)
{
    printf(F("O - Enter Servo offset mode\n"));
}

//-----------------------------------------------------------------------------
// handleTerminal
//-----------------------------------------------------------------------------
bool PhoenixServoUSC::handleTerminal(u8 *psz, u8 bLen)
{
    if ((bLen == 1) && ((*psz == 'o') || (*psz == 'O'))) {
        handleServoOffsets();
    }
}

void PhoenixServoUSC::setLegs1500ms(void)
{
    for (u8 leg = 0; leg < CONFIG_NUM_LEGS; leg++) {
#if (CONFIG_DOF_PER_LEG == 4)
        writeServo(leg, 1500, 1500, 1500, 1500);
#else
        writeServo(leg, 1500, 1500, 1500);
#endif
    }
    commit(500);
}

//-----------------------------------------------------------------------------
//	handleServoOffsets
//-----------------------------------------------------------------------------
static const char *apszLegs[]    = {"RR","RM","RF", "LR", "LM", "LF"};
static const char *apszLJoints[] = {" Coxa", " Femur", " Tibia", " tArs"};

void PhoenixServoUSC::move(int servo, int val, unsigned int time)
{
    char    fmt[12];
    char    buf[20];

    memcpy_P(fmt, PSTR("#%dP%dT%d\r\n\x00"), 12);

    memset(buf, 0, sizeof(buf));
    sprintf(buf, fmt, mServos[servo], val, time);
    mSerial->write(buf);
    printf(buf);
    delay(time);
}

void PhoenixServoUSC::handleServoOffsets(void)
{
    int  data;
    s16  sSN = 0;
    bool fNew = TRUE;
    bool fExit = FALSE;
    int  ich;
    s16  sOffset;

    printf(F("Find Servo Zeros.\n$-Exit, +- changes, *-change servo\n"));
    printf(F("    0-5 Chooses a leg, C-Coxa, F-Femur, T-Tibia, A-tArs\n"));
    attachServos();
    setLegs1500ms();

    while (!fExit) {
        if (fNew) {
            sOffset = mServoOffsets[sSN];
            printf(F("Servo: %s-%s (%d)\n"), apszLegs[sSN / CONFIG_DOF_PER_LEG], apszLJoints[sSN % CONFIG_DOF_PER_LEG], sOffset);

            // Now lets wiggle the servo
            move(sSN, 1500 + sOffset + 250, 250);
            move(sSN, 1500 + sOffset - 250, 250);
            move(sSN, 1500 + sOffset, 250);
            fNew = FALSE;
        }

    	data = CONFIG_DBG_SERIAL.read();

    	if (data != -1) {
            if (data == '$')
        	    fExit = TRUE;
            else if ((data == '+') || (data == '-')) {
                if (data == '+')
                    sOffset += 5;
        	    else
                    sOffset -= 5;
            	printf(F(" %4d\n"), sOffset);
                mServoOffsets[sSN] = sOffset;
                move(sSN, 1500 + sOffset, 200);
            } else if ((data >= '0') && (data <= '5')) {
            	fNew = TRUE;
            	sSN = (sSN % CONFIG_DOF_PER_LEG) + (data - '0') * CONFIG_DOF_PER_LEG;
            } else if ((data == 'c') || (data == 'C')) {
            	fNew = TRUE;
            	sSN = (sSN / CONFIG_DOF_PER_LEG) * CONFIG_DOF_PER_LEG + 0;
            } else if ((data == 'f') || (data == 'F')) {
            	fNew = TRUE;
            	sSN = (sSN / CONFIG_DOF_PER_LEG) * CONFIG_DOF_PER_LEG + 1;
            } else if ((data == 't') || (data == 'T')) {
            	fNew = TRUE;
            	sSN = (sSN / CONFIG_DOF_PER_LEG) * CONFIG_DOF_PER_LEG + 2;
#if (CONFIG_DOF_PER_LEG == 4)
            } else if ((data == 'a') || (data == 'A')) {
            	fNew = TRUE;
            	sSN = (sSN / CONFIG_DOF_PER_LEG) * CONFIG_DOF_PER_LEG + 3;
#endif
            } else if (data == '*') {
            	fNew = TRUE;
            	sSN++;
            	if (sSN == CONFIG_NUM_LEGS * CONFIG_DOF_PER_LEG)
                    sSN = 0;
            }
        }
    }

    printf(F("Servo offsets : "));
    for (u8 i = 0; i < CONFIG_NUM_LEGS * CONFIG_DOF_PER_LEG; i++) {
        printf(F("%4d, "), mServoOffsets[i]);
    }

    printf(F("\nSave Changes? Y/N: "));
    while (((data = CONFIG_DBG_SERIAL.read()) == -1) || ((data >= 10) && (data <= 15)));
    if ((data == 'Y') || (data == 'y')) {
        u8  *pb = (u8*)&mServoOffsets;
        u8  bChkSum = 0;

        EEPROM.write(0, CONFIG_NUM_LEGS * CONFIG_DOF_PER_LEG);
        for (sSN=0; sSN < sizeof(mServoOffsets); sSN++) {
            EEPROM.write(sSN + 2, *pb);
            bChkSum += *pb++;
        }
        EEPROM.write(1, bChkSum);
    } else {
        loadServosConfig();
    }

    release();
}
#endif


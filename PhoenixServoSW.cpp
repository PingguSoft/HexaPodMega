//====================================================================
//Project Lynxmotion Phoenix
//
// Servo Driver: This version is setup to use the main processor to
//    drive the servos, using my hacked library ServoEx, which is based
//    on the Servo class.
//====================================================================
#include <EEPROM.h>
#include "PhoenixServoSW.h"

//Servo Pin numbers - May be SSC-32 or actual pins on main controller, depending on configuration.
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

// 12 64 0 0 A1 FF 56 FF 0 0 92 FF 0 0 0 0
static const s16 TBL_LEGS_OFFSET[] PROGMEM = {
    0, -190, -215,
    0, -150,    0,
    0,  -90, -110,
    0,   15, -140,
    0, -130,    0,
    0, -115,  -75
};


PhoenixServoSW::PhoenixServoSW(void)
{

}

//--------------------------------------------------------------------
// Helper function to load the servo offsets from the EEPROM
//--------------------------------------------------------------------
void PhoenixServoSW::loadServosConfig(void)
{
    u8    *pb = (u8*)&mServoOffsets;
    u8    bChkSum = 0;
    int   i;

    memset(mServoOffsets, 0, sizeof(mServoOffsets));

#if 1
    if (EEPROM.read(0) == 6 * CONFIG_DOF_PER_LEG) {
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
#endif

    printf(F("Load default !!\n"));
    for (i = 0; i < 6*CONFIG_DOF_PER_LEG; i++) {
        mServoOffsets[i] = (s16)pgm_read_word(&TBL_LEGS_OFFSET[i]);
    }
}


//--------------------------------------------------------------------
//init
//--------------------------------------------------------------------
void PhoenixServoSW::init(void) {
    u8 i;

    printf(F("%s\n"), __PRETTY_FUNCTION__);

    mBoolServosAttached = false;
    loadServosConfig();

#ifdef PIN_ANALOG_VOLT
    // If we have a voltage pin, we are doing averaging of voltages over
    // 8 reads, so lets prefill that array...
    for (i = 0; i < 8; i++)
        getBattVolt();
#endif
}

//--------------------------------------------------------------------
//getBattVolt - Maybe should try to minimize when this is called
// as it uses the serial port... Maybe only when we are not interpolating
// or if maybe some minimum time has elapsed...
//--------------------------------------------------------------------
u16 PhoenixServoSW::getBattVolt(void) {
    mVoltIdx  = (++mVoltIdx) & 0x7;
    mVoltSum -= mVoltBuf[mVoltIdx];
    mVoltBuf[mVoltIdx] = analogRead(PIN_ANALOG_VOLT);
    mVoltSum += mVoltBuf[mVoltIdx];

    return ((long)((long)mVoltSum * 125 * (CONFIG_VOLT_R1 + CONFIG_VOLT_R2))/(long)(2048 * (long)CONFIG_VOLT_R2));
}

bool PhoenixServoSW::checkVoltage(void) {
#if 0 //def CONFIG_VOLT_OFF
    u16     volt;
    bool    on = TRUE;

    volt = getBattVolt();
    printf(F("VOLT:%d\n"), volt);
    if ((volt < CONFIG_VOLT_OFF) || (volt >= 1999)) {
        if (mVoltWarnBeepCnt < 5) {
            mVoltWarnBeepCnt++;
        } else {
            printf(F("volt went low :%d\n"), volt);
            on = FALSE;
            mVoltWarnBeepCnt = 0;
        }
    }
    return on;
#else
    return TRUE;
#endif
}

//--------------------------------------------------------------------
// Attach Servos...
//--------------------------------------------------------------------
void PhoenixServoSW::attachServos(void) {
    u8 tot = 0;

    if (!mBoolServosAttached) {
        for (u8 j=0; j < 6; j++) {
            // BUGBUG:: will probably need to add additional stuff here to get the servo offsets
            // and calculate min/max to use...
            mServoLegs[tot++].attach(pgm_read_byte(&TBL_COXA_PIN[j]));
            mServoLegs[tot++].attach(pgm_read_byte(&TBL_FEMUR_PIN[j]));
            mServoLegs[tot++].attach(pgm_read_byte(&TBL_TIBIA_PIN[j]));
#if (CONFIG_DOF_PER_LEG == 4)
            mServoLegs[tot++].attach(pgm_read_byte(&TBL_TARS_PIN[j]));
#endif
        }
        mBoolServosAttached = true;
    }
}


//------------------------------------------------------------------------------------------
//[start] Does whatever preperation that is needed to starrt a move of our servos
//------------------------------------------------------------------------------------------
void PhoenixServoSW::start(void)    // Start the update
{
    attachServos();
    mSGM.start();    // tell the group move system we are starting a new move
}

//------------------------------------------------------------------------------------------
//[write] Do the output to the SSC-32 for the servos associated with
//         the Leg number passed in.
//------------------------------------------------------------------------------------------
#define PWM_DIV       991  //old 1059;
#define PF_CONST      592  //old 650 ; 900*(1000/PWM_DIV)+PF_CONST must always be 1500

// A PWM/deg factor of 10,09 give PWM_DIV = 991 and PF_CONST = 592
// For a modified 5645 (to 180 deg travel): PWM_DIV = 1500 and PF_CONST = 900.
#if (CONFIG_DOF_PER_LEG == 4)
void PhoenixServoSW::write(u8 leg, s16 sCoxaAngle1, s16 sFemurAngle1, s16 sTibiaAngle1, s16 sTarsAngle1)
#else
void PhoenixServoSW::write(u8 leg, s16 sCoxaAngle1, s16 sFemurAngle1, s16 sTibiaAngle1)
#endif
{
  u16    wCoxaSSCV;        // Coxa value in SSC units
  u16    wFemurSSCV;        //
  u16    wTibiaSSCV;        //
#if (CONFIG_DOF_PER_LEG == 4)
  u16    wTarsSSCV;        //
#endif
  //Update Right Legs
  //g_InputController.AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
  if (leg < 3) {
    wCoxaSSCV = ((long)(sCoxaAngle1 +900))*1000/PWM_DIV+PF_CONST;
    wFemurSSCV = ((long)(-sFemurAngle1+900)*1000/PWM_DIV+PF_CONST);
    wTibiaSSCV = ((long)(-sTibiaAngle1+900))*1000/PWM_DIV+PF_CONST;
#if (CONFIG_DOF_PER_LEG == 4)
    wTarsSSCV = ((long)(-sTarsAngle1+900))*1000/PWM_DIV+PF_CONST;
#endif
  }
  else {
    wCoxaSSCV = ((long)(-sCoxaAngle1 +900))*1000/PWM_DIV+PF_CONST;
    wFemurSSCV = ((long)((long)(-sFemurAngle1+900))*1000/PWM_DIV+PF_CONST);
    wTibiaSSCV = ((long)(-sTibiaAngle1+900))*1000/PWM_DIV+PF_CONST;
#if (CONFIG_DOF_PER_LEG == 4)
    wTarsSSCV = ((long)(sTarsAngle1+900))*1000/PWM_DIV+PF_CONST;
#endif
  }

  // Now lets tell the servos their next  location...
  u8 i = leg * CONFIG_DOF_PER_LEG;
  mServoLegs[i].writeMicroseconds(wCoxaSSCV + mServoOffsets[i]);
  i++;
  mServoLegs[i].writeMicroseconds(wFemurSSCV + mServoOffsets[i]);
  i++;
  mServoLegs[i].writeMicroseconds(wTibiaSSCV + mServoOffsets[i]);
#if (CONFIG_DOF_PER_LEG == 4)
  i++;
  mServoLegs[i].writeMicroseconds(wTarsSSCV + mServoOffsets[i]);
#endif
}


//--------------------------------------------------------------------
//[commit Updates the positions of the servos - This outputs
//         as much of the command as we can without committing it.  This
//         allows us to once the previous update was completed to quickly
//        get the next command to start
//--------------------------------------------------------------------
void PhoenixServoSW::commit(u16 wMoveTime)
{
    mSGM.commit(wMoveTime);
}

//--------------------------------------------------------------------
//[FREE SERVOS] Frees all the servos
//--------------------------------------------------------------------
void PhoenixServoSW::release(void)
{
    for (u8 i = 0; i < 6 * CONFIG_DOF_PER_LEG; i++) {
        mServoLegs[i].detach();
    }
    mBoolServosAttached = false;
}

#ifdef CONFIG_TERMINAL
//==============================================================================
// showTerminal: Allow the Terminal monitor to call the servo driver
//      to allow it to display any additional commands it may have.
//==============================================================================
void PhoenixServoSW::showTerminal(void)
{
    printf(F("O - Enter Servo offset mode\n"));
}

//==============================================================================
// handleTerminal: The terminal monitor will call this to see if the
//     command the user entered was one added by the servo driver.
//==============================================================================
bool PhoenixServoSW::handleTerminal(u8 *psz, u8 bLen)
{
    if ((bLen == 1) && ((*psz == 'o') || (*psz == 'O'))) {
        handleServoOffsets();
    }
}

void PhoenixServoSW::setLegs1500ms(void)
{
    for (u8 i = 0; i < 6 * CONFIG_DOF_PER_LEG; i++) {
        mServoLegs[i].writeMicroseconds(1500 + mServoOffsets[i]);
    }
}

//==============================================================================
//	handleServoOffsets - Find the zero points for each of our servos...
//     	Will use the new servo function to set the actual pwm rate and see
//    	how well that works...
//==============================================================================
static const char *apszLegs[]    = {"RR","RM","RF", "LR", "LM", "LF"};  // Leg Order
static const char *apszLJoints[] = {" Coxa", " Femur", " Tibia", " tArs"}; // which joint on the leg...

void PhoenixServoSW::move(int servo, int val, unsigned int time)
{
    mSGM.start();
	mServoLegs[servo].write(val);
    mSGM.commit(time);
}

void PhoenixServoSW::handleServoOffsets(void)
{
    int data;
    s16 sSN = 0;             // which servo number
    bool fNew = true;    // is this a new servo to work with?
    bool fExit = false;    // when to exit
    int ich;
    s16 sOffset;

    if (!checkVoltage()) {
        // Voltage is low...
        printf(F("Low Voltage: fix or hit $ to abort\n"));
        while (!checkVoltage()) {
            if (Serial.read() == '$')  return;
        }
    }


    // OK lets move all of the servos to their zero point.
    printf(F("Find Servo Zeros.\n$-Exit, +- changes, *-change servo\n"));
    printf(F("    0-5 Chooses a leg, C-Coxa, F-Femur, T-Tibia\n"));

    // don't continue here until we have a valid voltage to work with.
    attachServos();
    setLegs1500ms();

    while(!fExit) {
        if (fNew) {
            sOffset = mServoOffsets[sSN];
            printf(F("Servo: %s-%s (%d)\n"), apszLegs[sSN/CONFIG_DOF_PER_LEG], apszLJoints[sSN%CONFIG_DOF_PER_LEG], sOffset);

            // Now lets wiggle the servo
            mSGM.wait(0xffffff);    // wait for any active servos to finish moving...
            move(sSN, 1500+sOffset+250, 500);

            mSGM.wait(0xffffff);    // wait for any active servos to finish moving...
            move(sSN, 1500+sOffset-250, 500);

            mSGM.wait(0xffffff);    // wait for any active servos to finish moving...
            move(sSN, 1500+sOffset, 500);
            fNew = false;
        }

        //get user entered data
    	data = Serial.read();
        //if data received
    	if (data !=-1)     {
            if (data == '$')
        	    fExit = true;    // not sure how the keypad will map so give NL, CR, LF... all implies exit
            else if ((data == '+') || (data == '-')) {
                if (data == '+')
                    sOffset += 5;        // increment by 5us
        	    else
                    sOffset -= 5;        // increment by 5us

            	printf(F(" %4d\n"), sOffset);
                mServoOffsets[sSN] = sOffset;
                move(sSN, 1500+sOffset, 500);
            } else if ((data >= '0') && (data <= '5')) {
                // direct enter of which servo to change
            	fNew = true;
            	sSN = (sSN % CONFIG_DOF_PER_LEG) + (data - '0')*CONFIG_DOF_PER_LEG;
            } else if ((data == 'c') && (data == 'C')) {
            	fNew = true;
            	sSN = (sSN / CONFIG_DOF_PER_LEG) * CONFIG_DOF_PER_LEG + 0;
            } else if ((data == 'c') && (data == 'C')) {
            	fNew = true;
            	sSN = (sSN / CONFIG_DOF_PER_LEG) * CONFIG_DOF_PER_LEG + 1;
            } else if ((data == 'c') && (data == 'C')) {
                // direct enter of which servo to change
            	fNew = true;
            	sSN = (sSN / CONFIG_DOF_PER_LEG) * CONFIG_DOF_PER_LEG + 2;
            } else if (data == '*') {
                    // direct enter of which servo to change
            	fNew = true;
            	sSN++;
            	if (sSN == 6*CONFIG_DOF_PER_LEG)
                    sSN = 0;
            }
        }
    }

    printf(F("Find Servo exit "));
    for (u8 i = 0; i < 6 * CONFIG_DOF_PER_LEG; i++) {
        printf(F(" %4d"), mServoOffsets[i]);
    }

    printf(F("\nSave Changes? Y/N: "));
    //get user entered data
    while (((data = Serial.read()) == -1) || ((data >= 10) && (data <= 15)));
    if ((data == 'Y') || (data == 'y')) {
        // Ok they asked for the data to be saved.  We will store the data with a
        // number of servos (u8)at the start, followed by a u8 for a checksum...followed by our offsets array...
        // Currently we store these values starting at EEPROM address 0. May later change...
        u8 *pb = (u8*)&mServoOffsets;
        u8 bChkSum = 0;  //
        EEPROM.write(0, 6*CONFIG_DOF_PER_LEG);    // Ok lets write out our count of servos
        for (sSN=0; sSN < sizeof(mServoOffsets); sSN++) {
            EEPROM.write(sSN + 2, *pb);
            bChkSum += *pb++;
        }
        // Then write out to address 1 our calculated checksum
        EEPROM.write(1, bChkSum);
    } else {
        loadServosConfig();
    }

    release();
}
#endif  // Terminal monitor

void PhoenixServoSW::processBackground(void)
{
}

//====================================================================
//Project Lynxmotion Phoenix
//
// Servo Driver: This version is setup to use the main processor to
//    drive the servos, using my hacked library ServoEx, which is based
//    on the Servo class.
//====================================================================
#include <EEPROM.h>
#include "ServoEx.h"

//Servo Pin numbers - May be SSC-32 or actual pins on main controller, depending on configuration.
static const u8 TBL_COXA_PIN[] PROGMEM = {
  cRRCoxaPin,  cRMCoxaPin,  cRFCoxaPin,  cLRCoxaPin,  cLMCoxaPin,  cLFCoxaPin};
static const u8 TBL_FEMUR_PIN[] PROGMEM = {
  cRRFemurPin, cRMFemurPin, cRFFemurPin, cLRFemurPin, cLMFemurPin, cLFFemurPin};
static const u8 TBL_TIBIA_PIN[] PROGMEM = {
  cRRTibiaPin, cRMTibiaPin, cRFTibiaPin, cLRTibiaPin, cLMTibiaPin, cLFTibiaPin};
#ifdef CONFIG_4DOF
static const u8 TBL_TARS_PIN[] PROGMEM = {
  cRRTarsPin, cRMTarsPin, cRFTarsPin, cLRTarsPin, cLMTarsPin, cLFTarsPin};
#endif

// 12 64 0 0 A1 FF 56 FF 0 0 92 FF 0 0 0 0
static const s16 TBL_LEGS_OFFSET[] PROGMEM = {
    0,  -95, -170,
    0, -110,    0,
    0,  -90,  -35,
    0,   15, -140,
    0, -130,    0,
    0, -115,  -45
};


//=============================================================================
// Global - Local to this file only...
//=============================================================================
#ifdef CONFIG_4DOF
#define NUM_SERVOS_PER_LEG 4
#else
#define NUM_SERVOS_PER_LEG 3
#endif
ServoEx          g_aservoLegs[6 * NUM_SERVOS_PER_LEG];        // Define the servo objects...
s16            g_asLegOffsets[6 * NUM_SERVOS_PER_LEG];
cServoGroupMove  g_cSGM;
bool g_fServosAttached;


//--------------------------------------------------------------------
// Helper function to load the servo offsets from the EEPROM
//--------------------------------------------------------------------
void LoadServosConfig(void) {

  u8 *pb = (u8*)&g_asLegOffsets;
  u8 bChkSum = 0;      //
  int i;

#if 0
  if (EEPROM.read(0) == 6*NUM_SERVOS_PER_LEG) {

#ifdef DEBUG
        DBG_SERIAL.println("Load from EEPROM");
#endif

    for (i=0; i < sizeof(g_asLegOffsets); i++) {
          *pb = EEPROM.read(i+2);
          bChkSum += *pb++;
    }

    // now see if the checksum matches.
    if (bChkSum == EEPROM.read(1))
      return;    // we have valid data
  }
#endif

#ifdef DEBUG
  DBG_SERIAL.println("Load default");
#endif
  // got to here, something not right, set up 0's for servo offsets.
  for (i = 0; i < 6*NUM_SERVOS_PER_LEG; i++) {
//    g_asLegOffsets[i] = 0;
    g_asLegOffsets[i] = (s16)pgm_read_word(&TBL_LEGS_OFFSET[i]);
  }
}


//--------------------------------------------------------------------
//Init
//--------------------------------------------------------------------
void ServoDriver::Init(void) {
  u8 bServoIndex;
  g_fServosAttached = false;  // remember we are not attached. Could simply ask one of our servos...

#ifdef OPT_GPPLAYER
  _fGPEnabled = false;  // starts off assuming that it is not enabled...
  _fGPActive = false;
#endif

  // Need to read in the servo offsets... but for now just init all to 0
  for (bServoIndex = 0; bServoIndex < 6*NUM_SERVOS_PER_LEG; bServoIndex++)
      g_asLegOffsets[bServoIndex] = 0;

  LoadServosConfig();

#ifdef PIN_ANALOG_VOLT
  // If we have a voltage pin, we are doing averaging of voltages over
  // 8 reads, so lets prefill that array...
  for (bServoIndex = 0; bServoIndex < 8; bServoIndex++)
    GetBatteryVoltage();

#endif
}
//--------------------------------------------------------------------
//GetBatteryVoltage - Maybe should try to minimize when this is called
// as it uses the serial port... Maybe only when we are not interpolating
// or if maybe some minimum time has elapsed...
//--------------------------------------------------------------------

#ifdef PIN_ANALOG_VOLT
#ifndef CVADR1
#define CVADR1      30  // VD Resistor 1 - reduced as only need ratio... 30K and 10K
#define CVADR2      10  // VD Resistor 2
#endif

word  g_awVoltages[8]={
  0,0,0,0,0,0,0,0};
word  g_wVoltageSum = 0;
u8  g_iVoltages = 0;

word ServoDriver::GetBatteryVoltage(void) {
  g_iVoltages = (++g_iVoltages)&0x7;  // setup index to our array...
  g_wVoltageSum -= g_awVoltages[g_iVoltages];
  g_awVoltages[g_iVoltages] = analogRead(PIN_ANALOG_VOLT);
  g_wVoltageSum += g_awVoltages[g_iVoltages];

  return ((long)((long)g_wVoltageSum*125*(CVADR1+CVADR2))/(long)(2048*(long)CVADR2));

}
#endif
//--------------------------------------------------------------------
//[GP PLAYER]
//--------------------------------------------------------------------
#ifdef OPT_GPPLAYER

//--------------------------------------------------------------------
//[FIsGPSeqDefined]
//--------------------------------------------------------------------
bool ServoDriver::FIsGPSeqDefined(uint8_t iSeq)
{
  // for now assume that we don't support GP Player
  return false;
}

//--------------------------------------------------------------------
// Setup to start sequence number...
//--------------------------------------------------------------------
void ServoDriver::GPStartSeq(uint8_t iSeq)
{

}

//--------------------------------------------------------------------
//[GP PLAYER]
//--------------------------------------------------------------------
void ServoDriver::GPPlayer(void)
{
}

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
uint8_t ServoDriver::GPNumSteps(void)          // How many steps does the current sequence have
{
  return 0;
}

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
uint8_t ServoDriver::GPCurStep(void)           // Return which step currently on...
{
  return 0xff;
}

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
void ServoDriver::GPSetSpeedMultiplyer(s16 sm)      // Set the Speed multiplier (100 is default)
{
}


#endif // OPT_GPPLAYER


//--------------------------------------------------------------------
// Attach Servos...
//--------------------------------------------------------------------
void AttachServos(void) {
  u8 bServoIndex;
  if (!g_fServosAttached) {
    bServoIndex = 0;
    for ( u8 LegIndex=0; LegIndex < 6; LegIndex++) {
      // BUGBUG:: will probably need to add additional stuff here to get the servo offsets
      // and calculate min/max to use...
      g_aservoLegs[bServoIndex++].attach(pgm_read_byte(&TBL_COXA_PIN[LegIndex]));
      g_aservoLegs[bServoIndex++].attach(pgm_read_byte(&TBL_FEMUR_PIN[LegIndex]));
      g_aservoLegs[bServoIndex++].attach(pgm_read_byte(&TBL_TIBIA_PIN[LegIndex]));
#ifdef CONFIG_4DOF
      g_aservoLegs[bServoIndex++].attach(pgm_read_byte(&TBL_TARS_PIN[LegIndex]));
#endif
    }
    g_fServosAttached = true;
  }
}


//------------------------------------------------------------------------------------------
//[BeginServoUpdate] Does whatever preperation that is needed to starrt a move of our servos
//------------------------------------------------------------------------------------------
void ServoDriver::BeginServoUpdate(void)    // Start the update
{
    // First lets make sure our servos are attached.
    AttachServos();

    g_cSGM.start();    // tell the group move system we are starting a new move
}

//------------------------------------------------------------------------------------------
//[OutputServoInfoForLeg] Do the output to the SSC-32 for the servos associated with
//         the Leg number passed in.
//------------------------------------------------------------------------------------------
#define cPwmDiv       991  //old 1059;
#define cPFConst      592  //old 650 ; 900*(1000/cPwmDiv)+cPFConst must always be 1500
// A PWM/deg factor of 10,09 give cPwmDiv = 991 and cPFConst = 592
// For a modified 5645 (to 180 deg travel): cPwmDiv = 1500 and cPFConst = 900.
#ifdef CONFIG_4DOF
void ServoDriver::OutputServoInfoForLeg(u8 LegIndex, s16 sCoxaAngle1, s16 sFemurAngle1, s16 sTibiaAngle1, s16 sTarsAngle1)
#else
void ServoDriver::OutputServoInfoForLeg(u8 LegIndex, s16 sCoxaAngle1, s16 sFemurAngle1, s16 sTibiaAngle1)
#endif
{
  word    wCoxaSSCV;        // Coxa value in SSC units
  word    wFemurSSCV;        //
  word    wTibiaSSCV;        //
#ifdef CONFIG_4DOF
  word    wTarsSSCV;        //
#endif
  //Update Right Legs
  g_InputController.AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
  if (LegIndex < 3) {
    wCoxaSSCV = ((long)(sCoxaAngle1 +900))*1000/cPwmDiv+cPFConst;
    wFemurSSCV = ((long)(-sFemurAngle1+900)*1000/cPwmDiv+cPFConst);
    wTibiaSSCV = ((long)(-sTibiaAngle1+900))*1000/cPwmDiv+cPFConst;
#ifdef CONFIG_4DOF
    wTarsSSCV = ((long)(-sTarsAngle1+900))*1000/cPwmDiv+cPFConst;
#endif
  }
  else {
    wCoxaSSCV = ((long)(-sCoxaAngle1 +900))*1000/cPwmDiv+cPFConst;
    wFemurSSCV = ((long)((long)(-sFemurAngle1+900))*1000/cPwmDiv+cPFConst);
    wTibiaSSCV = ((long)(-sTibiaAngle1+900))*1000/cPwmDiv+cPFConst;
#ifdef CONFIG_4DOF
    wTarsSSCV = ((long)(sTarsAngle1+900))*1000/cPwmDiv+cPFConst;
#endif
  }
  // Now lets tell the servos their next  location...
  uint8_t bServoIndex = LegIndex * NUM_SERVOS_PER_LEG;
  g_aservoLegs[bServoIndex].writeMicroseconds(wCoxaSSCV + g_asLegOffsets[bServoIndex]);
  bServoIndex++;
  g_aservoLegs[bServoIndex].writeMicroseconds(wFemurSSCV + g_asLegOffsets[bServoIndex]);
  bServoIndex++;
  g_aservoLegs[bServoIndex].writeMicroseconds(wTibiaSSCV + g_asLegOffsets[bServoIndex]);
#ifdef CONFIG_4DOF
  bServoIndex++;
  g_aservoLegs[bServoIndex].writeMicroseconds(wTarsSSCV + g_asLegOffsets[bServoIndex]);
#endif
}


//--------------------------------------------------------------------
//[CommitServoDriver Updates the positions of the servos - This outputs
//         as much of the command as we can without committing it.  This
//         allows us to once the previous update was completed to quickly
//        get the next command to start
//--------------------------------------------------------------------
void ServoDriver::CommitServoDriver(word wMoveTime)
{
    g_cSGM.commit(wMoveTime);

}

//--------------------------------------------------------------------
//[FREE SERVOS] Frees all the servos
//--------------------------------------------------------------------
void ServoDriver::FreeServos(void)
{
  for (u8 bServoIndex=0; bServoIndex < 6*NUM_SERVOS_PER_LEG; bServoIndex++) {
    // BUGBUG:: will probably need to add additional stuff here to get the servo offsets
    // and calculate min/max to use...
    g_aservoLegs[bServoIndex].detach();
  }
  g_fServosAttached = false;
}

#ifdef OPT_TERMINAL_MONITOR
extern void FindServoOffsets(void);

//==============================================================================
// ShowTerminalCommandList: Allow the Terminal monitor to call the servo driver
//      to allow it to display any additional commands it may have.
//==============================================================================
void ServoDriver::ShowTerminalCommandList(void)
{
#ifdef OPT_FIND_SERVO_OFFSETS
  DBG_SERIAL.println(F("O - Enter Servo offset mode"));
#endif
}

//==============================================================================
// ProcessTerminalCommand: The terminal monitor will call this to see if the
//     command the user entered was one added by the servo driver.
//==============================================================================
bool ServoDriver::ProcessTerminalCommand(u8 *psz, u8 bLen)
{
#ifdef OPT_FIND_SERVO_OFFSETS
  if ((bLen == 1) && ((*psz == 'o') || (*psz == 'O'))) {
    FindServoOffsets();
  }
#endif

}


//=================================================================================================================
#ifdef OPT_FIND_SERVO_OFFSETS
void AllLegServos1500()
{
    for (u8 bServoIndex=0; bServoIndex < 6*NUM_SERVOS_PER_LEG; bServoIndex++) {
        g_aservoLegs[bServoIndex].writeMicroseconds(1500 + g_asLegOffsets[bServoIndex]);
    }
}

//==============================================================================
//	FindServoOffsets - Find the zero points for each of our servos...
//     	Will use the new servo function to set the actual pwm rate and see
//    	how well that works...
//==============================================================================

void FindServoOffsets()
{
    // not clean but...
    static char *apszLegs[] = {"RR","RM","RF", "LR", "LM", "LF"};  // Leg Order
    static char *apszLJoints[] = {" Coxa", " Femur", " Tibia", " tArs"}; // which joint on the leg...
    int data;
    s16 sSN = 0;             // which servo number
    bool fNew = true;    // is this a new servo to work with?
    bool fExit = false;    // when to exit
    int ich;
    s16 sOffset;

    if (checkVoltage()) {
        // Voltage is low...
        Serial.println("Low Voltage: fix or hit $ to abort");
        while (checkVoltage()) {
            if (Serial.read() == '$')  return;
        }
    }


    // OK lets move all of the servos to their zero point.
    Serial.println("Find Servo Zeros.\n$-Exit, +- changes, *-change servo");
    Serial.println("    0-5 Chooses a leg, C-Coxa, F-Femur, T-Tibia");

    // don't continue here until we have a valid voltage to work with.
    AttachServos();
    AllLegServos1500();


    while(!fExit) {
        if (fNew) {
            sOffset = g_asLegOffsets[sSN];
            Serial.print("Servo: ");
        	Serial.print(apszLegs[sSN/NUM_SERVOS_PER_LEG]);
        	Serial.print(apszLJoints[sSN%NUM_SERVOS_PER_LEG]);
        	Serial.print("(");
        	Serial.print(sOffset, DEC);
        	Serial.println(")");

        // Now lets wiggle the servo
            g_cSGM.wait(0xffffff);    // wait for any active servos to finish moving...
            g_aservoLegs[sSN].move(1500+sOffset+250, 500);

            g_cSGM.wait(0xffffff);    // wait for any active servos to finish moving...
            g_aservoLegs[sSN].move(1500+sOffset-250, 500);

            g_cSGM.wait(0xffffff);    // wait for any active servos to finish moving...
            g_aservoLegs[sSN].move(1500+sOffset, 500);
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

    	Serial.print("    ");
    	Serial.println(sOffset, DEC);
        g_asLegOffsets[sSN] = sOffset;
        g_aservoLegs[sSN].move(1500+sOffset, 500);
    } else if ((data >= '0') && (data <= '5')) {
        // direct enter of which servo to change
    	fNew = true;
    	sSN = (sSN % NUM_SERVOS_PER_LEG) + (data - '0')*NUM_SERVOS_PER_LEG;
        } else if ((data == 'c') && (data == 'C')) {
    	fNew = true;
    	sSN = (sSN / NUM_SERVOS_PER_LEG) * NUM_SERVOS_PER_LEG + 0;
        } else if ((data == 'c') && (data == 'C')) {
    	fNew = true;
    	sSN = (sSN / NUM_SERVOS_PER_LEG) * NUM_SERVOS_PER_LEG + 1;
        } else if ((data == 'c') && (data == 'C')) {
        // direct enter of which servo to change
    	fNew = true;
    	sSN = (sSN / NUM_SERVOS_PER_LEG) * NUM_SERVOS_PER_LEG + 2;
        } else if (data == '*') {
            // direct enter of which servo to change
    	fNew = true;
    	sSN++;
    	if (sSN == 6*NUM_SERVOS_PER_LEG)
            sSN = 0;
        }
    }
    }
    Serial.print("Find Servo exit ");
    for (sSN=0; sSN < 6*NUM_SERVOS_PER_LEG; sSN++){
        Serial.print(" ");
        Serial.print(g_asLegOffsets[sSN], DEC);
    }

    Serial.print("\nSave Changes? Y/N: ");

    //get user entered data
    while (((data = Serial.read()) == -1) || ((data >= 10) && (data <= 15)))
    ;

    if ((data == 'Y') || (data == 'y')) {
        // Ok they asked for the data to be saved.  We will store the data with a
        // number of servos (u8)at the start, followed by a u8 for a checksum...followed by our offsets array...
        // Currently we store these values starting at EEPROM address 0. May later change...
    //
        u8 *pb = (u8*)&g_asLegOffsets;
        u8 bChkSum = 0;  //
        EEPROM.write(0, 6*NUM_SERVOS_PER_LEG);    // Ok lets write out our count of servos
        for (sSN=0; sSN < sizeof(g_asLegOffsets); sSN++) {
            EEPROM.write(sSN+2, *pb);
            bChkSum += *pb++;
        }
        // Then write out to address 1 our calculated checksum
        EEPROM.write(1, bChkSum);
    } else {
        LoadServosConfig();
    }

	g_ServoDriver.FreeServos();

}
#endif  // OPT_FIND_SERVO_OFFSETS
#endif  // Terminal monitor


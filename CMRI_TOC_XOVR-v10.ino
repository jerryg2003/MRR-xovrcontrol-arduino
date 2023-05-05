//***WIP: Take out actionDelaySet? 12-26-20

/* Code provided "AS IS" with no warranties! PLEASE DO NOT DISTRIBUTE FURTHER WITHOUT PERMISSION OF AUTHOR
   Distributed subject to license as specified in the Github reposoitory
   Code on Github may be modified or withdrawn at any time

   Copyright (c) 2019. 2020, 2021, 2022, 2023 Jerry Grochow
*/


#define VERSION "SKETCH:  CMRI485_TOC_XOVR_v10 2021-12-18 1620"

/*Sketch to control twin-coil switch machine relays OR servos (10/2021) via Arduino, C/MRI, JMRI sensors and turnouts
  THIS VERSION USED FOR DOUBLE CROSS-OVER (FOUR TURNOUTS) AS FOLLOWS (BUT COULD BE USED FOR INDEPENDENT TOs):
  TWO OPPOSING LEGS ARE ELECTRICALLY LINKED SO THAT ONE PB OR ONE JMRI/CMRI BIT SETS THEM BOTH
  - SET UP FOR TWIN-COIL SWITCH MACHINES CONTROLLING 2 TURNOUTS EACH, VIA RELAYS: ONE TRIGGERS CLOSED AND ONE TRIGGERS THROWN
  - [TWIN-COIL MACHINES ARE OPERATED BY CAPACITATIVE DISCHARGE CIRCUIT: MUST ALLOW TIME (>1.5 SECONDS) AFTER ANY ACTION FOR CAPACITOR TO RECHARGE]
  - SET UP FOR SERVOS WITH 2 SERVOSetup for servos with check on movement completion.

  When JMRI NOT active, TWO BUTTONS MUST BE PRESSED TO CHANGE THE DOUBLE XOVR FROM STRAIGHT THROUGH TO CROSS-OVER
  When JMRI active, JMRI/CMRI WILL SEND CHANGED BITS. PB PUSH WILL CAUSE JMRI ROUTE TO BE INVOKED TO FLIP BOTH SETS OF TO's

  Jerry Grochow 2018-11-26
  2019-07-06 v7 change CMRI to bool
  2019-07-06 V7b change when prev=cur, just return if less than delaywait.
  2019-12-25 v8  recorganize code into subroutines
  2019-12-27 v9  add JMRI route control upon PB press when JMRI active; reorganize CheckPBState
  2020-10-23 v9a change pbRouteDelay to 1000
  2021-10-01 v10 allow either servos or switch machine relays
  2021-12-18     clean up comments, constants
*/

//*** REQUIRES JMRI PANEL 26A OR LATER ***

const bool NOJMRI = false;   //true = ignore JMRI; false = wait for JMRI
const bool DEBUG  = false;   //Print statements
const bool DEBUGC = false;   //CMRI Debug bits
const bool DEBUGT = false;   //Debug timing and delays

/*
  Twin-coil switch machine use two Arduino pins for the two accessory relays
  Arduino pins for relays specified in TOCPin[] (two relays control each set of turnouts)
  Servos use one arduino pin each, but two turnouts are controlled together

  ___TO1____    ___TO2____
            \  /
             \/
             /\
  ___TO2____/  \___TO1____
  
  In twin-coil set up, two turnouts are ganged together for control by relays (pins 0/1 and 2/3)
  In servo set up, each of the four turnouts is controlled individually (pins 0/1 for TO1 and 2/3 for TO2)
  ** Each pair of turnouts (TO1 and TO2) must be of the same type, either twin-coil or servo. **

  Arduino pin for PB specified in PBPin[] (will reverse TO state)
  Arduino pins for track directional LEDs specified in routeLED[]
** NOTE: JMRI starts numbering C/MRI bits with 1 so C/MRI bit 0 in Arduino code is 1 in JMRI table
  C/MRI output bit 47 from JMRI tells arduino to go active [when JMRI inactive, still allow PB control of TOs]
  C/MRI input bit 23 from arduino handshakes to JMRI that arduino is active
  C/MRI output bit 0 controls Turnout 1 via TOC relays 0 and 1; input bit 1 is used for feedback to JMRI
  C/MRI output bit 2 controls Turnout 2 via TOC relays 2 and 3; input bit 3 is used for feedback to JMRI, etc.
  C/MRI input bit 8 is to set Route 0 (Straight-through) [Second CMRI byte - set "1"]
  C/MRI input bit 9 is to set Route 1 (Crossover)  [Second CMRI byte - set "2"]
** JMRI SETUP:
  Set up JMRI Light CLxx48 to send C/MRI output bit 47 when JMRI goes active
  Set up JMRI Sensor CSxx24 controlled by C/MRI input bit 23 sent by arduino to handshake
  Set up JMRI Turnouts CTxx01, CTxx03, etc. to send C/MRI bit 0, 2, etc.
  ** NOTE: Setup turnouts using 1-bit steady-state CMRI output!
  Set up JMRI Sensors CSxx02, CSxx04, etc. controlled by C/MRI input bit 1, 3, etc.
  Set up JMRI LRoute triggered by Sensor CSxx09, CSxx10, etc. C/MRI input bit 8, 9, etc. [see subroutine SetJMRIRoute]
*/

/*CMRI DEBUG
  Bit 12 (10)  curJMRICommand[0]
  Bit 13 (20)  prevJMRICommand[0]
  Bit 14 (40)  curJMRICommand[1]
  Bit 15 (80)  prevJMRICommand[1]
*/

/*JMRI CMRI monitor shows:
  Byte 0:
  Transmit: 5 (bits 0 and 2) to go from through to crossover
  Receive:  A (bits 1 and 3) to indicate crossover selected
  Transmit: 0 to go from crossover to through
  Receive:  0 to indicate thru selected
  Byte 1:
  Receive:  1 Straightthrough route initiated by PB
  Receive:  2 Crossover route initiated by PB
*/


#include <CMRIFast.h>                  //Simulate CMRI node
#include <Auto485.h>                   //Provide for RS485 communication
#include <MyVarSpeedServo.h>           //Allow servo motor control of crossing gates (added: isMoving)


const int CMRI_ADDR       = 3;         //CMRI Board #3
const int DE_PIN          = 2;         //For RS485 communication
const int JMRI_ACTIVE_BIT = 47;        //JMRI sends this bit
const int MC_ACTIVE_BIT   = 23;        //Communicate that MC board active on this CMRI bit i/o
const int JMRI_HB_BIT     = 22;        //Heartbit bit for JMRI

const bool CLOSED = false;             //Bits received from JMRI for turnout control
const bool THROWN = true;              //For turnout control

const bool SRVOCTRL = true;            //Servo controlled turnouts
const bool TWINCOIL = false;           //Twin-coil controlled turnouts

//****** Values for curAction **************
const int PBError    = -2;
const int PBHeld     = -1;
const int NoAction   =  0;
const int PBAction   =  1;
const int JMRIAction =  2;
const int PBRouteAct =  3;

//***** Timings *******************************
const int pulseDuration    = 250;      //Pulse the turnout relay
const int shortWait        = 12;       //Wait for turnout to throw: too long and response to JMRI transmission will be slow
const int actionDelayDur   = 2500;     //Time to wait between actions for capacitor to recharge (tested works at 3000)
const int pbDelayTime      = 1500;     //Time to lock out PB press
const int routeDelayTime   = 2000;     //Time to reset route to JMRI (less than pbDelayTime
const int pbResetTimeDelay = 6000;     //Time to hold PB for arduino reset

//****** Arduino pins *************************
int TOCPin[]    = {3, 4, 5, 6};        //Pins to which relays or servos are attached
//TEST int TOCPin[]    = {5, 6, 7, 8};             //Pins to which relays or servos are attached
//EACH two TOCPins must be of the same type: either a relay for twin-coil switch machines, or two individual servos
//Servos require a single pin but two separate TOs, must be PWM pins
bool toPairType[]  = {TWINCOIL, TWINCOIL}; //true = pair is servos; false = twin-coil relay
int PBPin[]        = {7, 9};           //Pushbutton to reverse state of each set of turnouts  (two PBs to get full crossover)
//TEST int PBPin[]        = {12, 0};     //Pushbutton to reverse state of each set of turnouts  (two PBs to get full crossover)
int routeLED[]  = {14, 18, 16, 18};    //One LED assumed to be for each leg of turnout (main [CLOSED] and diverging [THROWN])
//TEST int routeLED[]  = {9, 13, 10, 13}; //One LED assumed to be for each leg of turnout (main [CLOSED] and diverging [THROWN])
const int numTO = 2;                   //Number of turnouts being controlled (number of sets of relays]

//******* Servo positions *********************
int TOCClosedPos[]  = { 0,  0,  0,  0};     //Servo position for CLOSED (0 for twin-coil machines)
int TOCThrownPos[]  = { 1,  1,  1,  1};     //Servo position for THROWN (1 for twin-coil machines)
int curTOCPos[]     = { 0,  0,  0,  0};
uint8_t servoSpeed  = 23;                   //Variable speed servo

//****** State variables **********************
bool JMRIPanelAvail    = false;             //Set if JMRI is turned on and ready
bool prevJMRICommand[] = {CLOSED, CLOSED};  //Will contain info from JMRI
bool curJMRICommand[]  = {CLOSED, CLOSED};
int  prevPBState[]     = { -1, -1};         //Initialize state of each PB
int  curPBState[]      = { -1, -1};
bool pbPush[]          = {false, false};    //Set when PB pushed
bool anyPB             = false;             //Declared here to use in subroutines
//variable declared in loop:  anyChange=false; to keep track of any change in state
bool pbRouteSet        = false;             //To know when to reset JMRI Route info

//****** Time state variables *****************
bool              actionDelaySet   = false;        //[CHECK THIS]Needed to ensure actionDelay only set once at a time
unsigned long int actionDelayUntil = 0;            //Set up to lock out TO change actions for time to recharge capacitor
bool              pbDelaySet       = false;        //Needed to ensure pbDelay only set once at a time
unsigned long int pbDelayUntil     = 0;            //Lock out PB for some time
unsigned long int pbResetTimer[]   = {0, 0};       //Time PB started hold
unsigned long int routeDelayUntil  = 0;            //Time to reset route
int               pulseOn[]        = { -1, -1};    //If switch machine pulse still on and needs to be shut off at some point in time
unsigned long int pulseOffTime[]   = {0, 0, 0, 0}; //One variable for each switch machine coil
unsigned long int curTime          = 0;

//****** DEBUG ********************************
int  loopCount      = 0;

//=================================== Create related objects ===================

MyVarSpeedServo servo[4];                //Set up max number of servos
Auto485 bus(DE_PIN);                     //Initiate communications object
CMRIFast cmri(CMRI_ADDR, 24, 48, bus);   //SMINI = 24 inputs, 48 outputs
//=================================== Setup =====================================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DE_PIN, OUTPUT);

  //TEST CONFIG:
  pinMode(11,   OUTPUT);  digitalWrite (11, LOW);

  for (int i = 0; i < numTO; i++) {
    int ii = i * 2;
    int ij = ii + 1;
    pinMode(TOCPin[ii],   OUTPUT);        digitalWrite (TOCPin[ii], HIGH);  //First relay: HIGH = Unpowered
    pinMode(TOCPin[ij],   OUTPUT);        digitalWrite (TOCPin[ij], HIGH);  //Second relay: HIGH = Unpowered
    pinMode(PBPin[i],     INPUT_PULLUP);                                    //Pushbutton: LOW = pressed
    pinMode(routeLED[ii], OUTPUT);        digitalWrite (routeLED[ii], LOW); //First LED off
    pinMode(routeLED[ij], OUTPUT);        digitalWrite (routeLED[ij], LOW); //Second LED off

    //For convenience on my layout, PB and LEDs plugged into two adjacent arduino pins so second pin set to ground
    pinMode(PBPin[i] + 1,     OUTPUT);    digitalWrite(PBPin[i] + 1, LOW);
    pinMode(routeLED[ii] + 1, OUTPUT);    digitalWrite (routeLED[ii] + 1, LOW);
    pinMode(routeLED[ij] + 1, OUTPUT);    digitalWrite (routeLED[ij] + 1, LOW);
  }

  //****************************SKETCH NAME***********************************
  bus.begin(19200, SERIAL_8N2);                       //MAKE SURE JMRI CONNECTION SET TO THIS SPEED
  bus.println(VERSION);
  //***********************************************************************

  //Flash LED that board is ready
  digitalWrite (LED_BUILTIN, HIGH);
  delay (1500);
  digitalWrite (LED_BUILTIN, LOW);


  //For my layout, initialize Xover to CLOSED (both relays)
  curTime = millis();
  prevJMRICommand[0] = CLOSED;
  curJMRICommand[0]  = CLOSED;
  ChangeTOState(0, 0, 1);                     //Set turnout CLOSED (Block 0, TO 0/1)
  if (toPairType[0] == TWINCOIL)   {          //Check if twin-coil switch machine pulse to shut off
    while (pulseOn[0] == 0 && millis() < pulseOffTime[0]);  //Wait here for pulse duration
    digitalWrite(TOCPin[0], HIGH);            //Turn pulse off (pulseOn set to point to which pin to pulse
    pulseOn[0] = -1;                          //Re-init to -1
  }
  RouteLEDs(0, 0, 1);                         //Turn on CLOSED route LED
  delay(actionDelayDur);                      //Wait for capacitor to recharge

  curTime = millis();
  prevJMRICommand[1] = CLOSED;
  curJMRICommand[1]  = CLOSED;
  ChangeTOState(1, 2, 3);                     //Set turnout CLOSED (Block 1, TO 2/3)
  if (toPairType[1] == TWINCOIL)    {         //Check if twin-coil switch machine pulse to shut off
    while (pulseOn[1] == 2 && millis() < pulseOffTime[2]);  //Wait here for pulse duration
    digitalWrite(TOCPin[2], HIGH);            //Turn pulse off (pulseOn set to point to which pin to pulse
    pulseOn[1] = -1;                          //Re-init to -1
  }
  RouteLEDs(1, 2, 3);                         //Turn on CLOSED route LED

  cmri.set_byte(1, 0);                        //No need for JMRI to set route

  actionDelayUntil = millis() + actionDelayDur; //To allow capacitor to recharge

  bus.println("**Setup complete: " + String(millis()));
}

//============================loop=================================================
void loop() {

  curTime        = millis();                   //Set current time for this loop
  loopCount      ++;                           //Increment for debugging
  bool anyChange = false;                      //Init to capture any change in turnout states
  anyPB          = false;                      //Init to capture any PB push

  cmri.process();                              //Get data from JMRI

  //======================See if JMRI open and available ============================
  JMRIPanelAvail = IsPanelOpen();              //SUBROUTINE

  //====================See if route needs to be reset =====================================
  if (JMRIPanelAvail && curTime > routeDelayUntil  && pbRouteSet) { //Allow time between actions
    if (DEBUG)  bus.println ("245 RouteSetDelay");   //** DEBUG **
    pbRouteSet = false;
    cmri.set_byte(1, 0);                        //JMRI already processed; reset JMRI bits
  }

  //==============If everything settled down, cycle thru all listed turnouts looking for commands/PBs ====
  for (int i = 0; i < numTO; i++)   {

    curTime = millis();                         //Update current time..
    int ii = i * 2;
    int ij = ii + 1;

    //========== Handle TOs in process of moving ================================================
    //========================Shut off pulse for twin-coil switch machine========================
    if (toPairType[i] == TWINCOIL)    {          //false = relays for twin coil; see if pulse still on
      //===================== Check if twin-coil switch machine pulse to shut off =================
      if (pulseOn[i] > -1)         {             //Init to -1, otherwise, will have TOC relay number
        if (DEBUG) bus.println("262 IfPulseOn:  " + String(pulseOffTime[pulseOn[i]]) + " " + String(i) + " " + String(pulseOn[i]));
        if (curTime < pulseOffTime[pulseOn[i]]) { //See if pulse still working and for which relay
          delay(2 * shortWait);
          continue;                              //Go check next turnout
        }
        else        {
          digitalWrite(TOCPin[pulseOn[i]], HIGH); //Turn pulse off (pulseOn set to point to which pin to pulse
          pulseOn[i] = -1;                       //Re-init to -1
        }
      }
    }
    //====================See if any servos still moving and wait =================================
    else       {                                 //true = servo, see if still moving
      for (int j = i * 2; j < 2;  j++)            {  //Need to check both servos in pair
        if (servo[j].attached())            {
          if (servo[j].isMoving(curTOCPos[j]))  {      //Check if still moving to expected position
            if (DEBUG) bus.println ("278 ServoMoving: " + String(i) + " " + String(curTOCPos[i])); //** DEBUG **
            delay (2 * shortWait);
            continue;
          }
          else servo[i].detach();                       //If stopped moving, detach
        }
      }
    }

    //====================See if in action delay (have to do this after moving check above=======================
    if (curTime < actionDelayUntil) {                //ALLOW time between actions, resend current state
      if (DEBUG) bus.println ("289 ActionDelay return");    //** DEBUG **
      delay (3 * shortWait);
      continue;                                     //EXIT
    }
    actionDelaySet = false;                     //Needed later so delay only set once

    if (DEBUGT) bus.println(" 295: " + String(millis()) + " TO: " + String(i));


    //================Set loop variables =============================================
    prevPBState[i]     = curPBState[i];          //Save current to previous PB state
    prevJMRICommand[i] = curJMRICommand[i];      //Save current to previous TO state
    int curAction      = NoAction;               //Will be set later if action

    //================If JMRI panel not open, then only allow PB operation===========
    if (JMRIPanelAvail) {                        //See if JMRI panel available
      curJMRICommand[i] = cmri.get_bit(ii);        //Get state of TurnOut as commanded by JMRI;  otherwise, leave state as is
      if (DEBUGC)              {
        cmri.set_bit(12 + ii, curJMRICommand[i]);
        cmri.set_bit(13 + ii, prevJMRICommand[i]);
      }
      if (curJMRICommand[i] != prevJMRICommand[i]) {
        curAction = JMRIAction;                      //JMRI command received!
      }
    }

    if (curAction == NoAction) {                 //If panel not active or JMRI route not changed, check for PB
      if (curTime > pbDelayUntil)   {               //PB lock out check
        pbDelaySet = false;                          //Clear PB delay set
        curAction = CheckPBAction (i);               //Check PB action [SUBROUTINE];  could change curAction
        if (DEBUG) bus.println("319 After CheckPBAction: " + String(curAction));
      }
      else break;
    }
    
    //curAction may have been changed by CheckPBAction...
    if (curAction == PBHeld) {                     //PB being held down; NO ACTION necessary; exit this turnout loop only
      if (DEBUG)  bus.println ("326 PB held down return");     //**DEBUG
      delay (3 * shortWait);
      continue;                                       //EXIT THIS TURNOUT LOOP ONLY
    }

    else if (curAction == JMRIAction || curAction == PBAction) { //Either JMRI or PB wants change in state (curAction may be changed by CheckPBAction)
      anyChange = true;                            //Set if any change in state
      ChangeTOState (i, ii, ij);                   //Throw TOs {SUBROUTINE}
      if (DEBUG) bus.println("334: " + String(pulseOn[i]));
      RouteLEDs(i, ii, ij);                        //Update lights if change in state (SUBROUTINE)
    }
    else if (curAction == PBError) {
      bus.println ("338 Loop: Error in PB Push routine");
    }
    //NOTE: if curAction = PBRouteAct then JMRI will do the work of setting turnouts in the next go around


    //============================== REPEAT FOR ALL TURNOUTS ======================================

  }

  //==============================Finish ============================================================

}     //END LOOP


//==================================================================================================

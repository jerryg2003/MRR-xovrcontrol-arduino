//***WIP: Take out actionDelaySet? 12-26

/*Sketch to control twin-coil switch machine relays via Arduino, C/MRI, JMRI sensors and turnouts
  SET UP FOR TWIN-COIL SWITCH MACHINES CONTROLLED BY TWO RELAYS: ONE TRIGGERS CLOSED AND THE OTHER TRIGGERS THROWN
  SETUP FOR TWIN-COIL MACHINES THRU CAPACITATIVE DISCHARGE CIRCUIT: MUST ALLOW TIME (>1.5 SECONDS) AFTER ANY ACTION FOR CAPACITOR TO CHARGE

  THIS VERSION USED FOR DOUBLE CROSS-OVER (FOUR SWITCH MACHINES) AS FOLLOWS (BUT COULD BE USED FOR INDEPENDENT TOs):
  TWO OPPOSING LEGS ARE ELECTRICALLY LINKED SO THAT ONE PB OR ONE JMRI/CMRI BIT SETS THEM BOTH
  When JMRI NOT active, TWO BUTTONS MUST BE PRESSED TO CHANGE THE DOUBLE XOVR FROM STRAIGHT THROUGH TO CROSS-OVER
  When JMRI active, JMRI/CMRI WILL SEND CHANGED BITS. PB PUSH WILL CAUSE JMRI ROUTE TO BE INVOKED TO FLIP BOTH SETS OF TO's

  Jerry Grochow 2018-11-26
  2019-07-06 v7 change CMRI to bool
  2019-07-06 V7b change when prev=cur, just return if less than delaywait.
  2019-12-25 v8 recorganize code into subroutines
  2019-12-27 v9 add JMRI route control upon PB press when JMRI active; reorganize CheckPBState
*/

/*Twin-coil switch machine use three Arduino pins, two for the accessory relays and one for the pushbutton
  Arduino pins for relays specified in TOCPin[] (two relays control each turnout)
  Arduino pin for PB specified in PB[] (will reverse TO state)
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


#include <CMRI.h>
#include <Auto485.h>                        //Allow RS485 communication

const int CMRI_ADDR = 3;                    //CMRI Board #3
const int DE_PIN = 2;                       //For RS485 communication

const bool CLOSED = false;                  //Bits received from JMRI for turnout control
const bool THROWN = true;                   //For turnout control

//****** Values for curAction **************
const int PBError    = -2;
const int PBHeld     = -1;
const int NoAction   =  0;
const int PBAction   =  1;
const int JMRIAction =  2;
const int PBRouteAct =  3;

//***** Timings *******************************
const int pulseTime       = 250;            //Pulse the turnout relay
const int waitTime        = 12;             //Wait for turnout to throw: too long and response to JMRI transmission will be slow
const int actionDelayTime = 2500;           //Time to wait between actions for capacitor to recharge (tested works at 3000)
const int PBDelayTime     = 1500;           //Time to lock out PB press
const int routeDelayTime  = 500;            //Time to reset route to JMRI

//****** Arduino pins *************************
int TOCPin[]    = {3, 4, 5, 6};             //Pins to which relays are attached;  two relays for each twin-coil switch machine
int PB[]        = {7, 9};                   //Pushbutton to reverse state of each set of turnouts  (two PBs to get full crossover)
int routeLED[]  = {14, 18, 16, 18};         //One LED assumed to be for each leg of turnout (main [CLOSED] and diverging [THROWN])
const int numTO = 2;                        //Number of turnouts being controlled (number of sets of relays]

//****** State variables **********************
bool JMRIPanelAvail    = false;             //Set if JMRI is turned on and ready
int  curAction         = NoAction;          //Determined by JMRI and PB analysis
bool prevJMRICommand[] = { false, false};   //Will contain info from JMRI
bool curJMRICommand[]  = { false, false};
int  prevPBState[]     = { -1, -1};         //Initialize state of each PB
int  curPBState[]      = { -1, -1};
bool pbPush[]          = {false, false};    //Set when PB pushed
bool anyPB             = false;             //Declared here to use in subroutines
//variable declared in loop:  anyChange=false; to keep track of any change in state
bool pbRouteSet        = false;             //To know when to reset JMRI Route info

//****** Time state variables *****************
bool actionDelaySet           = false;      //[CHECK THIS]Needed to ensure actionDelay only set once at a time
unsigned long int actionDelay = 0;          //Set up to lock out TO change actions for time to recharge capacitor
bool pbDelaySet               = false;      //Needed to ensure pbDelay only set once at a time
unsigned long int pbDelay     = 0;          //Lock out PB for some time
unsigned long int routeDelay  = 0;          //Time to reset route
unsigned long int curTime     = 0;

//****** DEBUG ********************************
int  loopCount      = 0;

//=================================== Create related objects ===================

Auto485 bus(DE_PIN);                        //Initializes RS485 communication
CMRI cmri(CMRI_ADDR, 24, 48, bus);          //SMINI = 24 CMRI inputs to JMRI, 48 CMRI outputs from JMRI

//=================================== Setup =====================================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  for (int i = 0; i < numTO; i++) {
    int ii = i * 2;
    int ij = ii + 1;
    pinMode(TOCPin[ii],   OUTPUT);        digitalWrite (TOCPin[ii], HIGH);  //First relay: HIGH = Unpowered
    pinMode(TOCPin[ij],   OUTPUT);        digitalWrite (TOCPin[ij], HIGH);  //Second relay: HIGH = Unpowered
    pinMode(PB[i],        INPUT_PULLUP);                                    //Pushbutton: LOW = pressed
    pinMode(routeLED[ii], OUTPUT);        digitalWrite (routeLED[ii], LOW); //First LED off
    pinMode(routeLED[ij], OUTPUT);        digitalWrite (routeLED[ij], LOW); //Second LED off

    //For convenience on my layout, PB and LEDs plugged into two adjacent arduino pins so second pin set to ground
    pinMode(PB[i] + 1,        OUTPUT);    digitalWrite(PB[i] + 1, LOW);
    pinMode(routeLED[ii] + 1, OUTPUT);    digitalWrite (routeLED[ii] + 1, LOW);
    pinMode(routeLED[ij] + 1, OUTPUT);    digitalWrite (routeLED[ij] + 1, LOW);
  }

  //****************************SKETCH NAME***********************************
  bus.begin(19200, SERIAL_8N2);                       //MAKE SURE JMRI CONNECTION SET TO THIS SPEED
  bus.println("SKETCH:  CMRI485_TOC_XOVR_v9 2019-12-29 1300");
  //***********************************************************************

  //Flash LED that board is ready
  digitalWrite (LED_BUILTIN, HIGH);
  delay (1500);
  digitalWrite (LED_BUILTIN, LOW);

  //For my layout, initialize Xover to CLOSED (both relays)
  //  cmri.set_bit(1, false);                   //CLOSED feedback to JMRI sensor:  Turnout 1 (relays 0 and 1) feedback is on C/MRI bit 1
  prevJMRICommand [0] = CLOSED;
  curJMRICommand [0]  = CLOSED;
  ChangeTOState(0, 0, 1);                       //Set turnout CLOSED
  RouteLEDs(0, 0, 1);                           //Turn on CLOSED route LED
  delay(actionDelayTime);                       //Wait for capacitor to recharge
  //  cmri.set_bit(3, false);                   //CLOSED feedback to JMRI sensor:  Turnout 2 (relays 2 and 3) feedback is on C/MRI bit 3
  prevJMRICommand [1] = CLOSED;
  curJMRICommand [1]  = CLOSED;
  ChangeTOState(1, 2, 3);                       //Set turnout CLOSED
  RouteLEDs(1, 2, 3);                           //Turn on CLOSED route LED
  
  cmri.set_byte(1, 0);                          //No need for JMRI to set route

  actionDelay = millis() + actionDelayTime;     //To allow capacitor to recharge

}

//============================loop=================================================
void loop() {

  curTime        = millis();                   //Set current time for this loop
  bool anyChange = false;                      //Init to capture any change in turnout states
  anyPB          = false;                      //Init to capture any PB push

  cmri.process();                              //Get data from JMRI
  //======================See if JMRI open and available ============================
  JMRIPanelAvail = IsPanelOpen();              //SUBROUTINE

   //====================See if route needs to be reset =====================================
    if (JMRIPanelAvail && routeDelay < curTime) { //ALLOW time between actions, resend current state
      //bus.println ("RouteSetDelay170");       //** DEBUG **
      cmri.set_byte(1, 0);                      //JMRI already processed; reset JMRI bits
    }


  //==========================Cycle thru all listed turnouts=======================
  for (int i = 0; i < numTO; i++)   {

    //====================See if in action delay =====================================
    if (actionDelay > curTime) {                //ALLOW time between actions, resend current state
      //bus.println ("ActionDelay180 return");    //** DEBUG **
      delay (3 * waitTime);
      return;                                     //EXIT
    }
    actionDelaySet = false;                     //Needed later so delay only set once

 
    //================Set loop variables =============================================
    prevPBState[i]     = curPBState[i];          //Save current to previous PB state
    prevJMRICommand[i] = curJMRICommand[i];      //Save current to previous TO state
    curAction          = NoAction;               //Will be set later if action
    loopCount          = loopCount + 1;          //DEBUG

    int ii = i * 2;
    int ij = ii + 1;

    //================If JMRI panel not open, then only allow PB operation===========
    if (JMRIPanelAvail) {                        //See if JMRI panel available
      curJMRICommand[i] = cmri.get_bit(ii);        //Get state of TurnOut as commanded by JMRI;  otherwise, leave state as is
      if (curJMRICommand[i] != prevJMRICommand[i]) {
        curAction = JMRIAction;                      //JMRI command received!
      }
    }
    if (curAction == NoAction) {                 //If panel not active or JMRI route not changed, check for PB
      if (pbDelay >= curTime) {                    //PB lock out check
        break;
      }
      else {                                       //If PB action allowed
        pbDelaySet = false;                          //Clear PB delay set
        curAction = CheckPBAction (i);               //Check PB action [SUBROUTINE];  could change curAction
      }
    }
    if (curAction == PBHeld) {                     //PB being held down; NO ACTION necessary; exit this turnout loop only
      //bus.println ("PB held down return 214");     //**DEBUG
      delay (3 * waitTime);
      break;                                       //EXIT THIS TURNOUT LOOP ONLY
    }
    if (curAction == JMRIAction || curAction == PBAction) { //Either JMRI or PB wants change in state (curAction may be changed by CheckPBAction)
      anyChange = true;                            //Set if any change in state
      ChangeTOState (i, ii, ij);                   //Throw TOs {SUBROUTINE}
      RouteLEDs(i, ii, ij);                        //Update lights if change in state (SUBROUTINE)
    }
    if (curAction == PBError) {
      bus.println ("Loop 234: Error in PB Push routine");
    }
    //NOTE: if curAction = PBRouteAct then JMRI will do the work of setting turnouts in the next go around


    //============================== REPEAT FOR ALL TURNOUTS ======================================

  }      //END TURNOUT LOOP

  //==============================Finish ============================================================

  if (pbRouteSet)  {                                //Set delay to clear route to JMRI
    pbRouteSet = false;
    routeDelay = curTime + routeDelayTime;
  }
  return;

}

//==================================================================================================
//===================== SUBROUTINES ================================================================
//==================================================================================================

//===================== IS JMRI PANEL OPEN ===================================
bool IsPanelOpen ()   {

  //***DEBUG
  //  bus.println("JMRIPanelAvail254: " + String(curTime) + " " + String(cmri.get_bit(47)));
  // return(true);

  if (cmri.get_bit(47) == true) {
    cmri.set_bit(23, true);
    return (true);
  }
  else {
    cmri.set_bit(23, false);
    digitalWrite (LED_BUILTIN, HIGH);         //Fiddle
    delay (100);
    digitalWrite (LED_BUILTIN, LOW);
    delay (100);
    return (false);
  }
}

//======================== Check for push button =========================
int CheckPBAction (int k)   {         //k= turnout

  //NOTE: PB action is different if JMRI available or not.  
  //      If JMRI not available, then each PB controls a single turnout (or, in case of double Xover, a set of two).
  //      If JMRI is available, then ANY PB sets JMRI route and JMRI does the rest.

  //**DEBUG
  //  bus.println ("CheckPBState275: " + String(k));

  curPBState[k] = digitalRead(PB[k]);         //Get state of PB from Arduino
  if (curPBState[k] == LOW)  {                //LOW = pressed
    delay (2 * waitTime);                     //Make sure not just noise
    curPBState[k] = digitalRead(PB[k]);       //Reread
  }

  //Action table-->  curPBState:   pbPush:
  //          (0)     LOW(pushed)   false    --> Just pushed; DO THE WORK: toggle turnout or set route
  //          (1)     LOW(pushed)   true     --> Still pushed (being held down); exit
  //          (2)     HIGH          false    --> PB idle
  //          (3)     HIGH          true     --> PB just released; reset pbPush; wait before accepting more button presses

  //Create single number to simplify code that follows
  int comboPBState = (int) curPBState[k] * 2 + (int) pbPush [k] * 1;

  switch (comboPBState)  {
    case 0:                                     //PB just pressed; TAKE ACTION BASED ON WHETHER JMRI AVAILABLE
      pbPush [k] = true;
      anyPB = true;
      if (JMRIPanelAvail)  {                           //If JMRI avail, PB sets/toggles Route
        pbRouteSet = SetJMRIRoute();
        return (PBRouteAct);
      }
      else {                                      //If JMRI not avail, then PB toggles individual turnout
        if (prevJMRICommand[k] == THROWN) {         //Push Button toggles state
          curJMRICommand[k] = CLOSED;
        }
        else {
          curJMRICommand[k] = THROWN;
        }
        return (PBAction);                        //Return that action based on PB push
      }
    case 1:                                     //PB still being pushed
      //***DEBUG
      //bus.println ("PB held down return: 311"); //** DEBUG **
      return (PBHeld);                                //PB being held down
    case 2:                                     //PB idle
      return (NoAction);
    case 3:                                     //PB HIGH and PbPush=true means button was just released
      pbPush [k] =  false;                        //Reset: done with button push
      if (pbDelaySet == false) {                  //Set delay
        pbDelaySet = true;
        pbDelay = curTime + PBDelayTime;            //No more input for 2 seconds
      }
      return (NoAction);
    default:
      return (PBError);
  }
}

//======================= Change TO State ===============================
void ChangeTOState (int k, int m, int n)  {           //k=turnout, m = k*2, n= k*2+

  //**DEBUG
  //  bus.println ("ChangeTOState331: " + String(k));

  if (curJMRICommand[k] != THROWN)  {           //JMRI state may be CLOSED or UNKNOWNSTATE (**FEATURE NOT AVAIL.)
    digitalWrite (TOCPin[m], LOW);                //Pulse the turnout relay to set CLOSED
    delay (pulseTime);                            //Pulse only
    digitalWrite (TOCPin[m], HIGH);
    if (JMRIPanelAvail) {                         //If JMRI available, set feedback bits
      cmri.set_bit(n, false);                       //CLOSED feedback to JMRI sensor:  Turnout i feedback is on C/MRI bit i*2+1
    }
  }
  else {                                        //JMRI says state should be THROWN
    digitalWrite (TOCPin[n], LOW);                //Pulse the turnout relay to set THROWN
    delay (pulseTime);                            //Pulse only
    digitalWrite (TOCPin[n], HIGH);
    if (JMRIPanelAvail) {                         //If JMRI available, set feedback bits
      cmri.set_bit(n, true);                        //THROWN feedback to JMRI sensor:  Turnout i feedback is on C/MRI bit i*2+1
    }
  }

  if (actionDelaySet == false)  {               //In case code gets thru to here...only allow one delay set at a time
    actionDelay = curTime + actionDelayTime;      //Set up to wait for capacitor to recharge
    actionDelaySet = true;
  }
}

//======================= Set JMRI route ================================
bool SetJMRIRoute ()     {              //Sets route for Crossover or Straightthrough

  switch (cmri.get_byte(1))  {                //Check previous route setting
    case 0:                                     //If not previously set, check current JMRI commands
      if (curJMRICommand[0] || curJMRICommand [1])  {
        cmri.set_byte(1, 1);                      //If any command is set to crossover, then make route straight through
      }
      else  {
        cmri.set_byte(1, 2);                      //If commands are set to straight thru, then make route crossover
      }
      return (true);
    case 1:                                     //Toggle route from straight through
      cmri.set_byte(1, 2);                        //Crossover route
      return (true);
    case 2:                                     //Toggle route from crossover
      cmri.set_byte(1, 1);                        //Straight through route
      return (true);
    default:
      cmri.set_byte (1, 255);                   //ERROR
      return (false);
  }
  return (false);                               //Never gets here
}

//=======================  Turn on status lights  ========================
void RouteLEDs (int k, int m, int n)  {                //k=turnout; m=k*2;  n=k*2+1

  if (curJMRICommand[k] == CLOSED) {
    digitalWrite (routeLED[m], HIGH);            //Turn on CLOSED route LED
    digitalWrite(routeLED[n], LOW);
  }
  else {
    digitalWrite(routeLED[m], LOW);              //Turn on THROWN route LED
    digitalWrite(routeLED[n], HIGH);
  }

}

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=

//**DEBUG
//bus.println ("Init0: " + String(loopCount) + "  " + String(prevJMRICommand[0]) + String(curJMRICommand[0]) + " " + String(prevPBState[0]) + String(curPBState[0]));
//bus.println ("Init1: " + String(loopCount) + "  " + String(prevJMRICommand[1]) + String(curJMRICommand[1]) + " " + String(prevPBState[1]) + String(curPBState[1]));

  //**DEBUG
  //  bus.println("Loop153: " + String (curTime) + " lpCt: " + String(loopCount) + " actD: " + String(actionDelay) + " PBD: " + String(pbDelay));


    //**DEBUG
    //    bus.println("Loop184: " + String (curTime) + " lpCt: " + String(loopCount) + " " + String(i) + " curAct: " + String(curAction));


    //**DEBUG
    //      bus.println ("Loop223: " +  String (curTime) + " lpCt: " + String(loopCount) + " " + String(i) + " JMRI c/p: " + String(curJMRICommand[i]) + String(prevJMRICommand[i]) + " PB c/p/pu: " + String(curPBState[i]) + String(prevPBState[i]) + " " + String(pbPush[i]));


    //**DEBUG
    //   bus.println ("Loop238: " +  String (curTime) + " lpCt: " + String(loopCount) + " " + String(i) + " JMRI c/p: " + String (curJMRICommand[i]) + String(prevJMRICommand[i]) + " PB c/p/pu: "  + String(curPBState[i]) + String(prevPBState[i]) + " " + String(pbPush[i]) + " anyCng: " + String(anyChange));

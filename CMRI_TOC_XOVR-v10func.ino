/*Sketch to control twin-coil switch machine relays OR servos (10/20210) via Arduino, C/MRI, JMRI sensors and turnouts
  SET UP FOR TWIN-COIL SWITCH MACHINES CONTROLLED BY TWO RELAYS: ONE TRIGGERS CLOSED AND THE OTHER TRIGGERS THROWN
  SETUP FOR TWIN-COIL MACHINES THRU CAPACITATIVE DISCHARGE CIRCUIT: MUST ALLOW TIME (>1.5 SECONDS) AFTER ANY ACTION FOR CAPACITOR TO CHARGE
  Setup for servos with check on movement completion.
*/

/* Code provided "AS IS" with no warranties! PLEASE DO NOT DISTRIBUTE FURTHER WITHOUT PERMISSION OF AUTHOR
   Distributed subject to license as specified in the Github reposoitory
   Code on Github may be modified or withdrawn at any time

   Copyright (c) 2019, 2020, 2021, 2022, 2023 Jerry Grochow
*/


//===================== FUNCTIONS ================================================================
//==================================================================================================

//========================= ARDUINO RESET FUNCTION ======================================

void(* ResetFunc) (void) = 0;//declare reset function at address 0


//======================== Check for push button =========================
int CheckPBAction (int k)   {         //k= turnout

  //NOTE: PB action is different if JMRI available or not.
  //      If JMRI not available, then each PB controls a single turnout (or, in case of double Xover, a set of two).
  //      If JMRI is available, then ANY PB sets JMRI route and JMRI does the rest.

  if (DEBUG)  bus.println ("24s CheckPBState: " + String(k));

  if (PBPin[k] == 0)  return (NoAction);        //No PB for this turnout, therefore not action

  curPBState[k] = digitalRead(PBPin[k]);         //Get state of PB from Arduino
  if (curPBState[k] == LOW)  {                //LOW = pressed
    delay (2 * shortWait);                     //Make sure not just noise
    curPBState[k] = digitalRead(PBPin[k]);       //Reread
  }

  //Action table-->  curPBState:   pbPush:
  //          (0)     LOW(pushed)   false    --> Just pushed; DO THE WORK: toggle turnout or set route
  //          (1)     LOW(pushed)   true     --> Still pushed (being held down); exit
  //          (2)     HIGH          false    --> PB idle
  //          (3)     HIGH          true     --> PB just released; reset pbPush; wait before accepting more button presses

  //Create single number to simplify code that follows
  int comboPBState = (int) curPBState[k] * 2 + (int) pbPush[k] * 1;

  switch (comboPBState)  {
    case 0:                                     //PB just pressed; TAKE ACTION BASED ON WHETHER JMRI AVAILABLE
      if (DEBUG)  bus.println ("45s PB just pushed"); //** DEBUG **
      pbPush[k] = true;
      anyPB = true;
      pbResetTimer[k] = curTime + pbResetTimeDelay;        //Start timer for reset
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
      if (DEBUG)  bus.println ("62s PB held down return"); //** DEBUG **
      //*** WARNING: ARDUINO RESET FUNCTION FOLLOWS ***
      if (pbResetTimer[k] != 0 && curTime > pbResetTimer[k]) {              //Very long PB push and hold
        bus.println("65s PB HELP DOWN 6 SECONDS / ARDUINO RESET");
        digitalWrite(LED_BUILTIN, HIGH);
        delay(2000);
        digitalWrite(LED_BUILTIN, LOW);
        ResetFunc();                               //RESET FUNCTION RESET ARDUINO
      }
      //*** WARNING: ARDUINO RESET FUNCTION ABOVE ***
      else                                        //Not very long PB push and hold
        return (PBHeld);                          //PB being held down
    case 2:                                     //PB idle
      return (NoAction);
    case 3:                                     //PB HIGH and PbPush=true means button was just released
      if (DEBUG)  bus.println ("77s PB just released"); //** DEBUG **
      pbPush[k] =  false;                        //Reset: done with button push
      pbResetTimer[k] = 0;
      if (pbDelaySet == false) {                  //Set delay
        pbDelaySet = true;
        pbDelayUntil = curTime + pbDelayTime;            //No more input for 2 seconds
      }
      return (NoAction);
    default:
      return (PBError);
  }
}


//======================= Change TO State ===============================
void ChangeTOState (int k, int m, int n)  {         //k=turnout, m = k*2, n= k*2+1

  if (DEBUGC) cmri.set_bit (20, loopCount % 2);             //DEBUG

  if (DEBUG)  bus.println ("91s ChangeTOState: " + String(k));

  if (curJMRICommand[k] == THROWN)  {               //JMRI says state should be THROWN so THROW it
    if (toPairType[k] == SRVOCTRL)   {              //TOC Type is SERVO so both servos must be moved individually
      curTOCPos[m] = ThrowServo(m, TOCThrownPos[m]);
      curTOCPos[n] = ThrowServo(n, TOCThrownPos[n]);
    }
    else    {                                       //TOC Type is Twin-coil
      digitalWrite (TOCPin[n], LOW);                //Pulse the turnout relay to set THROWN
      //Pulse set on only**** Set off in mainline
      //digitalWrite (TOCPin[n], HIGH);
      pulseOn[k] = n;                               //Relay coil n has been turned on
      pulseOffTime[n] = curTime + pulseDuration;    //Time to turn this relay coil off
      curTOCPos[k] = TOCThrownPos[k];
    }
    if (JMRIPanelAvail) {                           //If JMRI available and TO thrown, set feedback bits
      cmri.set_bit(n, true);                        //Turnout i feedback is on C/MRI bit i*2+1
    }
  }
  else {                                            //JMRI command not THROWN, so make it CLOSED
    if (toPairType[k] == SRVOCTRL)   {              //TOC Type is SERVO so both servos must be moved individually
      curTOCPos[m] = ThrowServo(m, TOCClosedPos[m]);
      curTOCPos[n] = ThrowServo(n, TOCClosedPos[n]);
    }
    else    {                                       //TOC Type is Twin-coil
      digitalWrite (TOCPin[m], LOW);                //Pulse the turnout relay to set CLOSED
      //Pulse set on only**** Set off in mainline
      //digitalWrite (TOCPin[m], HIGH);
      pulseOn[k] = m;                               //Relay coil m has been turned on
      pulseOffTime[m] = curTime + pulseDuration;    //Time to turn this relay coil off
      curTOCPos[k] = TOCClosedPos[k];
    }
    if (JMRIPanelAvail) {                          //If JMRI available and TO closed, set feedback bits
      cmri.set_bit(n, false);                      //Turnout i feedback is on C/MRI bit i*2+1
    }
  }

  if (actionDelaySet == false)  {               //In case code gets thru to here...only allow one delay set at a time
    actionDelayUntil = curTime + actionDelayDur;      //Set up to wait for capacitor to recharge
    actionDelaySet = true;
  }

  if (DEBUG)  bus.println ("133s ChangeTOState exit: pulseOn: " + String(pulseOn[k]) + " actionDelaySet: " + String(actionDelaySet));


}


//================= Throw turnout SERVOS ========================================
int ThrowServo(int k, int pos) {         //k=servo number; pos=position to move to

  //Check that gate has finished prior move and then detach
  if (servo[k].isMoving(curTOCPos[k]))   return (curTOCPos[k]);
  if (servo[k].attached())   {
    servo[k].detach();
    delay(shortWait);
  }

  servo[k].attach(TOCPin[k]);                      // attaches the servo to the servo object
  delay(shortWait);
  servo[k].write(pos, servoSpeed, false);  // sets the servo position, don't wait for complete
  delay(shortWait);                             // waits a bit
  return (pos);

}


//======================= Set JMRI route ================================
bool SetJMRIRoute ()     {              //Sets route for Crossover or Straightthrough

  switch (cmri.get_byte(1))  {                //Check previous route setting
    case 0:                                     //If not previously set, check current JMRI commands
      if (curJMRICommand[0] || curJMRICommand[1])  {
        cmri.set_byte(1, 1);                      //If any command is set to crossover, then make route straight through
      }
      else  {
        cmri.set_byte(1, 2);                      //If commands are set to straight thru, then make route crossover
      }
      break;
    case 1:                                     //Toggle route from straight through
      cmri.set_byte(1, 2);                        //Crossover route
      break;
    case 2:                                     //Toggle route from crossover
      cmri.set_byte(1, 1);                        //Straight through route
      break;
    default:
      cmri.set_byte (1, 255);                   //ERROR
      return (false);
  }
  routeDelayUntil = curTime + routeDelayTime;
  return (true);                               //Route set
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


//===================== IS JMRI PANEL OPEN ===================================
bool IsPanelOpen ()   {

  if (NOJMRI)   {
    JMRIPanelAvail = true;
    return (true);
  }

  if (cmri.get_bit(JMRI_ACTIVE_BIT)) {       //See if JMRI bit turned on
    if (!JMRIPanelAvail)   {                 //Just turned on
      JMRIPanelAvail = true;
      cmri.set_bit(MC_ACTIVE_BIT, true);
      //Set heartbeat bit
      long int curTimeSec = curTime / 1000;
      if (curTimeSec % 5 == 0)        {
        if (curTimeSec % 10 == 0) cmri.set_bit(JMRI_HB_BIT, true);
        else cmri.set_bit(JMRI_HB_BIT, false);
      }
    }
    else       {                             //Was already on
      //cmri.set_bit(MC_ACTIVE_BIT, true);     //Unnecessary:  should already be on
    }
    return (true);
  }
  else {                                //Otherwise, JMRI bit is off
    JMRIPanelAvail = false;
    cmri.set_bit(MC_ACTIVE_BIT, false);
    cmri.set_bit(JMRI_HB_BIT, false);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(70);
    loopCount = loopCount + 1;
    if (DEBUG && loopCount % 10 == 0) bus.println("WaitforPanel: " + String(loopCount) + "  " + String(prevJMRICommand[0]) + String(curJMRICommand[0]) + " " + String(prevPBState[0]) + String(curPBState[0]));
    return (false);
  }

}


//the end

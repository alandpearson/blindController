
/* Automatic Roller/Roman Blind controller for JeeNode arduino clones
    Works by assessing reading from LDR and driving blinds
    accordingly. SunDip feature will partially close the blinds if it it too bright
    for longer than SunDipThreshold time and open them again afterwards.
    Configurable via RF
    Different modes of operation possible from manual to fully automated.
    Don't forget to  define which room you are in, and the properties of the room
    below in the hash defines..

    (c) 2018 alandpearson@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.


  PLEASE NOTE : THIS VERSION HAS AN ISSUE WITH THE BMP280 sensor causing
  board to crash.  Therefore, do not #DEFINE BMP280. Still investigating why
  a BMP280 on the SDA/SCL I2C ports cause this behaviour at setup. ADP 1/1/2021
  I'm pretty sure this is no longer true ^ but leaving this note here. ADP Jul 2022

*/

//Version number reported in status packet
#define VERSION 14


//Which room am I in (see blindController.h) ?
//This must be BEFORE blindController.h include
#define ANNEOFFICE

#include <JeeLib.h>
#include <util/crc16.h>
#include <avr/wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "blindController.h"

// RF12 communication settings
#define NETGRP     210      //network group (can be in the range 1-250).
#define RF_freq RF12_433MHZ     //Freq of RF12B can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ.
#define EMONPI_NODEID 5   // ID of raspberry PI with JeeNode receiver attached
#define RF69_COMPAT 0
#define MAX_TX_ATTEMPTS 4

/*Bolt on for wh1050 weather station reception + relay
   You should normally disable this as it is a special case
   for one of my blindControllers doing double duty
    - alandpearson
*/
#ifdef WX_REPEATER
#define WXNODEID 16
#include <RTClib.h>
#include "wh1050.h"
#endif



bool timerRunning = false ;
bool lastActionWasManual = false ;  //if the last action taken was by manual operation (pressing buttons or RF command)

unsigned long timerStartTime = 0 ;
unsigned long timerRunTime = 0 ;
unsigned long lastAction = 0 ;       //last time we made an automatic operation
unsigned long lastStatusPrint = 0 ; //last time we printed status to console
unsigned long lastPosUpdate = 0 ; //last time we went through UpdateBlindPosition function


float blindPosition[2] = {0, 0} ; // Rough % Estimate of Blind Position, based on time in doOpen or doClose (valid after init has run)
float clearBlindPosition[2] = {255, 255} ;  //what write to nvram if we are clearing the blindPosition

unsigned long lastUpdateTime[2] = {0, 0} ; //Last time blindPosition was updated
unsigned long motorRunStart[2] = {0, 0} ;
unsigned long motorRunStop[2] = {0, 0} ;
unsigned long motorRunTime[2] = {0, 0} ;
bool motorRunning[2] = {0, 0} ;

bool initDone = 0 ;      // Intitialisation completed


#ifdef BREADBOARD
//Breadboard ports
PortI2C BLIND (2) ;  // JeeLabs DC Motor Driver attached here
Port LEDLDR (3) ;      //State LED = DIO, LDR = AIO
Port REED (4) ;     //Reed Limit sensors, CloseSensor=DIO, OpenSensor =AIO
Port BUTTONS(1) ;   //Up/down BUTTONS
#else
//Production unit ports
PortI2C BLIND (4) ;  // JeeLabs DC Motor Driver attached here
Port LEDLDR (3) ;      //State LED = DIO, LDR = AIO
Port REED (2) ;     //Reed Limit sensors, CloseSensor=DIO, OpenSensor =AIO
Port BUTTONS(1) ;   //Up/down BUTTONS
#endif



#ifdef DUALBLINDS
const byte numBlinds = 2 ;
#else
const byte numBlinds = 1 ;
#endif

DeviceI2C motor (BLIND, I2C_ADDR);
blindReportPayload report ;
blindConfig config;
blindCalibration calibration;

byte defaultOpMode = 3;     //this is copy of config.opMode on startup (to remember the default opMode as it can be toggled during runtime


//Temperature Sensing DS18B20 definitions
#define ONE_WIRE_BUS 4      // temp sensor connection - Arduino pin4 (Port1, DIO in Jeelib) uses the OPEN switch IO pin
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tSensors(&oneWire);
bool ds18b20Present = false ;       // DS18B20 temp sensor found on OPEN I/O pin ?



static word calcCrc (const void* ptr, byte len) {
  word crc = ~0;
  for (byte i = 0; i < len; ++i)
    crc = _crc16_update(crc, ((const byte*) ptr)[i]);
  return crc;
}

static void loadConfig () {
  // eeprom_read_block(&config, RF12_EEPROM_ADDR, sizeof config);
  // this uses 166 bytes less flash than eeprom_read_block(), no idea why
  Serial.println(F("loading config")) ;
  for (byte i = 0; i < sizeof config; ++ i)
    ((byte*) &config)[i] = eeprom_read_byte(RF12_EEPROM_ADDR + i);
}

static void saveConfig () {
  Serial.println(F("saving config")) ;
  config.crc = calcCrc(&config, sizeof config - 2);
  // eeprom_write_block(&config, RF12_EEPROM_ADDR, sizeof config);
  // this uses 170 bytes less flash than eeprom_write_block(), no idea why
  eeprom_update_byte(RF12_EEPROM_ADDR, ((byte*) &config)[0]);
  for (byte i = 0; i < sizeof config; ++ i)
    eeprom_update_byte(RF12_EEPROM_ADDR + i, ((byte*) &config)[i]);

}


static void loadCalibration() {

  unsigned int startAddress = RF12_EEPROM_ADDR + sizeof config ;
  Serial.println F(("loading calibration data")) ;
  for (byte i = 0; i < sizeof calibration; ++ i) {
    ((byte*) &calibration)[i] = eeprom_read_byte(startAddress + i);
  }
}


static void saveCalibration () {
  // Save calibration data to EEPROM right after config
  unsigned int startAddress = RF12_EEPROM_ADDR ;

  startAddress = startAddress + sizeof config ;
  Serial.println F(("saving calibration data")) ;
  eeprom_update_byte(startAddress, ((byte*) &calibration)[0]);
  for (byte i = 0; i < sizeof calibration; ++ i)
    eeprom_update_byte(startAddress + i, ((byte*) &calibration)[i]);

}

static void loadBlindPos() {

  unsigned int startAddress = RF12_EEPROM_ADDR + sizeof config + sizeof calibration ;

  Serial.println F(("load blindPosition data")) ;
  for (byte i = 0; i < sizeof blindPosition; ++ i) {
    ((byte*) &blindPosition)[i] = eeprom_read_byte(startAddress + i);
  }

}

static void saveBlindPos(byte clearPos) {

  // Save blind position data to EEPROM right after calibration data
  // if clearPos > 0 then write zero for the blindPosition
  // this happens when the blind enters a moving state

  unsigned int startAddress = RF12_EEPROM_ADDR + sizeof config + sizeof calibration ;

  if (initDone) {
    Serial.println F(("save blind position data")) ;
    if ( clearPos == 1 ) {
      eeprom_update_byte(startAddress, ((byte*) &clearBlindPosition)[0]);
      for (unsigned int i = 0; i < sizeof blindPosition; ++ i)
        eeprom_update_byte(startAddress + i, ((byte*) &clearBlindPosition)[i]);
    } else {
      eeprom_update_byte(startAddress, ((byte*) &blindPosition)[0]);
      for (unsigned int i = 0; i < sizeof blindPosition; ++ i)
        eeprom_update_byte(startAddress + i, ((byte*) &blindPosition)[i]);
    }
  }
}

#ifdef DEBUG
static void printConfig (blindConfig cfg) {

  Serial.print(F("nodeid:")); Serial.println(cfg.nodeId);
  Serial.print(F("autoOpenMin:")); Serial.println(cfg.autoOpenMin);
  Serial.print(F("autoCloseMin:")); Serial.println(cfg.autoCloseMin);
  Serial.print(F("sunDipSec:")); Serial.println(cfg.sunDipSec);
  Serial.print(F("sunDipPosition:")); Serial.println( cfg.sunDipPosition); //set to 0 to disable
  Serial.print(F("openThreshold:")); Serial.println(cfg.openThreshold);
  Serial.print(F("closeThreshold:")); Serial.println(cfg.closeThreshold);
  Serial.print(F("sundipThreshold:")); Serial.println( cfg.sundipThreshold);
  Serial.print(F("autoWaitMin:") ); Serial.println(cfg.autoWaitMin);
  Serial.print(F("manWaitMin:") ); Serial.println(cfg.manWaitMin);
  Serial.print(F("opMode:") ); Serial.println(cfg.opMode);
  Serial.print(F("underRun:") ); Serial.println(cfg.underRun);
  Serial.print(F("overRun:") ); Serial.println(cfg.overRun);
  Serial.print(F("crc:") ); Serial.println(cfg.crc);
}

static void printCalibration() {

  for (int blind = 0; blind < numBlinds; blind++) {
    Serial.print F(("Calibration Data for blind ")) ;
    Serial.println(blind) ;

    Serial.print F(("Open time:"));
    Serial.println( calibration.openTime[blind] ) ;
    Serial.print F(("Close time:"));
    Serial.println( calibration.closeTime[blind] ) ;
    Serial.print F(("BlindPosition:")) ; Serial.println(blindPosition[blind]) ;

  }


}
#endif

byte checkConfig (blindConfig cfg) {

  //returns 1 if the config is bad or not for this node or 0 if it seems reasonable

  //check the crc
  Serial.print F(("calculated crc:")) ;
  Serial.println(calcCrc(&cfg, sizeof cfg - 2) ) ;
  if (cfg.crc != calcCrc(&cfg, sizeof cfg - 2) ) {
    Serial.println F(("checkConfig crc bad"));
    return (1);
  }

  if (cfg.nodeId != MYNODEID ) {
    //this config not for us
    return (1) ;
  } else if ( cfg.autoOpenMin > 240 || cfg.autoCloseMin > 240 || cfg.autoWaitMin > 240 || cfg.manWaitMin > 240 ) {
    //none of thse should be > 4hrs
    return (1);
  } else if (cfg.sunDipSec > 600 || cfg.sunDipPosition > 100 ) {
    return (1);
  } else if ( cfg.openThreshold > 1000 || cfg.closeThreshold > 1024 || cfg.sundipThreshold > 500 ) {
    return (1) ;
  } else if (cfg.opMode > 3) {
    return (1) ;
  } else if (cfg.overRun > 20 || cfg.underRun > 10 ) {
    Serial.println F(("cfg.overrun or cfg.underrun invalid"));
    return (1) ;
  } else {
    //config good
    Serial.println F(("checkConfig config OK"));
    return (0);
  }

}


static void defaultConfig () {
  //Initialise config struct with default values for this node

  config.nodeId = MYNODEID ;                  // used by rf12_config, offset 0
  config.autoOpenMin = AUTO_OPEN_MIN ;        //How long should light intensity be above OPEN_THRESH for before opening blind (5 mins) (also used for sundip exiting)
  config.autoCloseMin = AUTO_CLOSE_MIN ;     //How long should light intensity be below CLOSE_THRESH for before closing blind (5 mins)
  config.sunDipSec = SUNDIP_SEC  ;          // How long should light intensity be above SUNDIP_THRESH for before closing blind (30 sec)
  config.sunDipPosition = SUNDIP_POS ;      //What % closed blind should move to when high light intensity sensed
  config.openThreshold = OPEN_THRESH ;  //Light intensity to trigger blind opening
  config.closeThreshold = CLOSE_THRESH ; //Light intensity to trigger blind closing
  config.sundipThreshold = SUNDIP_THRESH ; ////Light intensity to trigger blind sundipping
  config.autoWaitMin  = AUTO_WAIT_MIN ; //Do not take automatic actions greater than this period
  config.manWaitMin = MAN_WAIT_MIN ; //Do not take automatic actions after a manual operation for this time
  config.opMode = 3 ;                //Fully automatic mode
  config.underRun = UNDERRUN ;       // 0 minus this value is the minimum position the blind can run to (in case top limit switch missed)
  config.overRun = OVERRUN;          // 100 plus this value is the maximum position the blind can run to (in case bottom limit switch missed)
  config.crc = 0;

  /*
    //also reset blind calibration values
    calibration.openTime[0] = 0 ;
    calibration.openTime[1] = 0;
    calibration.closeTime[0] = 0 ;
    calibration.closeTime[1] = 0 ;
    //config.pad[RF12_EEPROM_SIZE - 8];
  */

}

static void motorSetup () {
  motor.send();
  motor.write(MCP_IODIR);
  motor.write(240); // p0-p3 outputs, p4-p7 inputs
  motor.stop();
  motor.send();
  motor.write(MCP_GPPU) ; // enable pullups on p4-p7
  motor.write(240) ;
  motor.stop();
}

static void motorWrite (byte value) {
  motor.send();
  motor.write(MCP_GPIO);
  motor.write(value);
  motor.stop();
}

static byte exp_read () {
  motor.send();
  motor.write(MCP_GPIO);
  motor.receive();
  byte result = motor.read(1);
  motor.stop();
  return result;
}


void LED ( byte state ) {
  //Set LEDs both onboard and external to state
  LEDLDR.digiWrite(state);
  digitalWrite(pcbLEDPin,  !state) ; //onboard LED has inverse logic as it is switched on gnd
}


void motorRun (  int blind,  motorDirections direction) {

  //sleep for 400ms before changing motor directions
  //delay(400);

  //Keep track of motorRunTime
  //motorRunTime & motorRunning are only set to zero by STOP direction (below)
  if (direction == UP || direction == DOWN) {
    if (motorRunning[blind] == 0 ) {
      motorRunStart[blind] = millis() ;
      motorRunning[blind] = 1 ;
      LED(1);
    } else {
      motorRunTime[blind] = millis() - motorRunStart[blind] ;
      if (direction == DOWN && (motorRunTime[blind] > CLOSE_MAX_MSEC)) {
        //stop everything
        motorWrite(0);
        halt("Blind close time exceeded max time allowed") ;
      } else if (direction == UP && (motorRunTime[blind] > OPEN_MAX_MSEC)) {
        //stop everything
        motorWrite(0);
        halt("Blind open time exceed max time allowed") ;
      }
    }
  }


#ifdef DUALBLINDS
  //H-bridges are driven independently for each blind

  if (direction == UP) {
    //Open
    if (blind == 0) {
      motorWrite((exp_read() & 252) + 1);
    } else if (blind == 1) {
      motorWrite((exp_read() & 243) + 4);
    }
  } else if (direction == DOWN) {
    //Close
    if (blind == 0) {
      motorWrite((exp_read() & 252) + 2);
    } else if (blind == 1) {
      motorWrite((exp_read() & 243) + 8);
    }
  } else {
    //Stop
    if (blind == 0) {
      motorWrite((exp_read() & 252) + 0);
    } else if (blind == 1) {
      motorWrite((exp_read() & 243) + 0);
    } else {
      //stop everything
      motorWrite(0);
    }
    motorRunning[blind] = 0 ;
    updateBlindPosition() ;   //Do the final update of blind positions on stop command.
    motorRunTime[blind] = 0;

  }
#else
  // Only one blind
  // H-bridges driven together allowing either or both can be used on same blind (e.g motor each end of blind)
  blind = 0 ;

  if (direction == UP) {
    //Open
    motorWrite((exp_read() & 240) + 5);
  } else if (direction == DOWN) {
    //Close
    motorWrite((exp_read() & 240) + 10);
  } else {
    //Stop
    motorWrite((exp_read() & 240) + 0);
    motorRunning[blind] = 0 ;
    updateBlindPosition () ;   //Do the final update of blind positions on stop command.
    motorRunTime[blind] = 0 ;
  }
#endif

}


#ifdef LIMIT_SWT_IO_EXP
byte topLimitSwitch(int blind) {

  if (blind == 0 ) {
    // Get input P7 only
    if ((exp_read() & 128)  == 128) {
      return 0;
    } else {
      return 1 ;
    }
  } else if (blind == 1) {
    // Get input P5 only
    if ((exp_read() & 32)  == 32) {
      return 0;
    } else {
      return 1 ;
    }
  }


}

byte bottomLimitSwitch(int blind) {
  if (blind == 0 ) {
    // Get input P6 only
    if ((exp_read() & 64)  == 64) {
      return 0;
    } else {
      return 1 ;
    }
  } else if (blind == 1) {
    // Get input P4 only
    if ((exp_read() & 16)  == 16) {
      return 0;
    } else {
      return 1 ;
    }
  }

}

#else
//We can only support one blind if not using IO Expander.
//So this config with DUALBLINDS is only useful for breadboard testing
byte topLimitSwitch(int blind) {
  if (blind > 0 ) {
    return 1;
  }
  return (! REED.digiRead()) ;
}

byte bottomLimitSwitch(int blind) {
  if (blind > 0 ) {
    return 1;
  }
  return (! REED.digiRead2()) ;
}
#endif


#ifdef OPENCLOSE_SWT_IO_EXP
byte openSwitch() {
  // Get input P7 only
  if ((exp_read() & 128)  == 128) {
    return 0;
  } else {
    return 1 ;
  }
}

byte closeSwitch() {
  // Get input P6 only
  if ((exp_read() & 64)  == 64) {
    return 0;
  } else {
    return 1 ;
  }
}
#else
byte openSwitch() {
  return (! BUTTONS.digiRead()) ;
}

byte closeSwitch() {
  return (! BUTTONS.digiRead2()) ;
}
#endif



void doClose (int blind) {

#ifdef DEBUG
  // Serial.print(F("c"));
#endif
  if (bottomLimitSwitch(blind) == 1 || blindPosition[blind] >= (100 + config.overRun) ) {
    //Blind fully closed, stop
    motorRun(blind, STOP) ;
    blindPosition[blind] = 100 ;
    setBsmState(blind, CLOSED) ;
  } else {
    motorRun(blind, DOWN) ; // Close the blind
    setBsmState(blind, CLOSING) ;
  }



}

void doOpen (int blind) {

#ifdef DEBUG
  //Serial.print(F("o"));
#endif
  if (topLimitSwitch(blind) == 1 || blindPosition[blind] <= (0 - config.underRun ) ) {
    //Blind fully open, stop
    motorRun(blind, STOP) ;
    blindPosition[blind] = 0 ;
    setBsmState(blind, OPEN) ;
  } else {
    motorRun(blind, UP) ; // open the blind
    setBsmState(blind, OPENING) ;
  }

}

void doSunDip(int blind) {

  //CLOSE the blind to config.sunDipPosition, but only if blind not already past that position
  //Sundip only works on blind 0

  blind = 0 ;

  if ( config.sunDipPosition != 0  && (!bottomLimitSwitch(blind)) && ((double)blindPosition[blind] < (double)config.sunDipPosition) && (blindPosition[blind] < 100) ) {
#ifdef DEBUG
    Serial.print(F("doSunDip blind:"));
    Serial.println(blind);
#endif
    motorRun(blind, DOWN) ; // Close the blind
    setBsmState(blind, SUNDIPPING) ;
  } else if (config.sunDipPosition != 0) {  //only if sundip not disabled
    setBsmState(blind, SUNDIP);
    motorRun(blind, STOP);
  }

}



void jogBlind() {
  //make a small blind movement to acknowledge something
  motorRun(0, STOP) ;
  delay(400) ;
  motorRun(0, DOWN) ;
  delay(400) ;
  motorRun(0, STOP) ;
  delay(700);
  motorRun(0, UP) ;
  delay(400) ;
  motorRun(0, STOP) ;
  delay(700);
}


void checkBlindPosition() {
  /* Check the position of the blind has not hit reed switches.
      Call this often. Very often.
      If reed switches are hit, motor(STOP) is called.
      It also resets the hardware watchdog
  */
  for (int blind = 0; blind < numBlinds; blind++) {

    if ( motorRunning[blind] == 1) {

      if ( (bottomLimitSwitch(blind) == 1 || blindPosition[blind] >= (100 + config.overRun) ) && (blindStateMachine[blind] == CLOSING || blindStateMachine[blind] == SUNDIPPING) ) {
        //Blind fully closed, stop
        //Must check we are not opening as closed reed switch may be active for a short time when opening as magnet moves away
#ifdef DEBUG
        Serial.println(F("Blind "));
        Serial.print(blind) ;
        Serial.println(F(" closed")) ;
#endif
        motorRun(blind, STOP) ;
        setBsmState(blind, CLOSED) ;
        //blindPosition[blind] = 100 ;
      }

      if ( (topLimitSwitch(blind) == 1 || blindPosition[blind] <= (0 - config.underRun) ) && blindStateMachine[blind] == OPENING) {
        //Must check we are not closing as open reed switch may be active for a short time when opening as magnet moves away
#ifdef DEBUG
        Serial.println(F("Blind "));
        Serial.print(blind) ;
        Serial.println(F(" opened")) ;
#endif
        motorRun(blind, STOP) ;
        setBsmState(blind, OPEN) ;
        //blindPosition[blind] = 0 ;
      }

    }//motorRunning

  }//for blind

}

//Keep track of the blind position by using time of motor running and converting to % closed
//Seperate timings are used for open & close and for each blind, and % updated according to operation & direction (open/close speeds)
void updateBlindPosition() {

  unsigned long motorRunTimeDelta[2] = {0, 0} ;
  static unsigned long lastMotorRunTime[2] = {0, 0} ;

  if (initDone == 0) {
    //Do not try to update blind position if init not completed
#ifdef DEBUG
    Serial.println(F("updateBlindPosition called before init complete - ignored"));
#endif
    return ;
  }

  for (int blind = 0; blind < numBlinds; blind++) {

    //if fully open or closed set blindPosition accordingly
    if (blindStateMachine[blind] == OPEN ) {
      blindPosition[blind] = 0 ;
      lastMotorRunTime[blind] = 0 ;
    } else if (blindStateMachine[blind] == CLOSED ) {
      blindPosition[blind] = 100 ;
      lastMotorRunTime[blind] = 0 ;
    } else if (blindStateMachine[blind] == OPENING || blindStateMachine[blind] == CLOSING || blindStateMachine[blind] == SUNDIPPING) {

      if ( motorRunTime[blind] == 0 ) {
        lastMotorRunTime[blind] = 0 ;
      }

      motorRunTimeDelta[blind] = motorRunTime[blind] - lastMotorRunTime[blind] ;


      //have to do math to guess...
      if (blindStateMachine[blind] == OPENING) {
        //get runTime as a % of openTime and subtract from Blind1Position
        blindPosition[blind] -=  ( motorRunTimeDelta[blind] / (float)calibration.openTime[blind])  * 100;
      } else if (blindStateMachine[blind] == CLOSING || blindStateMachine[blind] == SUNDIPPING) {
        //get runTime as a % of closeTime and add to Blind1Position
        blindPosition[blind] += (motorRunTimeDelta[blind] / (float)calibration.closeTime[blind]) * 100 ;
      } else {
        //Serial.print("updateBlindPosition: blind is not moving, blindStateMachine:") ;
        //Serial.println(blindStateMachine) ;
      }

      lastMotorRunTime[blind] = motorRunTime[blind] ;

#ifdef DEBUG
      Serial.print(F("updateBlindPosition: blind:")) ; Serial.print(blind) ;
      Serial.print(F(" position:")) ; Serial.print(blindPosition[blind]) ;
      Serial.print (F(" runTime:")) ; Serial.println(motorRunTime[blind]) ;
#endif


      if (blindPosition[blind] > (100 + config.overRun) || blindPosition[blind] < (0 - config.underRun) ) {
        // We have a problem, the blind cannot be < 0% open or > 100% closed !
        Serial.print(F("updateBlindPosition: blindPosition out of range config.overrun or config.underrun for blind " )) ;
        Serial.println(blind) ;
        //Serial.println("Recalibrating! ") ;
        //calibrate() ;
      }

    } else {
      lastMotorRunTime[blind] = 0 ;
    } // if blindStateMachine[blind] == OPEN

  } // for blind < numBlinds

}


//Read the LDR
word readLDR() {
  word ldrVal = 0;
  ldrVal = LEDLDR.anaRead();
  return ldrVal ;
}


//Check Timer State Machine
void updateTsm() {

  //Evaluate if timer has hit any thresholds & update timeStateMachine
  timerRunTime = millis() - timerStartTime ;
  if (timerStateMachine == OPEN_WAIT && timerRunTime > ((long)config.autoOpenMin * 60000u)) {
#ifdef DEBUG
    Serial.println(F("Timer stop config.autoOpenMin expired"));
#endif
    timerStateMachine = OPEN_EXPIRE ;
    timerRunning = false ;
  } else if (timerStateMachine == CLOSE_WAIT && timerRunTime > ((long)config.autoCloseMin * 60000u)) {
#ifdef DEBUG
    Serial.println(F("Timer stop config.autoCloseMin expired"));
#endif
    timerStateMachine = CLOSE_EXPIRE ;
    timerRunning = false ;
  } else if (timerStateMachine == SUNDIP_WAIT && timerRunTime > ((word)config.sunDipSec * 1000u)) {
#ifdef DEBUG
    Serial.println(F("Timer stop config.sunDipSec expired"));
#endif
    timerStateMachine = SUNDIP_EXPIRE ;
    timerRunning = false ;
  }


}


//Start Timer state machine
void startTsm( timerStateMachines tsmTimer ) {

  if (!timerRunning) {
    timerRunning = true ;
    timerStartTime = millis() ;
    timerStateMachine = tsmTimer ;
  } else if (tsmTimer == timerStateMachine) {
    timerRunTime = millis() - timerStartTime ;
  } else {
    //a different timer has been started
    timerStateMachine = tsmTimer ;
    timerStartTime = millis() ;
    timerRunTime = 0 ;
  }

}


void stopTsm() {

  timerRunning = false ;
  timerStateMachine = TSTOP ;
#ifdef DEBUG
  Serial.println(F("Timer stop"));
#endif

}


void blinkLED() {

  static byte LEDstate = 0 ;
  static  unsigned long lastBlink = 0 ;

  if (millis() - lastBlink > LEDBLINK_INTERVAL) {
    // LEDstate = ! LEDstate ;
    LED(0);
    LED(1) ;
    delay(100);
    LED(0);
    lastBlink = millis() ;
  }

}


void halt(String msg) {

  //set opMode to 0
  config.opMode = 0 ;

  setBsmState(0, HALT) ;
#ifdef DUALBLINDS
  setBsmState(1, HALT) ;
#endif


  lastStatusPrint = millis() ;
  Serial.println(F("Error condition, code execution stopped")) ;
  Serial.println(msg) ;
  while (1) {
    LED(1);
    delay(200);
    LED(0);
    delay(500) ;
    if (millis() - lastStatusPrint > STATUS_INTERVAL) {
      reportState() ;
      lastStatusPrint = millis() ;
    }

    wdt_reset() ;
  }

}


int checkCalibration() {
  /* Do we need to calibrate, is the eeprom calibration data good ? */

  int rc = 0 ;    // Set to 1 if we need to calibrate

  for (int blind = 0; blind < numBlinds; blind++) {

    if (calibration.openTime[blind] < 5000 || calibration.openTime[blind] > OPEN_MAX_MSEC) {
      rc = 1 ;
    }
    if (calibration.closeTime[blind] < 5000 || calibration.closeTime[blind] > CLOSE_MAX_MSEC) {
      rc = 1 ;
    }

  }

#ifdef DEBUG
  if ( rc == 0 ) {
    Serial.println(F("Calibration data OK")) ;
  } else if (rc == 1 ) {
    Serial.println(F("Calibration data outside of range, recalibration required")) ;
  } else if (rc == 2 ) {
    Serial.println(F("blindPosition data outside of range, recalibration required")) ;
  } else {
    Serial.println(F("nvRam data verification issue") ) ;
  }
#endif

  return (rc) ;
}


void calibrate() {
  /* discover travel times for blind open and close */

  unsigned long startTime = 0 ;

  initDone = 0 ;
  blindPosition[0] = 255 ;
  blindPosition[1] = 255 ;

  calibration.openTime[0] = 0;
  calibration.closeTime[0] = 0;
  calibration.openTime[1] = 0;
  calibration.closeTime[1] = 0;


  //Initialise blind position
  //Return to top until topReed is triggered

  for (int blind = 0; blind < numBlinds; blind++) {

    setBsmState(blind, PARTIAL_OPEN) ; //don't know blind position yet... initialise below to find out...

    motorRun(blind, STOP) ;

    //Open all the way first
    Serial.println(F("calibrate:Initialise blind to open position.."));
    startTime = millis();
    while (blindStateMachine[blind] != OPEN ) {
      doOpen(blind);
      delay (100) ;
      calibration.openTime[blind] = millis() - startTime ;
      if (calibration.openTime[blind] > OPEN_MAX_MSEC) {
        motorRun(blind, STOP) ;
        halt("calibrate:Blind exceeded open time threshold") ;
      }
    }
    //Time how long blind takes to close
    Serial.println(F("calibrate:Timing blind move to closed position.."));

    delay(500) ;

    startTime = millis() ;
    while (blindStateMachine[blind] != CLOSED ) {
      doClose(blind);
      delay (100) ;
      calibration.closeTime[blind] = millis() - startTime ;
      if (calibration.closeTime[blind] > CLOSE_MAX_MSEC ) {
        motorRun(blind, STOP) ;
        halt("caibrate:Blind exceeded close time threshold during callibration - HALT") ;
      }
    }


    motorRun(blind, STOP) ;
    blindPosition[blind] = 100 ;

    Serial.println(F("calibrate:Blind closed...timing move to open position.."));
    delay(500) ;

    //Time how long blind takes to open
    startTime = millis();
    while (blindStateMachine[blind] != OPEN ) {
      doOpen(blind);
      delay (100) ;
      calibration.openTime[blind] = millis() - startTime ;
      if (calibration.openTime[blind] > OPEN_MAX_MSEC ) {
        motorRun(blind, STOP) ;
        halt("calibrate:Blind exceeded open time threshold") ;
      }
    }
    motorRun(blind, STOP) ;
    blindPosition[blind] = 0 ;
#ifdef DEBUG
    Serial.print(F("calibrate:Blind "));
    Serial.print(blind) ;
    Serial.println (F(" open time:"));
    Serial.println(calibration.openTime[blind]) ;
    Serial.print(F("calibrate:Blind ")) ;
    Serial.print(blind) ;
    Serial.println(F(" close time:"));
    Serial.println(calibration.closeTime[blind]) ;
#endif
    setBsmState(blind, OPEN) ;
  }

  Serial.println(F("calibrate:Complete"));
  saveBlindPos(1) ;   // wipe blind position from nvRAM
  saveCalibration() ;

}


//Set BlindStateMachine State
void setBsmState (  int blind, enum blindStateMachines newBsmState) {

  if (blindStateMachine[blind] != newBsmState) {
    oldBsmState[blind] = blindStateMachine[blind] ;
    blindStateMachine[blind] = newBsmState ;
    reportState() ;
#ifdef DEBUG
    Serial.print(F("New State for blind:"));
    Serial.print(blind) ;
    Serial.print(F(" state:")) ;
    Serial.println(blindStateMachine[blind]);
#endif

    if ( blindStateMachine[blind] == SUNDIP || blindStateMachine[blind] == PARTIAL_OPEN || blindStateMachine[blind] == OPEN || blindStateMachine[blind] == CLOSED) {
      //only save blind position in NVRAM when blind is static and state has changed
      saveBlindPos(0) ;
    } else {
      //wipe blind position from NVRAM when blind is travelling
      //other wise we have to update it constantly during travel
      //if we loose power while blind is travelling then NVRAM will have
      //0 for blindPosition, causing setup() to move blind to fully open
      saveBlindPos(1) ;
    }

  }

}

void reportState() {
  // Report status via serial & RF
  bool needToSend = 1;
  byte numSendAttempts = 0 ;

  report.destID =  EMONPI_NODEID ;
  report.opMode = config.opMode ;
  report.bsm0 = blindStateMachine[0] ;
  report.bsm1 = blindStateMachine[1] ;
  report.tsm = timerStateMachine ;
  report.ldrVal = readLDR() ;
  report.position0 = blindPosition[0] ;
  report.position1 = blindPosition[1] ;

  //if we found a temp sensor during setup on the 'OPEN' switch IO pin
  //get a reading from the first temp sensor only
  if (ds18b20Present) {
    double rawtemp = (tSensors.getTempCByIndex(0));
    if ((rawtemp > -30) && (rawtemp < 70)) {
      //is temperature withing reasonable limits?
      report.temp = (int) (rawtemp * 10) ;
    } else {
      report.temp = -1275 ;
    }
    Serial.print(F("Temperature:")) ;
    Serial.println((double)report.temp / 10) ;
  }

  Serial.print(F("blind0StateMachine:")) ;
  Serial.println(report.bsm0) ;
  Serial.print(F("timerStateMachine:")) ;
  Serial.println(report.tsm) ;
  Serial.print(F("Blind0 Position:")) ;
  Serial.println(report.position0) ;
#ifdef DUALBLINDS
  Serial.print(F("blind1StateMachine:")) ;
  Serial.println(report.bsm1) ;
  Serial.print(F("Blind1 Position:")) ;
  Serial.println(report.position1) ;
#endif
  Serial.print(F("ldrVal=")) ;
  Serial.println(report.ldrVal) ;

  //only send when allowed to do so
  needToSend = 1;
  numSendAttempts = 0 ;




#ifdef WX_REPEATER
  //only send if we are in FSK mode
  if (rxMode == FSK) {
#endif
    while ( needToSend && numSendAttempts <= MAX_TX_ATTEMPTS) {
      rf12_recvDone() ;
      if (rf12_canSend()) {
        rf12_sendStart(0, &report, sizeof report);
        needToSend = 0;
      } else {
        numSendAttempts ++ ;
        delay(300);
      }
    }
#ifdef WX_REPEATER
  } //(if rxMode==FSK)
#endif
  if (numSendAttempts >= MAX_TX_ATTEMPTS) {
    Serial.print(F("Failed to reportState RF after ")) ; Serial.print(numSendAttempts - 1) ; Serial.println (F(" tries")) ;
    rf12_initialize(config.nodeId, RF_freq, NETGRP);  //Initialize RFM12
  }

  return ;
}

void doRfRecv() {

  // Did we receive any commands via RF ?
  if ( (rf12_recvDone() && rf12_crc == 0)  && ((rf12_hdr & RF12_HDR_CTL) == 0) ) {

    EmoncmsPayload emoncms;

    bool blindRfData = 0 ;                  //if the RX packet is a blind control one
    int node_id = (rf12_hdr & 0x1F);        // Extract transmitter node ID

    if (node_id == EMONPI_NODEID )     {    // did it come from EMONPI
      // The packet data is contained in rf12_data, the *(EmoncmsPayload*) part tells the compiler
      // the format of the data is so that it can be copied correctly

      if (sizeof(emoncms) == rf12_len) {
        emoncms = *(EmoncmsPayload*) rf12_data;
        blindRfData = true ;

#ifdef WX_REPEATER
      } else if (sizeof(rtcData) == rf12_len  && rf12_data[1] == 0x0A )     {  //is it emonGLCD packet?
        rtcData = *(PayloadRTC*) rf12_data;
#ifdef DEBUG
        Serial.println(F("Received RTC data from Emonhub")) ;
#endif
        if  (DateTime(rtcData.year, rtcData.month, rtcData.day, rtcData.hour, rtcData.min, rtcData.sec).isValid() ) {
          RTC_Millis::adjust(DateTime(rtcData.year, rtcData.month, rtcData.day, rtcData.hour, rtcData.min, rtcData.sec )) ;
          lastRtcUpdate = millis() ;

        } // if DateTime(rtcData.year, rtcData.month...


#endif

      } else {

#ifdef DEBUG
        Serial.print (F("Invalid rf_data received.")) ;
        Serial.print (F("Expected rf_data the same size as emonCMSPayload:" )); Serial.println(sizeof(emoncms) ) ;
        Serial.print (F("rf12_data size:" )); Serial.println(rf12_len ) ;
#endif
      }

      if ( emoncms.destID == config.nodeId && blindRfData ) {
#ifdef DEBUG
        Serial.print(F("RF blind control command received:")) ;
#endif
        switch (emoncms.cmd) {
          //avoid using case 0:
          //when emonpi boots, in the firmware it sends an RF packet to nodes 1 thru 10 with first byte 0 - ignore it

          case 1 :
#ifdef DEBUG
            Serial.println(F("Open")) ;
#endif
            motorRun(0, STOP) ;
            setBsmState(0, OPENING);
#ifdef DUALBLINDS
            motorRun(1, STOP) ;
            setBsmState(1, OPENING);
#endif
            delay(400); //let the motor rest before direction change
            //Consider this a manual action
            lastAction = millis() ;
            lastActionWasManual = true ;
            break;

          case 2 :
#ifdef DEBUG
            Serial.println(F("Close")) ;
#endif
            motorRun(0, STOP) ;
            setBsmState(0, CLOSING);
#ifdef DUALBLINDS
            motorRun(1, STOP) ;
            setBsmState(1, CLOSING);
#endif
            delay(400); //let the motor rest before direction change
            //Consider this a manual action
            lastAction = millis() ;
            lastActionWasManual = true ;
            break ;

          case 3 :
#ifdef DEBUG
            Serial.println(F("Sundip")) ;
#endif
            setBsmState(0, SUNDIPPING) ;
            lastAction = millis() + ((long)config.manWaitMin * 60000);
            break ;

          case 4:
#ifdef DEBUG
            Serial.println(F("New config"));
#endif
            if (checkConfig(emoncms.newConfig) == 0) {
              config = emoncms.newConfig ;
              saveConfig();
              loadConfig();
#ifdef DEBUG
              printConfig(config) ;
#endif
              jogBlind() ;
            } else {
              Serial.println(F("New config invalid"));
            }
            break ;

          case 5:
#ifdef DEBUG
            Serial.println(F("Reset default config"));
#endif
            defaultConfig() ;
            saveConfig() ;
#ifdef DEBUG
            printConfig(config) ;
#endif
            jogBlind() ;
            break ;

          case 6:
            if (emoncms.opMode < 4) {
              config.opMode = emoncms.opMode ;

#ifdef DEBUG
              Serial.print(F("New opmode:")) ;
              Serial.println(config.opMode) ;
#endif

              jogBlind() ;
            } else {
#ifdef DEBUG
              Serial.println(F("Invalid opmode received")) ;
#endif
            }

          default :
#ifdef DEBUG
            Serial.println(F("Unrecognised blind control command - ignored")) ;
#endif

            break ;

        } //switch emoncms.cmd

      } // if emoncms.destid==condig.nodeid

    } // if nodeid==emonpi

  }
  rf12_recvDone();

}  // act on RF commands received



void setup() {
  Serial.begin(57600);
  Serial.print(F("Blind Controller version "));
  Serial.print(VERSION);
  Serial.println(" started");

  //enable pullup
  LEDLDR.digiWrite2(1);
  pinMode(pcbLEDPin, OUTPUT);
  REED.mode(INPUT_PULLUP) ;
  REED.mode2(INPUT_PULLUP) ;
  LEDLDR.mode(OUTPUT);
  LEDLDR.mode2(INPUT_PULLUP) ;
  BUTTONS.mode(INPUT_PULLUP);
  BUTTONS.mode2(INPUT_PULLUP);

  //we're alive !
  LED(1) ;

  loadConfig() ;      //Load config (if any) from EEPROM
  if (config.nodeId != MYNODEID || checkConfig(config) != 0 ) {
    Serial.println(F("NVRAM config appears corrupted - using defaults & resaving to NVRAM"));
    defaultConfig() ;
    saveConfig() ;
  } else {
    loadCalibration() ;   //Load calibration & blind position data from EEPROM
    loadBlindPos() ;
    Serial.println(F("EEPROM config, calibration & last blind position loaded"));
  }


  //RF setup
  rf12_initialize(config.nodeId, RF_freq, NETGRP);  //Initialize RFM12

#ifdef DEBUG
  Serial.println(F("RF config")) ;
  Serial.print("Node: ");  Serial.print(config.nodeId);  Serial.print(" Freq: ");
  if (RF_freq == RF12_433MHZ) Serial.print("433Mhz");
  Serial.print(F(" Network: ")); Serial.println(NETGRP);
#endif




  //Iniitalise DC motor board
  motorSetup() ;

  if ( ! motor.isPresent()) {
    //dc motor board not found !
#ifdef BREADBOARD
    Serial.println(F("\nSetup: Motor drive I2C board not found.. but you know this\n")) ;
    delay(1000);
#else
    halt(F("Setup: DC motor drive I2C board not found!")) ;
#endif
  }

  //remember my opMode as it can be toggled during execution by pressing up+down buttons together
  defaultOpMode = config.opMode ;
#ifdef DEBUG
  printConfig(config);
  printCalibration() ;
#endif




  /*
    #ifdef BREADBOARD
    Serial.println("Breadboard node, skipping calibration. Blinds set to 60sec open, 45sec close and now in OPEN position");
    calibration.openTime[0] = 60000u ;
    calibration.closeTime[0] = 45000u ;
    blindPosition[0] = 0 ;
    setBsmState(0, OPEN) ;

    #ifdef DUALBLINDS
    calibration.openTime[1] = 60000u ;
    calibration.closeTime[1] = 45000u ;
    blindPosition[1] = 0 ;
    setBsmState(1, OPEN) ;
    #endif

    #else
  */
  delay(2000);


  if ((openSwitch() && closeSwitch()) || checkCalibration() ) {
    //if both open & close  active OR stored calibration values invalid run calibration
    defaultConfig() ;
    saveConfig() ;
    calibrate() ;
  }



  for (int blind = 0; blind < numBlinds; blind++) {

    motorRun(blind, STOP) ;
    // Figure out blind position (check limit switches, nvram, or else open blind till topLimit hit)

    // Check limit switches as they are definitive
    if ( topLimitSwitch(blind) ) {
      blindPosition[blind] = 0 ;
      setBsmState(blind, OPEN) ;
    } else if (bottomLimitSwitch(blind) ) {
      blindPosition[blind] = 100 ;
      setBsmState(blind, CLOSED) ;
    } else if (blindPosition[blind] >= 1 && blindPosition[blind] <= 99)   {
      setBsmState(blind, PARTIAL_OPEN) ;
    } else if ( blindPosition[blind] <= 1 && (blindPosition[blind] >= 0 - config.underRun) ) {
      setBsmState(blind, OPEN);
    } else if ( blindPosition[blind] >= 99 && (blindPosition[blind] <= 100 + config.overRun) ) {
      setBsmState(blind, CLOSED);
    } else {
      //only move blind to open if all else fails this avoids blind opening after a power cut
      Serial.println(F("setup:Initialise blind to closed position.."));
      setBsmState(blind, PARTIAL_OPEN) ;
      while (blindStateMachine[blind] != CLOSED ) {
        doClose(blind);
        delay (100) ;
      } // while !=OPEN

    }
    motorRun(blind, STOP) ;
  } //for blind




  cli() ;
  wdt_reset();
  // Enter Watchdog Configuration mode:// Enter Watchdog Configuration mode:
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  // Set Watchdog settings, 8 sec timeout
  WDTCSR = (0 << WDIE) | (1 << WDE) | (1 << WDP3) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0);
  sei() ;


  lastAction = 0 ;
  initDone = 1 ;
  // unless cleared in loop() after we run for 30 secs
  reportState() ;


  LED(0) ;

#ifdef WX_REPEATER
  wxSetup() ;
#endif


  //Do we have a DS18B20 connected ?
  tSensors.begin();                         // start up the DS18B20 temp sensor onboard
  if (tSensors.getDS18Count() > 0) {
    Serial.println(F("DS18B20 temperature sensor present.")) ;
    ds18b20Present = true ;
  } else {
    ds18b20Present = false ;
  }

  Serial.println(F("Setup complete."));
}

void loop() {

  word ldrVal = 0 ;


  //reset the wdt
  wdt_reset() ;

  // Run the blindStateMachine machine

  if (millis() - lastPosUpdate > POS_UPDATE_INTERVAL) {
    checkBlindPosition() ;  //also resets the watchdog timer
    updateBlindPosition() ;
    lastPosUpdate = millis() ;
  }


  //In all modes if the BSM is OPENING or CLOSING, call doOpen, doClose or doSunDip to complete the state
  //SunDip completion is necessary even in manual mode as auto-mode invokes it but then locks out due to config.autoWaitMin
  //meaning SunDip would not complete. However SunDip can never be invoked in manual mode.

  if (blindStateMachine[0] == OPENING ) {
    doOpen(0) ;
  } else if (blindStateMachine[0] == CLOSING) {
    doClose(0) ;
  } else if (blindStateMachine[0] == SUNDIPPING) {
    // if we are SUNDIPPING, complete the action
    doSunDip(0) ;
  }

#ifdef DUALBLINDS
  if (blindStateMachine[1] == OPENING ) {
    doOpen(1) ;
  } else if (blindStateMachine[1] == CLOSING) {
    doClose(1) ;
  }
#endif


  if ( openSwitch()  || closeSwitch() ) {
    //debounce
    delay(100) ;
  }


  if ( openSwitch()  &&  closeSwitch() ) {
    //Mode Switch
    LED(1);
    if (config.opMode == 0 ) {
      config.opMode = defaultOpMode ;
    } else {
      config.opMode = 0;
    }

    Serial.print(F("OpMode now:")) ;
    Serial.println(config.opMode) ;
    jogBlind();
    delay(500);
    LED(0);
  }


  //In all modes allow manual switches to work

  if ( openSwitch()  ) {      //OPEN button pressed

#ifdef DEBUG
    Serial.println(F("Open button press"));
#endif
    lastAction = millis() ;
    lastActionWasManual = true ;

    stopTsm() ;  //stop all timers

    for (int blind = 0; blind < numBlinds; blind++) {
    
      motorRun(blind, STOP) ;

      switch (blindStateMachine[blind]) {
        case CLOSED:
          setBsmState(blind, OPENING);
          doOpen(blind) ;
          continue ;
        case OPEN:
#ifdef DEBUG
          Serial.print(F("Blind ")); Serial.print(blind); Serial.println (F(" already Open"));
#endif
          continue ;
        case CLOSING:
          setBsmState(blind, PARTIAL_OPEN);
          continue ;
        case OPENING:
          setBsmState(blind, PARTIAL_OPEN);
          continue ;
        case SUNDIP:
          setBsmState(blind, OPENING);
          doOpen(blind) ;
          continue ;
        default:
          // OPEN_WAIT || CLOSE_WAIT || PARTIAL_OPEN:
          setBsmState(blind, OPENING);
          doOpen(blind) ;
      }

    }

  } else if ( closeSwitch()) {   //CLOSE button pressed

    Serial.println(F("Close button press"));
    lastAction = millis() ;
    lastActionWasManual = true ;
    stopTsm() ;  //stop all timers

    for (int blind = 0; blind < numBlinds; blind++) {
      motorRun(blind, STOP) ;

      switch (blindStateMachine[blind]) {
        case CLOSED:
#ifdef DEBUG
          Serial.print(F("Blind ")); Serial.print(blind); Serial.println (F(" already Closed"));
#endif
          continue ;
        case OPEN:
          setBsmState(blind, CLOSING) ;
          doClose(blind) ;
          continue ;
        case CLOSING:
          setBsmState(blind, PARTIAL_OPEN);
          continue ;
        case OPENING:
          setBsmState(blind, PARTIAL_OPEN);
          continue ;
        case SUNDIP:
          setBsmState(blind, CLOSING) ;
          doClose(blind) ;
          continue ;
        default:
          // OPEN_WAIT || CLOSE_WAIT || PARTIAL_OPEN:
          setBsmState(blind, CLOSING) ;
          doClose(blind) ;
      }
    }

  }


  /************************************************************************************************************************
     AUTOMATIC MODES  0-HALT, 1-Close Only, 2-Close & Sundip, 3-Open,Close & Sundip
     Logic should work like this (depending on which mode we are in)
     If CLOSED we can OPEN
     If OPEN we can CLOSE, SUNDIP
     if SUNDIPPED we can OPEN, CLOSE
     if PARTIAL_OPEN we can CLOSE only  (if blind manually opened or closed, don't want to OPEN until full close cycle)
   ************************************************************************************************************************/

  // Read the LDR
  ldrVal = readLDR() ;

  if (config.opMode >= 1) {
    if (blindStateMachine[0] != OPENING && blindStateMachine[0] != CLOSING && blindStateMachine[0] != SUNDIPPING) {
      //Only validate states when blind is finished moving
      // Update the timer state machine
      updateTsm() ;

    }
  }

  //Ensure automatic actions only occur either manWaitMin after last manual action
  if (lastActionWasManual && millis() - lastAction > ((long)config.manWaitMin * 60000ul) ) {
    lastActionWasManual = 0 ;
  }

  if ((! lastActionWasManual) && millis() - lastAction > ((long)config.autoWaitMin * 60000ul) ) {
    //Ensure automatic actions only occur autoWaitMin after last auto action

    //if (lastAction < now - autowait) && (lastAction < now - manwait)

    /************************************************************************************************************************
       Opmode 1
       Auto Close Only
     ************************************************************************************************************************/
    if (config.opMode >= 1) {
      //Mode 1 - close only
      // +auto close logic

      if ( blindStateMachine[0] == OPEN || blindStateMachine[0] == PARTIAL_OPEN || blindStateMachine[0] == SUNDIP ) {

        if (ldrVal > config.closeThreshold && timerStateMachine == CLOSE_EXPIRE) {
          //close the blind it's dark for long enough
          Serial.println(F("Auto Closing...")) ;
          setBsmState(0, CLOSING);
#ifdef DUALBLINDS
          setBsmState(1, CLOSING);
#endif
          stopTsm() ;
          lastAction = millis();
        } else if (ldrVal > config.closeThreshold && timerStateMachine != CLOSE_WAIT && timerStateMachine != CLOSE_EXPIRE) {
          startTsm(CLOSE_WAIT) ;
#ifdef DEBUG
          Serial.println(F("Timer config.autoCloseMin triggered"));
#endif
        } else if (ldrVal < config.closeThreshold && timerStateMachine == CLOSE_WAIT) {
          // Not dark enough
          stopTsm() ;
        }

      }

    }

    /************************************************************************************************************************
       Opmode 2
       Auto Close & Sundip
     ************************************************************************************************************************/
    if (config.opMode >= 2 && config.sunDipPosition != 0 ) {
      //Mode 2 close & sundip
      // +sundip logic
      if ( blindStateMachine[0] == OPEN ) {
        if (ldrVal < config.sundipThreshold && timerStateMachine == SUNDIP_EXPIRE) {
          //Sundip checks first - do the dip..
          Serial.println(F("Sundipping...")) ;
          //We only sundip the main blind (although the exit sundip code exits both blinds)
          setBsmState(0, SUNDIPPING) ;
          stopTsm() ;
          lastAction = millis() ;

        } else if ( ldrVal < config.sundipThreshold && timerStateMachine != SUNDIP_WAIT ) {
          startTsm(SUNDIP_WAIT) ;
#ifdef DEBUG
          Serial.println(F("Timer config.sundipThreshold triggered"));
#endif
        } else if ( ldrVal  > config.sundipThreshold && timerStateMachine == SUNDIP_WAIT ) {
          stopTsm() ;
        }
      }

      if ( blindStateMachine[0] == SUNDIP) {


        if (timerStateMachine == OPEN_EXPIRE) {
          Serial.println(F("Auto Opening (Sundip mode exiting)...")) ;
          setBsmState(0, OPENING) ;
#ifdef DUALBLINDS
          setBsmState(1, OPENING) ;
#endif
          stopTsm() ;
          lastAction = millis();
        } else if (timerStateMachine == CLOSE_EXPIRE) {
          Serial.println(F("Auto Closing (Sundip mode exiting)...")) ;
          setBsmState(0, CLOSING) ;
#ifdef DUALBLINDS
          setBsmState(1, CLOSING) ;
#endif
          stopTsm() ;
          lastAction = millis();//if OPEN_EXPIRE
        }


        if ( (ldrVal > config.sundipThreshold && ldrVal < config.closeThreshold)  && timerStateMachine != OPEN_WAIT) {
          // If its not dark enough to close but not bright enough for sundip, then open the blind
          startTsm(OPEN_WAIT) ;
#ifdef DEBUG
          Serial.println(F("Begin exit SUNDIP - timer started config.autoOpenMin triggered"));
#endif
        } else if ( (ldrVal > config.sundipThreshold && ldrVal > config.closeThreshold)  && timerStateMachine != CLOSE_WAIT) {
#ifdef DEBUG
          Serial.println(F("Begin exit SUNDIP - timer started config.autoCloseMin triggered"));
#endif
          startTsm(CLOSE_WAIT) ;
        } else if ( (ldrVal < config.sundipThreshold) && (timerStateMachine == OPEN_WAIT || timerStateMachine == CLOSE_WAIT) ) {
          //sun is out again, stay in sundip
          stopTsm() ;
        }


      }//if SUNDIP


    } //if opmode>=2


    /************************************************************************************************************************
       Opmode 3
       Auto Close, Sundip & Open
     ************************************************************************************************************************/
    if (config.opMode >= 3 ) {
      //Mode 3 open, close & sundip
      // +auto open logic

      if ( blindStateMachine[0] == CLOSED ) {

        if (ldrVal < config.openThreshold && timerStateMachine == OPEN_EXPIRE )  {
          //If its bright, only open if we are fully closed (so we don't open sundip or where blind has been partially closed manually)
          //Consider SUNDIP_EXPIRE here too, as it could be triggered due to light level being higher that config.openThreshold
          Serial.println(F("Auto Opening")) ;
          setBsmState(0, OPENING) ;
#ifdef DUALBLINDS
          setBsmState(1, OPENING) ;
#endif
          stopTsm() ;
          lastAction = millis();
        } else if (ldrVal < config.openThreshold && timerStateMachine != OPEN_WAIT ) {
          startTsm(OPEN_WAIT) ;
#ifdef DEBUG
          Serial.println(F("Timer config.autoOpenMin triggered"));
#endif
        } else if (ldrVal > config.openThreshold && timerStateMachine == OPEN_WAIT) {
          // Not bright enough
          stopTsm() ;
        }

      }

    }

  }  //(if millis - lastAction..)

#ifdef WX_REPEATER

  //Has anything talked to us via RF ?
  if (rxMode == FSK) {
    doRfRecv() ;
  }
#else
  doRfRecv() ;
#endif

  if (! (motorRunning[0] || motorRunning[1])  ) {
    blinkLED() ;
  }

  if (millis() - lastStatusPrint > STATUS_INTERVAL) {
    tSensors.requestTemperatures();   //this must be done here not in reportState as it screws up timing upon state change for dual blinds
    reportState() ;
    lastStatusPrint = millis() ;
  }


#ifdef WX_REPEATER
  //don't go into the WxLoop if blind is travelling - we may miss polling the limit switches
  if (blindStateMachine[0] != OPENING && blindStateMachine[0] != CLOSING && blindStateMachine[0] != SUNDIPPING ) {
    byte wxState = wxLoop() ;
    if (wxState == 1 ) {
      //not synced
      LED(1) ;
    } else if (wxState == 2 ) {
      //synced but nothing received
      //don't touch the LED - let normal blind flash mode work
    } else if (wxState == 3 ) {
      //received a packet for our synced Wx station
      LED(1) ;
      delay (100) ;
      LED(0) ;
      delay (100) ;
      LED(1) ;
      delay (100) ;
      LED(0) ;

    }
  }
#endif



} //loop

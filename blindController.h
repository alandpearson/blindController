/* 
 *  blindController structs, default timers, room definitions and state machine definitions
 */


//Room Definitions


#undef DUALBLINDS
#undef LIMIT_SWT_IO_EXP
#undef OPENCLOSE_SWT_IO_EXP
#undef DEBUG


#ifdef LOUNGE
#define MYNODEID 2
#define DUALBLINDS
#define LIMIT_SWT_IO_EXP
#define I2C_ADDR 0x26

#elif defined MASTERBED
#define MYNODEID 3
#define I2C_ADDR 0x20

#elif defined SNUG
#define MYNODEID 4
#define OPENCLOSE_SWT_IO_EXP
#define I2C_ADDR 0x26

#elif defined AINEBED
#define MYNODEID 6
#define I2C_ADDR 0x20

#elif defined GUESTBED
#define MYNODEID 7
#define I2C_ADDR 0x20

#elif defined ALOFFICE
#define MYNODEID 8
#define I2C_ADDR 0x20
//also do double duty as weather station repeater
#define WX_REPEATER
#define BMP280
#define DEBUG

#elif defined DINING
#define MYNODEID 9
#define I2C_ADDR 0x20

#elif defined DAITHIBED
#define MYNODEID 10
#define I2C_ADDR 0x20

#elif defined UPPERHALL
#define MYNODEID 11
#define I2C_ADDR 0x20

#elif defined KITCHEN
#define MYNODEID 12
#define I2C_ADDR 0x20
#define DUALBLINDS
#define LIMIT_SWT_IO_EXP

#elif defined ANNEOFFICE
#define MYNODEID 13
#define I2C_ADDR 0x20

#elif defined TESTBOX
#define MYNODEID 15
#define OPENCLOSE_SWT_IO_EXP
#define I2C_ADDR 0x26

#else
#define BREADBOARD
#define MYNODEID 15
#define DEBUG
#define WX_REPEATER
#undef BMP280

#define I2C_ADDR 0x26
#endif



/*
  Trigger timers
   Timers are in seconds
   If an event happens (e.g Bright Sun) then it must stay
   at the threshold for the amount of TrigSec to be valid.
   Avoid opening and closing the blind everytime a cloud appears.
*/
#define AUTO_OPEN_MIN 5       //How long should light intensity be above OPEN_THRESH for before opening blind (5 mins) (also used for sundip exiting)
#define AUTO_CLOSE_MIN 5      //How long should light intensity be below CLOSE_THRESH for before closing blind (5 mins) 
#define SUNDIP_SEC  30        //How long should light intensity be above SUNDIP_THRESH for before closing blind (30 secs)

/* Light thresholds to trigger blind movements */
#define OPEN_THRESH     500     //Light intensity to trigger blind opening
#define CLOSE_THRESH    800     //Light intensity to trigger blind closing
#define SUNDIP_THRESH   30     //Light intensity to trigger blind sundiping

/* Limits */
#define CLOSE_MAX_MSEC 50000     //Max time allowed to close blind (45 secs) - if you go above 65, you need to change openTime to LONG var type
#define OPEN_MAX_MSEC  60000     //Max time allowed to open blind (60 secs) - if you go above 65, you need to change closeTime to LONG var type
#define AUTO_WAIT_MIN 10 //Do not take automatic actions greater than this period (ten minutes)
#define MAN_WAIT_MIN 60  //Do not take automatic actions after a manual operation for this time 1hr



/* Misc */
#define STATUS_INTERVAL 600000    //Update status every 10 mins (and when state changes)
#define POS_UPDATE_INTERVAL 1000  //Update blindPos once every 1 sec
#define pcbLEDPin 9    //Onboard LED
#define SUNDIP_POS 25      //What % closed blind should move to when high light intensity sensed
#define UNDERRUN 3      // 0 minus this value is the min position % the blind can run to (in case top limit switch missed)
#define OVERRUN 1       // 100 plus this value is the max position % the blind can run to (in case bottom limit switch missed)

#define LEDBLINK_INTERVAL 15000 //msecs between LED flashes in normal operation

enum motorDirections  { UP, DOWN, STOP }  ;
enum blindStateMachines { CLOSED, OPEN, SUNDIP, CLOSING, OPENING, SUNDIPPING, PARTIAL_OPEN, NOTPRESENT, HALT };
enum timerStateMachines {SUNDIP_WAIT, OPEN_WAIT, CLOSE_WAIT, SUNDIP_EXPIRE, OPEN_EXPIRE, CLOSE_EXPIRE, TSTOP } ;
/*
   timerStateMachine machine
   OPEN_WAIT - Auto Open Evaluation (wait for AUTO_OPEN_TIMER to expire)
   CLOSE_WAIT -Auto Close Evaluation (wait for AUTO_CLOSE_TIMER to expire)
*/


//report to emoncms structure
typedef struct
{
  byte destID = 0 ;
  byte opMode = 0 ;
  int bsm0 = 7 ; //NOTPRESENT (updated when initiated)
  int bsm1 = 7 ; //NOTPRESENT (updated when initiated)
  int tsm = 6 ;  // TSTOP
  int ldrVal = 0;
  int temp = -1275 ;  //temp * 10 from DS18B20 (-127.5 = sensor not present)
  float position0 = 0;
  float position1 = 0 ;
  int fwVersion = VERSION ;
} blindReportPayload ;

typedef struct {
  byte nodeId ;            // used by rf12_config, offset 0
  byte autoOpenMin ;       // //How long should light intensity be above OPEN_THRESH for before opening blind (5 mins) (also used for sundip exiting)
  byte autoCloseMin ;     //How long should light intensity be below CLOSE_THRESH for before closing blind (5 mins)
  word sunDipSec ;     // How long should light intensity be above SUNDIP_THRESH for before closing blind (30 sec)
  byte sunDipPosition ;  //What % closed blind should move to when high light intensity sensed
  word openThreshold ;  //Light intensity to trigger blind opening
  word closeThreshold ; //Light intensity to trigger blind closing
  word sundipThreshold ; ////Light intensity to trigger blind sundipping
  byte autoWaitMin ; //Do not take automatic actions greater than this period
  byte manWaitMin ; //Do not take automatic actions after a manual operation for this time
  byte opMode ;     //Operation mode, 0=fully automatic, 1=close only
  byte underRun ;   // 0 minus this value is the minimum position % the blind can run to (in case top limit switch missed)
  byte overRun ;    // 100 plus this value is the maximum position % the blind can run to (in case bottom limit switch missed)
  word crc;
  //byte pad[RF12_EEPROM_SIZE - 8];

} blindConfig;

typedef struct {
  //   Timers of how long blind takes to operate
  unsigned int openTime[2] ; //How long blind takes to open
  unsigned int closeTime[2] ; //How long blind takes to close
} blindCalibration;


typedef struct {
  byte destID ;
  byte cmd ;    // 1 = open, 2 = close, 3 = sundip, 4 = new config, 5 Reset default config, 6 = new opmode
  byte opMode ;
  blindConfig newConfig ;
} EmoncmsPayload;


enum {
  MCP_IODIR, MCP_IPOL, MCP_GPINTEN, MCP_DEFVAL, MCP_INTCON,
  MCP_IOCON, MCP_GPPU, MCP_INTF, MCP_INTCAP, MCP_GPIO, MCP_OLAT
};

enum blindStateMachines blindStateMachine[2] = {CLOSED, CLOSED} ;
enum blindStateMachines oldBsmState[2] = {CLOSED, CLOSED};

enum timerStateMachines timerStateMachine = TSTOP ;
enum motorDirections motorDirection = STOP ;

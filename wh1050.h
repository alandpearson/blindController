/// @dir wh1050_relay
///2020-12-24 alandpearson@gmail.com
/// Based on the feneralized decoder and relay for 868 MHz and 433 MHz OOK signals.
// 2010-04-11 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php




// RF12 communication settings
//Set the RFM12B to Jeenode OOK RSI connection
#define OOK_INT 3    //3 = D3 = ext int 1 = JeeNode P1-4 IRQ
//4 = D4 = JeeNode P1 - DIO
//7 = D7 = JeeNode P4 - DIO
//14 = A0/D14 = Jeemode P1 - AIO
//17 = A3/D17 = Jeemode P4 - AIO

#include "decoders.h"
#include "rf12_helper.h"


// I/O pin allocations, leave any of these undefined to omit the code for it
//NB if using RFM12 as OOk receiver, it needs to be connected to an Arduino pin
//So you must also use PIN_433 to tell the code which pin the RFM12 is connected to


#define SYNC_EFFORTS 2  // during sync, how many times we need to receive a broadcast from same WH1050 Id

#define SYNC_TIMEOUT_SECS 600 // We must see the same WH1050 Id in this time, or we will try to resync
//The below settings are important. The WH1050 will send data every 48 secs
//Ideal is in you catch that window on the nail, but that isn't always possible
//So what you want to do is turn on OOK a few secs before we expect the packet
//and run in OOK mode for a few secs after we would expect it
#define OOK_ON_DURATION 3 // Secs we are in OOK receive mode (maximum) - Will be less if we recieve a packet


//WX_SYNC_LISTEN_SECS
//The max time we can stay in OOK listening for a Weather station
//This will be applied on a hourly basis
//This has the affect if we cannot find a station in WX_SYNC_LISTEN_SECS
//then we must wait a full hour to try to sync again
//This value should not be too big, or code that uses this library
//will not be able to send/receive in FSK mode for the duration
#define WX_SYNC_LISTEN_SECS 180  //Maximum time we can be looking for a weather station while syncing (seconds)
//We don't want this too long as we will be in OOK mode all this time,
//meaning FSK reception not possible and interrupts going mad, maybe screwing blind timers.

bool wxReady = false ;        // If data has been received via OOK Decoder and ready to be processed
static uint8_t wxPacket[9];
uint32_t wxLastHeard = 0 ;    // Last millis() we heard a broadcast from any wx station
uint32_t mywxLastHeard = 0 ; // Last millis() we heard a broadcast from wx station we are synced to

static uint8_t prev_rssi = 0; //last packet width
uint32_t  lastRtcUpdate = 0 ; // Last millis() we heard an RTC packet via RF network
uint32_t fskOnTime = 0 ;      //Time we switched RF12 to FSK
uint32_t ookOnTime = 0 ;      //Time we switched RF12 to OOK
DateTime dcfData ;            //Struct to hold the DCF data to be sent via OOK

enum rxModes { FSK, OOK };
rxModes rxMode ;


WH1080DecoderV2a wh1080v2a_9(9);
PayloadRTC rtcData ;


//Do we have onbaord BMP280 pressure sensor ?
#ifdef BMP280
#include <BMP280_DEV.h>                           // Include the BMP280_DEV.h library
BMP280_DEV bmp280;                                // Instantiate (create) a BMP280_DEV object and set-up for I2C operation
#endif




#ifdef DEBUG

void reportSerial (const char* s, class DecodeOOK& decoder) {
  byte pos;
  const byte* data = decoder.getData(pos);
  Serial.print(s);
  Serial.print(":[");
  for (byte i = 0; i < pos; ++i) {
    //Serial.print(data[i] >> 4, HEX);
    //Serial.print(data[i] & 0x0F, HEX);
    Serial.print(data[i], HEX) ;
    Serial.print(':') ;
  }
  Serial.println(']');
}

#endif

void sendWxData(wh1050Payload data) {

  if (rxMode == FSK ) {
    rf12_initialize(WXNODEID, RF12_433MHZ, NETGRP);
    delay(500) ;
    while (!rf12_canSend())
      rf12_recvDone();
    rf12_sendStart(WXNODEID, &data, sizeof data);
    rf12_sendWait(1);
    Serial.println(F("Sent RF WX reading")) ;
  } else {
    Serial.println(F("Cant RF send - not in FSK mode!")) ;
  }

  rf12_initialize(MYNODEID, RF12_433MHZ, NETGRP);


}


void fskReceiveOn() {

  if (rxMode != FSK) {
    rf12_stop_rx_OOK();
    rf12_initialize(MYNODEID, RF12_433MHZ, NETGRP);
#ifdef DEBUG
    Serial.println(F("FSK mode")) ;
    delay(500) ;
#endif

    rxMode = FSK ;
    fskOnTime = millis() ;
  }
}

void ookReceiveOn() {

  if (rxMode != OOK) {
    //  rf12_initialize(NODEID, RF12_433MHZ, NETGRP);
    rf12_initialize(0, RF12_433MHZ, 0);

    delay (250) ;
    rf12_start_rx_OOK() ;
#ifdef DEBUG
    Serial.println(F("OOK mode")) ;
#endif

    rxMode = OOK ;
    ookOnTime = millis() ;
  }
}



bool pollOOK() {
  // Read the RSSI value from the last interrupt
  blockInterrupts();
  unsigned long rssi_diff = rf12_pulse;
  unsigned long rssitime = rf12_tstamp;
  uint8_t rssi = rf12_signal;
  allowInterrupts();

  if (rssi != prev_rssi) {

    if (rssi_diff > 32000)
      rssi_diff = 32000;

    if (wh1080v2a_9.nextPulse(rssi_diff, prev_rssi) ) {
      byte num_bytes = 0;
      const uint8_t* bytes = wh1080v2a_9.getData(num_bytes);
      memcpy(wxPacket, bytes, num_bytes);
#ifdef DEBUG
      reportSerial("WH1080DecoderV2a", wh1080v2a_9); //Note: clears part of data buffer!
#endif
      wh1080v2a_9.resetDecoder();
      wxLastHeard = millis() ;

      wxReady = 1;
    }

  }

  prev_rssi = rssi;

  return (wxReady) ;

}



bool wxSync(bool resync = 0 ) {


  static byte count = 0 ;
  static byte wxStnID = 0, lastwxStnID = 0 ;
  static byte synced = 0 ;
  static byte syncing = 0 ;
  static uint32_t nextSyncAttempt = 0 ;
  static uint32_t nextOOKonTime = 0 ;



  if ( (millis() - mywxLastHeard) > (SYNC_TIMEOUT_SECS * 1000ul) ) {
    synced = 0 ;
  }

  if (! synced && millis() > nextSyncAttempt || resync ) {

    //Get syncing !

    if (resync) {
      //reset the last ookOnTime timestamp
      //by entering FSK then OOK mode
      fskReceiveOn() ;
    }

    ookReceiveOn() ;

    if (millis() - ookOnTime >  (WX_SYNC_LISTEN_SECS * 1000ul) ) {
      //We've been in syncing too long and not found a Weather Station
      //Give up for another hour
      nextSyncAttempt = millis() + 3600000ul ;
      nextOOKonTime = nextSyncAttempt ;
      Serial.println(F("wxSync: Give up sync for 1 hr")) ;
      fskReceiveOn() ;
    } else {
      //Do the sync dance

      if (!synced && !syncing) {
        Serial.println(F("wxSync: Resyncing")) ;
        count = 0 ;
      }

      if ( pollOOK() && (count < SYNC_EFFORTS) ) {

        //only do this so we can display the decoded packet if in DEBUG mode
        wh1050Payload wxReading = decodeSensorData(wxPacket) ;

        wxStnID = wxReading.stnID ;
        Serial.print(F("Sync: heard Wx stn ID:")) ; Serial.println(wxStnID, HEX);

        if ( wxStnID == lastwxStnID ) {
          lastwxStnID = wxStnID ;
          mywxLastHeard = millis() ;
          count ++ ;
        } else {
          lastwxStnID = wxStnID ;
          count = 1;
        }

        wxReady = 0;
      }

      // Have we got a valid (same) station ID at least 3 times in a row ?

      if (count >= SYNC_EFFORTS && wxStnID == lastwxStnID ) {
        // good - we found a station SYNC_EFFORTS times in a row, set the global var to be this stations ID
        myStnId = wxStnID ;
        if ( count == SYNC_EFFORTS) {
          //only print this message once per sync
          Serial.print(F("Synced to Wx stn ID:")) ; Serial.println(wxStnID, HEX) ;
          count ++;
        }
        synced = 1;
        syncing = 0 ;
      } else {
        synced = 0 ;
        syncing = 1;
      }

    }

  } else {
    //Synced or not trying to sync.
    //Manage going in and out of OOK mode
    //every 48 seconds for OOK_ON_DURATION
    //Turn OOK mode on 48secs - OOK_ON_DURATION / 2
    //to get OOK each side of the window we expect the packet

    //if we find OOK has been on for more than OOK_ON_DURATION
    //turn it off. In this case, we did not receive a packet
    //as expected, so recalculate the next expected time

    //only turn OOK on at nextOOKonTime (avoids going on/off when not synced and giving up for 1 hour above)

    //The next time we expect a packet
    //As OOK_ON_DURATION is in seconds, multiply *500 will give us half the value in millisecs
    if (millis() - wxLastHeard < 48000  && nextOOKonTime < millis() ) {
      nextOOKonTime = wxLastHeard + ( 48000ul - (OOK_ON_DURATION * 500ul) ) ;
    } else if ( nextOOKonTime < millis() ) {
      nextOOKonTime = nextOOKonTime + (48000ul) ;
      //do not add on the OOK_ON_DURATION as this has been factored in
      //above. Otherwise we would drift by OOK_ON_DURATION/2 everytime a packet is missed
    }


    //Make sure we go back into FSK mode if OOK has been on too long

    if ( rxMode == OOK && millis() > ookOnTime + (OOK_ON_DURATION * 1000ul) ) {
      fskReceiveOn() ;
    } else if (rxMode == FSK && millis() >= nextOOKonTime) {
      ookReceiveOn() ;
    }

  }

  return (synced) ;

}



void wxSetup() {
  Serial.println(F("\nwxRelay start up.."));



  //Start the emulated RTCClock
  RTC_Millis::begin (&dcfData) ;



#ifdef BMP280

#ifdef DEBUG
  Serial.println(F("BMX280 sensor setup")) ;
#endif

  bmp280.begin(BMP280_I2C_ALT_ADDR);      // Default initialisation, place the BMP280 into SLEEP_MODE
  bmp280.setTimeStandby(TIME_STANDBY_4000MS);     // Set the standby time to 4 seconds
  bmp280.startNormalConversion();                 // Start BMP280 continuous conversion in NORMAL_MODE

#ifdef DEBUG
  Serial.println(F("BMX280 setup done")) ;
#endif


#endif

  ookReceiveOn() ;


  Serial.println(F("wxRelay started")) ;
}


byte wxLoop() {

  byte state = 0 ; //used as return value so caller can see our state


  //Find a wh1050 weather station and keep us in sync with it
  if ( wxSync(0) ) {
    state = 2;

    //If we received wh1050 data (received & decoded successfully)
    if ( pollOOK() ) {


      //put the nice data into a wh1050Payload struct
      wh1050Payload wxReading = decodeSensorData(wxPacket) ;

#ifdef BMP280
      bmp280.getCurrentTempPres( wxReading.intempC, wxReading.hpa)  ;
#ifdef DEBUG
      Serial.print(wxReading.hpa); Serial.print(F(" hpa, ")) ;
      Serial.print(wxReading.intempC); Serial.println(F(" C"));
#endif
#endif

      wxReady = false ;


      if (wxReading.stnID == myStnId ) {

        state = 3 ;

        mywxLastHeard = wxLastHeard ;
        //bang out a DCF time sync packet if we received a RTC packet from emonhub in the last 24 hours
        //& minutes between 3 & 8 so as not to clog the airwaves
        dcfData = RTC_Millis::now () ;

        if (millis() < lastRtcUpdate + 86400000ul && lastRtcUpdate != 0 && (dcfData.minute() >= 3 && dcfData.minute() <= 8) ) {
          delay(500) ;

          sendDcfPacket( myStnId, dcfData );
#ifdef DEBUG
          Serial.println(F("Sent DCF data")) ;
#endif
          delay(300) ;

        }

        //Go into FSK mode immediately after packet RX
        fskReceiveOn() ;


        //Send Wx packet out via RFM12
        sendWxData(wxReading) ;

      } else {
#ifdef DEBUG
        //Heard another wx station - don't update mywxLastHeard
        Serial.println(F("Got Wx data for ID not synced to - ignored")) ;
#endif
      }


    } //if (pollOOK())


  } else {

    //Have not found a Wx Station
    state = 1 ;

  }


  return (state) ;


}

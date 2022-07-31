/// @file
/// Generalized decoder framework for 868 MHz and 433 MHz OOK signals.
// 2010-04-11 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <Arduino.h>
#include <util/crc16.h>

#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))


int myStnId = 00 ;

typedef enum
{
  MsgType_Sensor = 0x5,
  MsgType_DCF    = 0x6
} MsgType;

typedef struct
{
  // byte destID = 0 ; // when sending to EMON we change this to be the address of EMON node
  byte stnID = 0 ; //  WH1050 station id
  byte lowBatt = 0 ; // battery status
  float tempC = 0 ; // temp in Deg C
  byte relH = 0 ; // rel humidity
  float windAvg = 0 ; // wind avg m/s
  float windGust = 0 ; // wind gust m/s
  float rain = 0 ;    // rain in mm as reported by sensor, cumlative
#ifdef BMP280
  float hpa = 0 ;     //Pressure  (from BMP280 sensor)
  float intempC = 0 ; //inside temp (from BMP280 sensor)
#endif  
  long  lastHeard = 0 ;  //millis from when we last heard from this station
} wh1050Payload ;



/// This is the general base class for implementing OOK decoders.
class DecodeOOK {
  protected:
    byte total_bits, bits, flip, state, pos, data[25];
    // the following fields are used to deal with duplicate packets
    word lastCrc, lastTime;
    byte repeats, minGap, minCount;

    // gets called once per incoming pulse with the width in us
    // return values: 0 = keep going, 1 = done, -1 = no match
    virtual char decode (word width) = 0;

    // add one bit to the packet data buffer
    virtual void gotBit (char value) {
      total_bits++;
      byte *ptr = data + pos;
      *ptr = (*ptr >> 1) | (value << 7);

      if (++bits >= 8) {
        bits = 0;
        if (++pos >= sizeof data) {
          resetDecoder();
          return;
        }
      }
      state = OK;
    }

    // store a bit using Manchester encoding
    void manchester (char value) {
      flip ^= value; // manchester code, long pulse flips the bit
      gotBit(flip);
    }

    // move bits to the front so that all the bits are aligned to the end
    void alignTail (byte max = 0) {
      // align bits
      if (bits != 0) {
        data[pos] >>= 8 - bits;
        for (byte i = 0; i < pos; ++i)
          data[i] = (data[i] >> bits) | (data[i + 1] << (8 - bits));
        bits = 0;
      }
      // optionally shift bytes down if there are too many of 'em
      if (max > 0 && pos > max) {
        byte n = pos - max;
        pos = max;
        for (byte i = 0; i < pos; ++i)
          data[i] = data[i + n];
      }
    }

    void reverseBits () {
      for (byte i = 0; i < pos; ++i) {
        byte b = data[i];
        for (byte j = 0; j < 8; ++j) {
          data[i] = (data[i] << 1) | (b & 1);
          b >>= 1;
        }
      }
    }

    void reverseNibbles () {
      for (byte i = 0; i < pos; ++i)
        data[i] = (data[i] << 4) | (data[i] >> 4);
    }

    bool checkRepeats () {
      // calculate the checksum over the current packet
      word crc = ~0;
      for (byte i = 0; i < pos; ++i)
        crc = _crc16_update(crc, data[i]);
      // how long was it since the last decoded packet
      word now = millis() / 100; // tenths of seconds
      word since = now - lastTime;
      // if different crc or too long ago, this cannot be a repeated packet
      if (crc != lastCrc || since > minGap)
        repeats = 0;
      // save last values and decide whether to report this as a new packet
      lastCrc = crc;
      lastTime = now;
      return repeats++ == minCount;
    }

  public:
    enum { UNKNOWN, T0, T1, T2, T3, OK, DONE };

    DecodeOOK (byte gap = 5, byte count = 0)
      : lastCrc (0), lastTime (0), repeats (0), minGap (gap), minCount (count)
    {
      resetDecoder();
    }

    virtual bool nextPulse (word width) {
      if (state != DONE)
        switch (decode(width)) {
          case -1: // decoding failed
            resetDecoder();
            break;
          case 1: // decoding finished
            while (bits)
              gotBit(0); // padding
            state = checkRepeats() ? DONE : UNKNOWN;
            break;
        }
      return state == DONE;
    }

    const byte* getData (byte& count) const {
      count = pos;
      return data;
    }

    virtual void resetDecoder () {
      total_bits = bits = pos = flip = 0;
      state = UNKNOWN;
    }
};

// 433 MHz decoders


uint8_t crc8( uint8_t *addr, uint8_t len)
{
  uint8_t crc = 0;

  // Indicated changes are from reference CRC-8 function in OneWire library
  while (len--) {
    uint8_t inbyte = *addr++;
    for (uint8_t i = 8; i; i--) {
      uint8_t mix = (crc ^ inbyte) & 0x80; // changed from & 0x01
      crc <<= 1; // changed from right shift
      if (mix) crc ^= 0x31;// changed from 0x8C;
      inbyte <<= 1; // changed from right shift
    }
  }
  return crc;
}


//Alecto WS3000, WS4000, Fine Offset WH1080,WH1050 ....
//Only uses pulse widths, no signal level information
//Detection on preample of 8 bits one.
//Message end on crc8
//Capabable of decoding both 9 and 10 byte messgaes (WS3000, WS4000)
class WH1080DecoderV2 : public DecodeOOK {
  protected:
    byte msglen;
  public:
    WH1080DecoderV2 (byte msg_len = 10, byte gap = 5, byte count = 0) : DecodeOOK(gap, count), msglen(msg_len) {}

    //Dallas One-Wire CRC-8 - incremental calculation.
    uint8_t crc8_update( uint8_t crc, uint8_t inbyte)
    {
      uint8_t i;
      for (i = 8; i; i--) {
        uint8_t mix = (crc ^ inbyte) & 0x01;
        crc >>= 1;
        if (mix) crc ^= 0x8C;
        inbyte >>= 1;
      }
      return crc;
    }

    //Dallas One-Wire CRC-8.
    uint8_t crc8( const uint8_t *addr, uint8_t len)
    {
      uint8_t crc = 0;
      while (len--) {
        uint8_t inbyte = *addr++;
        crc = crc8_update(crc, inbyte);
      }
      return crc;
    }

    // see also http://lucsmall.com/2012/04/29/weather-station-hacking-part-2/
    // 200 < bit-1 < 800 < low < 1200 < bit-0 < 1700
    virtual char decode (word width) {
      if (140 <= width && width < 2500) {
        byte w = width >= 1000;
        //option 1: only looking at durations
        byte is_low = (1100 <= width && width < 1500);
        switch (state) {
          case UNKNOWN:
            if (!is_low) {

              if (!w) {
                // Short pulse = 1
                ++flip;
                if (flip >= 15) {
                  flip = 0;
                  state = T0;
                }
              } else {
                // Long pulse. Reset decoder
                return -1;
              }
            }  else {
              ++flip;
            }
            break;
          case OK:
            if (!is_low) {
              gotBit(!w);
              state = T0;
            } else {
              //expecting high signal, got low
              return -1;
            }
            break;
          case T0:
            if (is_low) {
              state = OK;
            } else {
              //expecting low signal, got high
              return -1;
            }
            break;
        }
      } else {
        //signal (low or high) out of spec
        return -1;
      }
      if (total_bits >= msglen * 8) {
        if (crc8(data, msglen - 1) == data[msglen - 1]) {
          reverseBits();
          return 1;
        } else {
          //failed crc at maximum message length
          return -1;
        }
      }
      return 0;
    }
};


//Alecto WS3000, WS4000, Fine Offset WH1080, ....
//Uses signal level indformation. More tolerant to jitter.
//Preamble detection fuzzier to allow receiver AGC to work
//Message end on crc8
//Capabable of decoding both 9 and 10 byte messgaes (WS3000, WS4000)
class WH1080DecoderV2a : public WH1080DecoderV2 {
  protected:
    byte last_signal;

  public:
    WH1080DecoderV2a(byte msg_len = 10, byte gap = 5, byte count = 0) : WH1080DecoderV2(msg_len, gap, count) {}

    virtual bool nextPulse (word width, byte signal) {
      last_signal = signal;
      return DecodeOOK::nextPulse(width);
    }

    // see also http://lucsmall.com/2012/04/29/weather-station-hacking-part-2/
    // 200 < bit-1 < 800 < low < 1200 < bit-0 < 1700
    virtual char decode (word width) {
      if (200 <= width && width < 2500) {
        //Serial.println(width) ;
        byte w = width >= 1000;
        //option 2: having knowledge on the signal value, we can allow for more jitter on timing
        byte is_low = !last_signal;
        switch (state) {
          case UNKNOWN:
            //Get preamble
            //Basically it looks at the previous bit and expects the next bit to be the opposite
            //8 times (is_low is the opposite of the last signal)
            if (!is_low) {
              if (!w) {
                // Short pulse = 1
                ++flip;
                if (flip >= 8) {
                  flip = 0;
                  state = T0;
                }
              } else {
                //We have a long pulse (zero)
                if (flip > 3) {
                  //if we got 4 preambles (ie. 4 high bits in a row) 
                  //then that's enough. It can take receiver AGCs time
                  //to ramp up and adjust and that can mean we miss some
                  //of the preamble
                  state = T0;
                } else {
                  // Long pulse and we didn't get at least 3 preamble bits
                  // Reset decoder
                  return -1;

                }
              }
            }
            break;
          case OK:
            if (!is_low) {
              gotBit(!w);
              state = T0;
            } else {
              //expecting high signal, got low
              return -1;
            }
            break;
          case T0:
            if (is_low) {
              state = OK;
            } else {
              //expecting low signal, got high
              return -1;
            }
            break;
        }
      } else {
        //signal (low or high) out of spec
        return -1;

      }
      if (total_bits >= msglen * 8) {
        if (crc8(data, msglen - 1) == data[msglen - 1]) {
          reverseBits();
          return 1;
        } else {
          return -1;
        }
      }
      return 0;
    }


};





#ifdef DEBUG
char* formatDouble( double val, byte precision, char* ascii, uint8_t ascii_len) {
  // formats val with number of decimal places determine by precision
  // precision is a number from 0 to 6 indicating the desired decimial places

  snprintf(ascii, ascii_len, "%d", int(val));
  if ( precision > 0) {
    strcat(ascii, ".");
    unsigned long frac;
    unsigned long mult = 1;
    byte padding = precision - 1;
    while (precision--)
      mult *= 10;

    if (val >= 0)
      frac = (val - int(val)) * mult;
    else
      frac = (int(val) - val ) * mult;
    unsigned long frac1 = frac;
    while ( frac1 /= 10 )
      padding--;
    while (  padding--)
      strcat(ascii, "0");
    char str[7];
    snprintf(str, sizeof(str), "%d", frac);
    strcat(ascii, str);
  }
}

#endif

wh1050Payload decodeSensorData( uint8_t* sbuf) {

  wh1050Payload wh1050 ;

  //Not used in WH1050
  char *compass[] = {"N  ", "NNE", "NE ", "ENE", "E  ", "ESE", "SE ", "SSE", "S  ", "SSW", "SW ", "WSW", "W  ", "WNW", "NW ", "NNW"};
  uint8_t windbearing = 0;
  double temperature = 0 ;
  char str[110];
  str[0] = 0;
  // station id
  wh1050.stnID = (sbuf[0] << 4) | (sbuf[1] >> 4);

  // temperature
  temperature = ((sbuf[1] & B00000011) << 8) + sbuf[2];
  temperature = (temperature - 400 ) / 10 ;
  if (sbuf[1] & B00001000) {
    temperature = -temperature;
  }

  wh1050.tempC = temperature ;

  // bit 3 of sbuf[1] is batt status
  if (sbuf[1] & B00000100) {
    wh1050.lowBatt = 1;
  } else {
    wh1050.lowBatt = 0;
  }

  //humidity
  wh1050.relH = sbuf[3] & 0x7F;
  //wind speed
  wh1050.windAvg = sbuf[4] * 0.34;
  //wind gust
  wh1050.windGust = sbuf[5] * 0.34;
  //rainfall
  wh1050.rain = (((sbuf[6] & 0x0F) << 8) | sbuf[7]) * 0.3;
  wh1050.lastHeard = millis() ;


#ifdef DEBUG

  char tstr[6];
  formatDouble(wh1050.tempC, 1, tstr, sizeof(tstr));
  char wsstr[6];
  formatDouble(wh1050.windAvg, 1, wsstr, sizeof(wsstr));
  char wgstr[6];
  formatDouble(wh1050.windGust, 1, wgstr, sizeof(wgstr));
  char rstr[7];
  formatDouble(wh1050.rain, 1, rstr, sizeof(rstr));


  snprintf(str, sizeof(str), "ID: %2X, T=%5s C, relH=%2d%%, Wvel=%5sm/s, Wmax=%5sm/s, Rain=%5smm, LowBatt=%1d",
           wh1050.stnID,
           tstr,
           wh1050.relH,
           wsstr,
           wgstr,
           rstr,
           wh1050.lowBatt);

  Serial.println(str);
#endif

  return wh1050 ;

}


static uint8_t dec2bcd( const uint8_t v )
{
  return ((v / 10) << 4) | (v % 10);
}




// Dumb Arduino IDE pre-processing bug - can't put this in the main source file!
/// Structure used to defined each entry in the decoder table.
typedef struct {
  char typecode;
  const char* name;
  DecodeOOK* decoder;
} DecoderInfo;

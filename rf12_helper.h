#define STATUS_RSSI    (1 << 8)

#if OOK_INT == 3
#define PINCHGINT_OOK 0
#define RF12_RX_EXTINT 1
#elif OOK_INT == 2
#define PINCHGINT_OOK 0
#define RF12_RX_EXTINT 0
#else
#define PINCHGINT_OOK 1
#define RF12_RX_EXTINT 1
#endif

//#define RF12_RX_DATA 3  //D3 = bit 3 in PORTD - JeeNode!
//#define RF12_RX_EXTINT 1

#if OOK_INT < 8
#define RF12_RX_DATA OOK_INT
#define RF12_RX_PIN PIND
//#define RF12_RX_DDR DDRD
#elif OOK_INT < 14
#define RF12_RX_DATA OOK_INT - 8
#define RF12_RX_PIN PINB
//#define RF12_RX_DDR DDRB
#else
#define RF12_RX_DATA OOK_INT - 14
#define RF12_RX_PIN PINC
//#define RF12_RX_DDR DDRC
#endif


volatile static unsigned long   rf12_pulse;
volatile static unsigned long   rf12_signal;
volatile static unsigned long   rf12_tstamp;
static unsigned long            prev_tstamp = 0;
long rf12_dur_sum = 0;
long rf12_flip_count = 0;
long rf12_high_count = 0;
long rf12_low_count = 0;
long rf12_short_count = 0;
long rf12_int_dur = 0;




//PayloadRTC is the same struct that emonGLCD uses for its power 
//and happens to contain a clock update, so we'll sniff it and use it
//for our own RTC too.
typedef struct {
  byte id ;
  byte type ;     //should always be 0A - EmonPi Power Packet
  byte  hour, min, sec, day, month, year ;
  int utilityW, solarW, utilityKwh, solarKwh;
} PayloadRTC;   



static void blockInterrupts () {
#if PINCHGINT_OOK
#if OOK_INT < 8
  bitClear(EIMSK, PCIE2);
#elif OOK_INT < 14
  bitClear(EIMSK, PCIE0);
#else
  bitClear(EIMSK, PCIE1);
#endif
#else
  bitClear(EIMSK, RF12_RX_EXTINT);
#endif
}

static void allowInterrupts () {
#if PINCHGINT_OOK
#if OOK_INT < 8
  bitSet(EIMSK, PCIE2);
#elif OOK_INT < 14
  bitSet(EIMSK, PCIE0);
#else
  bitSet(EIMSK, PCIE1);
#endif
#else
  bitSet(EIMSK, RF12_RX_EXTINT);
#endif
}



#if OOK_INT >= 14
//#define VECT PCINT1_vect
#define VECT ANALOG_COMP_vect
#elif OOK_INT >= 8
#define VECT PCINT0_vect
#else
#define VECT PCINT2_vect
#endif



void rf12_data_int(void) {
  rf12_signal = bitRead(RF12_RX_PIN, RF12_RX_DATA);
  rf12_tstamp = micros();
  // determine the pulse length in microseconds, for either polarity
  rf12_pulse = rf12_tstamp - prev_tstamp;
  prev_tstamp = rf12_tstamp;
  if (rf12_signal) {
    rf12_high_count++;
    //Serial.print(rf12_pulse);
    //Serial.print("-");
  } else {
    rf12_low_count++;
    //Serial.println(rf12_pulse);
  }
  rf12_flip_count++;
  rf12_dur_sum += rf12_pulse;
  if (rf12_pulse < 80)
  {
    rf12_short_count++;
  }
  rf12_int_dur += micros() - prev_tstamp;
  //Serial.println("rf12_data");
}

#if PINCHGINT_OOK
ISR(VECT) {
  rf12_data_int();
}
#endif

void rf12_interruptcontrol(byte attach) {
#if PINCHGINT_OOK
#if OOK_INT < 8
  if (attach) {
    bitClear(DDRD, OOK_INT);      // input
    bitSet(PORTD, OOK_INT);       // pull-up
    bitSet(PCMSK2, OOK_INT);      // pin-change
    bitSet(PCICR, PCIE2);         // enable
  } else
    bitClear(PCMSK2, OOK_INT);
#elif OOK_INT < 14
  if (attach) {
    bitClear(DDRB, OOK_INT - 8);  // input
    bitSet(PORTB, OOK_INT - 8);   // pull-up
    bitSet(PCMSK0, OOK_INT - 8);  // pin-change
    bitSet(PCICR, PCIE0);         // enable
  } else
    bitClear(PCMSK0, OOK_INT - 8);
#else
  if (attach) {
    bitClear(DDRC, OOK_INT - 14); // input
    bitSet(PORTC, OOK_INT - 14);  // pull-up
    //bitSet(PCMSK1, OOK_INT - 14); // pin-change
    //bitSet(PCICR, PCIE1);         // enable

    // enable analog comparator with fixed voltage reference
    ACSR = _BV(ACBG) | _BV(ACI) | _BV(ACIE);
    ADCSRA &= ~ _BV(ADEN);
    ADCSRB |= _BV(ACME);
    ADMUX = OOK_INT - 14;
  } else
    bitClear(PCMSK1, OOK_INT - 14);
#endif
#else

  if (attach) {
    attachInterrupt(RF12_RX_EXTINT, rf12_data_int, CHANGE);
  } else {
    detachInterrupt(RF12_RX_EXTINT);
  }
#endif
}

void rf12_start_rx_OOK()
{
  //   Serial.println("would be nice") ;
  prev_tstamp = micros();
  rf12_pulse = 0;
  rf12_signal = bitRead(RF12_RX_PIN, RF12_RX_DATA);
   
  rf12_dur_sum = 0;
  rf12_flip_count = 0;
  rf12_high_count = 0;
  rf12_low_count = 0;
  rf12_short_count = 0;
  rf12_int_dur = 0;

  rf12_control(0x8017); // 8027    868 Mhz;disabel tx register; disable RX
  //         fifo buffer; xtal cap 12pf, same as xmitter
  //rf12_control(0x82c0); // 82C0    enable receiver; enable basebandblock
  //rf12_control(0x82D8); // 82D8    enable receiver; enable basebandblock , crysal Osc + synth
  rf12_control(0x82F8); // 82F8    enable receiver & transmitter; enable basebandblock , crysal Osc + synth

  rf12_control(0xA620); // A620    433.92 MHz
  rf12_control(0xc691); // C691    c691 datarate 2395 kbps 0xc647 = 4.8kbps

//These setting are critical and determine the noise floor for OOK on the RFM
//If wrong then due to the damn capacitor documented here :
//the pulse widths can be all over the shop and OOK decode will not work.
//I've found the 97 + 91dbm to be the most successful

   rf12_control(0x9481); // 9481    VDI; FAST;200khz;GAIn Max; DRSSI -97dbm (too noisey) (best for AlOffice Jeenode)
// rf12_control(0x9482); //        VDI; FAST;200khz;GAIn Max; DRSSI -91dbm (too noisey - 30/12/2020 looks good) (best for USB Jeenode)
// rf12_control(0x9483); // 9483    VDI; FAST;200khz;GAIn Max; DRSSI -85dbm (looks good - 30/12/2020 a bit deaf)
// rf12_control(0x9484); // 9483    VDI; FAST;200khz;GAIn Max; DRSSI -79dbm (deaf)


  rf12_control(0xC220); // C220    datafiltercommand; ** not documented cmd
  rf12_control(0xCA00); // CA00    FiFo and resetmode cmd; FIFO fill disabeld
  rf12_control(0xC473); // C473    AFC run only once; enable AFC; enable
  //         frequency offset register; +3 -4
  rf12_control(0xCC67); // CC67    pll settings command
  rf12_control(0xB800); // TX register write command not used
  rf12_control(0xC800); // disable low dutycycle
  rf12_control(0xC040); // 1.66MHz,2.2V not used see 82c0
  
  rf12_interruptcontrol(1); //attach

}

void rf12_stop_rx_OOK()
{
  rf12_interruptcontrol(0); //detach
  rf12_dur_sum += micros() - prev_tstamp;
 
}




// Turn transmitter on or off, but also apply asymmetric correction and account
// for 25 us SPI overhead to end up with the proper on-the-air pulse widths.
// With thanks to JGJ Veken for his help in getting these values right.

static void ookPulse(int on, int off) {
  rf12_onOff(1);
  //  delayMicroseconds(on + 150);
  delayMicroseconds(on + 150);

  rf12_onOff(0);
  //delayMicroseconds(off - 200);

  delayMicroseconds(off - 150);

  }

static void fs20sendBits(word data, byte bits) {

  for (word mask = bit(bits - 1); mask != 0; mask >>= 1) {
    int width = data & mask ? 500 : 1500 ;

    ookPulse(width, 1000);
  }
}


static void sendPacket(uint8_t const * data, int len, uint8_t repeat = 1)
{

  while (repeat-- > 0)
  {

    uint16_t tx = 0;
    uint8_t b = 16;
    for (uint8_t i = 0; i < len; ++i)
    {
      fs20sendBits(data[i], 8) ;
    }



   delay(200);
  }
}


void sendDcfPacket( const uint8_t id,  DateTime &data )
{
  byte preamble = 0xFF ;
  uint8_t msg[10];

  msg[0] = preamble;
  msg[1] = (MsgType_DCF << 4) | (id >> 4);
  msg[2] = (id << 4) | 0x0a;
  msg[3] = dec2bcd(data.hour());// | 0x80;
  msg[4] = dec2bcd(data.minute());
  msg[5] = dec2bcd(data.second());
  msg[6] = dec2bcd(data.year() - 2000);
  msg[7] = dec2bcd(data.month());// | 0x40;
  msg[8] = dec2bcd(data.day());
  // WH1080 format is 11 bytes inc this : msg[9] = 0x45;
  msg[9] = crc8(&msg[1], 8);

  /*
    for (int i = 0; i < 15; i++) {
      msg[1] = 0x60 | 0xc ;
      Serial.print("  DCF ID:") ; Serial.println(msg[1], HEX) ;
      sendPacket(msg, ARRAY_SIZE(msg));

    }
  */
  //msg[1] = 0x6c  ;

  sendPacket(msg, ARRAY_SIZE(msg), 3);
}


void start_rx_OOK()
{
  rf12_start_rx_OOK();
}

void stop_rx_OOK()
{
  rf12_stop_rx_OOK();
}

#include <SPI.h>                        //SPI Support
#include <RH_RF95.h>                    //LoRa Radio support
#include <RHReliableDatagram.h>         //Retransmission and ACK/NAK protocol
#include <RTCZero.h>
#include <wiring.c>
#include <Arduino.h>
#include <samd.h>

//#define debug
#define SLEEP
#define TurnOffExtraStuff

#ifdef debug
  #include <SdFat.h>      //stupidly only have this here for cout. ;)  Judge me if you want its only for debugging.
  ArduinoOutStream cout(Serial);    //only here for debugging
#endif

#define SLEEPSECS 0   //don't set to 60 or higher as the %60 below breaks things
#define SLEEPMINS 1
#define MISSED_BEFORE_SLEEP_AWAKE 100
#define MISSED_BEFORE_SLEEP_DOZE 1
//#define UPDATE_RATE 1000
#define RADIO_RX_TIMEOUT 1000

#define ANEMOMETER_SPEED_PIN 5
#define ANEMOMETER_DIR_PIN 6

#define RED_LED_PIN 13

#define BATTERY_PIN A7

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0
#define RH_RF95_MAX_MESSAGE_LEN 32
#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

RH_RF95 rf95(RFM95_CS, RFM95_INT);
RTCZero rtc;

const byte seconds = 0;
const byte minutes = 0;
const byte hours = 0;

const byte day = 1;
const byte month = 1;
const byte year = 17;

void setup()
{
  #ifdef debug
    Serial.begin(115200);
	while(!Serial);  //wait for USB port to be initialized
  #endif

  delay(5000);  //long delay to allow for reprogramming if some sleep thing goes sideways.

  pinMode(RED_LED_PIN, OUTPUT );
  digitalWrite(RED_LED_PIN, LOW);

  //setup radio pins
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  //reset the radio
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  rtc.begin();
  rtc.setTime(hours, minutes, seconds); //set time to bogus value
  rtc.setDate(day, month, year);        //set date to bogus value

  rtc.attachInterrupt(isrRTC);
  rtc.enableAlarm(rtc.MATCH_SS); // Match seconds only

  while (!rf95.init()) {
    #ifdef debug
      Serial.println("LoRa radio init failed");
    #endif
    while (1);
  }
  #ifdef debug
    Serial.println("LoRa radio init OK!");
  #endif

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    #ifdef debug
      Serial.println("setFrequency failed");
    #endif
    while (1);
  }
  #ifdef debug
    Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  #endif

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
 
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(5, false);

  //Bw125Cr45Sf128 - Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range.
  //Bw500Cr45Sf128 - Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range.
  //Bw31_25Cr48Sf512 - Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range.
  //Bw125Cr48Sf4096 - Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, CRC on. Slow+long range.
  rf95.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf128);

  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_SPEED_PIN), isrSpeed, FALLING);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_DIR_PIN), isrDirection, FALLING);
  //tcConfigure(1000);  //1 second timer 

  //start the alarm for 1 sleeptime from end of setup()
  rtc.setAlarmTime(rtc.getHours(), (rtc.getMinutes()+SLEEPMINS)%60, (rtc.getSeconds()+SLEEPSECS)%60); 

  //AHBMASK:  CLK_HPBA_AHB CLK_HPBB_AHB CLK_HPBC_AHB CLK_DSU_AHB CLK_NVMCTRL_AHB CLK_DMAC_AHB CLK_USB_AHB
  //APBAMASK:  CLK_PAC0_APB CLK_PM_APB CLK_SYSCTRL_APB CLK_GCLK_APB CLK_WDT_APB CLK_RTC_APB CLK_EIC_APB
  //APBBMASK:  CLK_PAC1_APB CLK_DSU_APB CLK_NVMCTRL_APB CLK_PORT_APB CLK_DMAC_APB CLK_USB_APB
  //APBCMASK:  CLK_SERCOM0_APB CLK_SERCOM1_APB CLK_SERCOM2_APB CLK_SERCOM3_APB CLK_SERCOM4_APB CLK_SERCOM5_APB CLK_TCC0_APB CLK_TCC1_APB CLK_TCC2_APB CLK_TC3_APB CLK_TC4_APB CLK_TC5_APB CLK_ADC_APB CLK_DAC_APB

  #ifdef TurnOffExtraStuff
    REG_PM_APBAMASK &= ~PM_APBAMASK_WDT;

    REG_PM_APBBMASK &= ~PM_APBBMASK_DSU;
    REG_PM_APBBMASK &= ~PM_APBBMASK_NVMCTRL;
    REG_PM_APBBMASK &= ~PM_APBBMASK_USB;
    
    REG_PM_APBCMASK &= ~PM_APBCMASK_SERCOM0;
    REG_PM_APBCMASK &= ~PM_APBCMASK_SERCOM1;
    REG_PM_APBCMASK &= ~PM_APBCMASK_SERCOM2;
    REG_PM_APBCMASK &= ~PM_APBCMASK_SERCOM3;
    REG_PM_APBCMASK &= ~PM_APBCMASK_TCC2;
    REG_PM_APBCMASK &= ~PM_APBCMASK_TC3;
    REG_PM_APBCMASK &= ~PM_APBCMASK_TC4;
    REG_PM_APBCMASK &= ~PM_APBCMASK_TC5;
    REG_PM_APBCMASK &= ~PM_APBCMASK_TC6;
    REG_PM_APBCMASK &= ~PM_APBCMASK_TC7;
    REG_PM_APBCMASK &= ~PM_APBCMASK_DAC;
    REG_PM_APBCMASK &= ~PM_APBCMASK_PTC;
    REG_PM_APBCMASK &= ~PM_APBCMASK_I2S;
    REG_PM_APBCMASK &= ~PM_APBCMASK_AC;
    //REG_PM_APBCMASK &= ~PM_APBCMASK_ADC;
  #endif
}

bool newDataAvail;

const unsigned long DEBOUNCE = 60000ul;
const unsigned long TIMEOUT = 1600000ul;

unsigned long prevSpeedPulse;
unsigned long speedPulseWidth;
unsigned long dirPulseTime;
unsigned long dirPulseWidth;
volatile unsigned long speedPulse = 0ul;    // Time capture of speed pulse
volatile unsigned long dirPulse = 0ul;      // Time capture of direction pulse
volatile unsigned long speedTime = 0ul;     // Time between speed pulses (microseconds)
volatile unsigned long directionTime = 0ul; // Time between direction pulses (microseconds)
volatile boolean newData = false;           // New speed pulse received
volatile unsigned long lastUpdate = 0ul;    // Time of last serial output

// Knots is actually stored as (Knots * 100). Deviations below should match these units.
const int BAND_0 = 10 * 100;
const int BAND_1 = 80 * 100;

const int SPEED_DEV_LIMIT_0 = 5 * 100;  // Deviation from last measurement to be valid. Band_0: 0 to 10 knots
const int SPEED_DEV_LIMIT_1 = 10 * 100; // Deviation from last measurement to be valid. Band_1: 10 to 80 knots
const int SPEED_DEV_LIMIT_2 = 30 * 100; // Deviation from last measurement to be valid. Band_2: 80+ knots

const int DIR_DEV_LIMIT_0 = 25; // Deviation from last measurement to be valid. Band_0: 0 to 10 knots
const int DIR_DEV_LIMIT_1 = 18; // Deviation from last measurement to be valid. Band_1: 10 to 80 knots
const int DIR_DEV_LIMIT_2 = 10; // Deviation from last measurement to be valid. Band_2: 80+ knots

int16_t _windDirection;
uint16_t _windSpeed;
uint16_t battVolts;

uint8_t data[7];
// Dont put this on the stack:
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
bool awake = false;
uint8_t messagesMissed = 0;
uint8_t messagesMissedAlotment = MISSED_BEFORE_SLEEP_DOZE;
bool  firstDatum = true;

void loop ()
{
  static uint16_t prevSpeed, prevVolts;
  static int16_t prevDir;
  static bool messageSent = false;
  static uint32_t lastTime;
  
  if(!awake)
  {
    #ifdef debug
      cout << "sending \"McFly?\" to see if anyone is out there" << endl;
    #endif
    memcpy(&data, "McFly\0", 6);
    rf95.send((uint8_t *)data, 6);
    rf95.waitPacketSent();
    messageSent = true;
    messagesMissedAlotment = MISSED_BEFORE_SLEEP_DOZE;
  }
  else if(awake && newDataAvail /*&& millis() > lastTime + UPDATE_RATE*/) {
    memcpy(&data, &_windSpeed, 2);
    #ifdef debug
      cout << _windSpeed << " ";
    #endif
    memcpy(&data[2], &_windDirection, 2);
    #ifdef debug
      cout << _windDirection << endl;
    #endif
    battVolts = getBatteryVoltage();
    memcpy(&data[4],&battVolts, 2);
    newDataAvail = false;
  
    //blip(RED_LED_PIN, 1, 10);
    if(!firstDatum) {     //throw away first data point because the timers aren't trustworthy after waking
      rf95.send((uint8_t *)data, 6);
      rf95.waitPacketSent();
      prevSpeed = _windSpeed;
      prevDir = _windDirection;
      prevVolts = battVolts;
      messageSent = true;
      lastTime = millis();
    }
    firstDatum = false;
  }
  digitalWrite(RED_LED_PIN, LOW);
  
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  if(messageSent) {
    messageSent = false;
    if (rf95.waitAvailableTimeout(RADIO_RX_TIMEOUT)) {
      if (rf95.recv(buf, &len))
      {
        messagesMissed = 0;
        messagesMissedAlotment = MISSED_BEFORE_SLEEP_AWAKE;
        awake = true;
        //keep pushing out the alarm so that we only hit the isr and set awake to false if we go to sleep
        rtc.setAlarmTime(rtc.getHours(), (rtc.getMinutes()+SLEEPMINS)%60, (rtc.getSeconds()+SLEEPSECS)%60);
        //cout << "Got reply: " << buf << endl << "RSSI: " << rf95.lastRssi() << endl;
        #ifdef SLEEP
          rf95.sleep();
        #endif
      }
      else
      {
        #ifdef debug
          Serial.println("Receive failed");
        #endif
      }
    }
    else {      //no one is acknologing our transmissions so lets go to sleep and try again later
      messagesMissed++;
      #ifdef debug
        cout << "In space no one can hear you scream " << messagesMissed << endl;
      #endif
      if(messagesMissed > messagesMissedAlotment) {
        awake = false;
        firstDatum = true;
        #ifdef SLEEP
          USBDevice.detach();
          rf95.sleep();
          rtc.standbyMode();
        #endif
      }
    }
  }
}

void isrSpeed() {
  //Serial.println("isrSpeed");
  if (((micros() - speedPulse) > DEBOUNCE) && (digitalRead(ANEMOMETER_SPEED_PIN) == LOW))
  {
    // Work out time difference between last pulse and now
      speedTime = micros() - speedPulse;
        
      // Direction pulse should have occured after the last speed pulse
      if (dirPulse - speedPulse >= 0)
        directionTime = dirPulse - speedPulse;

      processNewData();
		  speedPulse = micros(); // Capture time of the new speed pulse
  }
}

void isrDirection() {
  //Serial.println("isrDirection");
  if (((micros() - dirPulse) > DEBOUNCE) && (digitalRead(ANEMOMETER_DIR_PIN) == LOW))
  {
    dirPulse = micros(); // Capture time of direction pulse
	}
}

void TC5_Handler (void) {
    if(micros() - speedPulse > TIMEOUT) {
		  _windSpeed = 0;
		  newDataAvail = true;
	  } 
    TC5->COUNT16.INTFLAG.bit.MC0 = 1; //don't change this, it's part of the timer code
  }

void isrRTC() {
  //update the alarmtime and return to loop (which will put us to sleep again).
  rtc.setAlarmTime(rtc.getHours(), (rtc.getMinutes()+SLEEPMINS)%60, (rtc.getSeconds()+SLEEPSECS)%60);
  awake = false;
  firstDatum = true;
  messagesMissed = 0;
  messagesMissedAlotment = MISSED_BEFORE_SLEEP_DOZE;
}

static void failBlink() {
  while (true) {
    digitalWrite(RED_LED_PIN, HIGH);
    delay(75);
    digitalWrite(RED_LED_PIN, LOW);
    delay(75);
  }
} //failBlink

static void blip(int ledPin, int times, int dur) {
  //Toggles a pin in the direction opposite that it currently sits a number of times with a delay of dur between them.
  //The reason for "opposite direction" is tha this allows one to "blink" a light that is on steady as well.
  //This function assumes that pin is already configured as an output.
  for (int i = 0; i < times; i++) {
    digitalWrite(ledPin, !digitalRead(ledPin));
    delay(dur);
    digitalWrite(ledPin, !digitalRead(ledPin));
    delay(dur);
  }
} //blip

static uint16_t getBatteryVoltage () {
  static boolean firstRead = true;
  
  uint32_t measBatt = analogRead(BATTERY_PIN);

  if(firstRead) {
    delay(100);
    measBatt = analogRead(BATTERY_PIN);  //I've been getting bad results on the first read so throw the first one out. 
    firstRead = false;
  }
  measBatt *= 2;    // the board has a divide by 2, so multiply back
  measBatt *= 330;  // Multiply by 3.3V, the analog reference voltage * 100 so we don't have to do float math
  measBatt /= 1024; // convert "counts" to voltage

  return uint16_t(measBatt);
}  //getBatteryVoltage

void processNewData() {
	uint32_t dirPulse_, speedPulse_;
  uint32_t speedTime_;
  uint32_t directionTime_;
  int16_t windDirection = 0l, rps = 0l, knots = 0l;

  static int16_t prevKnots = 0;
  static int16_t prevDir = 0;
  int dev = 0;

  // Get snapshot of data into local variables. Note: an interrupt could trigger here disable them for safety
  noInterrupts();
  dirPulse_ = dirPulse;
  speedPulse_ = speedPulse;
  speedTime_ = speedTime;
  directionTime_ = directionTime;
  interrupts();					//TODO: benchmark things and if you can guarantee that thigns get done with high winds this step isn't needed

  //Serial.print("spdPulse: "); Serial.println(speedPulse_);
  //Serial.print("dirPulse: "); Serial.println(dirPulse_);


  // Make speed zero, if the pulse delay is too long (drop sails its time for a beer)
  if (micros() - speedPulse_ > TIMEOUT) {
    speedTime_ = 0ul;
    knots = 0;
	}

    // The following converts revolutions per 100 seconds (rps) to knots x 100
    // This calculation follows the Peet Bros. piecemeal calibration data
  if (speedTime_ > 0)
  {
    rps = 100000000 / speedTime_; //revolutions per 100s

    if (rps < 323)
    {
        knots = (rps * rps * -11) / 11507 + (293 * rps) / 115 - 12;
    }
    else if (rps < 5436)
    {
        knots = (rps * rps / 2) / 11507 + (220 * rps) / 115 + 96;
    }
    else
    {
        knots = (rps * rps * 11) / 11507 - (957 * rps) / 115 + 28664;
    }
  
		if(knots < 0) { knots = 0; }   //make sure knots is never negative
        
		dev = (int)knots - prevKnots;

    // Only update output if in deviation limit
    if (checkSpeedDev(knots, dev))
    {
      //Serial.print("Wind Speed: "); Serial.println(knots/100.0);  //show wind speed
      _windSpeed = knots;  //write it into the global for loop();
      if (directionTime_ > speedTime_)
      {
        windDirection = 999; // For debugging only
      }
      else
      {
        // Calculate direction from captured pulse times
        windDirection = (((directionTime_ * 360) / speedTime_)) % 360;

        // Find deviation from previous value
        dev = (int)windDirection - prevDir;

        // Check deviation is in range
        if (checkDirDev(knots, dev))
        {
          _windDirection = windDirection;
          newDataAvail = true;
        }
        prevDir = windDirection;
      }
    }
    prevKnots = knots; // Update, even if outside deviation limit, cause it might be valid!?
  }
}

bool checkDirDev(long knots, int dev)
{
  if (knots < BAND_0)
  {
    if ((abs(dev) < DIR_DEV_LIMIT_0) || (abs(dev) > 360 - DIR_DEV_LIMIT_0))
      return true;
  }
  else if (knots < BAND_1)
  {
    if ((abs(dev) < DIR_DEV_LIMIT_1) || (abs(dev) > 360 - DIR_DEV_LIMIT_1))
    return true;
  }
  else
  {
    if ((abs(dev) < DIR_DEV_LIMIT_2) || (abs(dev) > 360 - DIR_DEV_LIMIT_2))
    return true;
  }
  return false;
}

bool checkSpeedDev(long knots, int dev)
{
  if (knots < BAND_0)
  {
    if (abs(dev) < SPEED_DEV_LIMIT_0)
      return true;
    }
    else if (knots < BAND_1)
    {
      if (abs(dev) < SPEED_DEV_LIMIT_1)
      return true;
    }
    else
    {
      if (abs(dev) < SPEED_DEV_LIMIT_2)
      return true;
    }
    return false;
}

//////////////////////////////////////////////////////Timer Counter Configuration///////////////////////////////////////////////////
void tcConfigure(int sampleRate)
{
  // Enable GCLK for TCC2 and TC5 (timer counter input clock)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
  while (GCLK->STATUS.bit.SYNCBUSY);

  tcReset(); //reset TC5

  // Set Timer counter Mode to 16 bits
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT32;
  // Set TC5 mode as match frequency
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  //set prescaler and enable TC5
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_ENABLE;
  //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
  TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
  delay(10);

  // Configure interrupt request
  NVIC_DisableIRQ(TC5_IRQn);
  NVIC_ClearPendingIRQ(TC5_IRQn);
  NVIC_SetPriority(TC5_IRQn, 0);
  NVIC_EnableIRQ(TC5_IRQn);

  // Enable the TC5 interrupt request
  TC5->COUNT16.INTENSET.bit.MC0 = 1;
  delay(10); //wait until TC5 is done syncing 
} 
//Reset TC5 
void tcReset()
{
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  delay(10);
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}
//disable TC5
void tcDisable()
{
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  delay(10);
}

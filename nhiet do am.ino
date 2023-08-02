/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

/*******************************************************************************/
 // Region definition (will change de frequency bands
 // Define only 1 country
 //
#define CFG_EU 1
//#define CFG_VN 1

/*******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "LowPower.h"
//Sensors librairies
#include <Wire.h>
#include <LTR303.h>

#include "SHTC3.h"




#include <SoftwareSerial.h>

#include <TinyGPS.h>

#include "PMS.h"

PMS pms(Serial);
PMS::DATA data;


TinyGPS gps;
SoftwareSerial ss(11 , 12);

static void smartdelay(unsigned long ms);

// Create an LTR303 object, here called "light":

LTR303 lightsensor;
//KXTJ3 myIMU(0x0E); // Address can be 0x0E or 0x0F
SHTC3 s(Wire);
uint8_t pm1,pm25,pm10;
uint32_t lat,lon;
float   sampleRate = 6.25;  // HZ - Samples per second - 0.781, 1.563, 3.125, 6.25, 12.5, 25, 50, 100, 200, 400, 800, 1600Hz
uint8_t accelRange = 2;     // Accelerometer range = 2, 4, 8, 16g
 uint16_t t ;
 uint16_t h ;
// Global variables:

// LoRaWAN end-device address (DevAddr)

static const u4_t DEVADDR = 0xd7f8e98d;

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0xaa,0xba,0xaa,0x58,0x6a,0xb9,0x0b,0x19,0x93,0xf7,0xc4,0x80,0xbc,0x7c,0x72,0xe6 };


// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = {  0x5e,0xd8,0x0c,0x2b,0xc0,0xa7,0x53,0x83,0xe7,0x42,0x18,0x30,0xcb,0x86,0x68,0x18  };


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;


// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 8,
    .dio = {6, 6, 6},
};

// ---------------------------------------------------------------------------------
// Functions
// ---------------------------------------------------------------------------------

extern volatile unsigned long timer0_overflow_count;
extern volatile unsigned long timer0_millis;
void addMillis(unsigned long extra_millis) {
  uint8_t oldSREG = SREG;
  cli();
  timer0_millis += extra_millis;
  SREG = oldSREG;
  sei();
}

void do_sleep(unsigned int sleepyTime) {
  unsigned int eights = sleepyTime / 8;
  unsigned int fours = (sleepyTime % 8) / 4;
  unsigned int twos = ((sleepyTime % 8) % 4) / 2;
  unsigned int ones = ((sleepyTime % 8) % 4) % 2;

        Serial.print("Sleep during ");
        Serial.print(sleepyTime);
        Serial.println("sec");
        delay(50);

        Serial.end();
        
          for ( int x = 0; x < eights; x++) {
            // put the processor to sleep for 8 seconds
            LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
            // LMIC uses micros() to keep track of the duty cycle, so
              // hack timer0_overflow for a rude adjustment:
              cli();
              timer0_overflow_count+= 8 * 64 * clockCyclesPerMicrosecond();
              sei();
          }
          for ( int x = 0; x < fours; x++) {
            // put the processor to sleep for 4 seconds
            LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
            // LMIC uses micros() to keep track of the duty cycle, so
              // hack timer0_overflow for a rude adjustment:
              cli();
              timer0_overflow_count+= 4 * 64 * clockCyclesPerMicrosecond();
              sei();
          }
          for ( int x = 0; x < twos; x++) {
            // put the processor to sleep for 2 seconds
            LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
            // LMIC uses micros() to keep track of the duty cycle, so
              // hack timer0_overflow for a rude adjustment:
              cli();
              timer0_overflow_count+= 2 * 64 * clockCyclesPerMicrosecond();
              sei();
          }
          for ( int x = 0; x < ones; x++) {
            // put the processor to sleep for 1 seconds
            LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
            // LMIC uses micros() to keep track of the duty cycle, so
              // hack timer0_overflow for a rude adjustment:
              cli();
              timer0_overflow_count+=  64 * clockCyclesPerMicrosecond();
              sei();
          }
          addMillis(sleepyTime * 1000);

          Serial.begin(9600);
}
uint32_t flat,flon;
// ReadVcc function to read MCU Voltage
long readVcc() {
      long result;
      // Read 1.1V reference against AVcc
      ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
      delay(2); // Wait for Vref to settle
      ADCSRA |= _BV(ADSC); // Convert
      while (bit_is_set(ADCSRA,ADSC));
      result = ADCL;
      result |= ADCH<<8;
      result = 1126400L / result; // Back-calculate AVcc in mV
      return result;
}

// Read Light function for 
double readLight() { 
        double result;
        unsigned int data0, data1;
        lightsensor.getData(data0,data1);
    
    // Perform lux calculation:

    lightsensor.getLux(0,1,data0,data1,result);  
            return result;
}



void onEvent (ev_t ev) {
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
              for (int i = 0; i < LMIC.dataLen; i++) {
              if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
              Serial.print(F("0"));
              }
              Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
              }
              Serial.println("");
            }
            // Schedule next transmission
            
            os_setTimedCallback(&sendjob,os_getTime()+sec2osticks(1), do_send);
                     
            do_sleep(TX_INTERVAL);
                        
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        //Serial.println(F("OP_TXRXPEND"));
    } 
    
    else {      
            
           
            // if( myIMU.readRegisterInt16( &dataHighres, KXTJ3_OUT_X_L ) == 0 ){}
            // int16_t x = dataHighres/16.384;
            // if( myIMU.readRegisterInt16( &dataHighres, KXTJ3_OUT_Y_L ) == 0 ){}
            // int16_t y = dataHighres/16.384;
            // if( myIMU.readRegisterInt16( &dataHighres, KXTJ3_OUT_Z_L ) == 0 ){}
            // int16_t z = dataHighres/16.384;

            
int couter=0;
  while (couter < 150) {
  
  
  
  couter++;
  if (couter <  98)
  {
    delay(10);
  if (pms.read(data))
  {
    
    pm1=data.PM_AE_UG_1_0;
    
    pm25=data.PM_AE_UG_2_5;
    
    pm10=data.PM_AE_UG_10_0;
    
  }
  
  }
  else { if ((couter >=98) & (couter <150)) {


  
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
  
  
  gps.f_get_position(&flat, &flon, &age);
 
  lat=flat*10000;
  lon=flon*10000; 
 
  
  //smartdelay(1000);
  }
      Serial.print("PM 1.0 (ug/m3): ");
    Serial.println(pm1);
    Serial.print("PM 2.5 (ug/m3): ");
    Serial.println(pm25);
    Serial.print("PM 10.0 (ug/m3): ");
    Serial.println(pm10);
     Serial.println(flat);

  Serial.println();
  Serial.println(flon);
  Serial.println();
    s.sample();
    Serial.print(F("[SHTC3] T:"));
    Serial.print(s.readTempC());
    Serial.print(F(" Cº  /   H: "));
    Serial.print(s.readHumidity());
    Serial.println(F(" %"));
    delay(2000);
    t = s.readTempC()*1000;
    h = s.readHumidity()*1000;
   }
      
  }
           



            unsigned char mydata[15];

         mydata[0] = (t >> 0) & 0xFF ;//nhiet do
         mydata[1] = (t >> 8) & 0xFF;//nhiet do
          
         mydata[2] = (h >> 0) & 0xFF ;//do am
         mydata[3] = (h >> 8) & 0xFF;//do am
          
         

        mydata[4] = pm1 & 0xFF;//pm1

        mydata[5] = pm25 & 0xFF;//pm25

        mydata[6] = pm10 & 0xFF;//pm10
                

        mydata[7] = (lat >> 0) & 0xff;
        mydata[8] = (lat >> 8) & 0xff;
        mydata[9] = (lat >> 16) & 0xff;
        mydata[10] = (lat >> 24) & 0xff;

        mydata[11] = (lon >> 0) & 0xff;
        mydata[12] = (lon >> 8) & 0xff;
        mydata[13] = (lon >> 16) & 0xff;
        mydata[14] = (lon >> 24) & 0xff;
            
      //      mydata[13] = l >> 8; // Luminosity
       //     mydata[14] = l & 0xFF; // Luminosity
        
             Serial.print(mydata[0]);
              Serial.print("  ");
               Serial.print(mydata[1]);
              Serial.print("  ");
               Serial.print(mydata[2]);
              Serial.print("  ");
               Serial.print(mydata[3]);
              Serial.print("  ");
               Serial.print(mydata[4]);
              Serial.print("  ");
                Serial.print(mydata[5]);
              Serial.print("  ");
               Serial.print(mydata[6]);
              Serial.print("  ");
               Serial.print(mydata[7]);
              Serial.print("  ");
               Serial.print(mydata[8]);
              Serial.print("  ");
               Serial.print(mydata[9]);
              Serial.print("  ");
                Serial.print(mydata[10]);
              Serial.print("  ");
               Serial.print(mydata[11]);
              Serial.print("  ");
               Serial.print(mydata[12]);
              Serial.print("  ");
               Serial.print(mydata[13]);
               Serial.print("  ");
               Serial.print(mydata[14]);
               
           

              
Serial.println();
     
            LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
          
            
      
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {

    Serial.begin(9600);
    Serial.println("Starting");
    
    ss.begin(9600);
    Wire.begin();

    s.begin(true);
    
    // Set-up sensors
    lightsensor.begin();    
    lightsensor.setPowerUp();

 
  
  // Detection threshold, movement duration and polarity

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();


    /* This function is intended to compensate for clock inaccuracy (up to ±10% in this example), 
    but that also works to compensate for inaccuracies due to software delays. 
    The downside of this compensation is a longer receive window, which means a higher battery drain. 
    So if this helps, you might want to try to lower the percentage (i.e. lower the 10 in the above call), 
    often 1% works well already. */
    
    LMIC_setClockError(MAX_CLOCK_ERROR * 2 / 100);

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_EU)
    LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);   // g-band
    LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);  // g-band
    LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);   // g-band
    LMIC_setupChannel(3, 927700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);   // g-band
    LMIC_setupChannel(4, 923100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);   // g-band
    LMIC_setupChannel(5, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);   // g-band
    LMIC_setupChannel(6, 922700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    
    #elif defined(CFG_VN)
    // Set up the 8 channels used    
    LMIC_setupChannel(0, 921400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 921600000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 921800000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 922000000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 922200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 922400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 922600000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 922800000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 922700000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band     
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
  
  
  
 
    os_runloop_once();
}







static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
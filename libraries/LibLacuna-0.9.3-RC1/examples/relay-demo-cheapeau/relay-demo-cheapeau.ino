/*
 *  _                                   ____                       
 * | |    __ _  ___ _   _ _ __   __ _  / ___| _ __   __ _  ___ ___ 
 * | |   / _` |/ __| | | | '_ \ / _` | \___ \| '_ \ / _` |/ __/ _ \
 * | |__| (_| | (__| |_| | | | | (_| |  ___) | |_) | (_| | (_|  __/
 * |_____\__,_|\___|\__,_|_| |_|\__,_| |____/| .__/ \__,_|\___\___|
 *                                           |_|                   
 * Copyright (C) 2020 Lacuna Space Ltd.
 *
 * Description: Lacuna TTN Relay (demo)
 * 
 * Licensae: Revised BSD License, see LICENSE-LACUNA.TXT file included in the project
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/*
 * You can install these libraries via the Arduino library manager:
 *  
 *  http://librarymanager/All#Sodaq_LSM303AGR
 *  http://librarymanager/All#SparkFun_BME280
 *  http://librarymanager/All#SparkFun_Ublox
 *  http://librarymanager/All#SparkFun_ATECCX08a
 *  
 */

#ifndef REGION
#define REGION R_EU868
#endif

// Use GPS
#define GPS

// max. 250 seconds for GPS fix
#define FIXTIME 200

/* LOWPOWER enables sleep mode */
//#define LOWPOWER

/* Relay interval in minutes */
#define RELAY_INTERVAL  30

#include <LibLacuna.h>
#include <time.h>
#include <RTC.h>
#include <SPI.h>
#include <LTR303.h> // https://github.com/automote/LTR303
#include <SparkFun_Ublox_Arduino_Library.h>   // http://librarymanager/All#SparkFun_Ublox
#include <KX0231025IMU.h>     // http://librarymanager/All#KX023
#include <Wire.h>

// This device (transmit)
static byte networkKey[] = { 
    // TODO: Replace with your network key
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
static byte appKey[] = {
    // TODO: Replace with your application key
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
// Replace with your device address
static byte deviceAddress[] = { 0x01, 0x02, 0x03, 0x04 };

// Device to be relayed (receive)
static byte relay_networkKey[] = { 
    // TODO: Replace with relay network key
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
static byte relay_appKey[] = { };
static byte relay_deviceAddress[] = { 0x0A, 0x0B, 0x0C, 0x0D };


static lsLoraWANParams loraWANParams;
static lsLoraWANParams relay_loraWANParams;

static char payload[255];
static int payloadLength;

static byte relay_payload[255];
uint8_t relay_length;

uint32_t FixEpoch;
uint8_t  TimeToFix;
uint32_t last_epoch;
uint32_t alarmepoch;

uint32_t relay_epoch = 0;
uint16_t relay_false_detects = 0;
uint16_t relay_uplinks = 0;
uint16_t relay_framecounter = 0;
int8_t relay_rssi = 0;
int8_t relay_snr = 0;

volatile uint8_t alarm = 0;
extern volatile int preamble_detect;

// LPP
uint8_t  store[4096];
uint8_t  lpp_age = 0;
uint32_t oldest_record;
uint32_t newest_record;

// Env variable
int16_t temp = 0;
uint16_t pressure = 0;
float gnss_lat;
float gnss_lon;
float gnss_alt;
int nb_sat;
 // Define GPS
SFE_UBLOX_GPS myGPS;
// Define Accelerometer
KX0231025Class *imu;
// Define luminosity
LTR303 light;

lsLoraTxParams txParams;
lsLoraTxParams relayParams;

void setup()
{
  pinMode(LS_LED_BLUE, OUTPUT);
  digitalWrite(LS_LED_BLUE, HIGH);
  delay(200);
  digitalWrite(LS_LED_BLUE, LOW);
  
  Serial.begin(9600);
  while (!Serial && millis() < 3000);

  Serial.println("Initializing");

  Serial.print("LibLacuna version: ");
  Serial.println(LIBLACUNA_VERSION);

  analogReadResolution(12);

  Wire.begin();
  

  pinMode(LS_GPS_ENABLE, OUTPUT);
  digitalWrite(LS_GPS_ENABLE, HIGH);
  pinMode(LS_GPS_V_BCKP, OUTPUT);
  digitalWrite(LS_GPS_V_BCKP, HIGH);

  digitalWrite(LS_VERSION_ENABLE, LOW);  

  delay(70);

  // GPS
  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
      Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring."));
      //while (1);
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration();        //Save the current settings to flash and BBR


  #ifdef GPS
   bool fix = updategps();
  #endif

  digitalWrite(LS_GPS_ENABLE, LOW);
  digitalWrite(LS_VERSION_ENABLE, LOW);
  digitalWrite(LS_GPS_V_BCKP, LOW);  
 
  lsCreateDefaultLoraWANParams(&loraWANParams, networkKey, appKey, deviceAddress);
  loraWANParams.txPort = 2;
  loraWANParams.rxEnable = false;

  lsCreateDefaultLoraWANParams(&relay_loraWANParams, relay_networkKey, relay_appKey, relay_deviceAddress);
 
  lsSX126xConfig cfg;
  lsCreateDefaultSX126xConfig(&cfg, BOARD_VERSION);
  
  int result = lsInitSX126x(&cfg, REGION);
  if(result) Serial.println(lsErrorToString(result));

  /* LoRa parameters for device (forward) */
  lsCreateDefaultLoraTxParams(&txParams, REGION);

  txParams.power = 14;
  txParams.spreadingFactor = lsLoraSpreadingFactor_7;
  txParams.codingRate = lsLoraCodingRate_4_5;
  txParams.invertIq = false;
  txParams.frequency = 868100000;
  txParams.bandwidth = lsLoraBandwidth_125_khz;
  txParams.syncWord = LS_LORA_SYNCWORD_PUBLIC;
  txParams.preambleLength = 8;

  /* LoRa parameters for relay (receive) */
  lsCreateDefaultLoraTxParams(&relayParams, REGION);

  relayParams.spreadingFactor = lsLoraSpreadingFactor_7;
  relayParams.invertIq = false;
  relayParams.frequency = 866600000;
  relayParams.bandwidth = lsLoraBandwidth_125_khz;
  relayParams.syncWord = LS_LORA_SYNCWORD_PUBLIC;

  setAlarm(RTC.getEpoch() + RELAY_INTERVAL*60);

  ///////////// Sensors  /////////////////////

  // HP203B I2C address is 0x76(118)
#define HP203B_Addr 0x76

 // Define accelerometer
  imu = new KX0231025Class(Wire, 0x1E);  
  imu->begin(0X00, 0, 0); // low power, 2G, 12.5Hz

// Create an LTR303 object, here called "light":
LTR303 light;

// Global variables LTR303:
unsigned char gain;     // Gain setting, values = 0-7 
unsigned char integrationTime;  // Integration ("shutter") time in milliseconds
unsigned char measurementRate;  // Interval between DATA_REGISTERS update
}

void loop() 
{
  /* set sleep_func to 'LS200_Sleep' for low-power or 'NULL' for debug/serial */
#ifdef LOWPOWER
  int rxlength = lsRelayLora(&relay_loraWANParams, &relayParams, relay_payload, RF200_Sleep);
#else
  int rxlength = lsRelayLora(&relay_loraWANParams, &relayParams, relay_payload, NULL);
#endif


  if (rxlength > 0) { 
    /* valid relay data received */
    relay_length = rxlength;
    relay_uplinks++;
    relay_rssi = relayParams.rssi;
    relay_snr = relayParams.snr;
    relay_framecounter = (relay_payload[7] << 8) + relay_payload[6];
    relay_epoch = RTC.getEpoch();

    Serial.println("Valid uplink received.");
    Serial.print("  SNR: ");
    Serial.println(relayParams.snr);
    Serial.print("  RSSI: ");
    Serial.println(relayParams.rssi);
    Serial.print("  SignalRSSI: ");
    Serial.println(relayParams.signalrssi);
    Serial.print("  ");
    for (char n = 0; n < rxlength; n++)
        {
          Serial.print (relay_payload[n],HEX);
          Serial.write (" ");    
        }
    Serial.println();
	// relay received data
      int lora_result  = lsSendLora(&txParams, (byte *)relay_payload, relay_length, false);
      Serial.print("LoRa TX Relay Result: ");
      Serial.println(lsErrorToString(lora_result)); 
      relay_length = 0;
      
      digitalWrite(LS_LED_BLUE, HIGH);
      delay(50);
      digitalWrite(LS_LED_BLUE, LOW);

    
  }

  if (alarm == 1) {
    /* wakeup to relay */
    Serial.println("Alarm");
    alarm = 0;
    digitalWrite(LS_LED_BLUE, HIGH);
    delay(100);
    digitalWrite(LS_LED_BLUE, LOW);
    Serial.print("Epoch: ");
    Serial.println(RTC.getEpoch());
    generateRelayStatus(false);
    int lora_result  = lsSendLoraWAN(&loraWANParams, &txParams, (byte *)payload, payloadLength);
    Serial.print("LoRa TX Status Result: ");
    Serial.println(lsErrorToString(lora_result)); 
    if (relay_length) {
      int lora_result  = lsSendLora(&txParams, (byte *)relay_payload, relay_length, false);
      Serial.print("LoRa TX Relay Result: ");
      Serial.println(lsErrorToString(lora_result)); 
      relay_length = 0;
    } else {
      Serial.println("Nothing to relay");
    }
    setAlarm(RTC.getEpoch() + RELAY_INTERVAL*60);  
  } else if (!rxlength) {
      relay_false_detects++;
      Serial.println("False detect");
  }
}

void alarmMatch() { 
  alarm = 1; 
  preamble_detect = 1;
}

void setAlarm(uint32_t alarmepoch)
{  
    time_t t;
    struct tm tm;

    t = (time_t)alarmepoch;
    gmtime_r(&t, &tm);
    
    RTC.setAlarmTime(tm.tm_hour, tm.tm_min, tm.tm_sec);
    RTC.setAlarmDay(tm.tm_mday);
    
    RTC.enableAlarm(RTC.MATCH_HHMMSS);
    RTC.attachInterrupt(alarmMatch);
}

void RF200_Sleep()  
{
  digitalWrite(LS_GPS_ENABLE, LOW);
  digitalWrite(LS_VERSION_ENABLE, LOW);
  digitalWrite(LS_GPS_V_BCKP, LOW);
  
    
  SPI.end();
  STM32.stop();
  SPI.begin();
}

void generateRelayStatus(bool satellite) 
{
  HP203B(); // Measure temp and pressure
  uint8_t lum =(int) LTR303_(); 
 
  uint16_t voltage_adc = (uint16_t)analogRead(LS_BATVOLT_PIN);
  uint16_t voltage = (uint16_t)((LS_ADC_AREF / 4.096) * (LS_BATVOLT_R1 + LS_BATVOLT_R2) / LS_BATVOLT_R2 * (float)voltage_adc);

  float x, y, z;
  imu->readAcceleration(x, y, z);

  int8_t acc_x = (int8_t) 100*x;
  int8_t acc_y = (int8_t)100*y;
  int8_t acc_z = (int8_t)100*z;

  Serial.println("Read sensors");
  Serial.print("  Voltage: ");
  Serial.println(voltage/1000.0f);
  Serial.print("  Temp: ");
  Serial.println(temp/100.0f);
  Serial.print("  Lum: ");
  Serial.println(lum);
  Serial.print("  Pressure: ");
  Serial.println(pressure);

  Serial.print("  x = ");
  Serial.print(acc_x);
  Serial.print(", y = ");
  Serial.print(acc_y);
  Serial.print(", z = ");
  Serial.println(acc_z);

  uint32_t LatitudeBinary = ((gnss_lat + 90) / 180) * 16777215;
  uint32_t LongitudeBinary = ((gnss_lon + 180) / 360) * 16777215;
  int16_t  altitudeGps = gnss_alt; 

  uint32_t relay_delay = RTC.getEpoch() - relay_epoch;

  payload[0] = satellite;
  payload[1] = (temp >> 8) & 0xff;
  payload[2] = temp & 0xff;
  payload[3] = (voltage_adc >> 8) & 0xff;
  payload[4] = voltage_adc & 0xff;
  payload[5] = (lum >> 8) & 0xff;
  payload[6] = lum & 0xff; 
  payload[7] = (pressure >> 8) & 0xff;
  payload[8] = pressure & 0xff;
  payload[9] = acc_x;
  payload[10] = acc_y;
  payload[11] = acc_z;
  payload[12] = ( LatitudeBinary >> 16 ) & 0xFF;
  payload[13] = ( LatitudeBinary >> 8 ) & 0xFF;
  payload[14] = LatitudeBinary & 0xFF;
  payload[15] = ( LongitudeBinary >> 16 ) & 0xFF;
  payload[16] = ( LongitudeBinary >> 8 ) & 0xFF;
  payload[17] = LongitudeBinary & 0xFF;
  payload[18] = nb_sat;
  payload[19] = TimeToFix;
  payload[20] = (relay_false_detects >> 8) & 0xff;
  payload[21] = relay_false_detects & 0xff;
  payload[22] = (relay_uplinks >> 8) & 0xff;
  payload[23] = relay_uplinks & 0xff;
  payload[24] = relay_rssi;
  payload[25] = relay_snr;
  payload[26] = (relay_framecounter >> 8) & 0xff;
  payload[27] = relay_framecounter & 0xff;
  payload[28] = (relay_delay >> 24) & 0xff;
  payload[29] = (relay_delay >> 16) & 0xff;
  payload[30] = (relay_delay >> 8) & 0xff;
  payload[31] = relay_delay & 0xff;
        
  payloadLength = 32;
}

byte updategps() {

   digitalWrite(LS_GPS_ENABLE, HIGH);
   delay(200);

   long startTime = millis(); 
    
   while (millis() - startTime < (FIXTIME*1000))
   {

    delay(1000);
      digitalWrite(LS_LED_BLUE, HIGH);
      delay(50);
      digitalWrite(LS_LED_BLUE, LOW);
      delay(100);
      digitalWrite(LS_LED_BLUE, HIGH);
      delay(50);
      digitalWrite(LS_LED_BLUE, LOW);
      

      long latitude = myGPS.getLatitude();
      long longitude = myGPS.getLongitude();    
      long altitude = myGPS.getAltitudeMSL();      
      byte SIV = myGPS.getSIV();
 
      Serial.println("Checking GPS");

      Serial.print("  GPS position: ");
      Serial.print(latitude/1.0e7,4);
      Serial.print(", ");
      Serial.print(longitude/1.0e7,4);
      Serial.print(" alt: ");
      Serial.print(altitude/1.0e6,2);
      Serial.print(" (");
      Serial.print(SIV);
      Serial.println(" satellites)");
 
      byte fixType = myGPS.getFixType();
      Serial.print(F("  GPS Fix: "));
      if(fixType == 0) Serial.print(F("No fix"));
      else if(fixType == 1) Serial.print(F("Dead reckoning"));
      else if(fixType == 2) Serial.print(F("2D"));
      else if(fixType == 3) Serial.print(F("3D"));
      else if(fixType == 4) Serial.print(F("GNSS+Dead reckoning"));
      else if(fixType == 5) Serial.print(F("Time only"));
      
      uint16_t new_pdop = (uint16_t)myGPS.getPDOP();
      Serial.print(", PDOP: ");
      Serial.println(new_pdop);
      
      Serial.print("  GPS time: ");
      Serial.print(myGPS.getYear());
      Serial.print("-");
      Serial.print(myGPS.getMonth());
      Serial.print("-");
      Serial.print(myGPS.getDay());
      Serial.print(" ");
      Serial.print(myGPS.getHour());
      Serial.print(":");
      Serial.print(myGPS.getMinute());
      Serial.print(":");
      Serial.println(myGPS.getSecond());
      
      uint32_t unixt = unixTimestamp(myGPS.getYear(),myGPS.getMonth(),myGPS.getDay(),myGPS.getHour(),myGPS.getMinute(),myGPS.getSecond());
      
      Serial.print("  Unix time GPS: ");
      Serial.println(unixt);
      
      if (fixType == 2 || fixType == 3 ) {
      // wait for 3D fix
      //if ( fixType == 3 ) {
        delay(1000);
        Serial.println("Updating time");
        unixt = unixTimestamp(myGPS.getYear(),myGPS.getMonth(),myGPS.getDay(),myGPS.getHour(),myGPS.getMinute(),myGPS.getSecond());

        
        Serial.print("  Unix time GPS: ");
        Serial.println(unixt);
        Serial.println("Updating position");
        gnss_lat = latitude/1e7;
        gnss_lon = longitude/1e7;
        if (fixType == 3) {
          gnss_alt = altitude/1e3;
        } else {
          gnss_alt = 0;
        }
        FixEpoch=unixt;
        nb_sat=SIV;

        int lpp_result = lpp_get_records_age(store, &oldest_record, &newest_record);
        if(lpp_result) {
            Serial.print("Error: ");
            Serial.println(lpp_error_to_string(lpp_result));
        } else {
           Serial.print("Newest LPP record: ");
           Serial.println(newest_record);
           Serial.print("Oldest LPP record: ");
           Serial.println(oldest_record);
        } 

        lpp_age = ((unixt-oldest_record)/86400);

        Serial.print("  LPP Epoch age (days): ");
        Serial.println( lpp_age );
      
        // if we have a fix, time is valid
        int32_t diff = unixt - RTC.getEpoch();
        Serial.print("  Correcting RTC by ");
        Serial.print(diff);
        Serial.println(" seconds.");
        // compensate our wakeup time
        last_epoch = last_epoch + diff;
        RTC.setEpoch(unixt);
        TimeToFix = (millis() - startTime)/1000;
        
        return(1);
    }
  }
  return(0);  
}

// Read HP203B
void HP203B(){

// HP203B 
unsigned int data[6];

  // Start I2C transmission
  Wire.beginTransmission(HP203B_Addr);
  // Send OSR and channel setting command
  Wire.write(0x40 |0x04 | 0x00);
  // Stop I2C transmission
  Wire.endTransmission();
  delay(500);

  // Start I2C transmission
  Wire.beginTransmission(HP203B_Addr);
  // Select data register
  Wire.write(0x10);
  // Stop I2C transmission
  Wire.endTransmission();

  // Request 6 bytes of data
  Wire.requestFrom(HP203B_Addr, 6);

  // Read 6 bytes of data
  // cTemp msb, cTemp csb, cTemp lsb, pressure msb, pressure csb, pressure lsb
  if (Wire.available() == 6)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();
    data[4] = Wire.read();
    data[5] = Wire.read();
  }

  // Convert the data to 20-bits
  float cTemp = (((data[0] & 0x0F) * 65536) + (data[1] * 256) + data[2]) / 100.00;
  float pressure_hp = (((data[3] & 0x0F) * 65536) + (data[4] * 256) + data[5]) / 100.00;

  // Start I2C transmission
  Wire.beginTransmission(HP203B_Addr);
  // Send OSR and channel setting command
  Wire.write(0x40 | 0x04 | 0x01);
  // Stop I2C transmission
  Wire.endTransmission();
  delay(500);

  // Start I2C transmission
  Wire.beginTransmission(HP203B_Addr);
  // Select data register
  Wire.write(0x31);
  // Stop I2C transmission
  Wire.endTransmission();

  // Request 3 bytes of data
  Wire.requestFrom(HP203B_Addr, 3);

  // Read 3 bytes of data
  // altitude msb, altitude csb, altitude lsb
  if (Wire.available() == 3)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
  }

  // Convert the data to 20-bits
  float altitude = (((data[0] & 0x0F) * 65536) + (data[1] * 256) + data[2]) / 100.00;

 
  temp = (cTemp)*100; // Convert signed to unsigned
  pressure = pressure_hp; 
}


unsigned long unixTimestamp(int year, int month, int day, int hour, int min, int sec) {
  const short days_since_beginning_of_year[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
  int leap_years = ((year-1)-1968)/4
                  - ((year-1)-1900)/100
                  + ((year-1)-1600)/400;
  long days_since_1970 = (year-1970)*365 + leap_years
                      + days_since_beginning_of_year[month-1] + day-1;
  if ( (month>2) && (year%4==0 && (year%100!=0 || year%400==0)) )
    days_since_1970 += 1; /* +leap day, if year is a leap year */
  return sec + 60 * ( min + 60 * (hour + 24*days_since_1970) );
}


double LTR303_(){
  int ms = 1000;
  delay(ms);
  unsigned int data0, data1;

  if (light.getData(data0,data1)) {
  
     double lux;    // Resulting lux value
    boolean good;  // True if neither sensor is saturated
    
    // Perform lux calculation:

    good = light.getLux(0,1,data0,data1,lux);
    
    // Print out the results:
    Serial.println("LTR303: ");
    Serial.print(" lux: ");
    Serial.println(lux);
    if (good) Serial.println(" (good)"); else Serial.println(" (BAD)");
    return lux;
  }
  
  return 0;
}

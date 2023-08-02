/*
 *  _                                   ____                       
 * | |    __ _  ___ _   _ _ __   __ _  / ___| _ __   __ _  ___ ___ 
 * | |   / _` |/ __| | | | '_ \ / _` | \___ \| '_ \ / _` |/ __/ _ \
 * | |__| (_| | (__| |_| | | | | (_| |  ___) | |_) | (_| | (_|  __/
 * |_____\__,_|\___|\__,_|_| |_|\__,_| |____/| .__/ \__,_|\___\___|
 *                                           |_|                   
 * Copyright (C) 2019 Lacuna Space Ltd.
 *
 * Description: Simple LoRaWAN example sketch
 * 
 * License: Revised BSD License, see LICENSE-LACUNA.TXT file included in the project
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef REGION
#define REGION R_EU868
#endif

// Interval between transmissions
#define TX_INTERVAL 600

// Use GPS
#define GPS

// max. 250 seconds for GPS fix
#define FIXTIME 200

#include <LibLacuna.h>
#include <time.h>
#include <RTC.h>
#include <SPI.h>
#include <LTR303.h> // https://github.com/automote/LTR303
#include <SparkFun_Ublox_Arduino_Library.h>   // http://librarymanager/All#SparkFun_Ublox
#include <KX0231025IMU.h>     // http://librarymanager/All#KX023
#include <Wire.h>

// Keys and device address are MSB
static byte networkKey[] = { 
    // Replace with your network key
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
static byte appKey[] = {
    // Replace with your application key
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
// Replace with your device address
static byte deviceAddress[] =  { 0x01, 0x02, 0x03, 0x04 };

static char payload[255];
static int payloadLength;

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


uint32_t FixEpoch;
uint8_t  TimeToFix;

uint32_t last_epoch;
uint32_t alarmepoch;

uint32_t rxtxepoch = 0;
uint8_t  txinterval = 0;
uint16_t rx_duration;
bool     rxtx;
uint8_t  satellite_id;

int32_t sleepseconds;
uint8_t wakeupreason = 0;



// LPP
uint8_t  store[4096];
uint8_t  lpp_age = 0;
uint32_t oldest_record;
uint32_t newest_record;

static lsLoraWANParams loraWANParams;
static lsLoraTxParams txParams;

void setup() {
  Serial.begin(9600);

  analogReadResolution(12);
  Wire.begin();

  pinMode(LS_LED_BLUE, OUTPUT);

  pinMode(LS_GPS_ENABLE, OUTPUT);
  digitalWrite(LS_GPS_ENABLE, HIGH);
  pinMode(LS_GPS_V_BCKP, OUTPUT);
  digitalWrite(LS_GPS_V_BCKP, HIGH);

  while (!Serial && millis() < 3000);
  
  Serial.println("Initializing");

  Serial.print("Configured Region: ");
  #if REGION == R_EU868
    Serial.println("Europe 862-870 Mhz");
  #elif REGION  == R_US915
    Serial.println("US 902-928 Mhz");
  #elif REGION == R_AS923
    Serial.println("Asia 923 Mhz");
  #elif REGION == R_IN865
    Serial.println("India 865-867 Mhz");
  #else 
    Serial.println("Undefined");  
  #endif

   delay(70);

   if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
      Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring."));
      //while (1);
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration();        //Save the current settings to flash and BBR

   
  // SX1262 configuration for lacuna LS200 board
  lsSX126xConfig cfg;
  lsCreateDefaultSX126xConfig(&cfg, BOARD_VERSION);

  // Initialize SX1262
  int result = lsInitSX126x(&cfg, REGION);
  Serial.print("E22/SX1262: ");
  Serial.println(lsErrorToString(result));

  // LoRaWAN session parameters
  lsCreateDefaultLoraWANParams(&loraWANParams, networkKey, appKey, deviceAddress);
  loraWANParams.txPort = 1;
  loraWANParams.rxEnable = true;
 
  
  // transmission parameters for terrestrial LoRa
  lsCreateDefaultLoraTxParams(&txParams, REGION);
  txParams.spreadingFactor = lsLoraSpreadingFactor_7;
  txParams.preambleRelay = true;  // activate communication to relay
  txParams.frequency = 866600000;

  Serial.print("Terrestrial Uplink Frequency: ");
  Serial.println(txParams.frequency/1e6);


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

void loop() {

  digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);
  delay(100);
  digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);

#ifdef GPS
  bool fix = updategps();
#endif

  Serial.println("Sending LoRa message"); 
  generateMessage(false); 
  int lora_result  = lsSendLoraWAN(&loraWANParams, &txParams, (byte *)payload, payloadLength);

  if (loraWANParams.rxpayloadLength) {
    Serial.println("Downlink received");
    Serial.print("Length: ");
    Serial.println(loraWANParams.rxpayloadLength);
    Serial.print("Port: ");
    Serial.println(loraWANParams.rxPort);
    Serial.print("Frame Counter: ");
    Serial.println(loraWANParams.framecounter_down);
    Serial.print("SNR: ");
    Serial.println(txParams.snr);
    Serial.print("RSSI: ");
    Serial.println(txParams.rssi);
    Serial.print("SignalRSSI: ");
    Serial.println(txParams.signalrssi);
    Serial.print("Payload: ");
    
    for (char n = 0; n < loraWANParams.rxpayloadLength; n++)
        {
          Serial.print (payload[n],HEX);
          Serial.write (" ");
          
        }
    Serial.println();
  }
  Serial.print("Result: ");
  Serial.println(lsErrorToString(lora_result));

#ifdef DEBUGSLEEP
  delay(TX_INTERVAL*1000);
#else
  setAlarm(RTC.getEpoch() + TX_INTERVAL);
  RF200_sleep();
#endif

 
}

void alarmMatch() { }

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


void RF200_sleep()  
{
  digitalWrite(LS_GPS_ENABLE, LOW);
  digitalWrite(LS_VERSION_ENABLE, LOW);
  digitalWrite(LS_GPS_V_BCKP, LOW);
    
  SPI.end();
  delay(10);
  STM32.stop();

  // Sleep...
  
  SPI.begin();
  digitalWrite(LS_GPS_ENABLE, LOW);
  digitalWrite(LS_VERSION_ENABLE, LOW);
  digitalWrite(LS_GPS_V_BCKP, LOW);
  
}

void generateMessage(bool satellite) {
 
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
  Serial.print("  lum: ");
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
  
  uint32_t fixage = (RTC.getEpoch() - FixEpoch)/60; // in minutes

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
  payloadLength = 20;
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

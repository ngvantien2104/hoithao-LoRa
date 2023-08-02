/*
 *  _                                   ____                       
 * | |    __ _  ___ _   _ _ __   __ _  / ___| _ __   __ _  ___ ___ 
 * | |   / _` |/ __| | | | '_ \ / _` | \___ \| '_ \ / _` |/ __/ _ \
 * | |__| (_| | (__| |_| | | | | (_| |  ___) | |_) | (_| | (_|  __/
 * |_____\__,_|\___|\__,_|_| |_|\__,_| |____/| .__/ \__,_|\___\___|
 *                                           |_|                   
 * Copyright (C) 2020 Lacuna Space Ltd.
 *
 * Description: CW for antenna analysis example sketch
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
#define TX_INTERVAL 5

#include <LibLacuna.h>
#include <SPI.h>

static lsLoraTxParams txParams;

void setup() {
  Serial.begin(9600);

  pinMode(LS_LED_BLUE, OUTPUT);

  pinMode(LS_GPS_ENABLE, OUTPUT);
  digitalWrite(LS_GPS_ENABLE, LOW);
  pinMode(LS_GPS_V_BCKP, OUTPUT);
  digitalWrite(LS_GPS_V_BCKP, LOW);

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

  // SX1262 configuration for lacuna LS200 board
  lsSX126xConfig cfg;
  lsCreateDefaultSX126xConfig(&cfg, BOARD_VERSION);

  // Initialize SX1262
  int result = lsInitSX126x(&cfg, REGION);
  Serial.print("E22/SX1262: ");
  Serial.println(lsErrorToString(result));
 
  // transmission parameters for CW
  lsCreateDefaultLoraTxParams(&txParams, REGION);
 
}

void loop() {

  digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);
  delay(100);
  digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);

  txParams.power = 17;
  txParams.frequency = 868000000;

  Serial.print("Frequency: ");
  Serial.print(txParams.frequency/1e6);
  Serial.print(" Mhz, Power: ");
  Serial.print(txParams.power);
  Serial.println(" dBm.");
  
  // Transmit CW at 868 Mhz for 5 seconds
  Serial.println("Starting CW"); 
  int cw_result  = lsStartCW(&txParams);
  delay(5000);
  Serial.println("Stopping CW"); 
  cw_result  = lsStopCW();

  // Wait 1 second
  delay(1000);

  // Sweep 862 to 865 Mhz in 50 kHz steps of 25 ms duration
  Serial.println("Starting sweep CW"); 
  int sweep_result  = lsSweepCW(&txParams, 862000000, 865000000, 50000, 25);
  Serial.println("Sweep CW finished"); 
  
  delay(TX_INTERVAL*1000);

}

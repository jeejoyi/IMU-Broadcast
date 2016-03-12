////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Arduino
//
//  Copyright (c) 2014, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#define BLE_DEBUG 1
#define IMU_DEBUG 0
#define MES_BUF 18
// Bluetooth Libraries
#include "SPI.h"
#include "lib_aci.h"
#include "aci_setup.h"
#include "uart_over_ble.h"
#include "services.h"



// Libraries for IMU
#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h" 
#include "CalLib.h"
#include <EEPROM.h>
#include "string.h"
#include "DSRTCLib.h"
#include <Wire.h>
#include <avr/power.h>
#include <avr/sleep.h>

static struct aci_state_t aci_state;

// IMU Necessities  
RTIMU *imu;                                           // the IMU object
RTIMUSettings settings;                               // the settings object

// Buffer for bluetooth broadcast
uint8_t ble_rx_buffer[21];
uint8_t ble_rx_buffer_len = 0;
uint8_t * message;



//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port
#define  SERIAL_PORT_SPEED  9600

unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;

unsigned long time;

void setup()
{
  int errcode;
  message = new uint8_t[MES_BUF];
  Serial.begin(SERIAL_PORT_SPEED);
  Wire.begin();
  BLEsetup();
  // Initialize IMU and Create IMU object
  imu = RTIMU::createIMU(&settings);                        // create the imu object
  imu->IMUInit();

  
  lastDisplay = lastRate = millis();
  sampleCount = 0;
  pinMode(10, OUTPUT);

  
  
}

void loop()
{  
    unsigned long now = millis();
    unsigned long delta;
    
    time = millis();
    aci_loop();
    imu->IMURead(time);
    
//  fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
    sampleCount++;

    message[0] = sampleCount;
    getMessage(0, imu->getAccel());
    getMessage(6, imu->getGyro());
    //getMessage(7, imu->getCompass());
    lib_aci_set_local_data(&aci_state, PIPE_BROADCAST_BROADCAST_SET, (uint8_t*) message, MES_BUF); 
    //Serial.println(time);
    // Display to Serial port debugging
    if ((delta = now - lastRate) >= 1 && IMU_DEBUG)
    {
      RTVector3 data;
      Serial.print(time); Serial.print(",");
      Serial.print(imu->getAccel().data(0)); Serial.print(",");
      Serial.print(imu->getAccel().data(1)); Serial.print(",");
      Serial.print(imu->getAccel().data(2)); Serial.println();
      sampleCount = 0;
      lastRate = now;
    }

}

void getMessage(int index, RTVector3 data)
{
  int16_t x = round(1000 * data.data(0));
  int16_t y=  round(1000 * data.data(1));
  int16_t z = round(1000 * data.data(2));
  Serial.print("x: "); Serial.print(data.data(0) * 1000); Serial.print(" ");  Serial.println(data.data(0)); 
  Serial.print("y: "); Serial.print(data.data(1) * 1000); Serial.print(" "); Serial.println(data.data(1));
  Serial.print("z: "); Serial.print(data.data(2) * 1000); Serial.print(" "); Serial.println(data.data(2));
  message[index + 0] = (x >> 8) & 0xFF;
  message[index + 1] = (x)      & 0xFF;
  message[index + 2] = (y >> 8) & 0xFF;
  message[index + 3] = (y)      & 0xFF;
  message[index + 4] = (z >> 8) & 0xFF;
  message[index + 5] = (z)      & 0xFF;
  Serial.print("x: "); Serial.print(x);
  Serial.print(" message"); Serial.print(index + 0); Serial.print(": "); Serial.print(message[index + 0], BIN);
  Serial.print(" message"); Serial.print(index + 1); Serial.print(": "); Serial.println(message[index + 1], BIN);
  
  Serial.print("y: "); Serial.print(y);
  Serial.print(" message"); Serial.print(index + 2); Serial.print(": "); Serial.print(message[index + 2], BIN);
  Serial.print(" message"); Serial.print(index + 3); Serial.print(": "); Serial.println(message[index + 3], BIN);
  
  Serial.print("z: "); Serial.print(z);
  Serial.print(" message"); Serial.print(index + 4); Serial.print(": "); Serial.print(message[index + 4], BIN);
  Serial.print(" message"); Serial.print(index + 5); Serial.print(": "); Serial.println(message[index + 5], BIN);
}





// Arduino UNO with CMPS14 i2c (calibration)
// Copyright (C) 2022 https://www.roboticboat.uk
// 6ab8ac9b-75f3-45dd-ace5-fda74dc22fb1
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
// These Terms shall be governed and construed in accordance with the laws of 
// England and Wales, without regard to its conflict of law provisions.


// Arduino 1.8.19 IDE

#include <Wire.h>

#define _i2cAddress         0x60
#define CALIBRATION_QUALITY_ADDRESS  0x1E

// https://stackoverflow.com/questions/111928 (nice trick)
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"

#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

// Timer
unsigned long mytime;

// Character array
char Message[100];

void writeToCMPS14(byte n){

  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);
    
  // Want the Command Register
  Wire.write(byte(0x00));

  // Send some data    
  Wire.write(n);
  
  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){

    Serial.println("communication error");
  }
  else
  { 
    Serial.println("OK");
  }

  // Wait 100ms
  delay(100);
}

byte PrintCalibrationQuality(){

  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(CALIBRATION_QUALITY_ADDRESS);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0) return 0;
  
  // Request 1 byte from CMPS14
  int nReceived = Wire.requestFrom(_i2cAddress, 1);

  // Timed out so return
  if (nReceived != 1) return 0;
  
  // Read the values
  byte calibration = Wire.read();

  sprintf(Message,"Calibration " BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(calibration));
  Serial.print(Message);
  return calibration;
}

void setup() {

  // Keep the User informed
  Serial.begin(9600);

  // Set i2c network
  Wire.begin();
  delay(1000);

  writeToCMPS14(0x98);
  writeToCMPS14(0x95);
  writeToCMPS14(0x99);
  writeToCMPS14(0x81);

  byte value = 0;
  Serial.println(value & 0x03);
  while ((value & 0x03) != 3) {
    delay(1000);
    value = PrintCalibrationQuality();
    Serial.println(value & 0x03); 
  }

  writeToCMPS14(0x98);
  writeToCMPS14(0x95);
  writeToCMPS14(0x99);
  writeToCMPS14(0x80);

  writeToCMPS14(0xF0);
  writeToCMPS14(0xF5);
  writeToCMPS14(0xF6);


  PrintCalibrationQuality();
}

void loop() {
  PrintCalibrationQuality();

}

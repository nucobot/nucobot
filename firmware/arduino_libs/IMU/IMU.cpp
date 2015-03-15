
/************************************************************************/
/*                                                                      */
/*    IMU.h --  IMU Driver for MPU6000                                  */
/*                                                                      */
/************************************************************************/
/*  Author:   Thar0l (Gusarov Aleksei)                                  */
/*  Copyright 2015, Thar0l.                                             */
/************************************************************************/
/*
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
/************************************************************************/
/*  File Description:                                                   */
/*                                                                      */
/*  This header file contains the object class declarations and other   */
/*  interface declarations need to use the gyroscope and accelerometer  */  
/*  MPU6000                                                             */
/*                                                                      */
/************************************************************************/
/*  Revision History:                                                   */
/*                                                                      */
/*  01/03/2015(Thar0l): created                                         */
/*                                                                      */
/************************************************************************/

#include <SPI.h>
#include <IMU.h>

void IMU::SPIwrite(byte reg, byte data) 
{
  uint8_t dump;
  digitalWrite(pin, LOW);
  dump = SPI.transfer(reg);
  dump = SPI.transfer(data);
  digitalWrite(pin, HIGH);
}


uint8_t IMU::SPIread(byte reg)
{
  uint8_t dump;
  uint8_t return_value;
  uint8_t addr = reg | 0x80;
  digitalWrite(pin, LOW);
  dump = SPI.transfer(addr);
  return_value = SPI.transfer(0x00);
  digitalWrite(pin, HIGH);
  return(return_value);
}


int IMU::AcceX()
{
  uint8_t AcceX_H = SPIread(0x3B);
  uint8_t AcceX_L = SPIread(0x3C);
  int16_t AcceX = AcceX_H<<8 | AcceX_L;
  return(AcceX);
}


int IMU::AcceY()
{
  uint8_t AcceY_H = SPIread(0x3D);
  uint8_t AcceY_L = SPIread(0x3E);
  int16_t AcceY = AcceY_H<<8 | AcceY_L;
  return(AcceY);
}


int IMU::AcceZ()
{
  uint8_t AcceZ_H = SPIread(0x3F);
  uint8_t AcceZ_L = SPIread(0x40);
  int16_t AcceZ = AcceZ_H<<8 | AcceZ_L;
  return(AcceZ);
}


int IMU::GyroX()
{
  uint8_t GyroX_H = SPIread(0x43);
  uint8_t GyroX_L = SPIread(0x44);
  int16_t GyroX = GyroX_H<<8 | GyroX_L;
  return(GyroX);
}


int IMU::GyroY()
{
  uint8_t GyroY_H = SPIread(0x45);
  uint8_t GyroY_L = SPIread(0x46);
  int16_t GyroY = GyroY_H<<8 | GyroY_L;
  return(GyroY);
}


int IMU::GyroZ()
{
  uint8_t GyroZ_H = SPIread(0x47);
  uint8_t GyroZ_L = SPIread(0x48);
  int16_t GyroZ = GyroZ_H<<8 | GyroZ_L;
  return(GyroZ);
}

void IMU::Init(int ChipSelPin)
{
  pin = ChipSelPin;
  SPIwrite(0x6B, 0x80);   // DEVICE_RESET @ PWR_MGMT_1, reset device
  delay(150);
  SPIwrite(0x6B, 0x03);  // TEMP_DIS @ PWR_MGMT_1, wake device and select GyroZ clock
  delay(150);
  SPIwrite(0x6A, 0x10);  // I2C_IF_DIS @ USER_CTRL, disable I2C interface
  delay(150);
  SPIwrite(0x19, 0x00);  // SMPRT_DIV @ SMPRT_DIV, sample rate at 1000Hz
  delay(150);
  SPIwrite(0x1A, 0x03);  // DLPF_CFG @ CONFIG, digital low pass filter at 42Hz
  delay(150);
  SPIwrite(0x1B, 0x00);  // FS_SEL @ GYRO_CONFIG, gyro scale at 250dps
  delay(150);
  SPIwrite(0x1C, 0x00);  // AFS_SEL @ ACCEL_CONFIG, accel scale at 2g (1g=8192)
  delay(150);
}
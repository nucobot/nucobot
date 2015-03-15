/************************************************************************/
/*																		*/
/*		IMU.h	--	Interface Declarations for IMU.cpp			        */
/*																		*/
/************************************************************************/
/*	Author:		Thar0l (Gusarov Aleksei)								*/
/*	Copyright 2015, Thar0l.      										*/
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
/*  File Description:													*/
/*																		*/
/*	This header file contains the object class declarations and other	*/
/*	interface declarations need to use the gyroscope and accelerometer  */	
/*	MPU6000		               											*/
/*																		*/
/************************************************************************/
/*  Revision History:													*/
/*																		*/
/*	01/03/2015(Thar0l): created											*/
/*																		*/
/************************************************************************/


#include <SPI.h>
#include <math.h>


class IMU 
{
private:
	int pin;
	void SPIwrite(byte reg, byte data);
	uint8_t SPIread(byte reg);	
public:
	void Init(int ChipSelPin);
	int AcceX();
	int AcceY();
	int AcceZ();
	int GyroX();
	int GyroY();
	int GyroZ();
};
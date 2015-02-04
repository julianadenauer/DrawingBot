/*
 PAN3204.cpp - Part of optical mouse sensor library for Arduino
 Copyright (c) 2008 Martijn The.  All right reserved.
 http://www.martijnthe.nl/
 
 Based on sketches by Benoît Rousseau.
 
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

/******************************************************************************
 * Includes
 ******************************************************************************/

#include "PAN3204.h"

/******************************************************************************
 * Definitions
 ******************************************************************************/
 
#define Delta_Y				0x04
#define Delta_X				0x03
#define Motion_Status		0x02
#define Operation_Mode		0x05

#define Mask_Motion			0x80
#define Mask_DYOVF			0x10
#define Mask_DXOVF			0x08

/******************************************************************************
 * Constructors
 ******************************************************************************/


PAN3204::PAN3204(uint8_t sclkPin, uint8_t sdioPin) : OptiMouse::OptiMouse(sclkPin, sdioPin)
{

}

/******************************************************************************
 * User API
 ******************************************************************************/

bool PAN3204::verifyPID(void)
{
	uint8_t val = readRegister(0x00);
//	Serial.print(val);
	if (val == 0x30)
	{
		val = readRegister(0x01);
//		Serial.print(", ");
//		Serial.print(val);
		return ((val & 0xf0) == 0x50);
//		Serial.print(" ");
	}
//	Serial.print(" ");
	
	return false;
}
 
void PAN3204::updateStatus(void)
{
	_status = readRegister(Motion_Status);
}

signed char PAN3204::dx(void)
{
	return (signed char) readRegister(Delta_X);
}

signed char PAN3204::dy(void)
{
	return (signed char) readRegister(Delta_Y);
}

uint8_t PAN3204::motion() const
{
	return (uint8_t) (_status & Mask_Motion) == Mask_Motion;
}

uint8_t PAN3204::dxOverflow() const
{
	return (uint8_t) (_status & Mask_DXOVF) == Mask_DXOVF;
}
uint8_t PAN3204::dyOverflow() const
{
	return (uint8_t) (_status & Mask_DYOVF) == Mask_DYOVF;
}


void PAN3204::setOperationMode(bool enableShutter, bool enableSleep)
{
	uint8_t val = (1 << 5);	// must-have value (datasheet)
	
	if (enableSleep) val |= ((1 << 4) | (1 << 3)); 	// (all sleep modes enabled)	
	if (enableShutter) val |= (1 << 7);

	writeRegister(Operation_Mode, val);
}


// Private Methods /////////////////////////////////////////////////////////////


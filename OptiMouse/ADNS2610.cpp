/*
 ADNS2610.cpp - Part of optical mouse sensor library for Arduino
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

#if defined(ARDUINO) && ARDUINO < 100
#include "WConstants.h"
#endif

#include "OptiMouse.h"
#include "ADNS2610.h"

/******************************************************************************
 * Definitions
 ******************************************************************************/

#define Delta_Y				0x02
#define Delta_X				0x03

/******************************************************************************
 * Constructors
 ******************************************************************************/


ADNS2610::ADNS2610(uint8_t sclkPin, uint8_t sdioPin) : OptiMouse::OptiMouse(sclkPin, sdioPin)
{

}

/******************************************************************************
 * User API
 ******************************************************************************/

bool ADNS2610::verifyPID(void)
{
	uint8_t val = readRegister(0x01);
	if ((val & 0xe0) == 0x00)
	{
		val = readRegister(0x11);
		return ((val & 0xf) == 0xf);
	}	
	return false;
}
 
void ADNS2610::setSleepEnabled(bool enabled)
{
	uint8_t val = readRegister(0x0);
	if (enabled) val &= 0xfe;
	else val |= 0x1;
	writeRegister(0x0, val);
} 
 
signed char ADNS2610::dx(void)
{
	return (signed char) readRegister(Delta_X);
}

signed char ADNS2610::dy(void)
{
	return (signed char) readRegister(Delta_Y);
}

// Private Methods /////////////////////////////////////////////////////////////


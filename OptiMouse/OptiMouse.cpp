/*
 OptiMouse.cpp - Part ofoptical mouse sensor library for Arduino
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

#include "OptiMouse.h"

/******************************************************************************
 * Definitions
 ******************************************************************************/


/******************************************************************************
 * Constructors
 ******************************************************************************/

OptiMouse::OptiMouse(uint8_t sclkPin, uint8_t sdioPin)
{
  _sclkPin = sclkPin;
  _sdioPin = sdioPin;
  pinMode (_sclkPin, OUTPUT);
  pinMode (_sdioPin, INPUT);
}


/******************************************************************************
 * User API
 ******************************************************************************/

void OptiMouse::begin(void)
{
	// Re-sync (see datasheet §5.4):
	// Toggle the SLCK line from high to low to high....
	digitalWrite(_sclkPin, HIGH);                     
	delayMicroseconds(5);
	digitalWrite(_sclkPin, LOW);
	delayMicroseconds(1);
	digitalWrite(_sclkPin, HIGH);
	
	// Wait at least tSIWTT (0.9 second?) for the
	// OptiMouse serial transaction timer to time out:
	delay(1000);
}

// Private Methods /////////////////////////////////////////////////////////////


/* 
achim: zumindest die adns chips verlangen 250ns für die phasenlänge von sclk. 
dies wird hier möglicherweise nur durch den digitalWrite(...) funktionsaufruf eingehalten
(atmel datasheet: max. schaltgeschw. eines pin?)
*/

uint8_t OptiMouse::readRegister(uint8_t address)
{
	int i;
	uint8_t r = 0;
		
	// achim: sdio is held by mousesensor until sclk goes down, thus clock should
	// go down before sdio is configured as output.
	digitalWrite (_sclkPin, LOW);	
	delayMicroseconds(1);

	// Configure sdio as output
	pinMode (_sdioPin, OUTPUT);

	// Initialize read by sending 0 as msb
	digitalWrite (_sdioPin, LOW);
	digitalWrite (_sclkPin, HIGH);
	
	// hier jetzt timing? sclk pin wird evtl. zu schnell wieder high gesetzt? (for schleifen init?)
	
	// Write the address of the register we want to read:	
	for (i=6; i>=0; i--)
	{
		digitalWrite (_sclkPin, LOW);
		digitalWrite (_sdioPin, address & (1 << i));		// achim: optimize!!!  aber dann timing???
		digitalWrite (_sclkPin, HIGH);
	}
	
	// achim:  sdio  muss seinen wert für adns2610 jetzt mindestens 250ns halten:
	delayMicroseconds(1); 
	
	// achim: jetzt steht in _sdioPin output register möglicherweise noch eine 1! Wenn 
	// dann auf input geschaltet wird ist pullup gesetzt. Im Datenblatt steht hi-z was wohl für
	// hohe impedanz steht, daher folgende zeile dazugefügt damit pullup sicher aus ist:
	digitalWrite(_sdioPin, LOW);
	
	// Switch data line from OUTPUT to INPUT
	pinMode (_sdioPin, INPUT);
	
	// Wait a bit...
	delayMicroseconds(99);		// für adns2610 wird gefordert dass sclk für insgesamt 100us high bleibt
								// für die pan-chips anscheinend nur 3 us
								
								// hier define für verschiedene chips einführen
								
								
	// Fetch the data!
	for (i=7; i>=0; i--)
	{                             
		digitalWrite (_sclkPin, LOW);
		digitalWrite (_sclkPin, HIGH);
		r |= (digitalRead (_sdioPin) << i);		// achim: optimize!!! aber timing???
	}
	
	//delayMicroseconds(100);		// achim: so ein delay ist in keinem datasheet nach der datenausgabe von read notwendig!
									// bei den adns wird höchstens 250ns angegeben die in jedem fall gewährleistet sein dürften
									// --> das datasheet für 2083 habe ich allerdings nicht gefunden!
	delayMicroseconds(1);			// zur sicherheit aber trotzdem 1us...								
	
	
	return r;
}

void OptiMouse::writeRegister(uint8_t address, uint8_t data)
{
	int i;
	
	// Set MSB high, to indicate write operation:
//	address |= 0x80;
	
	// achim: in case of former read, sdio is held by mousesensor until sclk goes down, thus clock should
	// go down before sdio is configured as output.
	digitalWrite (_sclkPin, LOW);	
	delayMicroseconds(1);

	// Configure sdio as output
	pinMode (_sdioPin, OUTPUT);

	// Initialize write by sending 1 as msb
	digitalWrite (_sdioPin, HIGH);
	digitalWrite (_sclkPin, HIGH);
	
	// hier jetzt timing? sclk pin wird evtl. zu schnell wieder high gesetzt? (timing vom for-schleifen init?)
	
	// Write the address:
	for (i=6; i>=0; i--)
	{
		digitalWrite (_sclkPin, LOW);
		digitalWrite (_sdioPin, address & (1 << i));		// achim: optimize!!! aber timing???
		digitalWrite (_sclkPin, HIGH);
	}
	
	// Write the data:
	for (i=7; i>=0; i--)
	{
		digitalWrite (_sclkPin, LOW);
		digitalWrite (_sdioPin, data & (1 << i));			// achim: optimize!!! aber timing???
		digitalWrite (_sclkPin, HIGH);
	}
	
	delayMicroseconds(100);	// zumindest adns2610 und adns2051 wollen 100us zwischen write/write und write/read sehen
}

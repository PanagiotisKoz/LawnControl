/*
 * 	MCP4131.h
 *
 *	<one line to give the program's name and a brief idea of what it does.>
 *
 *	Copytight (C) 17 Οκτ 2019 Panagiotis charisopoulos
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __arm__
#error You must use this code only for RPi
#endif

#ifndef MCP4131_H_
#define MCP4131_H_

#include <stdint.h>
#include <unistd.h>
#include "SPI.h"

class MCP4131 : private SPI{
public:

	enum Pot_fixed_values : uint8_t {
		pot_min = 0x00,
		pot_max = 0x80,
		pot_mid = 0x40
	};

	MCP4131( );
	MCP4131( const MCP4131& ) = delete; // non construction-copyable
	MCP4131& operator=( const MCP4131& ) = delete; // non copyable

	void Set_pot( uint8_t value ) { Write( static_cast<uint8_t>( Reg::wiper ), value ); }
	void Set_pot_min( ) { Write( static_cast<uint8_t>( Reg::wiper ), Pot_fixed_values::pot_min ); }
	void Set_pot_max( ) { Write( static_cast<uint8_t>( Reg::wiper ), Pot_fixed_values::pot_max ); }
	void Set_pot_middle( ) { Write( static_cast<uint8_t>( Reg::wiper ), Pot_fixed_values::pot_mid ); }


	void Enable() { Write( static_cast<uint8_t>( Reg::tcon ), static_cast<uint8_t>( Pot::enable ) ); }
	void Disable() { Write( static_cast<uint8_t>( Reg::tcon ), static_cast<uint8_t>( Pot::disable ) );
						usleep(50000); }

	virtual ~MCP4131();

private:

	enum class Reg : uint8_t {
		wiper	= 0x00,
		tcon	= 0x40
	};

	enum class Pot : uint8_t {
		disable = 0x08,
		enable  = 0x0f
	};
};

#endif /* MCP4131_H_ */

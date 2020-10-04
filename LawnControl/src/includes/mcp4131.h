/*
 * 	mcp4131.h
 *
 *	The MCP4131 device uses an SPI interface. This device support 7-bit
 *	resistor network, Potentiometer and Rheostat pinouts.
 *
 *	This code offer interface to manipulate this chip.
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

#include "spi.h"

class MCP4131 final {
public:

	enum Pot_fixed_values : uint8_t {
		pot_min = 0x00, pot_max = 0x80, pot_mid = 0x40
	};

	MCP4131() : m_enabled { false } {}
	~MCP4131() {}

	void set_pot( uint8_t value );
	void set_pot_min();
	void set_pot_max();
	void set_pot_middle();

	bool is_enabled() const { return m_enabled; }

	void enable();
	void disable();

private:

	enum class Reg : uint8_t {
		wiper = 0x00, tcon = 0x40
	};

	enum class Pot : uint8_t {
		disable = 0x08, enable = 0x0f
	};

	SPI m_SPI;
	bool m_enabled;
};

#endif /* MCP4131_H_ */

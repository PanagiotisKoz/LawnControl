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

#ifndef MCP4131_H_
#define MCP4131_H_

#include <stdint.h>

class MCP4131 {
public:
	enum class Channel : uint8_t {
		channel0,
		channel1
	};

	enum Pot_fixed_values : uint8_t {
		pot_min = 0x00,
		pot_max = 0x80,
		pot_mid = 0x40
	};

	// This class must be created once. So we use singletone pattern.
	static MCP4131& Init(const Channel channel = Channel::channel0, const int speed = 1000000 ){
		static MCP4131 instance{ channel, speed };
	    return instance;
	}

	void Set_pot( uint8_t value ) { write( static_cast<uint8_t>( Reg::wiper ), value ); }
	void Set_pot_min( ) { write( static_cast<uint8_t>( Reg::wiper ), Pot_fixed_values::pot_min ); }
	void Set_pot_max( ) { write( static_cast<uint8_t>( Reg::wiper ), Pot_fixed_values::pot_max ); }
	void Set_pot_middle( ) { write( static_cast<uint8_t>( Reg::wiper ), Pot_fixed_values::pot_mid ); }


	void Enable() { write( static_cast<uint8_t>( Reg::tcon ), static_cast<uint8_t>( Pot::enable ) ); }
	void Disable() { write( static_cast<uint8_t>( Reg::tcon ), static_cast<uint8_t>( Pot::disable ) ); }

	virtual ~MCP4131();

private:
	void write(const uint8_t reg, uint8_t value);

	MCP4131(const Channel channel, const int speed );

	int 	 m_spi_fd;
	uint32_t m_spi_speed;
	Channel	 m_spi_channel;

	enum class Reg : uint8_t {
		wiper	= 0x00,
		tcon	= 0x40
	};

	enum class Pot : uint8_t {
		disable = 0x08,
		enable  = 0x0f
	};

	const uint8_t m_spi_BPW = 8; // SPI bits per word.
};

#endif /* MCP4131_H_ */

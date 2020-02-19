/*
 * 	mcp4131.cpp
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

#include "includes/mcp4131.h"

void MCP4131::set_pot( uint8_t value )
{
	m_SPI.write( static_cast< uint8_t >( Reg::wiper ), value );
}

void MCP4131::set_pot_min()
{
	m_SPI.write( static_cast< uint8_t >( Reg::wiper ), Pot_fixed_values::pot_min );
}

void MCP4131::set_pot_max()
{
	m_SPI.write( static_cast< uint8_t >( Reg::wiper ), Pot_fixed_values::pot_max );
}

void MCP4131::set_pot_middle()
{
	m_SPI.write( static_cast< uint8_t >( Reg::wiper ), Pot_fixed_values::pot_mid );
}

void MCP4131::enable()
{
	m_SPI.write( static_cast< uint8_t >( Reg::tcon ), static_cast< uint8_t >( Pot::enable ) );
	m_enabled = true;
}

void MCP4131::disable()
{
	m_SPI.write( static_cast< uint8_t >( Reg::tcon ), static_cast< uint8_t >( Pot::disable ) );
	m_enabled = false;
}

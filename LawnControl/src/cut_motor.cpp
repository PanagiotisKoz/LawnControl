/*
 * 	cut_motor.cpp
 *
 *	Copytight (C) 25 Νοε 2019 Panagiotis charisopoulos
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

#include "includes/cut_motor.h"
#include <string>
#include <thread>
#include <pigpiod_if2.h>
#include <stdexcept>

constexpr auto tag = "Cut motor -- ";

Cut_motor::Cut_motor( int brake_pin )
	: m_brake_pin { brake_pin }, m_brake_on { false },
	  m_pot_value{ 0 }
{
	m_pi = pigpio_start( NULL, NULL );
	if ( m_pi < 0 ) {
		std::string msg( tag );
		msg += "Failed to connect to pigpiod. Please check if daemon is running.";
		throw std::runtime_error( msg );
	}

	set_mode( m_pi, m_brake_pin, PI_OUTPUT );

	brake( false );

	Motor::set_power( 0, Motor::Direction::forward );
}

Cut_motor::~Cut_motor()
{
	Motor::power_off();
	pigpio_stop( m_pi );
}

void Cut_motor::brake( bool enable )
{
	// CAUTION DO NOT CHANGE CODE SEQUENCE.

	if ( enable ) {
		Motor::power_off();

		if ( !Motor::is_pwr_on() ) {
			gpio_write( m_pi, m_brake_pin, 1 );
			m_brake_on = true;
		}
	}
	else {
		gpio_write( m_pi, m_brake_pin, 0 );
		m_brake_on = false;
	}
}

void Cut_motor::apply_power()
{
	// CAUTION DO NOT CHANGE CODE SEQUENCE.

	// If is_pwr_on is false means stop.
	if ( !Motor::is_pwr_on() ) {
		// Caution: To stop motor mcp4131 digital pot must be set to max value.
		if ( !m_mcp4131.is_enabled() )
			m_mcp4131.enable();

		m_mcp4131.set_pot_max();
		m_mcp4131.disable();
		return;
	}

	if ( m_brake_on )
		return;

	if ( !m_mcp4131.is_enabled() )
		m_mcp4131.enable();

	uint8_t pot_value = MCP4131::pot_max - ( MCP4131::pot_max * Motor::get_power() / Max_power );
	if ( pot_value != m_pot_value ) {
		m_mcp4131.set_pot( pot_value );
		m_pot_value = pot_value;
	}
}

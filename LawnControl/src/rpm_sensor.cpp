/*
 * 	rpm_sensor.cpp
 *
 *	<one line to give the program's name and a brief idea of what it does.>
 *
 *	Copytight (C) 14 Ιαν 2020 Panagiotis charisopoulos
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

#include "includes/rpm_sensor.h"

#include <pigpiod_if2.h>
#include <string>
#include <functional>
#include <stdexcept>
#include <numeric>

constexpr auto tag = "RPM Sensor -- ";

constexpr float Micros_per_minute = 60000000.0f;
constexpr unsigned Glitch_filter = 100;

Rpm_sensor::Rpm_sensor( unsigned interrupt_pin, unsigned int pulses_per_rev, unsigned watchdog )
	: m_callback_id{ 0 },m_interrupt_pin{ interrupt_pin },
	  m_pulses_per_rev{ pulses_per_rev },
	  m_last_tick{ 0 }, m_watchdog{ watchdog },
	  m_period{ 0.0f}
{
	m_pi = pigpio_start( NULL, NULL );
	if ( m_pi < 0 ) {
		std::string msg( tag );
		msg += "Failed to connect to pigpiod. Please check if daemon is running.";
		throw std::runtime_error( msg );
	}

	set_mode( m_pi, m_interrupt_pin, PI_INPUT );
	set_glitch_filter( m_pi, m_interrupt_pin, Glitch_filter );
}

Rpm_sensor::~Rpm_sensor()
{
	disable();
	pigpio_stop( m_pi );
}

void Rpm_sensor::enable()
{
	m_callback_id = callback_ex(m_pi, m_interrupt_pin, EITHER_EDGE, on_interrupt, this );

	set_watchdog( m_pi, m_interrupt_pin, m_watchdog );
}

void Rpm_sensor::disable()
{
	callback_cancel( m_callback_id );

	set_watchdog( m_pi, m_interrupt_pin, 0 );

	m_period = 0;
}

float Rpm_sensor::get_rpm() const
{
	// For multithreading safety.
	const std::lock_guard<std::mutex> lock( m_mutex );

	float rpm = 0.0f;

	if ( m_period != 0 ) {
		rpm = Micros_per_minute / ( m_period * m_pulses_per_rev );
	}

	return rpm;
}

void Rpm_sensor::on_interrupt( int pi, unsigned pin, unsigned level, unsigned tick, void *userdata )
{
	Rpm_sensor* p = reinterpret_cast< Rpm_sensor* >( userdata );

	if ( level == 1 ) { // Rising edge.
		if ( p->m_last_tick != 0 ) {
			p->m_periods_sum.push_back( tick - p->m_last_tick );
			if ( p->m_periods_sum.size() >=  p->m_pulses_per_rev )
				p->m_periods_sum.pop_front();

			{
				const std::lock_guard<std::mutex> lock( p->m_mutex );

				p->m_period = std::accumulate( p->m_periods_sum.begin(),
										p->m_periods_sum.end(), 0 )
										/ p->m_periods_sum.size();
			}

			if ( p->m_on_rpm_ready != nullptr ) {
				p->m_on_rpm_ready( p->get_rpm() );
			}
		}

		p->m_last_tick = tick;
	}
	if ( level == 2 ) { // Watchdog enabled so
		if ( p->m_on_rpm_ready ) {
			p->m_on_rpm_ready( 0.0f );
		}

		// Reset variables for rpm measure.
		p->m_period = 0;
		p->m_last_tick = 0;
		p->m_periods_sum.clear();
	}

}

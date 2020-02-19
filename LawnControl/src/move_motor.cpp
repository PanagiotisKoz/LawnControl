/*
 * 	move_motor.cpp
 *
 *	Copytight (C) 25 Οκτ 2019 Panagiotis charisopoulos
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

#include "includes/move_motor.h"
#include <string>
#include <stdexcept>
#include <pigpiod_if2.h>
#include <thread>

constexpr auto tag = "Move motor -- ";
constexpr int Max_pwm_speed = 255;
constexpr unsigned PWM_frequency = 25000;

Move_motor::Move_motor( int pwm_pin, int direction_pin )
	: m_direction_pin { direction_pin }, m_pwm_pin { pwm_pin }
{
	m_pi = pigpio_start( NULL, NULL );
	if ( m_pi < 0 ) {
		std::string msg( tag );
		msg += "Failed to connect to pigpiod. Please check if daemon is running.";
		throw std::runtime_error( msg );
	}

	set_mode( m_pi, m_pwm_pin, PI_OUTPUT );
	set_mode( m_pi, m_direction_pin, PI_OUTPUT );
	if ( set_PWM_frequency( m_pi, m_pwm_pin, PWM_frequency ) < 0 ) {
		std::string msg( tag );
		msg += "Failed to set frequency " + std::to_string( PWM_frequency )
			+ " on gpio pin " + std::to_string( m_pwm_pin ) + ".";
		throw std::runtime_error( msg );
	}

	gpio_write( m_pi, m_direction_pin, 0 );
	set_PWM_dutycycle( m_pi, m_pwm_pin, 0 );
}

Move_motor::~Move_motor()
{
	gpio_write( m_pi, m_direction_pin, 0 );
	set_PWM_dutycycle( m_pi, m_pwm_pin, 0 );

	pigpio_stop( m_pi );
}

void Move_motor::apply_power()
{
	// If is_pwr_on is false means stop.
	if ( !Motor::is_pwr_on() ) {
		set_PWM_dutycycle( m_pi, m_pwm_pin, 0 );
		return;
	}

	switch ( Motor::get_direction() ) {
		case Direction::forward:
			gpio_write( m_pi, m_direction_pin, 0 );
			break;
		case Direction::backward:
			gpio_write( m_pi, m_direction_pin, 1 );
			break;
	}

	int pwm_duty = Max_pwm_speed * Motor::get_power() / Max_power;

	set_PWM_dutycycle( m_pi, m_pwm_pin, pwm_duty );
}

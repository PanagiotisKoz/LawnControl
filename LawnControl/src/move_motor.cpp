/*
 * 	Motor.cpp
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

#include <Motor.h>
#include <wiringPi.h>

constexpr int L_mtr_sel_dir_pin = 27; // Right motor direction select pim.
constexpr int Pwm_l_mtr_pin = 13; // Left motor pwm source pin.
constexpr int R_mtr_sel_dir_pin = 17; // Left motor direction select pin.
constexpr int Pwm_r_mtr_pin = 12; // Right motor pwm source pin.
constexpr int Stop_speed = 0; // Right motor pwm source pin.

Motor::Motor()
{
	wiringPiSetupGpio();

	pinMode(Pwm_r_mtr_pin, PWM_OUTPUT);
	pinMode(L_mtr_sel_dir_pin, OUTPUT);

	pinMode(Pwm_l_mtr_pin, PWM_OUTPUT);
	pinMode(R_mtr_sel_dir_pin, OUTPUT);
}

void Motor::LMotor_move(Direction direction, uint16_t speed)
{
	if ( speed > 1024 )
		speed = 1024;

	switch ( direction ) {
		case Direction::forward:
			digitalWrite( L_mtr_sel_dir_pin, 0 );
			break;
		case Direction::backward:
			digitalWrite( L_mtr_sel_dir_pin, 1 );
			break;
	}

	pwmWrite( Pwm_l_mtr_pin, speed);
}

void Motor::LMotor_stop()
{
	digitalWrite( L_mtr_sel_dir_pin, 0 );
	pwmWrite( Pwm_l_mtr_pin, Stop_speed);
}

void Motor::RMotor_move(Direction direction, uint16_t speed)
{
	if ( speed > 1024 )
		speed = 1024;

	switch ( direction ) {
		case Direction::forward:
			digitalWrite( R_mtr_sel_dir_pin, 0 );
			break;
		case Direction::backward:
			digitalWrite( R_mtr_sel_dir_pin, 1 );
			break;
	}

	pwmWrite( Pwm_r_mtr_pin, speed);
}

void Motor::RMotor_stop()
{
	pwmWrite( Pwm_r_mtr_pin, Stop_speed);
}

Motor::~Motor()
{
}


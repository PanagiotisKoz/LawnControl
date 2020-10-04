/*
 * 	move_motor.h
 *
 *	This code manipulates dc motor that is rensponsible to move lawn mower,
 *	through LawnMower controller V1.1 board.
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

#ifndef __arm__
#error You must use this code only for RPi
#endif

#ifndef MOVE_MOTOR_H_
#define MOVE_MOTOR_H_

#include <stdint.h>
#include "motor.h"

class Move_motor final : public Motor {
public:
	Move_motor( int pwm_pin, int direction_pin );

	~Move_motor();
private:
	void apply_power();

	int m_pi; // Used for pigpiod_if2 library.
	int m_direction_pin;
	int m_pwm_pin;
};

#endif /* MOTOR_H_ */

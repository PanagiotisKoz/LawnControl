/*
 * 	Motor.h
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

#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>

class Motor {
public:
	enum class Direction {
		forward,
		backward
	};

	Motor();

	void LMotor_move( Direction, uint16_t speed ); // Speed value must be 0..1024.
	void LMotor_stop();

	void RMotor_move( Direction, uint16_t speed ); // Speed value must be 0..1024
	void RMotor_stop();

	virtual ~Motor();
};

#endif /* MOTOR_H_ */

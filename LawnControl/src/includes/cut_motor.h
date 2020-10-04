/*
 * 	cut_motor.h
 *
 *	This class provides interface to control cutting motor.
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

#ifndef CUT_MOTOR_H_
#define CUT_MOTOR_H_

#include "motor.h"
#include "mcp4131.h"

class Cut_motor final: public Motor {
public:
	explicit Cut_motor( int brake_pin );
	virtual ~Cut_motor();

	void brake( bool enable );

	bool is_brake_on() { return m_brake_on;	}

private:
	void apply_power();

	int m_pi; // Used for pigpiod_if2 library.
	int m_brake_pin;
	bool m_brake_on;
	uint8_t m_pot_value;
	MCP4131 m_mcp4131;

};

#endif /* CUTMOTOR_H_ */

/*
 * 	Sliderzaxis.h
 *
 *	This class manipulates slider that is rensponsible to move cutting disk,
 *	up down through LawnMower controller V1.1 board. The stepper system is T8-Z60.
 *
 *	Copytight (C) 26 Οκτ 2019 Panagiotis charisopoulos
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

#ifndef SLIDERZAXIS_H_
#define SLIDERZAXIS_H_

#include "Stepmtr.h"

class Slider_zaxis {
public:
	const unsigned short int Max_distance = 58; // Value is in mm.

	enum class Direction {
		up,
		down
	};

	Slider_zaxis();
	Slider_zaxis( const Slider_zaxis& ) = delete; // non construction-copyable
	Slider_zaxis& operator=( const Slider_zaxis& ) = delete; // non copyable

	// Returns slider current position.
	unsigned short int Get_cur_pos() { return m_current_pos; }

	// Set slider to specific position. Takes argument value in mm. Max 70mm.
	void Set_position( unsigned short int Pos_mm );

	void Reset();

	virtual ~Slider_zaxis();

	unsigned short int m_current_pos; // Holds slider current position in mm.

private:
	// Moves slider from current position.
	void Move_distance( Direction, unsigned int distance_mm );

	Step_mtr m_step_mottor;
};

#endif /* STEPPERMOTOR_H_ */

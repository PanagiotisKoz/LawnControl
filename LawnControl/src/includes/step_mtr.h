/*
 * 	Stepmtr.h
 *
 *	Class for step motor Nema 4240-15A.
 *
 *	Copytight (C) 27 Οκτ 2019 Panagiotis charisopoulos
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

#ifndef STEPMTR_H_
#define STEPMTR_H_

class Step_mtr {
public:
	enum class Direction {
		left,
		right
	};

	// pin1 = coil a1, pin2 = coil a2, pin3 = coil b1, pin4 = coil b2
	Step_mtr( const int pin1, const int pin2, const int pin3, const int pin4 );
	Step_mtr( const Step_mtr& ) = delete; // non construction-copyable
	Step_mtr& operator=( const Step_mtr& ) = delete; // non copyable

	void Move_steps( Direction dir, unsigned long int steps); // Each step is 1.8 degrees.

	virtual ~Step_mtr();
private:
	void Hold();
	void Stop();

	int m_pin1;
	int m_pin2;
	int m_pin3;
	int m_pin4;
};

#endif /* STEPMTR_H_ */

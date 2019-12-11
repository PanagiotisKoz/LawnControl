/*
 * 	Sliderzaxis.cpp
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

#include <Sliderzaxis.h>
#include <wiringPi.h>
#include <cmath>

constexpr int In1a_pin = 5; // Pin input In1a of L6207n.
constexpr int In2a_pin = 6; // Pin input In2a of L6207n.
constexpr int In1b_pin = 19; // Pin input In1b of L6207n.
constexpr int In2b_pin = 16; // Pin input In2b of L6207n.

// Motor specifications.
constexpr unsigned short int Steps_per_mm = 100; // Each mm is 100 steps.

Slider_zaxis::Slider_zaxis()
			: m_current_pos{ 0 }, m_step_mottor( In2b_pin, In1b_pin, In2a_pin, In1a_pin )
{
	Reset();
}

void Slider_zaxis::Move_distance(Direction direction, unsigned int distance_mm )
{
	if ( distance_mm == 0 )
		return;

	unsigned long int steps { 0 };
	switch ( direction ) {
		case Direction::down:
			steps = distance_mm * Steps_per_mm ;
			m_step_mottor.Move_steps(Step_mtr::Direction::left, steps);

			m_current_pos -= distance_mm;
			break;
		case Direction::up:
			steps = distance_mm * Steps_per_mm ;
			m_step_mottor.Move_steps(Step_mtr::Direction::right, steps);

			m_current_pos += distance_mm;
			break;
	}
}

Slider_zaxis::~Slider_zaxis()
{
}

void Slider_zaxis::Reset()
{
	Move_distance( Direction::up, Max_distance);
	m_current_pos = Max_distance;
}

void Slider_zaxis::Set_position(unsigned short int Pos_mm)
{
	if ( Pos_mm > Max_distance )
		Pos_mm = Max_distance;

	short int distance = m_current_pos - Pos_mm;

	Direction dir = distance > 0 ? Direction::down : Direction::up;
	Move_distance( dir, std::abs( distance ) );
}

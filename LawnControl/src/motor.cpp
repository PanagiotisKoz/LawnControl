/*
 * 	motor.cpp
 *
 *	<one line to give the program's name and a brief idea of what it does.>
 *
 *	Copytight (C) 21 Δεκ 2019 Panagiotis charisopoulos
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

#include "includes/motor.h"

Motor::Motor()
	: m_direction { Direction::forward }, m_power { 0 }
{
}

Motor::~Motor()
{
}

void Motor::set_power( uint8_t power, Direction dir )
{
	if ( m_direction != dir ) {
		// If we change motor direction, we must disable power first.
		m_power = 0;
		apply_power();
		m_direction = dir;
	}

	if ( power > Motor::Max_power )
		power = Motor::Max_power;

	m_power = power;

	apply_power();
}

void Motor::power_off()
{
	m_power = 0;
	apply_power();
}

void Motor::increase_pwr( uint8_t how_much )
{
	set_power( m_power + how_much, m_direction );
}

void Motor::decrease_pwr( uint8_t how_much )
{
	if ( ( m_power - how_much ) < 0 )
		m_power = 0;

	set_power( m_power, m_direction );
}

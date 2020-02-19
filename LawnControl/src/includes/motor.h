/*
 * 	motor.h
 *
 *	Base class to control motors.
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

#ifndef MOTOR_H_
#define MOTOR_H_

#include <cstdint>

constexpr uint8_t Max_power = 100;

class Motor {
public:
	const static unsigned Max_power = 100;
	const static unsigned Min_power = 0;

	enum class Direction {
		forward, backward
	};

	explicit Motor();
	virtual ~Motor();

	Direction get_direction() const { return m_direction; }
	bool is_pwr_on() const { return m_power > 0 ? true : false; }
	uint8_t get_power() const { return m_power; }

	// "power" must be between 0-100. 0 to stop motor.
	void set_power( uint8_t power, Direction dir );
	void power_off();

	void increase_pwr( uint8_t how_much = 1 );
	void decrease_pwr( uint8_t how_much = 1 );

private:
	virtual void apply_power() = 0;

	Direction m_direction;

	uint8_t m_power;
};

#endif /* INCLUDES_MOTOR_H_ */

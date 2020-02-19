/*
 * 	rpm_sensor.h
 *
 *	This class interfaces TCRT5000 Reflective Optical Sensor as rpm sensor.
 *
 *	Copytight (C) 14 Ιαν 2020 Panagiotis charisopoulos
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

#ifndef INCLUDES_RPM_SENSOR_H_
#define INCLUDES_RPM_SENSOR_H_

#include <deque>
#include <mutex>
#include <map>
#include <functional>

class Rpm_sensor final{
public:
	explicit Rpm_sensor( unsigned interrupt_pin, unsigned pulses_per_rev, unsigned watchdog );
	~Rpm_sensor();

	void enable();
	void disable();

	void register_on_rpm_ready( std::function< void( float ) > listener ) { m_on_rpm_ready = listener; }

	float get_rpm() const;

private:
	int m_pi; // Used for pigpiod_if2 library.
	int m_callback_id; // Id for cancel callback on destroy.

	unsigned m_interrupt_pin;
	unsigned m_pulses_per_rev;
	unsigned m_last_tick;
	unsigned m_watchdog;

	float m_period; // Average period per revolution.
	std::deque< unsigned int > m_periods_sum;

	mutable std::mutex m_mutex;
	std::function<void( float )> m_on_rpm_ready;

	static void on_interrupt( int pi, unsigned pin,
							  unsigned level, unsigned tick, void *userdata );
};

#endif /* INCLUDES_RPM_SENSOR_H_ */

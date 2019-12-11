/*
 * 	CutMotor.h
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

#ifndef CUTMOTOR_H_
#define CUTMOTOR_H_

#include "MCP4131.h"
#include <thread>
#include <mutex>

class Cut_motor {
public:
	Cut_motor(Cut_motor const&) = delete;
    void operator= (Cut_motor const&) = delete;

	static Cut_motor& Instance()
	{
		static Cut_motor instance;
    	return instance;
	}

	void Brake( bool enable = false );

	void Set_rpm( unsigned int rpm );
	double Get_rpm();

	void Start( unsigned int rpm );
	void Stop();

	static void Execute_ISR();

private:
	Cut_motor();
	virtual ~Cut_motor();

	void Rpm_count();

	void Speed_PID();

	MCP4131 m_mcp4131;

	std::thread m_speed_PID_thread;

	unsigned int m_count_pulses;
	std::chrono::milliseconds m_rotation_time_ms;
	std::chrono::high_resolution_clock::time_point m_start_rotation_time;

	double m_rpm;
	unsigned int m_set_rpm;

	bool m_is_running;
	bool m_brake_on;

	std::mutex m_mtx;
};

#endif /* CUTMOTOR_H_ */

/*
 * 	CutMotor.cpp
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

#include <Cut_motor.h>
#include <wiringPi.h>
#include <chrono>
#include <functional>

constexpr int Cut_sens_interrupt_pin = 25;
constexpr int Cut_mtr_break_pin = 18;
constexpr uint16_t Max_rpm = 3500; // Max allowable cutting speed in rpm.
constexpr uint16_t PID_delta_millisecs = 40; // How many milliseconds pid must wait to catch set value.
constexpr int Rotation_pulses = 2; // How many pulses to count for one rotation.
constexpr unsigned int rpm_deviation = 100; // How many rpm is allowed to variate from set value.

Cut_motor::Cut_motor()
	: m_count_pulses {0}, m_rpm {0}, m_set_rpm {0}, m_is_running {false}, m_brake_on {false}
{
	wiringPiSetupGpio();

	pinMode( Cut_mtr_break_pin, OUTPUT );
	pinMode( Cut_sens_interrupt_pin, INPUT );

	Brake( false );
}

void Cut_motor::Brake( bool enable )
{
	if ( enable ) {
		if ( !m_is_running ) {
			digitalWrite( Cut_mtr_break_pin, 1);
			m_brake_on = true;
		}
	}
	else {
			digitalWrite( Cut_mtr_break_pin, 0);
			m_brake_on = false;
	}
}

void Cut_motor::Start(unsigned int rpm)
{
	m_is_running = true;

	Set_rpm(rpm);

	m_mcp4131.Enable();

	if ( m_brake_on )
		Brake( false );

	wiringPiISR(Cut_sens_interrupt_pin, INT_EDGE_FALLING, Cut_motor::Instance().Execute_ISR );

	m_speed_PID_thread = std::thread( &Cut_motor::Speed_PID, this );
}

void Cut_motor::Stop()
{
	m_is_running = false;

	if ( m_speed_PID_thread.joinable() )
		m_speed_PID_thread.join();

	m_mcp4131.Set_pot_max();
	m_mcp4131.Disable();
}

void Cut_motor::Rpm_count()
{
	if ( m_count_pulses == 0 )
		m_start_rotation_time = std::chrono::high_resolution_clock::now();

	++m_count_pulses;

	if ( m_count_pulses == Rotation_pulses ) {
		m_rotation_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>
									( std::chrono::high_resolution_clock::now() - m_start_rotation_time );
		if ( m_rotation_time_ms.count() != 0 ) {
			m_mtx.lock();
			// Ignore false reading.
			if ( m_rotation_time_ms.count() > 10 )
				m_rpm = 60000 / m_rotation_time_ms.count();
			m_mtx.unlock();
		}
	}

	if ( m_count_pulses > Rotation_pulses )
		m_count_pulses = 0;
}

void Cut_motor::Set_rpm(unsigned int rpm)
{
	if ( m_is_running ) {
		m_mtx.lock();
		if ( rpm <= Max_rpm )
			m_set_rpm = rpm;
		else
			m_set_rpm = Max_rpm;
		m_mtx.unlock();
	}
}

double Cut_motor::Get_rpm()
{
	double tmp;

	m_mtx.lock();
	tmp = m_rpm;
	m_mtx.unlock();

	return tmp;
}

void Cut_motor::Execute_ISR()
{
	Cut_motor::Instance().Rpm_count();
}

void Cut_motor::Speed_PID()
{
	unsigned short int pot_value = 255;

	while ( m_is_running ) {
		if ( m_set_rpm != m_rpm ) {
			if ( m_set_rpm > ( m_rpm + rpm_deviation ) ) {
				if ( pot_value > 1)
					--pot_value;
			}
			if ( m_set_rpm < ( m_rpm - rpm_deviation ) ) {
				if ( pot_value < 254 )
					++pot_value;
			}

			m_mcp4131.Set_pot( pot_value );
		}
		std::this_thread::sleep_for( std::chrono::milliseconds( PID_delta_millisecs ) );
	}
}

Cut_motor::~Cut_motor()
{
}

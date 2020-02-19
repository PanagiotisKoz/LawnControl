/*
 * 	lawn_logic_manager.h
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

#ifndef INCLUDES_LAWN_LOGIC_MANAGER_H_
#define INCLUDES_LAWN_LOGIC_MANAGER_H_

#include "event_system.h"
#include "cut_motor.h"
#include "ina226.h"
#include "mcp4131.h"
#include "pid_controller.h"
#include "move_motor.h"
#include "rpm_sensor.h"
#include "mpu6050.h"
#include "slider_zaxis.h"

#include <vector>
#include <thread>

class Lawn_logic_manager final {
public:
	Lawn_logic_manager();
	~Lawn_logic_manager();

	void on_property_get( std::shared_ptr< IEvent > event );
	void on_property_set( std::shared_ptr< IEvent > event );

	void on_move( std::shared_ptr< IEvent > event );
	void on_stop( std::shared_ptr< IEvent > event );

	float get_current();
	float get_voltage();
	float get_power();
	float get_ambient_temp();

	void set_blade_rpm( unsigned long rpm ); // Zero to stop
	void set_blade_height( unsigned height_mm ); // Max is 58mm.

	void stop_move();
	void emergency_stop();
private:
	void stop_cut();
	void control_blade_speed( float rpm );

	Cut_motor m_cut_mtr;
	Move_motor m_left_mtr, m_right_mtr;
	Slider_zaxis m_slider_zaxis;

	// Power monitor sensors.
	INA226 m_gen_pwr_sens;
	INA226 m_cut_mtr_pwr_sens;

	// Motion sensor
	MPU6050 m_motion_sens;

	// RPM sensors
	Rpm_sensor m_cut_mtr_rpm_sens;
	Rpm_sensor m_left_mtr_rpm_sens;
	Rpm_sensor m_right_mtr_rpm_sens;

	// PID controller.
	PID_controller m_cut_mtr_pid;

	bool m_emergency_on;
};

#endif /* INCLUDES_LAWN_LOGIC_MANAGER_H_ */

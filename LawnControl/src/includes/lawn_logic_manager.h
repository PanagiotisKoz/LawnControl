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
	static void on_batt_low( int pi, unsigned pin,
							  unsigned level, unsigned tick, void *userdata );
	static void on_batt_charge( int pi, unsigned pin,
							  unsigned level, unsigned tick, void *userdata );

	int m_pi; // Used for pigpiod_if2 library.
	int m_callback_id_batt_low; // Id for cancel callback on destroy.
	int m_callback_id_batt_charging; // Id for cancel callback on destroy.

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
	bool m_low_batt;
	bool m_batt_charging;
};

#endif /* INCLUDES_LAWN_LOGIC_MANAGER_H_ */

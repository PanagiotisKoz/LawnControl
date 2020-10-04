#include "includes/lawn_logic_manager.h"
#include "includes/rpm_sensor.h"
#include "includes/pid_controller.h"
#include "includes/cut_motor.h"
#include "includes/ina226.h"
#include "includes/move_motor.h"
#include "includes/global_messages.h"
#include <iostream>
#include <exception>
#include <pigpiod_if2.h>
#include <stdexcept>
#include <sstream>

// Power measure sensors constants.
constexpr uint8_t Gen_pwr_measure_sens_addr = 0x41; // General power measure sensor address.
constexpr uint8_t Cut_mtr_pwr_measure_sens_addr = 0x40; // Cutting motor power sensor address.
constexpr float Gen_shunt_res = 0.012f; // General current sensor shunt resistor.
constexpr float Cut_mtr_shunt_res = 0.02f; // Cutting motor current sensor shunt resistor.
constexpr int Cut_mtr_shunt_volt_lmt = -9; // Cutting motor shunt voltage limit in millivolt.
constexpr float Gen_bus_under_volt_lmt = 23.3; // General under voltage limit.
constexpr unsigned Gen_bus_alert_pin = 26; // Alert pin for batteries under voltage.
constexpr unsigned Batt_charging_alert_pin = 23; // Charge indication pin.
constexpr float Max_batt_voltage = 26.2f;
constexpr float Min_batt_voltage = 22.2f;

// Motion sensor constants.
constexpr uint8_t Motion_sens_addr = 0x68;
constexpr unsigned Motion_sens_int_pin = 22; // Interrupt pin of mpu6050.

// Motors pins
constexpr int Pwm_l_mtr_pin = 13; // Left motor pwm source pin.
constexpr int L_mtr_sel_dir_pin = 27; // Left motor direction select pim.
constexpr int R_mtr_sel_dir_pin = 17; // Right motor direction select pin.
constexpr int Pwm_r_mtr_pin = 12; // Right motor pwm source pin.

constexpr int Cut_mtr_break_pin = 18; // Pin that enables brake.

/* TCRT5000 sensors pins. */
constexpr unsigned Cut_mtr_rpm_sens_interrupt_pin = 25;
constexpr unsigned L_mtr_rpm_sens_interrupt_pin = 20;
constexpr unsigned R_mtr_rpm_sens_interrupt_pin = 21;

// TCRT5000 pulses per revolution by motor.
constexpr unsigned Move_mtr_rpm_sens_pulses_per_rev = 5;
constexpr unsigned Cut_mtr_rpm_sens_pulses_per_rev = 2;

// Slider constants.
constexpr int Slider_limit_switch_pin = 24; // Pin input upper limit switch.
constexpr int Slider_lower_height = 30; // Lower blade height from ground in mm.
constexpr int Slider_max_height = 90; // Lower blade height from ground in mm.


constexpr int I2C_bus = 1;

// Cut motor PID tunings.
constexpr float Cut_mtr_pid_kp_ki = 0.001f;
constexpr float Cut_mtr_pid_kd = 1.5f;

// Motor min - max values.
constexpr unsigned Cut_mtr_max_rpm = 2500;
constexpr unsigned Cut_mtr_min_rpm = 500;
constexpr unsigned Move_mtr_max_rpm = 98;

// Motor rpm sensors watchdog.
constexpr unsigned Cut_mtr_Watchdog = 1000; // Value in ms that sensor not detect pulse.
constexpr unsigned Move_mtr_Watchdog = 5000; // Value in ms that sensor not detect pulse.


constexpr auto tag = "Lawn logic manager -- ";

Lawn_logic_manager::Lawn_logic_manager()
	try : m_cut_mtr( Cut_mtr_break_pin ),
		m_left_mtr( Pwm_l_mtr_pin, L_mtr_sel_dir_pin ),
		m_right_mtr( Pwm_r_mtr_pin, R_mtr_sel_dir_pin ),
		m_slider_zaxis( Slider_limit_switch_pin ),
		m_gen_pwr_sens( I2C_bus, Gen_pwr_measure_sens_addr ),
		m_cut_mtr_pwr_sens( I2C_bus, Cut_mtr_pwr_measure_sens_addr ),
		m_motion_sens( I2C_bus, Motion_sens_addr, Motion_sens_int_pin ),
		m_cut_mtr_rpm_sens( Cut_mtr_rpm_sens_interrupt_pin, Cut_mtr_rpm_sens_pulses_per_rev, Cut_mtr_Watchdog ),
		m_left_mtr_rpm_sens( L_mtr_rpm_sens_interrupt_pin, Move_mtr_rpm_sens_pulses_per_rev, Move_mtr_Watchdog ),
		m_right_mtr_rpm_sens( R_mtr_rpm_sens_interrupt_pin, Move_mtr_rpm_sens_pulses_per_rev, Move_mtr_Watchdog ),
		m_cut_mtr_pid( Cut_mtr_pid_kp_ki, Cut_mtr_pid_kp_ki, Cut_mtr_pid_kd, 0 ),
		m_emergency_on{ false }, m_low_batt{ false }, m_batt_charging{ false }
{
	m_pi = pigpio_start( NULL, NULL );
	if ( m_pi < 0 ) {
		std::string msg( tag );
		msg += msg_gpio_no_connects;
		throw std::runtime_error( msg );
	}

	set_mode( m_pi, Gen_bus_alert_pin, PI_INPUT );
	m_callback_id_batt_low = callback_ex(m_pi, Gen_bus_alert_pin, EITHER_EDGE, on_batt_low, this );
	m_callback_id_batt_charging = callback_ex(m_pi, Batt_charging_alert_pin, EITHER_EDGE, on_batt_charge, this );

	// Initialize sensors
	m_gen_pwr_sens.config( INA226::Avg_samples::smpl_4, INA226::VBUSCT_VSHCT::ct_588us,
			INA226::VBUSCT_VSHCT::ct_2p116ms, INA226::Mode::shunt_bus_cont );
	m_cut_mtr_pwr_sens.config( INA226::Avg_samples::smpl_4, INA226::VBUSCT_VSHCT::ct_588us,
			INA226::VBUSCT_VSHCT::ct_2p116ms, INA226::Mode::shunt_bus_cont );
	m_gen_pwr_sens.calibrate( 6.4, Gen_shunt_res );
	m_cut_mtr_pwr_sens.calibrate( 6.4, Cut_mtr_shunt_res );

	// Initialize alert pins ( charging pin and low batt pin).
	if ( gpio_read( m_pi,  Batt_charging_alert_pin ) == 0 )
		m_batt_charging = true;

	if ( gpio_read( m_pi,  Gen_bus_alert_pin ) == 0 )
		m_low_batt = true;

	// This action will alert us through gpio pin 26 when voltage drop under 11.3V.
	m_gen_pwr_sens.set_alert_func( INA226::Alert_func::bus_under_voltage, Gen_bus_under_volt_lmt );
	// Sets current limit for cutting motor.
	m_cut_mtr_pwr_sens.set_alert_func( INA226::Alert_func::shunt_under_voltage, Cut_mtr_shunt_volt_lmt );

	m_cut_mtr_pid.set_out_limits( Cut_motor::Min_power, Cut_motor::Max_power );

	m_cut_mtr_rpm_sens.register_on_rpm_ready(
		std::bind(&Lawn_logic_manager::control_blade_speed, this, std::placeholders::_1 ) );

	// Add listeners
	Event_manager& event_mngr = Event_manager::get_instance();

	event_mngr.add_receiver( Event_property_get::id,
		std::bind( &Lawn_logic_manager::on_property_get, this, std::placeholders::_1 ) );
	event_mngr.add_receiver( Event_property_set::id,
		std::bind( &Lawn_logic_manager::on_property_set, this, std::placeholders::_1 ) );
	event_mngr.add_receiver( Event_stop::id,
		std::bind( &Lawn_logic_manager::on_stop, this, std::placeholders::_1 ) );
	event_mngr.add_receiver( Event_move::id,
		std::bind( &Lawn_logic_manager::on_move, this, std::placeholders::_1 ) );

} catch ( std::exception& e ) {
	std::string msg( tag );
	msg += e.what();
	throw std::runtime_error( msg );
}

Lawn_logic_manager::~Lawn_logic_manager()
{
	callback_cancel( m_callback_id_batt_low );
	callback_cancel( m_callback_id_batt_charging );

	emergency_stop();
	pigpio_stop( m_pi );
}

bool Lawn_logic_manager::has_alarm()
{
	if ( m_batt_charging ) {
		std::shared_ptr< IEvent > response = std::make_shared< Event_server_response >
				( Event_server_response::Responses::batt_charging_alert );
		Event_manager::get_instance().send_event( response );
		return true;
	}

	if ( m_low_batt ) {
		std::shared_ptr< IEvent > response = std::make_shared< Event_server_response >
				( Event_server_response::Responses::low_batt_alert );
		Event_manager::get_instance().send_event( response );
		return true;
	}

	return false;
}

void Lawn_logic_manager::on_property_get( std::shared_ptr< IEvent > event )
{
	std::shared_ptr< Event_property_get > request =
			std::static_pointer_cast< Event_property_get >( event );

	if ( has_alarm() )
		return;

	std::shared_ptr< IEvent > response;
	if ( m_batt_charging ) {
		response = std::make_shared< Event_server_response >
					( Event_server_response::Responses::batt_charging_alert );
		Event_manager::get_instance().send_event( response );
		return;
	}

	if ( m_low_batt ) {
		response = std::make_shared< Event_server_response >
					( Event_server_response::Responses::low_batt_alert );
		Event_manager::get_instance().send_event( response );
		return;
	}

	std::stringstream data;
	request->serialize( data );
	data << " ";

	unsigned percentage{ 0 };
	switch ( request->get_property_id() ) {
		case Event_property_get::Properties::get_blade_height:
			data << m_slider_zaxis.get_cur_pos();
			break;
		case Event_property_get::Properties::get_current:
			data << get_current();
			break;
		case Event_property_get::Properties::get_power:
			data << get_power();
			break;
		case Event_property_get::Properties::get_temp:
			data << get_ambient_temp();
			break;
		case Event_property_get::Properties::get_voltage:
			data << get_voltage();
			break;
		case Event_property_get::Properties::get_batt_percentage:
			percentage = ( ( get_voltage() - Min_batt_voltage ) * 100 ) / ( Max_batt_voltage - Min_batt_voltage );
			if ( percentage > 100 )
				percentage = 100;
			data << percentage;
			break;
		case Event_property_get::Properties::unknow:
			response = std::make_shared< Event_server_response >
						( Event_server_response::Responses::property_unknow );
	}

	if( response == nullptr ) {
		response = std::make_shared< Event_server_response >
					( Event_server_response::Responses::property_return, data );
	}

	Event_manager::get_instance().send_event( response );
}

void Lawn_logic_manager::on_property_set( std::shared_ptr< IEvent > event )
{
	if ( has_alarm() )
		return;

	std::shared_ptr< IEvent > response;

	std::shared_ptr< Event_property_set > request =
			std::static_pointer_cast< Event_property_set >( event );

	switch ( request->get_property_id() ) {
		case Event_property_set::Properties::blade_height:
			set_blade_height( request->get_value() );
			break;
		case Event_property_set::Properties::blade_run:
			set_blade_rpm( request->get_value() );
			break;
		case Event_property_set::Properties::unknow:
			response = std::make_shared< Event_server_response >
						( Event_server_response::Responses::property_unknow );
	}

	if( response == nullptr ) {
		response = std::make_shared< Event_server_response >
					( Event_server_response::Responses::ok );
	}

	Event_manager::get_instance().send_event( response );
}

void Lawn_logic_manager::on_move( std::shared_ptr< IEvent > event )
{
	if ( has_alarm() )
		return;

	std::shared_ptr< IEvent > response;

	std::shared_ptr< Event_move > request =
			std::static_pointer_cast< Event_move >( event );

	unsigned short power = request->get_strength();
	if ( power > 0 ) {
		switch( request->get_direction() ) {
			case Event_move::Direction::forward:
				m_left_mtr.set_power( power, Move_motor::Direction::forward);
				m_right_mtr.set_power( power, Move_motor::Direction::forward);
				break;
			case Event_move::Direction::backward:
				m_left_mtr.set_power( power, Move_motor::Direction::backward);
				m_right_mtr.set_power( power, Move_motor::Direction::backward);
				break;
			case Event_move::Direction::left:
				m_left_mtr.set_power( power, Move_motor::Direction::backward);
				m_right_mtr.set_power( power, Move_motor::Direction::forward);
				break;
			case Event_move::Direction::right:
				m_left_mtr.set_power( power, Move_motor::Direction::forward);
				m_right_mtr.set_power( power, Move_motor::Direction::backward);
				break;
			case Event_move::Direction::fr:
				m_left_mtr.set_power( power, Move_motor::Direction::forward);
				m_right_mtr.set_power( ( power / 2 ), Move_motor::Direction::forward);
				break;
			case Event_move::Direction::fl:
				m_left_mtr.set_power( ( power / 2 ), Move_motor::Direction::forward);
				m_right_mtr.set_power( power, Move_motor::Direction::forward);
				break;
			case Event_move::Direction::br:
				m_left_mtr.set_power( power, Move_motor::Direction::backward);
				m_right_mtr.set_power( ( power / 2 ), Move_motor::Direction::backward);
				break;
			case Event_move::Direction::bl:
				m_left_mtr.set_power( ( power / 2 ), Move_motor::Direction::backward);
				m_right_mtr.set_power( power, Move_motor::Direction::backward);
				break;
			case Event_move::Direction::unknow:
				response =  std::make_shared< Event_server_response >
						( Event_server_response::Responses::command_unknow );
				break;
		}
	}
	else {
		m_left_mtr.power_off();
		m_right_mtr.power_off();
	}

	if( response == nullptr ) {
		response = std::make_shared< Event_server_response >
					( Event_server_response::Responses::ok );
	}

	Event_manager::get_instance().send_event( response );
}

void Lawn_logic_manager::on_stop( std::shared_ptr< IEvent > event )
{
	stop_cut();
	stop_move();
	set_blade_height( Slider_max_height );
}

float Lawn_logic_manager::get_current()
{
	return ( -1 ) * m_gen_pwr_sens.get_current();
}

float Lawn_logic_manager::get_voltage()
{
	return m_gen_pwr_sens.get_voltage();
}

void Lawn_logic_manager::set_blade_rpm( unsigned long rpm )
{
	if ( m_emergency_on )
		return;

	if ( rpm == 0 ) {
		stop_cut();
		return;
	}

	if ( rpm < Cut_mtr_min_rpm )
		rpm = Cut_mtr_min_rpm;
	if ( rpm > Cut_mtr_max_rpm )
		rpm = Cut_mtr_max_rpm;

	m_cut_mtr_pid.set_target( rpm );

	if( !m_cut_mtr.is_pwr_on() ) {
		m_cut_mtr_rpm_sens.enable();
	}
}

void Lawn_logic_manager::set_blade_height( unsigned height_mm )
{
	m_slider_zaxis.set_position( height_mm - Slider_lower_height );
}

void Lawn_logic_manager::stop_cut()
{
	m_cut_mtr_rpm_sens.disable();
	m_cut_mtr.power_off();
}

void Lawn_logic_manager::stop_move()
{
	if ( m_left_mtr.is_pwr_on() )
		m_left_mtr.power_off();
	if ( m_right_mtr.is_pwr_on() )
		m_right_mtr.power_off();
}

void Lawn_logic_manager::emergency_stop()
{
	m_emergency_on = true;
	stop_cut();
	stop_move();

	m_cut_mtr.brake( true );

	m_slider_zaxis.cancel();
}

float Lawn_logic_manager::get_power()
{
	return get_voltage() * get_current();
}

float Lawn_logic_manager::get_ambient_temp()
{
	return m_motion_sens.get_ambient_temp();
}

void Lawn_logic_manager::control_blade_speed( float rpm )
{
	uint8_t power = m_cut_mtr_pid.calculate( rpm );
	m_cut_mtr.set_power( power, Cut_motor::Direction::forward );
}

void Lawn_logic_manager::on_batt_low( int pi, unsigned pin, unsigned level, unsigned tick, void *userdata )
{
	Lawn_logic_manager* p = reinterpret_cast< Lawn_logic_manager* >( userdata );

	if ( level == 0 ) {
		p->m_low_batt = true;
		p->stop_cut();
		p->stop_move();
	}
	else
		p->m_low_batt = false;
}

void Lawn_logic_manager::on_batt_charge( int pi, unsigned pin, unsigned level, unsigned tick, void *userdata )
{
	Lawn_logic_manager* p = reinterpret_cast< Lawn_logic_manager* >( userdata );

	if ( level == 0 ) {
		p->m_batt_charging = true;
		p->stop_cut();
		p->stop_move();
	}
	else
		p->m_batt_charging = false;
}

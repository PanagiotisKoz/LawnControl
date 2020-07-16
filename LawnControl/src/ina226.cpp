#include <vector>
#include <cmath>
#include "includes/ina226.h"

// The following group declares all INA226 internal registers.
constexpr uint8_t Reg_config 		= 0x00;
constexpr uint8_t Reg_shunt_voltage = 0x01;
constexpr uint8_t Reg_bus_voltage 	= 0x02;
constexpr uint8_t Reg_power 		= 0x03;
constexpr uint8_t Reg_current 		= 0x04;
constexpr uint8_t Reg_calibration 	= 0x05;
constexpr uint8_t Reg_mask_enable 	= 0x06;
constexpr uint8_t Reg_alert_limit 	= 0x07;
constexpr uint8_t Reg_manufac_ID 	= 0x08;
constexpr uint8_t Reg_die_ID 		= 0x09;

// Reset bit position of configuration register.
constexpr uint8_t Reset_bit = 15;

// Configuration register bit masks.
constexpr uint16_t Cfg_avg_samples_mask = 0x71ff;
constexpr uint8_t Cfg_avg_samples_lmove = 9;
constexpr uint16_t Cfg_VBUSCT_mask 		= 0x7e3f;
constexpr uint8_t Cfg_VBUSCT_lmove 		= 6;
constexpr uint16_t Cfg_VSHCT_mask 		= 0x7fc7;
constexpr uint8_t Cfg_VSHCT_lmove 		= 3;
constexpr uint16_t Cfg_Mode_mask 		= 0x7ff8;

// Alert mask/enable register bit masks.
constexpr uint16_t Alrt_latch_mask 		= 0x0001;
constexpr uint16_t Alrt_polarity_mask 	= 0x0010;

// 0.00512 is an internal fixed value used to ensure scaling is maintained properly
constexpr float Fixed_val 				= 0.00512f;
constexpr uint16_t Current_LSB_divider 	= 0x8000;
constexpr float Shunt_voltage_divider 	= 0.0025f;
constexpr uint8_t Power_divider 		= 25;

constexpr float Voltage_resolution 		= 0.00125f; // Voltage resolution of device is 1,25mV.

INA226::INA226( const int i2cbus, const uint8_t address )
	: m_current_lsb { 0 }, m_I2C_driver( i2cbus, address )
{
	reset();
}

void INA226::config( const Avg_samples avg_smpls, const VBUSCT_VSHCT bus_v_ct, const VBUSCT_VSHCT shunt_v_ct,
	Mode mode )
{
	uint16_t reg_curr = read16_t( Reg_config, 2 );

	if ( avg_smpls != Avg_samples::as_is ) {
		reg_curr = ( reg_curr & Cfg_avg_samples_mask )
			| ( static_cast< uint16_t >( avg_smpls ) << Cfg_avg_samples_lmove );
	}

	if ( bus_v_ct != VBUSCT_VSHCT::as_is ) {
		reg_curr = ( reg_curr & Cfg_VBUSCT_mask ) | ( static_cast< uint16_t >( bus_v_ct ) << Cfg_VBUSCT_lmove );
	}

	if ( shunt_v_ct != VBUSCT_VSHCT::as_is ) {
		reg_curr = ( reg_curr & Cfg_VSHCT_mask ) | ( static_cast< uint16_t >( shunt_v_ct ) << Cfg_VSHCT_lmove );
	}

	if ( mode != Mode::as_is ) {
		reg_curr = ( reg_curr & Cfg_Mode_mask ) | static_cast< uint16_t >( mode );
	}

	I2C_driver::I2C_buffer buffer;
	buffer.reg = Reg_config;
	buffer.data.push_back( reg_curr >> 8 );
	buffer.data.push_back( reg_curr );
	m_I2C_driver.write( buffer );
}

void INA226::calibrate( const float expected_max_current, const float shunt_resistor_value_ohm )
{

	m_current_lsb = expected_max_current / Current_LSB_divider;
	float cal_val = Fixed_val / ( m_current_lsb * shunt_resistor_value_ohm );

	I2C_driver::I2C_buffer buffer;
	buffer.reg = Reg_calibration;
	buffer.data.push_back( static_cast< uint16_t >( cal_val ) >> 8 );
	buffer.data.push_back( cal_val );
	m_I2C_driver.write( buffer );
}

void INA226::set_avg_samples( const Avg_samples smpl )
{
	config( smpl, VBUSCT_VSHCT::as_is, VBUSCT_VSHCT::as_is, Mode::as_is );
}

void INA226::reset()
{
	m_I2C_driver.write_bit( Reg_config, Reset_bit, true );
}

float INA226::get_shunt_voltage()
{
	int16_t ret = read16_t( Reg_shunt_voltage, 2 );

	return ret * Shunt_voltage_divider;
}

float INA226::get_voltage()
{
	int16_t ret = read16_t( Reg_bus_voltage, 2 );

	return ret * Voltage_resolution;
}

float INA226::get_current()
{
	int16_t ret = read16_t( Reg_current, 2 );

	return ret * m_current_lsb;
}

float INA226::get_power()
{

	int16_t ret = read16_t( Reg_power, 2 );

	return ret * ( Power_divider * m_current_lsb );
}

void INA226::set_alert_func( Alert_func func, float value, bool latch_enable, bool invert_polarity )
{
	uint16_t mask = ( 1 << static_cast< uint8_t >( func ) );

	if ( latch_enable )
		mask |= Alrt_latch_mask;

	if ( invert_polarity )
		mask |= Alrt_polarity_mask;

	I2C_driver::I2C_buffer buffer;
	buffer.reg = Reg_mask_enable;
	buffer.data.push_back( mask >> 8 );
	buffer.data.push_back( mask );
	m_I2C_driver.write( buffer );

	if ( func != Alert_func::conv_rdy ) {
		buffer.data.clear();

		int16_t set_val { 0 };

		switch ( func ) {
			case Alert_func::bus_over_voltage:
			case Alert_func::bus_under_voltage:
				set_val = value / Voltage_resolution;
				break;
			case Alert_func::shunt_over_voltage:
			case Alert_func::shunt_under_voltage:
				set_val = value / Shunt_voltage_divider;
				break;
			case Alert_func::pwr_over_lmt:
				set_val = value / Power_divider / m_current_lsb;
				break;
		}

		buffer.reg = Reg_alert_limit;
		buffer.data.push_back( set_val >> 8 );
		buffer.data.push_back( set_val );
		m_I2C_driver.write( buffer );
	}
}

uint16_t INA226::read16_t( uint8_t reg, uint8_t length )
{
	std::vector< uint8_t > tmp = m_I2C_driver.read( reg, 2 );

	return ( ( tmp.at( 0 ) << 8 ) | tmp.at( 1 ) );
}

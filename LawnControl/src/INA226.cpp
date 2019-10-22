/*
 * 	INA226.cpp
 *
 *	<one line to give the program's name and a brief idea of what it does.>
 *
 *	Copytight (C) 21 Οκτ 2019 Panagiotis charisopoulos
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

#include "INA226.h"
#include <vector>
#include <cmath>

// The following group declares all INA226 internal registers.
constexpr uint8_t reg_config		= 0x00;
constexpr uint8_t reg_shunt_voltage = 0x01;
constexpr uint8_t reg_bus_voltage	= 0x02;
constexpr uint8_t reg_power			= 0x03;
constexpr uint8_t reg_current		= 0x04;
constexpr uint8_t reg_calibration	= 0x05;
constexpr uint8_t reg_mask_enable	= 0x06;
constexpr uint8_t reg_alert_limit	= 0x07;
constexpr uint8_t reg_manufac_ID	= 0x08;
constexpr uint8_t reg_die_ID		= 0x09;

// Reset bit position of configuration register.
constexpr uint8_t Reset_bit = 15;

// Configuration register bit masks.
constexpr uint16_t Avg_samples_mask = 0x71ff;
constexpr uint8_t  Avg_samples_lmove = 9;
constexpr uint16_t VBUSCT_mask = 0x7e3f;
constexpr uint8_t  VBUSCT_lmove = 6;
constexpr uint16_t VSHCT_mask = 0x7fc7;
constexpr uint8_t  VSHCT_lmove = 3;
constexpr uint16_t Mode_mask = 0x7ff8;

// 0.00512 is an internal fixed value used to ensure scaling is maintained properly
constexpr float Fixed_val = 0.00512f;
constexpr uint16_t Current_LSB_divider = 0x8000;
constexpr float Shunt_voltage_divider = 0.0025f;
constexpr uint8_t Power_divider = 25;

constexpr float Voltage_resolution = 0.00125f; // Voltage resolution of device is 1,25mV.

INA226::INA226( const int i2cbus, const uint8_t address )
	: I2C_driver( i2cbus, address ), m_current_lsb {0}
{
	Reset();
}

void INA226::Config(const Avg_samples avg_smpls, const VBUSCT_VSHCT bus_v_ct,
		const VBUSCT_VSHCT shunt_v_ct, Mode mode)
{
	uint16_t reg_curr = Read16_t( reg_config, 2 );

	if ( avg_smpls != Avg_samples::as_is ) {
		reg_curr = ( reg_curr & Avg_samples_mask ) | ( static_cast<uint16_t>( avg_smpls ) << Avg_samples_lmove ) ;
	}

	if ( bus_v_ct != VBUSCT_VSHCT::as_is ) {
		reg_curr = ( reg_curr & VBUSCT_mask ) | ( static_cast<uint16_t>( bus_v_ct ) << VBUSCT_lmove );
	}

	if ( shunt_v_ct != VBUSCT_VSHCT::as_is ) {
		reg_curr = ( reg_curr & VSHCT_mask ) | ( static_cast<uint16_t>( shunt_v_ct ) << VSHCT_lmove );
	}

	if ( mode != Mode::as_is ) {
		reg_curr = ( reg_curr & Mode_mask ) | static_cast<uint16_t>( mode );
	}

	I2C_buffer buffer;
	buffer.reg = reg_config;
	buffer.data.push_back( reg_curr >> 8 );
	buffer.data.push_back( reg_curr );
	Write( buffer );
}

void INA226::Calibrate( const float exp_max_current, const float shunt_resistor_value_ohm)
{

	m_current_lsb = exp_max_current / Current_LSB_divider;
	float cal_val = Fixed_val / ( m_current_lsb * shunt_resistor_value_ohm );

	I2C_buffer buffer;
	buffer.reg = reg_calibration;
	buffer.data.push_back( static_cast<uint16_t>( cal_val ) >> 8 );
	buffer.data.push_back( cal_val );
	Write( buffer );
}

void INA226::Set_avg_samples(const Avg_samples smpl)
{
	Config( smpl, VBUSCT_VSHCT::as_is, VBUSCT_VSHCT::as_is, Mode::as_is );
}

void INA226::Reset()
{
	Write_bit( reg_config, Reset_bit, true );
}

INA226::~INA226()
{
	// TODO Auto-generated destructor stub
}

float INA226::Get_shunt_voltage()
{
	int16_t ret = Read16_t( reg_shunt_voltage, 2 );

	return ret * Shunt_voltage_divider;
}

float INA226::Get_voltage()
{
	int16_t ret = Read16_t( reg_bus_voltage, 2 );

	return ret * Voltage_resolution;
}

float INA226::Get_current()
{
	int16_t ret = Read16_t( reg_current, 2 );

	return ret * m_current_lsb;
}

float INA226::Get_power()
{

	int16_t ret = Read16_t( reg_power, 2 );

	return ret * ( Power_divider * m_current_lsb );
}

uint16_t INA226::Read16_t( uint8_t reg, uint8_t length)
{
	std::vector <uint8_t> tmp = Read( reg, 2 );

	return ( ( tmp.at(0) << 8 ) | tmp.at(1) );
}

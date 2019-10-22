/*
 * 	INA226.h
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

#ifndef __arm__
#error You must use this code only for RPi
#endif

#ifndef INA226_H_
#define INA226_H_

#include <RPi_i2c.h>

class INA226: private I2C_driver {
public:

	enum class Avg_samples : uint8_t {
		smpl_1,
		smpl_4,
		smpl_16,
		smpl_64,
		smpl_128,
		smpl_256,
		smpl_512,
		smpl_1024,
		as_is
	};

	enum class VBUSCT_VSHCT : uint8_t{
		ct_140us,
		ct_204us,
		ct_332us,
		ct_588us,
		ct_1p1ms,
		ct_2p116ms,
		ct_4p156ms,
		ct_8p244ms,
		as_is
	};

	enum class Mode : uint8_t{
		pwr_down,
		shunt_volt_trig,
		bus_volt_trig,
		shunt_bus_trig,
		pwr_down1,
		shunt_volt_cont,
		bus_volt_cont,
		shunt_bus_cont,
		as_is
	};

	INA226( const int i2cbus, const uint8_t address );
	INA226( const INA226& ) = delete; // non construction-copyable
	INA226& operator=( const INA226& ) = delete; // non copyable

	/*
	 *	This function controls the conversion time settings for both
	 *	the shunt and bus voltage measurements as well as the averaging mode used.
	 *	The operating mode that controls what signals are selected to be measured
	 *	is also configured with this function.
	 */
	void Config( const Avg_samples avg_smpls, const VBUSCT_VSHCT bus_v_ct, const VBUSCT_VSHCT shunt_v_ct, Mode mode);
	void Calibrate ( const float exp_max_current, const float shunt_resistor_value_ohm );

	float Get_voltage();
	float Get_shunt_voltage();
	float Get_current();
	float Get_power();

	void Set_avg_samples ( const Avg_samples smpl);
	void Reset();
	virtual ~INA226();

private:

	uint16_t Read16_t( uint8_t reg, uint8_t length);
	float m_current_lsb; // Current value/bit "divider".
};

#endif /* INA226_H_ */

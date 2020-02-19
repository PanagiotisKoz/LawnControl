/*
 * 	ina226.h
 *
 *	The INA226 is a digital current sense amplifier with an I2C- and
 *	SMBus-compatible interface. It provides digital current, voltage
 *	and power readings necessary for accurate decision-making in
 *	precisely-controlled systems.
 *	Programmable registers allow flexible configuration for measurement
 *	resolution as well as continuous-versus-triggered operation. Detailed
 *	register information appears at http://www.ti.com/lit/ds/symlink/ina226.pdf.
 *
 *	This code offer interface to manipulate this chip.
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

#include "rpi_i2c.h"

class INA226 final {
public:

	enum class Avg_samples : uint8_t {
		smpl_1, smpl_4, smpl_16, smpl_64, smpl_128, smpl_256, smpl_512, smpl_1024, as_is
	};

	enum class VBUSCT_VSHCT : uint8_t {
		ct_140us, ct_204us, ct_332us, ct_588us, ct_1p1ms, ct_2p116ms, ct_4p156ms, ct_8p244ms, as_is
	};

	enum class Mode : uint8_t {
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

	enum class Alert_func : uint8_t {
		conv_rdy = 10, pwr_over_lmt, bus_under_voltage, bus_over_voltage, shunt_under_voltage, shunt_over_voltage
	};

	INA226( const int i2cbus, const uint8_t address );
	~INA226() {}

	/*
	 *	This function controls the conversion time settings for both
	 *	the shunt and bus voltage measurements as well as the averaging mode used.
	 *	The operating mode that controls what signals are selected to be measured
	 *	is also configured with this function.
	 */
	void config( const Avg_samples avg_smpls, const VBUSCT_VSHCT bus_v_ct, const VBUSCT_VSHCT shunt_v_ct, Mode mode );

	void calibrate( const float expected_max_current, const float shunt_resistor_value_ohm );

	float get_voltage();
	float get_shunt_voltage();
	float get_current();
	float get_power();

	/* Selects the function that is enabled to control the Alert pin as well as how that pin functions.
	 * The 'value' used to compare to the function selected in the 'func'
	 * to determine if a limit has been exceeded.
	 * 'value' ignored when 'conv_rdy function is selected.
	 * If bus_under_voltage, bus_over_voltage or pwr_over_lmt selected, 'value' must be set in Volt or Watt.
	 * If shunt_under_voltage or shunt_over_voltage selected, 'value' must be set in millivolt.
	 */
	void set_alert_func( Alert_func func, float value, bool latch_enable = false, bool invert_polarity = false );

	void set_avg_samples( const Avg_samples smpl );
	void reset();

private:

	uint16_t read16_t( uint8_t reg, uint8_t length );
	float m_current_lsb; // Current value/bit "divider".
	I2C_driver m_I2C_driver;
};

#endif /* INA226_H_ */

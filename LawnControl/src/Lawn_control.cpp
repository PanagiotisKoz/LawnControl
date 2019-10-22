/*
 *	lawn_control.cpp
 *
 *	This is program for testing drivers interface.
 *
 *	Copyright (C) 2019 Panagiotis Charisopoulos.
 *
 *	This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 * 	You should have received a copy of the GNU General Public License
 * 	along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Cut_motor.h>
#include <Sliderzaxis.h>
#include "MPU6050.h"
#include "MCP4131.h"
#include "Motor.h"
#include "INA226.h"
#include <iostream>
#include <unistd.h>
#include <thread>
#include <future>
//#include <sys/stat.h>
//#include <linux/spi/spidev.h>

using namespace std;

constexpr int I2C_bus = 1;

// Power measure Sensors constants.
constexpr uint8_t Gen_pwr_measure_sens_addr = 0x41; // General power measure sensor address.
constexpr uint8_t Cut_mtr_pwr_measure_sens_addr = 0x40; // Cutting motor power sensor address.
constexpr float Gen_shunt_res = 0.012f; // General current sensor shunt resistor.
constexpr float Cut_mtr_shunt_res = 0.02f; // Cutting motor current sensor shunt resistor.
constexpr int Cut_mtr_shunt_volt_lmt = -10; // Cutting motor shunt voltage limit in millivolt.
constexpr float Gen_bus_under_volt_lmt = 23.3; // General under voltage limit.

int main() {

	try {
		Motor mv_mtrs;
		INA226 Gen_pwr_sens(I2C_bus, Gen_pwr_measure_sens_addr);
		INA226 Cut_mtr_pwr_sens(I2C_bus, Cut_mtr_pwr_measure_sens_addr);
		Cut_motor& cutmtr = Cut_motor::Instance();
		Slider_zaxis stepper;

		// Initialize sensors
		Gen_pwr_sens.Config( INA226::Avg_samples::smpl_4, INA226::VBUSCT_VSHCT::ct_588us,
					INA226::VBUSCT_VSHCT::ct_2p116ms, INA226::Mode::shunt_bus_cont);
		Cut_mtr_pwr_sens.Config( INA226::Avg_samples::smpl_4, INA226::VBUSCT_VSHCT::ct_588us,
				INA226::VBUSCT_VSHCT::ct_2p116ms, INA226::Mode::shunt_bus_cont);
		Gen_pwr_sens.Calibrate( 6.4, Gen_shunt_res );
		Cut_mtr_pwr_sens.Calibrate( 6.4, Cut_mtr_shunt_res );

		Gen_pwr_sens.Set_alert_func( INA226::Alert_func::bus_under_voltage, Gen_bus_under_volt_lmt);
		Cut_mtr_pwr_sens.Set_alert_func( INA226::Alert_func::shunt_under_voltage, Cut_mtr_shunt_volt_lmt);
		// End sensors initialization.
	}
	catch ( std::runtime_error& err ) {

	}

	return 0;
}

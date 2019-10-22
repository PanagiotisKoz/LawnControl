/*
 *	lawn_control.cpp
 *
 *	I2C implementation for Raspberry Pi.
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

#include "MPU6050.h"
#include "MCP4131.h"
#include "INA226.h"
#include <iostream>
#include <unistd.h>
#include <thread>
#include <linux/spi/spidev.h>

using namespace std;
constexpr int I2C_bus = 1;

bool bStop = false;

void measure( INA226 &s0, INA226 &s1)
{
	while( !bStop ) {
		cout << "Sens0: " << s0.Get_voltage() << "V, " <<
				-s0.Get_current() << "A\t";
		usleep( 1000 );
		cout << "Sens1: " << s1.Get_voltage() << "V, " <<
				-s1.Get_current() << "A" << endl;
	}
}

int main() {

	INA226 sens0(I2C_bus, 0x41);
	INA226 sens1(I2C_bus, 0x40);

	sens0.Config( INA226::Avg_samples::smpl_4, INA226::VBUSCT_VSHCT::ct_588us,
					INA226::VBUSCT_VSHCT::ct_2p116ms, INA226::Mode::shunt_bus_cont);
	sens1.Config( INA226::Avg_samples::smpl_4, INA226::VBUSCT_VSHCT::ct_588us,
				INA226::VBUSCT_VSHCT::ct_2p116ms, INA226::Mode::shunt_bus_cont);
	sens0.Calibrate( 6.4, 0.008 );
	sens1.Calibrate( 6.4, 0.012 );

	thread t1 { measure, ref(sens0), ref(sens1) };
	MCP4131 dpot;

	MPU6050::Device gy_521 ( I2C_bus );
	try {
		cout << "Device id is: 0x" << hex << +gy_521.Device_id() << endl;
		cout << dec;
		cout << "Ambient temperature is: " << fixed << gy_521.Ambient_temp() << "C" << endl;
		gy_521.Disable_temp_sensor();
		//gy_521.Load_firmware("/home/pi/projects/lawncontrol/debug/mpu6050_v6.1_firmware.bin");
		MPU6050::Gyro_accel_data gdata = gy_521.Get_gyro_raw_data();
		cout << "GX:" << gdata.X << " GY:" << gdata.Y << " GZ:" << gdata.Z << endl;

		dpot.Enable();
		dpot.Set_pot_middle();
		sleep(5);
	}
	catch ( std::runtime_error &e ) {
		cerr << e.what() << endl;
		dpot.Set_pot_max();
		dpot.Disable();
		return -1;
	}
	bStop = true;
	t1.join();

	dpot.Set_pot_max();
	dpot.Disable();
	return 0;
}

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
#include <iostream>
#include <unistd.h>
#include <linux/spi/spidev.h>

using namespace std;
constexpr int I2C_bus = 1;

int main() {
	MPU6050::Device gy_521 = MPU6050::Device::Init( I2C_bus );

	try {
		cout << "Device id is: 0x" << hex << +gy_521.Device_id() << endl;
		cout << dec;
		cout << "Ambient temperature is: " << fixed << gy_521.Ambient_temp() << "C" << endl;
		gy_521.Disable_temp_sensor();
		//gy_521.Load_firmware("/home/pi/projects/lawncontrol/debug/mpu6050_v6.1_firmware.bin");
		MPU6050::Gyro_accel_data gdata = gy_521.Get_gyro_raw_data();
		cout << "GX:" << gdata.X << " GY:" << gdata.Y << " GZ:" << gdata.Z << endl;


		MCP4131 dpot = MCP4131::Init( MCP4131::Channel::channel0, 1000000 );
		dpot.Enable();
		dpot.Set_pot_middle();
		sleep(5);
		dpot.Set_pot_max();
		dpot.Disable();

	return 0;
	}
	catch ( std::runtime_error &e ) {
		cerr << e.what() << endl;
		return -1;
	}
}

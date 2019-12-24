/*
 *	MPU6050.h
 *
 *	MPU6050 Driver declaration.
 *
 *	IMPORTAND!! Any call to member function, implicit set device to power on state.
 *				This implementation does not implement to control i2c slaves,
 *				so mpu is in i2c bypass mode by default.
 *
 *				When this object created, initializes device with following:
 *				Resets device, Set DLPF to 42hz	and finally put device into
 *				standby mode, until some function request it to wake again.
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
#ifndef __arm__
#error You must use this code only for RPi
#endif

#ifndef MPU6050_H_
#define MPU6050_H_

#include "RPi_i2c.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace MPU6050 {

constexpr uint8_t GY_521_address = 0x68;

struct Gyro_accel_data {
	short int X;
	short int Y;
	short int Z;
};

class Device {
public:
	enum class Dlpf : uint8_t {
		bw_256hz,
		bw_188hz,
		bw_98hz,
		bw_42hz,
		bw_20hz,
		bw_10hz,
		bw_5hz
	};

	enum class Guro_fsr : uint8_t {
		gfs_250dps,
		gfs_500dps,
		gfs_1000dps,
		gfs_2000dps
	};

	enum class Accel_fsr : uint8_t {
		afs_2g,
		afs_4g,
		afs_8g,
		afs_16g
	};

	Device( const int i2cbus, const uint8_t address = GY_521_address );
	Device( const Device& ) = delete; // non construction-copyable
	Device& operator=( const Device& ) = delete; // non copyable

	uint8_t Device_id(); // Get device id.

	// Returns ambient temperature. Please enable temperature sensor first.
	// If temperature sensor is disabled returns an odd value.
	float 	Ambient_temp();

	Gyro_accel_data	Get_gyro_raw_data(); // Returns data read directly by the gyro registers;
	Gyro_accel_data	Get_accel_raw_data(); // Returns data read directly by the accel registers;

	void	Set_dlpf(Dlpf bw); // Sets digital low pass filter.
	void	Set_gyro_rate(uint16_t rate = 50); // Accepted value is: 4hz < value > 1000hz.
	void	Set_gyro_fsr( Guro_fsr range = Guro_fsr::gfs_2000dps ); // Selects the full scale range of the gyroscope outputs.
	void	Set_accel_fsr( Accel_fsr range = Accel_fsr::afs_8g ); // Selects the full scale range of the gyroscope outputs.

	// Functions, for fifo management.
	void 	setTempFIFOEnabled(bool enable);
	void 	setXGyroFIFOEnabled(bool enable);
	void 	setYGyroFIFOEnabled(bool enable);
	void 	setZGyroFIFOEnabled(bool enable);
	void 	setAccelFIFOEnabled(bool enable);
	void	Enable_fifo(bool enable); // Enables FIFO operations
	void	Reset_fifo();

	// Functions, for power management.
	void Standby(); // This function puts device into standby mode.
	void Wake_up(); // This function put device into normal mode.
	void Reset(); // This function reset device.
	bool Get_pwr_state() { return m_pwr_state; }

	void	Disable_temp_sensor(bool Disable = true); // By default sensor is On. True set sensor Off, false set sensor On.

	// Functions related to Invensense Motion Driver.
	void		Load_firmware(const std::string file_path); // Return false if failed;

	~Device();

private:

	// Function related to Invensense Motion Driver.
	void 		Write_mem(uint16_t mem_address, uint16_t length, std::vector<uint8_t> data);
	std::vector<uint8_t> Read_mem(uint16_t mem_address, uint16_t length);

	bool m_dmp_on; // If dmp is enabled then true;
	bool m_pwr_state; // True for device power up.
	I2C_driver m_I2C_driver;
};

} // End of namespace MPU6050.

#endif /* CMPU6050_H_ */

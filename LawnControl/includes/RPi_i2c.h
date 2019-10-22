/*
 *	RPi_i2c.h
 *
 *	I2C interface for Raspberry Pi.
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

#ifndef _RPI_HW_DRIVER_I2C_HPP_
#define _RPI_HW_DRIVER_I2C_HPP_

#include <vector>
#include <cstdint>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <mutex>

/*!
	@class I2C_driver
	@brief Is responsible to communicate with i2c devices that is connected to RPi.
*/
class I2C_driver {

public:
	struct I2C_buffer {
		uint8_t reg; // Register address to write
		std::vector <uint8_t> data;
	};

	// Constructor.
	I2C_driver( const int i2cbus, uint8_t addr );
	I2C_driver( const I2C_driver& ) = delete; // non construction-copyable
	I2C_driver& operator=( const I2C_driver& ) = delete; // non copyable

	// Destructor.
	virtual ~I2C_driver();

	// Writes byte/s values of type i2c_byte_buffer to I2C bus.
	void Write( I2C_buffer buffer);

	// Reads 'count' of byte/s from register of I2C bus.
	std::vector<uint8_t> Read( uint8_t i2c_register, const uint16_t length );

	// Reads bit value of register at position and return bit state. Returns true for 1 or false for 0.
	bool Read_bit( uint8_t reg, uint8_t position);

	// Writes bit value at position in register. State must be true for 1 and false for 0.
	void Write_bit( uint8_t reg, uint8_t position, bool state);



private:
	enum Smbus_data_type : uint8_t {
		byte 				 = I2C_SMBUS_BYTE_DATA,
		word 				 = I2C_SMBUS_WORD_DATA,
		block_data  		 = I2C_SMBUS_BLOCK_DATA,
		i2c_read_block_data  = I2C_SMBUS_I2C_BLOCK_DATA,
		i2c_write_block_data = I2C_SMBUS_I2C_BLOCK_BROKEN
	};

	enum Smbus_access : uint8_t {
		write = 0,
		read
	};

	//! The device path.
	int m_i2cbus;

	//! The I2C slave address.
	uint32_t m_addr;

	//! File descriptor of the device.
	int m_dev_fd;

	//! Data buffer used for I2C transmission.
	uint8_t m_buffer[3];

	//! I2C adapter functionality.
	unsigned long m_funcs;

	std::mutex m_mutex;
};

#endif /* _RPI_HW_DRIVER_I2C_HPP_ */

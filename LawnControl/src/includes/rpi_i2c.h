/**
 * @brief This class implements communication with i2c devices that is connected to RPi.
 *
 * @author Panagiotis Charisopoulos.
 * @date October 22 2019.
 * @license
 * 		This program is free software: you can redistribute it and/or modify
 * 		it under the terms of the GNU General Public License as published by
 * 		the Free Software Foundation, either version 3 of the License, or
 * 		(at your option) any later version.\n\n
 * 		This program is distributed in the hope that it will be useful,
 * 		but WITHOUT ANY WARRANTY; without even the implied warranty of
 * 		MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * 		GNU General Public License for more details.\n\n
 * 		You should have received a copy of the GNU General Public License
 * 		along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * @copyright © 2019 Panagiotis Charisopoulos. All rights reserved.
 */

#ifndef __arm__
#error You must use this code only for RPi
#endif

#ifndef _RPI_I2C_H_
#define _RPI_I2C_H_

#include <vector>
#include <cstdint>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <mutex>

class I2C_driver final {

public:
	//! Holds the device register address and data
	struct I2C_buffer {
		uint8_t reg; ///< Device register address for read/write.
		std::vector< uint8_t > data; ///< Vector of bytes.
	};

	I2C_driver( const int i2cbus, uint8_t addr );
	~I2C_driver();

	void write( I2C_buffer buffer );

	std::vector< uint8_t > read( uint8_t i2c_register, const uint16_t length );

	bool read_bit( uint8_t reg, uint8_t position );

	void write_bit( uint8_t reg, uint8_t position, bool state );

private:
	enum Smbus_data_type : uint8_t {
		byte = I2C_SMBUS_BYTE_DATA,
		word = I2C_SMBUS_WORD_DATA,
		block_data = I2C_SMBUS_BLOCK_DATA,
		i2c_read_block_data = I2C_SMBUS_I2C_BLOCK_DATA,
		i2c_write_block_data = I2C_SMBUS_I2C_BLOCK_BROKEN
	};

	enum Smbus_access : uint8_t {
		_write = 0, _read
	};

	int m_i2cbus; // Holds I2C bus number.

	uint32_t m_addr; // The I2C slave address.

	int m_dev_fd; // File descriptor of I2c device.

	uint8_t m_buffer[ 3 ]; // Data buffer used for I2C transmission.

	unsigned long m_funcs; // I2C adapter functionality.

	std::mutex m_mutex; //
};

#endif /* _RPI_HW_DRIVER_I2C_HPP_ */

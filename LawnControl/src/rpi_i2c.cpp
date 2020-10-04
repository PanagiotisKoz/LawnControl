#include "includes/rpi_i2c.h"
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <stdexcept>
#include <errno.h>
#include <thread>
#include <string.h>
#include <sys/ioctl.h>

constexpr auto tag = "I2C -- ";
/**
 *  @brief Constructor.
 *  @param i2cbus I2C bus number to use.
 *  @param addr of device.
 */
I2C_driver::I2C_driver( const int i2cbus, uint8_t addr )
	: m_i2cbus { i2cbus }, m_addr { addr }
{
	std::string msg( tag );
	std::string i2cbuspath { "/dev/i2c-" };
	i2cbuspath += std::to_string( i2cbus );

	// Open device file
	if ( ( m_dev_fd = open( i2cbuspath.c_str(), O_RDWR ) ) < 0 ) {
		msg += "Failed to open i2c bus. Because: " + std::string( strerror( errno ) );
		throw std::runtime_error { msg };
	}

	// Select slave device
	if ( ioctl( m_dev_fd, I2C_SLAVE, addr ) < 0 ) {
		msg += "Failed to select slave device. Because: " + std::string( strerror( errno ) );
		throw std::runtime_error { msg };
	}

	/* Get adapter functionality */
	if ( ioctl( m_dev_fd, I2C_FUNCS, &m_funcs ) < 0 ) {
		msg += "Failed to get adapter functionality. Because: " + std::string( strerror( errno ) );
		throw std::runtime_error { msg };
	}

}

I2C_driver::~I2C_driver()
{
	::close( m_dev_fd );
}

/**
 * @brief Writes n byte/s of type i2c_byte_buffer to I2C bus.
 * @param buffer accepts a I2C_buffer type.
 */
void I2C_driver::write( I2C_buffer buffer )
{
	std::unique_lock< std::mutex > scoped_lock( m_mutex );

	if ( buffer.data.size() == 0 )
		return;

	int data_size { static_cast< int > ( buffer.data.size() ) }; // Acquire buffer data size to minimize function calls.

	if ( data_size < 1 )
		return;

	struct i2c_smbus_ioctl_data ioctl_args;
	i2c_smbus_data smbus_data;

	ioctl_args.read_write = Smbus_access::_write;
	ioctl_args.command = buffer.reg;

	// Flag for ioctl return value.
	int ioctl_ret { 0 };

	if ( m_funcs & I2C_FUNC_SMBUS_WRITE_I2C_BLOCK ) {
		ioctl_args.size = Smbus_data_type::i2c_write_block_data;

		// Keeps the number of bytes that we must write.
		uint8_t this_write { 0 };

		for ( auto i = 0; i < data_size; i += this_write ) {
			this_write = std::min( I2C_SMBUS_BLOCK_MAX - 1, data_size - i );

			smbus_data.block[ 0 ] = this_write;

			// Copy data from vector byte by byte.
			std::copy( buffer.data.begin() + i, buffer.data.begin() + i + this_write, &smbus_data.block[ 1 ] );

			ioctl_args.data = &smbus_data;

			ioctl_ret = ioctl( m_dev_fd, I2C_SMBUS, &ioctl_args );
			if ( ioctl_ret == -1 )
				break;
		}
	}
	else {
		ioctl_args.size = Smbus_data_type::byte;

		for ( uint8_t i = 0; i < data_size; ++i ) {
			smbus_data.byte = buffer.data.at( i );
			ioctl_args.data = &smbus_data;
			ioctl_ret = ioctl( m_dev_fd, I2C_SMBUS, &ioctl_args );
			if ( ioctl_ret == -1 )
				break;
		}
	}

	if ( ioctl_ret == -1 ) {
		std::string msg { strerror( errno ) };
		msg.insert( 0, "i2c Write: failed to write to the bus. Because: " );
		throw std::runtime_error { msg };
	}
}

/**
 * @brief Reads bit value of register at position and return bit state.
 * @param reg device register to access.
 * @param length bytes to read.
 * @return
 */
std::vector< uint8_t > I2C_driver::read( uint8_t reg, const uint16_t length )
{
	std::unique_lock< std::mutex > scoped_lock( m_mutex );

	std::vector< uint8_t > ret_vec;

	if ( length == 0 ) {
		return ret_vec;
	}

	i2c_smbus_data smbus_data;
	struct i2c_smbus_ioctl_data ioctl_args;

	ioctl_args.read_write = Smbus_access::_read;
	ioctl_args.command = reg;
	ioctl_args.data = &smbus_data;
	ioctl_args.size = Smbus_data_type::byte;

	int ioctl_ret { 0 };

	// Keeps the number of bytes that we must read.
	uint8_t this_read { 0 };

	auto i { 0 };
	while ( i < length ) {
		this_read = std::min( I2C_SMBUS_BLOCK_MAX - 1, length - i );
		if ( m_funcs & I2C_FUNC_SMBUS_READ_I2C_BLOCK ) {
			ioctl_args.size = Smbus_data_type::i2c_read_block_data;
			ioctl_args.data->block[ 0 ] = this_read;
			ioctl_ret = ioctl( m_dev_fd, I2C_SMBUS, &ioctl_args );
			if ( ioctl_ret == -1 )
				break;

			std::copy( std::begin( ioctl_args.data->block ) + 1, std::begin( ioctl_args.data->block ) + this_read + 1,
				std::back_inserter( ret_vec ) );
			i += this_read;
		}
		else {
			ioctl_args.size = Smbus_data_type::byte;

			ioctl_ret = ioctl( m_dev_fd, I2C_SMBUS, &ioctl_args );
			if ( ioctl_ret != -1 )
				break;

			ret_vec.push_back( ioctl_args.data->byte );
		}
	}

	if ( ioctl_ret == -1 ) {
		std::string msg( tag );
		msg += "Failed to read to the bus. Because: " + std::string( strerror( errno ) );
		throw std::runtime_error { msg };
	}

	return ret_vec;
}

/**
 * @brief Reads bit value at position in register.
 * @param reg device register to access.
 * @param bit_number valid values are 0 - 15.
 * @return
 */
bool I2C_driver::read_bit( uint8_t reg, uint8_t bit_number )
{
	if ( bit_number > 15 )
		return false;

	int num_bytes { bit_number / 8 };

	std::vector< uint8_t > recv_reg = I2C_driver::read( reg, num_bytes + 1 );

	// High byte is in position 0 of vector.
	int pos { 0 };
	if ( num_bytes != 0 ) {
		if ( bit_number > 7 )
			bit_number %= 8;
		else
			pos = 1;
	}

	return ( recv_reg.at( pos ) & ( 1 << bit_number ) );
}

/**
 * @brief Writes bit value at position in register.
 * @param reg device register to access.
 * @param bit_number valid values are 0 - 15.
 * @param state false for 0, true for 1.
 */
void I2C_driver::write_bit( uint8_t reg, uint8_t bit_number, bool state )
{
	if ( bit_number > 15 )
		return;

	int num_bytes { bit_number / 8 };
	std::vector< uint8_t > recv_reg = I2C_driver::read( reg, num_bytes + 1 );

	// High byte is in position 0 of vector.
	int pos { 0 };

	if ( num_bytes != 0 ) {
		if ( bit_number > 7 )
			bit_number %= 8;
		else
			pos = 1;
	}

	state ? recv_reg.at( pos ) |= ( 1 << bit_number ) : recv_reg.at( pos ) &= ~( 1 << bit_number );

	I2C_buffer buff;

	buff.reg = static_cast< uint8_t >( reg );
	buff.data = recv_reg;

	I2C_driver::write( buff );
}

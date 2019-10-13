/*
 *	MPU6050.h
 *
 *	MPU6050 Driver implementation.
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
#include <stdexcept>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <fstream>
#include <iterator>
#include <vector>
#include <iostream>

constexpr uint16_t Dev_mem_size = 3*1024; // Maximum device memory size 3 KB.
constexpr unsigned short Mem_start_address = 0x0400; // Set DMP program counter to this address.
constexpr unsigned short DMP_sample_rate = 200; // Sampling rate used when DMP is enabled.


namespace MPU6050 {


Device::Device( const int i2cbus, const uint8_t address )
		: I2C_driver( i2cbus, address ), m_dmp_on{ true }
{
	Reset();
	Set_gyro_fsr();
	Set_accel_fsr();
	Set_dlpf(Dlpf::bw_42hz);
	Set_gyro_rate();
	Standby();
}

Device::~Device()
{
	Reset();
}

bool Device::Mpu_pwr_on()
{
	return !Read_bit( Register::pwr_mgmt_1, Pwr_mgmt_1::standby_bit );
}

uint8_t Device::Device_id()
{
	return Read( Register::who_am_i, 1).at(0) ;
}


void MPU6050::Device::Disable_temp_sensor(bool disable)
{
	disable ? Write_bit( Register::pwr_mgmt_1, Pwr_mgmt_1::temp_dis_bit, true ) :
			Write_bit( Register::pwr_mgmt_1, Pwr_mgmt_1::temp_dis_bit, true );
}

// Returns ambient temperature. Please enable temperature sensor first.
// If temperature sensor is disabled returns an odd value.
float Device::Ambient_temp()
{
	if ( !Mpu_pwr_on() )
		Wake_up();

	if ( Read_bit( Register::pwr_mgmt_1, Pwr_mgmt_1::temp_dis_bit ) )
		Disable_temp_sensor( false );

	short int raw = ( Read( Register::temp_out_h, 1 ).at(0) << 8) |
			Read( Register::temp_out_l, 1 ).at(0);


	/*
	 * The following calculation, resulting from manual
	 * "MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2"
	 */
	float temp = raw / 340.0f + 35.0f;
	return temp;
}

/*
 * This function uploading Digital Motion Processing (DMP) firmware to device.
 * The MPU-6050 collects gyroscope and accelerometer data while synchronizing data
 * sampling at a user defined rate. The The total dataset obtained by the MPU-6050
 * includes 3-Axis gyroscope data, 3-Axis accelerometer data, and temperature data.
 * The MPU’s calculated output to the system processor can also include heading
 * data from a digital 3-axis third party magnetometer.
 *
 * The FIFO buffers the complete data set, reducing timing requirements on the
 * system processor by allowing the processor burst read the FIFO data. After
 * burst reading the FIFO data, the system processor can save power by entering
 * a low-power sleep mode while the MPU collects more data.
 *
 * Programmable interrupt supports features such as gesture recognition, panning,
 * zooming, scrolling, tap detection, and shake detection.
 *
 * Digitally-programmable low-pass filters.
 *
 * Low-power pedometer functionality allows the host processor to sleep while the
 * DMP maintains the step count.
 */
void MPU6050::Device::Load_firmware(const std::string file_path)
{
	if ( !Mpu_pwr_on() )
		Wake_up();

	std::ifstream firmware_file ( file_path, std::ifstream::in | std::ifstream::binary );

	if ( firmware_file.fail() )	{
		std::string err_msg { "mpu6050: Failed to open firmware file: " };
		err_msg += file_path;
		throw std::runtime_error { err_msg };
	}

	// Stop eating new lines in binary mode!!!
	firmware_file.unsetf(std::ios::skipws);

	// Read file and put it in buffer
	std::vector<uint8_t> buffer( std::istreambuf_iterator<char>(firmware_file), {} );

	/* Check bank boundaries. */
	if ( buffer.size() > Dev_mem_size ){
		std::string err_msg { "mpu6050: You are trying to load firmware that is too big. Max size must be <= " };
		err_msg += Dev_mem_size + ".";
		throw std::runtime_error { err_msg };
	}

	constexpr unsigned int load_chunk = 16;

	// Ensure that firmware loaded once since mpu6050 powered up.
	std::vector<uint8_t> mem_contents;
	for ( uint16_t i = 0; i < Dev_mem_size; i += load_chunk ){
		std::vector<uint8_t> read_chunk ( Read_mem(i, load_chunk ) );
		mem_contents.insert(mem_contents.end(), read_chunk.begin(), read_chunk.end() );
	}

	if ( !std::equal( buffer.begin(), buffer.end(), mem_contents.begin() ) ) {
		int this_write {0};
		uint firm_length{ buffer.size() };

		for (uint i = 0; i < firm_length; i += this_write) {
			this_write = std::min(load_chunk, firm_length - i);

			std::vector<uint8_t> write_buff {&buffer[i], &buffer[i+this_write]};

			Write_mem(i, this_write, write_buff);

			mem_contents = Read_mem(i, this_write);

			if ( !std::equal( mem_contents.begin(), mem_contents.end(), write_buff.begin() ) )
				throw std::runtime_error{ "mpu6050: Data validation error while uploading firmware." };
		}

		I2C_buffer buff;
		buff.reg = Register::dmp_cfg_1;
		buff.data.push_back(Mem_start_address >> 8);
		buff.data.push_back(Mem_start_address & 0xFF );

		I2C_driver::Write( buff );
	}
}

// This function puts device into standby mode.
void Device::Standby()
{
	Write_bit( Register::pwr_mgmt_1, Pwr_mgmt_1::standby_bit, true );
}

// This function put device into normal mode.
void Device::Wake_up()
{
	Write_bit( Register::pwr_mgmt_1, Pwr_mgmt_1::standby_bit, false );
	Write_bit( Register::pwr_mgmt_1, Pwr_mgmt_1::clk_select_bit, true);

	using namespace std::chrono_literals;
	std::this_thread::sleep_for(100ms);
}

// This function reset device and put it into standby mode.
void Device::Reset()
{
	Write_bit( Register::pwr_mgmt_1, Pwr_mgmt_1::reset_bit, true);

	using namespace std::chrono_literals;
	std::this_thread::sleep_for(100ms);
}

Gyro_accel_data Device::Get_gyro_raw_data()
{
	if ( !Mpu_pwr_on() )
		Wake_up();

	Gyro_accel_data recv_data;
	recv_data.X = ( Read(Register::gyro_xout_h, 1).at(0) << 8 ) |
					Read(Register::gyro_xout_l, 1).at(0);
	recv_data.Y = ( Read(Register::gyro_yout_h, 1).at(0) << 8 ) |
					Read(Register::gyro_yout_l, 1).at(0);
	recv_data.Z = ( Read(Register::gyro_zout_h, 1).at(0) << 8 ) |
					Read(Register::gyro_zout_l, 1).at(0);

	return recv_data;
}
Gyro_accel_data Device::Get_accel_raw_data()
{
	if ( !Mpu_pwr_on() )
		Wake_up();

	Gyro_accel_data recv_data;
	recv_data.X = ( Read(Register::accel_xout_h, 1).at(0) << 8 ) |
					Read(Register::accel_xout_l, 1).at(0);
	recv_data.Y = ( Read(Register::accel_yout_h, 1).at(0) << 8 ) |
					Read(Register::accel_yout_l, 1).at(0);
	recv_data.Z = ( Read(Register::accel_zout_h, 1).at(0) << 8 ) |
					Read(Register::accel_zout_l, 1).at(0);

	return recv_data;
}

void Device::Set_dlpf(Dlpf bw)
{
	if ( !Mpu_pwr_on() )
		Wake_up();

	I2C_buffer buff;

	buff.reg = Register::config;
	buff.data.push_back( bw );

	Write( buff );
}

void Device::setTempFIFOEnabled(bool enable)
{
}

void Device::setXGyroFIFOEnabled(bool enable)
{
}

void Device::setYGyroFIFOEnabled(bool enable)
{
}

void Device::setZGyroFIFOEnabled(bool enable)
{
}

void Device::setAccelFIFOEnabled(bool enable)
{
}

void Device::Set_gyro_rate(uint16_t rate)
{
	if ( !Mpu_pwr_on() )
		Wake_up();

	if ( m_dmp_on )
		return;

	if (rate < 4)
		rate = 4;

	if (rate > 1000)
		rate = 1000;

	I2C_buffer buff;

	buff.reg = Register::smplrt_div;
	buff.data.push_back( 1000 / rate - 1 );

	Write( buff );

}

void Device::Set_gyro_fsr(Guro_fsr range)
{
	if ( !Mpu_pwr_on() )
		Wake_up();

	I2C_buffer buff;

	buff.reg = Register::gyro_config;
	buff.data.push_back( range << 3 );

	Write( buff );
}

void Device::Set_accel_fsr(Accel_fsr range)
{
	if ( !Mpu_pwr_on() )
		Wake_up();

	I2C_buffer buff;

	buff.reg = Register::accel_config;
	buff.data.push_back( range << 3 );

	Write( buff );
}

void Device::Enable_fifo(bool enable)
{
	if ( !Mpu_pwr_on() )
		Wake_up();

	Write_bit( Register::user_ctrl, User_control::fifo_enable, enable);
}

void Device::Reset_fifo( )
{
	if ( !Mpu_pwr_on() )
		Wake_up();

	Write_bit( Register::user_ctrl, User_control::fifo_reset, true );
}

bool Device::Read_bit(Register reg, uint8_t bit_number)
{
	uint8_t recv_reg { Read( reg, 1 ).at(0) };
	return ( recv_reg & ( 1 << bit_number ) );
}

void Device::Write_bit(Register reg, uint8_t bit_number,
		bool state)
{
	uint8_t recv_reg { Read( reg, 1 ).at(0) };
	state ? recv_reg |= ( 1 << bit_number ) : recv_reg &= ~( 1 << bit_number );

	I2C_buffer buff;

	buff.reg = reg;
	buff.data.push_back( recv_reg );

	Write( buff );
}

std::vector<uint8_t> MPU6050::Device::Read_mem(uint16_t mem_address, uint16_t length)
{

	I2C_buffer sel_bank;

    sel_bank.reg = Register::bank_sel;
    sel_bank.data.push_back( mem_address >> 8 );
    sel_bank.data.push_back( mem_address & 0xff );

    I2C_driver::Write(sel_bank);

    return I2C_driver::Read( Register::mem_r_w, length);
}

void MPU6050::Device::Write_mem(uint16_t mem_address, uint16_t length,
		std::vector<uint8_t> data)
{
	I2C_buffer sel_bank;

	sel_bank.reg = Register::bank_sel;
	sel_bank.data.push_back( mem_address >> 8 );
	sel_bank.data.push_back( mem_address & 0xff );

	I2C_driver::Write(sel_bank);

	I2C_buffer buff;
	buff.reg = Register::mem_r_w;
	buff.data = data;
	I2C_driver::Write(buff);
}

} //End of namespace MPU6050.


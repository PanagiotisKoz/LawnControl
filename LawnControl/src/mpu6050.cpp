/*
 *	MPU6050.cpp
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
#include <chrono>
#include <thread>
#include <fstream>

constexpr uint16_t Dev_mem_size = 3*1024; // Maximum device memory size 3 KB.
constexpr unsigned short Mem_start_address = 0x0400; // Set DMP program counter to this address.
constexpr unsigned short DMP_sample_rate = 200; // Sampling rate used when DMP is enabled.

// The following group declares all MPU6050 internal registers.
constexpr uint8_t reg_xg_offs_tc			= 0x00; // [7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
constexpr uint8_t reg_yg_offs_tc      		= 0x01; // [7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
constexpr uint8_t reg_zg_offs_tc      		= 0x02; // [7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
constexpr uint8_t reg_x_fine_gain     		= 0x03; // [7:0] X_FINE_GAIN
constexpr uint8_t reg_y_fine_gain     		= 0x04; // [7:0] Y_FINE_GAIN
constexpr uint8_t reg_z_fine_gain     		= 0x05; // [7:0] Z_FINE_GAIN
constexpr uint8_t reg_xa_offs_h       		= 0x06; // Accel X-axis offset cancellation register high byte
constexpr uint8_t reg_xa_offs_l_tc    		= 0x07; // Accel X-axis offset cancellation register low byte
constexpr uint8_t reg_ya_offs_h      		= 0x08; // Accel Y-axis offset cancellation register high byte
constexpr uint8_t reg_ya_offs_l_tc    		= 0x09; // Accel Y-axis offset cancellation register low byte
constexpr uint8_t reg_za_offs_h       		= 0x0A; // Accel Z-axis offset cancellation register high byte
constexpr uint8_t reg_za_offs_l_tc    		= 0x0B; // Accel Z-axis offset cancellation register low byte
constexpr uint8_t reg_self_test_x     		= 0x0D; // [7:5] XA_TEST[4-2], [4:0] XG_TEST[4-0]
constexpr uint8_t reg_self_test_y     		= 0x0E; // [7:5] YA_TEST[4-2], [4:0] YG_TEST[4-0]
constexpr uint8_t reg_self_test_z     		= 0x0F; // [7:5] ZA_TEST[4-2], [4:0] ZG_TEST[4-0]
constexpr uint8_t reg_self_test_a     		= 0x10; // [5:4] XA_TEST[1-0], [3:2] YA_TEST[1-0], [1:0] ZA_TEST[1-0]
constexpr uint8_t reg_xg_offs_usrh    		= 0x13; // Gyro X-axis offset cancellation register high byte
constexpr uint8_t reg_xg_offs_usrl    		= 0x14; // Gyro X-axis offset cancellation register low byte
constexpr uint8_t reg_yg_offs_usrh    		= 0x15; // Gyro Y-axis offset cancellation register high byte
constexpr uint8_t reg_yg_offs_usrl    		= 0x16; // Gyro Y-axis offset cancellation register low byte
constexpr uint8_t reg_zg_offs_usrh    		= 0x17; // Gyro Z-axis offset cancellation register high byte
constexpr uint8_t reg_zg_offs_usrl    		= 0x18; // Gyro Z-axis offset cancellation register low byte
constexpr uint8_t reg_smplrt_div      		= 0x19; // Sample Rate Divider
constexpr uint8_t reg_config          		= 0x1A; // Configuration
constexpr uint8_t reg_gyro_config     		= 0x1B; // Gyroscope Configuration
constexpr uint8_t reg_accel_config    		= 0x1C; // Accelerometer Configuration
constexpr uint8_t reg_ff_thr          		= 0x1D;
constexpr uint8_t reg_ff_dur          		= 0x1E;
constexpr uint8_t reg_mot_thr         		= 0x1F;
constexpr uint8_t reg_mot_dur         		= 0x20;
constexpr uint8_t reg_zrmot_thr       		= 0x21;
constexpr uint8_t reg_zrmot_dur       		= 0x22;
constexpr uint8_t reg_fifo_en         		= 0x23; // FIFO Enable
constexpr uint8_t reg_i2c_mst_ctrl    		= 0x24;
constexpr uint8_t reg_i2c_slv0_addr   		= 0x25;
constexpr uint8_t reg_i2c_slv0_reg    		= 0x26;
constexpr uint8_t reg_i2c_slv0_ctrl   		= 0x27;
constexpr uint8_t reg_i2c_slv1_addr   		= 0x28;
constexpr uint8_t reg_i2c_slv1_reg    		= 0x29;
constexpr uint8_t reg_i2c_slv1_ctrl   		= 0x2A;
constexpr uint8_t reg_i2c_slv2_addr   		= 0x2B;
constexpr uint8_t reg_i2c_slv2_reg    		= 0x2C;
constexpr uint8_t reg_i2c_slv2_ctrl   		= 0x2D;
constexpr uint8_t reg_i2c_slv3_addr   		= 0x2E;
constexpr uint8_t reg_i2c_slv3_reg    		= 0x2F;
constexpr uint8_t reg_i2c_slv3_ctrl   		= 0x30;
constexpr uint8_t reg_i2c_slv4_addr   		= 0x31;
constexpr uint8_t reg_i2c_slv4_reg    		= 0x32;
constexpr uint8_t reg_i2c_slv4_do     		= 0x33;
constexpr uint8_t reg_i2c_slv4_ctrl   		= 0x34;
constexpr uint8_t reg_i2c_slv4_di     		= 0x35;
constexpr uint8_t reg_i2c_mst_status  		= 0x36;
constexpr uint8_t reg_int_pin_cfg     		= 0x37;
constexpr uint8_t reg_int_enable      		= 0x38;
constexpr uint8_t reg_dmp_int_status  		= 0x39;
constexpr uint8_t reg_int_status      		= 0x3A;
constexpr uint8_t reg_accel_xout_h    		= 0x3B;
constexpr uint8_t reg_accel_xout_l    		= 0x3C;
constexpr uint8_t reg_accel_yout_h    		= 0x3D;
constexpr uint8_t reg_accel_yout_l    		= 0x3E;
constexpr uint8_t reg_accel_zout_h    		= 0x3F;
constexpr uint8_t reg_accel_zout_l    		= 0x40;
constexpr uint8_t reg_temp_out_h      		= 0x41;
constexpr uint8_t reg_temp_out_l      		= 0x42;
constexpr uint8_t reg_gyro_xout_h     		= 0x43;
constexpr uint8_t reg_gyro_xout_l     		= 0x44;
constexpr uint8_t reg_gyro_yout_h     		= 0x45;
constexpr uint8_t reg_gyro_yout_l     		= 0x46;
constexpr uint8_t reg_gyro_zout_h     		= 0x47;
constexpr uint8_t reg_gyro_zout_l      		= 0x48;
constexpr uint8_t reg_ext_sens_data_00  	= 0x49;
constexpr uint8_t reg_ext_sens_data_01  	= 0x4A;
constexpr uint8_t reg_ext_sens_data_02  	= 0x4B;
constexpr uint8_t reg_ext_sens_data_03  	= 0x4C;
constexpr uint8_t reg_ext_sens_data_04  	= 0x4D;
constexpr uint8_t reg_ext_sens_data_05  	= 0x4E;
constexpr uint8_t reg_ext_sens_data_06  	= 0x4F;
constexpr uint8_t reg_ext_sens_data_07  	= 0x50;
constexpr uint8_t reg_ext_sens_data_08  	= 0x51;
constexpr uint8_t reg_ext_sens_data_09  	= 0x52;
constexpr uint8_t reg_ext_sens_data_10  	= 0x53;
constexpr uint8_t reg_ext_sens_data_11  	= 0x54;
constexpr uint8_t reg_ext_sens_data_12  	= 0x55;
constexpr uint8_t reg_ext_sens_data_13  	= 0x56;
constexpr uint8_t reg_ext_sens_data_14  	= 0x57;
constexpr uint8_t ext_sens_data_15  		= 0x58;
constexpr uint8_t reg_ext_sens_data_16  	= 0x59;
constexpr uint8_t reg_ext_sens_data_17  	= 0x5A;
constexpr uint8_t reg_ext_sens_data_18  	= 0x5B;
constexpr uint8_t reg_ext_sens_data_19  	= 0x5C;
constexpr uint8_t reg_ext_sens_data_20  	= 0x5D;
constexpr uint8_t reg_ext_sens_data_21  	= 0x5E;
constexpr uint8_t reg_ext_sens_data_22  	= 0x5F;
constexpr uint8_t reg_ext_sens_data_23  	= 0x60;
constexpr uint8_t reg_mot_detect_status 	= 0x61;
constexpr uint8_t reg_i2c_slv0_do      		= 0x63;
constexpr uint8_t reg_i2c_slv1_do      		= 0x64;
constexpr uint8_t reg_i2c_slv2_do      		= 0x65;
constexpr uint8_t reg_i2c_slv3_do      		= 0x66;
constexpr uint8_t reg_i2c_mst_delay_ctrl	= 0x67;
constexpr uint8_t reg_signal_path_reset  	= 0x68;
constexpr uint8_t reg_mot_detect_ctrl    	= 0x69;
constexpr uint8_t reg_user_ctrl        		= 0x6A;
constexpr uint8_t reg_pwr_mgmt_1       		= 0x6B;
constexpr uint8_t reg_pwr_mgmt_2       		= 0x6C;
constexpr uint8_t reg_bank_sel         		= 0x6D;
constexpr uint8_t reg_mem_start_addr   		= 0x6E;
constexpr uint8_t reg_mem_r_w          		= 0x6F;
constexpr uint8_t reg_dmp_cfg_1        		= 0x70;
constexpr uint8_t reg_dmp_cfg_2        		= 0x71;
constexpr uint8_t reg_fifo_counth      		= 0x72;
constexpr uint8_t reg_fifo_countl      		= 0x73;
constexpr uint8_t reg_fifo_r_w         		= 0x74;
constexpr uint8_t reg_who_am_i         		= 0x75;


/*
 *	The following group declares, the values ​​that registers above must have
 *in order to enable or disable device's capabilities.
 *
 *	For extra study please refer to InveSense document
 *"MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2"
 */

// Register 107 – Power Management 1.
constexpr uint8_t Pwr_mgmt_1_clk_select_bit	= 0; // Always use PLL with X axis gyroscope reference.
constexpr uint8_t Pwr_mgmt_1_temp_dis_bit	= 3;
constexpr uint8_t Pwr_mgmt_1_cycle_bit		= 5;
constexpr uint8_t Pwr_mgmt_1_standby_bit	= 6;
constexpr uint8_t Pwr_mgmt_1_reset_bit		= 7;

// Register 35 – FIFO enable.
constexpr uint8_t fifo_enbl_accel_reg_bit 	= 3;
constexpr uint8_t fifo_enbl_gz_reg_bit		= 4;
constexpr uint8_t fifo_enbl_gy_reg_bit		= 5;
constexpr uint8_t fifo_enbl_gx_reg_bit		= 6;
constexpr uint8_t fifo_enbl_temp_reg_bit	= 7;

// Register 106 – User control.
constexpr uint8_t usr_ctrl_reset_sig_paths	= 0;
constexpr uint8_t usr_ctrl_fifo_reset 		= 2;
constexpr uint8_t usr_ctrl_fifo_enable		= 6;

namespace MPU6050 {


Device::Device( const int i2cbus, const uint8_t address )
		: m_dmp_on{ true }, m_pwr_state { false }, m_I2C_driver( i2cbus, address )
{
	Reset();
	Set_gyro_fsr();
	Set_accel_fsr();
	Set_dlpf(Dlpf::bw_42hz);
	Set_gyro_rate();
}

Device::~Device()
{
	Reset();
}

uint8_t Device::Device_id()
{
	return m_I2C_driver.Read( reg_who_am_i, 1).at(0) ;
}


void MPU6050::Device::Disable_temp_sensor(bool disable)
{
	if ( !m_pwr_state )
		Wake_up();

	disable ? m_I2C_driver.Write_bit( reg_pwr_mgmt_1, Pwr_mgmt_1_temp_dis_bit, true ) :
			m_I2C_driver.Write_bit( reg_pwr_mgmt_1, Pwr_mgmt_1_temp_dis_bit, true );
}

// Returns ambient temperature. Please enable temperature sensor first.
// If temperature sensor is disabled returns an odd value.
float Device::Ambient_temp()
{
	if ( !m_pwr_state )
		Wake_up();

	if ( m_I2C_driver.Read_bit( reg_pwr_mgmt_1, Pwr_mgmt_1_temp_dis_bit ) )
		Disable_temp_sensor( false );

	short int raw = ( m_I2C_driver.Read( reg_temp_out_h, 1 ).at(0) << 8) |
						m_I2C_driver.Read( reg_temp_out_l, 1 ).at(0);


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
	if ( !m_pwr_state )
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

		I2C_driver::I2C_buffer buff;
		buff.reg = reg_dmp_cfg_1;
		buff.data.push_back(Mem_start_address >> 8);
		buff.data.push_back(Mem_start_address & 0xFF );

		m_I2C_driver.Write( buff );
	}
}

// This function puts device into standby mode.
void Device::Standby()
{
	m_I2C_driver.Write_bit( reg_pwr_mgmt_1, Pwr_mgmt_1_standby_bit, true );

	m_pwr_state = false;
}

// This function put device into normal mode.
void Device::Wake_up()
{
	m_I2C_driver.Write_bit( reg_pwr_mgmt_1, Pwr_mgmt_1_standby_bit, false );
	m_I2C_driver.Write_bit( reg_pwr_mgmt_1, Pwr_mgmt_1_clk_select_bit, true);

	using namespace std::chrono_literals;
	std::this_thread::sleep_for(100ms);

	m_pwr_state = true;
}

// This function reset device and put it into standby mode.
void Device::Reset()
{
	m_I2C_driver.Write_bit( reg_pwr_mgmt_1, Pwr_mgmt_1_reset_bit, true);

	using namespace std::chrono_literals;
	std::this_thread::sleep_for(100ms);

	m_pwr_state = false;
}

Gyro_accel_data Device::Get_gyro_raw_data()
{
	if ( !m_pwr_state )
		Wake_up();

	Gyro_accel_data recv_data;
	recv_data.X = ( m_I2C_driver.Read( reg_gyro_xout_h, 1).at(0) << 8 ) |
					m_I2C_driver.Read( reg_gyro_xout_l, 1).at(0);
	recv_data.Y = ( m_I2C_driver.Read( reg_gyro_yout_h, 1).at(0) << 8 ) |
					m_I2C_driver.Read( reg_gyro_yout_l, 1).at(0);
	recv_data.Z = ( m_I2C_driver.Read( reg_gyro_zout_h, 1).at(0) << 8 ) |
					m_I2C_driver.Read( reg_gyro_zout_l, 1).at(0);

	return recv_data;
}
Gyro_accel_data Device::Get_accel_raw_data()
{
	if ( !m_pwr_state )
		Wake_up();

	Gyro_accel_data recv_data;
	recv_data.X = ( m_I2C_driver.Read( reg_accel_xout_h, 1).at(0) << 8 ) |
					m_I2C_driver.Read( reg_accel_xout_l, 1).at(0);
	recv_data.Y = ( m_I2C_driver.Read( reg_accel_yout_h, 1).at(0) << 8 ) |
					m_I2C_driver.Read( reg_accel_yout_l, 1).at(0);
	recv_data.Z = ( m_I2C_driver.Read( reg_accel_zout_h, 1).at(0) << 8 ) |
					m_I2C_driver.Read( reg_accel_zout_l, 1).at(0);

	return recv_data;
}

void Device::Set_dlpf(Dlpf bw)
{
	if ( !m_pwr_state )
		Wake_up();

	I2C_driver::I2C_buffer buff;

	buff.reg = reg_config;
	buff.data.push_back( static_cast<uint8_t>( bw ) );

	m_I2C_driver.Write( buff );
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
	if ( !m_pwr_state )
		Wake_up();

	if ( m_dmp_on )
		return;

	if (rate < 4)
		rate = 4;

	if (rate > 1000)
		rate = 1000;

	I2C_driver::I2C_buffer buff;

	buff.reg = reg_smplrt_div;
	buff.data.push_back( 1000 / rate - 1 );

	m_I2C_driver.Write( buff );

}

void Device::Set_gyro_fsr(Guro_fsr range)
{
	if ( !m_pwr_state )
		Wake_up();

	I2C_driver::I2C_buffer buff;

	buff.reg = reg_gyro_config;
	buff.data.push_back( static_cast<uint8_t>( range ) << 3 );

	m_I2C_driver.Write( buff );
}

void Device::Set_accel_fsr(Accel_fsr range)
{
	if ( !m_pwr_state )
		Wake_up();

	I2C_driver::I2C_buffer buff;

	buff.reg = reg_accel_config;
	buff.data.push_back( static_cast<uint8_t>( range ) << 3 );

	m_I2C_driver.Write( buff );
}

void Device::Enable_fifo(bool enable)
{
	if ( !m_pwr_state )
		Wake_up();

	m_I2C_driver.Write_bit( reg_user_ctrl, usr_ctrl_fifo_enable, enable);
}

void Device::Reset_fifo( )
{
	if ( !m_pwr_state )
		Wake_up();

	m_I2C_driver.Write_bit( reg_user_ctrl, usr_ctrl_fifo_reset, true );
}

std::vector<uint8_t> MPU6050::Device::Read_mem(uint16_t mem_address, uint16_t length)
{

	I2C_driver::I2C_buffer sel_bank;

    sel_bank.reg = reg_bank_sel;
    sel_bank.data.push_back( mem_address >> 8 );
    sel_bank.data.push_back( mem_address & 0xff );

    m_I2C_driver.Write(sel_bank);

    return m_I2C_driver.Read( reg_mem_r_w, length);
}

void MPU6050::Device::Write_mem(uint16_t mem_address, uint16_t length,
		std::vector<uint8_t> data)
{
	I2C_driver::I2C_buffer sel_bank;

	sel_bank.reg = reg_bank_sel;
	sel_bank.data.push_back( mem_address >> 8 );
	sel_bank.data.push_back( mem_address & 0xff );

	m_I2C_driver.Write(sel_bank);

	I2C_driver::I2C_buffer buff;
	buff.reg = reg_mem_r_w;
	buff.data = data;
	m_I2C_driver.Write(buff);
}

} //End of namespace MPU6050.


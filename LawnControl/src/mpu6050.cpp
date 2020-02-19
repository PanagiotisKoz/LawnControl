#include "includes/mpu6050.h"
#include <chrono>
#include <string>
#include <iostream>
#include <fstream>
#include <pigpiod_if2.h>

constexpr uint16_t dev_mem_size = 3 * 1024; // Maximum device memory size 3 KB.
constexpr uint16_t fifo_max_size = 1024; // Maximum device memory size 3 KB.
constexpr unsigned short mem_start_address = 0x0400; // Set DMP program counter to this address.

/*
 *  The following group declares all MPU6050 internal registers.
 *  Please see Invensenses "MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2"
 *  and "Application Note Programming Sequence for DMP Hardware Functions".
 */
constexpr uint8_t reg_xg_offs_tc 		= 0x00;
constexpr uint8_t reg_yg_offs_tc 		= 0x01;
constexpr uint8_t reg_zg_offs_tc 		= 0x02;
constexpr uint8_t reg_x_fine_gain 		= 0x03;
constexpr uint8_t reg_y_fine_gain 		= 0x04;
constexpr uint8_t reg_z_fine_gain 		= 0x05;
constexpr uint8_t reg_xa_offs_h 		= 0x06;
constexpr uint8_t reg_xa_offs_l_tc 		= 0x07;
constexpr uint8_t reg_ya_offs_h 		= 0x08;
constexpr uint8_t reg_ya_offs_l_tc 		= 0x09;
constexpr uint8_t reg_za_offs_h 		= 0x0A;
constexpr uint8_t reg_za_offs_l_tc 		= 0x0B;
constexpr uint8_t reg_self_test_x 		= 0x0D;
constexpr uint8_t reg_self_test_y 		= 0x0E;
constexpr uint8_t reg_self_test_z 		= 0x0F;
constexpr uint8_t reg_self_test_a 		= 0x10;
constexpr uint8_t reg_xg_offs_usrh 		= 0x13;
constexpr uint8_t reg_xg_offs_usrl 		= 0x14;
constexpr uint8_t reg_yg_offs_usrh 		= 0x15;
constexpr uint8_t reg_yg_offs_usrl 		= 0x16;
constexpr uint8_t reg_zg_offs_usrh 		= 0x17;
constexpr uint8_t reg_zg_offs_usrl 		= 0x18;
constexpr uint8_t reg_smplrt_div 		= 0x19;
constexpr uint8_t reg_config 			= 0x1A;
constexpr uint8_t reg_gyro_config 		= 0x1B;
constexpr uint8_t reg_accel_config 		= 0x1C;
constexpr uint8_t reg_ff_thr 			= 0x1D;
constexpr uint8_t reg_ff_dur 			= 0x1E;
constexpr uint8_t reg_mot_thr 			= 0x1F;
constexpr uint8_t reg_mot_dur 			= 0x20;
constexpr uint8_t reg_zrmot_thr 		= 0x21;
constexpr uint8_t reg_zrmot_dur 		= 0x22;
constexpr uint8_t reg_fifo_enable 		= 0x23;
constexpr uint8_t reg_i2c_mst_ctrl 		= 0x24;
constexpr uint8_t reg_i2c_slv0_addr 	= 0x25;
constexpr uint8_t reg_i2c_slv0_reg 		= 0x26;
constexpr uint8_t reg_i2c_slv0_ctrl 	= 0x27;
constexpr uint8_t reg_i2c_slv1_addr 	= 0x28;
constexpr uint8_t reg_i2c_slv1_reg 		= 0x29;
constexpr uint8_t reg_i2c_slv1_ctrl 	= 0x2A;
constexpr uint8_t reg_i2c_slv2_addr 	= 0x2B;
constexpr uint8_t reg_i2c_slv2_reg 		= 0x2C;
constexpr uint8_t reg_i2c_slv2_ctrl 	= 0x2D;
constexpr uint8_t reg_i2c_slv3_addr 	= 0x2E;
constexpr uint8_t reg_i2c_slv3_reg 		= 0x2F;
constexpr uint8_t reg_i2c_slv3_ctrl 	= 0x30;
constexpr uint8_t reg_i2c_slv4_addr 	= 0x31;
constexpr uint8_t reg_i2c_slv4_reg 		= 0x32;
constexpr uint8_t reg_i2c_slv4_do 		= 0x33;
constexpr uint8_t reg_i2c_slv4_ctrl 	= 0x34;
constexpr uint8_t reg_i2c_slv4_di 		= 0x35;
constexpr uint8_t reg_i2c_mst_status 	= 0x36;
constexpr uint8_t reg_int_pin_cfg 		= 0x37;
constexpr uint8_t reg_int_enable 		= 0x38;
constexpr uint8_t reg_dmp_int_status 	= 0x39;
constexpr uint8_t reg_int_status 		= 0x3A;
constexpr uint8_t reg_accel_xout_h 		= 0x3B;
constexpr uint8_t reg_accel_xout_l 		= 0x3C;
constexpr uint8_t reg_accel_yout_h 		= 0x3D;
constexpr uint8_t reg_accel_yout_l 		= 0x3E;
constexpr uint8_t reg_accel_zout_h 		= 0x3F;
constexpr uint8_t reg_accel_zout_l 		= 0x40;
constexpr uint8_t reg_temp_out_h 		= 0x41;
constexpr uint8_t reg_temp_out_l 		= 0x42;
constexpr uint8_t reg_gyro_xout_h 		= 0x43;
constexpr uint8_t reg_gyro_xout_l 		= 0x44;
constexpr uint8_t reg_gyro_yout_h 		= 0x45;
constexpr uint8_t reg_gyro_yout_l 		= 0x46;
constexpr uint8_t reg_gyro_zout_h 		= 0x47;
constexpr uint8_t reg_gyro_zout_l 		= 0x48;
constexpr uint8_t reg_ext_sens_data_00 	= 0x49;
constexpr uint8_t reg_ext_sens_data_01 	= 0x4A;
constexpr uint8_t reg_ext_sens_data_02 	= 0x4B;
constexpr uint8_t reg_ext_sens_data_03 	= 0x4C;
constexpr uint8_t reg_ext_sens_data_04 	= 0x4D;
constexpr uint8_t reg_ext_sens_data_05 	= 0x4E;
constexpr uint8_t reg_ext_sens_data_06 	= 0x4F;
constexpr uint8_t reg_ext_sens_data_07 	= 0x50;
constexpr uint8_t reg_ext_sens_data_08 	= 0x51;
constexpr uint8_t reg_ext_sens_data_09 	= 0x52;
constexpr uint8_t reg_ext_sens_data_10 	= 0x53;
constexpr uint8_t reg_ext_sens_data_11 	= 0x54;
constexpr uint8_t reg_ext_sens_data_12 	= 0x55;
constexpr uint8_t reg_ext_sens_data_13 	= 0x56;
constexpr uint8_t reg_ext_sens_data_14 	= 0x57;
constexpr uint8_t ext_sens_data_15 		= 0x58;
constexpr uint8_t reg_ext_sens_data_16 	= 0x59;
constexpr uint8_t reg_ext_sens_data_17 	= 0x5A;
constexpr uint8_t reg_ext_sens_data_18 	= 0x5B;
constexpr uint8_t reg_ext_sens_data_19 	= 0x5C;
constexpr uint8_t reg_ext_sens_data_20 	= 0x5D;
constexpr uint8_t reg_ext_sens_data_21 	= 0x5E;
constexpr uint8_t reg_ext_sens_data_22 	= 0x5F;
constexpr uint8_t reg_ext_sens_data_23 	= 0x60;
constexpr uint8_t reg_mot_detect_status = 0x61;
constexpr uint8_t reg_i2c_slv0_do 		= 0x63;
constexpr uint8_t reg_i2c_slv1_do 		= 0x64;
constexpr uint8_t reg_i2c_slv2_do 		= 0x65;
constexpr uint8_t reg_i2c_slv3_do 		= 0x66;
constexpr uint8_t reg_i2c_mst_delay_ctrl = 0x67;
constexpr uint8_t reg_signal_path_reset = 0x68;
constexpr uint8_t reg_mot_detect_ctrl 	= 0x69;
constexpr uint8_t reg_user_ctrl 		= 0x6A;
constexpr uint8_t reg_pwr_mgmt_1 		= 0x6B;
constexpr uint8_t reg_pwr_mgmt_2 		= 0x6C;
constexpr uint8_t reg_bank_sel 			= 0x6D;
constexpr uint8_t reg_mem_start_addr 	= 0x6E;
constexpr uint8_t reg_mem_r_w 			= 0x6F;
constexpr uint8_t reg_dmp_fw_start_H	= 0x70;
constexpr uint8_t reg_dmp_fw_start_L	= 0x71;
constexpr uint8_t reg_fifo_counth 		= 0x72;
constexpr uint8_t reg_fifo_countl 		= 0x73;
constexpr uint8_t reg_fifo_r_w 			= 0x74;
constexpr uint8_t reg_who_am_i 			= 0x75;

// Registers that exist when firmware 6.12 loaded.
constexpr uint16_t reg_gyro_mount_matrix_config_1  		= 0x426;
constexpr uint16_t reg_gyro_mount_matrix_config_2  		= 0x427;
constexpr uint16_t reg_gyro_mount_matrix_config_3  		= 0x428;
constexpr uint16_t reg_gyro_mount_matrix_config_sign_1 	= 0x456;
constexpr uint16_t reg_gyro_mount_matrix_config_sign_2 	= 0x457;
constexpr uint16_t reg_gyro_mount_matrix_config_sign_3 	= 0x458;
constexpr uint16_t reg_accel_mount_matrix_config_1 		= 0x42A;
constexpr uint16_t reg_accel_mount_matrix_config_2 		= 0x42B;
constexpr uint16_t reg_accel_mount_matrix_config_3 		= 0x42C;
constexpr uint16_t reg_accel_mount_matrix_config_sign_1 = 0x434;
constexpr uint16_t reg_accel_mount_matrix_config_sign_2 = 0x435;
constexpr uint16_t reg_accel_mount_matrix_config_sign_3 = 0x436;
constexpr uint16_t reg_3axis_lpq_en						= 0xA9D;
constexpr uint16_t reg_6axis_lpq_en						= 0xAA3;
constexpr uint16_t reg_raw_data_en						= 0xAAB;
constexpr uint16_t reg_fifo_rate_div					= 0x216;
constexpr uint16_t reg_enable_fifo_rate_div				= 0xAC4;



/*
 * The following group declares, the values ​​that registers above must have
 * in order to enable or disable device's capabilities.
 *
 * For extra study please refer to InveSense document
 * "MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2"
 */

// Register 107 – Power Management 1.
constexpr uint8_t pwr_mgmt_1_clk_select_bit = 0; // Always use PLL with X axis gyroscope reference.
constexpr uint8_t pwr_mgmt_1_temp_dis_bit = 3;
constexpr uint8_t pwr_mgmt_1_dmp_reset_bit = 4;
constexpr uint8_t pwr_mgmt_1_cycle_bit = 5;
constexpr uint8_t pwr_mgmt_1_standby_bit = 6;
constexpr uint8_t pwr_mgmt_1_reset_bit = 7;

// Register 58 – Interrupt status.
constexpr uint8_t data_rdy_int_bit = 0; // Always use PLL with X axis gyroscope reference.
constexpr uint8_t dmp_int_bit = 1;
constexpr uint8_t fifo_oflow_int_bit = 4;
constexpr uint8_t zmot_int_bit = 5;
constexpr uint8_t mot_int_bit = 6;
constexpr uint8_t ff_int_bit = 7;

// Register 37 - INT Pin / Bypass Enable Configuration
constexpr uint8_t setup_interrupts = 0x1E;

// Register 35 – FIFO enable.
constexpr uint8_t fifo_enbl_accel_reg_bit = 3;
constexpr uint8_t fifo_enbl_gz_reg_bit = 4;
constexpr uint8_t fifo_enbl_gy_reg_bit = 5;
constexpr uint8_t fifo_enbl_gx_reg_bit = 6;
constexpr uint8_t fifo_enbl_temp_reg_bit = 7;

// Register 106 – User control.
constexpr uint8_t usr_ctrl_reset_sig_paths_bit = 0;
constexpr uint8_t usr_ctrl_fifo_reset_bit = 2;
constexpr uint8_t usr_ctrl_fifo_dmp_reset_bit = 3;
constexpr uint8_t usr_ctrl_fifo_enable_bit = 6;
constexpr uint8_t usr_ctrl_fifo_dmp_enable_bit = 7;

// DMP constants to configure orientation.
// Gyro orientation axes dmp constants.
constexpr uint8_t dmp_constant_gyro_x_axis = 0x4C;
constexpr uint8_t dmp_constant_gyro_y_axis = 0xCD;
constexpr uint8_t dmp_constant_gyro_z_axis = 0x6C;
// Gyro axes sign dmp constants.
constexpr uint8_t dmp_constant_gyro_sign_x_axis = 0x36;
constexpr uint8_t dmp_constant_gyro_sign_y_axis = 0x56;
constexpr uint8_t dmp_constant_gyro_sign_z_axis = 0x76;

// Accel orientation axes dmp constants.
constexpr uint8_t dmp_constant_accel_x_axis = 0x0C;
constexpr uint8_t dmp_constant_accel_y_axis = 0xC9;
constexpr uint8_t dmp_constant_accel_z_axis = 0x2C;
// Accel axes sign dmp constants.
constexpr uint8_t dmp_constant_accel_sign_x_axis = 0x26;
constexpr uint8_t dmp_constant_accel_sign_y_axis = 0x46;
constexpr uint8_t dmp_constant_accel_sign_z_axis = 0x66;

// DMP constants to enable or control features.
// 3-Axis Low Power Quaternion enable value.
constexpr uint8_t val_3a_lpq_en[] = { 0xC0, 0xC2, 0xC4, 0xC6 };
// 6-Axis Low Power Quaternion enable value.
constexpr uint8_t val_6a_lpq_en[] = { 0x20, 0x28, 0x30, 0x30 };
// Raw data enable value.
constexpr uint8_t val_raw_data_en[] = { 0xA3, 0xC0, 0xC8, 0xC2, 0xC4, 0xCC, 0xC6, 0xA3, 0xA3, 0xA3 };
// FIFO Rate Divider Enable value.
constexpr uint8_t val_enable_fifo_rate_div[] = { 0xFE, 0xF2, 0xAB, 0xC4, 0xAA, 0xF1, 0xDF, 0xDF, 0xBB, 0xAF, 0xDF, 0xDF };
// FIFO Max rate.
constexpr uint8_t mpu_FIFO_max_rate = 150;

constexpr char tag[] = "MPU6050 -- ";
constexpr auto msg_no_wake = "Attempted to access the chip while it is not enabled."
							 " Please call \"wake_up\" function first.";
constexpr auto msg_no_temp_enabled = "Attempted to access temperature sensor without first enabled.";
constexpr auto msg_firmware_already_loaded = "Firmware already loaded. Attempted reupload.";
constexpr auto msg_firmware_not_loaded = "Attempted to access dmp without load firmware.";
constexpr auto msg_rate_change_not_allowed = "Attempted to change gyro rate while dmp is on.";
constexpr auto msg_dmp_off = "Attempted to change dmp feature while dmp is off.";

/**
 * @brief Class constructor.
 * @param i2cbus I2C bus number.
 * @param address Address of MPU6050.
 * @exception std::runtime_error with error description.
 */
MPU6050::MPU6050( const int i2cbus, const uint8_t address, unsigned interrupt_pin )
	try: m_dmp_on{ false }, m_firmware_loaded{ false},
		m_pwr_on{ false }, m_temp_sens_on{ true }, m_interrupt_on{ false },
		m_callback_id{ 0 },	m_interrupt_pin{ interrupt_pin },
		m_I2C_driver( i2cbus, address ),
		m_fifo_flags( Fifo_en_flags::accel ),
		m_mpu_packet{ 0, 0, 0, 0, 0, 0, 0 },
		m_mpu_packet_size{ 0 },
		m_exit_thread{ false }, m_more_data{ false }
{
	// Check if device is connected.
	if ( device_id() != address ) {
		throw std::runtime_error( "Cannot connect to mpu6xx0 device. Please check address." );
	}

	m_pi = pigpio_start( NULL, NULL );
	if ( m_pi < 0 ) {
		std::string msg( tag );
		msg += "Failed to connect to pigpiod. Please check if daemon is running.";
		throw std::runtime_error( msg );
	}

	set_mode( m_pi, m_interrupt_pin, PI_INPUT );

	init_mpu();

	m_callback_id = callback_ex( m_pi, m_interrupt_pin, EITHER_EDGE, on_interrupt, this );
	m_read_fifo_thread = std::thread{ &MPU6050::fifo_thread, this };

}catch ( std::exception& e ) {
	std::string msg( tag );
	msg += e.what();
	throw std::runtime_error( msg );
}

/**
 * @brief Register a function to call when mpu trigers an interrupt.
 *
 * @param listener function example: void on_interrupt();
 */
void MPU6050::register_on_interrupt( std::function< void( Fifo_mpu_packet ) > listener )
{
	m_listener = listener;
}

void MPU6050::set_clock_source( Clock_source clk_src )
{
	// Set mpu clock source.
	uint8_t reg_val =  read_reg( reg_pwr_mgmt_1 );
	reg_val &= ( 0xFF & static_cast< uint8_t >( clk_src ) );
	write_reg( reg_pwr_mgmt_1, reg_val );
}

/**
 * @brief Initializes chip for use.
 * This function also sets the chip ready to load dmp firmware.\n
 * The power state does not changed.
 */
void MPU6050::init_mpu()
{
	bool was_on = m_pwr_on;

	reset();

	if ( !m_pwr_on )
		wake_up();

	// Set I2c to bypass mode and enable interrupts to pin.
	write_reg( reg_int_pin_cfg, setup_interrupts );
	set_clock_source( Clock_source::x_axis_gyro_ref );
	set_gyro_fsr( Gyro_fsr::gfs_2000dps );
	set_accel_fsr( Accel_fsr::afs_2g );
	set_gyro_rate( mpu_FIFO_max_rate );

	if ( !was_on )
		standby();
}

/**
 * @brief Class distructor.
 * Clean up memory and put mpu6050 in sleep mode.
 */
MPU6050::~MPU6050()
{
	reset();

	callback_cancel( m_callback_id );

	if ( m_read_fifo_thread.joinable() ) {
		m_exit_thread = true;
		m_data_ready.notify_one();
		m_read_fifo_thread.join();
	}

	pigpio_stop( m_pi );
}

/**
 * @brief Returns device id.
 */
uint8_t MPU6050::device_id()
{
	return m_I2C_driver.read( reg_who_am_i, 1 ).at( 0 );
}

/**
 * @brief Disables embended temperature sensor.
 * @note By default sensor is On.
 * @param disable true set sensor Off, false set sensor On.
 */
void MPU6050::set_temp_sensor( bool on )
{
	if ( !m_pwr_on )
		throw std::logic_error( std::string( tag ) + std::string( msg_no_wake ) );

	if ( on == m_temp_sens_on )
		return;

	if ( m_interrupt_on ) {
		on ?
			m_fifo_flags.set( Fifo_en_flags::temp_sensor ) :
			m_fifo_flags.unset( Fifo_en_flags::temp_sensor );
		enable_interrupt( false );
		load_in_fifo( m_fifo_flags );
		reset_fifo();
		enable_interrupt( true );
	}

	on ? m_I2C_driver.write_bit( reg_pwr_mgmt_1, pwr_mgmt_1_temp_dis_bit, true ) :
		 m_I2C_driver.write_bit( reg_pwr_mgmt_1, pwr_mgmt_1_temp_dis_bit, false );

	m_temp_sens_on = on;
}

/**
 * @brief Returns ambient temperature.
 * @warning To use this function enable temperature sensor first if disabled.\n
 * If temperature sensor is disabled returns an odd value.
 *
 * @return Ambient temperature. Zero if temp sensor is disabled.
 */
float MPU6050::get_ambient_temp()
{
	if ( !m_pwr_on )
		throw std::logic_error( std::string( tag ) + std::string( msg_no_wake ) );

	if( !m_temp_sens_on )
		throw std::logic_error( std::string( tag ) + std::string( msg_no_temp_enabled ) );

	uint16_t raw{ 0 };
	if ( !m_interrupt_on )
		raw = ( read_reg( reg_temp_out_h ) << 8 ) | read_reg( reg_temp_out_l );
	else {
		std::lock_guard< std::mutex > lk( m_mutex );
		raw = m_mpu_packet.temp;
	}

	/*
	 * The following calculation, resulting from manual
	 * "MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2"
	 */
	float temp = raw / 340.0f + 35.0f;
	return temp;
}

/**
 * @brief Upload motion firmware to mpu.
 * @warning This function on error throws a std::runtime_error.\n
 * This is not a fatal situation, because library may work without
 * firmware loaded.\n You may recover from this error by reset device.
 * @param file_path full file path that is contains the firmware.
 * @exception std::runtime_error with error description.
 *
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
 * Low-power pedometer functionality allows the host processor to sleep while the
 * DMP maintains the step count.
 */
void MPU6050::load_firmware( const std::string file_path )
{
	if ( !m_pwr_on )
		throw std::logic_error( std::string( tag ) + std::string( msg_no_wake ) );
	if ( m_firmware_loaded )
		throw std::logic_error( std::string( tag ) + std::string( msg_firmware_already_loaded ) );

	init_mpu();

	std::ifstream firmware_file( file_path, std::ifstream::in | std::ifstream::binary );

	if ( firmware_file.fail() ) {
		std::string err_msg( tag );
		err_msg += "Failed to open firmware file: " + file_path;
		throw std::runtime_error { err_msg };
	}

	// Stop eating new lines in binary mode!!!
	firmware_file.unsetf( std::ios::skipws );

	// Read file and put it in buffer
	std::vector < uint8_t > buffer( std::istreambuf_iterator< char >( firmware_file ), { } );

	/* Check bank boundaries. */
	if ( buffer.size() > dev_mem_size ) {
		std::string err_msg( tag );
		err_msg += "You are trying to load firmware that is too big. Max size must be <= ";
		err_msg	+= std::to_string( dev_mem_size ) +  ".";
		throw std::runtime_error { err_msg };
	}

	constexpr unsigned int load_chunk = 16;

	// Ensure that firmware loaded once since mpu6050 powered up.
	std::vector < uint8_t > mem_contents;
	for ( uint16_t i = 0; i < dev_mem_size; i += load_chunk ) {
		std::vector < uint8_t > read_chunk( read_mem( i, load_chunk ) );
		mem_contents.insert( mem_contents.end(), read_chunk.begin(), read_chunk.end() );
	}

	if ( !std::equal( buffer.begin(), buffer.end(), mem_contents.begin() ) ) {
		int this_write { 0 };
		uint firm_length { buffer.size() };

		for ( uint i = 0; i < firm_length; i += this_write ) {
			this_write = std::min( load_chunk, firm_length - i );

			std::vector < uint8_t > write_buff { &buffer[ i ], &buffer[ i + this_write ] };

			write_mem( i, write_buff );

			mem_contents = read_mem( i, this_write );

			if ( !std::equal( mem_contents.begin(), mem_contents.end(), write_buff.begin() ) ) {
				std::string err_msg( tag );
				err_msg += "Data validation error while uploading firmware.";
				throw std::runtime_error { err_msg };
			}
		}

		I2C_driver::I2C_buffer buff;
		buff.reg = reg_dmp_fw_start_H;
		buff.data.push_back( mem_start_address >> 8 );
		buff.data.push_back( mem_start_address & 0xFF );

		m_I2C_driver.write( buff );
	}
	m_firmware_loaded = true;
}

/**
 * \brief This function puts device into standby mode.
 */
void MPU6050::standby()
{
	m_I2C_driver.write_bit( reg_pwr_mgmt_1, pwr_mgmt_1_standby_bit, true );

	m_pwr_on = false;
}
/**
 * \brief This function put device into normal mode.
 */
void MPU6050::wake_up( )
{
	m_I2C_driver.write_bit( reg_pwr_mgmt_1, pwr_mgmt_1_standby_bit, false );
	m_I2C_driver.write_bit( reg_pwr_mgmt_1, pwr_mgmt_1_clk_select_bit, true );

	using namespace std::chrono_literals;
	std::this_thread::sleep_for( 100ms );

	m_pwr_on = true;
}

/**
 * \brief This function reset device and put it into standby mode.
 */
void MPU6050::reset()
{
	m_I2C_driver.write_bit( reg_pwr_mgmt_1, pwr_mgmt_1_reset_bit, true );

	using namespace std::chrono_literals;
	std::this_thread::sleep_for( 100ms );

	m_pwr_on =  false;
}

/**
 * \brief Returns data read directly by the gyro registers
 * \return MPU6050::Gyro_accel_data type
 */
MPU6050::Gyro_accel_data MPU6050::get_gyro_raw_data()
{
	if ( !m_pwr_on )
		throw std::logic_error( std::string( tag ) + std::string( msg_no_wake ) );

	Gyro_accel_data recv_data = { 0, 0, 0 };

	if ( !m_pwr_on )
		throw std::logic_error( std::string( tag ) + std::string( msg_no_wake ) );

	if ( m_interrupt_on ) {
		std::lock_guard< std::mutex > lk( m_mutex );
		recv_data.x = m_mpu_packet.gyro_x;
		recv_data.y = m_mpu_packet.gyro_y;
		recv_data.z = m_mpu_packet.gyro_z;
	}
	else {
		std::vector< uint8_t > data( m_I2C_driver.read( reg_gyro_xout_h, 6 ) );
		if ( data.size() == 6 ) {
			recv_data.x = ( data[ 0 ] << 8 ) | data[ 1 ];
			recv_data.y = ( data[ 2 ] << 8 ) | data[ 3 ];
			recv_data.z = ( data[ 4 ] << 8 ) | data[ 5 ];
		}
	}

	return recv_data;
}

/**
 * \brief Returns data read directly by the gyro registers
 * \return MPU6050::Gyro_accel_data type
 */
MPU6050::Gyro_accel_data MPU6050::get_accel_raw_data()
{
	if ( !m_pwr_on )
		throw std::logic_error( std::string( tag ) + std::string( msg_no_wake ) );

	Gyro_accel_data recv_data = { 0, 0, 0 };

	if ( m_interrupt_on ) {
		std::lock_guard< std::mutex > lk( m_mutex );
		recv_data.x = m_mpu_packet.accel_x;
		recv_data.y = m_mpu_packet.accel_y;
		recv_data.z = m_mpu_packet.accel_z;
	}
	else {
		std::vector< uint8_t > data( m_I2C_driver.read( reg_accel_xout_h, 6 ) );
		if ( data.size() == 6 ) {
			recv_data.x = ( data[ 0 ] << 8 ) | data[ 1 ];
			recv_data.y = ( data[ 2 ] << 8 ) | data[ 3 ];
			recv_data.z = ( data[ 4 ] << 8 ) | data[ 5 ];
		}
	}

	return recv_data;
}

/**
 * @brief Sets digital low pass filter.
 * @param bw takes one of enum class Dlpf.
 */
void MPU6050::set_dlpf( Dlpf bw )
{
	if ( !m_pwr_on )
		throw std::logic_error( std::string( tag ) + std::string( msg_no_wake ) );

	write_reg( reg_config, static_cast< uint8_t >( bw ) );
}


/**
 * @brief Sets the sampling rate of gyro.
 *
 * @param rate accepted value is: 4hz < value > 1000hz.\n Default value is 50.
 */
void MPU6050::set_gyro_rate( uint16_t rate )
{
	if ( !m_pwr_on )
		throw std::logic_error( std::string( tag ) + std::string( msg_no_wake ) );
	if ( m_dmp_on )
		throw std::logic_error( std::string( tag ) + std::string( msg_rate_change_not_allowed ) );

	if ( rate < 4 )
		rate = 4;

	if ( rate > 1000 )
		rate = 1000;

	write_reg( reg_smplrt_div, 1000 / rate - 1 );

	Dlpf lpf_rate;
	rate = rate >> 1;
	if (rate >= 188)
        lpf_rate = Dlpf::bw_188hz;
    else if (rate >= 98)
        lpf_rate = Dlpf::bw_98hz;
    else if (rate >= 42)
        lpf_rate = Dlpf::bw_42hz;
    else if (rate >= 20)
        lpf_rate = Dlpf::bw_20hz;
    else if (rate)
        lpf_rate = Dlpf::bw_10hz;
    else {
        lpf_rate = Dlpf::bw_5hz;
    }

	set_dlpf( lpf_rate );// Automatically set LPF to 1/2 sampling rate.
}

/**
 * @brief Selects the full scale range of the gyroscope outputs.
 * @param range takes one of enum class Gyro_fsr.\n Default value is Gyro_fsr::gfs_2000dps.
 */
void MPU6050::set_gyro_fsr( Gyro_fsr range )
{
	if ( !m_pwr_on )
		throw std::logic_error( std::string( tag ) + std::string( msg_no_wake ) );

	write_reg( reg_gyro_config, static_cast< uint8_t >( range ) << 3 );
}

/**
 * @brief Selects the full scale range of the gyroscope outputs.
 * @param range takes one of enum class Accel_fsr.\n Default value is Accel_fsr::afs_2g.
 */
void MPU6050::set_accel_fsr( Accel_fsr range )
{
	if ( !m_pwr_on )
		throw std::logic_error( std::string( tag ) + std::string( msg_no_wake ) );

	write_reg( reg_accel_config, static_cast< uint8_t >( range ) << 3 );
}

/**
 * @brief Sets which sensor measurements are loaded into the FIFO buffer.
 * When a flag of Fifo_en_flags is set, data from the sensor data registers will be
 * loaded into the FIFO buffer.
 * @warning All other flags that not included in "flags" variable will be disabled.
 * @warning This function not working if dmp firmware 6.12 is loaded.
 * @param flags
 */
void MPU6050::load_in_fifo( Bitflag< Fifo_en_flags > flags )
{
	if ( !m_pwr_on )
		throw std::logic_error( std::string( tag ) + std::string( msg_no_wake ) );

	if ( !m_temp_sens_on )
		flags.unset( Fifo_en_flags::temp_sensor );

	if ( flags.test( Fifo_en_flags::slv0 ) ) //}
		flags.unset( Fifo_en_flags::slv0 );  //}
	if ( flags.test( Fifo_en_flags::slv1 ) ) //}
		flags.unset( Fifo_en_flags::slv1 );  //} We not support i2c master interface.
	if ( flags.test( Fifo_en_flags::slv2 ) ) //}
		flags.unset( Fifo_en_flags::slv2 );  //}

	uint8_t packet_size{ 0 };
	if ( flags.test( Fifo_en_flags::accel ) )
		packet_size += 6;
	if ( flags.test( Fifo_en_flags::gyro_x ) )
		packet_size += 2;
	if ( flags.test( Fifo_en_flags::gyro_y ) )
		packet_size += 2;
	if ( flags.test( Fifo_en_flags::gyro_z ) )
		packet_size += 2;
	if ( flags.test( Fifo_en_flags::temp_sensor ) )
		packet_size += 2;
	m_mpu_packet_size  = packet_size;

	write_reg( reg_fifo_enable, flags.value() );
	m_fifo_flags = flags;
}

/**
 * @brief Enables FIFO operations.
 * @param enable true to enable FIFO.
 */
void MPU6050::enable_fifo( bool enable )
{
	if ( !m_pwr_on )
		throw std::logic_error( std::string( tag ) + std::string( msg_no_wake ) );

	m_I2C_driver.write_bit( reg_user_ctrl, usr_ctrl_fifo_enable_bit, enable );

	if ( m_dmp_on )
		m_I2C_driver.write_bit( reg_user_ctrl, usr_ctrl_fifo_dmp_enable_bit, true );
}

/**
 * @brief Resets FIFO buffer.
 */
void MPU6050::reset_fifo()
{
	if ( !m_pwr_on )
		throw std::logic_error( std::string( tag ) + std::string( msg_no_wake ) );

	enable_interrupt( false ); // Clear interrupts.

	if ( m_dmp_on )
		m_I2C_driver.write_bit( reg_user_ctrl, usr_ctrl_fifo_dmp_reset_bit, true );
	else
		m_I2C_driver.write_bit( reg_user_ctrl, usr_ctrl_fifo_reset_bit, true );

	std::this_thread::sleep_for( std::chrono::milliseconds( 50 ) );

	enable_interrupt( true );
}

/**
 *  @brief Enable/disable data ready interrupt.
 *  If the DMP is on, the DMP interrupt is enabled. Otherwise, the data ready
 *  interrupt is used.
 *  @param enable true to enable interrupt.
 */
void MPU6050::enable_interrupt( bool enable )
{
	if ( !m_pwr_on )
		throw std::logic_error( std::string( tag ) + std::string( msg_no_wake ) );
	if ( enable == m_interrupt_on ) // Discard to write same thinks.
		return;

	uint8_t value{ 0 };

	if ( enable ) {
		enable_fifo( true );
		m_dmp_on ? value = static_cast< uint8_t >( Interrupt_flags::dmp ) :
			value = static_cast< uint8_t >( Interrupt_flags::data_ready );

		value |= static_cast< uint8_t >( Interrupt_flags::fifo_everflow );
		write_reg( reg_int_enable, value );

		if ( !m_dmp_on ) {
			load_in_fifo( Bitflag< Fifo_en_flags > (
					  	  Fifo_en_flags::accel, Fifo_en_flags::gyro_x,
						  Fifo_en_flags::gyro_y, Fifo_en_flags::gyro_z,
						  Fifo_en_flags::temp_sensor ) );
		}

		m_interrupt_on = true;
	}
	else {
		write_reg( reg_int_enable, 0 );
		load_in_fifo( Bitflag< Fifo_en_flags >() );
		enable_fifo( false );
		m_interrupt_on = false;
	}
}

/**
 * @brief This function binds chip orientation to mounted frame orientation.
 * For example if chip orientation mounted as y horizontal, x vertical, z looks down and
 * your object orientation is x horizontal, y vertical and z looks up, then
 * you must call this function as this: mount_on_frame( Chip_axis::y_axis, Chip_axis::minus_x_axis, Chip_axis::minus_z_axis );\n
 * Another example it would be, chip orientation mounted as y horizontal, x vertical, z looks up and
 * your object orientation is x horizontal, y vertical and z looks up, then
 * you must call this function as this: mount_on_frame( Chip_axis::y_axis, Chip_axis::x_axis, Chip_axis::z_axis );
 * @warning This function only runs when the firmware is loaded.
 * @param x_frame_as bind x object axis to Chip_axis
 * @param y_frame_as bind y object axis to Chip_axis
 * @param z_frame_as bind z object axis to Chip_axis
 */
void MPU6050::mount_on_frame( Chip_axis x_frame_as, Chip_axis y_frame_as, Chip_axis z_frame_as )
{
	if ( !m_pwr_on )
		throw std::logic_error( std::string( tag ) + std::string( msg_no_wake ) );

	if ( !m_dmp_on)
		throw std::logic_error( std::string( tag ) + std::string( msg_dmp_off ) );

	Chip_axis frame_mount[] = { x_frame_as, y_frame_as, z_frame_as };

	std::vector< uint8_t > gyro_mount( 3 );
	std::vector< uint8_t > gyro_mount_sign( 3 );

	std::vector< uint8_t > accel_mount( 3 );
	std::vector< uint8_t > accel_mount_sign( 3 );

	for ( int i = 0; i < 3; ++i ) {
		switch ( frame_mount[ i ] ) {
			case Chip_axis::x:
				gyro_mount.push_back( dmp_constant_gyro_x_axis );
				gyro_mount_sign.push_back( dmp_constant_gyro_sign_x_axis );
				accel_mount.push_back( dmp_constant_accel_x_axis );
				accel_mount_sign.push_back( dmp_constant_accel_sign_x_axis );
				break;
			case Chip_axis::y:
				gyro_mount.push_back( dmp_constant_gyro_y_axis );
				gyro_mount_sign.push_back( dmp_constant_gyro_sign_y_axis );
				accel_mount.push_back( dmp_constant_accel_y_axis );
				accel_mount_sign.push_back( dmp_constant_accel_sign_y_axis );
				break;
			case Chip_axis::z:
				gyro_mount.push_back( dmp_constant_gyro_z_axis );
				gyro_mount_sign.push_back( dmp_constant_gyro_sign_z_axis );
				accel_mount.push_back( dmp_constant_accel_z_axis );
				accel_mount_sign.push_back( dmp_constant_accel_sign_z_axis );
				break;
			case Chip_axis::minus_x:
				gyro_mount.push_back( dmp_constant_gyro_x_axis );
				gyro_mount_sign.push_back( dmp_constant_gyro_sign_x_axis | 0x01 );
				accel_mount.push_back( dmp_constant_accel_x_axis );
				accel_mount_sign.push_back( dmp_constant_accel_sign_x_axis | 0x01 );
				break;
			case Chip_axis::minus_y:
				gyro_mount.push_back( dmp_constant_gyro_x_axis );
				gyro_mount_sign.push_back( dmp_constant_gyro_sign_x_axis | 0x01 );
				accel_mount.push_back( dmp_constant_accel_x_axis );
				accel_mount_sign.push_back( dmp_constant_accel_sign_x_axis | 0x01 );
				break;
			case Chip_axis::minus_z:
				gyro_mount.push_back( dmp_constant_gyro_x_axis );
				gyro_mount_sign.push_back( dmp_constant_gyro_sign_x_axis | 0x01 );
				accel_mount.push_back( dmp_constant_accel_x_axis );
				accel_mount_sign.push_back( dmp_constant_accel_sign_x_axis | 0x01 );
				break;
		}
	}

	write_mem( reg_gyro_mount_matrix_config_1, gyro_mount );
	write_mem( reg_gyro_mount_matrix_config_sign_1, gyro_mount_sign );
	write_mem( reg_accel_mount_matrix_config_1, accel_mount );
	write_mem( reg_accel_mount_matrix_config_sign_1, accel_mount_sign );
}

/**
 * @brief Set FIFO divider.
 * By default, data will be written from the DMP to the FIFO at 200Hz.
 * These registers set a N+1integer divider to reduce the data output rate.
 * Ex1: for N=0 (the default) the output data is 200/(0 + 1) = 200 Hz.
 * Ex2: for N=7, the rate at which FIFO receives data is 200/ (7+1) = 25 Hz.
 * @warning This function only runs when the firmware is loaded.
 * @param rate in hertz between 0 to 200.
 */
void MPU6050::dmp_set_fifo_rate( uint8_t rate_hz )
{
	if ( !m_pwr_on )
		throw std::logic_error( std::string( tag ) + std::string( msg_no_wake ) );

	if ( !m_dmp_on)
		throw std::logic_error( std::string( tag ) + std::string( msg_dmp_off ) );

	if ( mpu_FIFO_max_rate < rate_hz )
		rate_hz = 200;

	uint16_t divider = mpu_FIFO_max_rate / ( rate_hz - 1 );
	std::vector< uint8_t > data;
	data.push_back( divider >> 8 );
	data.push_back( divider & 0xFF );
	write_mem( reg_fifo_rate_div, data ); // Set divider.

	std::vector< uint8_t >  en_div( val_enable_fifo_rate_div, std::end( val_enable_fifo_rate_div ) );
	write_mem( reg_enable_fifo_rate_div, en_div); // Enable divider.
}

void MPU6050::enable_dmp( bool enable )
{
	if ( !m_pwr_on )
		throw std::logic_error( std::string( tag ) + std::string( msg_no_wake ) );
	if ( !m_firmware_loaded )
		throw std::logic_error( std::string( tag ) + std::string( msg_firmware_not_loaded ) );

	if ( enable ){
		enable_interrupt( false );
		set_gyro_rate( mpu_FIFO_max_rate );
		load_in_fifo( Bitflag< Fifo_en_flags >() );
		enable_interrupt( true );
		reset_fifo();
		m_dmp_on = true;
	}
	else {
		enable_interrupt( false );
		load_in_fifo( m_fifo_flags ); // Restore fifo configuration.
		reset_fifo();
		m_dmp_on = false;
	}
}

void MPU6050::read_fifo()
{
	std::lock_guard< std::mutex > lk( m_mutex );
	if ( !m_dmp_on ){
		std::vector< uint8_t >data( m_I2C_driver.read( reg_fifo_r_w, m_mpu_packet_size ) );
		uint8_t index{ 0 };
		if ( m_fifo_flags.test( Fifo_en_flags::accel ) ) {
			m_mpu_packet.accel_x = ( data[ index + 0 ] << 8 ) | data[ index + 1 ];
			m_mpu_packet.accel_y = ( data[ index + 2 ] << 8 ) | data[ index + 3 ];
			m_mpu_packet.accel_z = ( data[ index + 4 ] << 8 ) | data[ index + 5 ];
			index += 6;
		}

		if ( m_fifo_flags.test( Fifo_en_flags::temp_sensor ) ) {
			m_mpu_packet.temp = ( data[ index + 0 ] << 8 ) | data[ index + 1 ];
			index += 2;
		}

		if ( m_fifo_flags.test( Fifo_en_flags::gyro_x ) ) {
			m_mpu_packet.gyro_x = ( ( data[ index + 0 ] << 8 ) | data[ index + 1 ] ) / 340.0f + 35.0f;
			index += 2;
		}

		if ( m_fifo_flags.test( Fifo_en_flags::gyro_y ) ) {
			m_mpu_packet.gyro_y = ( ( data[ index + 0 ] << 8 ) | data[ index + 1 ] ) / 340.0f + 35.0f;
			index += 2;
		}

		if ( m_fifo_flags.test( Fifo_en_flags::gyro_z ) ) {
			m_mpu_packet.gyro_z = ( ( data[ index + 0 ] << 8 ) | data[ index + 1 ] ) / 340.0f + 35.0f;
			index += 2;
		}
	}
	else {

	}
}

void MPU6050::fifo_thread() {
	while ( true ) {
		std::unique_lock< std::mutex > lk( m_mutex );
		m_data_ready.wait( lk, [&]() {
			return m_more_data || m_exit_thread; } );

		if ( m_exit_thread )
			return;

		int bytes_in_fifo = ( read_reg( reg_fifo_counth ) << 8 )|
						      read_reg( reg_fifo_countl ) ;

		std::cout << "Fifo bytes: " << bytes_in_fifo << std::endl;

		m_more_data = bytes_in_fifo > m_mpu_packet_size;

		if ( bytes_in_fifo < m_mpu_packet_size ) {
			continue;
		}

		/* Check overflow bit. */
		if ( bytes_in_fifo > ( fifo_max_size >> 8 ) ) {
			/* FIFO is 50% full, better check overflow bit. */
			if ( m_I2C_driver.read_bit( reg_int_status,  fifo_oflow_int_bit ) ) {
				reset_fifo();
				continue;
			}
		}

		lk.unlock();
		read_fifo();
	}
}

void MPU6050::write_reg( uint8_t reg, uint8_t value )
{
	I2C_driver::I2C_buffer buff;
	buff.reg = reg;
	buff.data.push_back( value );

	m_I2C_driver.write( buff );
}

uint8_t MPU6050::read_reg( uint8_t reg )
{
	std::vector< uint8_t > ret( m_I2C_driver.read( reg, 1 ) );

	return ret[ 0 ];
}

/**
 * @brief Reads from register address of mpu.
 * @param mem_address address of mpu register.
 * @param bytes how many bytes
 * @return vector of requested bytes.
 */
std::vector< uint8_t > MPU6050::read_mem( uint16_t mem_address, uint16_t bytes )
{

	I2C_driver::I2C_buffer sel_bank;

	sel_bank.reg = reg_bank_sel;
	sel_bank.data.push_back( mem_address >> 8 );
	sel_bank.data.push_back( mem_address & 0xff );

	m_I2C_driver.write( sel_bank );

	return m_I2C_driver.read( reg_mem_r_w, bytes );
}

/**
 * @brief Writes from register address of mpu.
 * @param mem_address address of mpu register.
 * @param data vector of bytes to write.
 */
void MPU6050::write_mem( uint16_t mem_address, std::vector< uint8_t > data )
{
	I2C_driver::I2C_buffer sel_bank;

	sel_bank.reg = reg_bank_sel;
	sel_bank.data.push_back( mem_address >> 8 );
	sel_bank.data.push_back( mem_address & 0xff );

	m_I2C_driver.write( sel_bank );

	I2C_driver::I2C_buffer buff;
	buff.reg = reg_mem_r_w;
	buff.data = data;
	m_I2C_driver.write( buff );
}

void MPU6050::on_interrupt( int pi, unsigned pin, unsigned level, unsigned tick, void *userdata )
{
	MPU6050* p = reinterpret_cast< MPU6050* >( userdata );

	p->m_more_data = true;
	p->m_data_ready.notify_one();

	if ( !p->m_dmp_on ) {
		if ( p->m_listener != nullptr ) {
			std::unique_lock< std::mutex > lk( p->m_mutex );
			p->m_listener( p->m_mpu_packet );
		}
	}
}


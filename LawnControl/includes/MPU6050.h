/*
 *	MPU6050.h
 *
 *	MPU6050 Driver declaration.
 *
 *	IMPORTAND!! The device power state, managed by driver.
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
#include <string>

namespace MPU6050 {

struct Gyro_accel_data {
	short int X;
	short int Y;
	short int Z;
};

class Device : private I2C_driver{
public:
	enum Dlpf : uint8_t {
		bw_256hz,
		bw_188hz,
		bw_98hz,
		bw_42hz,
		bw_20hz,
		bw_10hz,
		bw_5hz
	};

	enum Guro_fsr : uint8_t {
		gfs_250dps,
		gfs_500dps,
		gfs_1000dps,
		gfs_2000dps
	};

	enum Accel_fsr : uint8_t {
		afs_2g,
		afs_4g,
		afs_8g,
		afs_16g
	};

	// This class must be created once. So we use singletone pattern.
	static Device& Init(const int i2cbus, const uint8_t address){
		static Device instance{ i2cbus, address };
	    return instance;
	}

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
	void	Standby(); // This function puts device into standby mode.
	void	Wake_up(); // This function put device into normal mode.
	void	Reset(); // This function reset device.

	void	Disable_temp_sensor(bool Disable = true); // By default sensor is On. True set sensor Off, false set sensor On.

	// Functions related to Invensense Motion Driver.
	void		Load_firmware(const std::string file_path); // Return false if failed;

	~Device();

private:
	// The following enum declares all MPU6050 internal registers.
	enum Register : uint8_t {
		xg_offs_tc			= 0x00, // [7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
		yg_offs_tc      	= 0x01, // [7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
		zg_offs_tc      	= 0x02, // [7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
		x_fine_gain     	= 0x03, // [7:0] X_FINE_GAIN
		y_fine_gain     	= 0x04, // [7:0] Y_FINE_GAIN
		z_fine_gain     	= 0x05, // [7:0] Z_FINE_GAIN
		xa_offs_h       	= 0x06, // Accel X-axis offset cancellation register high byte
		xa_offs_l_tc    	= 0x07, // Accel X-axis offset cancellation register low byte
		ya_offs_h      		= 0x08, // Accel Y-axis offset cancellation register high byte
		ya_offs_l_tc    	= 0x09, // Accel Y-axis offset cancellation register low byte
		za_offs_h       	= 0x0A, // Accel Z-axis offset cancellation register high byte
		za_offs_l_tc    	= 0x0B, // Accel Z-axis offset cancellation register low byte
		self_test_x     	= 0x0D, // [7:5] XA_TEST[4-2], [4:0] XG_TEST[4-0]
		self_test_y     	= 0x0E, // [7:5] YA_TEST[4-2], [4:0] YG_TEST[4-0]
		self_test_z     	= 0x0F, // [7:5] ZA_TEST[4-2], [4:0] ZG_TEST[4-0]
		self_test_a     	= 0x10, // [5:4] XA_TEST[1-0], [3:2] YA_TEST[1-0], [1:0] ZA_TEST[1-0]
		xg_offs_usrh    	= 0x13, // Gyro X-axis offset cancellation register high byte
		xg_offs_usrl    	= 0x14, // Gyro X-axis offset cancellation register low byte
		yg_offs_usrh    	= 0x15, // Gyro Y-axis offset cancellation register high byte
		yg_offs_usrl    	= 0x16, // Gyro Y-axis offset cancellation register low byte
		zg_offs_usrh    	= 0x17, // Gyro Z-axis offset cancellation register high byte
		zg_offs_usrl    	= 0x18, // Gyro Z-axis offset cancellation register low byte
		smplrt_div      	= 0x19, // Sample Rate Divider
		config          	= 0x1A, // Configuration
		gyro_config     	= 0x1B, // Gyroscope Configuration
		accel_config    	= 0x1C, // Accelerometer Configuration
		ff_thr          	= 0x1D,
		ff_dur          	= 0x1E,
		mot_thr         	= 0x1F,
		mot_dur         	= 0x20,
		zrmot_thr       	= 0x21,
		zrmot_dur       	= 0x22,
		fifo_en         	= 0x23, // FIFO Enable
		i2c_mst_ctrl    	= 0x24,
		i2c_slv0_addr   	= 0x25,
		i2c_slv0_reg    	= 0x26,
		i2c_slv0_ctrl   	= 0x27,
		i2c_slv1_addr   	= 0x28,
		i2c_slv1_reg    	= 0x29,
		i2c_slv1_ctrl   	= 0x2A,
		i2c_slv2_addr   	= 0x2B,
		i2c_slv2_reg    	= 0x2C,
		i2c_slv2_ctrl   	= 0x2D,
		i2c_slv3_addr   	= 0x2E,
		i2c_slv3_reg    	= 0x2F,
		i2c_slv3_ctrl   	= 0x30,
		i2c_slv4_addr   	= 0x31,
		i2c_slv4_reg    	= 0x32,
		i2c_slv4_do     	= 0x33,
		i2c_slv4_ctrl   	= 0x34,
		i2c_slv4_di     	= 0x35,
		i2c_mst_status  	= 0x36,
		int_pin_cfg     	= 0x37,
		int_enable      	= 0x38,
		dmp_int_status  	= 0x39,
		int_status      	= 0x3A,
		accel_xout_h    	= 0x3B,
		accel_xout_l    	= 0x3C,
		accel_yout_h    	= 0x3D,
		accel_yout_l    	= 0x3E,
		accel_zout_h    	= 0x3F,
		accel_zout_l    	= 0x40,
		temp_out_h      	= 0x41,
		temp_out_l      	= 0x42,
		gyro_xout_h     	= 0x43,
		gyro_xout_l     	= 0x44,
		gyro_yout_h     	= 0x45,
		gyro_yout_l     	= 0x46,
		gyro_zout_h     	= 0x47,
		gyro_zout_l      	= 0x48,
		ext_sens_data_00  	= 0x49,
		ext_sens_data_01  	= 0x4A,
		ext_sens_data_02  	= 0x4B,
		ext_sens_data_03  	= 0x4C,
		ext_sens_data_04  	= 0x4D,
		ext_sens_data_05  	= 0x4E,
		ext_sens_data_06  	= 0x4F,
		ext_sens_data_07  	= 0x50,
		ext_sens_data_08  	= 0x51,
		ext_sens_data_09  	= 0x52,
		ext_sens_data_10  	= 0x53,
		ext_sens_data_11  	= 0x54,
		ext_sens_data_12  	= 0x55,
		ext_sens_data_13  	= 0x56,
		ext_sens_data_14  	= 0x57,
		ext_sens_data_15  	= 0x58,
		ext_sens_data_16  	= 0x59,
		ext_sens_data_17  	= 0x5A,
		ext_sens_data_18  	= 0x5B,
		ext_sens_data_19  	= 0x5C,
		ext_sens_data_20  	= 0x5D,
		ext_sens_data_21  	= 0x5E,
		ext_sens_data_22  	= 0x5F,
		ext_sens_data_23  	= 0x60,
		mot_detect_status 	= 0x61,
		i2c_slv0_do       	= 0x63,
		i2c_slv1_do       	= 0x64,
		i2c_slv2_do       	= 0x65,
		i2c_slv3_do       	= 0x66,
		i2c_mst_delay_ctrl 	= 0x67,
		signal_path_reset  	= 0x68,
		mot_detect_ctrl    	= 0x69,
		user_ctrl        	= 0x6A,
		pwr_mgmt_1       	= 0x6B,
		pwr_mgmt_2       	= 0x6C,
		bank_sel         	= 0x6D,
		mem_start_addr   	= 0x6E,
		mem_r_w          	= 0x6F,
		dmp_cfg_1        	= 0x70,
		dmp_cfg_2        	= 0x71,
		fifo_counth      	= 0x72,
		fifo_countl      	= 0x73,
		fifo_r_w         	= 0x74,
		who_am_i         	= 0x75,
	};

	/*
	 *	The following enum declares, the values ​​that registers above must have
	 *in order to enable or disable device's capabilities.
	 *
	 *	For extra study please refer to InveSense document
	 *"MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2"
	 */

	enum Pwr_mgmt_1 : uint8_t { 	// Register 107 – Power Management 1.
		clk_select_bit	= 0, // Always use PLL with X axis gyroscope reference.
		temp_dis_bit	= 3,
		cycle_bit		= 5,
		standby_bit,
		reset_bit
	};

	enum fifo_enable {
		accel_reg_bit	= 3,
		gz_reg_bit,
		gy_reg_bit,
		gx_reg_bit,
		temp_reg_bit
	};

	enum User_control {
		reset_sig_paths	= 0,
		fifo_reset 		= 2,
		fifo_enable		= 6
	};

	Device( const int i2cbus, const uint8_t address );

	bool Mpu_pwr_on();

	bool Read_bit( Register reg , uint8_t bit_number);
	void Write_bit( Register reg, uint8_t bit_number, bool state);

	// Function related to Invensense Motion Driver.
	void 		Write_mem(uint16_t mem_address, uint16_t length, std::vector<uint8_t> data);
	std::vector<uint8_t> Read_mem(uint16_t mem_address, uint16_t length);

	bool 		m_dmp_on; // If dmp is enabled then true;
};

} // End of namespace MPU6050.

#endif /* CMPU6050_H_ */

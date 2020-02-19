/**
 * @brief This class implements InvenSense mpu6050 motion sensor functionality.
 * To use mpu features, you must first call this functions with following order:\n
 * <TT>load_firmware(Args..),<br>    mount_on_frame(Args..)</TT>
 * @author Panagiotis Charisopoulos.
 * @date October 20 2019
 * @version 1.0
 * @warning	This implementation does not implement to control i2c slaves,
 * 			so mpu is set to i2c bypass mode.
 * @note When this object created, initializes device with following:
 * 		 Resets device, set clock source to gyro x axis reference, set gyroscope sample rate
 * 		 to 2000 degrees per second and rate divider to 50, set DLPF to 42hz, set accelerometer
 * 		 full scale range to 2g, disables fifo output and finally put device into standby mode.
 * 		 The same think happens when firmware loaded.
 * @note If "load_firmware" used the mpu initialized. This means all previous configurations
 * 		 will be lost.
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
#pragma once

#ifndef __arm__
#error You must use this code only for RPi
#endif

#include "rpi_i2c.h"
#include "bitflag.h"
#include <stdint.h>
#include <vector>
#include <thread>
#include <condition_variable>
#include <functional>

class MPU6050 final {
public:
	struct Fifo_mpu_packet {
		uint16_t accel_x;
		uint16_t accel_y;
		uint16_t accel_z;
		uint16_t temp;
		uint16_t gyro_x;
		uint16_t gyro_y;
		uint16_t gyro_z;
	};

	/**
	 * @brief Used for returning gyro and accel raw data.
	 */
	struct Gyro_accel_data {
		uint16_t x; ///< Holds rotation value by X axis.
		uint16_t y; ///< Holds rotation value by Y axis.
		uint16_t z; ///< Holds rotation value by Z axis.
	};

	/**
	 * @brief Chip axis.
	 * This enum, used to bind chip orientation with your object orientation.
	 * e.g. your object maybe is a breadboard.
	 */
	enum class Chip_axis : uint8_t {
		x,      //!< mpu x_axis.
		y,      //!< mpu y_axis.
		z,      //!< mpu z_axis.
		minus_x,//!< mpu reversed x_axis.
		minus_y,//!< mpu reversed y_axis.
		minus_z //!< mpu reversed z_axis.
	};

	MPU6050( const int i2cbus, const uint8_t address, unsigned interrupt_pin );
	~MPU6050();

	uint8_t device_id(); // Get device id.

	float get_ambient_temp();

	Gyro_accel_data get_gyro_raw_data();
	Gyro_accel_data get_accel_raw_data();

	// functions, for interrupts.
	void enable_interrupt( bool enable );
	void register_on_interrupt( std::function< void( Fifo_mpu_packet ) > listener);

	// Functions, for power management.
	void standby(); // This function puts device into standby mode.
	void wake_up(); // This function put device into normal mode.
	void reset(); // This function reset device.
	void set_temp_sensor( bool on );

	/**
	 * @brief Returns current mpu's power state.
	 * @return True if mpu is wake. False for standby.
	 */
	bool get_pwr_state() { return m_pwr_on; }

	// DMP functions.
	void load_firmware( const std::string file_path ); // Throws runtime error with message;
	void mount_on_frame( Chip_axis x_frame_as, Chip_axis y_frame_as, Chip_axis z_frame_as );
	void dmp_set_fifo_rate( uint8_t rate_hz );
	void enable_dmp( bool enable );

private:
	/**
	 * @brief This enumeration used for low pass filter sampling rates.
	 */
	enum class Dlpf : uint8_t {
		bw_256hz, bw_188hz, bw_98hz, bw_42hz, bw_20hz, bw_10hz, bw_5hz
	};

	/**
	 * @brief This enumeration used for gyro sampling rates.
	 */
	enum class Gyro_fsr : uint8_t {
		gfs_250dps, gfs_500dps, gfs_1000dps, gfs_2000dps
	};

	/**
	 * @brief This enumeration used for accelerometer sampling rates.
	 */
	enum class Accel_fsr : uint8_t {
		afs_2g, afs_4g, afs_8g, afs_16g
	};

	/**
	 * @brief Fifo enable flags.
	 * This enum, determines flags for enabling which sensor
	 * measurements are loaded into the FIFO buffer.
	 */
	enum class Fifo_en_flags : uint8_t {
		slv0, ///< This flag enables EXT_SENS_DATA registers associated with Slave 0 to be written into the FIFO buffer.
		slv1, ///< Similarly for Slave 1.
		slv2, ///< Similarly for Slave 2.
		accel, ///< This flag enables ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L to be written into the FIFO buffer.
		gyro_z, ///< This flag enables GYRO_ZOUT_H and GYRO_ZOUT_L to be written into the FIFO buffer.
		gyro_y, ///< This flag enables GYRO_YOUT_H and GYRO_YOUT_L to be written into the FIFO buffer.
		gyro_x, ///< This flag enables GYRO_XOUT_H and GYRO_XOUT_L to be written into the FIFO buffer.
		temp_sensor ///< This flag enables TEMP_OUT_H and TEMP_OUT_L to be written into the FIFO buffer.
	};

	enum class Clock_source : uint8_t {
		internal,
		x_axis_gyro_ref,
		y_axis_gyro_ref,
		z_axis_gyro_ref
	};

	/**
	 * @brief This enumeration used for enable interrupt generation by interrupt sources.
	 */
	enum class Interrupt_flags : uint8_t {
		data_ready 		= 1, 	  ///< This flag enables the Data Ready interrupt, which occurs each time a write operation to all of the sensor registers has been completed.
		dmp 			= 1 << 1, ///< This flag enables DMP interrupts. Used only when dmp 6.12 firmware loaded to mpu.
		fifo_everflow 	= 1 << 4  ///< This flag enables a FIFO buffer overflow to generate an interrupt.
		// This interrupts not used.
		//i2c_master 	 	= 1 << 3, ///< This flag enables any of the I2C Master interrupt sources to generate an interrupt.
		/*zero_motion		= 1 << 5, ///< This flag enables Free Fall detection to generate an interrupt. Used only when dmp 6.12 firmware loaded to mpu.
		motion			= 1 << 6, ///< this flag enables Motion detection to generate an interrupt. Used only when dmp 6.12 firmware loaded to mpu.
		free_fall 		= 1 << 7///< this flag enables Zero Motion detection to generate an interrupt. Used only when dmp 6.12 firmware loaded to mpu.*/
	};

	std::vector< uint8_t > read_mem( uint16_t mem_address, uint16_t bytes );

	void set_clock_source( Clock_source clk_src );

	bool m_dmp_on; ///< If future on then true.
	bool m_firmware_loaded;
	bool m_pwr_on; ///< True if device is waked.
	bool m_temp_sens_on;
	bool m_interrupt_on;

	int m_pi; // Used for pigpiod_if2 library.
	int m_callback_id; // Id for cancel callback on destroy.
	unsigned m_interrupt_pin;
	I2C_driver m_I2C_driver; ///< I2C driver interface.

	std::function< void( Fifo_mpu_packet ) > m_listener;

	Bitflag< Fifo_en_flags > m_fifo_flags;

	// Functions, for fifo management.
	Fifo_mpu_packet m_mpu_packet;
	uint8_t m_mpu_packet_size;

	void set_dlpf( Dlpf bw );
	void set_gyro_rate( uint16_t rate );
	void set_gyro_fsr( Gyro_fsr range );
	void set_accel_fsr( Accel_fsr range );

	void load_in_fifo( Bitflag< Fifo_en_flags > flags );
	void enable_fifo( bool enable );
	void reset_fifo();
	void read_fifo();

	bool m_exit_thread;
	bool m_more_data;
	std::thread m_read_fifo_thread;
	std::condition_variable m_data_ready;
	std::mutex m_mutex;
	void fifo_thread();

	void init_mpu( );
	uint8_t read_reg( uint8_t reg );
	void write_reg( uint8_t reg, uint8_t value);
	// Function related to Invensense Motion Driver.
	void write_mem( uint16_t mem_address, std::vector< uint8_t > data );

	static void on_interrupt( int pi, unsigned pin,
							  unsigned level, unsigned tick, void *userdata );
};

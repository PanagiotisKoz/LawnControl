/*
 *	lawn_control.cpp
 *
 *	This is program for con.
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

#include "includes/server.h"
#include "includes/lawn_logic_manager.h"
#include "includes/mpu6050.h"
#include <cmath>
#include <boost/asio.hpp>
#include <syslog.h>
#include <type_traits>
#include <stdexcept>
#include <iostream>

constexpr unsigned Listening_port = 8080;

void on_signal( MPU6050::Fifo_mpu_packet packet )
{
	/*double roll{ 0.0f }, pitch{ 0.0f };
	roll = atan2( packet.accel_x, packet.accel_z ) * 57.3248f;
	pitch = atan2( ( -packet.accel_x ), sqrt( packet.accel_y * packet.accel_y + packet.accel_z * packet.accel_z ) ) * 57.3248f;
	std::cerr << "roll:" << roll <<	" Pitch:" << pitch << std::endl;*/
	std::cerr << "Ax:" << packet.accel_x << " Ay:" <<
			packet.accel_y << " Az:" << packet.accel_z << std::endl;
}

int main()
{
	try {
		/*MPU6050 mpu( 1, 0x68, 22 );
		MPU6050::Gyro_accel_data data;
		mpu.wake_up();
		mpu.register_on_interrupt( on_signal );
		mpu.enable_interrupt( true );
		while (1) {
			/*data = mpu.get_accel_raw_data();
			std::cerr << "Ax:" << data.x <<
		" Ay:" << data.y << " Az:" << data.z << std::endl;
			std::this_thread::sleep_for( std::chrono::milliseconds( 500 ) );
		}*/
		Lawn_logic_manager logic_manager;
		boost::asio::io_service io_service;

		Server server( io_service, Listening_port );

		// Register signal handlers so that the service may be shut down.
		boost::asio::signal_set signals( io_service, SIGINT, SIGTERM, SIGHUP );
		signals.async_wait( boost::bind(&boost::asio::io_service::stop, &io_service ) );

		std::cerr << "<" << LOG_INFO << ">" << "Daemon running" << std::endl;
		server.listen();
		io_service.run();
		std::cerr << "<" << LOG_INFO << ">" << "Daemon stopped" << std::endl;
	} catch ( std::runtime_error& err ) {
		std::cerr << "<" << LOG_ERR << ">" << err.what() << std::endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

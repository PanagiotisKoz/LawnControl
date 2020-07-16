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

int main()
{
	try {
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

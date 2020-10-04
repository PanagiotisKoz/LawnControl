/*
 * 	server.h
 *
 *	Copytight (C) 28 Δεκ 2019 Panagiotis charisopoulos
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef SERVER_H_
#define SERVER_H_

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <sstream>
#include <thread>
#include "event_system.h"

using boost::asio::ip::tcp;

class Server {
public:
	Server( boost::asio::io_service& io_service, uint32_t port );
	~Server();

	void listen();
	void stop();

	void on_send( std::shared_ptr< IEvent > event );

private:
	void async_read();
	void async_send( std::stringstream& os );
	void recover( const boost::system::error_code& error );

	void handle_accept( const boost::system::error_code& error );
	void handle_read( const boost::system::error_code& error );
	void handle_write( const boost::system::error_code& error, std::stringstream& os );
	void handle_request( std::istream& data );

	boost::asio::io_service& m_io_service;
	tcp::socket m_socket;
	tcp::acceptor m_acceptor;

	std::thread m_thread_listen;
};

#endif /* SERVER_H_ */

#include <boost/exception/all.hpp>
#include <iostream>
#include <syslog.h>
#include <cstring>
#include "Server.h"

Server::Server(boost::asio::io_service& io_service, uint32_t port)
	: m_io_service(io_service), m_socket(io_service),
	m_acceptor(io_service, tcp::endpoint(tcp::v4(), port))
{}

void Server::start()
{
	// Check whether the server was stopped by a signal before this
	// completion handler had a chance to run.
	if (!m_acceptor.is_open())
	{
		return;
	}

	m_acceptor.async_accept( m_socket, boost::bind(&Server::handle_accept, this, m_socket,
			boost::asio::placeholders::error ) );
}

void Server::handle_accept( tcp::socket socket, const boost::system::error_code& error)
{
	if (!error)
	{
		try
		{

		}
		catch (boost::exception& e)
		{
			std::cerr << "<" << LOG_ERR << ">" << "Session exception: "
				<< boost::diagnostic_information(e) << std::endl;
		}
	}
	else
	{

	}

	start();
}

void Server::handle_read(const boost::system::error_code& error, size_t bytes_transferred)
{
	if (!error)
	{
		try
		{

		}
		catch (boost::exception& e)
		{
			std::cerr << "<" << LOG_ERR << ">" << "Session exception: "
				<< boost::diagnostic_information(e) << std::endl;
			delete this;
		}
	}
	else
	{
		delete this;
	}
}

void Server::handle_write(const boost::system::error_code& error)
{
	if (!error)
	{
		try
		{
			m_socket.async_read_some(boost::asio::buffer(m_recv_buffer, 1),
				boost::bind(&Server::handle_read, this,
					boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred));
		}
		catch (boost::exception& e)
		{
			std::cerr << "<" << LOG_ERR << ">" << "Session exception: "
				<< boost::diagnostic_information(e) << std::endl;
			delete this;
		}
	}
	else
	{
		delete this;
	}
}

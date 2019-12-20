#include <boost/exception/all.hpp>
#include <iostream>
#include <syslog.h>
#include <cstring>
#include "Server.h"
#include "ScaleSocketProtocol.h"

extern Chx711 *g_hx711_sensor;

Session::Session(boost::asio::io_service& io_service)
	: m_socket(io_service)
{ }

void Session::start()
{
	m_socket.async_read_some(boost::asio::buffer(m_recv_buffer, 1),
		boost::bind(&Session::handle_read, this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
}

void Session::handle_read(const boost::system::error_code& error, size_t bytes_transferred)
{
	if (!error)
	{
		try
		{
			using namespace ScaleSocketProtocol;
			std::ostream send_buff(&m_send_buffer);
			
			switch (m_recv_buffer[0])
			{
				case ScaleSocketProtocol::GetIntegral:
					if (g_hx711_sensor->TareInProgress())
						send_buff << ScaleSocketProtocol::TareInProgress;
					else
						send_buff << g_hx711_sensor->GetIntegral();
					break;
				case ScaleSocketProtocol::GetTare:
					g_hx711_sensor->Tare(3000);
					send_buff << ScaleSocketProtocol::TareComplete;
					break;
				default:
					send_buff << ScaleSocketProtocol::UnknowCommand;
			}

			boost::asio::async_write(m_socket, m_send_buffer,
				boost::bind(&Session::handle_write, this,
					boost::asio::placeholders::error));
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

void Session::handle_write(const boost::system::error_code& error)
{
	if (!error)
	{
		try
		{
			m_socket.async_read_some(boost::asio::buffer(m_recv_buffer, 2),
				boost::bind(&Session::handle_read, this,
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
Server::Server(boost::asio::io_service& io_service, uint32_t port)
	: m_io_service(io_service),
	m_acceptor(io_service, tcp::endpoint(tcp::v4(), port))
{
	start_accept();
}

void Server::start_accept()
{
	// Check whether the server was stopped by a signal before this
	// completion handler had a chance to run.
	if (!m_acceptor.is_open())
	{
		return;
	}

	Session* new_session = new Session(m_io_service);

	m_acceptor.async_accept(new_session->socket(),
		boost::bind(&Server::handle_accept, this, new_session,
			boost::asio::placeholders::error));
}

void Server::handle_accept(Session* new_session, const boost::system::error_code& error)
{
	if (!error)
	{
		try
		{
			new_session->start();
		}
		catch (boost::exception& e)
		{
			std::cerr << "<" << LOG_ERR << ">" << "Session exception: "
				<< boost::diagnostic_information(e) << std::endl;
			delete new_session;
		}
	}
	else
	{
		delete new_session;
	}

	start_accept();
}
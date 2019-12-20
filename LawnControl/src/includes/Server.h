#pragma once

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/array.hpp>
#include <Chx711.h>

using boost::asio::ip::tcp;

class Session : public boost::enable_shared_from_this<Session>
{
public:
	Session(boost::asio::io_service& io_service);

	tcp::socket& socket() { return m_socket; };

	void start();

private:
	void handle_read(const boost::system::error_code& error, size_t bytes_transferred);

	void handle_write(const boost::system::error_code& error);

private:
	tcp::socket m_socket;
	boost::array<char, 1> m_recv_buffer;
	boost::asio::streambuf m_send_buffer;
};

class Server
{
public:
	Server(boost::asio::io_service& io_service, uint32_t port);

private:
	void start_accept();

private:
	void handle_accept(Session* new_session, const boost::system::error_code& error);
	boost::asio::io_service& m_io_service;
	tcp::acceptor m_acceptor;
};


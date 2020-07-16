#include "includes/server.h"
#include "includes/event_system.h"
#include <boost/asio.hpp>
#include <boost/exception/info.hpp>
#include <functional>
#include <syslog.h>
#include <iostream>

constexpr auto tag = "TCP Server -- ";

Server::Server( boost::asio::io_service& io_service, uint32_t port )
	try	: m_io_service( io_service ), m_socket( io_service ),
	  	  m_acceptor( m_io_service, tcp::endpoint( tcp::v4(), port ) )
{
	Event_manager& event_mngr = Event_manager::get_instance();
	event_mngr.add_receiver( Event_server_response::id,
		std::bind( &Server::on_send, this, std::placeholders::_1 ) );
} catch ( std::exception& e ) {
	std::string msg( tag );
	msg += e.what();
	throw std::runtime_error( msg );
}

Server::~Server()
{
	stop();
}

void Server::listen()
{
	// Check whether the server was stopped by a signal before this
	// completion handler had a chance to run.
	if ( !m_acceptor.is_open() ) {
		return;
	}

	m_acceptor.async_accept( m_socket, boost::bind( &Server::handle_accept, this, boost::asio::placeholders::error ) );
}

void Server::handle_accept( const boost::system::error_code& error )
{
	if ( !error ) {
		// Log connection accepted.
		std::cerr << "<" << LOG_INFO << ">" << "Accept connection from "
			<< m_socket.remote_endpoint().address().to_string() << " port " << m_socket.remote_endpoint().port()
			<< std::endl;

		// Start receiving.
		m_socket.set_option( boost::asio::socket_base::enable_connection_aborted( true ) );
		m_socket.set_option( boost::asio::socket_base::keep_alive( true ) );
		m_socket.set_option( boost::asio::ip::tcp::no_delay( true ) );
		m_socket.non_blocking( true );

		// Send server ok.
		std::stringstream ss;
		ss << Mower_event_ids::server_response_id::ok;
		async_send( ss );

		async_read();
	}
	else {
		recover( error );
	}
}

void Server::async_read()
{
	m_socket.async_receive( boost::asio::null_buffers(),
		boost::bind( &Server::handle_read, this, boost::asio::placeholders::error() ) );
}

void Server::handle_read( const boost::system::error_code& error )
{
	if ( !error ) {
		int bytes_to_receive { 1 };
		if ( m_socket.available() > 0 )
			bytes_to_receive = m_socket.available();

		boost::asio::streambuf buffer;
		boost::asio::streambuf::mutable_buffers_type bufs = buffer.prepare( bytes_to_receive );
		boost::system::error_code error;
		m_socket.read_some( bufs, error );
		if ( error ) {
			recover( error );
			return;
		}

		buffer.commit( bytes_to_receive );

		std::istream ss( &buffer );

		handle_request( ss );

		async_read();
	}
	else {
		if ( error.value() == boost::asio::error::try_again )
			async_read();
		else
			recover( error );
	}
}

void Server::recover( const boost::system::error_code& error )
{
	std::string msg( tag );

	switch ( error.value() ) {
		case boost::asio::error::eof:
			msg += " Connection closed by client.";
			std::cerr << "<" << LOG_INFO << ">" << msg << std::endl;
			break;
		default:
			std::cerr << "<" << LOG_WARNING << ">" << error.message() << std::endl;
	}

	if ( m_socket.is_open() )
		m_socket.close();

	std::shared_ptr< Event_stop > event = std::make_shared< Event_stop >( );
	Event_manager::get_instance().send_event( event );

	listen();
}

void Server::stop()
{
	if ( m_socket.is_open() )
		m_socket.close();

	if ( m_acceptor.is_open() )
		m_acceptor.close();
}

void Server::on_send( std::shared_ptr< IEvent > event )
{
	if ( m_socket.is_open() ) {
		std::shared_ptr< Event_server_response > msg = std::static_pointer_cast< Event_server_response >( event );

		std::stringstream ss;
		msg->serialize( ss );
		async_send( ss );
	}
}

void Server::async_send( std::stringstream& os )
{
	if ( os.rdbuf()->in_avail() == 0 )
		return;

	os << '\r';
	m_socket.async_write_some( boost::asio::buffer( os.str() ),
		boost::bind( &Server::handle_write, this, boost::asio::placeholders::error, boost::ref( os ) ) );
}

void Server::handle_write( const boost::system::error_code& error, std::stringstream& os )
{
	if ( error ) {
		if ( error.value() == boost::asio::error::try_again )
			async_send( os );
		else
			recover( error );
	}
}

void Server::handle_request( std::istream& data )
{
	unsigned request { 0 };
	data >> request;

	std::shared_ptr< IEvent > event;

	switch ( request ) {
		case Event_property_get::id:
			event = std::make_shared< Event_property_get >( data );
			break;
		case Event_property_set::id:
			event = std::make_shared< Event_property_set >( data );
			break;
		case Event_move::id:
			event = std::make_shared< Event_move >( data );
			break;
		default:
			std::shared_ptr< Event_server_response > e = std::make_shared< Event_server_response >
				( Event_server_response::Responses::command_unknow );

			on_send( e );
	}

	if ( event != nullptr )
		Event_manager::get_instance().send_event( event );

}

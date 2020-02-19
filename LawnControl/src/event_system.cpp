#include <iostream>
#include <syslog.h>
#include <future>

#include "includes/event_system.h"

constexpr auto tag = "Event Manager -- ";

Event_move::Event_move( Direction direction, int strength )
	: m_direction{ direction }, m_strength{ strength } { }

Event_move::Event_move( std::istream& is )
	: m_direction{ Direction::unknow }, m_strength{ 0 }
{
	unsigned direction{ 0 };
	is >> direction;
	is >> m_strength;

	switch( direction ) {
		case Mower_event_ids::move_direction_ids::forward:
			m_direction = Direction::forward;
			break;
		case Mower_event_ids::move_direction_ids::backward:
			m_direction = Direction::backward;
			break;
		case Mower_event_ids::move_direction_ids::left:
			m_direction = Direction::left;
			break;
		case Mower_event_ids::move_direction_ids::right:
			m_direction = Direction::right;
			break;
		case Mower_event_ids::move_direction_ids::fr:
			m_direction = Direction::fr;
			break;
		case Mower_event_ids::move_direction_ids::fl:
			m_direction = Direction::fl;
			break;
		case Mower_event_ids::move_direction_ids::br:
			m_direction = Direction::br;
			break;
		case Mower_event_ids::move_direction_ids::bl:
			m_direction = Direction::bl;
			break;
	}
}

Event_property_get::Event_property_get( Properties property )
	: m_property_id{ property } { }

Event_property_get::Event_property_get( std::istream& is )
	: m_property_id{ Properties::unknow }
{
	unsigned property{ 0 };
	is >> property;

	switch ( property ) {
		case Mower_event_ids::properties_ids::blade_height:
			m_property_id = Properties::get_blade_height;
			break;
		case Mower_event_ids::properties_ids::current_GET:
			m_property_id = Properties::get_current;
			break;
		case Mower_event_ids::properties_ids::power_GET:
			m_property_id = Properties::get_power;
			break;
		case Mower_event_ids::properties_ids::temp_GET:
			m_property_id = Properties::get_temp;
			break;
		case Mower_event_ids::properties_ids::voltage_GET:
			m_property_id = Properties::get_voltage;
			break;
	}
}

Event_property_set::Event_property_set( Properties property_id, unsigned value )
	: m_property_id{ property_id }, m_value{ value } { }

Event_property_set::Event_property_set( std::istream& is )
	: m_property_id{ Properties::unknow }, m_value{ 0 }
{
	unsigned property{ 0 };
	is >> property;
	is >> m_value;

	switch ( property ){
		case Mower_event_ids::properties_ids::blade_height:
			m_property_id = Properties::blade_height;
			break;
		case Mower_event_ids::properties_ids::blade_run:
			m_property_id = Properties::blade_run;
			break;
	}
}

void Event_server_response::serialize( std::stringstream& ss )
{
	int response{ 0 };

	switch ( m_response ) {
		case Responses::ok:
			response = Mower_event_ids::server_response_id::ok;
			break;
		case Responses::fatal_error:
			response = Mower_event_ids::server_response_id::fatal_error;
			break;
		case Responses::command_unknow:
			response = Mower_event_ids::server_response_id::command_unknow;
			break;
		case Responses::property_return:
			response = Mower_event_ids::server_response_id::property_return;
			break;
		case Responses::property_unknow:
			response = Mower_event_ids::server_response_id::property_unknow;
			break;
	}

	ss << response;
	if ( !m_data.empty() )
		ss << " " << m_data;
}

Event_manager::Event_manager()
	try : m_run{ true },
	m_dispatch_thread( std::bind( &Event_manager::dispatch_event, this ) )
{
} catch ( std::exception& e ) {
	std::string msg( tag );
	msg += e.what();
	throw std::runtime_error( msg );
}

Event_manager::~Event_manager()
{
	m_run = false; // Set this to false cause dispatch event thread to stop.
	if ( m_dispatch_thread.joinable() ) {
		m_cv.notify_all();
		m_dispatch_thread.join();
	}
}

void Event_manager::add_receiver( unsigned event, Event_handler event_handler )
{
	Event_listener_lst& event_listener_lst = m_receivers[ event ];

	event_listener_lst.push_back( event_handler );
}

void Event_manager::send_event( std::shared_ptr< IEvent > event )
{
	m_events.push( event );
	m_cv.notify_one();
}

void Event_manager::dispatch_event()
{
	while ( m_run ) {
		std::unique_lock< std::mutex > lck{ m_mutex };
		m_cv.wait( lck, [&](){ return !m_events.empty() || !m_run; } );
		if ( !m_events.empty() ) {
			auto event = m_events.front();
			m_events.pop();
			auto findIt = m_receivers.find( event->get_event_id() );

			if ( findIt != m_receivers.end() ) {
				const Event_listener_lst& event_listener_lst = findIt->second;
				for ( Event_listener_lst::const_iterator it = event_listener_lst.begin();
					it != event_listener_lst.end();	++it ) {

					std::function< void( std::shared_ptr< IEvent > ) > listener = ( *it );
					listener( event ); // call the listener
				}
			}
			else
				std::cerr << "<" << LOG_ALERT << ">" << tag << "Message: "
						<< event->get_event_id() << " ignored."	<< std::endl;
		}
	}
}

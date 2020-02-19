/*
 * 	event_system.h
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

#ifndef EVENT_SYSTEM_H_
#define EVENT_SYSTEM_H_

#include <functional>
#include <sstream>
#include <list>
#include <map>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>


#include "mower_event_ids.h"
#include "local_event_ids.h"

class IEvent {
public:
	virtual ~IEvent() {}

	virtual unsigned get_event_id() const = 0;
	virtual void serialize( std::stringstream& ss ) = 0;
};

class Event_move: public IEvent {
public:
	static const unsigned id{ Mower_event_ids::general_event_ids::move };

	enum class Direction {
		forward,
		backward,
		left,
		right,
		fr,
		fl,
		br,
		bl,
		unknow
	};

	explicit Event_move( Direction direction, int strength );
	explicit Event_move( std::istream& is );

	~Event_move() { }

	unsigned get_event_id()	const { return id; }
	Direction get_direction()	{ return m_direction; }
	int get_strength() { return m_strength;	}
	void serialize( std::stringstream& ss ) {}

private:
	Direction m_direction;
	int m_strength;
};

class Event_shutdown: public IEvent {
public:
	static const unsigned id{ Mower_event_ids::general_event_ids::shutdown };

	explicit Event_shutdown() { }

	~Event_shutdown() { }

	unsigned get_event_id() const { return id; }

	void serialize( std::stringstream& ss ) { }
};

class Event_stop: public IEvent {
public:
	static const unsigned id{ Local_event_ids::stop };

	explicit Event_stop() { }

	~Event_stop() { }

	unsigned get_event_id() const { return id; }

	void serialize( std::stringstream& ss ) { }
};

class Event_property_get: public IEvent {
public:
	static const unsigned id{ Mower_event_ids::general_event_ids::property_GET };

	enum class Properties {
		get_blade_height,
		get_current,
		get_power,
		get_temp,
		get_voltage,
		unknow
	};

	explicit Event_property_get( Properties property );
	explicit Event_property_get( std::istream& is );

	~Event_property_get() { }

	unsigned get_event_id() const { return id; }
	Properties get_property_id() const { return m_property_id; }

	void serialize( std::stringstream& ss )	{ }

private:
	Properties m_property_id;
};

class Event_property_set: public IEvent {
public:
	static const unsigned id{ Mower_event_ids::general_event_ids::property_SET };

	enum class Properties {
		blade_height,
		blade_run,
		unknow
	};

	explicit Event_property_set( Properties, unsigned value );
	explicit Event_property_set( std::istream& is );

	~Event_property_set() {}

	unsigned get_event_id()	const { return id; }
	Properties get_property_id() const { return m_property_id; }
	unsigned get_value() const { return m_value; }

	void serialize( std::stringstream& os )	{ }

private:
	Properties m_property_id;
	unsigned m_value;
};

class Event_server_response: public IEvent {
public:
	static const unsigned id{ Local_event_ids::response };

	enum class Responses {
		ok,
		fatal_error,
		command_unknow,
		property_unknow,
		property_return
	};

	explicit Event_server_response( Responses response = Responses::ok )
		: m_response { response } { }

	explicit Event_server_response( Responses response, std::stringstream& data )
		: m_response { response }, m_data( data.str() ) { }

	~Event_server_response() { }

	unsigned get_event_id()	const { return id;	}

	void serialize( std::stringstream& ss );

private:
	Responses m_response;
	std::string m_data;
};

class Event_manager final {
public:
	static Event_manager& get_instance()
	{
		static Event_manager instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}

	typedef std::list< std::function< void( std::shared_ptr< IEvent > ) > > Event_listener_lst;
	typedef std::function< void( std::shared_ptr< IEvent > ) > Event_handler;

	Event_manager( const Event_manager() & ) = delete;
	Event_manager operator=( const Event_manager() & ) = delete;

	~Event_manager();

	void add_receiver( unsigned event, Event_handler event_handler );

	void send_event( std::shared_ptr< IEvent > event );

private:
	Event_manager();
	void dispatch_event();

	std::map< unsigned, Event_listener_lst > m_receivers;
	std::queue< std::shared_ptr< IEvent > > m_events;

	bool m_run; // True to run dispatch event.
	mutable std::mutex m_mutex;
	std::condition_variable m_cv; // Condition variable for signaling arrived event.
	std::thread m_dispatch_thread;
};

#endif /* EVENT_SYSTEM_H_ */

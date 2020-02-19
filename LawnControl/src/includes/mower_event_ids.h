/*
 *  mower_event_ids.h
 *
 * 	Copytight (C) 27 Οκτ 2019 Panagiotis charisopoulos
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

#ifndef MOWER_EVENT_IDS_H_
#define MOWER_EVENT_IDS_H_

// We use int values for compatibility with java.
namespace Mower_event_ids {

	// Event ids from 0000 to 10000 is for communication events.
	namespace general_event_ids {
		constexpr unsigned property_GET = 30;
		constexpr unsigned property_SET = 31;
		constexpr unsigned move = 40;
		constexpr unsigned shutdown = 10000;
	}

	namespace server_response_id {
		constexpr unsigned ok = 8;
		constexpr unsigned fatal_error = 10;
		constexpr unsigned command_unknow = 11;
		constexpr unsigned property_unknow = 12;
		constexpr unsigned property_return = 13;
	}

	namespace move_direction_ids {
		constexpr unsigned forward = 100;
		constexpr unsigned backward = 101;
		constexpr unsigned left = 102;
		constexpr unsigned right = 103;
		constexpr unsigned fr = 104;
		constexpr unsigned fl = 105;
		constexpr unsigned br = 106;
		constexpr unsigned bl = 107;
	}

	namespace properties_ids {
		constexpr unsigned blade_run = 200;
		constexpr unsigned blade_height = 201;
		constexpr unsigned camera_on = 210;
		constexpr unsigned current_GET = 220;
		constexpr unsigned voltage_GET = 221;
		constexpr unsigned power_GET = 222;
		constexpr unsigned temp_GET = 230;
	}
}
#endif /* MOWER_EVENTS_H_ */

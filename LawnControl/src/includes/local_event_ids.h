/*
 *  local_event_ids.h
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

#ifndef LOCAL_EVENT_IDS_H_
#define LOCAL_EVENT_IDS_H_

// We use int values for compatibility with java.
namespace Local_event_ids {
	// Event ids from 10001 to 32767 is for local events.
	constexpr unsigned response = 10001; // Used to send through TCP ip.
	constexpr unsigned stop		= 10010; // Used to stop all motors.
}

#endif /* LOCAL_EVENT_IDS_H_ */

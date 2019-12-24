/*
 *  Prorocol.h
 *
 *  Communication protocol description..
 *  First byte is command.
 *  If command is "Property_SET" or "Property_GET" second byte is property
 *  to set or get. The following bytes to 32 used for value/s.
 *
 *  If command is accepted from server, then it responses with "Ok" or "Ok" followed by a value
 *  if "Property_GET" committed by the client. Else it response with "Unknow_cmd" or
 *  "Unknow_property".
 *
 *  Also if server can't execute a command because of hardware error it responses with
 *  "Fatal_error".
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

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <stdint.h>

namespace LM_protocol {

	struct Message {
		uint8_t Message_length;
		uint8_t Data[];
	};

	namespace General_cmd {
    	constexpr uint8_t Ok          	  = 0x0A;
    	constexpr uint8_t Shutdown	 	  = 0x55;
    	constexpr uint8_t Unknow_cmd 	  = 0x0F;
    	constexpr uint8_t Property_SET	  = 0x70;
    	constexpr uint8_t Property_GET	  = 0x40;
    	constexpr uint8_t Unknow_property = 0x20;
    	constexpr uint8_t Fatal_error	  = 0x10;
	}

	namespace Move_cmd {
    	constexpr uint8_t Forward	= 0x71;
    	constexpr uint8_t Backward	= 0x72;
    	constexpr uint8_t Left		= 0x73;
    	constexpr uint8_t Right		= 0x74;
    	constexpr uint8_t FR		= 0x75;
    	constexpr uint8_t FL		= 0x76;
    	constexpr uint8_t BR		= 0x77;
    	constexpr uint8_t BL		= 0x78;
	}

    namespace Property {
    	constexpr uint8_t Blade_run		= 0x79;
    	constexpr uint8_t Camera_on	   	= 0x7A;
    	constexpr uint8_t Blade_height 	= 0x7B;
    }
}
#endif /* PROTOCOL_COMMANDS_H_*/

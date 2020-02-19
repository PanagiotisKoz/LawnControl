/**
 * @brief This class implements bipolar step motor Nema 4240-15A functionality.
 *
 * @author Panagiotis Charisopoulos.
 * @date October 30 2019
 *
 * @license
 * 		This program is free software: you can redistribute it and/or modify
 * 		it under the terms of the GNU General Public License as published by
 * 		the Free Software Foundation, either version 3 of the License, or
 * 		(at your option) any later version.\n\n
 * 		This program is distributed in the hope that it will be useful,
 * 		but WITHOUT ANY WARRANTY; without even the implied warranty of
 * 		MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * 		GNU General Public License for more details.\n\n
 * 		You should have received a copy of the GNU General Public License
 * 		along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * @copyright © 2019 Panagiotis Charisopoulos. All rights reserved.
 */

#ifndef STEP_MTR_H_
#define STEP_MTR_H_

#include <pigpiod_if2.h>
#include <vector>

class Step_bipolar_mtr final {
public:
	/**
	 * @brief This enumeration used to select rotation direction.
	 */
	enum class Direction {
		left, right
	};

	explicit Step_bipolar_mtr( const unsigned pin1, const unsigned pin2,
					   const unsigned pin3, const unsigned pin4,
					   const unsigned max_pps_load );
	~Step_bipolar_mtr();

	void move_steps( Direction dir, unsigned long steps );
	void abort(); // Cancels out last move_steps command.

private:
	int m_pi; // Used for pigpiod_if2 library.
	unsigned m_pps; // Holds pulses per second.
	unsigned m_pin1;
	unsigned m_pin2;
	unsigned m_pin3;
	unsigned m_pin4;

	// Holds half step pulse pattern.
	std::vector< gpioPulse_t > m_half_step_pulses;
};

#endif /* STEPMTR_H_ */

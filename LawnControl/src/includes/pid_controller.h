/**
 * @brief This class implements a closed loop PID controller.
 *
 * @author Panagiotis Charisopoulos.
 * @date January 19 2020
 * 
 * @note Default output limits for PID controller are: 0 to
 * 		 std::numeric_limits< double >::max().
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
#ifndef INCLUDES_PID_CONTROLLER_H_
#define INCLUDES_PID_CONTROLLER_H_

#include <chrono>

class PID_controller {
public:
	explicit PID_controller( float kp, float ki, float kd, double SP );
	~PID_controller() {}

	void set_target( double SP ) { m_SP = SP; } ///< Set PID setpoint (SP), after the object has been created.
	void set_out_limits( double min, double max );
	void set_tunnings( float kp, float ki, float kd );

	double calculate( double PV );

	void reset();

private:
	float m_kp, m_ki, m_kd;
	double m_min_limit, m_max_limit;
	double m_SP;

	double m_integral;
	double m_prev_error;
	// Required to calculate derivative.
	std::chrono::time_point< std::chrono::high_resolution_clock > m_prev_time;
};

#endif /* INCLUDES_PID_CONTROLLER_H_ */

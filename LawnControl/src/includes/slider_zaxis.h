/*
 * 	slider_zaxis.h
 *
 *	This class manipulates slider that is rensponsible to move cutting disk,
 *	up down through LawnMower controller V1.1 board. The stepper system is T8-Z60.
 *
 *	Copytight (C) 26 Οκτ 2019 Panagiotis charisopoulos
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

#ifndef __arm__
#error You must use this code only for RPi
#endif

#ifndef SLIDER_ZAXIS_H_
#define SLIDER_ZAXIS_H_

#include "step_bipolar_mtr.h"
#include <deque>
#include <mutex>
#include <thread>
#include <tuple>
#include <condition_variable>

class Slider_zaxis final {
public:
	constexpr static float max_distance = 58.0f; // Value is in mm.

	enum class Direction {
		up, down
	};

	Slider_zaxis( unsigned limit_swtch_pin );
	~Slider_zaxis();

	// Returns slider current position.
	float get_cur_pos() const { return m_current_pos; }

	void cancel();
	void wait_moves_finish();
	// Set slider to specific position. Takes argument value in mm. Max 70mm.
	void set_position( float Pos_mm );

	void reset();

private:
	typedef std::tuple< Direction, float > params;
	std::deque < params > m_pending_tasks;
	std::thread m_process_tasks;
	std::mutex m_pending_tasks_mutex;
	std::condition_variable m_task_pending;
    std::condition_variable m_task_completion;
    bool m_cancel_tasks;

	float m_current_pos; // Holds slider current position in mm.
	Step_bipolar_mtr m_step_mottor;

	int m_pi; // Used for pigpiod_if2 library.
	unsigned m_top_limit_swtch_pin;
	int m_callback_id; // Id for cancel callback on destroy.


	static void on_interrupt( int pi, unsigned pin,
							  unsigned level, unsigned tick, void *userdata );

	// Moves slider from current position.
	void move_distance( Direction, float distance_mm );
	void proccess_tasks();
};

#endif /* STEPPERMOTOR_H_ */

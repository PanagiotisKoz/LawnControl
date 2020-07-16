#include "includes/slider_zaxis.h"
#include "includes/global_messages.h"
#include <cmath>
#include <thread>
#include <stdexcept>
#include <functional>
#include <string>
#include <pigpiod_if2.h>

constexpr auto tag = "Slider Z axis -- ";

constexpr int In1a_pin = 5; // Pin input In1a of L6207n.
constexpr int In2a_pin = 6; // Pin input In2a of L6207n.
constexpr int In1b_pin = 19; // Pin input In1b of L6207n.
constexpr int In2b_pin = 16; // Pin input In2b of L6207n.

/*
 * Motor characteristics
 * Max pulses per second with load for motor is 2200pps.
 * We choose 2130 PPS.
 *
 * Step rotation 1.8 degrees/step, pitch = 4mm so
 * 360 degrees / 1.8 degrees= 200 s/rev (steps/rev)
 * Each rev = 4mm so 200s / 4mm = 50 s/mm (for full stepping).
 */
constexpr unsigned int PPS = 2130;

// Motor specifications.
constexpr unsigned short int Steps_per_mm = 50; // Each mm is 50 steps.
constexpr unsigned Glitch_filter = 1000;

Slider_zaxis::Slider_zaxis( unsigned limit_swtch_pin )
	: m_cancel_tasks{ false }, m_current_pos { max_distance },
	  m_step_mottor( In1a_pin, In2a_pin, In1b_pin, In2b_pin, PPS ),
	  m_top_limit_swtch_pin{ limit_swtch_pin },
	  m_callback_id{ 0 }
{
	m_pi = pigpio_start( NULL, NULL );
	if ( m_pi < 0 ) {
		std::string msg( tag );
		msg += msg_gpio_no_connects;
		throw std::runtime_error( msg );
	}

	set_mode( m_pi, m_top_limit_swtch_pin, PI_INPUT );

	set_glitch_filter( m_pi, m_top_limit_swtch_pin, Glitch_filter );
	m_callback_id = callback_ex( m_pi, m_top_limit_swtch_pin, FALLING_EDGE, on_interrupt, this );

	if ( gpio_read( m_pi, m_top_limit_swtch_pin ) )
		 m_pending_tasks.emplace_back( std::make_tuple( Direction::up, std::abs( max_distance ) ) );

	m_process_tasks = std::thread( &Slider_zaxis::proccess_tasks, this );
}

Slider_zaxis::~Slider_zaxis()
{
	m_cancel_tasks = true;
	m_task_pending.notify_all();

	if ( m_process_tasks.joinable() )
		m_process_tasks.join();

	callback_cancel( m_callback_id );
	pigpio_stop( m_pi );
}

void Slider_zaxis::reset()
{
	move_distance( Direction::up, max_distance );
	m_current_pos = max_distance;
}

void Slider_zaxis::set_position( float Pos_mm )
{
	if ( Pos_mm > max_distance )
		Pos_mm = max_distance;

	float distance = m_current_pos - Pos_mm;

	Direction dir = distance > 0 ? Direction::down : Direction::up;

    m_pending_tasks.emplace_back( std::make_tuple( dir, std::abs( distance ) ) );

    if ( dir == Direction:: down ) {
       	m_current_pos -= std::abs( distance );
    }
    else {
     	m_current_pos += std::abs( distance );
    }

    m_task_pending.notify_one();
}

void Slider_zaxis::move_distance( Direction direction, float distance_mm )
{
	if ( distance_mm == 0 )
		return;

	unsigned long int steps { 0 };
	if ( direction == Direction:: down ) {
		steps = distance_mm * Steps_per_mm;
		m_step_mottor.move_steps( Step_bipolar_mtr::Direction::left, steps );
	}
	else {
		steps = distance_mm * Steps_per_mm;
		m_step_mottor.move_steps( Step_bipolar_mtr::Direction::right, steps );
	}
}

void Slider_zaxis::on_interrupt( int pi, unsigned pin, unsigned level, unsigned tick, void *userdata )
{
	Slider_zaxis* p = reinterpret_cast< Slider_zaxis* >( userdata );
	p->m_step_mottor.abort();
}

void Slider_zaxis::cancel()
{
    m_pending_tasks.clear();
    m_step_mottor.abort();
}

void Slider_zaxis::wait_moves_finish()
{
	std::unique_lock<std::mutex> lock( m_pending_tasks_mutex );
    // block until no pending tasks remain and all workers are idle
    m_task_completion.wait(lock, [this]() {
        return m_pending_tasks.empty();
    });
}

void Slider_zaxis::proccess_tasks()
{
	while (true) {
		std::unique_lock<std::mutex> lock( m_pending_tasks_mutex );

		// wait unless exiting or there's pending tasks
		m_task_pending.wait(lock, [this](){
           return m_cancel_tasks || m_pending_tasks.size() > 0;
		});

		if ( m_cancel_tasks ) {
            return;
        }

		Direction dir;
		float distance;
        std::tie( dir, distance ) = std::move( m_pending_tasks.front() );
        m_pending_tasks.pop_front();

        move_distance( dir, distance );

        m_task_completion.notify_one();
    }
}

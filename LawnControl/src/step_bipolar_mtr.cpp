#include "includes/step_bipolar_mtr.h"
#include "includes/global_messages.h"
#include <algorithm>
#include <string>
#include <stdexcept>
#include <chrono>


constexpr auto tag = "Step bipolar motor -- ";

constexpr unsigned Default_ppm = 2000; // Almost all bipolar steppers support this rate.

// How many milliseconds to wait until stepper current slow decays.
constexpr unsigned Hold_duration_ms = 10;

/**
 * @brief Constructor.
 * @param pin1 BCM gpio pin on coil a1.
 * @param pin2 BCM gpio pin on coil a2.
 * @param pin3 BCM gpio pin on coil b1.
 * @param pin4 BCM gpio pin on coil b2.
 * @param max_pps max pulses per second when stepper is loaded.
 */
Step_bipolar_mtr::Step_bipolar_mtr( const unsigned pin1, const unsigned pin2, const unsigned pin3,
					const unsigned pin4, const unsigned max_pps_load )
	: m_pps { max_pps_load }, m_pin1 { pin1 }, m_pin2 { pin2 }, m_pin3 { pin3 }, m_pin4 { pin4 }
{
	m_pi = pigpio_start( NULL, NULL );
	if ( m_pi < 0 ) {
		std::string msg( tag );
		msg += msg_gpio_no_connects;
		throw std::runtime_error( msg );
	}

	set_mode( m_pi, m_pin1, PI_OUTPUT );
	set_mode( m_pi, m_pin2, PI_OUTPUT );
	set_mode( m_pi, m_pin3, PI_OUTPUT );
	set_mode( m_pi, m_pin4, PI_OUTPUT );


	if ( m_pps == 0 )
		m_pps = Default_ppm;

	const unsigned pulse_width{ 1000000 / m_pps };

	//To avoid narrowing conversion warning and make code size small.
	const unsigned set_pin1{ static_cast< unsigned >( 1 ) << m_pin1 };
	const unsigned set_pin2{ static_cast< unsigned >( 1 ) << m_pin2 };
	const unsigned set_pin3{ static_cast< unsigned >( 1 ) << m_pin3 };
	const unsigned set_pin4{ static_cast< unsigned >( 1 ) << m_pin4 };

	// Pulse sequence for half step driving.
	m_half_step_pulses.push_back( { set_pin1 | set_pin4, set_pin2 | set_pin3, pulse_width } ); // 1001
	m_half_step_pulses.push_back( { set_pin4, set_pin1 | set_pin2 | set_pin3, pulse_width } ); // 0001
	m_half_step_pulses.push_back( { set_pin2 | set_pin4, set_pin1 | set_pin3, pulse_width } ); // 0101
	m_half_step_pulses.push_back( { set_pin2, set_pin1 | set_pin3 | set_pin4, pulse_width } ); // 0100
	m_half_step_pulses.push_back( { set_pin2 | set_pin3, set_pin1 | set_pin4, pulse_width } ); // 0110
	m_half_step_pulses.push_back( { set_pin3, set_pin1 | set_pin2 | set_pin4, pulse_width } ); // 0010
	m_half_step_pulses.push_back( { set_pin1 | set_pin3, set_pin2 | set_pin4, pulse_width } ); // 1010
	m_half_step_pulses.push_back( { set_pin1, set_pin2 | set_pin3 | set_pin4, pulse_width } ); // 1000
}

Step_bipolar_mtr::~Step_bipolar_mtr()
{
	abort();
	pigpio_stop( m_pi );
}

/**
 * @brief Stops every move of step motor.
 */
void Step_bipolar_mtr::abort()
{
	if( wave_tx_busy( m_pi ) ) {
		wave_tx_stop( m_pi );
	}

	wave_clear( m_pi );

	// Disable motor with slow current decay.
	gpio_write( m_pi, m_pin1, PI_HIGH );
	gpio_write( m_pi, m_pin2, PI_HIGH );
	gpio_write( m_pi, m_pin3, PI_HIGH );
	gpio_write( m_pi, m_pin4, PI_HIGH );
}

/**
 * @brief Rotate stepper motor axis by n steps in selected direction.
 * @param dir direction (left, right).
 * @param steps steps to rotate.
 */
void Step_bipolar_mtr::move_steps( Direction dir, unsigned long steps )
{
	std::vector< char > chain; // Chain to execute.

	for ( unsigned i = 0; i < m_half_step_pulses.size() ; ++i ) {
		wave_add_generic( m_pi, 1, &m_half_step_pulses[ i ] );

		chain.push_back( wave_create( m_pi ) );
	}

	if ( dir == Direction::right )
		std::reverse( chain.begin(), chain.end() );

	chain.insert( chain.begin(), { 255, 0 } ); // Add at start.  Loop start command
	char y = ( steps / 4 ) / 256;
	char x = ( steps / 4 ) % 256;
	chain.insert( chain.end(), { 255, 1, x, y } ); // Add at end. Loop x + y*256 times command.

	// Pulse to disable motor with slow current decay.
	unsigned _1{ 1 }; // To avoid narrowing conversion warning.
	gpioPulse_t disable { ( _1 << m_pin1 ) | ( _1 << m_pin2 )
						| ( _1 << m_pin3 ) | ( _1 << m_pin4 ),
						0, 5
	};

	wave_add_generic( m_pi, 1, &disable );
	chain.push_back( wave_create( m_pi ) );

	wave_chain( m_pi, &chain[ 0 ], chain.size() );

	while ( wave_tx_busy( m_pi ) ) { time_sleep(0.01); }
}

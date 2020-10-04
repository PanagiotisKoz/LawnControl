#include "includes/motor.h"

Motor::Motor()
	: m_direction { Direction::forward }, m_power { 0 }
{
}

Motor::~Motor()
{
}

void Motor::set_power( uint8_t power, Direction dir )
{
	if ( m_direction != dir ) {
		// If we change motor direction, we must disable power first.
		m_power = 0;
		apply_power();
		m_direction = dir;
	}

	if ( power > Motor::Max_power )
		power = Motor::Max_power;

	m_power = power;

	apply_power();
}

void Motor::power_off()
{
	m_power = 0;
	apply_power();
}

void Motor::increase_pwr( uint8_t how_much )
{
	set_power( m_power + how_much, m_direction );
}

void Motor::decrease_pwr( uint8_t how_much )
{
	if ( ( m_power - how_much ) < 0 )
		m_power = 0;

	set_power( m_power, m_direction );
}

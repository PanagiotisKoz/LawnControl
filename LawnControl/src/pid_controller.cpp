#include "includes/pid_controller.h"

#include <limits>
#include <algorithm>
/**
 * @breif Constructor.
 * @param kp set proportional gain.
 * @param ki set integral gain.
 * @param kd set derivative gain.
 * @param SP setpoint (SP).
 */
PID_controller::PID_controller( float kp, float ki, float kd, double SP )
	: m_kp{ kp }, m_ki{ ki }, m_kd{ kd },
	  m_min_limit{ 0 }, m_max_limit{ std::numeric_limits< double >::max() },
	  m_SP{ SP }, m_integral{ 0 }, m_prev_error{ 0 }
{
}

/**
 * @brief Set minimum and maximum output limits
 * @param min minimum output limit.
 * @param max maximum output limit.
 */
void PID_controller::set_out_limits( double min, double max )
{
	m_min_limit = min;
	m_max_limit = max;
}

/**
 * @brief Set tunings, after the object has been created.
 * @param kp set proportional gain.
 * @param ki set integral gain.
 * @param kd set derivative gain.
 */
void PID_controller::set_tunnings( float kp, float ki, float kd )
{
	m_kp = kp;
	m_ki = ki;
	m_kd = kd;
}

/**
 * @brief Calculates PID output value according an input value.
 * @param PV process variable (input value).
 * @return output value.
 */
double PID_controller::calculate( double PV )
{
	std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
	std::chrono::duration< float, std::nano > change = now - m_prev_time;

	double error = m_SP - PV;
	m_integral += error * m_ki;

	double derivative{ 0 };
	if ( change.count() != 0 )
		derivative = m_kd * ( error - m_prev_error ) / change.count();

	/* do the full calculation */
	double output = m_kp * error + m_integral + derivative;

	if ( output < m_min_limit )
		m_integral = m_min_limit;

    /* clamp output to bounds */
    output = std::min( output, m_max_limit);
	output = std::max( output, m_min_limit);

	/* required values for next round */
	m_prev_time = now;
	m_prev_error = error;

	return output;
}

/**
 * @breif Resets PID
 */
void PID_controller::reset()
{
	m_integral = 0;
	m_prev_error = 0;
}

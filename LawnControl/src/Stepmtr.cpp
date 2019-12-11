/*
 * 	Stepmtr.cpp
 *
 *	Class implementation for step motor Nema 4240-15A.
 *
 *	Copytight (C) 27 Οκτ 2019 Panagiotis charisopoulos
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

#include <Stepmtr.h>
#include <wiringPi.h>
#include <thread>
#include <chrono>

// Max pulse per second with load is 2600ppm. 2000 for silent operation.
constexpr unsigned int Pulse_rate = 2000;

constexpr int Limit_switch = 24; // Pin input lower limit switch.

Step_mtr::Step_mtr( const int pin1, const int pin2, const int pin3, const int pin4 )
		: m_pin1{ pin1 }, m_pin2{ pin2 }, m_pin3{ pin3 }, m_pin4{ pin4 }
{
	wiringPiSetupGpio();

	pinMode(m_pin1, OUTPUT);
	pinMode(m_pin2, OUTPUT);
	pinMode(m_pin3, OUTPUT);
	pinMode(m_pin4, OUTPUT);

	pinMode(Limit_switch, INPUT);

	digitalWrite(m_pin1, LOW);
	digitalWrite(m_pin2, LOW);
	digitalWrite(m_pin3, LOW);
	digitalWrite(m_pin4, LOW);
}

void Step_mtr::Hold()
{
	digitalWrite(m_pin1, HIGH);
	digitalWrite(m_pin2, HIGH);
	digitalWrite(m_pin3, HIGH);
	digitalWrite(m_pin4, HIGH);
}

void Step_mtr::Stop()
{
	digitalWrite(m_pin1, LOW);
	digitalWrite(m_pin2, LOW);
	digitalWrite(m_pin3, LOW);
	digitalWrite(m_pin4, LOW);
}

void Step_mtr::Move_steps(Direction dir, unsigned long int steps)
{
	unsigned short int pins_sequence[8];

	switch ( dir ) {
		case Direction::left:
			pins_sequence[0] = 0x08;
			pins_sequence[1] = 0x0a;
			pins_sequence[2] = 0x0e;
			pins_sequence[3] = 0x06;
			pins_sequence[4] = 0x07;
			pins_sequence[5] = 0x05;
			pins_sequence[6] = 0x0d;
			pins_sequence[7] = 0x09;
			break;
		case Direction::right:
			pins_sequence[0] = 0x09;
			pins_sequence[1] = 0x0d;
			pins_sequence[2] = 0x05;
			pins_sequence[3] = 0x07;
			pins_sequence[4] = 0x06;
			pins_sequence[5] = 0x0e;
			pins_sequence[6] = 0x0a;
			pins_sequence[7] = 0x08;
			break;
	}

	unsigned int period { 1000000 / Pulse_rate };

	unsigned short int position{ 0 };
	for ( unsigned long int i = 0; i <= steps; ++i ) {
		if ( position > 7 )
			position = 0;

		if ( dir == Direction::right && !digitalRead( Limit_switch ) )
			break;

		pins_sequence[position] & 0x08 ? digitalWrite(m_pin1, HIGH) : digitalWrite(m_pin1, LOW);
		pins_sequence[position] & 0x04 ? digitalWrite(m_pin2, HIGH) : digitalWrite(m_pin2, LOW);
		pins_sequence[position] & 0x02 ? digitalWrite(m_pin3, HIGH) : digitalWrite(m_pin3, LOW);
		pins_sequence[position] & 0x01 ? digitalWrite(m_pin4, HIGH) : digitalWrite(m_pin4, LOW);

		std::this_thread::sleep_for ( std::chrono::microseconds( period ) );
		++position;
	}

	Hold(); // Enable slow current decay.
	std::this_thread::sleep_for ( std::chrono::milliseconds( 5 ) );
	Stop(); // Brake the motor.
}

Step_mtr::~Step_mtr()
{
	Stop();
}

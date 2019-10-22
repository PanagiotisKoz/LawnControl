/*
 * 	spi.h
 *
 *	<one line to give the program's name and a brief idea of what it does.>
 *
 *	Copytight (C) 21 Οκτ 2019 Panagiotis charisopoulos
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

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>
#include <linux/spi/spidev.h>

class SPI {
public:

	enum class Channel : uint8_t {
		channel0,
		channel1
	};

	enum class Mode : uint8_t {
		mode0 = SPI_MODE_0,
		mode1 = SPI_MODE_1,
		mode2 = SPI_MODE_2,
		mode3 = SPI_MODE_3
	};

	SPI( const Channel channel = Channel::channel0, const int speed = 1000000, const Mode mode = Mode::mode3 );
	SPI( const SPI& ) = delete; // non construction-copyable
	SPI& operator=( const SPI& ) = delete; // non copyable
	virtual ~SPI();

	void Set_speed( const int speed );
	void Set_mode( const Mode mode );

	void Write(const uint8_t reg, uint8_t value);

private:
	int m_spi_fd;
	int	m_spi_speed;

	const uint8_t m_spi_BPW = 8; // SPI bits per word.
};

#endif /* SPI_H_ */

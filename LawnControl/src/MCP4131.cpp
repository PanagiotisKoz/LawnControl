/*
 * 	MCP4131.cpp
 *
 *	<one line to give the program's name and a brief idea of what it does.>
 *
 *	Copytight (C) 17 Οκτ 2019 Panagiotis charisopoulos
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

#include <MCP4131.h>
#include <stdexcept>
#include <string>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

MCP4131::~MCP4131()
{
	// TODO Auto-generated destructor stub
}

MCP4131::MCP4131( const Channel channel, const int speed )
				: m_spi_fd {-1}, m_spi_speed { 1000000 }, m_spi_channel {Channel::channel0}
{
	constexpr char spiDev0[] = "/dev/spidev0.0";
	constexpr char spiDev1[] = "/dev/spidev0.1";

	if ( speed > 10000000 || speed < 0 )
		m_spi_speed = 10000000;
	else
		m_spi_speed = speed;

	if ( (m_spi_fd = ::open (channel == Channel::channel0 ? spiDev0 : spiDev1, O_RDWR)) < 0 ) {
		std::string msg { strerror( errno ) };
		msg.insert(0, "MCP4131: failed to open spi bus. Because: " );
		throw std::runtime_error { msg };
	}

	const uint8_t mode { SPI_MODE_3 };

	if ( ioctl (m_spi_fd, SPI_IOC_WR_MODE, &mode) < 0 ) {
		std::string msg { strerror( errno ) };
		msg.insert(0, "MCP4131: SPI Mode Change failure. Because: " );
		throw std::runtime_error { msg };
	}

	if ( ioctl (m_spi_fd, SPI_IOC_WR_BITS_PER_WORD, &m_spi_BPW) < 0 ) {
		std::string msg { strerror( errno ) };
		msg.insert(0, "MCP4131: SPI BPW Change failure. Because: " );
		throw std::runtime_error { msg };
	}

	if ( ioctl (m_spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &m_spi_speed) < 0 ) {
		std::string msg { strerror( errno ) };
		msg.insert(0, "MCP4131: SPI Speed Change failure. Because: " );
		throw std::runtime_error { msg };
	}
}

void MCP4131::write(const uint8_t reg, uint8_t value)
{
	struct spi_ioc_transfer spi ;

	uint8_t cmd[] { reg, value };

	memset (&spi, 0, sizeof (spi)) ;

	spi.tx_buf        = (unsigned long)cmd;
	spi.rx_buf        = (unsigned long)cmd;
	spi.len           = 2;
	spi.delay_usecs   = 0;
	spi.speed_hz      = m_spi_speed;
	spi.bits_per_word = m_spi_BPW;

	if ( ioctl ( m_spi_fd, SPI_IOC_MESSAGE(1), &spi) < 0 ) {
		std::string msg { strerror( errno ) };
		msg.insert(0, "MCP4131: SPI write failure. Because: " );
		throw std::runtime_error { msg };
	}
}

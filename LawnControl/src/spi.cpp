#include "includes/spi.h"
#include <stdexcept>
#include <string>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

constexpr auto tag = "MCP4131 -- ";

constexpr char spiDev0[] = "/dev/spidev0.0";
constexpr char spiDev1[] = "/dev/spidev0.1";

SPI::SPI( const Channel channel, const int speed, const Mode mode )
	: m_spi_fd { -1 }, m_spi_speed { speed }
{

	std::string msg( tag );

	if ( ( m_spi_fd = ::open( channel == Channel::channel0 ? spiDev0 : spiDev1, O_RDWR ) ) < 0 ) {
		msg += "Failed to open spi bus. Because: " + std::string( strerror( errno ) );
		throw std::runtime_error { msg };
	}

	const uint8_t tmp_mode { static_cast< uint8_t >( mode ) };

	if ( ioctl( m_spi_fd, SPI_IOC_WR_MODE, &tmp_mode ) < 0 ) {
		msg +=  "SPI Mode Change failure. Because: " + std::string( strerror( errno ) );
		throw std::runtime_error { msg };
	}

	if ( ioctl( m_spi_fd, SPI_IOC_WR_BITS_PER_WORD, &m_spi_BPW ) < 0 ) {
		msg +=  "SPI BPW Change failure. Because: " + std::string( strerror( errno ) );
		throw std::runtime_error { msg };
	}

	if ( ioctl( m_spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &m_spi_speed ) < 0 ) {
		msg +=  "SPI Speed Change failure. Because: " + std::string( strerror( errno ) );
		throw std::runtime_error { msg };
	}
}

SPI::~SPI()
{
	::close( m_spi_fd );
}

void SPI::write( const uint8_t reg, uint8_t value )
{
	struct spi_ioc_transfer spi;

	uint8_t cmd[] { reg, value };

	memset( &spi, 0, sizeof( spi ) );

	spi.tx_buf = ( unsigned long ) cmd;
	spi.rx_buf = ( unsigned long ) cmd;
	spi.len = 2;
	spi.delay_usecs = 0;
	spi.speed_hz = m_spi_speed;
	spi.bits_per_word = m_spi_BPW;

	if ( ioctl( m_spi_fd, SPI_IOC_MESSAGE( 1 ), &spi ) < 0 ) {
		std::string msg( tag );
		msg +=  "SPI write failure. Because: " + std::string( strerror( errno ) );
		throw std::runtime_error { msg };
	}
}

void SPI::set_speed( const int speed )
{
	m_spi_speed = speed;

	if ( ioctl( m_spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &m_spi_speed ) < 0 ) {
		std::string msg( tag );
		msg +=  "SPI Speed Change failure. Because: " + std::string( strerror( errno ) );
		throw std::runtime_error { msg };
	}
}

void SPI::set_mode( const Mode mode )
{
	const uint8_t tmp_mode { static_cast< uint8_t >( mode ) };

	if ( ioctl( m_spi_fd, SPI_IOC_WR_MODE, &tmp_mode ) < 0 ) {
		std::string msg( tag );
		msg +=  "SPI Mode Change failure. Because: " + std::string( strerror( errno ) );
		throw std::runtime_error { msg };
	}
}

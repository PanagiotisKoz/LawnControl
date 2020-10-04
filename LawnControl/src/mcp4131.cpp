#include "includes/mcp4131.h"

void MCP4131::set_pot( uint8_t value )
{
	m_SPI.write( static_cast< uint8_t >( Reg::wiper ), value );
}

void MCP4131::set_pot_min()
{
	m_SPI.write( static_cast< uint8_t >( Reg::wiper ), Pot_fixed_values::pot_min );
}

void MCP4131::set_pot_max()
{
	m_SPI.write( static_cast< uint8_t >( Reg::wiper ), Pot_fixed_values::pot_max );
}

void MCP4131::set_pot_middle()
{
	m_SPI.write( static_cast< uint8_t >( Reg::wiper ), Pot_fixed_values::pot_mid );
}

void MCP4131::enable()
{
	m_SPI.write( static_cast< uint8_t >( Reg::tcon ), static_cast< uint8_t >( Pot::enable ) );
	m_enabled = true;
}

void MCP4131::disable()
{
	m_SPI.write( static_cast< uint8_t >( Reg::tcon ), static_cast< uint8_t >( Pot::disable ) );
	m_enabled = false;
}

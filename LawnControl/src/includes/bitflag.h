/**
 * @brief Simple implementation of a type safe bitflag.
 *
 * This class provide interface to use enum class as bitflag.
 * \example: The following example demonstrate use.
 * \code{.cpp}
 * #include "bitflag.h"
 * #include <iostream>
 * enum class foo : unsigned char {
 * 				bit0 = 1,
 * 				bit1 = 1 << 1,
 * 				bit2 = 1 << 2,
 * 				bit3 = 1 << 3,
 * 				bit4 = 1 << 4,
 * 				bit5 = 1 << 5,
 * 				bit6 = 1 << 6,
 * 				bit7 = 1 << 7
 * }
 *
 * int main()
 * {
 * 		Bitflag< foo > test( foo::bit0, foo::bit3, foo::bit7 );
 * 		std::cout << "Value of test:" << test.value();
 * 		test |= foo::bit2;
 * 		std::cout << "Value of test:" << test.value();
 *		test.unset( foo::bit0 );
 * 		std::cout << "Value of test:" << test.value();
 * }
 * \endcode
 *
 * @author Panagiotis Charisopoulos.
 * @date 5 Φεβ 2020
 * @version 1.0
 * @license
 * 		This program is free software: you can redistribute it and/or modify
 * 		it under the terms of the GNU General Public License as published by
 * 		the Free Software Foundation, either version 3 of the License, or
 * 		(at your option) any later version.\n\n
 * 		This program is distributed in the hope that it will be useful,
 * 		but WITHOUT ANY WARRANTY; without even the implied warranty of
 * 		MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * 		GNU General Public License for more details.\n\n
 * 		You should have received a copy of the GNU General Public License
 * 		along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * @copyright © 2019 Panagiotis Charisopoulos. All rights reserved.
 */

#pragma once

#include <type_traits>
#include <bitset>
template<typename T, typename Enable = void>
class Bitflag {};

template< typename Enum >
class Bitflag< Enum, typename std::enable_if< std::is_enum< Enum >::value >::type > {
public:
	using undelying = typename std::underlying_type<Enum>::type;
	constexpr const static int number_of_bits =
		std::numeric_limits< typename std::underlying_type< Enum >::type >::digits;

	constexpr Bitflag() : bits{ 0 } {}
	template < typename... Args >
	constexpr Bitflag( const Enum value, Args... args ) : Bitflag( args... )
	{
		set( value );
	}
	constexpr Bitflag( const Bitflag& other )
		: bits( other.bits ) {}

	constexpr Bitflag operator&( const Enum& value )
	{
		bits &=  static_cast< undelying >( value );
		return *this;
	}

	constexpr Bitflag operator|( const Enum& value )
	{
		bits |= static_cast< undelying >( value );
		return *this;
	}

	constexpr Bitflag operator^( const Enum& value )
	{
		bits ^= static_cast< undelying >( value );
		return *this;
	}	

	constexpr Bitflag operator~()
	{
		bits.flip();
		return *this;
	}

	constexpr Bitflag& operator|=( const Enum& value )
	{
		bits |= static_cast< undelying >( value );
		return *this;
	}
	constexpr Bitflag& operator&=( const Enum& value )
	{
		bits &= static_cast< undelying >( value );
		return *this;
	}	

	constexpr Bitflag& operator^=( const Enum& value )
	{
		bits ^= static_cast< undelying >( value );
		return *this;
	}

	constexpr bool any() const { return bits.any(); }	

	constexpr bool all() const { return bits.all();	}	

	constexpr bool none() const	{ return bits.none(); }
	constexpr operator bool() {	return any(); }

	constexpr bool test( Enum value ) const
	{
		return bits.test( static_cast< undelying >( value ) );
	}	

	constexpr void set( Enum value )
	{
		bits.set( static_cast< undelying >( value ) );
	}	

	constexpr void unset( Enum value )
	{
		bits.reset( static_cast< undelying >( value ) );
	}	

	constexpr void flip( Enum value )
	{
		bits.flip( static_cast< undelying >( value ) );
	}	

	constexpr undelying value()
	{
		return static_cast< undelying >( bits.to_ulong() );
	}

private:
	std::bitset< number_of_bits > bits;
};


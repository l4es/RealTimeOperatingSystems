/*
 ascii.h - ASCII character codes as used by cos
  
 Author:        Paul Barker
 Part of:       COS
 Created:       14/05/04
 Last Modified: 02/09/04

 Copyright (C) 2004 Paul Barker
    
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

                     (See file "Copying")
*/

/*
	This data is taken from http://www.bbsinc.com/iso8859.html
	
	We list all ASCII characters by name here for reference as well
	  as for the few places that need to use characters by name.
	This will also be useful when compiling in a different code page.
*/

#ifndef _COS_ASCII_H_
#define _COS_ASCII_H_

// control characters
#define ASCII_NUL			(0x00)
#define ASCII_SOH			(0x01)
#define ASCII_STX			(0x02)
#define ASCII_ETX			(0x03)
#define ASCII_EOT			(0x04)
#define ASCII_ENQ			(0x05)
#define ASCII_ACK			(0x06)
#define ASCII_BEL			(0x07)
#define ASCII_BS			(0x08)
#define ASCII_HT			(0x09)
#define ASCII_LF			(0x0A)
#define ASCII_VT			(0x0B)
#define ASCII_FF			(0x0C)
#define ASCII_CR			(0x0D)
#define ASCII_SO			(0x0E)
#define ASCII_SI			(0x0F)
#define ASCII_DLE			(0x10)
#define ASCII_DC1			(0x11)
#define ASCII_DC2			(0x12)
#define ASCII_DC3			(0x13)
#define ASCII_DC4			(0x14)
#define ASCII_NAK			(0x15)
#define ASCII_SYN			(0x16)
#define ASCII_ETB			(0x17)
#define ASCII_CAN			(0x18)
#define ASCII_EM			(0x19)
#define ASCII_SUB			(0x1A)
#define ASCII_ESC			(0x1B)
#define ASCII_FS			(0x1C)
#define ASCII_GS			(0x1D)
#define ASCII_RS			(0x1E)
#define ASCII_US			(0x1F)

// special characters and numbers
#define ASCII_SPACE			(0x20)
	// space
#define ASCII_EXCLAMATION		(0x21)
	// !
#define ASCII_DOUBLE_QUOTE		(0x22)
	// "
#define ASCII_NUMBER			(0x23)
	// #
#define ASCII_DOLLAR			(0x24)
	// $
#define ASCII_PERCENT			(0x25)
	// %
#define ASCII_AMPERSAND			(0x26)
	// &
#define ASCII_SINGLE_QUOTE		(0x27)
	// '
#define ASCII_LEFT_PARENTHESIS		(0x28)
	// (
#define ASCII_RIGHT_PARENTHESIS		(0x29)
	// )
#define ASCII_ASTERISK			(0x2A)
	// *
#define ASCII_PLUS			(0x2B)
	// +
#define ASCII_COMMA			(0x2C)
	// ,
#define ASCII_PERIOD			(0x2E)
	// .
#define ASCII_SLASH			(0x2F)
	// /

#define ASCII_0				(0x30)
#define ASCII_1				(0x31)
#define ASCII_2				(0x32)
#define ASCII_3				(0x33)
#define ASCII_4				(0x34)
#define ASCII_5				(0x35)
#define ASCII_6				(0x36)
#define ASCII_7				(0x37)
#define ASCII_8				(0x38)
#define ASCII_9				(0x39)

#define ASCII_COLON			(0x3A)
	// :
#define ASCII_SEMICOLON			(0x3B)
	// ;
#define ASCII_LESS_THAN			(0x3C)
	// <
#define ASCII_EQUAL			(0x3D)
	// =
#define ASCII_GREATER_THAN		(0x3E)
	// >
#define ASCII_QUESTION			(0x3F)
	// ?

// upper case letters
#define ASCII_AT			(0x40)
	// @
	
#define ASCII_A				(0x41)
#define ASCII_B				(0x42)
#define ASCII_C				(0x43)
#define ASCII_D				(0x44)
#define ASCII_E				(0x45)
#define ASCII_F				(0x46)
#define ASCII_G				(0x47)
#define ASCII_H				(0x48)
#define ASCII_I				(0x49)
#define ASCII_J				(0x4A)
#define ASCII_K				(0x4B)
#define ASCII_L				(0x4C)
#define ASCII_M				(0x4D)
#define ASCII_N				(0x4E)
#define ASCII_O				(0x4F)
#define ASCII_P				(0x50)
#define ASCII_Q				(0x51)
#define ASCII_R				(0x52)
#define ASCII_S				(0x53)
#define ASCII_T				(0x54)
#define ASCII_U				(0x55)
#define ASCII_V				(0x56)
#define ASCII_W				(0x57)
#define ASCII_X				(0x58)
#define ASCII_Y				(0x59)
#define ASCII_Z				(0x5A)

#define ASCII_LEFT_SQUARE		(0x5B)
	// [
#define ASCII_BACKSLASH			(0x5C)
	// \ 			(This text needed to stop multi-line comment)
#define ASCII_RIGHT_SQUARE		(0x5D)
	// ]
#define ASCII_CIRCUMFLEX		(0x5E)
	// ^
#define ASCII_UNDERSCORE		(0x5F)
	// _
	
// lower case letters
#define ASCII_GRAVE			(0x60)
	// `
	
#define ASCII_a				(0x61)
#define ASCII_b				(0x62)
#define ASCII_c				(0x63)
#define ASCII_d				(0x64)
#define ASCII_e				(0x65)
#define ASCII_f				(0x66)
#define ASCII_g				(0x67)
#define ASCII_h				(0x68)
#define ASCII_i				(0x69)
#define ASCII_j				(0x6A)
#define ASCII_k				(0x6B)
#define ASCII_l				(0x6C)
#define ASCII_m				(0x6D)
#define ASCII_n				(0x6E)
#define ASCII_o				(0x6F)
#define ASCII_p				(0x70)
#define ASCII_q				(0x71)
#define ASCII_r				(0x72)
#define ASCII_s				(0x73)
#define ASCII_t				(0x74)
#define ASCII_u				(0x75)
#define ASCII_v				(0x76)
#define ASCII_w				(0x77)
#define ASCII_x				(0x78)
#define ASCII_y				(0x79)
#define ASCII_z				(0x7A)

#define ASCII_LEFT_BRACE		(0x7B)
	// {
#define ASCII_VERTICAL_BAR		(0x7C)
	// | 
#define ASCII_RIGHT_BRACE		(0x7D)
	// }
#define ASCII_TILDE			(0x7E)
	// ~
	
#define ASCII_DELETE			(0x7F)

#endif // !_COS_ASCII_H_

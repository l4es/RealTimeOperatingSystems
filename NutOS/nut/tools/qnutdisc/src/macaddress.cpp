/*
 * Copyright (C) 2011 by Comm5 Tecnologia Ltda. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY COMM5 TECNOLOGIA LTDA AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL EGNITE
 * SOFTWARE GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 *
 */

#include "macaddress.h"

#include <QStringList>
#include <QDataStream>

MacAddress::MacAddress()
{
}

MacAddress::MacAddress(const MacAddress& other)
{
	data = other.data;
}

/*!
	Constructs a MacAddress from an hex, double dot representation to MacAddress.
	Example: MacAddress("00:1C:70:00:00:00");
*/
MacAddress::MacAddress( const QString& a )
{
	*this = MacAddress::fromString( a );
}

/*!
	Returns true if this is a valid MAC Address, in the [00:00:00:00:00:00, FF:FF:FF:FF:FF:FF]
*/
bool MacAddress::isValid() const
{
	return data.size() == 6;
}

QString MacAddress::toString() const
{
	return QString("%1:%2:%3:%4:%5:%6").arg((uchar)data[0],2,16,QLatin1Char('0'))
		.arg((uchar)data[1],2,16,QLatin1Char('0'))
		.arg((uchar)data[2],2,16,QLatin1Char('0'))
		.arg((uchar)data[3],2,16,QLatin1Char('0'))
		.arg((uchar)data[4],2,16,QLatin1Char('0'))
		.arg((uchar)data[5],2,16,QLatin1Char('0'));
}

QByteArray MacAddress::toByteArray() const
{
	return data;
}

QByteArray MacAddress::toByteArray( const QString& mac )
{
	return MacAddress( mac ).toByteArray();
}

/*!
	Converts an hex, double dot representation to MacAddress.
	Example: MacAddress("00:1C:70:00:00:00");
*/
MacAddress MacAddress::fromString( const QString& a )
{
	MacAddress result;
	QStringList parts = a.split(':', QString::SkipEmptyParts );
	if ( parts.size() == 6 )
	{
		for ( int i = 0; i < 6; ++i )
		{
			bool ok;
			result.data.append( parts[i].toInt( &ok, 16 ) );
			if ( !ok )
			{
				qDebug( "Invalid MAC Address" );
				return MacAddress();
			}
		}
	}
	else
		qDebug( "Invalid MAC Address" );
	return result;
}

MacAddress MacAddress::fromRawData(const QByteArray& a)
{
	if ( a.size() < 6 )
		return MacAddress();

	MacAddress result;
	for ( int i = 0; i < 6; ++i )
		result.data.append( a.at( i ) );
	return result;
}

bool MacAddress::operator ==( const MacAddress& other ) const
{
	return this->data == other.data;
}

bool MacAddress::operator <( const MacAddress& other ) const
{
	if ( !isValid() ) 
		return true;
	if ( !other.isValid() )
		return false;

	return memcmp( this->data.data(), other.data.data(), 6 ) < 0;
}

MacAddress& MacAddress::operator +=( int n )
{
	qDebug( qPrintable( this->toString() ) );

	QByteArray number;
	for ( ;n > 0; )
	{
		number.append( (n % 0xFF) );
		n /= 0xFF;
	}

	while ( number.size() < 6 )
		number.prepend( (char)0 );

	unsigned int carry = 0, accumulator;
	do 
	{
		for ( int i = 5; i >= 0; --i )
		{
			unsigned int d = static_cast<unsigned char>( data[i] );
			unsigned int n = number[i];
			accumulator = d + n + carry;
			if ( accumulator > 0xFF )
			{
				carry = accumulator / 0x100;
				accumulator %= 0x100;
			}
			else
			{
				carry = 0;
			}
			data[i] = accumulator;
		}
		if ( carry == 1 )
			carry = 0;
	} while( carry > 0 );

	return *this;
}

QDataStream &operator<<(QDataStream& ds, const MacAddress& mac)
{
	return ds << mac.toByteArray();
}

QDataStream &operator>>(QDataStream& ds, MacAddress& mac)
{
	char data[6];
	ds.readRawData( data, 6 );
	mac = MacAddress::fromRawData( QByteArray( data, 6 ) );
	return ds;
}

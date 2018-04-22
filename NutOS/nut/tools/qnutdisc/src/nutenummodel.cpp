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

#include "nutenummodel.h"
#include "macaddress.h"
#include <QDataStream>

#define DISCOVERY_PORT 9806
#define DISCOVERY_VERSION_1_0   0x10

#define DIST_REQUEST    0
#define DIST_ANNOUNCE   1
#define DIST_APPLY      2


NutEnumModel::NutEnumModel( QObject* parent ) : QAbstractTableModel( parent )
{
	connect( &socket, SIGNAL(readyRead()), this, SLOT(processPendingDatagrams()) );
	socket.bind( DISCOVERY_PORT, QUdpSocket::ShareAddress );
	broadcast();
}

int NutEnumModel::rowCount( const QModelIndex& parent /*= QModelIndex()*/ ) const
{
	Q_UNUSED( parent );
	return entries.size();
}

int NutEnumModel::columnCount ( const QModelIndex& parent /*= QModelIndex()*/ ) const
{
	Q_UNUSED( parent );
	return 5;
}

QVariant NutEnumModel::data( const QModelIndex &index, int role /*= Qt::DisplayRole*/ ) const
{

	if (!index.isValid() || index.row() >= entries.size() )
		return QVariant();

	if ( role != Qt::DisplayRole )
		return QVariant();

	NutDiscoveryData data = entries.at( index.row() );
	switch( index.column() )
	{
	case 0:
		return data.mac;
	case 1:
		return data.hostname;
	case 2:
		return data.ip_addr.toString();
	case 3:
		return data.ip_mask.toString();
	case 4:
		return data.gateway.toString();
	default:
		break;
	}
	return QVariant();
}

QVariant NutEnumModel::headerData( int section, Qt::Orientation orientation, int role /*= Qt::DisplayRole*/ ) const
{
	if ( role != Qt::DisplayRole )
		return QVariant();

	if ( orientation == Qt::Horizontal )
	{
		switch ( section )
		{
		case 0: return tr("MAC Address");
		case 1: return tr("Hostname");
		case 2: return tr("IP Address");
		case 3: return tr("Mask");
		case 4: return tr("Gateway");
		default: break;
		}
	}
	else
		return QString::number(section + 1);
	return QVariant();
}

void NutEnumModel::refresh()
{
	entries.clear();
	reset();
	broadcast();
}

QString macToString( quint8 mac[6] )
{
	return QString("%1:%2:%3:%4:%5:%6")
		.arg( (int)mac[0],2, 16, QLatin1Char('0') )
		.arg( (int)mac[1],2, 16, QLatin1Char('0') )
		.arg( (int)mac[2],2, 16, QLatin1Char('0') )
		.arg( (int)mac[3],2, 16, QLatin1Char('0') )
		.arg( (int)mac[4],2, 16, QLatin1Char('0') )
		.arg( (int)mac[5],2, 16, QLatin1Char('0') ).toUpper();
}

void NutEnumModel::broadcast()
{
	QByteArray datagram(128, 0);
	socket.writeDatagram( datagram.data(), datagram.size(), QHostAddress::Broadcast, DISCOVERY_PORT );
}

void NutEnumModel::processPendingDatagrams()
{
	QByteArray datagram;
	datagram.resize( socket.pendingDatagramSize() );
	socket.readDatagram( datagram.data(), datagram.size() );
	QDataStream stream ( datagram );
	stream.setByteOrder( QDataStream::BigEndian );
	quint32 dist_xid;
	unsigned char dist_type;
	unsigned char dist_ver;            /*!< \brief Telegram version. */
	quint8 dist_mac[6];         /*!< \brief Ethernet MAC address. */
	quint32 dist_ip_addr;        /*!< \brief Last used IP address. */
	quint32 dist_ip_mask;        /*!< \brief IP netmask. */
	quint32 dist_gateway;        /*!< \brief Default route. */
	quint32 dist_cip_addr;       /*!< \brief Configured IP address. */
	QByteArray dist_appendix( 100, 0 );

	stream >> dist_xid;
	stream >> dist_type;
	if ( dist_type != DIST_ANNOUNCE )
		return;

	stream >> dist_ver;
	stream.readRawData( (char*)dist_mac, 6 );
	stream >> dist_ip_addr;
	stream >> dist_ip_mask;
	stream >> dist_gateway;
	stream >> dist_cip_addr;
	stream.readRawData( dist_appendix.data(), 100 );

	NutDiscoveryData data;
	data.version = dist_ver;
	data.mac = macToString( dist_mac );
	data.last_ip_addr = QHostAddress( dist_ip_addr );
	data.ip_addr = QHostAddress( dist_cip_addr );
	data.ip_mask = QHostAddress( dist_ip_mask );
	data.gateway = QHostAddress( dist_gateway );
	if ( dist_ver == DISCOVERY_VERSION_1_0 )
	{
		data.hostname = QString::fromLatin1( dist_appendix.data() );
	}
	else
	{
		uint size = dist_appendix[0];
		data.hostname = QString::fromLatin1( dist_appendix.mid( 1, size ), size );
	}
	data.custom = dist_appendix.mid( data.hostname.length() );
	
	QList<NutDiscoveryData>::iterator it;
	for( it = entries.begin(); it != entries.end(); ++it )
	{
		if ( (*it).mac == data.mac )
		{
			if ( (*it) != data )
			{
				(*it) = data;
				emit dataChanged( createIndex( entries.indexOf( (*it) ), 0 ), createIndex( entries.indexOf( (*it) ), 3 ) );
			}
			return;
		}
	}

	beginInsertRows( QModelIndex(), entries.size(), entries.size() );
	entries.append( data );
	endInsertRows();

	//emit dataChanged( createIndex( 0, 0 ), createIndex( entries.size(), columnCount( QModelIndex() ) ) );
}

void NutEnumModel::changeDeviceSettings( const QString& mac, const QHostAddress& ip_addr, const QHostAddress& ip_mask, const QHostAddress& gateway )
{
	QList<NutDiscoveryData>::iterator it;
	for( it = entries.begin(); it != entries.end(); ++it )
	{
		if ( (*it).mac == mac )
		{
			QByteArray data( 128, 0 );
			QDataStream stream ( &data, QIODevice::WriteOnly );
			stream << (quint32) 1;
			stream << (quint8) DIST_APPLY;
			stream << (quint8) (*it).version;
			QByteArray d = MacAddress::toByteArray( mac );
			stream.writeRawData( d.data(), d.size() );
			stream << ( ip_addr.isNull() ? (*it).last_ip_addr.toIPv4Address() : (*it).ip_addr.toIPv4Address() );
			stream << ( ip_mask.isNull() ? (*it).ip_mask.toIPv4Address() : ip_mask.toIPv4Address() );
			stream << ( gateway.isNull() ? (*it).gateway.toIPv4Address() : gateway.toIPv4Address() );
			stream << ( ip_addr.isNull() ? (*it).ip_addr.toIPv4Address() : ip_addr.toIPv4Address() );
			if ( (*it).version == DISCOVERY_VERSION_1_0 )
			{
				int lengh = std::min( (*it).hostname.length(), 7 );
				stream.writeRawData( (*it).hostname.toLocal8Bit().data(), lengh );
			}
			else
			{
				int length = std::min( (*it).hostname.length(), 99 );
				stream << (quint8) length;
				stream.writeRawData( (*it).hostname.toLocal8Bit().data(), length );
			}
			
			socket.writeDatagram( data.data(), data.size(), QHostAddress::Broadcast, DISCOVERY_PORT );

			// update the list with our changes.
			broadcast();
			
			return;
		}
	}
}

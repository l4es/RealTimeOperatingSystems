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

#if !defined ( __NUTENUMMODEL_H__ )
#define __NUTENUMMODEL_H__

#include <QAbstractTableModel>
#include <QString>
#include <QHostAddress>
#include <QUdpSocket>

class NutEnumModel : public QAbstractTableModel
{
    Q_OBJECT
public:

    NutEnumModel( QObject* parent = 0 );

    int rowCount(const QModelIndex &parent = QModelIndex()) const;
    int columnCount ( const QModelIndex & parent = QModelIndex() ) const;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
    QVariant headerData(int section, Qt::Orientation orientation,
        int role = Qt::DisplayRole) const;


    void changeDeviceSettings( const QString& mac, const QHostAddress& ip_addr, const QHostAddress& ip_mask, const QHostAddress& gateway );

public slots:
    void refresh();

private slots:
    void processPendingDatagrams();
    void broadcast();

private:
    struct NutDiscoveryData
    {
        int version;
        QString mac;
        QHostAddress last_ip_addr;
        QHostAddress ip_mask;
        QHostAddress gateway;
        QHostAddress ip_addr;
        QString hostname;
        QString custom;

        bool operator==( const NutDiscoveryData& other )
        {
            return mac == other.mac;
        }

        bool operator!=( const NutDiscoveryData& other )
        {
            return ( mac != other.mac || ip_addr != other.ip_addr || ip_mask != other.ip_mask || gateway != other.gateway ||
                     hostname != other.hostname );
        }
    };
    QList<NutDiscoveryData> entries;
    QUdpSocket socket;
};

#endif // __NUTENUMMODEL_H__

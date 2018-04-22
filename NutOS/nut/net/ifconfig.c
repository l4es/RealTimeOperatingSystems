/*
 * Copyright (C) 2001-2004 by egnite Software GmbH
 * Copyright (c) 1993 by Digital Equipment Corporation
 * Copyright (c) 1983, 1993 by The Regents of the University of California
 *
 * All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 */

/*!
 * \file net/ifconfig.c
 * \brief Network interface configuration functions.
 *
 * \verbatim
 * $Id: ifconfig.c 5800 2014-08-27 14:30:23Z thiagocorrea $
 * \endverbatim
 */

#include <cfg/os.h>
#include <string.h>
#include <io.h>

#include <dev/ppp.h>

#include <net/if.h>
#include <net/ether.h>
#include <net/route.h>
#include <arpa/inet.h>
#include <netinet/ppp_fsm.h>
#include <netinet/if_ether.h>
#include <netinet/igmp.h>
#include <netinet/in.h>

#include <sys/event.h>
#include <dev/uart.h>
#include <sys/confnet.h>

#include <pro/dhcp.h>

#ifdef NUTDEBUG
#include <sys/osdebug.h>
#include <net/netdebug.h>
#endif

/*!
 * \addtogroup xgIP
 */
/*@{*/

/*
 * \brief Configure a network interface.
 *
 * Devices must have been registered by NutRegisterDevice() and CONFNET
 * cdn_mac should have a valid MAC address before calling this function
 *
 * Applications may alternatively call NutDhcpIfConfig(), which allows
 * automatic configuration by DHCP or the so called ARP method.
 *
 * \param name    Name of the device to configure.
 * \param ip_addr Specified IP address in network byte order. This must
 *                be a unique address within the Internet. If you do not
 *                directly communicate with other Internet hosts, you can
 *                use a locally assigned address.
 * \param ip_mask Specified IP network mask in network byte order.
 *                Typical Ethernet networks use 255.255.255.0, which
 *                allows up to 254 hosts.
 * \param gateway Specified IP for the default gateway in network byte order.
 *                This argument might be 0 in which case the default route
 *                will be disabled.
 *
 * \return 0 on success, -1 otherwise.
 *
 */
int NutNetStaticIfSetup(const char* name, uint32_t ip_addr, uint32_t ip_mask, uint32_t gateway)
{
    NUTDEVICE *dev;
    IFNET *nif;

    /*
     * Check if arguments are valid.
     */
    if (ip_addr == 0 || ip_mask == 0)
        return -1;

    /*
     * Check if this is a registered network device.
     */
    if ((dev = NutDeviceLookup(name)) == 0 || dev->dev_type != IFTYP_NET)
        return -1;

    /*
     * Setup Ethernet interface with MAC address.
     */
    nif = dev->dev_icb;
    if (nif->if_type == IFT_ETHER) {
        /* Check if ioctl is supported. */
        if (dev->dev_ioctl) {
            uint32_t flags;

            /* Driver has ioctl, use it. */
            dev->dev_ioctl(dev, SIOCGIFFLAGS, &flags);
            dev->dev_ioctl(dev, SIOCSIFADDR, confnet.cdn_mac);
            flags |= IFF_UP;
            dev->dev_ioctl(dev, SIOCSIFFLAGS, &flags);
        } else {
            /* No ioctl, set MAC address to start driver. */
            memcpy(nif->if_mac, confnet.cdn_mac, sizeof(nif->if_mac));
        }
    }

    nif->if_local_ip = ip_addr;

    /*
     * Add routing entries.
     */
    NutIpRouteAdd(ip_addr & ip_mask, ip_mask, 0, dev);
    if (gateway)
        NutIpRouteAdd(0, 0, gateway, dev);

    /*
     * Load confnet with current settings.
     */
    memcpy(confnet.cd_name, dev->dev_name, sizeof(confnet.cd_name));
    confnet.cdn_ip_addr = ip_addr;
    confnet.cdn_ip_mask = ip_mask;

    /*
     * Set gateway, if one was provided by the caller. Remove
     * gateway, if it's outside of our network.
     */
    if (gateway || (confnet.cdn_gateway & ip_mask) != (ip_addr & ip_mask))
        confnet.cdn_gateway = gateway;

    return 0;
}

/*!
 * \brief Network interface setup.
 *
 * \param dev     Identifies the network device to setup.
 * \param ip_addr Specified IP address in network byte order.
 * \param ip_mask Specified IP network mask in network byte order.
 * \param gateway Optional default gateway.
 *
 * \return 0 on success, -1 otherwise.
 *
 * \note Typical applications do not use this function, but call
 *       NutDhcpIfConfig() or NutNetIfConfig().
 */
int NutNetIfSetup(NUTDEVICE * dev, uint32_t ip_addr, uint32_t ip_mask, uint32_t gateway)
{
    IFNET *nif;

    nif = dev->dev_icb;

    /*
     * Use specified or previously used IP address.
     */
    if (ip_addr == 0 && (ip_addr = confnet.cdn_ip_addr) == 0)
        return -1;
    nif->if_local_ip = ip_addr;

    /*
     * Use specified or default mask.
     */
    if (ip_mask == 0) {
#ifdef __BIG_ENDIAN__
        ip_mask = 0xFFFFFF00UL;
#else
        ip_mask = 0x00FFFFFFUL;
#endif
    }
    nif->if_mask = ip_mask;

    /*
     * Add routing entries.
     */
    NutIpRouteAdd(ip_addr & ip_mask, ip_mask, 0, dev);
    if (gateway)
        NutIpRouteAdd(0, 0, gateway, dev);

    /*
     * Save configuration in EEPROM.
     */
    memcpy(confnet.cd_name, dev->dev_name, sizeof(confnet.cd_name));
    /* Never save an invalid MAC address. */
    if (!ETHER_IS_BROADCAST(nif->if_mac) && !ETHER_IS_ZERO(nif->if_mac)) {
        memcpy(confnet.cdn_mac, nif->if_mac, sizeof(nif->if_mac));
    }
    confnet.cdn_ip_addr = ip_addr;
    confnet.cdn_ip_mask = ip_mask;

    /*
     * Set gateway, if one was provided by the caller. Remove
     * gateway, if it's outside of our network.
     */
    if (gateway || (confnet.cdn_gateway & ip_mask) != (ip_addr & ip_mask))
        confnet.cdn_gateway = gateway;

    return NutNetSaveConfig();
}

/*!
 * \brief Configure a network interface.
 *
 * Devices must have been registered by NutRegisterDevice() before
 * calling this function.
 *
 * For Ethernet devices applications may alternatively call
 * NutDhcpIfConfig(), which allows automatic configuration by DHCP or
 * the so called ARP method.
 *
 * \param name    Name of the device to configure.
 * \param params  Pointer to interface specific parameters. For Ethernet
 *                interfaces this parameter may be a pointer to a buffer
 *                containing the 6 byte long MAC address. This will
 *                override the MAC address stored in the non-volatile
 *                configuration memory. If this memory is uninitialized
 *                or not available, the MAC address must be specified.
 *                For PPP client interfaces this parameter is NULL,
 *                while PPP server interfaces expect a pointer to a
 *                \ref PPPSERVER_CFG structure containing the server
 *                configuration.
 * \param ip_addr Specified IP address in network byte order. This must
 *                be a unique address within the Internet. If you do not
 *                directly communicate with other Internet hosts, you can
 *                use a locally assigned address. With PPP interfaces this
 *                may be set to 0.0.0.0, in which case the remote peer
 *                will be queried for an IP address.
 * \param ip_mask Specified IP network mask in network byte order.
 *                Typical Ethernet networks use 255.255.255.0, which
 *                allows up to 254 hosts. For PPP interfaces 255.255.255.255
 *                is the default.
 *
 * \return 0 on success, -1 otherwise.
 *
 * \note The whole interface configuration has become a mess over
 *       the years and need a major redesign.
 */
int NutNetIfConfig(const char *name, void *params, uint32_t ip_addr, uint32_t ip_mask)
{
    return NutNetIfConfig2(name, params, ip_addr, ip_mask, 0);
}

/*!
 * \brief Configure a network interface including the default gateway.
 *
 * Devices must have been registered by NutRegisterDevice() before
 * calling this function.
 *
 * For Ethernet devices applications may alternatively call
 * NutDhcpIfConfig(), which allows automatic configuration by DHCP or
 * the so called ARP method.
 *
 * \param name    Name of the device to configure.
 * \param params  Pointer to interface specific parameters. For Ethernet
 *                interfaces this parameter may be a pointer to a buffer
 *                containing the 6 byte long MAC address. This will
 *                override the MAC address stored in the non-volatile
 *                configuration memory. If this memory is uninitialized
 *                or not available, the MAC address must be specified.
 *                For PPP client interfaces this parameter is NULL,
 *                while PPP server interfaces expect a pointer to a
 *                \ref PPPSERVER_CFG structure containing the server
 *                configuration.
 * \param ip_addr Specified IP address in network byte order. This must
 *                be a unique address within the Internet. If you do not
 *                directly communicate with other Internet hosts, you can
 *                use a locally assigned address. With PPP interfaces this
 *                may be set to 0.0.0.0, in which case the remote peer
 *                will be queried for an IP address.
 * \param ip_mask Specified IP network mask in network byte order.
 *                Typical Ethernet networks use 255.255.255.0, which
 *                allows up to 254 hosts. For PPP interfaces 255.255.255.255
 *                is the default.
 * \param gateway Specified IP address of gateway or next router in LAN.
 *
 * \return 0 on success, -1 otherwise.
 *
 * \note I do not like this function, because setting a gateway should
 *       be handled by NutIpRouteAdd(). It's not yet deprecated, but I
 *       recommend not to use it in application code.
 */
int NutNetIfConfig2(const char *name, void *params, uint32_t ip_addr, uint32_t ip_mask, uint32_t gateway)
{
    NUTDEVICE *dev;
    IFNET *nif;

    /*
     * Check if this is a registered network device.
     */
    if ((dev = NutDeviceLookup(name)) == 0 || dev->dev_type != IFTYP_NET)
        return -1;

    /*
     * Setup Ethernet interfaces.
     */
    nif = dev->dev_icb;
    if (nif->if_type == IFT_ETHER) {
        /* Check if ioctl is supported. */
        if (dev->dev_ioctl) {
            uint32_t flags;

            /* Driver has ioctl, use it. */
            dev->dev_ioctl(dev, SIOCGIFFLAGS, &flags);
            dev->dev_ioctl(dev, SIOCSIFADDR, params);
            flags |= IFF_UP;
            dev->dev_ioctl(dev, SIOCSIFFLAGS, &flags);
        } else {
            /* No ioctl, set MAC address to start driver. */
            memcpy(nif->if_mac, params, sizeof(nif->if_mac));
        }
        return NutNetIfSetup(dev, ip_addr, ip_mask, gateway);
    }

    /*
     * Setup PPP interfaces.
     */
    else if (nif->if_type == IFT_PPP) {
        PPPDCB *dcb = dev->dev_dcb;
        PPPSERVER_CFG *ppsc = params;

        /*
         * Set the interface's IP address, make sure that the state
         * change queue is empty and switch hardware driver into
         * network mode.
         */
        dcb->dcb_local_ip = ip_addr;
        dcb->dcb_ip_mask = ip_mask ? ip_mask : 0xffffffff;
        if (ppsc) {
            dcb->dcb_remote_ip = ppsc->ppsc_remote_ip;
        }
        NutEventBroadcast(&dcb->dcb_state_chg);
        _ioctl(dcb->dcb_fd, HDLC_SETIFNET, &dev);

        /*
         * Wait for network layer up and configure the interface on
         * success.
         */
        if (NutEventWait(&dcb->dcb_state_chg, 60000) == 0 && dcb->dcb_ipcp_state == PPPS_OPENED) {
            return NutNetIfSetup(dev, dcb->dcb_local_ip, dcb->dcb_ip_mask, dcb->dcb_remote_ip);
        }
    }
    return -1;
}

int NutNetIfAddMcastAddr(const char *name, uint32_t ip_addr)
{
    NUTDEVICE *dev;
    IFNET *nif;
    int rc = -1;

    /*
     * Check if this is a registered network device.
     */
    if ((dev = NutDeviceLookup(name)) == 0 || dev->dev_type != IFTYP_NET)
        return -1;

    /*
     * Setup multicast address
     */
    nif = dev->dev_icb;
    if (nif->if_type == IFT_ETHER) {
        /* Check if ioctl is supported. */
        if (dev->dev_ioctl) {
            /* Driver has ioctl, use it. */
            rc = dev->dev_ioctl(dev, SIOCADDMULTI, &ip_addr);
            if ((rc == 0) && (ip_addr != INADDR_ALLHOSTS_GROUP)) {
                NutIgmpJoinGroup(dev, ip_addr);
            }
        }
    }

    return rc;
}

int NutNetIfDelMcastAddr(const char *name, uint32_t ip_addr)
{
    NUTDEVICE *dev;
    IFNET *nif;
    int rc = -1;

    /*
     * Check if this is a registered network device.
     */
    if ((dev = NutDeviceLookup(name)) == 0 || dev->dev_type != IFTYP_NET)
        return -1;

    /*
     * Setup multicast address
     */
    nif = dev->dev_icb;
    if (nif->if_type == IFT_ETHER) {
        /* Check if ioctl is supported. */
        if (dev->dev_ioctl) {
            /* Driver has ioctl, use it. */
            rc = dev->dev_ioctl(dev, SIOCDELMULTI, &ip_addr);
            if ((rc == 0) && (ip_addr != INADDR_ALLHOSTS_GROUP)) {
                NutIgmpLeaveGroup(dev, ip_addr);
            }
        }
    }

    return rc;
}



/*@}*/

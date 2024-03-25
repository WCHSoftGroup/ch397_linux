/*
 * USB ethernet driver for USB2.0 to 100Mbps ethernet chip ch397.
 *
 * Copyright (C) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Web: http://wch.cn
 * Author: WCH <tech@wch.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Update Log:
 * V1.0 - initial version
 * V1.1 - add support for kernel version beyond 2.6.33
 * V1.2 - add usb packet protocol length judgment
 *      - add support for VLAN network
 *      - add support for ch396, ch339, ch336
 * V1.3 - add support for frame in multiple usb packets
 */

#define DEBUG
#define VERBOSE

#undef DEBUG
#undef VERBOSE

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/usb/usbnet.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/if_vlan.h>

#define DRIVER_AUTHOR "WCH"
#define DRIVER_DESC   "USB ethernet driver for ch397, etc."
#define VERSION_DESC  "V1.3 On 2024.03"

/* control requests */
#define CH397_USB_GET_INFO 0x10
#define CH397_USB_RD_REG   0x11
#define CH397_USB_WR_REG   0x12
#define CH397_USB_RD_OTP   0x13
#define CH397_USB_WR_OTP   0x14
#define CH397_USB_RD_PHY   0x15
#define CH397_USB_WR_PHY   0x16
#define CH397_RESET_CMD	   0x18

#define CH397_MCAST_SIZE 12
#define CH397_MAX_MCAST	 12

#define CH397_TX_OVERHEAD 8
#define CH397_RX_OVERHEAD 8

#define CH397_ETH_MAC_CFG 0x40000700
#define CH397_ETH_MAC_H	  0x40000710
#define CH397_ETH_MAC_L	  0x40000714
#define CH397_ETH_BMSR	  0x40000740

#define CH397_LINK_STATUS (1 << 6)

#define CH397_MEDIUM_PS (1 << 7)
#define CH397_MEDIUM_FD (1 << 8)

#define ETH_HEADER_SIZE 14 /* size of ethernet header */

#define ETH_MIN_DATA_SIZE   46 /* minimum eth data size */
#define ETH_MIN_PACKET_SIZE (ETH_HEADER_SIZE + ETH_MIN_DATA_SIZE)

#define ETH_DEF_DATA_SIZE   1500 /* default data size */
#define ETH_DEF_PACKET_SIZE (ETH_HEADER_SIZE + ETH_DEF_DATA_SIZE)

struct ch397_int_data {
	u8 link;
	__le16 res1;
	__le16 res2;
	u8 status;
	__le16 res3;
} __packed;

typedef enum {
	STATE_S1 = 0,
	STATE_S2,
	STATE_S3,
} StateType;

struct ch397_rx_fixup_info {
	struct sk_buff *ch397_skb;
	u32 header;
	u32 remaining;
	u32 remaining_pad;
	u32 size;
	bool split_head;
	u8 len[8];
	u8 padlen;
	StateType state;
};

struct ch397_common_private {
	struct ch397_rx_fixup_info rx_fixup_info;
};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0))

static int __usbnet_read_cmd(struct usbnet *dev, u8 cmd, u8 reqtype, u16 value, u16 index, void *data, u16 size)
{
	void *buf = NULL;
	int err = -ENOMEM;

	netdev_dbg(dev->net,
		   "usbnet_read_cmd cmd=0x%02x reqtype=%02x"
		   " value=0x%04x index=0x%04x size=%d\n",
		   cmd, reqtype, value, index, size);

	if (data) {
		buf = kmalloc(size, GFP_KERNEL);
		if (!buf)
			goto out;
	}

	err = usb_control_msg(dev->udev, usb_rcvctrlpipe(dev->udev, 0), cmd, reqtype, value, index, buf, size,
			      USB_CTRL_GET_TIMEOUT);
	if (err > 0 && err <= size)
		memcpy(data, buf, err);
	kfree(buf);
out:
	return err;
}

static int __usbnet_write_cmd(struct usbnet *dev, u8 cmd, u8 reqtype, u16 value, u16 index, const void *data, u16 size)
{
	void *buf = NULL;
	int err = -ENOMEM;

	netdev_dbg(dev->net,
		   "usbnet_write_cmd cmd=0x%02x reqtype=%02x"
		   " value=0x%04x index=0x%04x size=%d\n",
		   cmd, reqtype, value, index, size);

	if (data) {
		buf = kmemdup(data, size, GFP_KERNEL);
		if (!buf)
			goto out;
	}

	err = usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0), cmd, reqtype, value, index, buf, size,
			      USB_CTRL_SET_TIMEOUT);
	kfree(buf);

out:
	return err;
}

/*
 * The function can't be called inside suspend/resume callback,
 * otherwise deadlock will be caused.
 */
int usbnet_read_cmd(struct usbnet *dev, u8 cmd, u8 reqtype, u16 value, u16 index, void *data, u16 size)
{
	int ret;

	if (usb_autopm_get_interface(dev->intf) < 0)
		return -ENODEV;
	ret = __usbnet_read_cmd(dev, cmd, reqtype, value, index, data, size);
	usb_autopm_put_interface(dev->intf);
	return ret;
}

/*
 * The function can't be called inside suspend/resume callback,
 * otherwise deadlock will be caused.
 */
int usbnet_write_cmd(struct usbnet *dev, u8 cmd, u8 reqtype, u16 value, u16 index, const void *data, u16 size)
{
	int ret;

	if (usb_autopm_get_interface(dev->intf) < 0)
		return -ENODEV;
	ret = __usbnet_write_cmd(dev, cmd, reqtype, value, index, data, size);
	usb_autopm_put_interface(dev->intf);
	return ret;
}

#endif

static int ch397_read(struct usbnet *dev, u8 cmd, u32 reg, u16 length, void *data)
{
	int err;
	u16 value = (u16)(reg & 0xFFFF);
	u16 index = (u16)((reg >> 16) & 0xFFFF);

	err = usbnet_read_cmd(dev, cmd, USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE, value, index, data, length);
	if (err != length && err >= 0)
		err = -EINVAL;

	return err;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
static int ch397_write(struct usbnet *dev, u8 cmd, u32 reg, u16 length, const void *data)
#else
static int ch397_write(struct usbnet *dev, u8 cmd, u32 reg, u16 length, void *data)
#endif
{
	int err;
	u16 value = (u16)(reg & 0xFFFF);
	u16 index = (u16)((reg >> 16) & 0xFFFF);

	err = usbnet_write_cmd(dev, cmd, USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE, value, index, data, length);
	if (err >= 0 && err < length)
		err = -EINVAL;

	return err;
}

static int ch397_read_shared_word(struct usbnet *dev, int phy, u8 reg, __le16 *value)
{
	int err;

	err = ch397_read(dev, CH397_USB_RD_PHY, CH397_ETH_BMSR | (reg << 16), 2, value);
	if (err < 0) {
		printk(KERN_ERR "Error getting link status.\n");
		return err;
	}

	return err;
}

static int ch397_mdio_read(struct net_device *net, int phy_id, int loc)
{
	struct usbnet *dev = netdev_priv(net);
	__le16 res;

	if (phy_id) {
		netdev_dbg(dev->net, "Only internal phy supported\n");
		return 0;
	}

	ch397_read_shared_word(dev, 1, loc, &res);

	netdev_dbg(dev->net, "ch397_mdio_read() phy_id=0x%02x, loc=0x%02x, returns=0x%04x\n", phy_id, loc,
		   le16_to_cpu(res));

	return le16_to_cpu(res);
}

static void ch397_mdio_write(struct net_device *net, int phy_id, int loc, int val)
{
	struct usbnet *dev = netdev_priv(net);

	if (phy_id) {
		netdev_dbg(dev->net, "Only internal phy supported\n");
		return;
	}

	netdev_dbg(dev->net, "ch397_mdio_write() phy_id=0x%02x, loc=0x%02x, val=0x%04x\n", phy_id, loc, val);
}

static void ch397_get_drvinfo(struct net_device *net, struct ethtool_drvinfo *info)
{
	/* Inherit standard device info */
	usbnet_get_drvinfo(net, info);
}

static u32 ch397_get_link(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);

	return mii_link_ok(&dev->mii);
}

static int ch397_ioctl(struct net_device *net, struct ifreq *rq, int cmd)
{
	struct usbnet *dev = netdev_priv(net);

	return generic_mii_ioctl(&dev->mii, if_mii(rq), cmd, NULL);
}

static const struct ethtool_ops ch397_ethtool_ops = {
	.get_drvinfo = ch397_get_drvinfo,
	.get_link = ch397_get_link,
	.get_msglevel = usbnet_get_msglevel,
	.set_msglevel = usbnet_set_msglevel,
	.nway_reset = usbnet_nway_reset,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 13, 0))
	.get_link_ksettings = usbnet_get_link_ksettings_mii,
	.set_link_ksettings = usbnet_set_link_ksettings_mii,
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 12, 0))
	.get_link_ksettings = usbnet_get_link_ksettings,
	.set_link_ksettings = usbnet_set_link_ksettings,
#else
	.get_settings = usbnet_get_settings,
	.set_settings = usbnet_set_settings,
#endif
};

static int ch397_get_mac_address(struct usbnet *dev, void *mac_addr)
{
	int err;
	u8 buf[4];

	memset(buf, 0, sizeof(buf));
	err = ch397_read(dev, CH397_USB_RD_REG, CH397_ETH_MAC_L, 4, buf);
	if (err < 0) {
		netdev_err(dev->net, "Error getting MAC address.\n");
		return err;
	}
	memcpy(mac_addr, buf, 4);

	memset(buf, 0, sizeof(buf));
	err = ch397_read(dev, CH397_USB_RD_REG, CH397_ETH_MAC_H, 2, buf);
	if (err < 0) {
		netdev_err(dev->net, "Error getting MAC address.\n");
		return err;
	}

	memcpy(mac_addr + 4, buf, 2);

	return 0;
}

static void __ch397_set_mac_address(struct usbnet *dev)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
	const u8 *dev_addr = dev->net->dev_addr;
#else
	u8 *dev_addr = dev->net->dev_addr;
#endif

	ch397_write(dev, CH397_USB_WR_REG, CH397_ETH_MAC_L, 4, dev_addr);
	ch397_write(dev, CH397_USB_WR_REG, CH397_ETH_MAC_H, 2, dev_addr + 4);
}

static int ch397_set_mac_address(struct net_device *net, void *p)
{
	struct sockaddr *addr = p;
	struct usbnet *dev = netdev_priv(net);

	if (!is_valid_ether_addr(addr->sa_data)) {
		dev_err(&net->dev, "not setting invalid mac address %pM\n", addr->sa_data);
		return -EINVAL;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0))
	eth_hw_addr_set(net, addr->sa_data);
#else
	memcpy(net->dev_addr, addr->sa_data, net->addr_len);
#endif
	__ch397_set_mac_address(dev);

	return 0;
}

static const struct net_device_ops ch397_netdev_ops = {
	.ndo_open = usbnet_open,
	.ndo_stop = usbnet_stop,
	.ndo_start_xmit = usbnet_start_xmit,
	.ndo_tx_timeout = usbnet_tx_timeout,
	.ndo_change_mtu = usbnet_change_mtu,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
	.ndo_get_stats64 = dev_get_tstats64,
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 12, 0))
	.ndo_get_stats64 = usbnet_get_stats64,
#endif
	.ndo_validate_addr = eth_validate_addr,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	.ndo_eth_ioctl = ch397_ioctl,
#else
	.ndo_do_ioctl = ch397_ioctl,
#endif
	.ndo_set_mac_address = ch397_set_mac_address,
};

static int ch397_bind(struct usbnet *dev, struct usb_interface *intf)
{
	int ret;
	u8 mac[ETH_ALEN];

	ret = usbnet_get_endpoints(dev, intf);
	if (ret)
		goto out;

	/* Get the MAC address */
	if (ch397_get_mac_address(dev, mac) < 0) {
		printk(KERN_ERR "Error reading MAC address\n");
		ret = -ENODEV;
		goto out;
	}

	/*
	 * Overwrite the auto-generated address only with good ones.
	 */
	if (is_valid_ether_addr(mac)) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0))
		eth_hw_addr_set(dev->net, mac);
#else
		memcpy(dev->net->dev_addr, mac, ETH_ALEN);
#endif
	} else {
		printk(KERN_WARNING "ch397: No valid MAC address in EEPROM, using %pM\n", dev->net->dev_addr);
		__ch397_set_mac_address(dev);
	}

	dev->net->netdev_ops = &ch397_netdev_ops;
	dev->net->ethtool_ops = &ch397_ethtool_ops;
	dev->rx_urb_size = 1024 * 2;

	dev->net->needed_headroom = 8;
	dev->net->needed_tailroom = 4;

	dev->mii.dev = dev->net;
	dev->mii.mdio_read = ch397_mdio_read;
	dev->mii.mdio_write = ch397_mdio_write;
	dev->mii.phy_id_mask = 0x1f;
	dev->mii.reg_num_mask = 0x1f;

	dev->driver_priv = kzalloc(sizeof(struct ch397_common_private), GFP_KERNEL);
	if (!dev->driver_priv)
		return -ENOMEM;

out:
	return ret;
}

static void ch397_unbind(struct usbnet *dev, struct usb_interface *intf)
{
	kfree(dev->driver_priv);
}

static void reset_ch397_rx_fixup_info(struct ch397_rx_fixup_info *rx)
{
	/* Reset the variables that have a lifetime outside of
	 * rx_fixup so that future processing starts from a
	 * known set of initial conditions.
	 */

	if (rx->ch397_skb) {
		/* Discard any incomplete Ethernet frame in the netdev buffer */
		kfree_skb(rx->ch397_skb);
		rx->ch397_skb = NULL;
	}

	/* Assume the Data header 32-bit word is at the start of the current
	 * or next URB socket buffer so reset all the state variables.
	 */
	rx->remaining = 0;
	rx->remaining_pad = 0;
	rx->split_head = false;
	rx->header = 0;
	rx->size = 0;
	rx->padlen = 0;
}

static int ch397_rx_fixup(struct usbnet *dev, struct sk_buff *skb)
{
	struct ethhdr *ehdr;
	struct ch397_common_private *dp = dev->driver_priv;
	struct ch397_rx_fixup_info *rx = &dp->rx_fixup_info;
	u32 offset = 0;
	u32 copy_length = 0;

	if (unlikely(skb->len < CH397_RX_OVERHEAD)) {
		dev_err(&dev->udev->dev, "unexpected tiny rx frame, len: %d\n", skb->len);
		return 0;
	}

	if (rx->state == STATE_S2) {
		u32 header;
		if (rx->remaining_pad && (rx->remaining_pad + 4 <= skb->len)) {
			offset = rx->remaining_pad;
			header = get_unaligned_le32(skb->data + offset);
			offset = 0;
			if ((header < ETH_MIN_PACKET_SIZE) || (header > ETH_DEF_PACKET_SIZE + VLAN_HLEN)) {
				netdev_err(dev->net, "%s : Bad Header Length in S2: 0x%x\n", __func__, header);
				reset_ch397_rx_fixup_info(rx);
			}
		}
	}

	while ((offset + CH397_RX_OVERHEAD) <= skb->len) {
		if (!rx->remaining) {
			if ((skb->len - offset < CH397_RX_OVERHEAD) || rx->split_head) {
				if (!rx->split_head) {
					rx->padlen = CH397_RX_OVERHEAD - (skb->len - offset);
					memcpy(rx->len, skb->data + offset, skb->len - offset);
					rx->split_head = true;
					offset = skb->len;
					netdev_dbg(dev->net, "%s : ch397 fixup 1.\n", __func__);
					break;
				} else {
					memcpy(rx->len + CH397_RX_OVERHEAD - rx->padlen, skb->data + offset,
					       rx->padlen);
					rx->header = get_unaligned_le32(rx->len);
					rx->split_head = false;
					offset += rx->padlen;
					netdev_dbg(dev->net, "%s : ch397 fixup 2, rx->header: 0x%x\n", __func__,
						   rx->header);
				}
			} else {
				rx->header = get_unaligned_le32(skb->data + offset);
				offset += CH397_RX_OVERHEAD;
			}

			/* get the packet length */
			rx->size = (rx->header + 3) & 0xFFFC;

			if ((rx->header < ETH_MIN_PACKET_SIZE) || (rx->header > ETH_DEF_PACKET_SIZE + VLAN_HLEN)) {
				netdev_err(dev->net, "%s : Bad Header Length: 0x%x\n", __func__, rx->header);
				reset_ch397_rx_fixup_info(rx);
				return 0;
			}

			if (rx->size > ((ETH_DEF_PACKET_SIZE + VLAN_HLEN + 3) & 0xFFFC)) {
				netdev_err(dev->net, "%s : Bad RX Length: 0x%x\n", __func__, rx->size);
				reset_ch397_rx_fixup_info(rx);
				return 0;
			}

			rx->ch397_skb = netdev_alloc_skb_ip_align(dev->net, rx->size);
			if (!rx->ch397_skb)
				return 0;

			rx->remaining = rx->header;
			rx->remaining_pad = rx->size;
			rx->state = STATE_S1;
		}

		if (rx->state != STATE_S2) {
			ehdr = (struct ethhdr *)(skb->data + offset);
			if (unlikely(is_multicast_ether_addr(ehdr->h_dest))) {
			} else if (unlikely(is_broadcast_ether_addr(ehdr->h_dest))) {
			} else {
				if (memcmp(ehdr->h_dest, dev->net->dev_addr, ETH_ALEN)) {
					netdev_err(dev->net, "%s : Bad dest mac addr, dest[0x%p], dev[0x%p]\n",
						   __func__, ehdr->h_dest, dev->net->dev_addr);
					return 0;
				}
			}
		}

		if (rx->remaining_pad > skb->len - offset) {
			copy_length = skb->len - offset;
			rx->remaining_pad -= copy_length;
			rx->state = STATE_S2;
			netdev_dbg(dev->net,
				   "%s : ch397 part of frame, copy_length: %d, remain_pad: %d, rx->size: %d\n",
				   __func__, copy_length, rx->remaining_pad, rx->size);
		} else {
			if (rx->state == STATE_S2) {
				copy_length = rx->remaining_pad;
				rx->state = STATE_S3;
			} else
				copy_length = rx->remaining;
			rx->remaining = 0;
		}

		if (rx->ch397_skb) {
			skb_put(rx->ch397_skb, copy_length);
			memcpy(rx->ch397_skb->data, skb->data + offset, copy_length);
			if (rx->state != STATE_S2) {
				usbnet_skb_return(dev, rx->ch397_skb);
				rx->ch397_skb = NULL;
			}
		}

		if (rx->state == STATE_S2)
			offset += copy_length;
		else if (rx->state == STATE_S1)
			offset += rx->remaining_pad;
		else
			offset += copy_length;
	}

	if (skb->len != offset) {
		netdev_err(dev->net, "%s : Bad SKB Length %d, offset: %d\n", __func__, skb->len, offset);
		reset_ch397_rx_fixup_info(rx);
		return 0;
	}

	return 1;
}

static struct sk_buff *ch397_tx_fixup(struct usbnet *dev, struct sk_buff *skb, gfp_t flags)
{
	int pad;
	int len = skb->len;
	struct ethhdr *eth = eth_hdr(skb);

	if (eth->h_proto == htons(ETH_P_8021Q))
		len = min(len, ETH_DEF_PACKET_SIZE + VLAN_HLEN);
	else
		len = min(len, ETH_DEF_PACKET_SIZE);
	if (len < ETH_MIN_PACKET_SIZE)
		len = ETH_MIN_PACKET_SIZE;

	len += CH397_TX_OVERHEAD;
	len = (len + 3) & 0xFFFFFFFC;

	if (len == 512 || len == 1024) {
		len += 4;
	}

	len -= CH397_TX_OVERHEAD;
	pad = len - skb->len;

	if (skb_headroom(skb) < CH397_TX_OVERHEAD || skb_tailroom(skb) < pad) {
		struct sk_buff *skb2;

		skb2 = skb_copy_expand(skb, CH397_TX_OVERHEAD, pad, flags);
		dev_kfree_skb_any(skb);
		skb = skb2;
		if (!skb)
			return NULL;
	}

	__skb_push(skb, CH397_TX_OVERHEAD);
	if (pad) {
		memset(skb->data + skb->len, 0, pad);
		__skb_put(skb, pad);
	}

	if (eth->h_proto == htons(ETH_P_8021Q))
		len = min(len, ETH_DEF_PACKET_SIZE + VLAN_HLEN);
	else
		len = min(len, ETH_DEF_PACKET_SIZE);
	skb->data[0] = len;
	skb->data[1] = len >> 8;
	skb->data[2] = len >> 16;
	skb->data[3] = len >> 24;

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 18, 12))
	usbnet_set_skb_tx_stats(skb, 1, 0);
#endif

	return skb;
}

static void ch397_status(struct usbnet *dev, struct urb *urb)
{
	struct ch397_int_data *event;
	int link;

	if (urb->actual_length < 8)
		return;

	event = urb->transfer_buffer;
	link = !!(event->link & CH397_LINK_STATUS);
	if (netif_carrier_ok(dev->net) != link) {
		usbnet_link_change(dev, link, 1);
		netdev_dbg(dev->net, "Link Status is: %d\n", link);
	}
}

static int ch397_link_reset(struct usbnet *dev)
{
	struct ethtool_cmd ecmd = { .cmd = ETHTOOL_GSET };
	u32 speed;
	__le32 value;
	int err;

	mii_check_media(&dev->mii, 1, 1);
	mii_ethtool_gset(&dev->mii, &ecmd);
	speed = ethtool_cmd_speed(&ecmd);

	err = ch397_read(dev, CH397_USB_RD_REG, CH397_ETH_MAC_CFG, 4, &value);
	if (err < 0) {
		netdev_err(dev->net, "Error getting mac configure value.\n");
		return err;
	}

	if (speed == SPEED_100)
		value |= CH397_MEDIUM_PS;
	else
		value &= ~CH397_MEDIUM_PS;

	if (ecmd.duplex == DUPLEX_FULL)
		value |= CH397_MEDIUM_FD;
	else
		value &= ~CH397_MEDIUM_FD;

	err = ch397_write(dev, CH397_USB_WR_REG, CH397_ETH_MAC_CFG, 4, &value);
	if (err < 0) {
		netdev_err(dev->net, "Error setting mac configure value.\n");
		return err;
	}

	netdev_dbg(dev->net, "link_reset() speed: %u duplex: %d\n", ethtool_cmd_speed(&ecmd), ecmd.duplex);

	return 0;
}

static const struct driver_info ch397_info = {
	.description = "WCH CH397 USB2.0 Ethernet",
	.flags = FLAG_ETHER | FLAG_LINK_INTR | FLAG_MULTI_PACKET,
	.bind = ch397_bind,
	.unbind = ch397_unbind,
	.rx_fixup = ch397_rx_fixup,
	.tx_fixup = ch397_tx_fixup,
	.status = ch397_status,
	.link_reset = ch397_link_reset,
	.reset = ch397_link_reset,
};

static int ch397_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);

	if (udev->actconfig->desc.bConfigurationValue != 1) {
		usb_driver_set_configuration(udev, 1);
		return -ENODEV;
	}

	return usbnet_probe(intf, id);
}

static const struct usb_device_id ch397_ids[] = {
	{
		USB_DEVICE_INTERFACE_CLASS(0x1a86, 0x5397, USB_CLASS_VENDOR_SPEC), /* ch397 chip */
		.driver_info = (unsigned long)&ch397_info,
	},

	{
		USB_DEVICE(0x1a86, 0x5397), /* ch397 chip */
		.driver_info = (unsigned long)&ch397_info,
	},

	{
		USB_DEVICE_INTERFACE_CLASS(0x1a86, 0x5396, USB_CLASS_VENDOR_SPEC), /* ch396 chip */
		.driver_info = (unsigned long)&ch397_info,
	},

	{
		USB_DEVICE(0x1a86, 0x5396), /* ch396 chip */
		.driver_info = (unsigned long)&ch397_info,
	},

	{
		USB_DEVICE_INTERFACE_CLASS(0x1a86, 0x5395, USB_CLASS_VENDOR_SPEC), /* ch339 chip */
		.driver_info = (unsigned long)&ch397_info,
	},

	{
		USB_DEVICE(0x1a86, 0x5395), /* ch339 chip */
		.driver_info = (unsigned long)&ch397_info,
	},

	{
		USB_DEVICE_INTERFACE_CLASS(0x1a86, 0x5394, USB_CLASS_VENDOR_SPEC), /* ch336 chip */
		.driver_info = (unsigned long)&ch397_info,
	},

	{
		USB_DEVICE(0x1a86, 0x5394), /* ch336 chip */
		.driver_info = (unsigned long)&ch397_info,
	},

	{},
};

MODULE_DEVICE_TABLE(usb, ch397_ids);

static struct usb_driver ch397_driver = {
	.name = "usb_ch397",
	.id_table = ch397_ids,
	.probe = ch397_probe,
	.disconnect = usbnet_disconnect,
	.suspend = usbnet_suspend,
	.resume = usbnet_resume,
	.disable_hub_initiated_lpm = 1,
};

module_usb_driver(ch397_driver);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(VERSION_DESC);
MODULE_LICENSE("GPL");

// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * USB Ethernet driver for WCH USB Ethernet controllers:
 *   - CH397: USB 2.0 to 10/100Mbps Ethernet
 *   - CH398: USB 3.0 to 10/100/1000Mbps Ethernet
 *
 * Copyright (C) 2026 Nanjing Qinheng Microelectronics Co., Ltd.
 * Web: http://wch.cn
 * Author: WCH <tech@wch.cn>
 *
 * This driver is inspired by the 6.15.1 version of net/usb/r8152.c
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
 * V1.4 - add support for multicast setting and parameters saving when autoneg off
 * V1.5 - add support for mac address filtering and fixed tx_fixup/rx_fixup
 * V1.6 - add support for rx NAPI and tx tasklet schedule
 * V1.7 - add support for WCH USB3.0 chip
 */

//#define DEBUG
//#define VERBOSE

//#define DEBUG_TX
//#define DEBUG_RX

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/mdio.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/if_vlan.h>
#include <linux/phy.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/tcp.h>
#include <linux/list.h>
#include <linux/atomic.h>
#include <linux/bitrev.h>
#include <linux/crc32.h>
#include <linux/suspend.h>
#include <uapi/linux/mdio.h>
#include <net/ip6_checksum.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 5, 0))
#include <net/gso.h>
#endif

#define DRIVER_AUTHOR "WCH"
#define DRIVER_DESC "USB ethernet driver for ch397, etc."
#define VERSION_DESC "V1.7 On 2026.04-23"
#define DESCRIPTION "WCH USB2.0/3.0 Ethernet Driver"

/* control requests */
#define CMD_GET_INFO 0x10
#define CMD_RD_REG 0x11
#define CMD_WR_REG 0x12
#define CMD_RD_OTH 0x15
#define CMD_WR_OTH 0x16
#define CMD_IC_RESET 0x18
#define CMD_WR_ETH_MACCFG 0x1E
#define CMD_WR_ETH_AUTONEG 0x1F
#define CMD_SET_MULTI_PACK 0x20
#define CMD_SET_MISC_CONFIG 0x21
#define CMD_SET_WOL_LINKSPD 0x23

#define CH397_MAX_MCAST 12
#define CH397_TX_OVERHEAD sizeof(struct ch397_tx_header)
#define CH397_RX_OVERHEAD sizeof(struct ch397_rx_header)
#define CH397_TX_ALIGN 4U
#define CH398_TX_LS_SIZE sizeof(__le32)
/* TX buffers need room for 4-byte alignment plus the CH398 HS TX_LS pad. */
#define CH397_TX_BUF_EXTRA (CH397_TX_ALIGN - 1 + CH398_TX_LS_SIZE)

#define CH397_USB_DELAY 3 /* Small delay after vendor control transfers. */
#define CH397_WORK_DELAY 100
#define CH397_CTRL_PIPE 0
#define CH397_USB_CTRL_GET_TIMEOUT 500
#define CH397_USB_CTRL_SET_TIMEOUT 500
#define CH397_RX_MAX_PENDING 4096
#define CH397_RX_COPYBREAK 256
/* percentage of total TX capacity */
#define CH398_RX_EARLY_SIZE_BYTES (12 * 1024)
/* Hardware coalesce time unit is 10us, so 20 means 200us. */
#define CH398_RX_EARLY_TIME_US (20)

#define mtu_to_size(m) ((m) + VLAN_ETH_HLEN)
#define size_to_mtu(s) ((s) - VLAN_ETH_HLEN)
#define WCH_MIN_JUMBO_SIZE (2048 - sizeof(struct ch397_tx_header))
#define WCH_MAX_TSO_SIZE \
	(dev->tx_urb_size - sizeof(struct ch397_tx_header))
#define WAKE_ALL (WAKE_PHY | WAKE_MAGIC)

/* Keep the watchdog timeout short enough to recover stalled USB TX promptly. */
#define TX_TIMEOUT_JIFFIES (5 * HZ)
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0))
#define strlcpy(dst, src, size) strscpy(dst, src, size)
#endif
#ifndef ETH_MIN_MTU
#define ETH_MIN_MTU 68
#endif
#ifndef BMCR_SPEED10
#define BMCR_SPEED10 0
#endif
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 19, 0))
#define skb_vlan_tag_present vlan_tx_tag_present
#define skb_vlan_tag_get vlan_tx_tag_get
#endif

/* ch397 reg */
#define CH397_ETH_MAC_H 0x40000710
#define CH397_ETH_MAC_L 0x40000714
#define CH397_ETH_MAC_HTHR 0x40000730
#define CH397_ETH_MAC_HTLR 0x40000734
#define CH397_ETH_BMSR 0x00000740

#define CH397_ETH_MAC_CFG 0x40000700
#define CH397_MEDIUM_PS (1 << 7)
#define CH397_MEDIUM_FD (1 << 8)
#define CH397_FLOWCTRL_EN (1 << 11)
#define CH397_TX_EN (1 << 13)
#define CH397_RX_EN (1 << 14)
#define CH397_RX_RA (1 << 19)
#define CH397_RX_PAM (1 << 20)
#define CH397_RX_PUF (1 << 21)
#define CH397_RX_PBD (1 << 22)
#define CH397_RX_PLM (1 << 23)
#define CH397_RX_RCVALL 0x00f80000

#define CH397_ETH_MACIER 0x4000073c
#define CH397_ENABLE_WOLPTN (1 << 10)
#define CH397_ENABLE_MAGIC (1 << 11)
#define CH397_ENABLE_RWTE (1 << 12)

#define CH397_PHY_PAGE_SLET 0x1F
#define CH397_PHY_PAGE0 0x0000
#define CH397_PHY_PAGE4 0x0004
#define CH397_PHY_CAP0EEE 0x10
#define CH397_PHY_EEE_10M_CAP (1 << 13)

/* ch398 reg */
#define CH398_DISABLE_MAC_EEE (1 << 0)
#define CH398_DISABLE_PHY_INTR (1 << 1)
#define CH398_ETH_MAC_H 0x40024010
#define CH398_ETH_MAC_L 0x40024014
#define CH398_ETH_MAC_HTHR 0x40024030
#define CH398_ETH_MAC_HTLR 0x40024034

#define CH398_ETH_MAC_CFG 0x40024000
#define CH398_MEDIUM_PS (1 << 7)
#define CH398_MEDIUM_FD (1 << 8)
#define CH398_FLOWCTRL_EN (1 << 11)
#define CH398_TX_EN (1 << 13)
#define CH398_RX_EN (1 << 14)
#define CH398_RX_RA (1 << 19)
#define CH398_RX_PAM (1 << 20)
#define CH398_RX_PUF (1 << 21)
#define CH398_RX_PBD (1 << 22)
#define CH398_RX_PLM (1 << 23)
#define CH398_RX_RCVALL 0x00f80000

#define CH398_ETH_MACIER 0x4002403c
#define CH398_ENABLE_MAGIC (1 << 24)
#define CH398_ENABLE_WOL (1 << 23)
#define CH398_ENABLE_MCAST (1 << 18)
#define CH398_ENABLE_BCAST (1 << 17)
#define CH398_ENABLE_UCAST (1 << 16)
#define CH398_ENABLE_WOLPTN (1 << 14)

#define CH398_ETH_MISC_CFG 0x400240e8
#define CH398_RX_STRIP_VLANEN (1 << 13)

#define CH398_PHY_SWTICH_EXTEN 0x0E
#define CH398_PHY_READ_EXTEN 0x0D
#define CH398_PHY_LOOP_STATUS_REG 0x10
#define CH398_PHY_EEE_1000M_ENABLE (1 << 1)
#define CH398_PHY_EEE_100M_ENABLE (1 << 0)
#define CH398_PHY_AUX_CTRL_REG 0x12
#define CH398_PHY_EEE_1000M_ADV (1 << 1)
#define CH398_PHY_EEE_100M_ADV (1 << 0)
#define CH398_PHY_EEE_LPA_REG 0x00DB

#define PHY_ADVERTISED_10_HALF BIT(0)
#define PHY_ADVERTISED_10_FULL BIT(1)
#define PHY_ADVERTISED_100_HALF BIT(2)
#define PHY_ADVERTISED_100_FULL BIT(3)
#define PHY_ADVERTISED_1000_HALF BIT(4)
#define PHY_ADVERTISED_1000_FULL BIT(5)
#define CH398_WOL_LINKSPD_ENABLE 0x5578
#define CH398_WOL_LINKSPD_10M 0x000A
#define CH398_WOL_LINKSPD_100M 0x0064
#define CH398_WOL_LINKSPD_1000M 0x0001

/* TX/RX resources */
#define TX_SS_URB_NUM 10
#define TX_HS_URB_NUM 10
#define TX_FS_URB_NUM 4
#define TX_SS_URB_SIZE (16 * 1024)
#define TX_HS_URB_SIZE (16 * 1024)
#define TX_FS_URB_SIZE (8 * 1024)
#define CH397_TX_URB_SIZE (4 * 1024)

#define RX_SS_URB_NUM 16
#define RX_HS_URB_NUM 10
#define RX_FS_URB_NUM 4
#define RX_SS_URB_SIZE (32 * 1024)
#define RX_HS_URB_SIZE (32 * 1024)
#define RX_FS_URB_SIZE (8 * 1024)
#define CH397_RX_URB_SIZE (16 * 1024)

struct ch39x_regs {
	u32 mac_cfg;
	u32 mac_addrh;
	u32 mac_addrl;
	u32 mac_hthr;
	u32 mac_htlr;
};

static const struct ch39x_regs ch397_regs = {
	.mac_cfg = CH397_ETH_MAC_CFG,
	.mac_addrh = CH397_ETH_MAC_H,
	.mac_addrl = CH397_ETH_MAC_L,
	.mac_hthr = CH397_ETH_MAC_HTHR,
	.mac_htlr = CH397_ETH_MAC_HTLR,
};

static const struct ch39x_regs ch398_regs = {
	.mac_cfg = CH398_ETH_MAC_CFG,
	.mac_addrh = CH398_ETH_MAC_H,
	.mac_addrl = CH398_ETH_MAC_L,
	.mac_hthr = CH398_ETH_MAC_HTHR,
	.mac_htlr = CH398_ETH_MAC_HTLR,
};

/* use ethtool to change the level for any given device */
static int msg_level = -1;

static const char ch397_gstrings[][ETH_GSTRING_LEN] = {
	"rx_packets",	 "tx_packets",	     "rx_overflow_packets",
	"rx_crc_errors", "rx_backlog_drops",
};

struct ch397;

struct ch397_rx_header {
	__le32 cmd_0;
#define RX_LEN_MASK 0xffffU
	__le32 cmd_1;
#define RX_VTAG_SHIFT 16
#define RX_VTAG_MASK 0xffffU
#define RX_VLAN_TAG BIT(15)
#define RX_UDPCS_ER BIT(9)
#define RX_TCPCS_ER BIT(8)
#define RX_IPCS_ER BIT(7)
#define RX_TCPV6_CSC BIT(3)
#define RX_TCPV4_CSC BIT(2)
#define RX_UDPV6_CSC BIT(1)
#define RX_UDPV4_CSC BIT(0)
} __packed;

struct ch397_tx_header {
	__le32 cmd_0;
#define TX_TCPHO_SHIFT 24
#define TX_TCPHO_MASK 0xffU
#define TX_GTSENDV4 BIT(23)
#define TX_GTSENDV6 BIT(22)
#define TX_LS BIT(20)
#define TX_LEN_MASK 0x3ffffU
	__le32 cmd_1;
#define TX_VTAG_SHIFT 16
#define TX_VTAG_MASK 0xffffU
#define TX_VLAN_TAG BIT(15)
#define TX_IPV4_CS BIT(13)
#define TX_UDP_CS BIT(12)
#define TX_TCP_CS BIT(11)
#define TX_MSS_MAX 0x7ffU
} __packed;

struct ch397_rx_agg {
	struct list_head list, info_list;
	struct urb *urb;
	struct ch397 *context;
	struct page *page;
	void *buffer;
};

struct ch397_tx_agg {
	struct list_head list, info_list;
	struct urb *urb;
	struct ch397 *context;
	void *buffer;
	void *head;
	u32 skb_num;
	u32 skb_len;
};

enum wch_version {
	WCH_CHIP_VER_UNKNOWN = 0,
	WCH_CHIP_VER_01,
	WCH_CHIP_VER_02
};

enum ch398_tx_csum_result {
	CH398_TX_CSUM_OK = 0,
	CH398_TX_CSUM_SW = 1,
};

enum ch397_flags {
	CH397_SET_RX_MF = 0,
	CH397_SET_RX_MODE,
	CH397_LINK_RESET,
	CH397_LINK_CHG,
	CH397_DEV_OPEN,
	CH397_DEV_UNPLUG, /* Physical USB unplug has been observed. */
	CH397_DEV_INACCESSIBLE, /* USB device is temporarily not reachable. */
	CH397_DEV_CHOOSE_SUSPEND,
	CH397_SCHEDULE_TASKLET,
	CH397_RX_EPROTO
};

struct ch397_info {
	u8 devtype;
	u8 val_h;
	u8 val_l;
	u8 sta;
	u8 rx_buf_size;
	u8 tx_buf_size;
	u8 reserved[2];
} __packed;

struct ch397_ndev_cfg {
	bool link;
	bool phy_wol;

	u16 speed; /* user configured forced speed */
	u8 duplex; /* user configured forced duplex */
	u8 autoneg;
	u32 advertising;
	u16 link_speed; /* current resolved link speed */
	u8 link_duplex; /* current resolved link duplex */
	u8 *intr_buff;

	bool eee_enabled; /* user requested enable/disable */
	bool eee_active; /* runtime resolved active state */
	u32 eee_advertised; /* local advertised EEE modes, MDIO_EEE_* bitmap */
	u32 rx_coalesce_usecs;
	u32 rx_max_coalesce_bytes;
	u8 flow_ctrl;
	bool suspend_10m;
	struct ch397_info dev_info;
};

struct ch397_intr_event {
	__le32 link_stat;
#define CH397_LINK_SPEED BIT(7)
#define CH397_LINK_RDY BIT(6)
#define CH397_DUPLEX_MODE BIT(0)
	__le32 rx_packets;
	__le16 rx_overflow_cnt;
	__le16 rx_crc_cnt;
	__le32 tx_packets;
} __packed;

struct ch398_intr_event {
	__le32 link_stat;
#define CH398_LINK_SPEED BIT(7)
#define CH398_LINK_RDY BIT(6)
#define CH398_SPEED_1G BIT(4)
#define CH398_DUPLEX_MODE BIT(0)
	__le32 rx_packets;
	__le16 rx_overflow_cnt;
	__le16 rx_crc_cnt;
	__le32 tx_packets;
	__le32 reserved_1;
	__le32 reserved_2;
	__le32 reserved_3;
	__le32 reserved_4;
} __packed;

struct ch397_intr_stats {
	u32 link_stat;
	u32 rx_packets;
	u16 rx_overflow_cnt;
	u16 rx_crc_cnt;
	u32 tx_packets;
};

struct ch39x_ops {
	int (*init)(struct ch397 *dev);
	int (*tx_fixup)(struct ch397 *dev, struct ch397_tx_agg *tx_agg);
	void (*rx_fixup)(struct ch397 *dev, struct sk_buff *skb,
			 struct ch397_rx_header *rx_header);
	void (*intr_status)(struct ch397 *dev, struct urb *urb);
	int (*set_speed)(struct ch397 *dev, u8 autoneg, u16 speed,
			 u8 duplex, u32 advertising);
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(6, 9, 0))
	int (*get_eee)(struct ch397 *dev, struct ethtool_eee *edata);
	int (*set_eee)(struct ch397 *dev, struct ethtool_eee *edata);
#else
	int (*get_eee)(struct ch397 *dev, struct ethtool_keee *edata);
	int (*set_eee)(struct ch397 *dev, struct ethtool_keee *edata);
#endif
	void (*get_wol)(struct ch397 *dev, struct ethtool_wolinfo *wol);
	int (*set_wol)(struct ch397 *dev, struct ethtool_wolinfo *wol);
	void (*get_ethtool_stats)(struct ch397 *dev, u64 *data);
	int (*set_mtu)(struct ch397 *dev, int mtu);
};

struct ch397 {
	struct usb_device *udev;
	struct net_device *ndev;
	struct mii_if_info mii;
	struct usb_interface *intf;
	struct mutex phy_mutex;
	struct mutex dev_mutex; /* serialise open/stop wrt suspend/resume */
	struct sk_buff_head rxq_pend, txq_pend;
	struct list_head rxq_info, rxq_used;
	struct list_head rxq_done, txq_info, txq_free;
	struct urb *intr_urb;
	struct delayed_work wq;
	struct napi_struct napi;
	struct tasklet_struct tx_bh;
	const struct ch39x_regs *regs;
	const struct ch39x_ops *dev_ops;
	struct ch397_ndev_cfg ndev_cfg;
	struct ch397_intr_stats ch397_intr;
	struct ch397_intr_stats ch398_intr;

	unsigned int pipe_in, pipe_out, pipe_intr;
	unsigned int pipe_ctrl_in, pipe_ctrl_out;
	unsigned int intr_interval;
	unsigned maxpacket;
	int msg_enable;
	size_t n_tx_urbs;
	size_t n_rx_urbs;
	size_t tx_urb_size;
	size_t rx_urb_size;
	bool tx_single_packet;
	u32 tx_qlen, rxq_pend_limit;
	u32 tx_qlen_scale;
	u32 max_mtu;
	spinlock_t rxq_lock, txq_lock;
	unsigned long flags;
	atomic_t rx_count;
	u32 rx_pending;
	u32 rx_copybreak;
	u64 rx_backlog_drops;
	bool hs_mode;
	u8 version;
	u8 *intr_buf;
#ifdef CONFIG_PM_SLEEP
	struct notifier_block pm_notifier;
	bool pm_notifier_held;
#endif
};

static inline struct sk_buff *ch397_napi_alloc_skb(struct ch397 *dev,
						   unsigned int length)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 19, 0))
	return netdev_alloc_skb_ip_align(dev->ndev, length);
#else
	return napi_alloc_skb(&dev->napi, length);
#endif
}

static inline bool ch397_napi_complete_done(struct napi_struct *napi,
					    int work_done)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 10, 0))
	napi_complete(napi);
	return true;
#else
	return napi_complete_done(napi, work_done);
#endif
}

static int ch397_init(struct ch397 *dev);
static int ch398_init(struct ch397 *dev);
static int ch397_tx_agg_fill(struct ch397 *dev,
			     struct ch397_tx_agg *tx_agg);
static int ch398_tx_agg_fill(struct ch397 *dev,
			     struct ch397_tx_agg *tx_agg);
static void ch397_rx_cmd_offload(struct ch397 *dev, struct sk_buff *skb,
				 struct ch397_rx_header *rx_header);
static void ch398_rx_cmd_offload(struct ch397 *dev, struct sk_buff *skb,
				 struct ch397_rx_header *rx_header);
static void ch397_intr_status(struct ch397 *dev, struct urb *urb);
static void ch398_intr_status(struct ch397 *dev, struct urb *urb);
static int ch397_set_speed(struct ch397 *dev, u8 autoneg, u16 speed,
			   u8 duplex, u32 advertising);
static int ch398_set_speed(struct ch397 *dev, u8 autoneg, u16 speed,
			   u8 duplex, u32 advertising);
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(6, 9, 0))
static int ch397_ethtool_get_eee(struct ch397 *dev,
				 struct ethtool_eee *data);
static int ch398_ethtool_get_eee(struct ch397 *dev,
				 struct ethtool_eee *data);
static int ch397_ethtool_set_eee(struct ch397 *dev,
				 struct ethtool_eee *data);
static int ch398_ethtool_set_eee(struct ch397 *dev,
				 struct ethtool_eee *data);
#else
static int ch397_ethtool_get_eee(struct ch397 *dev,
				 struct ethtool_keee *data);
static int ch398_ethtool_get_eee(struct ch397 *dev,
				 struct ethtool_keee *data);
static int ch397_ethtool_set_eee(struct ch397 *dev,
				 struct ethtool_keee *data);
static int ch398_ethtool_set_eee(struct ch397 *dev,
				 struct ethtool_keee *data);
#endif
static void ch397_ethtool_get_wol(struct ch397 *dev,
				  struct ethtool_wolinfo *wol);
static void ch398_ethtool_get_wol(struct ch397 *dev,
				  struct ethtool_wolinfo *wol);
static int ch397_ethtool_set_wol(struct ch397 *dev,
				 struct ethtool_wolinfo *wol);
static int ch398_ethtool_set_wol(struct ch397 *dev,
				 struct ethtool_wolinfo *wol);
static void ch397_intr_complete(struct urb *urb);
static void ch397_rx_complete(struct urb *urb);
static void ch397_get_stats(struct ch397 *dev, u64 *data);
static void ch398_get_stats(struct ch397 *dev, u64 *data);
static int ch398_set_mtu(struct ch397 *dev, int mtu);
static inline bool ch397_dev_unplugged(struct ch397 *dev);
static inline bool ch397_dev_inaccessible(struct ch397 *dev);
static void ch397_set_inaccessible(struct ch397 *dev);
static void ch397_set_accessible(struct ch397 *dev);
static void ch397_set_unplug(struct ch397 *dev);
static void ch397_update_usb_state(struct ch397 *dev, int status);

static const struct ch39x_ops ch397_ops = {
	.init = ch397_init,
	.tx_fixup = ch397_tx_agg_fill,
	.rx_fixup = ch397_rx_cmd_offload,
	.intr_status = ch397_intr_status,
	.set_speed = ch397_set_speed,
	.get_eee = ch397_ethtool_get_eee,
	.set_eee = ch397_ethtool_set_eee,
	.get_wol = ch397_ethtool_get_wol,
	.set_wol = ch397_ethtool_set_wol,
	.get_ethtool_stats = ch397_get_stats,
};

static const struct ch39x_ops ch398_ops = {
	.init = ch398_init,
	.tx_fixup = ch398_tx_agg_fill,
	.rx_fixup = ch398_rx_cmd_offload,
	.intr_status = ch398_intr_status,
	.set_speed = ch398_set_speed,
	.get_eee = ch398_ethtool_get_eee,
	.set_eee = ch398_ethtool_set_eee,
	.get_wol = ch398_ethtool_get_wol,
	.set_wol = ch398_ethtool_set_wol,
	.get_ethtool_stats = ch398_get_stats,
	.set_mtu = ch398_set_mtu,
};

static inline void *ch397_tx_agg_align(void *data)
{
	return (void *)ALIGN((uintptr_t)data, CH397_TX_ALIGN);
}

static void ch397_kill_tx_urb(struct ch397 *dev)
{
	struct ch397_tx_agg *tx_agg;

	/* txq_info is the stable ownership list for every TX aggregate.
	 * The runtime TX path only moves tx_agg->list on and off txq_free,
	 * so stop/free can safely enumerate txq_info here.
	 */
	list_for_each_entry(tx_agg, &dev->txq_info, info_list)
		usb_kill_urb(tx_agg->urb);
}

static void ch397_free_rx_agg(struct ch397 *dev,
			      struct ch397_rx_agg *rx_agg)
{
	list_del_init(&rx_agg->list);
	list_del_init(&rx_agg->info_list);

	usb_free_urb(rx_agg->urb);
	put_page(rx_agg->page);
	kfree(rx_agg);

	atomic_dec(&dev->rx_count);
}

static void ch397_free_tx_agg(struct ch397 *dev,
			      struct ch397_tx_agg *tx_agg)
{
	list_del_init(&tx_agg->list);
	list_del_init(&tx_agg->info_list);

	usb_free_urb(tx_agg->urb);
	kfree(tx_agg->buffer);
	kfree(tx_agg);
}

static struct ch397_rx_agg *ch397_alloc_rx_agg(struct ch397 *dev,
					       gfp_t mflags)
{
	struct net_device *ndev = dev->ndev;
	int node = ndev->dev.parent ? dev_to_node(ndev->dev.parent) : -1;
	unsigned int order = get_order(dev->rx_urb_size);
	struct ch397_rx_agg *rx_agg;
	unsigned long flags;

	rx_agg = kmalloc_node(sizeof(*rx_agg), mflags, node);
	if (!rx_agg)
		return NULL;

	rx_agg->page =
		alloc_pages(mflags | __GFP_COMP | __GFP_NOWARN, order);
	if (!rx_agg->page)
		goto free_rx;

	rx_agg->buffer = page_address(rx_agg->page);

	rx_agg->urb = usb_alloc_urb(0, mflags);
	if (!rx_agg->urb)
		goto free_buf;

	rx_agg->context = dev;

	INIT_LIST_HEAD(&rx_agg->list);
	INIT_LIST_HEAD(&rx_agg->info_list);
	spin_lock_irqsave(&dev->rxq_lock, flags);
	list_add_tail(&rx_agg->info_list, &dev->rxq_info);
	spin_unlock_irqrestore(&dev->rxq_lock, flags);

	atomic_inc(&dev->rx_count);

	return rx_agg;

free_buf:
	__free_pages(rx_agg->page, order);
free_rx:
	kfree(rx_agg);
	return NULL;
}

static void ch397_free_all_resources(struct ch397 *dev)
{
	struct ch397_rx_agg *rx_agg, *rx_agg_next;
	struct ch397_tx_agg *tx_agg, *tx_agg_next;
	unsigned long flags;

	ch397_kill_tx_urb(dev);

	spin_lock_irqsave(&dev->rxq_lock, flags);

	list_for_each_entry_safe(rx_agg, rx_agg_next, &dev->rxq_info,
				 info_list)
		ch397_free_rx_agg(dev, rx_agg);

	spin_unlock_irqrestore(&dev->rxq_lock, flags);

	WARN_ON(atomic_read(&dev->rx_count));

	list_for_each_entry_safe(tx_agg, tx_agg_next, &dev->txq_info,
				 info_list)
		ch397_free_tx_agg(dev, tx_agg);

	usb_free_urb(dev->intr_urb);
	dev->intr_urb = NULL;

	kfree(dev->intr_buf);
	dev->intr_buf = NULL;

	INIT_LIST_HEAD(&dev->rxq_done);
	INIT_LIST_HEAD(&dev->rxq_used);
	INIT_LIST_HEAD(&dev->txq_info);
	INIT_LIST_HEAD(&dev->txq_free);
}

static int ch397_alloc_all_resources(struct ch397 *dev)
{
	struct net_device *ndev = dev->ndev;
	unsigned int maxp, period;
	int node, i;
	int ret = 0;

	node = ndev->dev.parent ? dev_to_node(ndev->dev.parent) : -1;

	skb_queue_head_init(&dev->rxq_pend);
	skb_queue_head_init(&dev->txq_pend);
	spin_lock_init(&dev->rxq_lock);
	spin_lock_init(&dev->txq_lock);
	INIT_LIST_HEAD(&dev->rxq_info);
	INIT_LIST_HEAD(&dev->rxq_done);
	INIT_LIST_HEAD(&dev->rxq_used);
	INIT_LIST_HEAD(&dev->txq_info);
	INIT_LIST_HEAD(&dev->txq_free);
	atomic_set(&dev->rx_count, 0);

	for (i = 0; i < dev->n_rx_urbs; i++) {
		if (!ch397_alloc_rx_agg(dev, GFP_KERNEL)) {
			ret = -ENOMEM;
			goto error;
		}
	}

	for (i = 0; i < dev->n_tx_urbs; i++) {
		struct ch397_tx_agg *tx_agg;
		struct urb *urb;
		u8 *buf;

		tx_agg = kmalloc_node(sizeof(*tx_agg), GFP_KERNEL, node);
		if (!tx_agg) {
			ret = -ENOMEM;
			goto error;
		}

		buf = kmalloc_node(dev->tx_urb_size + CH397_TX_BUF_EXTRA,
				   GFP_KERNEL, node);
		if (!buf) {
			kfree(tx_agg);
			ret = -ENOMEM;
			goto error;
		}

		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb) {
			kfree(buf);
			kfree(tx_agg);
			ret = -ENOMEM;
			goto error;
		}

		INIT_LIST_HEAD(&tx_agg->list);
		INIT_LIST_HEAD(&tx_agg->info_list);
		tx_agg->context = dev;
		tx_agg->urb = urb;
		tx_agg->buffer = buf;
		tx_agg->head = ch397_tx_agg_align(buf);

		list_add_tail(&tx_agg->info_list, &dev->txq_info);
		list_add_tail(&tx_agg->list, &dev->txq_free);
	}

	period = dev->intr_interval;
	if (!period) {
		ret = -ENODEV;
		goto error;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 19, 0))
	maxp = usb_maxpacket(dev->udev, dev->pipe_intr);
#else
	maxp = usb_maxpacket(dev->udev, dev->pipe_intr, 0);
#endif
	if (!maxp) {
		ret = -ENODEV;
		goto error;
	}

	dev->intr_buf = kmalloc(maxp, GFP_KERNEL);
	if (!dev->intr_buf) {
		ret = -ENOMEM;
		goto error;
	}

	dev->intr_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->intr_urb) {
		ret = -ENOMEM;
		goto error;
	} else {
		usb_fill_int_urb(dev->intr_urb, dev->udev, dev->pipe_intr,
				 dev->intr_buf, maxp, ch397_intr_complete,
				 dev, period);
	}

	return ret;

error:
	ch397_free_all_resources(dev);

	return ret;
}

static inline bool rx_count_exceed(struct ch397 *dev)
{
	return atomic_read(&dev->rx_count) > dev->n_rx_urbs;
}

static inline int agg_offset(struct ch397_rx_agg *rx_agg, void *addr)
{
	return (int)(addr - rx_agg->buffer);
}

static inline void *rx_agg_align(void *data)
{
	return (void *)ALIGN((uintptr_t)data, 4);
}

static void ch397_record_rx_backlog_drop(struct ch397 *dev)
{
	struct net_device_stats *stats = &dev->ndev->stats;

	stats->rx_dropped++;
	stats->rx_errors++;
	dev->rx_backlog_drops++;
}

static struct ch397_rx_agg *ch397_get_free_rx_agg(struct ch397 *dev,
						  gfp_t mflags)
{
	struct ch397_rx_agg *rx_agg, *rx_agg_next, *rx_agg_free = NULL;
	unsigned long flags;

	spin_lock_irqsave(&dev->rxq_lock, flags);

	list_for_each_entry_safe(rx_agg, rx_agg_next, &dev->rxq_used,
				 list) {
		if (page_count(rx_agg->page) == 1) {
			if (!rx_agg_free) {
				list_del_init(&rx_agg->list);
				rx_agg_free = rx_agg;
				continue;
			}
			if (rx_count_exceed(dev)) {
				list_del_init(&rx_agg->list);
				ch397_free_rx_agg(dev, rx_agg);
			}
			break;
		}
	}

	spin_unlock_irqrestore(&dev->rxq_lock, flags);

	if (!rx_agg_free && atomic_read(&dev->rx_count) < dev->rx_pending)
		rx_agg_free = ch397_alloc_rx_agg(dev, mflags);

	return rx_agg_free;
}

static struct ch397_tx_agg *ch397_get_tx_agg(struct ch397 *dev)
{
	struct ch397_tx_agg *tx_agg = NULL;
	unsigned long flags;

	if (list_empty(&dev->txq_free))
		return NULL;

	spin_lock_irqsave(&dev->txq_lock, flags);
	if (!list_empty(&dev->txq_free)) {
		tx_agg = list_first_entry(&dev->txq_free,
					  struct ch397_tx_agg, list);
		/* Detach in-flight TX aggregates from txq_free.  The URB
		 * context provides the direct handle for completion, while
		 * txq_info remains the stable list for stop/free.
		 */
		list_del_init(&tx_agg->list);
	}
	spin_unlock_irqrestore(&dev->txq_lock, flags);

	return tx_agg;
}

static int ch397_read(struct ch397 *dev, u8 cmd, u32 reg, u16 length,
		      void *data)
{
	int ret;
	u16 value = (u16)(reg & 0xFFFF);
	u16 index = (u16)((reg >> 16) & 0xFFFF);
	void *tmp = NULL;

	if (ch397_dev_inaccessible(dev))
		return -ENODEV;

	if (length) {
		if (!data)
			return -EINVAL;
		tmp = kmalloc(length, GFP_KERNEL);
		if (!tmp)
			return -ENOMEM;
	}

	ret = usb_control_msg(
		dev->udev, dev->pipe_ctrl_in, cmd,
		USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE, value,
		index, tmp, length, CH397_USB_CTRL_GET_TIMEOUT);
	if (ret < 0) {
		ch397_update_usb_state(dev, ret);
		goto err;
	}
	if (ret != length) {
		ret = -EIO;
		goto out;
	}

	if (length)
		memcpy(data, tmp, length);

	dev_dbg(&dev->intf->dev,
		"ch397_read() cmd=0x%02x, reg=0x%08x, read=\n", cmd, reg);

	msleep(CH397_USB_DELAY);

	ret = 0;
out:
	kfree(tmp);
	return ret;
err:
	if (length)
		memset(data, 0xff, length);
	kfree(tmp);
	return ret;
}

static int ch397_write(struct ch397 *dev, u8 cmd, u32 reg, u16 length,
		       const void *data)
{
	int ret;
	u16 value = (u16)(reg & 0xFFFF);
	u16 index = (u16)((reg >> 16) & 0xFFFF);
	void *tmp = NULL;

	if (ch397_dev_inaccessible(dev))
		return -ENODEV;

	if (length) {
		if (!data)
			return -EINVAL;
		tmp = kmemdup(data, length, GFP_KERNEL);
		if (!tmp)
			return -ENOMEM;
	}

	ret = usb_control_msg(
		dev->udev, dev->pipe_ctrl_out, cmd,
		USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE, value,
		index, tmp, length, CH397_USB_CTRL_SET_TIMEOUT);
	if (ret < 0) {
		ch397_update_usb_state(dev, ret);
		goto out;
	}
	if (ret != length) {
		ret = -EIO;
		goto out;
	}

	dev_dbg(&dev->intf->dev,
		"ch397_write() cmd=0x%02x, reg=0x%08x, write=\n", cmd,
		reg);

	msleep(CH397_USB_DELAY);

	ret = 0;
out:
	kfree(tmp);
	return ret;
}

static int ch397_read_reg(struct ch397 *dev, u8 cmd, u32 reg, u16 length,
			  u32 *value)
{
	int ret;

	if (length == sizeof(__le16)) {
		__le16 tmp;

		ret = ch397_read(dev, cmd, reg, length, &tmp);
		if (ret < 0)
			return ret;

		*value = le16_to_cpu(tmp);
		return 0;
	}

	if (length == sizeof(__le32)) {
		__le32 tmp;

		ret = ch397_read(dev, cmd, reg, length, &tmp);
		if (ret < 0)
			return ret;

		*value = le32_to_cpu(tmp);
		return 0;
	}

	return -EINVAL;
}

static int ch397_write_reg(struct ch397 *dev, u8 cmd, u32 reg, u16 length,
			   u32 value)
{
	if (length == sizeof(__le16)) {
		__le16 tmp = cpu_to_le16((u16)value);

		return ch397_write(dev, cmd, reg, length, &tmp);
	}

	if (length == sizeof(__le32)) {
		__le32 tmp = cpu_to_le32(value);

		return ch397_write(dev, cmd, reg, length, &tmp);
	}

	return -EINVAL;
}

static int ch397_read_shared_word(struct ch397 *dev, int phy, u8 reg,
				  u16 *value)
{
	u32 tmp;
	int ret;

	ret = ch397_read_reg(dev, CMD_RD_OTH, CH397_ETH_BMSR | (reg << 16),
			     sizeof(__le16), &tmp);
	if (ret < 0) {
		printk(KERN_ERR "Error getting link status.\n");
		return ret;
	}

	*value = (u16)tmp;

	return ret;
}

static int ch397_write_shared_word(struct ch397 *dev, int phy, u8 reg,
				   u16 value)
{
	int ret;

	ret = ch397_write_reg(dev, CMD_WR_OTH,
			      CH397_ETH_BMSR | (reg << 16), sizeof(__le16),
			      value);
	if (ret < 0) {
		printk(KERN_ERR
		       "Error writing phy reg, phy: 0x%x, reg: 0x%x.\n",
		       phy, reg);
		return ret;
	}

	return ret;
}

static int ch397_mdio_read(struct net_device *ndev, int phy_id, int loc)
{
	struct ch397 *dev = netdev_priv(ndev);
	u16 res;
	int ret;

	if (ch397_dev_inaccessible(dev))
		return -ENODEV;

	if (phy_id) {
		netdev_dbg(dev->ndev, "Only internal phy supported\n");
		return 0;
	}

	ret = ch397_read_shared_word(dev, 1, loc, &res);
	if (ret < 0)
		return ret;

	netdev_dbg(
		dev->ndev,
		"ch397_mdio_read() phy_id=0x%02x, loc=0x%02x, returns=0x%04x\n",
		phy_id, loc, res);

	return res;
}

static void ch397_mdio_write(struct net_device *ndev, int phy_id, int loc,
			     int val)
{
	struct ch397 *dev = netdev_priv(ndev);

	if (ch397_dev_inaccessible(dev))
		return;

	if (phy_id) {
		netdev_dbg(dev->ndev, "Only internal phy supported\n");
		return;
	}

	ch397_write_shared_word(dev, 1, loc, val);

	netdev_dbg(
		dev->ndev,
		"ch397_mdio_write() phy_id=0x%02x, loc=0x%02x, val=0x%04x\n",
		phy_id, loc, val);
}

static int ch398_mdio_read_extn(struct ch397 *dev, int loc)
{
	ch397_mdio_write(dev->ndev, dev->mii.phy_id,
			 CH398_PHY_SWTICH_EXTEN, loc);
	return ch397_mdio_read(dev->ndev, dev->mii.phy_id,
			       CH398_PHY_READ_EXTEN);
}

static void ch397_phy_mmd_indirect(struct ch397 *dev, u16 prtad, u16 devad)
{
	u16 tmp16;

	tmp16 = devad;
	ch397_mdio_write(dev->ndev, dev->mii.phy_id, MII_MMD_CTRL, tmp16);

	tmp16 = prtad;
	ch397_mdio_write(dev->ndev, dev->mii.phy_id, MII_MMD_DATA, tmp16);

	tmp16 = devad | MII_MMD_CTRL_NOINCR;
	ch397_mdio_write(dev->ndev, dev->mii.phy_id, MII_MMD_CTRL, tmp16);

	return;
}

static int ch397_phy_read_mmd_indirect(struct ch397 *dev, u16 prtad,
				       u16 devad)
{
	int tmp16;

	ch397_phy_mmd_indirect(dev, prtad, devad);

	tmp16 = ch397_mdio_read(dev->ndev, dev->mii.phy_id, MII_MMD_DATA);

	return tmp16;
}

static int ch397_phy_write_mmd_indirect(struct ch397 *dev, u16 prtad,
					u16 devad, u16 data)
{
	ch397_phy_mmd_indirect(dev, prtad, devad);

	ch397_mdio_write(dev->ndev, dev->mii.phy_id, MII_MMD_DATA, data);

	return 0;
}

static void ch397_get_drvinfo(struct net_device *ndev,
			      struct ethtool_drvinfo *info)
{
	struct ch397 *dev = netdev_priv(ndev);
	char fw_version[32];

	snprintf(fw_version, sizeof(fw_version), "v1.%d", 0x30);
	strlcpy(info->driver, dev->intf->dev.driver->name,
		sizeof info->driver);
	strlcpy(info->version, VERSION_DESC, sizeof info->version);
	strlcpy(info->fw_version, fw_version, sizeof(info->fw_version));
	usb_make_path(dev->udev, info->bus_info, sizeof info->bus_info);
}

static void ch397_set_tx_qlen(struct ch397 *dev)
{
	dev->tx_qlen = dev->tx_qlen_scale * dev->tx_urb_size /
		       (mtu_to_size(dev->ndev->mtu) +
			sizeof(struct ch397_tx_header));
}

static int ch397_max_tx_mtu(struct ch397 *dev)
{
	size_t tx_overhead = CH397_TX_OVERHEAD;

	if (dev->version == WCH_CHIP_VER_02 && dev->hs_mode)
		tx_overhead += CH398_TX_LS_SIZE;

	if (dev->tx_urb_size <= tx_overhead + VLAN_ETH_HLEN)
		return ETH_MIN_MTU;

	return size_to_mtu(dev->tx_urb_size - tx_overhead);
}

static int ch397_nway_reset(struct net_device *ndev)
{
	struct ch397 *dev = netdev_priv(ndev);
	int ret;

	ret = usb_autopm_get_interface(dev->intf);
	if (ret < 0)
		goto out;

	mutex_lock(&dev->phy_mutex);
	ret = mii_nway_restart(&dev->mii);
	mutex_unlock(&dev->phy_mutex);

	usb_autopm_put_interface(dev->intf);

out:
	return ret;
}

static inline bool ch397_dev_unplugged(struct ch397 *dev)
{
	return test_bit(CH397_DEV_UNPLUG, &dev->flags);
}

static inline bool ch397_dev_inaccessible(struct ch397 *dev)
{
	return test_bit(CH397_DEV_INACCESSIBLE, &dev->flags);
}

static void ch397_set_inaccessible(struct ch397 *dev)
{
	set_bit(CH397_DEV_INACCESSIBLE, &dev->flags);
	smp_mb__after_atomic();
}

static void ch397_set_accessible(struct ch397 *dev)
{
	clear_bit(CH397_DEV_INACCESSIBLE, &dev->flags);
	smp_mb__after_atomic();
}

static void ch397_set_unplug(struct ch397 *dev)
{
	if (dev->udev->state != USB_STATE_NOTATTACHED)
		return;

	ch397_set_inaccessible(dev);
	set_bit(CH397_DEV_UNPLUG, &dev->flags);
	smp_mb__after_atomic();
}

static void ch397_update_usb_state(struct ch397 *dev, int status)
{
	if (status != -ENODEV && status != -ESHUTDOWN)
		return;

	ch397_set_inaccessible(dev);
	ch397_set_unplug(dev);
}

static u32 ch397_get_msglevel(struct net_device *ndev)
{
	struct ch397 *dev = netdev_priv(ndev);

	return dev->msg_enable;
}

static void ch397_set_msglevel(struct net_device *ndev, u32 level)
{
	struct ch397 *dev = netdev_priv(ndev);

	dev->msg_enable = level;
}

static u32 ch397_supported_advertising(struct ch397 *dev)
{
	if (dev->version == WCH_CHIP_VER_02)
		return PHY_ADVERTISED_10_FULL | PHY_ADVERTISED_100_FULL |
		       PHY_ADVERTISED_1000_FULL;

	return PHY_ADVERTISED_10_HALF | PHY_ADVERTISED_10_FULL |
	       PHY_ADVERTISED_100_HALF | PHY_ADVERTISED_100_FULL;
}

/* Normalize autonegotiation requests into an explicit advertising mask so
 * suspend/resume and link-reset replay the user's intent consistently.
 */
static int ch397_normalize_advertising(struct ch397 *dev, u8 autoneg,
				       u16 speed, u8 duplex,
				       u32 advertising, u32 *normalized)
{
	u32 support = ch397_supported_advertising(dev);
	u32 req = advertising & support;

	if (!normalized)
		return -EINVAL;

	if (autoneg == AUTONEG_DISABLE) {
		*normalized = 0;
		return 0;
	}

	if (autoneg != AUTONEG_ENABLE)
		return -EINVAL;

	if (req) {
		*normalized = req;
		return 0;
	}

	if (advertising)
		return -EINVAL;

	switch (speed) {
	case SPEED_10:
		if (dev->version == WCH_CHIP_VER_02) {
			if (duplex == DUPLEX_HALF)
				return -EINVAL;
			*normalized = PHY_ADVERTISED_10_FULL;
		} else if (duplex == DUPLEX_HALF) {
			*normalized = PHY_ADVERTISED_10_HALF;
		} else if (duplex == DUPLEX_FULL) {
			*normalized = PHY_ADVERTISED_10_FULL;
		} else {
			*normalized = PHY_ADVERTISED_10_HALF |
				      PHY_ADVERTISED_10_FULL;
		}
		return 0;
	case SPEED_100:
		if (dev->version == WCH_CHIP_VER_02) {
			if (duplex == DUPLEX_HALF)
				return -EINVAL;
			*normalized = PHY_ADVERTISED_100_FULL;
		} else if (duplex == DUPLEX_HALF) {
			*normalized = PHY_ADVERTISED_100_HALF;
		} else if (duplex == DUPLEX_FULL) {
			*normalized = PHY_ADVERTISED_100_FULL;
		} else {
			*normalized = PHY_ADVERTISED_100_HALF |
				      PHY_ADVERTISED_100_FULL;
		}
		return 0;
	case SPEED_1000:
		if (!(support & PHY_ADVERTISED_1000_FULL))
			return -EINVAL;
		if (duplex == DUPLEX_HALF)
			return -EINVAL;
		*normalized = PHY_ADVERTISED_1000_FULL;
		return 0;
	default:
		*normalized = support;
		return 0;
	}
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 12, 0))
static int ch397_get_settings(struct net_device *ndev,
			      struct ethtool_link_ksettings *cmd)
#else
static int ch397_get_settings(struct net_device *ndev,
			      struct ethtool_cmd *cmd)
#endif
{
	struct ch397 *dev = netdev_priv(ndev);
	int ret;

	if (!dev->mii.mdio_read)
		return -EOPNOTSUPP;

	ret = usb_autopm_get_interface(dev->intf);
	if (ret < 0)
		goto out;

	mutex_lock(&dev->phy_mutex);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 12, 0))
	mii_ethtool_get_link_ksettings(&dev->mii, cmd);
#else
	mii_ethtool_gset(&dev->mii, cmd);
#endif

	mutex_unlock(&dev->phy_mutex);

	usb_autopm_put_interface(dev->intf);

out:
	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 12, 0))
static int ch397_set_settings(struct net_device *ndev,
			      const struct ethtool_link_ksettings *cmd)
#else
static int ch397_set_settings(struct net_device *ndev,
			      struct ethtool_cmd *cmd)
#endif
{
	struct ch397 *dev = netdev_priv(ndev);
	struct ch397_ndev_cfg *cfg = &dev->ndev_cfg;
	u32 advertising = 0, normalized_advertising = 0;
	u8 autoneg, duplex;
	u16 speed;
	int ret;

	ret = usb_autopm_get_interface(dev->intf);
	if (ret < 0)
		goto out;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 12, 0))
	if (test_bit(ETHTOOL_LINK_MODE_10baseT_Half_BIT,
		     cmd->link_modes.advertising))
		advertising |= PHY_ADVERTISED_10_HALF;

	if (test_bit(ETHTOOL_LINK_MODE_10baseT_Full_BIT,
		     cmd->link_modes.advertising))
		advertising |= PHY_ADVERTISED_10_FULL;

	if (test_bit(ETHTOOL_LINK_MODE_100baseT_Half_BIT,
		     cmd->link_modes.advertising))
		advertising |= PHY_ADVERTISED_100_HALF;

	if (test_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT,
		     cmd->link_modes.advertising))
		advertising |= PHY_ADVERTISED_100_FULL;

	if (test_bit(ETHTOOL_LINK_MODE_1000baseT_Half_BIT,
		     cmd->link_modes.advertising))
		advertising |= PHY_ADVERTISED_1000_HALF;

	if (test_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
		     cmd->link_modes.advertising))
		advertising |= PHY_ADVERTISED_1000_FULL;
#else
	advertising = cmd->advertising;
#endif

	mutex_lock(&dev->phy_mutex);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 12, 0))
	autoneg = cmd->base.autoneg;
	speed = cmd->base.speed;
	duplex = cmd->base.duplex;
#else
	autoneg = cmd->autoneg;
	speed = ethtool_cmd_speed(cmd);
	duplex = cmd->duplex;
#endif

	ret = ch397_normalize_advertising(dev, autoneg, speed, duplex,
					  advertising,
					  &normalized_advertising);
	if (ret < 0)
		goto out_unlock;

	ret = dev->dev_ops->set_speed(dev, autoneg, speed, duplex,
				      normalized_advertising);
	if (!ret) {
		cfg->autoneg = autoneg;
		cfg->speed = speed;
		cfg->duplex = duplex;
		cfg->advertising = normalized_advertising;
	}

out_unlock:
	mutex_unlock(&dev->phy_mutex);

	usb_autopm_put_interface(dev->intf);

out:
	return ret;
}

static void ch397_ethtool_get_wol(struct ch397 *dev,
				  struct ethtool_wolinfo *wol)
{
	u32 macier;
	int ret;

	wol->supported = WAKE_PHY | WAKE_MAGIC;

	ret = ch397_read_reg(dev, CMD_RD_REG, CH397_ETH_MACIER,
			     sizeof(__le16), &macier);
	if (ret < 0)
		return;

	if (macier & CH397_ENABLE_MAGIC)
		wol->wolopts |= WAKE_MAGIC;
	if (macier & CH397_ENABLE_WOLPTN)
		wol->wolopts |= WAKE_PHY;

	return;
}

static void ch398_ethtool_get_wol(struct ch397 *dev,
				  struct ethtool_wolinfo *wol)
{
	u32 macier;
	int ret;

	wol->supported = WAKE_PHY | WAKE_UCAST | WAKE_MCAST | WAKE_BCAST |
			 WAKE_MAGIC;

	ret = ch397_read_reg(dev, CMD_RD_REG, CH398_ETH_MACIER,
			     sizeof(__le32), &macier);
	if (ret < 0)
		return;

	if (dev->ndev_cfg.phy_wol)
		wol->wolopts |= WAKE_PHY;
	if (macier & CH398_ENABLE_UCAST)
		wol->wolopts |= WAKE_UCAST;
	if (macier & CH398_ENABLE_MCAST)
		wol->wolopts |= WAKE_MCAST;
	if (macier & CH398_ENABLE_BCAST)
		wol->wolopts |= WAKE_BCAST;
	if (macier & CH398_ENABLE_MAGIC)
		wol->wolopts |= WAKE_MAGIC;

	return;
}

static u32 ch397_supported_wolopts(struct ch397 *dev)
{
	if (dev->version == WCH_CHIP_VER_02)
		return WAKE_PHY | WAKE_UCAST | WAKE_MCAST | WAKE_BCAST |
		       WAKE_MAGIC;

	return WAKE_PHY | WAKE_MAGIC;
}

static void ch397_get_wol(struct net_device *ndev,
			  struct ethtool_wolinfo *wol)
{
	struct ch397 *dev = netdev_priv(ndev);
	struct usb_device *udev = dev->udev;

	wol->supported = 0;
	wol->wolopts = 0;

	if (usb_autopm_get_interface(dev->intf) < 0)
		return;

	if (!(udev->actconfig->desc.bmAttributes &
	      USB_CONFIG_ATT_WAKEUP)) {
		wol->supported = 0;
		wol->wolopts = 0;
	} else {
		mutex_lock(&dev->phy_mutex);
		dev->dev_ops->get_wol(dev, wol);
		mutex_unlock(&dev->phy_mutex);
	}

	usb_autopm_put_interface(dev->intf);
}

static int ch397_ethtool_set_wol(struct ch397 *dev,
				 struct ethtool_wolinfo *wol)
{
	u32 macier;
	int ret;

	ret = ch397_read_reg(dev, CMD_RD_REG, CH397_ETH_MACIER,
			     sizeof(__le16), &macier);
	if (ret < 0)
		return ret;

	macier &= ~(CH397_ENABLE_WOLPTN | CH397_ENABLE_MAGIC);

	if (wol->wolopts & WAKE_MAGIC)
		macier |= CH397_ENABLE_MAGIC;

	if (wol->wolopts & WAKE_PHY)
		macier |= CH397_ENABLE_WOLPTN;

	return ch397_write_reg(dev, CMD_WR_REG, CH397_ETH_MACIER,
			       sizeof(__le16), macier);
}

static int ch398_ethtool_set_wol(struct ch397 *dev,
				 struct ethtool_wolinfo *wol)
{
	bool new_phy_wol = !!(wol->wolopts & WAKE_PHY);
	u32 macier;
	int ret;

	ret = ch397_read_reg(dev, CMD_RD_REG, CH398_ETH_MACIER,
			     sizeof(__le32), &macier);
	if (ret < 0)
		return ret;

	ret = ch397_write(dev, CMD_SET_MISC_CONFIG,
			  new_phy_wol ? 0 : CH398_DISABLE_PHY_INTR, 0,
			  NULL);
	if (ret < 0)
		return ret;

	macier &= ~(CH398_ENABLE_UCAST | CH398_ENABLE_MCAST |
		    CH398_ENABLE_BCAST | CH398_ENABLE_MAGIC);

	if (wol->wolopts & WAKE_UCAST)
		macier |= CH398_ENABLE_UCAST;
	if (wol->wolopts & WAKE_MCAST)
		macier |= CH398_ENABLE_MCAST;
	if (wol->wolopts & WAKE_BCAST)
		macier |= CH398_ENABLE_BCAST;
	if (wol->wolopts & WAKE_MAGIC)
		macier |= CH398_ENABLE_MAGIC;

	ret = ch397_write_reg(dev, CMD_WR_REG, CH398_ETH_MACIER,
			      sizeof(__le32), macier);
	if (ret < 0)
		return ret;

	dev->ndev_cfg.phy_wol = new_phy_wol;

	return 0;
}

static int ch397_set_wol(struct net_device *net,
			 struct ethtool_wolinfo *wol)
{
	struct ch397 *dev = netdev_priv(net);
	struct usb_device *udev = dev->udev;
	u32 supported = ch397_supported_wolopts(dev);
	int ret;

	if (wol->wolopts & ~supported)
		return -EOPNOTSUPP;

	if (!(udev->actconfig->desc.bmAttributes &
	      USB_CONFIG_ATT_WAKEUP) &&
	    wol->wolopts)
		return -EOPNOTSUPP;

	ret = usb_autopm_get_interface(dev->intf);
	if (ret < 0)
		return ret;

	mutex_lock(&dev->phy_mutex);
	ret = dev->dev_ops->set_wol(dev, wol);
	mutex_unlock(&dev->phy_mutex);

	if (!ret)
		device_set_wakeup_enable(&udev->dev, !!wol->wolopts);

	usb_autopm_put_interface(dev->intf);

	return ret;
}

static void ch397_get_strings(struct net_device *netdev, u32 stringset,
			      u8 *data)
{
	if (stringset == ETH_SS_STATS)
		memcpy(data, ch397_gstrings, sizeof(ch397_gstrings));
}

static int ch397_get_sset_count(struct net_device *ndev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return ARRAY_SIZE(ch397_gstrings);
	default:
		return -EOPNOTSUPP;
	}
}

static void ch397_get_stats(struct ch397 *dev, u64 *data)
{
	const struct ch397_intr_stats *e = &dev->ch397_intr;

	data[0] = e->rx_packets;
	data[1] = e->tx_packets;
	data[2] = e->rx_overflow_cnt;
	data[3] = e->rx_crc_cnt;
	data[4] = dev->rx_backlog_drops;
}

static void ch398_get_stats(struct ch397 *dev, u64 *data)
{
	const struct ch397_intr_stats *e = &dev->ch398_intr;

	data[0] = e->rx_packets;
	data[1] = e->tx_packets;
	data[2] = e->rx_overflow_cnt;
	data[3] = e->rx_crc_cnt;
	data[4] = dev->rx_backlog_drops;
}

static void ch397_get_ethtool_stats(struct net_device *ndev,
				    struct ethtool_stats *stats, u64 *data)
{
	struct ch397 *dev = netdev_priv(ndev);

	if (usb_autopm_get_interface(dev->intf) < 0)
		return;

	usb_autopm_put_interface(dev->intf);

	dev->dev_ops->get_ethtool_stats(dev, data);
}

static int ch397_get_tunable(struct net_device *ndev,
			     const struct ethtool_tunable *tunable,
			     void *d)
{
	struct ch397 *dev = netdev_priv(ndev);

	switch (tunable->id) {
	case ETHTOOL_RX_COPYBREAK:
		*(u32 *)d = dev->rx_copybreak;
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int ch397_set_tunable(struct net_device *ndev,
			     const struct ethtool_tunable *tunable,
			     const void *d)
{
	struct ch397 *dev = netdev_priv(ndev);
	u32 val;

	switch (tunable->id) {
	case ETHTOOL_RX_COPYBREAK:
		val = *(u32 *)d;
		if (val < ETH_ZLEN) {
			netif_err(dev, rx_err, ndev,
				  "Invalid rx copy break value\n");
			return -EINVAL;
		}

		if (dev->rx_copybreak != val) {
			if (ndev->flags & IFF_UP) {
				mutex_lock(&dev->dev_mutex);
				napi_disable(&dev->napi);
				dev->rx_copybreak = val;
				napi_enable(&dev->napi);
				mutex_unlock(&dev->dev_mutex);
			} else {
				dev->rx_copybreak = val;
			}
		}
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(5, 16, 0))
static void ch397_get_ringparam(struct net_device *ndev,
				struct ethtool_ringparam *ring)
#else
static void
ch397_get_ringparam(struct net_device *ndev,
		    struct ethtool_ringparam *ring,
		    struct kernel_ethtool_ringparam *kernel_ring,
		    struct netlink_ext_ack *extack)
#endif
{
	struct ch397 *dev = netdev_priv(ndev);

	ring->rx_max_pending = CH397_RX_MAX_PENDING;
	ring->rx_pending = dev->rx_pending;
}

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(5, 16, 0))
static int ch397_set_ringparam(struct net_device *ndev,
			       struct ethtool_ringparam *ring)
#else
static int
ch397_set_ringparam(struct net_device *ndev,
		    struct ethtool_ringparam *ring,
		    struct kernel_ethtool_ringparam *kernel_ring,
		    struct netlink_ext_ack *extack)
#endif
{
	struct ch397 *dev = netdev_priv(ndev);

	if (ring->rx_pending < (dev->n_rx_urbs * 2) ||
	    ring->rx_pending > CH397_RX_MAX_PENDING)
		return -EINVAL;

	if (dev->rx_pending != ring->rx_pending) {
		if (ndev->flags & IFF_UP) {
			mutex_lock(&dev->dev_mutex);
			napi_disable(&dev->napi);
			dev->rx_pending = ring->rx_pending;
			napi_enable(&dev->napi);
			mutex_unlock(&dev->dev_mutex);
		} else {
			dev->rx_pending = ring->rx_pending;
		}
	}

	return 0;
}

static int ch397_update_mac_flowctrl(struct ch397 *dev, u8 flow_ctrl)
{
	struct ch397_ndev_cfg *cfg = &dev->ndev_cfg;
	u32 mac_cfg;
	int ret;

	ret = ch397_read_reg(dev, CMD_RD_REG, dev->regs->mac_cfg,
			     sizeof(__le32), &mac_cfg);
	if (ret < 0)
		return ret;

	if (flow_ctrl)
		mac_cfg |= CH397_FLOWCTRL_EN;
	else
		mac_cfg &= ~CH397_FLOWCTRL_EN;

	if (cfg->dev_info.val_l >= 0x37)
		return ch397_write(dev, CMD_WR_ETH_MACCFG, mac_cfg, 0,
				   NULL);

	return ch397_write_reg(dev, CMD_WR_REG, dev->regs->mac_cfg,
			       sizeof(__le32), mac_cfg);
}

static void ch397_get_pauseparam(struct net_device *ndev,
				 struct ethtool_pauseparam *pause)
{
	struct ch397 *dev = netdev_priv(ndev);
	int bmcr, anar, lpa;
	u8 cap;

	memset(pause, 0, sizeof(*pause));

	if (usb_autopm_get_interface(dev->intf) < 0)
		return;

	mutex_lock(&dev->phy_mutex);

	bmcr = ch397_mdio_read(ndev, dev->mii.phy_id, MII_BMCR);
	anar = ch397_mdio_read(ndev, dev->mii.phy_id, MII_ADVERTISE);
	lpa = ch397_mdio_read(ndev, dev->mii.phy_id, MII_LPA);

	mutex_unlock(&dev->phy_mutex);

	usb_autopm_put_interface(dev->intf);

	if (bmcr < 0 || anar < 0 || lpa < 0)
		return;

	if (!(bmcr & BMCR_ANENABLE)) {
		return;
	}

	pause->autoneg = 1;

	cap = mii_resolve_flowctrl_fdx(anar, lpa);

	if (cap & FLOW_CTRL_RX)
		pause->rx_pause = 1;

	if (cap & FLOW_CTRL_TX)
		pause->tx_pause = 1;
}

static int ch397_set_pauseparam(struct net_device *ndev,
				struct ethtool_pauseparam *pause)
{
	struct ch397 *dev = netdev_priv(ndev);
	int bmcr;
	int old;
	u16 new1;
	u8 cap = 0;
	int ret;

	ret = usb_autopm_get_interface(dev->intf);
	if (ret < 0)
		return ret;

	mutex_lock(&dev->phy_mutex);

	bmcr = ch397_mdio_read(ndev, dev->mii.phy_id, MII_BMCR);
	if (bmcr < 0) {
		ret = bmcr;
		goto out;
	}

	if (pause->autoneg && !(bmcr & BMCR_ANENABLE)) {
		ret = -EINVAL;
		goto out;
	}

	if (pause->rx_pause)
		cap |= FLOW_CTRL_RX;

	if (pause->tx_pause)
		cap |= FLOW_CTRL_TX;

	old = ch397_mdio_read(ndev, dev->mii.phy_id, MII_ADVERTISE);
	if (old < 0) {
		ret = old;
		goto out;
	}

	new1 = (old & ~(ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM)) |
	       mii_advertise_flowctrl(cap);
	if (old != new1) {
		ch397_mdio_write(ndev, dev->mii.phy_id, MII_ADVERTISE,
				 new1);
		if (pause->autoneg) {
			ret = mii_nway_restart(&dev->mii);
			if (ret < 0)
				goto out;
		}
	}

	ret = ch397_update_mac_flowctrl(dev, cap);
	if (ret < 0)
		goto out;

	dev->ndev_cfg.flow_ctrl = cap;

out:
	mutex_unlock(&dev->phy_mutex);
	usb_autopm_put_interface(dev->intf);

	return ret;
}

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(6, 9, 0))
static int ch397_ethtool_get_eee(struct ch397 *dev,
				 struct ethtool_eee *data)
{
	int val;

	/* Get Supported EEE */
	val = ch397_phy_read_mmd_indirect(dev, MDIO_PCS_EEE_ABLE,
					  MDIO_MMD_PCS);
	if (val < 0)
		return val;
	data->supported = mmd_eee_cap_to_ethtool_sup_t(val);

	/* Get advertisement EEE */
	val = ch397_phy_read_mmd_indirect(dev, MDIO_AN_EEE_ADV,
					  MDIO_MMD_AN);
	if (val < 0)
		return val;
	data->advertised = mmd_eee_adv_to_ethtool_adv_t(val);

	/* Get LP advertisement EEE */
	val = ch397_phy_read_mmd_indirect(dev, MDIO_AN_EEE_LPABLE,
					  MDIO_MMD_AN);
	if (val < 0)
		return val;
	data->lp_advertised = mmd_eee_adv_to_ethtool_adv_t(val);

	return 0;
}
#else
static int ch397_ethtool_get_eee(struct ch397 *dev,
				 struct ethtool_keee *data)
{
	int val;

	/* Get Supported EEE */
	val = ch397_phy_read_mmd_indirect(dev, MDIO_PCS_EEE_ABLE,
					  MDIO_MMD_PCS);
	if (val < 0)
		return val;
	mii_eee_cap1_mod_linkmode_t(data->supported, val);

	/* Get advertisement EEE */
	val = ch397_phy_read_mmd_indirect(dev, MDIO_AN_EEE_ADV,
					  MDIO_MMD_AN);
	if (val < 0)
		return val;
	mii_eee_cap1_mod_linkmode_t(data->advertised, val);

	/* Get LP advertisement EEE */
	val = ch397_phy_read_mmd_indirect(dev, MDIO_AN_EEE_LPABLE,
					  MDIO_MMD_AN);
	if (val < 0)
		return val;
	mii_eee_cap1_mod_linkmode_t(data->lp_advertised, val);

	return 0;
}
#endif

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(6, 9, 0))
static int ch398_ethtool_get_eee(struct ch397 *dev,
				 struct ethtool_eee *data)
{
	int val;
	int eee_support = 0, eee_adv = 0;

	/* Get Supported EEE */
	val = ch397_mdio_read(dev->ndev, dev->mii.phy_id,
			      CH398_PHY_LOOP_STATUS_REG);
	if (val < 0)
		return val;
	if (val & CH398_PHY_EEE_1000M_ENABLE)
		eee_support |= MDIO_EEE_1000T;
	if (val & CH398_PHY_EEE_100M_ENABLE)
		eee_support |= MDIO_EEE_100TX;
	data->supported = mmd_eee_cap_to_ethtool_sup_t(eee_support);

	/* Get advertisement EEE */
	val = ch397_mdio_read(dev->ndev, dev->mii.phy_id,
			      CH398_PHY_AUX_CTRL_REG);
	if (val < 0)
		return val;
	if (val & CH398_PHY_EEE_1000M_ADV)
		eee_adv |= MDIO_EEE_1000T;
	if (val & CH398_PHY_EEE_100M_ADV)
		eee_adv |= MDIO_EEE_100TX;
	eee_adv &= eee_support;
	data->advertised = mmd_eee_adv_to_ethtool_adv_t(eee_adv);
	data->eee_enabled = !!eee_adv;

	/* Get LP advertisement EEE */
	val = ch398_mdio_read_extn(dev, CH398_PHY_EEE_LPA_REG);
	if (val < 0)
		return val;
	data->lp_advertised = mmd_eee_adv_to_ethtool_adv_t(val);

	data->eee_active = !!(data->advertised & data->lp_advertised);

	return 0;
}

#else
static int ch398_ethtool_get_eee(struct ch397 *dev,
				 struct ethtool_keee *data)
{
	__ETHTOOL_DECLARE_LINK_MODE_MASK(common);
	int val;
	int eee_support = 0, eee_adv = 0;

	/* Get Supported EEE */
	val = ch397_mdio_read(dev->ndev, dev->mii.phy_id,
			      CH398_PHY_LOOP_STATUS_REG);
	if (val < 0)
		return val;
	if (val & CH398_PHY_EEE_1000M_ENABLE)
		eee_support |= MDIO_EEE_1000T;
	if (val & CH398_PHY_EEE_100M_ENABLE)
		eee_support |= MDIO_EEE_100TX;
	mii_eee_cap1_mod_linkmode_t(data->supported, eee_support);

	/* Get advertisement EEE */
	val = ch397_mdio_read(dev->ndev, dev->mii.phy_id,
			      CH398_PHY_AUX_CTRL_REG);
	if (val < 0)
		return val;
	if (val & CH398_PHY_EEE_1000M_ADV)
		eee_adv |= MDIO_EEE_1000T;
	if (val & CH398_PHY_EEE_100M_ADV)
		eee_adv |= MDIO_EEE_100TX;
	eee_adv &= eee_support;
	mii_eee_cap1_mod_linkmode_t(data->advertised, eee_adv);
	data->eee_enabled = !!eee_adv;

	/* Get LP advertisement EEE */
	val = ch398_mdio_read_extn(dev, CH398_PHY_EEE_LPA_REG);
	if (val < 0)
		return val;
	mii_eee_cap1_mod_linkmode_t(data->lp_advertised, val);

	data->eee_enabled = dev->ndev_cfg.eee_enabled;

	linkmode_and(common, data->advertised, data->lp_advertised);
	data->eee_active = phy_check_valid(dev->ndev_cfg.link_speed,
					   dev->ndev_cfg.link_duplex,
					   common);

	return 0;
}
#endif

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(6, 9, 0))
static int ch397_get_eee(struct net_device *net, struct ethtool_eee *edata)
#else
static int ch397_get_eee(struct net_device *net,
			 struct ethtool_keee *edata)
#endif
{
	struct ch397 *dev = netdev_priv(net);
	int ret;

	ret = usb_autopm_get_interface(dev->intf);
	if (ret < 0)
		return ret;

	mutex_lock(&dev->phy_mutex);
	ret = dev->dev_ops->get_eee(dev, edata);
	mutex_unlock(&dev->phy_mutex);

	usb_autopm_put_interface(dev->intf);

	return ret;
}

static int ch397_disable_eee(struct ch397 *dev)
{
	int tmp16;

	ch397_mdio_write(dev->ndev, dev->mii.phy_id, CH397_PHY_PAGE_SLET,
			 CH397_PHY_PAGE4);

	tmp16 = ch397_mdio_read(dev->ndev, dev->mii.phy_id,
				CH397_PHY_CAP0EEE);
	if (tmp16 < 0)
		goto out;

	tmp16 &= ~CH397_PHY_EEE_10M_CAP;
	ch397_mdio_write(dev->ndev, dev->mii.phy_id, CH397_PHY_CAP0EEE,
			 tmp16);

	tmp16 = 0;
out:
	ch397_mdio_write(dev->ndev, dev->mii.phy_id, CH397_PHY_PAGE_SLET,
			 CH397_PHY_PAGE0);

	return tmp16;
}

static int ch397_enable_eee(struct ch397 *dev)
{
	int tmp16;

	ch397_mdio_write(dev->ndev, dev->mii.phy_id, CH397_PHY_PAGE_SLET,
			 CH397_PHY_PAGE4);

	tmp16 = ch397_mdio_read(dev->ndev, dev->mii.phy_id,
				CH397_PHY_CAP0EEE);
	if (tmp16 < 0)
		goto out;

	tmp16 |= CH397_PHY_EEE_10M_CAP;
	ch397_mdio_write(dev->ndev, dev->mii.phy_id, CH397_PHY_CAP0EEE,
			 tmp16);

	tmp16 = 0;
out:
	ch397_mdio_write(dev->ndev, dev->mii.phy_id, CH397_PHY_PAGE_SLET,
			 CH397_PHY_PAGE0);

	return tmp16;
}

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(6, 9, 0))
static int ch397_ethtool_set_eee(struct ch397 *dev,
				 struct ethtool_eee *data)
{
	u16 tmp16 = ethtool_adv_to_mmd_eee_adv_t(data->advertised);
	int ret;

	if (!dev->ndev_cfg.eee_enabled) {
		ret = ch397_disable_eee(dev);
		tmp16 = 0;
	} else {
		ret = ch397_enable_eee(dev);
	}
	if (ret < 0)
		return ret;

	ret = ch397_phy_write_mmd_indirect(dev, MDIO_AN_EEE_ADV,
					   MDIO_MMD_AN, tmp16);
	if (ret < 0)
		return ret;

	dev->ndev_cfg.eee_advertised = tmp16;

	return 0;
}
#else
static int ch397_ethtool_set_eee(struct ch397 *dev,
				 struct ethtool_keee *data)
{
	u16 tmp16 = linkmode_to_mii_eee_cap1_t(data->advertised);
	int ret;

	if (!dev->ndev_cfg.eee_enabled) {
		ret = ch397_disable_eee(dev);
		tmp16 = 0;
	} else {
		ret = ch397_enable_eee(dev);
	}
	if (ret < 0)
		return ret;

	ret = ch397_phy_write_mmd_indirect(dev, MDIO_AN_EEE_ADV,
					   MDIO_MMD_AN, tmp16);
	if (ret < 0)
		return ret;

	dev->ndev_cfg.eee_advertised = tmp16;

	return 0;
}
#endif

static int ch398_set_eee_adv(struct ch397 *dev, u16 adv)
{
	int tmp16;

	tmp16 = ch397_mdio_read(dev->ndev, dev->mii.phy_id,
				CH398_PHY_AUX_CTRL_REG);
	if (tmp16 < 0)
		return tmp16;

	tmp16 &= ~(CH398_PHY_EEE_1000M_ADV | CH398_PHY_EEE_100M_ADV);
	if (adv & MDIO_EEE_1000T)
		tmp16 |= CH398_PHY_EEE_1000M_ADV;
	if (adv & MDIO_EEE_100TX)
		tmp16 |= CH398_PHY_EEE_100M_ADV;
	ch397_mdio_write(dev->ndev, dev->mii.phy_id,
			 CH398_PHY_AUX_CTRL_REG, tmp16);

	return 0;
}

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(6, 9, 0))
static int ch398_ethtool_set_eee(struct ch397 *dev,
				 struct ethtool_eee *data)
{
	u16 tmp16 = ethtool_adv_to_mmd_eee_adv_t(data->advertised);
	int ret;

	tmp16 &= MDIO_EEE_1000T | MDIO_EEE_100TX;
	if (!dev->ndev_cfg.eee_enabled)
		tmp16 = 0;

	ret = ch398_set_eee_adv(dev, tmp16);
	if (ret < 0)
		return ret;

	dev->ndev_cfg.eee_advertised = tmp16;
	dev->ndev_cfg.eee_enabled = !!tmp16;

	return 0;
}
#else
static int ch398_ethtool_set_eee(struct ch397 *dev,
				 struct ethtool_keee *data)
{
	u16 tmp16 = linkmode_to_mii_eee_cap1_t(data->advertised);
	int ret;

	tmp16 &= MDIO_EEE_1000T | MDIO_EEE_100TX;
	if (!dev->ndev_cfg.eee_enabled)
		tmp16 = 0;

	ret = ch398_set_eee_adv(dev, tmp16);
	if (ret < 0)
		return ret;

	dev->ndev_cfg.eee_advertised = tmp16;
	dev->ndev_cfg.eee_enabled = !!tmp16;

	return 0;
}
#endif

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(6, 9, 0))
static int ch397_set_eee(struct net_device *net, struct ethtool_eee *edata)
#else
static int ch397_set_eee(struct net_device *net,
			 struct ethtool_keee *edata)
#endif
{
	struct ch397 *dev = netdev_priv(net);
	struct ch397_ndev_cfg *cfg = &dev->ndev_cfg;
	bool old_eee_enabled = cfg->eee_enabled;
	int ret;

	ret = usb_autopm_get_interface(dev->intf);
	if (ret < 0)
		return ret;

	mutex_lock(&dev->phy_mutex);
	cfg->eee_enabled = !!edata->eee_enabled;

	ret = dev->dev_ops->set_eee(dev, edata);
	if (ret) {
		cfg->eee_enabled = old_eee_enabled;
		goto out;
	}

	ret = mii_nway_restart(&dev->mii);
out:
	mutex_unlock(&dev->phy_mutex);
	usb_autopm_put_interface(dev->intf);

	return ret;
}

static const struct ethtool_ops ch397_ethtool_ops = {
	.get_drvinfo = ch397_get_drvinfo,
	.nway_reset = ch397_nway_reset,
	.get_link = ethtool_op_get_link,
	.get_msglevel = ch397_get_msglevel,
	.set_msglevel = ch397_set_msglevel,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 12, 0))
	.get_link_ksettings = ch397_get_settings,
	.set_link_ksettings = ch397_set_settings,
#else
	.get_settings = ch397_get_settings,
	.set_settings = ch397_set_settings,
#endif
	.get_wol = ch397_get_wol,
	.set_wol = ch397_set_wol,
	.get_strings = ch397_get_strings,
	.get_sset_count = ch397_get_sset_count,
	.get_ethtool_stats = ch397_get_ethtool_stats,
	.get_tunable = ch397_get_tunable,
	.set_tunable = ch397_set_tunable,
	.get_ringparam = ch397_get_ringparam,
	.set_ringparam = ch397_set_ringparam,
	.get_pauseparam = ch397_get_pauseparam,
	.set_pauseparam = ch397_set_pauseparam,
	.get_eee = ch397_get_eee,
	.set_eee = ch397_set_eee,
};

static void ch397_tx_complete(struct urb *urb)
{
	struct net_device_stats *stats;
	struct net_device *ndev;
	struct ch397_tx_agg *tx_agg;
	struct ch397 *dev;
	unsigned long flags;

	tx_agg = urb->context;
	if (!tx_agg)
		return;

	dev = tx_agg->context;
	if (!dev)
		return;

	ndev = dev->ndev;
	stats = &ndev->stats;

	if (urb->status == 0) {
		stats->tx_packets += tx_agg->skb_num;
		stats->tx_bytes += tx_agg->skb_len;
	} else {
		if (net_ratelimit())
			netdev_dbg(ndev, "Tx status %d\n", urb->status);
		stats->tx_errors += tx_agg->skb_num;
	}

	spin_lock_irqsave(&dev->txq_lock, flags);
	list_add_tail(&tx_agg->list, &dev->txq_free);
	spin_unlock_irqrestore(&dev->txq_lock, flags);

	usb_autopm_put_interface_async(dev->intf);

	if (!netif_carrier_ok(ndev))
		return;

	if (!test_bit(CH397_DEV_OPEN, &dev->flags))
		return;

	if (ch397_dev_inaccessible(dev))
		return;

	/* Restart TX bottom half if queued packets remain after this URB completes. */
	if (!skb_queue_empty(&dev->txq_pend))
		tasklet_schedule(&dev->tx_bh);
}

static int ch397_rx_submit(struct ch397 *dev, struct ch397_rx_agg *rx_agg,
			   gfp_t flags)
{
	int ret = 0;
	size_t size = dev->rx_urb_size;

	if (!netif_carrier_ok(dev->ndev) || ch397_dev_inaccessible(dev) ||
	    !test_bit(CH397_DEV_OPEN, &dev->flags))
		return 0;

	usb_fill_bulk_urb(rx_agg->urb, dev->udev, dev->pipe_in,
			  rx_agg->buffer, size, ch397_rx_complete, rx_agg);

	ret = usb_submit_urb(rx_agg->urb, flags);
	if (ret == -ENODEV || ret == -ESHUTDOWN) {
		ch397_update_usb_state(dev, ret);
		netif_device_detach(dev->ndev);
	} else if (ret) {
		struct urb *urb = rx_agg->urb;
		unsigned long flags;

		urb->actual_length = 0;
		spin_lock_irqsave(&dev->rxq_lock, flags);
		list_add_tail(&rx_agg->list, &dev->rxq_done);
		spin_unlock_irqrestore(&dev->rxq_lock, flags);
		netif_err(dev, rx_err, dev->ndev,
			  "Couldn't submit rx[%p], ret = %d\n", rx_agg,
			  ret);

		napi_schedule(&dev->napi);
	}

	return ret;
}

static void ch397_rx_complete(struct urb *urb)
{
	struct net_device *ndev;
	struct ch397 *dev;
	struct ch397_rx_agg *rx_agg;
	int urb_status = urb->status;
	unsigned long flags;

	rx_agg = urb->context;
	if (!rx_agg)
		return;

	dev = rx_agg->context;
	if (!dev)
		return;

	if (ch397_dev_inaccessible(dev))
		return;

	if (!test_bit(CH397_DEV_OPEN, &dev->flags))
		return;

	ndev = dev->ndev;

	/* Do not resubmit RX URBs while link is down; link-reset path will restart RX. */
	if (!netif_carrier_ok(ndev))
		return;

	usb_mark_last_busy(dev->udev);

	switch (urb_status) {
	case 0:
		if (urb->actual_length < ETH_ZLEN)
			break;

		spin_lock_irqsave(&dev->rxq_lock, flags);
		list_add_tail(&rx_agg->list, &dev->rxq_done);
		spin_unlock_irqrestore(&dev->rxq_lock, flags);
		napi_schedule(&dev->napi);
		return;
	case -ESHUTDOWN:
		ch397_update_usb_state(dev, urb_status);
		netif_device_detach(dev->ndev);
		return;
	case -EPROTO:
		urb->actual_length = 0;
		spin_lock_irqsave(&dev->rxq_lock, flags);
		list_add_tail(&rx_agg->list, &dev->rxq_done);
		spin_unlock_irqrestore(&dev->rxq_lock, flags);
		set_bit(CH397_RX_EPROTO, &dev->flags);
		schedule_delayed_work(&dev->wq, 1);
		return;
	case -ENOENT:
		return; /* the urb is in unlink state */
	case -ETIME:
		if (net_ratelimit())
			netdev_warn(ndev, "maybe reset is needed?\n");
		break;
	default:
		if (net_ratelimit())
			netdev_warn(ndev, "Rx status %d\n", urb_status);
		break;
	}

	ch397_rx_submit(dev, rx_agg, GFP_ATOMIC);
}

static void ch397_intr_status(struct ch397 *dev, struct urb *urb)
{
	struct ch397_intr_stats *event = &dev->ch397_intr;
	struct ch397_intr_event tmp;
	struct ch397_ndev_cfg *cfg = &dev->ndev_cfg;
	int link, carrier;

	if (urb->actual_length < sizeof(tmp))
		return;

	memcpy(&tmp, urb->transfer_buffer, sizeof(tmp));

	event->link_stat = le32_to_cpu(tmp.link_stat);
	event->rx_packets = le32_to_cpu(tmp.rx_packets);
	event->rx_overflow_cnt = le16_to_cpu(tmp.rx_overflow_cnt);
	event->rx_crc_cnt = le16_to_cpu(tmp.rx_crc_cnt);
	event->tx_packets = le32_to_cpu(tmp.tx_packets);

	link = !!(event->link_stat & CH397_LINK_RDY);
	carrier = netif_carrier_ok(dev->ndev);
	cfg->link = link;

	if (link) {
		cfg->link_speed = (event->link_stat & CH397_LINK_SPEED) ?
					  SPEED_10 :
					  SPEED_100;
		cfg->link_duplex = (event->link_stat & CH397_DUPLEX_MODE) ?
					   DUPLEX_FULL :
					   DUPLEX_HALF;
	}

	/* CH397 need reapply the remembered
	 * PHY configuration from workqueue context when link state changes.
	 */
	if (!link && link != carrier) {
		set_bit(CH397_LINK_CHG, &dev->flags);
		schedule_delayed_work(&dev->wq, CH397_WORK_DELAY);
	}

	if (link) {
		if (!carrier) {
			set_bit(CH397_LINK_RESET, &dev->flags);
			schedule_delayed_work(&dev->wq, CH397_WORK_DELAY);
		}
	} else {
		if (carrier) {
			netif_stop_queue(dev->ndev);
			set_bit(CH397_LINK_RESET, &dev->flags);
			schedule_delayed_work(&dev->wq, CH397_WORK_DELAY);
		}
	}
}

static void ch398_intr_status(struct ch397 *dev, struct urb *urb)
{
	struct ch397_intr_stats *event = &dev->ch398_intr;
	struct ch398_intr_event tmp;
	struct ch397_ndev_cfg *cfg = &dev->ndev_cfg;
	int link, carrier;

	if (urb->actual_length < sizeof(tmp))
		return;

	memcpy(&tmp, urb->transfer_buffer, sizeof(tmp));

	event->link_stat = le32_to_cpu(tmp.link_stat);
	event->rx_packets = le32_to_cpu(tmp.rx_packets);
	event->rx_overflow_cnt = le16_to_cpu(tmp.rx_overflow_cnt);
	event->rx_crc_cnt = le16_to_cpu(tmp.rx_crc_cnt);
	event->tx_packets = le32_to_cpu(tmp.tx_packets);

	link = !!(event->link_stat & CH398_LINK_RDY);
	carrier = netif_carrier_ok(dev->ndev);
	cfg->link = link;

	if (link) {
		if (event->link_stat & CH398_SPEED_1G) {
			cfg->link_speed = SPEED_1000;
		} else {
			if (event->link_stat & CH398_LINK_SPEED)
				cfg->link_speed = SPEED_10;
			else
				cfg->link_speed = SPEED_100;
		}

		if (event->link_stat & CH398_DUPLEX_MODE)
			cfg->link_duplex = DUPLEX_FULL;
		else
			cfg->link_duplex = DUPLEX_HALF;
	}

	if (link) {
		if (!carrier) {
			set_bit(CH397_LINK_RESET, &dev->flags);
			schedule_delayed_work(&dev->wq, CH397_WORK_DELAY);
		}
	} else {
		if (carrier) {
			netif_stop_queue(dev->ndev);
			set_bit(CH397_LINK_RESET, &dev->flags);
			schedule_delayed_work(&dev->wq, CH397_WORK_DELAY);
		}
	}
}

static void ch397_intr_complete(struct urb *urb)
{
	struct ch397 *dev = urb->context;
	int status = urb->status;

	if (!test_bit(CH397_DEV_OPEN, &dev->flags))
		return;

	if (ch397_dev_inaccessible(dev))
		return;

	switch (status) {
	/* success */
	case 0:
		break;

	/* software-driven interface shutdown */
	case -ENOENT: /* urb killed */
		return;
	case -ESHUTDOWN: /* hardware gone */
		ch397_update_usb_state(dev, status);
		netif_device_detach(dev->ndev);
		netif_dbg(dev, ifdown, dev->ndev,
			  "intr shutdown, code %d\n", status);
		return;
	/* NOTE:  not throttling like RX/TX, since this endpoint
	 * already polls infrequently
	 */
	default:
		netdev_dbg(dev->ndev, "intr status %d\n", status);
		goto resubmit;
	}

	dev->dev_ops->intr_status(dev, urb);

resubmit:
	status = usb_submit_urb(urb, GFP_ATOMIC);
	if (status == -ENODEV || status == -ESHUTDOWN) {
		ch397_update_usb_state(dev, status);
		netif_device_detach(dev->ndev);
	} else if (status) {
		netif_err(dev, intr, dev->ndev,
			  "can't resubmit intr, status %d\n", status);
	}
}

static int ch397_tx_agg_fill(struct ch397 *dev,
			     struct ch397_tx_agg *tx_agg)
{
	struct sk_buff_head skb_head;
	struct net_device_stats *stats = &dev->ndev->stats;
	int remain = dev->tx_urb_size;
	int urb_len;
	u8 *tx_data;
	int ret;

	__skb_queue_head_init(&skb_head);
	spin_lock(&dev->txq_pend.lock);
	skb_queue_splice_init(&dev->txq_pend, &skb_head);
	spin_unlock(&dev->txq_pend.lock);

	tx_data = tx_agg->head;
	tx_agg->skb_num = 0;
	tx_agg->skb_len = 0;

	/* Pack queued SKBs into one USB TX URB until the URB buffer is full. */
	while (remain >= ETH_ZLEN + sizeof(struct ch397_tx_header)) {
		struct ch397_tx_header *tx_header;
		struct sk_buff *skb;
		unsigned int len;

		skb = __skb_dequeue(&skb_head);
		if (!skb)
			break;

		len = ALIGN(skb->len + sizeof(*tx_header), 4);

		/* not enough space */
		if (len > remain) {
			__skb_queue_head(&skb_head, skb);
			break;
		}

		tx_data = ch397_tx_agg_align(tx_data);
		tx_header = (struct ch397_tx_header *)tx_data;
		tx_header->cmd_0 = cpu_to_le32(skb->len & TX_LEN_MASK);
		tx_header->cmd_1 = cpu_to_le32(0);
		tx_data += CH397_TX_OVERHEAD;

		len = skb->len;
		if (skb_copy_bits(skb, 0, tx_data, len) < 0) {
			stats->tx_dropped++;
			dev_kfree_skb_any(skb);
			tx_data -= CH397_TX_OVERHEAD;
			continue;
		}

		tx_data += ALIGN(len, 4);
		tx_agg->skb_num += skb_shinfo(skb)->gso_segs ?: 1;
		tx_agg->skb_len += len;
		dev_kfree_skb_any(skb);

		/* Ensure 4-byte alignment */
		remain = dev->tx_urb_size -
			 (int)(ch397_tx_agg_align(tx_data) - tx_agg->head);
		if (unlikely(dev->tx_single_packet))
			break;
	}

	if (!skb_queue_empty(&skb_head)) {
		spin_lock(&dev->txq_pend.lock);
		skb_queue_splice(&skb_head, &dev->txq_pend);
		spin_unlock(&dev->txq_pend.lock);
	}

	if (unlikely(!tx_agg->skb_num))
		return -EAGAIN;

	netif_tx_lock(dev->ndev);
	if (netif_queue_stopped(dev->ndev) &&
	    skb_queue_len(&dev->txq_pend) <= dev->tx_qlen)
		netif_wake_queue(dev->ndev);
	netif_tx_unlock(dev->ndev);

	ret = usb_autopm_get_interface_async(dev->intf);
	if (ret < 0)
		goto drop;

	urb_len = (int)(tx_data - (u8 *)tx_agg->head);

	usb_fill_bulk_urb(tx_agg->urb, dev->udev, dev->pipe_out,
			  tx_agg->head, urb_len, ch397_tx_complete,
			  tx_agg);

	tx_agg->urb->transfer_flags &= ~URB_ZERO_PACKET;
	if (urb_len % dev->maxpacket == 0)
		tx_agg->urb->transfer_flags |= URB_ZERO_PACKET;

	ret = usb_submit_urb(tx_agg->urb, GFP_ATOMIC);
	if (ret < 0)
		usb_autopm_put_interface_async(dev->intf);

drop:
	return ret;
}

static void ch398_tx_drop_skb(struct ch397 *dev, struct sk_buff *skb)
{
	struct net_device_stats *stats = &dev->ndev->stats;

	stats->tx_dropped++;
	dev_kfree_skb_any(skb);
}

/* Fall back to software segmentation or software checksum when CH398 TX
 * offload cannot be used for the current skb layout.
 */
static void ch398_tx_csum_workaround(struct ch397 *dev,
				     struct sk_buff *skb,
				     struct sk_buff_head *list)
{
	if (skb_shinfo(skb)->gso_size) {
		netdev_features_t features = dev->ndev->features;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
		struct sk_buff *segs, *seg, *next;
#else
		struct sk_buff *segs, *nskb;
#endif
		struct sk_buff_head seg_list;

		features &=
			~(NETIF_F_SG | NETIF_F_IPV6_CSUM | NETIF_F_TSO6 |
			  NETIF_F_TSO | NETIF_F_IP_CSUM);

		segs = skb_gso_segment(skb, features);
		if (IS_ERR(segs) || !segs) {
			ch398_tx_drop_skb(dev, skb);
			return;
		}

		__skb_queue_head_init(&seg_list);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
		skb_list_walk_safe(segs, seg, next)
		{
			skb_mark_not_on_list(seg);
			__skb_queue_tail(&seg_list, seg);
		}
#else
		do {
			nskb = segs;
			segs = segs->next;
			nskb->next = NULL;
			__skb_queue_tail(&seg_list, nskb);
		} while (segs);
#endif
		skb_queue_splice(&seg_list, list);
		dev_kfree_skb_any(skb);
		return;
	}

	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		if (skb_checksum_help(skb) < 0) {
			ch398_tx_drop_skb(dev, skb);
			return;
		}

		__skb_queue_head(list, skb);
		return;
	}

	ch398_tx_drop_skb(dev, skb);
}

/* Build CH398 TX offload descriptors. Return CH398_TX_CSUM_SW when the skb
 * must be transmitted through software fallback instead of hardware offload.
 */
static int ch398_tx_cmd_fill(struct ch397 *dev, struct sk_buff *skb,
			     struct ch397_tx_header *tx_header)
{
	u32 cmd_0, cmd_1 = 0;
	u32 mss = skb_shinfo(skb)->gso_size;

	cmd_0 = (skb->len & TX_LEN_MASK);

	if (mss) {
		u32 offset = skb_transport_offset(skb) + tcp_hdrlen(skb);

		if (offset > TX_TCPHO_MASK) {
			netif_warn(
				dev, tx_err, dev->ndev,
				"Invalid transport offset 0x%x for TSO\n",
				offset);
			return CH398_TX_CSUM_SW;
		}

		if (skb_cow_head(skb, 0) < 0)
			return -ENOMEM;

		switch (vlan_get_protocol(skb)) {
		case htons(ETH_P_IP):
			cmd_0 |= TX_GTSENDV4;
			break;
		case htons(ETH_P_IPV6):
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 7, 0))
			tcp_v6_gso_csum_prep(skb);
#else
			ipv6_hdr(skb)->payload_len = 0;
			tcp_hdr(skb)->check = ~csum_ipv6_magic(
				&ipv6_hdr(skb)->saddr,
				&ipv6_hdr(skb)->daddr, 0, IPPROTO_TCP, 0);
#endif
			cmd_0 |= TX_GTSENDV6;
			break;
		default:
			return CH398_TX_CSUM_SW;
		}

		cmd_0 |= (offset << TX_TCPHO_SHIFT);
		cmd_1 = min(mss, TX_MSS_MAX);
	} else if (skb->ip_summed == CHECKSUM_PARTIAL) {
		u8 ip_protocol = IPPROTO_RAW;

		switch (vlan_get_protocol(skb)) {
		case htons(ETH_P_IP):
			cmd_1 |= TX_IPV4_CS;
			ip_protocol = ip_hdr(skb)->protocol;
			break;
		case htons(ETH_P_IPV6):
			ip_protocol = ipv6_hdr(skb)->nexthdr;
			break;
		default:
			return CH398_TX_CSUM_SW;
		}

		if (ip_protocol == IPPROTO_TCP)
			cmd_1 |= TX_TCP_CS;
		else if (ip_protocol == IPPROTO_UDP)
			cmd_1 |= TX_UDP_CS;
		else
			return CH398_TX_CSUM_SW;
	}

	if (skb_vlan_tag_present(skb)) {
		cmd_1 |= TX_VLAN_TAG |
			 ((skb_vlan_tag_get(skb) & TX_VTAG_MASK)
			  << TX_VTAG_SHIFT);
	}

	tx_header->cmd_0 = cpu_to_le32(cmd_0);
	tx_header->cmd_1 = cpu_to_le32(cmd_1);

	return CH398_TX_CSUM_OK;
}

/* Aggregate CH398 TX packets into one URB and invoke software fallback when
 * hardware checksum or TSO constraints cannot be satisfied.
 */
static int ch398_tx_agg_fill(struct ch397 *dev,
			     struct ch397_tx_agg *tx_agg)
{
	struct ch397_tx_header *last_tx_header = NULL;
	struct sk_buff_head skb_head;
	struct net_device_stats *stats = &dev->ndev->stats;
	int remain = dev->tx_urb_size;
	size_t tx_buf_len;
	u8 *tx_data;
	int urb_len = 0, ret = 0;

	__skb_queue_head_init(&skb_head);
	spin_lock(&dev->txq_pend.lock);
	skb_queue_splice_init(&dev->txq_pend, &skb_head);
	spin_unlock(&dev->txq_pend.lock);

	tx_data = tx_agg->head;
	tx_agg->skb_num = 0;
	tx_agg->skb_len = 0;

	/* Pack queued SKBs into one USB TX URB. CH398 may fall back to software
	 * checksum or software segmentation when hardware offload cannot be used.
	 */
	while (remain >= ETH_ZLEN + sizeof(struct ch397_tx_header)) {
		struct ch397_tx_header *tx_header;
		struct sk_buff *skb;
		unsigned int len;

		skb = __skb_dequeue(&skb_head);
		if (!skb)
			break;

		len = ALIGN(skb->len + CH397_TX_OVERHEAD, 4);

		/* not enough space */
		if (len > remain) {
			__skb_queue_head(&skb_head, skb);
			break;
		}

		/* Ensure 4-byte alignment */
		tx_data = ch397_tx_agg_align(tx_data);
		tx_header = (struct ch397_tx_header *)tx_data;
		ret = ch398_tx_cmd_fill(dev, skb, tx_header);
		if (unlikely(ret == CH398_TX_CSUM_SW)) {
			ch398_tx_csum_workaround(dev, skb, &skb_head);
			continue;
		}
		if (unlikely(ret < 0)) {
			ch398_tx_drop_skb(dev, skb);
			continue;
		}

		tx_data += CH397_TX_OVERHEAD;

		len = skb->len;
		if (skb_copy_bits(skb, 0, tx_data, len) < 0) {
			stats->tx_dropped++;
			dev_kfree_skb_any(skb);
			tx_data -= CH397_TX_OVERHEAD;
			continue;
		}

		tx_data += ALIGN(len, 4);
		tx_agg->skb_num += skb_shinfo(skb)->gso_segs ?: 1;
		tx_agg->skb_len += len;
		dev_kfree_skb_any(skb);
		last_tx_header = tx_header;

		remain = dev->tx_urb_size -
			 (int)(ch397_tx_agg_align(tx_data) - tx_agg->head);
	}

	if (!skb_queue_empty(&skb_head)) {
		spin_lock(&dev->txq_pend.lock);
		skb_queue_splice(&skb_head, &dev->txq_pend);
		spin_unlock(&dev->txq_pend.lock);
	}

	if (unlikely(!tx_agg->skb_num))
		return -EAGAIN;

	netif_tx_lock(dev->ndev);
	if (netif_queue_stopped(dev->ndev) &&
	    skb_queue_len(&dev->txq_pend) <= dev->tx_qlen)
		netif_wake_queue(dev->ndev);
	netif_tx_unlock(dev->ndev);

	urb_len = (int)(tx_data - (u8 *)tx_agg->head);
	tx_buf_len = dev->tx_urb_size + CH397_TX_BUF_EXTRA -
		     ((u8 *)tx_agg->head - (u8 *)tx_agg->buffer);

	/* High-speed bulk OUT needs TX_LS when URB length is an exact 512-byte multiple. */
	if (unlikely((urb_len % dev->maxpacket) == 0) &&
	    unlikely(dev->hs_mode) && last_tx_header) {
		u32 cmd_0 = le32_to_cpu(last_tx_header->cmd_0);

		if (WARN_ON_ONCE(urb_len + CH398_TX_LS_SIZE > tx_buf_len))
			return -EMSGSIZE;

		memset(tx_data, 0, CH398_TX_LS_SIZE);
		cmd_0 |= TX_LS;
		last_tx_header->cmd_0 = cpu_to_le32(cmd_0);
		urb_len += CH398_TX_LS_SIZE;
	}

	ret = usb_autopm_get_interface_async(dev->intf);
	if (ret < 0)
		return ret;

	usb_fill_bulk_urb(tx_agg->urb, dev->udev, dev->pipe_out,
			  tx_agg->head, urb_len, ch397_tx_complete,
			  tx_agg);

	ret = usb_submit_urb(tx_agg->urb, GFP_ATOMIC);
	if (ret < 0)
		usb_autopm_put_interface_async(dev->intf);

	return ret;
}

static void ch397_tx_bh(struct ch397 *dev)
{
	int ret;

	do {
		struct net_device *ndev = dev->ndev;
		struct ch397_tx_agg *tx_agg;

		if (skb_queue_empty(&dev->txq_pend))
			break;

		tx_agg = ch397_get_tx_agg(dev);
		if (!tx_agg) {
			break;
		}

		ret = dev->dev_ops->tx_fixup(dev, tx_agg);
		if (!ret)
			continue;

		if (ret == -EAGAIN) {
			unsigned long flags;

			spin_lock_irqsave(&dev->txq_lock, flags);
			list_add_tail(&tx_agg->list, &dev->txq_free);
			spin_unlock_irqrestore(&dev->txq_lock, flags);

			if (!skb_queue_empty(&dev->txq_pend))
				tasklet_schedule(&dev->tx_bh);
			break;
		}

		if (ret == -ENODEV || ret == -ESHUTDOWN) {
			ch397_update_usb_state(dev, ret);
			netif_device_detach(ndev);
		} else {
			struct net_device_stats *stats = &ndev->stats;
			unsigned long flags;

			netif_warn(dev, tx_err, ndev, "failed tx_urb %d\n",
				   ret);
			stats->tx_dropped += tx_agg->skb_num;

			spin_lock_irqsave(&dev->txq_lock, flags);
			list_add_tail(&tx_agg->list, &dev->txq_free);
			spin_unlock_irqrestore(&dev->txq_lock, flags);
		}
	} while (ret == 0);
}

static u8 ch398_rx_cksum_offload(struct ch397 *dev,
				 struct ch397_rx_header *rx_header)
{
	u32 cmd_1 = le32_to_cpu(rx_header->cmd_1);

	if (!(dev->ndev->features & NETIF_F_RXCSUM))
		return CHECKSUM_NONE;

	if (((cmd_1 & RX_UDPV4_CSC) &&
	     !(cmd_1 & (RX_UDPCS_ER | RX_IPCS_ER))) ||
	    ((cmd_1 & RX_TCPV4_CSC) &&
	     !(cmd_1 & (RX_TCPCS_ER | RX_IPCS_ER))) ||
	    ((cmd_1 & RX_UDPV6_CSC) && !(cmd_1 & RX_UDPCS_ER)) ||
	    ((cmd_1 & RX_TCPV6_CSC) && !(cmd_1 & RX_TCPCS_ER)))
		return CHECKSUM_UNNECESSARY;

	return CHECKSUM_NONE;
}

static void ch398_rx_vlan_offload(struct sk_buff *skb,
				  struct ch397_rx_header *rx_header)
{
	u32 cmd_1 = le32_to_cpu(rx_header->cmd_1);

	if (cmd_1 & RX_VLAN_TAG)
		__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q),
				       (cmd_1 >> RX_VTAG_SHIFT) &
					       RX_VTAG_MASK);
}

static void ch397_rx_cmd_offload(struct ch397 *dev, struct sk_buff *skb,
				 struct ch397_rx_header *rx_header)
{
	(void)dev;
	(void)skb;
	(void)rx_header;

	/* CH397 has no RX checksum or VLAN offload metadata to translate. */
}

static void ch398_rx_cmd_offload(struct ch397 *dev, struct sk_buff *skb,
				 struct ch397_rx_header *rx_header)
{
	skb->ip_summed = ch398_rx_cksum_offload(dev, rx_header);
	ch398_rx_vlan_offload(skb, rx_header);
}

static int ch397_rx_skb_fill(struct ch397 *dev, int budget, int work_done)
{
	struct list_head *cursor, *next, rx_queue;
	unsigned long flags;
	int ret = 0;

	INIT_LIST_HEAD(&rx_queue);
	spin_lock_irqsave(&dev->rxq_lock, flags);
	list_splice_init(&dev->rxq_done, &rx_queue);
	spin_unlock_irqrestore(&dev->rxq_lock, flags);

	list_for_each_safe(cursor, next, &rx_queue) {
		struct ch397_rx_header *rx_header;
		struct ch397_rx_agg *rx_agg, *rx_agg_free;
		struct urb *urb;
		u8 *rx_data;
		int len_used = 0;

		if (work_done >= budget)
			break;

		list_del_init(cursor);
		rx_agg = list_entry(cursor, struct ch397_rx_agg, list);
		urb = rx_agg->urb;
		if (urb->status != 0 || urb->actual_length < ETH_ZLEN)
			goto submit;

		rx_agg_free = ch397_get_free_rx_agg(dev, GFP_ATOMIC);
		rx_data = rx_agg->buffer;
		rx_header = (struct ch397_rx_header *)rx_data;
		len_used += CH397_RX_OVERHEAD;

		while (urb->actual_length > len_used) {
			struct net_device *ndev = dev->ndev;
			struct net_device_stats *stats = &ndev->stats;
			struct sk_buff *skb;
			unsigned int net_pkt_len, pkt_len, rx_frag_head_sz;
			bool use_frags;

			/* Bound deferred RX backlog so NAPI budget overruns
			 * do not grow unbounded latency.
			 */
			if (unlikely(skb_queue_len(&dev->rxq_pend) >=
				     dev->rxq_pend_limit)) {
				ch397_record_rx_backlog_drop(dev);
				break;
			}

			net_pkt_len = le32_to_cpu(rx_header->cmd_0) &
				      0xffff;
			if ((net_pkt_len < ETH_ZLEN)) {
				netdev_err(
					dev->ndev,
					"%s : Bad Header Length: 0x%x\n",
					__func__, net_pkt_len);
				break;
			}

			pkt_len = net_pkt_len;

			len_used += net_pkt_len;
			if (urb->actual_length < len_used)
				break;

			rx_data += CH397_RX_OVERHEAD;

			if (!rx_agg_free ||
			    dev->rx_copybreak > net_pkt_len)
				use_frags = false;
			else
				use_frags = true;

			if (use_frags) {
				if (work_done >= budget) {
					rx_frag_head_sz =
						dev->rx_copybreak;
					skb = ch397_napi_alloc_skb(
						dev, rx_frag_head_sz);
				} else {
					rx_frag_head_sz = 0;
					skb = napi_get_frags(&dev->napi);
				}
			} else {
				rx_frag_head_sz = 0;
				skb = ch397_napi_alloc_skb(dev,
							   net_pkt_len);
			}

			if (!skb) {
				stats->rx_dropped++;
				goto find_next_rx;
			}

			dev->dev_ops->rx_fixup(dev, skb, rx_header);

			if (use_frags) {
				if (rx_frag_head_sz) {
					memcpy(skb->data, rx_data,
					       rx_frag_head_sz);
					skb_put(skb, rx_frag_head_sz);
					net_pkt_len -= rx_frag_head_sz;
					rx_data += rx_frag_head_sz;
					skb->protocol =
						eth_type_trans(skb, ndev);
				}

				skb_add_rx_frag(
					skb, 0, rx_agg->page,
					agg_offset(rx_agg, rx_data),
					net_pkt_len,
					SKB_DATA_ALIGN(net_pkt_len));
				get_page(rx_agg->page);
			} else {
				memcpy(skb->data, rx_data, net_pkt_len);
				skb_put(skb, net_pkt_len);
				skb->protocol = eth_type_trans(skb, ndev);
			}

			if (work_done < budget) {
				if (use_frags)
					napi_gro_frags(&dev->napi);
				else
					napi_gro_receive(&dev->napi, skb);

				work_done++;
				stats->rx_packets++;
				stats->rx_bytes += pkt_len;
			} else {
				__skb_queue_tail(&dev->rxq_pend, skb);
			}

find_next_rx:
			rx_data = rx_agg_align(rx_data + net_pkt_len);
			rx_header = (struct ch397_rx_header *)rx_data;
			len_used = agg_offset(rx_agg, rx_data);
			len_used += CH397_RX_OVERHEAD;
		}

		WARN_ON(!rx_agg_free && page_count(rx_agg->page) > 1);

		if (rx_agg_free) {
			spin_lock_irqsave(&dev->rxq_lock, flags);
			if (page_count(rx_agg->page) == 1) {
				list_add(&rx_agg_free->list,
					 &dev->rxq_used);
			} else {
				list_add_tail(&rx_agg->list,
					      &dev->rxq_used);
				rx_agg = rx_agg_free;
				urb = rx_agg->urb;
			}
			spin_unlock_irqrestore(&dev->rxq_lock, flags);
		}

submit:
		if (!ret) {
			ret = ch397_rx_submit(dev, rx_agg, GFP_ATOMIC);
		} else {
			urb->actual_length = 0;
			list_add_tail(&rx_agg->list, next);
		}
	}

	if (!list_empty(&rx_queue)) {
		spin_lock_irqsave(&dev->rxq_lock, flags);
		list_splice(&rx_queue, &dev->rxq_done);
		spin_unlock_irqrestore(&dev->rxq_lock, flags);
	}

	return work_done;
}

static int ch397_rx_bh(struct ch397 *dev, int budget)
{
	int work_done = 0;

	if (!skb_queue_empty(&dev->rxq_pend)) {
		while (work_done < budget) {
			struct sk_buff *skb =
				__skb_dequeue(&dev->rxq_pend);
			struct net_device *ndev = dev->ndev;
			struct net_device_stats *stats = &ndev->stats;
			unsigned int pkt_len;

			if (!skb)
				break;

			pkt_len = skb->len;
			napi_gro_receive(&dev->napi, skb);
			work_done++;
			stats->rx_packets++;
			stats->rx_bytes += pkt_len;
		}
	}

	if (list_empty(&dev->rxq_done) || work_done >= budget)
		goto out1;

	clear_bit(CH397_RX_EPROTO, &dev->flags);
	work_done = ch397_rx_skb_fill(dev, budget, work_done);

out1:
	return work_done;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 9, 0))
static void ch397_bh(struct tasklet_struct *t)
{
	struct ch397 *dev = from_tasklet(dev, t, tx_bh);
#else
static void ch397_bh(unsigned long param)
{
	struct ch397 *dev = (struct ch397 *)param;
#endif

	if (ch397_dev_inaccessible(dev))
		return;

	if (!test_bit(CH397_DEV_OPEN, &dev->flags))
		return;

	/* Do not submit TX work while carrier is down; link-reset path will restart traffic. */
	if (!netif_carrier_ok(dev->ndev))
		return;

	clear_bit(CH397_SCHEDULE_TASKLET, &dev->flags);

	ch397_tx_bh(dev);
}

static int ch397_poll(struct napi_struct *napi, int budget)
{
	struct ch397 *dev = container_of(napi, struct ch397, napi);
	int work_done;

	if (!budget)
		return 0;

	/* Drain completed RX work and recycle RX URBs within the NAPI budget. */
	work_done = ch397_rx_bh(dev, budget);

	if (work_done < budget) {
		if (!ch397_napi_complete_done(napi, work_done))
			goto out;

		/* Continue polling if more completed RX aggregates are still pending. */
		if (!list_empty(&dev->rxq_done)) {
			napi_schedule(napi);
		}
	}

out:
	return work_done;
}

static int ch397_start_rx(struct ch397 *dev)
{
	struct ch397_rx_agg *rx_agg, *rx_agg_next;
	struct list_head tmp_list;
	unsigned long flags;
	int ret = 0, i = 0;

	INIT_LIST_HEAD(&tmp_list);

	spin_lock_irqsave(&dev->rxq_lock, flags);

	INIT_LIST_HEAD(&dev->rxq_done);
	INIT_LIST_HEAD(&dev->rxq_used);

	list_splice_init(&dev->rxq_info, &tmp_list);

	spin_unlock_irqrestore(&dev->rxq_lock, flags);

	list_for_each_entry_safe(rx_agg, rx_agg_next, &tmp_list,
				 info_list) {
		INIT_LIST_HEAD(&rx_agg->list);

		/* Keep only n_rx_urbs submitted; extra RX aggregates stay on
		 * the reusable list.
		 */
		if (++i > dev->n_rx_urbs) {
			spin_lock_irqsave(&dev->rxq_lock, flags);
			list_add_tail(&rx_agg->list, &dev->rxq_used);
			spin_unlock_irqrestore(&dev->rxq_lock, flags);
		} else if (unlikely(ret < 0)) {
			spin_lock_irqsave(&dev->rxq_lock, flags);
			list_add_tail(&rx_agg->list, &dev->rxq_done);
			spin_unlock_irqrestore(&dev->rxq_lock, flags);
		} else {
			ret = ch397_rx_submit(dev, rx_agg, GFP_KERNEL);
		}
	}

	spin_lock_irqsave(&dev->rxq_lock, flags);
	WARN_ON(!list_empty(&dev->rxq_info));
	list_splice(&tmp_list, &dev->rxq_info);
	spin_unlock_irqrestore(&dev->rxq_lock, flags);

	return ret;
}

static int ch397_stop_rx(struct ch397 *dev)
{
	struct ch397_rx_agg *rx_agg, *rx_agg_next;
	struct list_head tmp_list;
	unsigned long flags;

	INIT_LIST_HEAD(&tmp_list);

	/* Move rxq_info to a temporary list so URBs can be killed outside the spinlock. */
	spin_lock_irqsave(&dev->rxq_lock, flags);
	list_splice_init(&dev->rxq_info, &tmp_list);
	spin_unlock_irqrestore(&dev->rxq_lock, flags);

	list_for_each_entry_safe(rx_agg, rx_agg_next, &tmp_list,
				 info_list) {
		/* Aggregates whose pages are still shared cannot be reused and must be freed. */
		if (page_count(rx_agg->page) > 1)
			ch397_free_rx_agg(dev, rx_agg);
		else
			usb_kill_urb(rx_agg->urb);
	}

	/* Move back the list of temp to the rx_info */
	spin_lock_irqsave(&dev->rxq_lock, flags);
	WARN_ON(!list_empty(&dev->rxq_info));
	list_splice(&tmp_list, &dev->rxq_info);
	spin_unlock_irqrestore(&dev->rxq_lock, flags);

	while (!skb_queue_empty(&dev->rxq_pend))
		dev_kfree_skb(__skb_dequeue(&dev->rxq_pend));

	return 0;
}

static void ch397_drop_tx_skb(struct ch397 *dev)
{
	struct net_device_stats *stats = &dev->ndev->stats;
	struct sk_buff_head skb_head, *txq_pend = &dev->txq_pend;
	struct sk_buff *skb;

	if (skb_queue_empty(txq_pend))
		return;

	__skb_queue_head_init(&skb_head);
	spin_lock_bh(&txq_pend->lock);
	skb_queue_splice_init(txq_pend, &skb_head);
	spin_unlock_bh(&txq_pend->lock);

	while ((skb = __skb_dequeue(&skb_head))) {
		dev_kfree_skb(skb);
		stats->tx_dropped++;
	}
}

#ifdef CONFIG_PM_SLEEP
static int ch397_notifier(struct notifier_block *nb, unsigned long action,
			  void *data)
{
	struct ch397 *dev = container_of(nb, struct ch397, pm_notifier);
	int ret;

	switch (action) {
	case PM_HIBERNATION_PREPARE:
	case PM_SUSPEND_PREPARE:
		if (!dev->pm_notifier_held) {
			ret = usb_autopm_get_interface(dev->intf);
			if (ret >= 0)
				dev->pm_notifier_held = true;
		}
		break;

	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		if (dev->pm_notifier_held) {
			usb_autopm_put_interface(dev->intf);
			dev->pm_notifier_held = false;
		}
		break;

	case PM_RESTORE_PREPARE:
	default:
		break;
	}

	return NOTIFY_DONE;
}
#endif

static void ch397_disable(struct ch397 *dev)
{
	ch397_drop_tx_skb(dev);
	ch397_kill_tx_urb(dev);

	ch397_stop_rx(dev);
}

static int ch397_open(struct net_device *ndev)
{
	struct ch397 *dev = netdev_priv(ndev);
	int ret;

	mutex_lock(&dev->dev_mutex);

	if (ch397_dev_unplugged(dev)) {
		ret = -ENODEV;
		goto out_unlock;
	}

	ch397_set_accessible(dev);

	ret = ch397_alloc_all_resources(dev);
	if (ret < 0)
		goto out_unlock;

	ret = usb_autopm_get_interface(dev->intf);
	if (ret < 0)
		goto err_free;

	netif_carrier_off(ndev);
	netif_start_queue(ndev);
	set_bit(CH397_DEV_OPEN, &dev->flags);

	/* Interrupt URB drives link-state updates and RX/TX statistics. */
	ret = usb_submit_urb(dev->intr_urb, GFP_KERNEL);
	if (ret < 0) {
		ch397_update_usb_state(dev, ret);
		goto err_pm;
	}

	napi_enable(&dev->napi);
	tasklet_enable(&dev->tx_bh);

	usb_autopm_put_interface(dev->intf);

#ifdef CONFIG_PM_SLEEP
	dev->pm_notifier_held = false;
	dev->pm_notifier.notifier_call = ch397_notifier;
	register_pm_notifier(&dev->pm_notifier);
#endif
	mutex_unlock(&dev->dev_mutex);
	return 0;

err_pm:
	clear_bit(CH397_DEV_OPEN, &dev->flags);
	netif_stop_queue(ndev);
	usb_autopm_put_interface(dev->intf);
err_free:
	ch397_free_all_resources(dev);
out_unlock:
	mutex_unlock(&dev->dev_mutex);
	return ret;
}

static int ch397_stop(struct net_device *ndev)
{
	struct ch397 *dev = netdev_priv(ndev);
	int pm;

	mutex_lock(&dev->dev_mutex);

	netif_dbg(dev, ifup, dev->ndev, "stop device");

#ifdef CONFIG_PM_SLEEP
	unregister_pm_notifier(&dev->pm_notifier);
	if (dev->pm_notifier_held) {
		usb_autopm_put_interface(dev->intf);
		dev->pm_notifier_held = false;
	}
#endif
	tasklet_disable(&dev->tx_bh);
	clear_bit(CH397_DEV_OPEN, &dev->flags);
	usb_kill_urb(dev->intr_urb);

	cancel_delayed_work_sync(&dev->wq);

	napi_disable(&dev->napi);
	netif_stop_queue(ndev);

	/* to not race resume */
	pm = usb_autopm_get_interface(dev->intf);
	if (pm < 0 || ch397_dev_inaccessible(dev)) {
		ch397_drop_tx_skb(dev);
		ch397_kill_tx_urb(dev);
		ch397_stop_rx(dev);
	} else
		ch397_disable(dev);

	netif_info(dev, ifdown, dev->ndev,
		   "ch397 stop stats: rx/tx %lu/%lu, errs %lu/%lu\n",
		   ndev->stats.rx_packets, ndev->stats.tx_packets,
		   ndev->stats.rx_errors, ndev->stats.tx_errors);

	if (!pm)
		usb_autopm_put_interface(dev->intf);

	ch397_free_all_resources(dev);

	mutex_unlock(&dev->dev_mutex);
	return 0;
}

static netdev_tx_t ch397_start_xmit(struct sk_buff *skb,
				    struct net_device *ndev)
{
	struct ch397 *dev = netdev_priv(ndev);

	if (skb)
		skb_tx_timestamp(skb);

	skb_queue_tail(&dev->txq_pend, skb);

	if (!list_empty(&dev->txq_free)) {
		if (test_bit(CH397_DEV_CHOOSE_SUSPEND, &dev->flags)) {
			set_bit(CH397_SCHEDULE_TASKLET, &dev->flags);
			schedule_delayed_work(&dev->wq, 0);
		} else {
			usb_mark_last_busy(dev->udev);
			tasklet_schedule(&dev->tx_bh);
		}
	} else if (skb_queue_len(&dev->txq_pend) > dev->tx_qlen) {
		netif_stop_queue(ndev);
	}

	return NETDEV_TX_OK;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static void ch397_tx_timeout(struct net_device *ndev, unsigned int txqueue)
#else
static void ch397_tx_timeout(struct net_device *ndev)
#endif
{
	struct ch397 *dev = netdev_priv(ndev);

	netif_warn(dev, tx_err, ndev, "ch397 tx timeout!\n");

	usb_queue_reset_device(dev->intf);
}

static int ch397_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd)
{
	struct ch397 *dev = netdev_priv(ndev);
	int ret;

	if (ch397_dev_inaccessible(dev))
		return -ENODEV;

	ret = usb_autopm_get_interface(dev->intf);
	if (ret < 0)
		return ret;

	mutex_lock(&dev->phy_mutex);
	ret = generic_mii_ioctl(&dev->mii, if_mii(rq), cmd, NULL);
	mutex_unlock(&dev->phy_mutex);

	usb_autopm_put_interface(dev->intf);

	return ret;
}

static int ch397_get_dev_info(struct ch397 *dev, void *dev_info)
{
	u8 buf[8];
	int ret;

	memset(buf, 0, sizeof(buf));
	ret = ch397_read(dev, CMD_GET_INFO, 0x00, 8, buf);
	if (ret < 0) {
		netdev_err(dev->ndev, "Error getting dev info.\n");
		return ret;
	}
	memcpy(dev_info, buf, 8);

	return ret;
}

static int ch397_get_mac_address(struct ch397 *dev, void *mac)
{
	int ret;

	ret = ch397_read(dev, CMD_RD_REG, dev->regs->mac_addrl, 4, mac);
	if (ret < 0) {
		netdev_err(dev->ndev, "Error getting MAC_L address.\n");
		return ret;
	}

	ret = ch397_read(dev, CMD_RD_REG, dev->regs->mac_addrh, 2,
			 mac + 4);
	if (ret < 0) {
		netdev_err(dev->ndev, "Error getting MAC_H address.\n");
		return ret;
	}

	return ret;
}

static int __ch397_set_mac_address(struct ch397 *dev, const u8 *addr)
{
	u8 dev_addr[ETH_ALEN + 2] = {};
	int ret;

	memcpy(dev_addr, addr, ETH_ALEN);

	ret = ch397_write(dev, CMD_WR_REG, dev->regs->mac_addrl, 4,
			  dev_addr);
	if (ret < 0)
		return ret;

	return ch397_write(dev, CMD_WR_REG, dev->regs->mac_addrh, 4,
			   dev_addr + 4);
}

static int ch397_set_mac_address(struct net_device *ndev, void *p)
{
	struct sockaddr *addr = p;
	struct ch397 *dev = netdev_priv(ndev);
	int ret;

	if (!is_valid_ether_addr(addr->sa_data)) {
		dev_err(&ndev->dev,
			"not setting invalid mac address %pM\n",
			addr->sa_data);
		return -EINVAL;
	}

	if (ch397_dev_inaccessible(dev))
		return -ENODEV;

	ret = usb_autopm_get_interface(dev->intf);
	if (ret < 0)
		return ret;

	mutex_lock(&dev->dev_mutex);

	ret = __ch397_set_mac_address(dev, addr->sa_data);
	if (!ret) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0))
		eth_hw_addr_set(ndev, addr->sa_data);
#else
		memcpy(ndev->dev_addr, addr->sa_data, ndev->addr_len);
#endif
	}

	mutex_unlock(&dev->dev_mutex);
	usb_autopm_put_interface(dev->intf);

	return ret;
}

static u32 swap_bytes(u32 value)
{
	u32 byte1 = (value & 0xFFFF0000) >> 16;
	u32 byte2 = (value & 0x0000FFFF) << 16;

	return byte1 | byte2;
}

static void _ch397_set_rx_mode(struct net_device *ndev)
{
	struct ch397 *dev = netdev_priv(ndev);
	struct ch397_ndev_cfg *cfg = &dev->ndev_cfg;
	struct netdev_hw_addr *ha;
	u32 val_set = 0, val_clear = 0;
	u32 crc_bits, hash[2], mac_cfg, multi_filter[2];
	bool was_running;
	int ret;

	was_running = !netif_queue_stopped(ndev);
	if (was_running)
		netif_stop_queue(ndev);

	multi_filter[0] = 0;
	multi_filter[1] = 0;

	val_clear |= CH398_RX_RA | CH398_RX_PAM | CH398_RX_PUF |
		     CH398_RX_PBD | CH398_RX_PLM;

	if (ndev->flags & IFF_PROMISC) {
		val_set |= CH398_RX_RA;
	} else {
		val_set |= CH398_RX_PUF | CH398_RX_PBD;

		if ((ndev->flags & IFF_ALLMULTI) ||
		    netdev_mc_count(ndev) > CH397_MAX_MCAST) {
			val_set |= CH398_RX_PAM;
		} else if (!netdev_mc_empty(ndev)) {
			val_set |= CH398_RX_PLM;

			netdev_for_each_mc_addr(ha, ndev) {
				crc_bits =
					crc32_le(~0, ha->addr, ETH_ALEN);
				crc_bits = bitrev32((~crc_bits)) >> 26;
				multi_filter[crc_bits >> 5] |=
					1 << (crc_bits & 31);
			}
		}
	}

	val_clear &= ~val_set;

	if (dev->version == WCH_CHIP_VER_02) {
		hash[0] = multi_filter[1];
		hash[1] = multi_filter[0];
	} else {
		hash[0] = swap_bytes(multi_filter[1]);
		hash[1] = swap_bytes(multi_filter[0]);
	}

	ret = ch397_write_reg(dev, CMD_WR_REG, dev->regs->mac_hthr,
			      sizeof(__le32), hash[0]);
	if (ret < 0) {
		netdev_err(dev->ndev,
			   "%s, Error setting mac hash high reg.\n",
			   __func__);
		goto out;
	}

	ret = ch397_write_reg(dev, CMD_WR_REG, dev->regs->mac_htlr,
			      sizeof(__le32), hash[1]);
	if (ret < 0) {
		netdev_err(dev->ndev,
			   "%s, Error setting mac hash low reg.\n",
			   __func__);
		goto out;
	}

	ret = ch397_read_reg(dev, CMD_RD_REG, dev->regs->mac_cfg,
			     sizeof(__le32), &mac_cfg);
	if (ret < 0) {
		netdev_err(dev->ndev,
			   "%s, Error reading mac configure value.\n",
			   __func__);
		goto out;
	}
	if (cfg->flow_ctrl)
		mac_cfg |= CH397_FLOWCTRL_EN;
	else
		mac_cfg &= ~CH397_FLOWCTRL_EN;
	mac_cfg = (mac_cfg | val_set) & ~val_clear;

	if (cfg->dev_info.val_l >= 0x37) {
		ret = ch397_write(dev, CMD_WR_ETH_MACCFG, mac_cfg, 0,
				  NULL);
		if (ret < 0) {
			netdev_err(
				dev->ndev,
				"%s, Error setting mac-configure value.\n",
				__func__);
			goto out;
		}
	} else {
		ret = ch397_write_reg(dev, CMD_WR_REG, dev->regs->mac_cfg,
				      sizeof(__le32), mac_cfg);
		if (ret < 0) {
			netdev_err(
				dev->ndev,
				"%s, Error setting mac configure value.\n",
				__func__);
			goto out;
		}
	}

out:
	if (was_running)
		netif_wake_queue(ndev);
}

static void ch397_set_rx_mode(struct net_device *ndev)
{
	struct ch397 *dev = netdev_priv(ndev);

	if (netif_carrier_ok(ndev)) {
		set_bit(CH397_SET_RX_MODE, &dev->flags);
		schedule_delayed_work(&dev->wq, 0);
	}
}

static int ch398_set_mtu(struct ch397 *dev, int mtu)
{
	u32 mac_cfg;
	int ret;

	if (mtu > WCH_MIN_JUMBO_SIZE) {
		ret = ch397_read_reg(dev, CMD_RD_REG, dev->regs->mac_cfg,
				     sizeof(__le32), &mac_cfg);
		if (ret < 0)
			return ret;
		mac_cfg |= CH397_ENABLE_RWTE;
		ret = ch397_write_reg(dev, CMD_WR_REG, dev->regs->mac_cfg,
				      sizeof(__le32), mac_cfg);
	} else {
		ret = ch397_read_reg(dev, CMD_RD_REG, dev->regs->mac_cfg,
				     sizeof(__le32), &mac_cfg);
		if (ret < 0)
			return ret;
		mac_cfg &= ~CH397_ENABLE_RWTE;
		ret = ch397_write_reg(dev, CMD_WR_REG, dev->regs->mac_cfg,
				      sizeof(__le32), mac_cfg);
	}

	return ret;
}

static int ch397_change_mtu(struct net_device *ndev, int new_mtu)
{
	struct ch397 *dev = netdev_priv(ndev);
	int ret;

	ret = usb_autopm_get_interface(dev->intf);
	if (ret < 0)
		return ret;

	mutex_lock(&dev->dev_mutex);

	if (new_mtu < ETH_MIN_MTU) {
		ret = -EINVAL;
		goto out;
	}

	if (new_mtu > dev->max_mtu) {
		ret = -EINVAL;
		goto out;
	}

	if (dev->dev_ops->set_mtu) {
		ret = dev->dev_ops->set_mtu(dev, new_mtu);
		if (ret < 0)
			goto out;
	}

	ndev->mtu = new_mtu;
	ch397_set_tx_qlen(dev);

	if (netif_running(ndev)) {
		if (netif_carrier_ok(ndev)) {
			netif_stop_queue(ndev);
			napi_disable(&dev->napi);
			tasklet_disable(&dev->tx_bh);
			ch397_disable(dev);
			_ch397_set_rx_mode(ndev);
			ch397_start_rx(dev);
			tasklet_enable(&dev->tx_bh);
			napi_enable(&dev->napi);
			/* ch397_start_rx() may park a failed RX submit on
			 * rxq_done while NAPI is disabled.
			 */
			if (!list_empty(&dev->rxq_done))
				napi_schedule(&dev->napi);
			netif_wake_queue(ndev);
		}
	}

out:
	mutex_unlock(&dev->dev_mutex);
	usb_autopm_put_interface(dev->intf);
	return ret;
}

static int ch397_rx_vlan_en(struct ch397 *dev, bool enable)
{
	u32 val;
	int ret;

	switch (dev->version) {
	case WCH_CHIP_VER_02:
		ret = ch397_read_reg(dev, CMD_RD_REG, CH398_ETH_MISC_CFG,
				     sizeof(__le32), &val);
		if (ret < 0)
			return ret;

		if (enable)
			val |= CH398_RX_STRIP_VLANEN;
		else
			val &= ~CH398_RX_STRIP_VLANEN;
		return ch397_write_reg(dev, CMD_WR_REG, CH398_ETH_MISC_CFG,
				       sizeof(__le32), val);
	default:
		return -EOPNOTSUPP;
	}
}

static int ch397_set_features(struct net_device *ndev,
			      netdev_features_t features)
{
	netdev_features_t changed = features ^ ndev->features;
	struct ch397 *dev = netdev_priv(ndev);
	bool restart = false;
	int ret = 0;

	if (!(changed & NETIF_F_HW_VLAN_CTAG_RX))
		return 0;

	ret = usb_autopm_get_interface(dev->intf);
	if (ret < 0)
		return ret;

	mutex_lock(&dev->dev_mutex);

	if (netif_running(ndev) && netif_carrier_ok(ndev)) {
		restart = true;
		netif_stop_queue(ndev);
		napi_disable(&dev->napi);
		tasklet_disable(&dev->tx_bh);
		ch397_disable(dev);
	}

	ret = ch397_rx_vlan_en(dev, features & NETIF_F_HW_VLAN_CTAG_RX);

	if (restart) {
		_ch397_set_rx_mode(ndev);
		ch397_start_rx(dev);
		tasklet_enable(&dev->tx_bh);
		napi_enable(&dev->napi);
		if (!list_empty(&dev->rxq_done))
			napi_schedule(&dev->napi);
		netif_wake_queue(ndev);
	}

	mutex_unlock(&dev->dev_mutex);

	usb_autopm_put_interface(dev->intf);

	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0))
static netdev_features_t ch397_features_check(struct sk_buff *skb,
					      struct net_device *ndev,
					      netdev_features_t features)
{
	struct ch397 *dev = netdev_priv(ndev);
	u32 gso_type = skb_shinfo(skb)->gso_type;
	u32 mss = skb_shinfo(skb)->gso_size;

	if (dev->version != WCH_CHIP_VER_02 || !mss)
		return features;

	if (!(gso_type & (SKB_GSO_TCPV4 | SKB_GSO_TCPV6))) {
		features &= ~NETIF_F_GSO_MASK;
		return features;
	}

	if (ALIGN(skb->len + CH397_TX_OVERHEAD, CH397_TX_ALIGN) >
	    dev->tx_urb_size)
		features &= ~NETIF_F_GSO_MASK;
	else if (skb_transport_offset(skb) + tcp_hdrlen(skb) >
		 TX_TCPHO_MASK)
		features &= ~NETIF_F_GSO_MASK;

	return features;
}
#endif

static const struct net_device_ops ch397_netdev_ops = {
	.ndo_open = ch397_open,
	.ndo_stop = ch397_stop,
	.ndo_start_xmit = ch397_start_xmit,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
	.ndo_tx_timeout = ch397_tx_timeout,
#else
	.ndo_tx_timeout = ch397_tx_timeout,
#endif
	.ndo_change_mtu = ch397_change_mtu,
	.ndo_validate_addr = eth_validate_addr,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	.ndo_eth_ioctl = ch397_ioctl,
#else
	.ndo_do_ioctl = ch397_ioctl,
#endif
	.ndo_set_mac_address = ch397_set_mac_address,
	.ndo_set_rx_mode = ch397_set_rx_mode,
	.ndo_set_features = ch397_set_features,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0))
	.ndo_features_check = ch397_features_check,
#endif
};

static int ch397_set_speed(struct ch397 *dev, u8 autoneg, u16 speed,
			   u8 duplex, u32 advertising)
{
	struct net_device *ndev = dev->ndev;
	u32 phy_id = dev->mii.phy_id;
	u16 bmcr = 0;
	int anar;
	u16 new_anar;
	u16 adv_mask;

	if (autoneg != AUTONEG_ENABLE && autoneg != AUTONEG_DISABLE)
		return -EINVAL;

	if (autoneg == AUTONEG_DISABLE) {
		if (duplex != DUPLEX_HALF && duplex != DUPLEX_FULL)
			return -EINVAL;

		switch (speed) {
		case SPEED_10:
			bmcr = BMCR_SPEED10;
			break;
		case SPEED_100:
			bmcr = BMCR_SPEED100;
			break;
		default:
			return -EINVAL;
		}

		if (duplex == DUPLEX_FULL) {
			bmcr |= BMCR_FULLDPLX;
			dev->mii.full_duplex = 1;
		} else {
			dev->mii.full_duplex = 0;
		}

		dev->mii.force_media = 1;

		ch397_mdio_write(ndev, phy_id, MII_BMCR, bmcr);

		netdev_dbg(
			ndev,
			"set speed forced: %uMbps %s-duplex, bmcr=0x%04x\n",
			speed, (duplex == DUPLEX_FULL) ? "full" : "half",
			bmcr);
		return 0;
	}

	anar = ch397_mdio_read(ndev, phy_id, MII_ADVERTISE);
	if (anar < 0)
		return anar;

	adv_mask = ADVERTISE_10HALF | ADVERTISE_10FULL |
		   ADVERTISE_100HALF | ADVERTISE_100FULL;

	new_anar = (u16)anar & ~adv_mask;

	if (advertising & PHY_ADVERTISED_10_HALF)
		new_anar |= ADVERTISE_10HALF;
	if (advertising & PHY_ADVERTISED_10_FULL)
		new_anar |= ADVERTISE_10FULL;
	if (advertising & PHY_ADVERTISED_100_HALF)
		new_anar |= ADVERTISE_100HALF;
	if (advertising & PHY_ADVERTISED_100_FULL)
		new_anar |= ADVERTISE_100FULL;

	if (!(new_anar & adv_mask)) {
		if (duplex != DUPLEX_HALF && duplex != DUPLEX_FULL)
			return -EINVAL;

		switch (speed) {
		case SPEED_10:
			if (duplex == DUPLEX_FULL)
				new_anar |= ADVERTISE_10FULL;
			else
				new_anar |= ADVERTISE_10HALF;
			break;
		case SPEED_100:
			if (duplex == DUPLEX_FULL)
				new_anar |= ADVERTISE_100FULL;
			else
				new_anar |= ADVERTISE_100HALF;
			break;
		default:
			return -EINVAL;
		}
	}

	new_anar &= ~ADVERTISE_SLCT;
	new_anar |= ADVERTISE_CSMA;

	if ((u16)anar != new_anar) {
		ch397_mdio_write(ndev, phy_id, MII_ADVERTISE, new_anar);
	}

	bmcr = BMCR_ANENABLE | BMCR_ANRESTART;

	dev->mii.force_media = 0;
	dev->mii.full_duplex = 0;

	ch397_mdio_write(ndev, phy_id, MII_BMCR, bmcr);

	netdev_dbg(
		ndev,
		"set speed autoneg: advertising=0x%08x, anar=0x%04x, bmcr=0x%04x\n",
		advertising, new_anar, bmcr);

	return 0;
}

static int ch398_set_speed(struct ch397 *dev, u8 autoneg, u16 speed,
			   u8 duplex, u32 advertising)
{
	struct net_device *ndev = dev->ndev;
	u32 phy_id = dev->mii.phy_id;
	u32 support, req;
	u16 bmcr = 0;
	int anar, gbcr, old;

	if (autoneg != AUTONEG_ENABLE && autoneg != AUTONEG_DISABLE)
		return -EINVAL;

	support = PHY_ADVERTISED_10_FULL | PHY_ADVERTISED_100_FULL |
		  PHY_ADVERTISED_1000_FULL;

	if (autoneg == AUTONEG_DISABLE) {
		if (duplex != DUPLEX_FULL)
			return -EINVAL;

		switch (speed) {
		case SPEED_10:
			bmcr = BMCR_FULLDPLX;
			break;
		case SPEED_100:
			bmcr = BMCR_FULLDPLX | BMCR_SPEED100;
			break;
		case SPEED_1000:
			if (!dev->mii.supports_gmii)
				return -EINVAL;

			anar = ch397_mdio_read(ndev, phy_id,
					       MII_ADVERTISE);
			if (anar < 0)
				return anar;
			old = anar;
			anar &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL |
				  ADVERTISE_100HALF | ADVERTISE_100FULL);
			if (anar != old)
				ch397_mdio_write(ndev, phy_id,
						 MII_ADVERTISE, anar);

			gbcr = ch397_mdio_read(ndev, phy_id, MII_CTRL1000);
			if (gbcr < 0)
				return gbcr;
			old = gbcr;
			gbcr &= ~(ADVERTISE_1000HALF | ADVERTISE_1000FULL);
			gbcr |= ADVERTISE_1000FULL;
			if (gbcr != old)
				ch397_mdio_write(ndev, phy_id,
						 MII_CTRL1000, gbcr);

			/* Keep autoneg enabled and advertise
			 * 1000BASE-T full duplex only.
			 */
			bmcr = BMCR_FULLDPLX | BMCR_ANENABLE |
			       BMCR_ANRESTART;
			break;
		default:
			return -EINVAL;
		}

		dev->mii.full_duplex = 1;
		dev->mii.force_media = 1;
		dev->mii.advertising = 0;
		if (speed == SPEED_1000)
			dev->mii.advertising = PHY_ADVERTISED_1000_FULL;
	} else {
		req = advertising & support;
		if (!req)
			return -EINVAL;

		anar = ch397_mdio_read(ndev, phy_id, MII_ADVERTISE);
		if (anar < 0)
			return anar;
		anar &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL |
			  ADVERTISE_100HALF | ADVERTISE_100FULL);

		if (req & PHY_ADVERTISED_10_FULL)
			anar |= ADVERTISE_10FULL;
		if (req & PHY_ADVERTISED_100_FULL)
			anar |= ADVERTISE_100FULL;

		ch397_mdio_write(ndev, phy_id, MII_ADVERTISE, anar);

		if (dev->mii.supports_gmii) {
			gbcr = ch397_mdio_read(ndev, phy_id, MII_CTRL1000);
			if (gbcr < 0)
				return gbcr;
			gbcr &= ~(ADVERTISE_1000HALF | ADVERTISE_1000FULL);

			if (req & PHY_ADVERTISED_1000_FULL)
				gbcr |= ADVERTISE_1000FULL;

			ch397_mdio_write(ndev, phy_id, MII_CTRL1000, gbcr);
		} else if (req & PHY_ADVERTISED_1000_FULL) {
			return -EINVAL;
		}

		bmcr = BMCR_ANENABLE | BMCR_ANRESTART;
		dev->mii.full_duplex = 1;
		dev->mii.force_media = 0;
		dev->mii.advertising = req;
	}

	ch397_mdio_write(ndev, phy_id, MII_BMCR, bmcr);

	return 0;
}

static int ch397_link_reset(struct ch397 *dev)
{
	struct net_device *ndev = dev->ndev;
	struct ch397_ndev_cfg *cfg = &dev->ndev_cfg;
	int lpa;

	lpa = ch397_mdio_read(ndev, dev->mii.phy_id, MII_LPA);
	if (lpa < 0)
		netdev_dbg(ndev, "failed to read LPA: %d\n", lpa);

	if (cfg->link) {
		if (!netif_carrier_ok(ndev)) {
			ch397_set_tx_qlen(dev);
			netif_stop_queue(ndev);
			napi_disable(&dev->napi);
			netif_carrier_on(ndev);
			clear_bit(CH397_SET_RX_MODE, &dev->flags);
			_ch397_set_rx_mode(ndev);
			ch397_start_rx(dev);
			napi_enable(&dev->napi);
			/* Kick deferred RX work after NAPI is enabled again.
			 * ch397_start_rx() queues restart-time submit failures
			 * onto rxq_done.
			 */
			if (!list_empty(&dev->rxq_done))
				napi_schedule(&dev->napi);
			netif_wake_queue(ndev);
			netdev_info(
				ndev,
				"link up, %uMbps, %s-duplex, autoneg-%s, lpa 0x%04X\n",
				cfg->link_speed,
				(cfg->link_duplex == DUPLEX_FULL) ?
					"full" :
					"half",
				(cfg->autoneg == AUTONEG_ENABLE) ? "on" :
								   "off",
				(lpa < 0) ? 0 : lpa);
		} else if (netif_queue_stopped(ndev) &&
			   skb_queue_len(&dev->txq_pend) < dev->tx_qlen) {
			netif_wake_queue(ndev);
		}
	} else {
		if (netif_carrier_ok(ndev)) {
			netif_carrier_off(ndev);
			tasklet_disable(&dev->tx_bh);
			napi_disable(&dev->napi);
			ch397_disable(dev);
			napi_enable(&dev->napi);
			tasklet_enable(&dev->tx_bh);
			netdev_info(dev->ndev, "link down\n");
		}
	}

	return 0;
}

/* Serialize deferred link, RX mode, and TX scheduling updates in process context. */
static void ch397_work_func(struct work_struct *work)
{
	struct ch397 *dev = container_of(work, struct ch397, wq.work);
	struct ch397_ndev_cfg *cfg = &dev->ndev_cfg;

	if (ch397_dev_inaccessible(dev) || !netif_running(dev->ndev))
		return;

	if (usb_autopm_get_interface(dev->intf) < 0)
		return;

	if (!test_bit(CH397_DEV_OPEN, &dev->flags))
		goto out;

	if (!mutex_trylock(&dev->dev_mutex)) {
		schedule_delayed_work(&dev->wq, 0);
		goto out;
	}

	if (test_and_clear_bit(CH397_LINK_RESET, &dev->flags))
		ch397_link_reset(dev);

	if (test_and_clear_bit(CH397_LINK_CHG, &dev->flags)) {
		mutex_lock(&dev->phy_mutex);
		dev->dev_ops->set_speed(dev, cfg->autoneg, cfg->speed,
					cfg->duplex, cfg->advertising);
		mutex_unlock(&dev->phy_mutex);
	}

	if (test_and_clear_bit(CH397_SET_RX_MODE, &dev->flags))
		_ch397_set_rx_mode(dev->ndev);

	/* don't schedule tasket before linking */
	if (test_and_clear_bit(CH397_SCHEDULE_TASKLET, &dev->flags) &&
	    netif_carrier_ok(dev->ndev))
		tasklet_schedule(&dev->tx_bh);

	if (test_and_clear_bit(CH397_RX_EPROTO, &dev->flags) &&
	    !list_empty(&dev->rxq_done))
		napi_schedule(&dev->napi);

	if (dev->flags)
		netdev_dbg(dev->ndev, "kevent done, flags = 0x%lx\n",
			   dev->flags);

	mutex_unlock(&dev->dev_mutex);

out:
	usb_autopm_put_interface(dev->intf);
}

static int ch397_init(struct ch397 *dev)
{
	struct ch397_ndev_cfg *cfg = &dev->ndev_cfg;
	struct ch397_intr_stats *event = &dev->ch397_intr;
	u16 pid = le16_to_cpu(dev->udev->descriptor.idProduct);
	u32 val;
	int ret = 0;

	if (ch397_dev_inaccessible(dev))
		return -ENODEV;

	if (ch397_read_reg(dev, CMD_RD_REG, dev->regs->mac_cfg,
			   sizeof(__le32), &val) < 0) {
		printk(KERN_ERR "Error getting mac configure value.\n");
		ret = -ENODEV;
		return ret;
	}

	dev->n_tx_urbs = TX_HS_URB_NUM;
	dev->n_rx_urbs = RX_HS_URB_NUM;
	dev->tx_urb_size = CH397_TX_URB_SIZE;
	dev->tx_single_packet = false;
	if (pid == 0x5394 || pid == 0x5395)
		dev->tx_single_packet = true;
	dev->rx_urb_size = CH397_RX_URB_SIZE;
	dev->tx_qlen_scale = 4;
	dev->rxq_pend_limit = 128;

	cfg->link = false;
	cfg->autoneg = AUTONEG_ENABLE;
	cfg->speed = SPEED_100;
	cfg->duplex = DUPLEX_FULL;
	cfg->link = false;
	cfg->phy_wol = true;
	cfg->flow_ctrl = FLOW_CTRL_RX | FLOW_CTRL_TX;
	cfg->advertising =
		PHY_ADVERTISED_10_HALF | PHY_ADVERTISED_10_FULL |
		PHY_ADVERTISED_100_HALF | PHY_ADVERTISED_100_FULL;

	memset(event, 0, sizeof(*event));

	return ret;
}

static int ch398_init(struct ch397 *dev)
{
	struct ch397_ndev_cfg *cfg = &dev->ndev_cfg;
	struct ch397_intr_stats *event = &dev->ch398_intr;
	u32 val;
	int ret = 0;

	if (ch397_dev_inaccessible(dev))
		return -ENODEV;

	msleep(10);

	if (ch397_read_reg(dev, CMD_RD_REG, dev->regs->mac_cfg,
			   sizeof(__le32), &val) < 0) {
		printk(KERN_ERR "%s, Error getting mac configure value.\n",
		       __func__);
		ret = -ENODEV;
		return ret;
	}

	switch (dev->udev->speed) {
	case USB_SPEED_SUPER:
		dev->n_tx_urbs = TX_SS_URB_NUM;
		dev->n_rx_urbs = RX_SS_URB_NUM;
		dev->tx_urb_size = TX_SS_URB_SIZE;
		dev->rx_urb_size = RX_SS_URB_SIZE;
		dev->tx_single_packet = false;
		dev->hs_mode = false;
		dev->tx_qlen_scale = 2;
		dev->rxq_pend_limit = 512;
		break;
	case USB_SPEED_HIGH:
		dev->n_tx_urbs = TX_HS_URB_NUM;
		dev->n_rx_urbs = RX_HS_URB_NUM;
		dev->tx_urb_size = TX_HS_URB_SIZE;
		dev->rx_urb_size = RX_HS_URB_SIZE;
		dev->tx_single_packet = false;
		dev->hs_mode = true;
		dev->tx_qlen_scale = 2;
		dev->rxq_pend_limit = 512;
		break;
	case USB_SPEED_FULL:
		dev->n_tx_urbs = TX_FS_URB_NUM;
		dev->n_rx_urbs = RX_FS_URB_NUM;
		dev->tx_urb_size = TX_FS_URB_SIZE;
		dev->rx_urb_size = RX_FS_URB_SIZE;
		dev->tx_single_packet = false;
		dev->hs_mode = true;
		dev->tx_qlen_scale = 1;
		dev->rxq_pend_limit = 64;
		break;
	default:
		netdev_warn(dev->ndev, "USB bus speed not supported\n");
		ret = -EIO;
		return ret;
	}

	cfg->link = false;
	cfg->phy_wol = true;
	cfg->autoneg = AUTONEG_ENABLE;
	cfg->speed = SPEED_1000;
	cfg->duplex = DUPLEX_FULL;
	cfg->flow_ctrl = FLOW_CTRL_RX | FLOW_CTRL_TX;
	cfg->advertising = PHY_ADVERTISED_10_FULL |
			   PHY_ADVERTISED_100_FULL |
			   PHY_ADVERTISED_1000_FULL;

	memset(event, 0, sizeof(*event));

	return ret;
}

static int wch_hw_init(struct ch397 *dev)
{
	u16 pid = le16_to_cpu(dev->udev->descriptor.idProduct);
	u8 version;

	switch (pid) {
	case 0x5394:
	case 0x5395:
	case 0x5396:
	case 0x5397:
		version = WCH_CHIP_VER_01;
		dev->regs = &ch397_regs;
		dev->dev_ops = &ch397_ops;
		break;
	case 0x5398:
		version = WCH_CHIP_VER_02;
		dev->regs = &ch398_regs;
		dev->dev_ops = &ch398_ops;
		break;
	case 0xe398:
		version = WCH_CHIP_VER_02;
		dev->regs = &ch398_regs;
		dev->dev_ops = &ch398_ops;
		break;
	default:
		version = WCH_CHIP_VER_UNKNOWN;
		dev_err(&dev->udev->dev, "Unknown version 0x%04x\n", pid);
		return -ENODEV;
	}

	dev->version = version;

	return 0;
}

static int ch397_set_flowctrl(struct ch397 *dev)
{
	int anar;

	anar = ch397_mdio_read(dev->ndev, dev->mii.phy_id, MII_ADVERTISE);
	if (anar < 0)
		return anar;

	anar |= (ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM);
	ch397_mdio_write(dev->ndev, dev->mii.phy_id, MII_ADVERTISE, anar);

	return 0;
}

static int wch_ethernet_init(struct ch397 *dev)
{
	struct ch397_ndev_cfg *cfg = &dev->ndev_cfg;
	int eee_adv;
	int max_mtu = ETH_DATA_LEN;
	u8 mac[ETH_ALEN] = {};
	u32 val;
	int ret = 0;

	msleep(100);

	if (ch397_read_reg(dev, CMD_RD_REG, dev->regs->mac_cfg,
			   sizeof(__le32), &val) < 0) {
		printk(KERN_ERR "Error getting mac configure value.\n");
		ret = -ENODEV;
		return ret;
	}

	if ((ret = ch397_get_mac_address(dev, mac)) < 0) {
		dev_err(&dev->udev->dev, "Error reading MAC address\n");
		ret = -ENODEV;
		return ret;
	}

	if (is_valid_ether_addr(mac)) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0))
		eth_hw_addr_set(dev->ndev, mac);
#else
		memcpy(dev->ndev->dev_addr, mac, ETH_ALEN);
#endif
	} else {
		dev_warn(
			&dev->udev->dev,
			"Invalid MAC address read from device, using random\n");
		eth_hw_addr_random(dev->ndev);
		ret = __ch397_set_mac_address(dev, dev->ndev->dev_addr);
		if (ret < 0) {
			dev_err(&dev->udev->dev,
				"Error writing random MAC address\n");
			return ret;
		}
	}

	switch (dev->version) {
	case WCH_CHIP_VER_01:
		max_mtu = ETH_DATA_LEN;
		dev->mii.supports_gmii = 0;
		break;
	case WCH_CHIP_VER_02:
		max_mtu = size_to_mtu(9 * 1024);
		dev->ndev->features |=
			NETIF_F_RXCSUM | NETIF_F_IP_CSUM | NETIF_F_SG |
			NETIF_F_TSO | NETIF_F_FRAGLIST |
			NETIF_F_IPV6_CSUM | NETIF_F_TSO6 |
			NETIF_F_HW_VLAN_CTAG_RX | NETIF_F_HW_VLAN_CTAG_TX;
		dev->ndev->hw_features =
			NETIF_F_RXCSUM | NETIF_F_IP_CSUM | NETIF_F_SG |
			NETIF_F_TSO | NETIF_F_FRAGLIST |
			NETIF_F_IPV6_CSUM | NETIF_F_TSO6 |
			NETIF_F_HW_VLAN_CTAG_RX | NETIF_F_HW_VLAN_CTAG_TX;
		dev->ndev->vlan_features =
			NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_TSO |
			NETIF_F_HIGHDMA | NETIF_F_FRAGLIST |
			NETIF_F_IPV6_CSUM | NETIF_F_TSO6;
		dev->mii.supports_gmii = 1;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 19, 0))
		netif_set_tso_max_size(dev->ndev, WCH_MAX_TSO_SIZE);
#else
		netif_set_gso_max_size(dev->ndev, WCH_MAX_TSO_SIZE);
#endif
		eee_adv = ch397_mdio_read(dev->ndev, dev->mii.phy_id,
					  CH398_PHY_AUX_CTRL_REG);
		if (eee_adv < 0)
			return eee_adv;
		eee_adv |=
			(CH398_PHY_EEE_1000M_ADV | CH398_PHY_EEE_100M_ADV);
		ch397_mdio_write(dev->ndev, dev->mii.phy_id,
				 CH398_PHY_AUX_CTRL_REG, eee_adv);
		cfg->eee_enabled = true;
		cfg->eee_active = false;
		cfg->eee_advertised = (MDIO_EEE_1000T | MDIO_EEE_100TX);
		cfg->rx_coalesce_usecs = CH398_RX_EARLY_TIME_US;
		cfg->rx_max_coalesce_bytes = CH398_RX_EARLY_SIZE_BYTES;
		break;
	default:
		dev_err(&dev->intf->dev, "Unknown Device\n");
		ret = -EIO;
		break;
	}

	dev->max_mtu = min_t(int, max_mtu, ch397_max_tx_mtu(dev));
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0))
	dev->ndev->min_mtu = ETH_MIN_MTU;
	dev->ndev->max_mtu = dev->max_mtu;
#endif

	if (ch397_get_dev_info(dev, &cfg->dev_info) < 0) {
		ret = -ENODEV;
		goto out;
	}

	dev->mii.dev = dev->ndev;
	dev->mii.mdio_read = ch397_mdio_read;
	dev->mii.mdio_write = ch397_mdio_write;
	dev->mii.phy_id = 0;
	dev->mii.phy_id_mask = 0x1f;
	dev->mii.reg_num_mask = 0x1f;

	ch397_set_tx_qlen(dev);

	dev->rx_copybreak = CH397_RX_COPYBREAK;
	dev->rx_pending = 10 * dev->n_rx_urbs;

	ch397_set_flowctrl(dev);

out:
	return ret;
}

static int ch397_check_endpoints(struct ch397 *dev)
{
	struct usb_host_interface *alt = dev->intf->cur_altsetting;
	const struct usb_endpoint_descriptor *bulk_in = NULL;
	const struct usb_endpoint_descriptor *bulk_out = NULL;
	const struct usb_endpoint_descriptor *intr_in = NULL;
	struct usb_device *udev = dev->udev;
	int i;

	if (!alt)
		return -ENODEV;

	for (i = 0; i < alt->desc.bNumEndpoints; i++) {
		const struct usb_endpoint_descriptor *desc;

		desc = &alt->endpoint[i].desc;
		if (!bulk_in && usb_endpoint_is_bulk_in(desc))
			bulk_in = desc;
		else if (!bulk_out && usb_endpoint_is_bulk_out(desc))
			bulk_out = desc;
		else if (!intr_in && usb_endpoint_is_int_in(desc))
			intr_in = desc;
	}

	if (!bulk_in || !bulk_out || !intr_in) {
		dev_err(&dev->intf->dev,
			"missing endpoint: bulk-in %d, bulk-out %d, intr-in %d\n",
			!!bulk_in, !!bulk_out, !!intr_in);
		return -ENODEV;
	}

	dev->pipe_ctrl_in = usb_rcvctrlpipe(udev, CH397_CTRL_PIPE);
	dev->pipe_ctrl_out = usb_sndctrlpipe(udev, CH397_CTRL_PIPE);
	dev->pipe_in =
		usb_rcvbulkpipe(udev, bulk_in->bEndpointAddress &
					      USB_ENDPOINT_NUMBER_MASK);
	dev->pipe_out =
		usb_sndbulkpipe(udev, bulk_out->bEndpointAddress &
					      USB_ENDPOINT_NUMBER_MASK);
	dev->pipe_intr =
		usb_rcvintpipe(udev, intr_in->bEndpointAddress &
					     USB_ENDPOINT_NUMBER_MASK);
	dev->intr_interval = intr_in->bInterval;

	return 0;
}

static int ch397_probe(struct usb_interface *intf,
		       const struct usb_device_id *id)
{
	struct ch397 *dev;
	struct net_device *ndev;
	struct usb_device *udev;
	u16 pid;
	int ret = 0;

	udev = interface_to_usbdev(intf);
	pid = le16_to_cpu(udev->descriptor.idProduct);

	if (pid == 0x5398 || pid == 0xe398) {
		if (udev->actconfig->desc.bConfigurationValue != 3) {
			ret = usb_driver_set_configuration(udev, 3);
			if (ret) {
				dev_err(&intf->dev,
					"failed to switch to USB configuration 3: %d\n",
					ret);
				return ret;
			}
			msleep(5);
			return -ENODEV;
		}
	} else if (pid == 0x5394 || pid == 0x5395 || pid == 0x5396 ||
		   pid == 0x5397) {
		if (udev->actconfig->desc.bConfigurationValue != 1) {
			ret = usb_driver_set_configuration(udev, 1);
			if (ret) {
				dev_err(&intf->dev,
					"failed to switch to USB configuration 1: %d\n",
					ret);
				return ret;
			}
			return -ENODEV;
		}
	}

	printk(KERN_INFO "ch397 device probe, driver version: %s\n",
	       VERSION_DESC);

	ndev = alloc_etherdev(sizeof(*dev));
	if (!ndev) {
		ret = -ENOMEM;
		goto out;
	}

	/* netdev_printk() needs this so do it as early as possible */
	SET_NETDEV_DEV(ndev, &intf->dev);

	dev = netdev_priv(ndev);
	dev->udev = udev;
	dev->intf = intf;
	dev->msg_enable =
		netif_msg_init(msg_level, NETIF_MSG_DRV | NETIF_MSG_PROBE |
						  NETIF_MSG_LINK);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
	netif_napi_add(ndev, &dev->napi, ch397_poll);
#else
	netif_napi_add(ndev, &dev->napi, ch397_poll, 64);
#endif

	INIT_DELAYED_WORK(&dev->wq, ch397_work_func);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 9, 0))
	tasklet_setup(&dev->tx_bh, ch397_bh);
#else
	tasklet_init(&dev->tx_bh, ch397_bh, (unsigned long)dev);
#endif
	tasklet_disable(&dev->tx_bh);

	mutex_init(&dev->phy_mutex);
	mutex_init(&dev->dev_mutex);

	dev->ndev = ndev;
	ndev->netdev_ops = &ch397_netdev_ops;
	ndev->watchdog_timeo = TX_TIMEOUT_JIFFIES;
	ndev->ethtool_ops = &ch397_ethtool_ops;

	/* driver requires remote-wakeup capability during autosuspend. */
	intf->needs_remote_wakeup = 1;

	ret = ch397_check_endpoints(dev);
	if (ret < 0)
		goto out1;

	ret = wch_hw_init(dev);
	if (ret < 0)
		goto out1;

	ret = dev->dev_ops->init(dev);
	if (ret < 0)
		goto out1;

	ret = wch_ethernet_init(dev);
	if (ret < 0)
		goto out1;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 19, 0))
	dev->maxpacket = usb_maxpacket(dev->udev, dev->pipe_out);
#else
	dev->maxpacket = usb_maxpacket(dev->udev, dev->pipe_out, 1);
#endif
	if (dev->maxpacket == 0) {
		ret = -ENODEV;
		goto out1;
	}

	ret = register_netdev(ndev);
	if (ret)
		goto out1;

	netif_info(dev, probe, dev->ndev,
		   "register '%s' at usb-%s-%s, %pM\n",
		   intf->dev.driver->name, udev->bus->bus_name,
		   udev->devpath, ndev->dev_addr);

	usb_set_intfdata(intf, dev);
	device_set_wakeup_enable(&udev->dev, true);
	netif_device_attach(ndev);

	return 0;

out1:
	tasklet_kill(&dev->tx_bh);
	netif_napi_del(&dev->napi);
	free_netdev(ndev);
out:
	return ret;
}

static const struct usb_device_id ch397_ids[] = {
	{
		USB_DEVICE(0x1a86, 0x5394), /* ch336 chip */
	},

	{
		USB_DEVICE(0x1a86, 0x5395), /* ch339 chip */
	},

	{
		USB_DEVICE(0x1a86, 0x5396), /* ch396 chip */
	},

	{
		USB_DEVICE(0x1a86, 0x5397), /* ch397 chip */
	},

	{
		USB_DEVICE(0x1a86, 0x5398), /* ch398 chip */
	},

	{
		USB_DEVICE(0x1a86, 0xe398), /* ch9153 chip */
	},

	{},
};

static void ch397_disconnect(struct usb_interface *intf)
{
	struct ch397 *dev = usb_get_intfdata(intf);
	struct usb_device *udev;
	struct net_device *ndev;

	if (!dev)
		return;

	udev = dev->udev;
	ndev = dev->ndev;

	netif_info(dev, probe, dev->ndev, "unregister '%s' usb-%s-%s\n",
		   intf->dev.driver->name, udev->bus->bus_name,
		   udev->devpath);

	usb_set_intfdata(intf, NULL);

	ch397_set_unplug(dev);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 3, 0))
	netif_napi_del(&dev->napi);
#endif
	unregister_netdev(ndev);
	tasklet_kill(&dev->tx_bh);

	free_netdev(ndev);
}

static bool delay_autosuspend(struct ch397 *dev)
{
	bool sw_linking = !!netif_carrier_ok(dev->ndev);
	bool hw_linking;
	u16 bmsr;
	int ret;

	ret = ch397_read_shared_word(dev, 1, MII_BMSR, &bmsr);
	if (ret < 0)
		return true;

	hw_linking = !!(bmsr & BMSR_LSTATUS);

	/* Delay autosuspend while software link state and PHY link state disagree.
	 * Otherwise the device may suspend before the link transition is handled.
	 */
	if (work_busy(&dev->wq.work) || sw_linking != hw_linking)
		return true;

	if (!skb_queue_empty(&dev->txq_pend))
		return true;
	else
		return false;
}

/* Runtime PM keeps the netdev attached. Resume tries to restart RX
 * immediately when the cached carrier state still matches PHY link state.
 */
static int ch397_runtime_resume(struct ch397 *dev)
{
	struct net_device *ndev = dev->ndev;
	struct napi_struct *napi;
	u16 bmsr;
	bool rx_started = false;
	int ret = 0;

	if (!(netif_running(ndev) && ndev->flags & IFF_UP)) {
		clear_bit(CH397_DEV_CHOOSE_SUSPEND, &dev->flags);
		return 0;
	}

	if (ch397_dev_unplugged(dev)) {
		clear_bit(CH397_DEV_CHOOSE_SUSPEND, &dev->flags);
		return -ENODEV;
	}

	ch397_set_accessible(dev);

	napi = &dev->napi;
	napi_disable(napi);
	set_bit(CH397_DEV_OPEN, &dev->flags);

	if (netif_carrier_ok(ndev)) {
		ret = ch397_read_shared_word(dev, 1, MII_BMSR, &bmsr);
		if (ret < 0) {
			netif_err(
				dev, ifup, ndev,
				"runtime resume failed to read PHY status: %d\n",
				ret);
			goto out1;
		}

		if (bmsr & BMSR_LSTATUS) {
			ret = ch397_start_rx(dev);
			if (ret < 0) {
				netif_err(
					dev, rx_err, ndev,
					"runtime resume failed to restart RX: %d\n",
					ret);
				goto out1;
			}

			rx_started = true;
		} else {
			netif_carrier_off(ndev);
			netif_stop_queue(ndev);
			ch397_disable(dev);
			netif_info(dev, link, ndev, "linking down\n");
		}
	}

	ret = usb_submit_urb(dev->intr_urb, GFP_NOIO);
	if (ret < 0) {
		if (rx_started)
			ch397_stop_rx(dev);

		netif_carrier_off(ndev);
		netif_stop_queue(ndev);
		if (ret == -ENODEV || ret == -ESHUTDOWN) {
			ch397_update_usb_state(dev, ret);
			netif_device_detach(ndev);
		}

		netif_err(dev, ifup, ndev,
			  "runtime resume failed to submit intr urb: %d\n",
			  ret);
		goto out;
	}

	napi_enable(napi);
	clear_bit(CH397_DEV_CHOOSE_SUSPEND, &dev->flags);
	smp_mb__after_atomic();

	if (!list_empty(&dev->rxq_done))
		napi_schedule(&dev->napi);

	return ret;

out1:
	netif_carrier_off(ndev);
	netif_stop_queue(ndev);
	ch397_disable(dev);
out:
	napi_enable(napi);
	clear_bit(CH397_DEV_CHOOSE_SUSPEND, &dev->flags);
	smp_mb__after_atomic();
	clear_bit(CH397_DEV_OPEN, &dev->flags);

	return ret;
}

/* System PM detaches from the net core and re-detects link state after the
 * machine resumes.
 */
static int ch397_system_resume(struct ch397 *dev)
{
	struct net_device *ndev = dev->ndev;
	int ret = 0;

	clear_bit(CH397_DEV_CHOOSE_SUSPEND, &dev->flags);

	if (ch397_dev_unplugged(dev))
		return -ENODEV;

	ch397_set_accessible(dev);

	netif_device_attach(ndev);

	if (netif_running(ndev) && (ndev->flags & IFF_UP)) {
		netif_carrier_off(ndev);
		set_bit(CH397_DEV_OPEN, &dev->flags);
		ret = usb_submit_urb(dev->intr_urb, GFP_NOIO);
		if (ret < 0) {
			clear_bit(CH397_DEV_OPEN, &dev->flags);
			netif_device_detach(ndev);
			ch397_update_usb_state(dev, ret);
			if (ch397_dev_inaccessible(dev) &&
			    !ch397_dev_unplugged(dev))
				usb_reset_device(dev->udev);
			netif_err(
				dev, ifup, ndev,
				"system resume failed to submit intr urb: %d\n",
				ret);
			return ret;
		}
	}

	return ret;
}

/* Runtime PM only quiesces the USB data path and may abort autosuspend when
 * link state or queued traffic changes underneath us.
 */
static int ch397_runtime_suspend(struct ch397 *dev)
{
	struct net_device *ndev = dev->ndev;
	struct napi_struct *napi = &dev->napi;
	struct tasklet_struct *tx_bh = &dev->tx_bh;
	int ret = 0;

	set_bit(CH397_DEV_CHOOSE_SUSPEND, &dev->flags);
	smp_mb__after_atomic();

	if (netif_running(ndev) && test_bit(CH397_DEV_OPEN, &dev->flags)) {
		clear_bit(CH397_DEV_OPEN, &dev->flags);
		usb_kill_urb(dev->intr_urb);

		if (netif_carrier_ok(ndev)) {
			napi_disable(napi);
			tasklet_disable(tx_bh);
			ch397_stop_rx(dev);
			tasklet_enable(tx_bh);
			napi_enable(napi);
		}

		if (delay_autosuspend(dev)) {
			ch397_runtime_resume(dev);
			ret = -EBUSY;
		}
	}

	return ret;
}

static int ch397_system_suspend(struct ch397 *dev)
{
	struct net_device *ndev = dev->ndev;

	/* System suspend must resume through the system-PM path even if the
	 * device happened to be runtime-suspended beforehand.
	 */
	clear_bit(CH397_DEV_CHOOSE_SUSPEND, &dev->flags);

	netif_device_detach(ndev);

	if (netif_running(ndev) && test_bit(CH397_DEV_OPEN, &dev->flags)) {
		struct napi_struct *napi = &dev->napi;

		clear_bit(CH397_DEV_OPEN, &dev->flags);
		usb_kill_urb(dev->intr_urb);
		tasklet_disable(&dev->tx_bh);
		napi_disable(napi);
		cancel_delayed_work_sync(&dev->wq);
		ch397_disable(dev);
		napi_enable(napi);
		tasklet_enable(&dev->tx_bh);
	}

	/* If we're inaccessible here then some of the work that we did to
	 * get the adapter ready for suspend didn't work. Queue up a wakeup
	 * event so we can try again.
	 */
	if (ch397_dev_inaccessible(dev) && !ch397_dev_unplugged(dev))
		pm_wakeup_event(&dev->udev->dev, 0);

	return 0;
}

/* Handle both runtime PM and system PM through the shared driver state. */
static int ch397_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct ch397 *dev = usb_get_intfdata(intf);
	int ret;

	mutex_lock(&dev->dev_mutex);

	netif_dbg(dev, ifdown, dev->ndev, "suspending: pm event %#x",
		  message.event);

	if (PMSG_IS_AUTO(message))
		ret = ch397_runtime_suspend(dev);
	else
		ret = ch397_system_suspend(dev);

	mutex_unlock(&dev->dev_mutex);

	return ret;
}

static int ch397_resume(struct usb_interface *intf)
{
	struct ch397 *dev = usb_get_intfdata(intf);
	int ret;

	mutex_lock(&dev->dev_mutex);

	if (test_bit(CH397_DEV_CHOOSE_SUSPEND, &dev->flags))
		ret = ch397_runtime_resume(dev);
	else
		ret = ch397_system_resume(dev);

	mutex_unlock(&dev->dev_mutex);

	return ret;
}

MODULE_DEVICE_TABLE(usb, ch397_ids);

static struct usb_driver ch397_driver = {
	.name = "usb_ch397",
	.id_table = ch397_ids,
	.probe = ch397_probe,
	.disconnect = ch397_disconnect,
	.suspend = ch397_suspend,
	.resume = ch397_resume,
	.supports_autosuspend = 1,
	.disable_hub_initiated_lpm = 1,
};

module_usb_driver(ch397_driver);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(VERSION_DESC);
MODULE_LICENSE("GPL");

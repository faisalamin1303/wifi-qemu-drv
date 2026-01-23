#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <net/cfg80211.h>
#include <net/rtnetlink.h>
#include <linux/etherdevice.h>
#include <linux/math64.h>
#include <linux/module.h>

#define WIFI_CMD_REG         0x00  // write
#define WIFI_STATUS_REG      0x04  // read/write
#define WIFI_RESP_COUNT      0x08  // read

#define WIFI_CMD_SCAN        1
#define WIFI_CMD_CONNECT     2
#define WIFI_CMD_TX_DATA     3
#define WIFI_CMD_RX_READY    4   /* optional */

#define WIFI_STATUS_IDLE     0
#define WIFI_STATUS_BUSY     1
#define WIFI_STATUS_DONE     2
#define WIFI_STATUS_RX_AVAIL 3
#define WIFI_STATUS_RX_DONE  4


#define WIFI_RESP_BASE       0x100 // BAR1 offset
#define WIFI_TX_BASE         0x200
#define WIFI_RX_BASE         0x400
#define WIFI_TX_LEN_REG      0x0C
#define WIFI_RX_LEN_REG      0x10

#define WIFI_MAX_FRAME_SIZE  1600


void __iomem *ptr_bar0, __iomem *ptr_bar1;

#define VID 0x1234
#define DID 0xbeef
/* TODO: device should provide the MAC addr*/
static const u8 wifi_mac_addr[ETH_ALEN] = {
    0x02, 0x11, 0x22, 0x34, 0x45, 0x56
};

static struct wiphy *common_wiphy;
static struct wireless_dev *common_wdev;
static struct net_device *common_netdev;

struct wifi_sim_wiphy_priv {
	// struct delayed_work scan_result;
	struct delayed_work scan_work;
	struct cfg80211_scan_request *scan_request;
	bool being_deleted;
};

struct wifi_sim_netdev_priv {
	struct delayed_work connect;
	struct delayed_work tx;
	struct delayed_work rx;
	// struct net_device *lowerdev;
	struct net_device *upperdev;
	u32 tx_packets;
	u32 tx_failed;
	u8 connect_requested_bss[ETH_ALEN];
	bool is_up;
	bool is_connected;
	bool being_deleted;
};

static struct ieee80211_channel channel_2ghz = {
	.band = NL80211_BAND_2GHZ,
	.center_freq = 2432,
	.hw_value = 2432,
	.max_power = 20,
};

static struct ieee80211_rate bitrates_2ghz[] = {
	{ .bitrate = 10 },
	{ .bitrate = 20 },
	{ .bitrate = 55 },
	{ .bitrate = 110 },
	{ .bitrate = 60 },
	{ .bitrate = 120 },
	{ .bitrate = 240 },
};

static struct ieee80211_supported_band band_2ghz = {
	.channels = &channel_2ghz,
	.bitrates = bitrates_2ghz,
	.band = NL80211_BAND_2GHZ,
	.n_channels = 1,
	.n_bitrates = ARRAY_SIZE(bitrates_2ghz),
	.ht_cap = {
		.ht_supported = true,
		.cap = IEEE80211_HT_CAP_SUP_WIDTH_20_40 |
		       IEEE80211_HT_CAP_GRN_FLD |
		       IEEE80211_HT_CAP_SGI_20 |
		       IEEE80211_HT_CAP_SGI_40 |
		       IEEE80211_HT_CAP_DSSSCCK40,
		.ampdu_factor = 0x3,
		.ampdu_density = 0x6,
		.mcs = {
			.rx_mask = {0xff, 0xff},
			.tx_params = IEEE80211_HT_MCS_TX_DEFINED,
		},
	},
};

static struct ieee80211_channel channel_5ghz = {
	.band = NL80211_BAND_5GHZ,
	.center_freq = 5240,
	.hw_value = 5240,
	.max_power = 20,
};

static struct ieee80211_rate bitrates_5ghz[] = {
	{ .bitrate = 60 },
	{ .bitrate = 120 },
	{ .bitrate = 240 },
};

#define RX_MCS_MAP (IEEE80211_VHT_MCS_SUPPORT_0_9 << 0 | \
		    IEEE80211_VHT_MCS_SUPPORT_0_9 << 2 | \
		    IEEE80211_VHT_MCS_SUPPORT_0_9 << 4 | \
		    IEEE80211_VHT_MCS_SUPPORT_0_9 << 6 | \
		    IEEE80211_VHT_MCS_SUPPORT_0_9 << 8 | \
		    IEEE80211_VHT_MCS_SUPPORT_0_9 << 10 | \
		    IEEE80211_VHT_MCS_SUPPORT_0_9 << 12 | \
		    IEEE80211_VHT_MCS_SUPPORT_0_9 << 14)

#define TX_MCS_MAP (IEEE80211_VHT_MCS_SUPPORT_0_9 << 0 | \
		    IEEE80211_VHT_MCS_SUPPORT_0_9 << 2 | \
		    IEEE80211_VHT_MCS_SUPPORT_0_9 << 4 | \
		    IEEE80211_VHT_MCS_SUPPORT_0_9 << 6 | \
		    IEEE80211_VHT_MCS_SUPPORT_0_9 << 8 | \
		    IEEE80211_VHT_MCS_SUPPORT_0_9 << 10 | \
		    IEEE80211_VHT_MCS_SUPPORT_0_9 << 12 | \
		    IEEE80211_VHT_MCS_SUPPORT_0_9 << 14)

static struct ieee80211_supported_band band_5ghz = {
	.channels = &channel_5ghz,
	.bitrates = bitrates_5ghz,
	.band = NL80211_BAND_5GHZ,
	.n_channels = 1,
	.n_bitrates = ARRAY_SIZE(bitrates_5ghz),
	.ht_cap = {
		.ht_supported = true,
		.cap = IEEE80211_HT_CAP_SUP_WIDTH_20_40 |
		       IEEE80211_HT_CAP_GRN_FLD |
		       IEEE80211_HT_CAP_SGI_20 |
		       IEEE80211_HT_CAP_SGI_40 |
		       IEEE80211_HT_CAP_DSSSCCK40,
		.ampdu_factor = 0x3,
		.ampdu_density = 0x6,
		.mcs = {
			.rx_mask = {0xff, 0xff},
			.tx_params = IEEE80211_HT_MCS_TX_DEFINED,
		},
	},
	.vht_cap = {
		.vht_supported = true,
		.cap = IEEE80211_VHT_CAP_MAX_MPDU_LENGTH_11454 |
		       IEEE80211_VHT_CAP_SUPP_CHAN_WIDTH_160_80PLUS80MHZ |
		       IEEE80211_VHT_CAP_RXLDPC |
		       IEEE80211_VHT_CAP_SHORT_GI_80 |
		       IEEE80211_VHT_CAP_SHORT_GI_160 |
		       IEEE80211_VHT_CAP_TXSTBC |
		       IEEE80211_VHT_CAP_RXSTBC_1 |
		       IEEE80211_VHT_CAP_RXSTBC_2 |
		       IEEE80211_VHT_CAP_RXSTBC_3 |
		       IEEE80211_VHT_CAP_RXSTBC_4 |
		       IEEE80211_VHT_CAP_MAX_A_MPDU_LENGTH_EXPONENT_MASK,
		.vht_mcs = {
			.rx_mcs_map = cpu_to_le16(RX_MCS_MAP),
			.tx_mcs_map = cpu_to_le16(TX_MCS_MAP),
		}
	},
};

struct wifi_fw_scan_cmd {
    u8  n_ssids;
    u8  n_channels;
    u8  flags;
    u8  reserved;

    struct {
        u8 len;
        u8 ssid[32];
    } ssids[4];

    u16 channels[32];
};

struct wifi_fw_scan_resp {
    u8  bssid[ETH_ALEN];
    s8  rssi;
    u8  channel;
    u8  ssid_len;
    u8  ssid[32];
};

// #define WIFI_MAX_PSK_LEN 32

struct wifi_fw_connect_cmd {
	u8 bssid[ETH_ALEN];
	struct {
        u8 len;
        u8 ssid[32];
    } ssid;
	u16 channel;
	// u8 psk_len;
    // u8 psk[WIFI_MAX_PSK_LEN];
};

struct wifi_fw_connect_resp {
	bool connect;
};


static u8 router_bssid[ETH_ALEN] = {};

static struct pci_device_id wifi_ids[] = {
	{PCI_DEVICE(VID, DID)},
	{},
};
MODULE_DEVICE_TABLE(pci, wifi_ids);


static void wifi_sim_inform_bss(struct wiphy *wiphy)
{
	u64 tsf = div_u64(ktime_get_boottime_ns(), 1000);
	struct cfg80211_bss *informed_bss;
	// const u8 bssid[ETH_ALEN] = { 0x02, 0x11, 0x22, 0x33, 0x44, 0x55 };
	

	int count = ioread32(ptr_bar0 + WIFI_RESP_COUNT);
	struct wifi_fw_scan_resp resp;
	
	printk("WIFI_SIM: Device response received !\n");
	printk("WIFI_SIM: Response SSID count = %d\n", count);
	
		for (int i = 0; i < count; i++) {
		memcpy_fromio(&resp,
			ptr_bar1 + WIFI_RESP_BASE + i * sizeof(resp),
			sizeof(resp));

		printk("WIFI_SIM:  Response SSID[%d]: len=%u, value=\"%.*s\"\n",
			i,
			resp.ssid_len,
			resp.ssid_len,
			resp.ssid);
		printk("WIFI_SIM: Response rssi = %d\n", resp.rssi);
			
		// struct {
		// 	u8 tag;
		// 	u8 len;
		// 	u8 ssid[8];
		// } __packed ssid = {
		// 	.tag = WLAN_EID_SSID,
		// 	.len = resp.ssid_len,
		// 	.ssid = resp.ssid,
		// };

		u8 ie[2 + 32];

		ie[0] = WLAN_EID_SSID;
		ie[1] = resp.ssid_len;
		memcpy(&ie[2], resp.ssid, resp.ssid_len);

		informed_bss = cfg80211_inform_bss(wiphy,
			&channel_5ghz,
			CFG80211_BSS_FTYPE_PRESP,
			resp.bssid,
			tsf,
			WLAN_CAPABILITY_ESS,
			0,
			ie,
			2 + resp.ssid_len,
			resp.rssi * 100,
			GFP_KERNEL);
		cfg80211_put_bss(wiphy, informed_bss);
	}

}

/* Called with the rtnl lock held. */
static int wifi_sim_scan(struct wiphy *wiphy,
			  struct cfg80211_scan_request *request)
{
	struct wifi_sim_wiphy_priv *priv = wiphy_priv(wiphy);
	// struct cfg80211_scan_info scan_info = { .aborted = false };

	wiphy_debug(wiphy, "scan\n");

	if (priv->scan_request || priv->being_deleted)
		return -EBUSY;

	priv->scan_request = request;

	struct wifi_fw_scan_cmd cmd = {0};
	int i;

	printk(KERN_INFO "WIFI_SIM: SCAN request received\n");
	/* Fill SSIDs */
	cmd.n_ssids = min(request->n_ssids, 4);
	// cmd.n_ssids = 4;
	printk(KERN_INFO "WIFI_SIM: Number of SSIDs = %d\n", cmd.n_ssids);

	for (i = 0; i < cmd.n_ssids; i++) {
		cmd.ssids[i].len = request->ssids[i].ssid_len;
		// cmd.ssids[i].len = 5;
		memcpy(cmd.ssids[i].ssid,
			request->ssids[i].ssid,
			request->ssids[i].ssid_len);

		// for(int j=0; j < 5; j++) {
		// 	cmd.ssids[i].ssid[j] = i + j;
		// }
		
		printk(KERN_INFO
           "WIFI_SIM: SSID[%d]: len=%u, value=\"%.*s\"\n",
           i,
           cmd.ssids[i].len,
           cmd.ssids[i].len,
           cmd.ssids[i].ssid);
	}

	for (i = 0; i < cmd.n_ssids; i++) {
		printk("WIFI_SIM: SSID[%d] = ", i);
		for(int j=0; j < 5; j++) {
			printk("%d ,", cmd.ssids[i].ssid[j]);
		}
		printk("\n");
	}

	/* Fill channels */
	cmd.n_channels = min(request->n_channels, 32);
	for (i = 0; i < cmd.n_channels; i++)
		cmd.channels[i] = request->channels[i]->hw_value;

	if (ioread32(ptr_bar0 + WIFI_STATUS_REG) == WIFI_STATUS_IDLE) {
		memcpy_toio(ptr_bar1 + WIFI_RESP_BASE, &cmd, sizeof(cmd));
		printk("WIFI_SIM: scan REQ sent to device\n");
		iowrite32(WIFI_CMD_SCAN, ptr_bar0 + WIFI_CMD_REG);
		// iowrite32(WIFI_STATUS_BUSY, ptr_bar0 + WIFI_STATUS_REG);
		printk("WIFI_SIM: scan CMD sent to device\n");
		schedule_delayed_work(&priv->scan_work, HZ * 2);
	} 
	else
		printk("WIFI_SIM: scan_req can't be forward to dev as WIFI_STATUS_REG != IDLE");

	return 0;
}

static void wifi_sim_scan_work(struct work_struct *work)
{
	printk("WIFI_SIM: Attempt reading Scan result...\n");

    struct wifi_sim_wiphy_priv *priv =
        container_of(work, struct wifi_sim_wiphy_priv, scan_work.work);

	//Todo: bar0, bar1 should be the members of struct wifi_sim_wiphy_priv i.e. ioread32(priv->bar0

	if (ioread32(ptr_bar0 + WIFI_STATUS_REG) != WIFI_STATUS_DONE) {
        schedule_delayed_work(&priv->scan_work, HZ * 2);
        return;
    }

	printk("WIFI_SIM: WIFI_STATUS_DONE ! \n");
	struct wiphy *wiphy = priv_to_wiphy(priv);


	wifi_sim_inform_bss(wiphy);

	struct cfg80211_scan_info scan_info = { .aborted = false };

	// rtnl_lock();
	// /* No real hardware → complete scan immediately */
	cfg80211_scan_done(priv->scan_request, &scan_info);

	// cfg80211_scan_done(priv->scan_request, false);
	priv->scan_request = NULL;
	iowrite32(WIFI_STATUS_IDLE, ptr_bar0 + WIFI_STATUS_REG);
	// rtnl_unlock();

}

/* May acquire and release the rdev BSS lock. */
static void wifi_sim_cancel_scan(struct wiphy *wiphy)
{
	struct wifi_sim_wiphy_priv *priv = wiphy_priv(wiphy);

	cancel_delayed_work_sync(&priv->scan_work);
	/* Clean up dangling callbacks if necessary. */
	if (priv->scan_request) {
		struct cfg80211_scan_info scan_info = { .aborted = true };
		/* Schedules work which acquires and releases the rtnl lock. */
		cfg80211_scan_done(priv->scan_request, &scan_info);
		priv->scan_request = NULL;
	}
}


/* Called with the rtnl lock held. */
static int wifi_sim_connect(struct wiphy *wiphy, struct net_device *netdev,
                            struct cfg80211_connect_params *sme)
{
    struct wifi_sim_netdev_priv *priv = netdev_priv(netdev);
	bool could_schedule;
	struct wifi_fw_connect_cmd cmd = {0};

    printk("WIFI_SIM: connect requested -> SSID: %*pE\n", sme->ssid_len, sme->ssid);
	printk("WIFI_SIM: connect requested -> BSSID: %*pE\n", ETH_ALEN, sme->bssid);

    if (priv->being_deleted || !priv->is_up)
        return -EBUSY;

    // /* Only accept our fake SSID */
    // if (sme->ssid_len != 8 || strncmp(sme->ssid, "Wifi_sim", 8) != 0) {
    //     wiphy_err(wiphy, "unknown SSID\n");
    //     return -ENOENT;
    // }

    /* Set BSSID */
    if (sme->bssid && sme->ssid && sme->channel) {
		printk("WIFI_SIM: connect req have BSSID, SSID, channel\n");
		ether_addr_copy(priv->connect_requested_bss, sme->bssid);
		memcpy(cmd.bssid, sme->bssid, ETH_ALEN);
		memcpy(cmd.ssid.ssid, sme->ssid, sme->ssid_len);
		cmd.ssid.len = sme->ssid_len;
		// Todo: logic to cpy channel
		// cmd.psk_len = sme->key_len;
    	// memcpy(cmd.psk, sme->key, sme->key_len);
	} 
	else {
	// 	wifi_sim_inform_bss(wiphy);  // Todo: Need to request for BSS-Info again, if no BSSID rcvd from scan req 
	// 	eth_zero_addr(priv->connect_requested_bss);
	return 0;                        // Todo: logic that will make to sned the connect req again 
	}

	

	if (ioread32(ptr_bar0 + WIFI_STATUS_REG) == WIFI_STATUS_IDLE) {
		memcpy_toio(ptr_bar1 + WIFI_RESP_BASE, &cmd, sizeof(cmd));
		printk("WIFI_SIM: connect REQ data sent to device\n");
		iowrite32(WIFI_CMD_CONNECT, ptr_bar0 + WIFI_CMD_REG);
		// iowrite32(WIFI_STATUS_BUSY, ptr_bar0 + WIFI_STATUS_REG);
		printk("WIFI_SIM: connect CMD sent to device\n");
		could_schedule = schedule_delayed_work(&priv->connect, HZ * 2);
		if (!could_schedule)
			return -EBUSY;
	} 
	else
		printk("WIFI_SIM: connect req can't be forward to dev as WIFI_STATUS_REG != IDLE");
	
    return 0;
}

static void wifi_sim_connect_complete(struct work_struct *work)
{
	printk("WIFI_SIM: Attempt reading Connect result...\n");
	
	struct wifi_sim_netdev_priv *priv =
		container_of(work, struct wifi_sim_netdev_priv, connect.work);

	if (ioread32(ptr_bar0 + WIFI_STATUS_REG) != WIFI_STATUS_DONE) {
        schedule_delayed_work(&priv->connect, HZ * 2);
        return;
    }	

	printk("WIFI_SIM: WIFI_STATUS_DONE ! \n");

	
	struct wifi_fw_connect_resp resp;
	u16 status = WLAN_STATUS_SUCCESS;
	u8 *requested_bss = priv->connect_requested_bss;

	memcpy_fromio(&resp, ptr_bar1 + WIFI_RESP_BASE, sizeof(resp));
	
	printk("WIFI_SIM: Response connect = %u\n", resp.connect);

	// if (is_zero_ether_addr(requested_bss))
	// 	requested_bss = NULL;

	if (!priv->is_up || !resp.connect)
		status = WLAN_STATUS_UNSPECIFIED_FAILURE;
	else
		priv->is_connected = true;

	// rtnl_lock();
	/* Schedules an event that acquires the rtnl lock. */
	cfg80211_connect_result(priv->upperdev, requested_bss, NULL, 0, NULL, 0,
				status, GFP_KERNEL);
	if (status == WLAN_STATUS_SUCCESS)			
		netif_carrier_on(priv->upperdev);

	iowrite32(WIFI_STATUS_IDLE, ptr_bar0 + WIFI_STATUS_REG);
	// rtnl_unlock();

}


/* May acquire and release the rdev event lock. */
static void wifi_sim_cancel_connect(struct net_device *netdev)
{
	struct wifi_sim_netdev_priv *priv = netdev_priv(netdev);

	/* If there is work pending, clean up dangling callbacks. */
	if (cancel_delayed_work_sync(&priv->connect)) {
		/* Schedules an event that acquires the rtnl lock. */
		cfg80211_connect_result(priv->upperdev,
					priv->connect_requested_bss, NULL, 0,
					NULL, 0,
					WLAN_STATUS_UNSPECIFIED_FAILURE,
					GFP_KERNEL);
	}
}


/* Called with the rtnl lock held. Acquires the rdev event lock. */
static int wifi_sim_disconnect(struct wiphy *wiphy, struct net_device *netdev,
				u16 reason_code)
{
	struct wifi_sim_netdev_priv *priv = netdev_priv(netdev);

	if (priv->being_deleted)
		return -EBUSY;

	wiphy_debug(wiphy, "disconnect\n");
	wifi_sim_cancel_connect(netdev);

	cfg80211_disconnected(netdev, reason_code, NULL, 0, true, GFP_KERNEL);
	priv->is_connected = false;
	netif_carrier_off(netdev);

	return 0;
}

/*
* NET DEVICE OPERATIONS
*/

static netdev_tx_t wifi_sim_start_xmit(struct sk_buff *skb,
					struct net_device *dev)
{
	// printk("WIFI_SIM: tx handler called by netdev handler !\n");

	if (ioread32(ptr_bar0 + WIFI_STATUS_REG) != WIFI_STATUS_IDLE) {
		// netif_stop_queue(dev);
        return NETDEV_TX_BUSY;
    }

	u32 len;
	struct wifi_sim_netdev_priv *priv = netdev_priv(dev);

    len = skb->len;
	priv->tx_packets++;
    if ((len > WIFI_MAX_FRAME_SIZE) || (!priv->is_connected)) {
		priv->tx_failed++;
        dev_kfree_skb(skb);
        return NET_XMIT_DROP;
    }

    printk("WIFI_SIM_TX: len=%u\n", len);
    print_hex_dump(KERN_INFO, "WIFI_SIM_TX: ",
                   DUMP_PREFIX_OFFSET, 16, 1,
                   skb->data, len, false);

    /* Copy frame to BAR1 */
    memcpy_toio(ptr_bar1 + WIFI_TX_BASE, skb->data, len);
    iowrite32(len, ptr_bar0 + WIFI_TX_LEN_REG);

	// tx_in_progress = 1;
    /* Notify device */
    iowrite32(WIFI_CMD_TX_DATA, ptr_bar0 + WIFI_CMD_REG);

    dev_kfree_skb(skb);
	schedule_delayed_work(&priv->tx, HZ * 2);

    return NET_XMIT_SUCCESS;

}

static void wifi_sim_xmit_complete(struct work_struct *work)
{
	printk("WIFI_SIM_TX: Checking whether Status register = DONE !...\n");

	struct wifi_sim_netdev_priv *priv =
		container_of(work, struct wifi_sim_netdev_priv, tx.work);

	if (ioread32(ptr_bar0 + WIFI_STATUS_REG) != WIFI_STATUS_DONE) {
        schedule_delayed_work(&priv->tx, HZ * 2);
        return;
    }	

	printk("WIFI_SIM: WIFI_STATUS_DONE ! \n");

    // netif_wake_queue(priv->upperdev);

	iowrite32(WIFI_STATUS_IDLE, ptr_bar0 + WIFI_STATUS_REG);
	// rtnl_unlock();

}

/* Called with rtnl lock held. */
static int wifi_sim_net_device_open(struct net_device *dev)
{
	struct wifi_sim_netdev_priv *priv = netdev_priv(dev);

	priv->is_up = true;
	return 0;
}

/* Called with rtnl lock held. */
static int wifi_sim_net_device_stop(struct net_device *dev)
{
	struct wifi_sim_netdev_priv *n_priv = netdev_priv(dev);

	n_priv->is_up = false;

	if (!dev->ieee80211_ptr)
		return 0;

	wifi_sim_cancel_scan(dev->ieee80211_ptr->wiphy);
	wifi_sim_cancel_connect(dev);
	netif_carrier_off(dev);

	cancel_delayed_work_sync(&n_priv->tx);

	return 0;
}

static void wifi_sim_get_stats64(struct net_device *dev,
                                 struct rtnl_link_stats64 *stats)
{
    struct wifi_sim_netdev_priv *priv = netdev_priv(dev);

    stats->tx_packets = priv->tx_packets;
    stats->tx_errors  = priv->tx_failed;
}

static const struct net_device_ops wifi_netdev_ops = {
	.ndo_start_xmit   = wifi_sim_start_xmit,
	.ndo_open	      = wifi_sim_net_device_open,
	.ndo_stop	      = wifi_sim_net_device_stop,
	.ndo_get_stats64  = wifi_sim_get_stats64,
	
	// .ndo_get_iflink = wifi_sim_net_device_get_iflink,
};


static struct wireless_dev *
wifi_sim_add_interface(struct wiphy *wiphy,
		       const char *name,
		       unsigned char name_assign_type,
		       enum nl80211_iftype type,
		       struct vif_params *params)
{
	struct net_device *netdev;
	struct wireless_dev *wdev;
	struct wifi_sim_netdev_priv *priv;

	if (type != NL80211_IFTYPE_STATION)
		return ERR_PTR(-EOPNOTSUPP);

	netdev = alloc_netdev(sizeof(*priv), name,
			      name_assign_type, ether_setup);
	if (!netdev)
		return ERR_PTR(-ENOMEM);

// #if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0)
    ether_addr_copy(netdev->dev_addr, wifi_mac_addr);
// #else
//     dev_addr_set(netdev, wifi_mac_addr);
// #endif

	netdev->netdev_ops = &wifi_netdev_ops;
	SET_NETDEV_DEV(netdev, wiphy_dev(wiphy));

	wdev = kzalloc(sizeof(*wdev), GFP_KERNEL);
	if (!wdev) {
		free_netdev(netdev);
		return ERR_PTR(-ENOMEM);
	}

	wdev->wiphy  = wiphy;
	wdev->netdev = netdev;
	wdev->iftype = type;

	netdev->ieee80211_ptr = wdev;

	priv = netdev_priv(netdev);
	priv->upperdev = netdev; 
	priv->being_deleted = false;
	priv->is_up = false;
	priv->is_connected = false;

	INIT_DELAYED_WORK(&priv->connect, wifi_sim_connect_complete);
	INIT_DELAYED_WORK(&priv->tx, wifi_sim_xmit_complete);

	if (register_netdev(netdev)) {
		kfree(wdev);
		free_netdev(netdev);
		return ERR_PTR(-EINVAL);
	}

	wiphy_info(wiphy, "interface %s created\n", netdev->name);
	return wdev;
}

static int
wifi_sim_del_interface(struct wiphy *wiphy, struct wireless_dev *wdev)
{
	struct net_device *netdev = wdev->netdev;
	struct wifi_sim_netdev_priv *priv = netdev_priv(netdev);

	/* Mark interface as being deleted */
	priv->being_deleted = true;

	netif_carrier_off(netdev);
	unregister_netdev(netdev);
	
	kfree(wdev);
	// free_netdev(netdev);   // unregister_netdev() let cfg80211 already schedule free_netdev() internally

	wiphy_info(wiphy, "interface removed\n");
	return 0;
}


static const struct cfg80211_ops wifi_sim_cfg80211_ops = {
	.scan = wifi_sim_scan,
	.connect = wifi_sim_connect,
	.disconnect = wifi_sim_disconnect,

	.add_virtual_intf = wifi_sim_add_interface,
	.del_virtual_intf = wifi_sim_del_interface,

	// .get_station = wifi_sim_get_station,
	// .dump_station = wifi_sim_dump_station,
};




/* Acquires and releases the rtnl lock. */
static struct wiphy *wifi_sim_make_wiphy(void) {
	struct wiphy *wiphy;
	struct wifi_sim_wiphy_priv *priv;
	int err;

	printk("WIFI_SIM: pci-wifi-drv - Creating wiphy \n");
	wiphy = wiphy_new(&wifi_sim_cfg80211_ops, sizeof(*priv));

	if (!wiphy)
		return NULL;

	wiphy->max_scan_ssids = 4;
	wiphy->max_scan_ie_len = 1000;
	wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;

	wiphy->bands[NL80211_BAND_2GHZ] = &band_2ghz;
	wiphy->bands[NL80211_BAND_5GHZ] = &band_5ghz;
	wiphy->bands[NL80211_BAND_60GHZ] = NULL;

	wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);

	priv = wiphy_priv(wiphy);
	priv->being_deleted = false;
	priv->scan_request = NULL;
	INIT_DELAYED_WORK(&priv->scan_work, wifi_sim_scan_work);

	err = wiphy_register(wiphy);
	if (err < 0) {
		printk("WIFI_SIM: pci-wifi-drv - wiphy can't be registered!\n");
		wiphy_free(wiphy);
		return NULL;
	}

	return wiphy;
}

static int wifi_probe(struct pci_dev *pdev, const struct pci_device_id *id) {
	
	int status;
	// int err;
	// void __iomem *ptr_bar0, __iomem *ptr_bar1;
	status = pcim_enable_device(pdev);
	if(status != 0){
		printk("WIFI_SIM: pci-wifi-drv - Error enabling device\n");
		return status;
	}
	ptr_bar0 = pcim_iomap(pdev, 0, pci_resource_len(pdev, 0));
	if(!ptr_bar0) {
		printk("WIFI_SIM: pci-wifi-drv - Error mapping BAR0\n");
		return -ENODEV;
	}

	ptr_bar1 = pcim_iomap(pdev, 1, pci_resource_len(pdev, 1));
	if(!ptr_bar1) {
		printk("WIFI_SIM: pci-wifi-drv - Error mapping BAR1\n");
		return -ENODEV;
	}
	// printk("pci-wifi-drv - ID: 0x%x\n", ioread32(ptr_bar0));
	// printk("pci-wifi-drv - Random Value: 0x%x\n", ioread32(ptr_bar0 + 0xc));
	
	// iowrite32(0x11223344, ptr_bar0 + 4);
	// mdelay(1);

	// printk("pci-wifi-drv - Inverse Pattern: 0x%x\n", ioread32(ptr_bar0 + 0x4));
	
	// iowrite32(0x44332211, ptr_bar1);	
	// printk("pci-wifi-drv - BAR1 offset 0: 0x%x\n", ioread8(ptr_bar1));
	// printk("pci-wifi-drv - BAR1 offset 0: 0x%x\n", ioread16(ptr_bar1));
	// printk("pci-wifi-drv - BAR1 offset 0: 0x%x\n", ioread32(ptr_bar1));
	
	common_wiphy = wifi_sim_make_wiphy();
	if (!common_wiphy)
		goto notifier;


	common_wdev = wifi_sim_add_interface(common_wiphy,
				"wlan0",
				NET_NAME_ENUM,
				NL80211_IFTYPE_STATION,
				NULL);

	if (IS_ERR(common_wdev)) {
		wiphy_err(common_wiphy, "failed to create default STA interface\n");
		return PTR_ERR(common_wdev);
	}


	return 0;

	notifier:
		return -ENODEV;
}

/* Acquires and releases the rtnl lock. */
static void wifi_sim_destroy_wiphy(struct wiphy *wiphy)
{
	struct wifi_sim_wiphy_priv *priv;

	WARN(!wiphy, "%s called with null wiphy", __func__);
	if (!wiphy)
		return;

	priv = wiphy_priv(wiphy);
	priv->being_deleted = true;
	wifi_sim_cancel_scan(wiphy);
	
	if (wiphy->registered)
		wiphy_unregister(wiphy);
	wiphy_free(wiphy);
}

static void wifi_remove(struct pci_dev *pdev)
{
	printk("WIFI_SIM: pci-wifi-drv - Removing the device\n");

	if (common_wiphy && common_wdev) {
        wifi_sim_del_interface(common_wiphy, common_wdev);
        common_wdev = NULL;
    }
	
	if (common_wiphy) {
		wifi_sim_destroy_wiphy(common_wiphy);
		common_wiphy = NULL;
	}


}



static struct pci_driver wifi_driver = {
	.name = "pci-wifi-driver",
	.probe = wifi_probe,
	.remove = wifi_remove,
	.id_table = wifi_ids,
};

module_pci_driver(wifi_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Faisal Amin");
MODULE_DESCRIPTION("Driver for a 80211 module test");
MODULE_ALIAS_RTNL_LINK("wifi_sim");
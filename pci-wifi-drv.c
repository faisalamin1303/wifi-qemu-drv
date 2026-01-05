#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <net/cfg80211.h>
#include <net/rtnetlink.h>
#include <linux/etherdevice.h>
#include <linux/math64.h>
#include <linux/module.h>

#define VID 0x1234
#define DID 0xbeef

static struct wiphy *common_wiphy;
static struct wireless_dev *common_wdev;
static struct net_device *common_netdev;

struct wifi_sim_wiphy_priv {
	// struct delayed_work scan_result;
	struct cfg80211_scan_request *scan_request;
	bool being_deleted;
};

struct wifi_sim_netdev_priv {
	// struct delayed_work connect;
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


static u8 router_bssid[ETH_ALEN] = {};

static struct pci_device_id wifi_ids[] = {
	{PCI_DEVICE(VID, DID)},
	{},
};
MODULE_DEVICE_TABLE(pci, wifi_ids);

/* Called with the rtnl lock held. */
static int wifi_sim_scan(struct wiphy *wiphy,
			  struct cfg80211_scan_request *request)
{
	struct wifi_sim_wiphy_priv *priv = wiphy_priv(wiphy);

	wiphy_debug(wiphy, "scan\n");

	if (priv->scan_request || priv->being_deleted)
		return -EBUSY;

	priv->scan_request = request;
	

	/* No real hardware → complete scan immediately */
	cfg80211_scan_done(request, false);

	priv->scan_request = NULL;

	return 0;
}

/* Called with the rtnl lock held. */
static int wifi_sim_connect(struct wiphy *wiphy, struct net_device *netdev,
			     struct cfg80211_connect_params *sme)
{
	struct wifi_sim_netdev_priv *priv = netdev_priv(netdev);
	// bool could_schedule;

	// if (priv->being_deleted || !priv->is_up)
	// 	return -EBUSY;

	// could_schedule = schedule_delayed_work(&priv->connect, HZ * 2);
	// if (!could_schedule)
	// 	return -EBUSY;

	if (sme->bssid) {
		ether_addr_copy(priv->connect_requested_bss, sme->bssid);
	} 
	// else {
	// 	wifi_sim_inform_bss(wiphy);
	// 	eth_zero_addr(priv->connect_requested_bss);
	// }

	wiphy_debug(wiphy, "connect\n");

	return 0;
}

/* Called with the rtnl lock held. Acquires the rdev event lock. */
static int wifi_sim_disconnect(struct wiphy *wiphy, struct net_device *netdev,
				u16 reason_code)
{
	struct wifi_sim_netdev_priv *priv = netdev_priv(netdev);

	if (priv->being_deleted)
		return -EBUSY;

	wiphy_debug(wiphy, "disconnect\n");
	// wifi_sim_cancel_connect(netdev);

	cfg80211_disconnected(netdev, reason_code, NULL, 0, true, GFP_KERNEL);
	priv->is_connected = false;
	netif_carrier_off(netdev);

	return 0;
}


/*
* NET DEVICE OPERATIONS
*/

/* Enters and exits a RCU-bh critical section. */
static netdev_tx_t wifi_sim_start_xmit(struct sk_buff *skb,
					struct net_device *dev)
{
	struct wifi_sim_netdev_priv *priv = netdev_priv(dev);

	priv->tx_packets++;
	if (!priv->is_connected) {
		priv->tx_failed++;
		return NET_XMIT_DROP;
	}

	// skb->dev = priv->lowerdev;
	// return dev_queue_xmit(skb);
	return 0;
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

	// wifi_sim_cancel_scan(dev->ieee80211_ptr->wiphy);
	// wifi_sim_cancel_connect(dev);
	// netif_carrier_off(dev);

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
	priv->is_up = false;
	priv->is_connected = false;

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

	printk("pci-wifi-drv - Creating wiphy \n");
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
	// INIT_DELAYED_WORK(&priv->scan_result, wifi_sim_scan_result);

	err = wiphy_register(wiphy);
	if (err < 0) {
		printk("pci-wifi-drv - wiphy can't be registered!\n");
		wiphy_free(wiphy);
		return NULL;
	}

	return wiphy;
}

// static struct net_device *wifi_sim_make_netdev(void) { 
// 	struct net_device *netdev;
// 	struct wifi_sim_netdev_priv *priv;
// 	int err;

// 	printk("pci-wifi-drv - Creating net_device\n");
// 	netdev = alloc_etherdev(sizeof(*priv));
// 	if(!netdev)
// 		return NULL;

// 	// SET_NETDEV_DEV(netdev, &pdev->dev); // Todo: where is pdev ??
// 	netdev->netdev_ops = &wifi_netdev_ops;
// 	netdev->needs_free_netdev  = true;

// 	// SET_NETDEV_DEV(netdev, &priv->lowerdev->dev);   // Todo: who will be parent ?   
// 	// 4. Link wiphy to net_device
// 	struct wireless_dev *wdev;
// 	wdev = netdev->ieee80211_ptr; 

// 	wdev = kzalloc(sizeof(*netdev->ieee80211_ptr), GFP_KERNEL);
// 	if (!netdev->ieee80211_ptr) {
// 		err = -ENOMEM;
// 		goto remove_handler;
// 	}

	

// 	wdev->wiphy = common_wiphy;
// 	wdev->netdev = netdev;
// 	wdev->iftype = NL80211_IFTYPE_STATION;
// 	register_netdev(netdev);

// 	return netdev;

// 	remove_handler:
// 		free_netdev(netdev);
// 		return NULL;

// }

static int wifi_probe(struct pci_dev *pdev, const struct pci_device_id *id) {
	
	int status;
	// int err;
	void __iomem *ptr_bar0, __iomem *ptr_bar1;
	status = pcim_enable_device(pdev);
	if(status != 0){
		printk("pci-wifi-drv - Error enabling device\n");
		return status;
	}
	ptr_bar0 = pcim_iomap(pdev, 0, pci_resource_len(pdev, 0));
	if(!ptr_bar0) {
		printk("pci-wifi-drv - Error mapping BAR0\n");
		return -ENODEV;
	}

	ptr_bar1 = pcim_iomap(pdev, 1, pci_resource_len(pdev, 1));
	if(!ptr_bar1) {
		printk("pci-wifi-drv - Error mapping BAR1\n");
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
	// wifi_sim_cancel_scan(wiphy);

	/* Abort ongoing scan */
	if (priv->scan_request) {
		cfg80211_scan_done(priv->scan_request, true);
		priv->scan_request = NULL;
	}
	
	if (wiphy->registered)
		wiphy_unregister(wiphy);
	wiphy_free(wiphy);
}

static void wifi_remove(struct pci_dev *pdev)
{
	printk("pci-wifi-drv - Removing the device\n");

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
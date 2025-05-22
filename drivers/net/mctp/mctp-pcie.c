#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <net/mctp.h>
#include <net/mctpdevice.h>
#include <uapi/linux/mctp-pcie.h>

static netdev_tx_t mctp_netdev_tx(struct sk_buff *skb, struct net_device *ndev)
{
	struct mctp_pcie_netdev *pcie_netdev = netdev_priv(ndev);
	ssize_t ret;

	if (pcie_netdev->ops->mctp_pcie_tx != NULL) {
		ret = pcie_netdev->ops->mctp_pcie_tx(skb, ndev);
	} else {
		ndev->stats.tx_dropped++;
		kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	if (ret > 0) {
		ndev->stats.tx_bytes += ret;
		ndev->stats.tx_packets++;
	} else if (ret == 0) {
		ndev->stats.tx_dropped++;
	} else {
		ndev->stats.tx_errors++;
		return NETDEV_TX_BUSY;
	}

	kfree_skb(skb);
	return NETDEV_TX_OK;
}

static int mctp_netdev_open(struct net_device *dev)
{
	struct mctp_pcie_netdev *pcie_netdev = netdev_priv(dev);

	if (pcie_netdev->ops->netdev_open == NULL) {
		return 0;
	} else {
		return pcie_netdev->ops->netdev_open(dev);
	}
}

static const struct net_device_ops mctp_pcie_netdev_ops = {
	.ndo_start_xmit = mctp_netdev_tx,
	.ndo_open = mctp_netdev_open,
};

// This is a workaround. May need a better way to pass the routing type from user space
static u8 mctp_netdev_decide_rtype(u8 *data, unsigned int len)
{
	// Only handle MCTP control messages. The lengh should be at least 6 bytes
	// Always set routing type to 2 (Route by ID) for other type of messages

	if (len < (MCTP_TRANSPORT_HDR_SIZE + 2)) {
		return ROUTING_TYPE_BY_ID;
	}

	if (data[4] != 0) {
		return ROUTING_TYPE_BY_ID;
	}

	// For MCTP Prepare for Endpoint Discovery and Endpoint Discovery control messages
	// If it is a request, the routing type must be 3 (Broadcast from Root Complex)
	// If it is a response, the routing type must be 0 (Route to Root Complex)
	if ((data[6] == 0x0B) || (data[6] == 0x0C)) {
		if (data[3] & 0x8) {
			return ROUTING_TYPE_BROADCAST_FROM_RC;
		} else {
			return ROUTING_TYPE_TO_RC;
		}
	} else {
		return ROUTING_TYPE_BY_ID;
	}
}

static int mctp_netdev_header_create(struct sk_buff *skb, struct net_device *dev,
		unsigned short type, const void *daddr, const void *saddr, unsigned int len)
{
	struct mctp_pcie_netdev *pcie_netdev = netdev_priv(dev);
	struct pcie_medium_hdr *hdr;
	struct mctp_hdr *mhdr;
	size_t len_dw = (len + 3) / 4;
	u8 rtype = ROUTING_TYPE_BY_ID;

	if (len < MCTP_TRANSPORT_HDR_SIZE) {
		return -EMSGSIZE;
	}

	if (len > dev->mtu) {
		return -EMSGSIZE;
	}

	rtype = mctp_netdev_decide_rtype(skb->data, len);

	if (!daddr || !saddr) {
		return -EINVAL;
	}

	skb_push(skb, sizeof(struct pcie_medium_hdr));
	skb_reset_mac_header(skb);
	hdr = (void *) skb_mac_header(skb);
	mhdr = mctp_hdr(skb);

	// Set routing type to Broadcast from Root Complex if the destination EID is broadcast EID
	if (mhdr->dest == 0xFF) {
		rtype = ROUTING_TYPE_BROADCAST_FROM_RC;
	}

	hdr->fmt_type = 0x70 | rtype;
	hdr->mbz = 0x00;
	hdr->mbz_attr_len_hi = (len_dw >> 8) & 3;
	hdr->len_lo = (len_dw & 0xFF) - 1; // The length field does not include MCTP header
	memcpy(&hdr->requester, saddr, 2);
	hdr->tag = ((4 - ((len - 4) % 4)) & 0x3) << 4;
	hdr->code = 0x7F;
	memcpy(&hdr->target, daddr, 2);
	hdr->vendor[0] = 0x1A;
	hdr->vendor[1] = 0xB4;
	mhdr->ver = 0x01;

	if (pcie_netdev->ops->netdev_header_create == NULL) {
		return sizeof(struct pcie_medium_hdr);
	} else {
		return pcie_netdev->ops->netdev_header_create(skb, dev, type, daddr, saddr, len);
	}
}

static const struct header_ops mctp_pcie_headops = {
	.create = mctp_netdev_header_create,
};

static void mctp_pcie_netdev_setup(struct net_device *dev)
{
	dev->type = ARPHRD_MCTP;

	dev->hard_header_len = PCIE_VDM_HDR_SIZE;
	dev->addr_len = MCTP_PCIE_ADDR_LEN;

	dev->netdev_ops	= &mctp_pcie_netdev_ops;
	dev->header_ops	= &mctp_pcie_headops;
}

void mctp_pcie_netdev_rx(struct net_device *ndev,
		struct mctp_pcie_pkt *rx_packet)
{
	struct mctp_skb_cb *cb;
	struct sk_buff *skb;
	u8 *hdr;
	int status, psize, padding_len;

	hdr = (u8 *) &rx_packet->pcie_hdr;

	/* hdr[3]: payload length in no.dwords */
	psize = hdr[3] * 4;
	if (psize > (ndev->mtu - MCTP_TRANSPORT_HDR_SIZE)) {
		ndev->stats.rx_dropped++;
		return;
	}

	padding_len = (hdr[6] >> 4) & 3;
	/* Remove padding bytes */
	psize -= padding_len;

	skb = netdev_alloc_skb(ndev, PCIE_VDM_HDR_SIZE + psize);
	if (!skb) {
		ndev->stats.rx_dropped++;
		return;
	}

	skb->protocol = htons(ETH_P_MCTP);
	skb_put_data(skb, rx_packet, PCIE_VDM_HDR_SIZE + psize);
	skb_reset_mac_header(skb);
	skb_pull(skb, sizeof(struct pcie_medium_hdr));
	skb_reset_network_header(skb);

	cb = __mctp_cb(skb);
	cb->halen = MCTP_PCIE_ADDR_LEN;
	memcpy(cb->haddr, rx_packet->pcie_hdr.requester, MCTP_PCIE_ADDR_LEN);

	status = netif_rx(skb);

	if (status == NET_RX_SUCCESS) {
		ndev->stats.rx_packets++;
		ndev->stats.rx_bytes += MCTP_TRANSPORT_HDR_SIZE + psize;
	} else {
		ndev->stats.rx_dropped++;
		kfree_skb(skb);
	}
}
EXPORT_SYMBOL_GPL(mctp_pcie_netdev_rx);

int mctp_pcie_register_netdev(const char *ifname, struct device *dev, u8 *netdev_lladdr,
		void *userdata, const struct mctp_pcie_netdev_ops *ops,
		struct net_device **ndev)
{
	struct mctp_pcie_netdev *pcie_netdev;
	int rc;

	*ndev = alloc_netdev(sizeof(struct mctp_pcie_netdev), ifname, NET_NAME_ENUM,
		mctp_pcie_netdev_setup);
	if (*ndev == NULL) {
		return -ENOMEM;
	}

	dev_net_set(*ndev, current->nsproxy->net_ns);
	SET_NETDEV_DEV(*ndev, dev);
	dev_addr_set(*ndev, netdev_lladdr);

	pcie_netdev = netdev_priv(*ndev);
	memcpy(pcie_netdev->netdev_lladdr, netdev_lladdr, MCTP_PCIE_ADDR_LEN);
	pcie_netdev->userdata = userdata;
	pcie_netdev->ops = ops;

	if (pcie_netdev->ops->netdev_setup != NULL) {
		pcie_netdev->ops->netdev_setup(*ndev);
	}

	rc = mctp_register_netdev(*ndev, NULL);
	if (rc < 0) {
		goto free_netdev;
	}

	return 0;

free_netdev:
	free_netdev(*ndev);
	return rc;
}
EXPORT_SYMBOL_GPL(mctp_pcie_register_netdev);

void mctp_pcie_unregister_netdev(struct net_device *ndev)
{
	mctp_unregister_netdev(ndev);
}
EXPORT_SYMBOL_GPL(mctp_pcie_unregister_netdev);

MODULE_LICENSE("GPL");

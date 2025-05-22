#ifndef __UAPI_MCTP_PCIE_H
#define __UAPI_MCTP_PCIE_H

#include <linux/types.h>
#include <net/mctp.h>

#define PCIE_VDM_HDR_SIZE 16
#define PCIE_MEDIUM_HDR_SIZE 12
#define MCTP_TRANSPORT_HDR_SIZE 4

#define ROUTING_TYPE_TO_RC 0
#define ROUTING_TYPE_BY_ID 2
#define ROUTING_TYPE_BROADCAST_FROM_RC 3

#define MCTP_PCIE_ADDR_LEN 2

struct pcie_medium_hdr {
	u8 fmt_type;
	u8 mbz;
	u8 mbz_attr_len_hi;
	u8 len_lo;
	u8 requester[2];
	u8 tag;
	u8 code;
	u8 target[2];
	u8 vendor[2];
} __packed;

struct mctp_pcie_pkt {
	struct pcie_medium_hdr pcie_hdr;
	struct mctp_hdr mctp_hdr;
	u8 payload[0];
} __packed;

struct mctp_pcie_netdev_ops {
	void (*netdev_setup)(struct net_device *dev);
	int (*netdev_open)(struct net_device *dev);
	int (*netdev_header_create)(struct sk_buff *skb, struct net_device *dev,
			unsigned short type, const void *daddr, const void *saddr, unsigned int len);
	ssize_t (*mctp_pcie_tx)(struct sk_buff *skb, struct net_device *ndev);
};

struct mctp_pcie_netdev {
	struct net_device *netdev;
	const struct mctp_pcie_netdev_ops *ops;
	u8 netdev_lladdr[MCTP_PCIE_ADDR_LEN];
	void *userdata;
};

int mctp_pcie_register_netdev(const char *ifname, struct device *dev, u8 *netdev_lladdr,
		void *userdata, const struct mctp_pcie_netdev_ops *ops,
		struct net_device **ndev);

void mctp_pcie_unregister_netdev(struct net_device *ndev);

void mctp_pcie_netdev_rx(struct net_device *ndev,
		struct mctp_pcie_pkt *rx_packet);

#endif /* __UAPI_MCTP_PCIE_H */

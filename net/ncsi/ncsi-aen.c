/*
 * Copyright Gavin Shan, IBM Corporation 2015.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>

#include <net/ncsi.h>
#include <net/net_namespace.h>
#include <net/sock.h>

#include "internal.h"
#include "ncsi-pkt.h"

static int ncsi_validate_aen_pkt(struct ncsi_aen_pkt_hdr *h,
				 const unsigned short payload)
{
	unsigned char *stream;
	__be32 *checksum, csum;
	__be32 high, low;
	int i;

	if (h->common.revision != NCSI_PKT_REVISION)
		return -EINVAL;
	if (ntohs(h->common.length) != payload)
		return -EINVAL;

	/*
	 * Validate checksum, which might be zeroes if the
	 * sender doesn't support checksum according to NCSI
	 * specification.
	 */
	checksum = (__be32 *)((void *)(h + 1) + payload - 4);
	if (ntohl(*checksum) == 0)
		return 0;

	csum = 0;
	stream = (unsigned char *)h;
	for (i = 0; i < sizeof(*h) + payload - 4; i += 2) {
		high = stream[i];
		low = stream[i + 1];
		csum += ((high << 8) | low);
	}

	csum = ~csum + 1;
	if (*checksum != htonl(csum))
		return -EINVAL;

	return 0;
}

static int ncsi_aen_handler_lsc(struct ncsi_dev_priv *ndp,
				struct ncsi_aen_pkt_hdr *h)
{
	struct ncsi_dev *nd = &ndp->ndp_ndev;
	struct ncsi_aen_lsc_pkt *lsc;
	struct ncsi_channel *nc;
	struct ncsi_channel_mode *ncm;
	int ret;

	ret = ncsi_validate_aen_pkt(h, 12);
	if (ret)
		return ret;

	/* Find the NCSI channel */
	ncsi_find_package_and_channel(ndp, h->common.channel, NULL, &nc);
	if (!nc)
		return -ENODEV;

	/* Update the link status */
	ncm = &nc->nc_modes[NCSI_MODE_LINK];
	lsc = (struct ncsi_aen_lsc_pkt *)h;
	ncm->ncm_data[2] = ntohl(lsc->status);
	ncm->ncm_data[4] = ntohl(lsc->oem_status);
	if (!ndp->ndp_active_channel ||
	    ndp->ndp_active_channel != nc ||
	    ncm->ncm_data[2] & 0x1)
		return 0;

	/* If this channel is the active one and the link is down,
	 * we have to choose another channel to be active one.
	 */
	ndp->ndp_flags |= NCSI_DEV_PRIV_FLAG_CHANGE_ACTIVE;
	ncsi_stop_dev(nd);

	return 0;
}

static int ncsi_aen_handler_cr(struct ncsi_dev_priv *ndp,
			       struct ncsi_aen_pkt_hdr *h)
{
	struct ncsi_dev *nd = &ndp->ndp_ndev;
	struct ncsi_channel *nc;
	int ret;

	ret = ncsi_validate_aen_pkt(h, 4);
	if (ret)
		return ret;

	/* Find the NCSI channel */
	ncsi_find_package_and_channel(ndp, h->common.channel, NULL, &nc);
	if (!nc)
		return -ENODEV;

	/* If the channel is active one, we need reconfigure it */
	if (!ndp->ndp_active_channel ||
	    ndp->ndp_active_channel != nc)
		return 0;

	ncsi_config_dev(nd);

	return 0;
}

static int ncsi_aen_handler_hncdsc(struct ncsi_dev_priv *ndp,
				   struct ncsi_aen_pkt_hdr *h)
{
	struct ncsi_dev *nd = &ndp->ndp_ndev;
	struct ncsi_channel *nc;
	struct ncsi_channel_mode *ncm;
	struct ncsi_aen_hncdsc_pkt *hncdsc;
	int ret;

	ret = ncsi_validate_aen_pkt(h, 4);
	if (ret)
		return ret;

	/* Find the NCSI channel */
	ncsi_find_package_and_channel(ndp, h->common.channel, NULL, &nc);
	if (!nc)
		return -ENODEV;

	/* If the channel is active one, we need reconfigure it */
	ncm = &nc->nc_modes[NCSI_MODE_LINK];
	hncdsc = (struct ncsi_aen_hncdsc_pkt *)h;
	ncm->ncm_data[3] = ntohl(hncdsc->status);
	if (ndp->ndp_active_channel != nc ||
	    ncm->ncm_data[3] & 0x1)
		return 0;

	/* If this channel is the active one and the link doesn't
	 * work, we have to choose another channel to be active one.
	 * The logic here is exactly similar to what we do when link
	 * is down on the active channel.
	 */
	ndp->ndp_flags |= NCSI_DEV_PRIV_FLAG_CHANGE_ACTIVE;
	ncsi_stop_dev(nd);

	return 0;
}

static struct ncsi_aen_handler {
	unsigned char	nah_type;
	int		(*nah_handler)(struct ncsi_dev_priv *ndp,
				       struct ncsi_aen_pkt_hdr *h);
} ncsi_aen_handlers[] = {
	{ NCSI_PKT_AEN_LSC,    ncsi_aen_handler_lsc    },
	{ NCSI_PKT_AEN_CR,     ncsi_aen_handler_cr     },
	{ NCSI_PKT_AEN_HNCDSC, ncsi_aen_handler_hncdsc },
	{ 0,                   NULL                    }
};

int ncsi_aen_handler(struct ncsi_dev_priv *ndp, struct sk_buff *skb)
{
	struct ncsi_aen_pkt_hdr *h;
	struct ncsi_aen_handler *nah;
	int ret;

	/* Find the handler */
	h = (struct ncsi_aen_pkt_hdr *)skb_network_header(skb);
	nah = ncsi_aen_handlers;
	while (nah->nah_handler) {
		if (nah->nah_type == h->type)
			break;

		nah++;
	}

	if (!nah->nah_handler) {
		pr_warn("NCSI: Received unrecognized AEN packet (0x%x)\n",
			h->type);
		return -ENOENT;
	}

	ret = nah->nah_handler(ndp, h);
	consume_skb(skb);

	return ret;
}

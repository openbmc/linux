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

/*
 * This function should be called after the data area has been
 * populated completely.
 */
static int ncsi_cmd_build_header(struct ncsi_pkt_hdr *h,
				 struct ncsi_cmd_arg *nca)
{
	__be32 csum, *checksum;
	__be32 low, high;
	unsigned char *stream;
	int i;

	h->mc_id        = 0;
	h->revision     = NCSI_PKT_REVISION;
	h->reserved     = 0;
	h->id           = nca->nca_id;
	h->type         = nca->nca_type;
	h->channel      = NCSI_TO_CHANNEL(nca->nca_package,
					  nca->nca_channel);
	h->length       = htons(nca->nca_payload);
	h->reserved1[0] = 0;
	h->reserved1[1] = 0;

	/* Calculate the checksum */
	csum = 0;
	stream = (unsigned char *)h;
	for (i = 0; i < sizeof(*h) + nca->nca_payload; i += 2) {
		high = stream[i];
		low = stream[i + 1];
		csum += ((high << 8) | low);
	}

	/* Fill with the calculated checksum */
	checksum = (__be32 *)((void *)(h + 1) + nca->nca_payload);
	csum = (~csum + 1);
	*checksum = htonl(csum);

	return 0;
}

static int ncsi_cmd_handler_default(struct sk_buff *skb,
				   struct ncsi_cmd_arg *nca)
{
	struct ncsi_cmd_pkt *cmd;

	if (!nca)
		return 0;

	cmd = (struct ncsi_cmd_pkt *)skb_put(skb, sizeof(*cmd));
	memset(cmd, 0, sizeof(*cmd));
	return ncsi_cmd_build_header(&cmd->cmd.common, nca);
}

static int ncsi_cmd_handler_sp(struct sk_buff *skb,
			       struct ncsi_cmd_arg *nca)
{
	struct ncsi_cmd_sp_pkt *cmd;

	if (!nca)
		return 4;

	cmd = (struct ncsi_cmd_sp_pkt *)skb_put(skb, sizeof(*cmd));
	memset(cmd, 0, sizeof(*cmd));
	cmd->hw_arbitration = nca->nca_bytes[0];
	return ncsi_cmd_build_header(&cmd->cmd.common, nca);
}

static int ncsi_cmd_handler_dc(struct sk_buff *skb,
			       struct ncsi_cmd_arg *nca)
{
	struct ncsi_cmd_dc_pkt *cmd;

	if (!nca)
		return 4;

	cmd = (struct ncsi_cmd_dc_pkt *)skb_put(skb, sizeof(*cmd));
	memset(cmd, 0, sizeof(*cmd));
	cmd->ald = nca->nca_bytes[0];
	return ncsi_cmd_build_header(&cmd->cmd.common, nca);
}

static int ncsi_cmd_handler_rc(struct sk_buff *skb,
			       struct ncsi_cmd_arg *nca)
{
	struct ncsi_cmd_rc_pkt *cmd;

	if (!nca)
		return 4;

	cmd = (struct ncsi_cmd_rc_pkt *)skb_put(skb, sizeof(*cmd));
	memset(cmd, 0, sizeof(*cmd));
	return ncsi_cmd_build_header(&cmd->cmd.common, nca);
}

static int ncsi_cmd_handler_ae(struct sk_buff *skb,
			       struct ncsi_cmd_arg *nca)
{
	struct ncsi_cmd_ae_pkt *cmd;

	if (!nca)
		return 8;

	cmd = (struct ncsi_cmd_ae_pkt *)skb_put(skb, sizeof(*cmd));
	memset(cmd, 0, sizeof(*cmd));
	cmd->mc_id = nca->nca_bytes[0];
	cmd->mode = htonl(nca->nca_dwords[1]);
	return ncsi_cmd_build_header(&cmd->cmd.common, nca);
}

static int ncsi_cmd_handler_sl(struct sk_buff *skb,
			       struct ncsi_cmd_arg *nca)
{
	struct ncsi_cmd_sl_pkt *cmd;

	if (!nca)
		return 8;

	cmd = (struct ncsi_cmd_sl_pkt *)skb_put(skb, sizeof(*cmd));
	memset(cmd, 0, sizeof(*cmd));
	cmd->mode = htonl(nca->nca_dwords[0]);
	cmd->oem_mode = htonl(nca->nca_dwords[1]);
	return ncsi_cmd_build_header(&cmd->cmd.common, nca);
}

static int ncsi_cmd_handler_svf(struct sk_buff *skb,
				struct ncsi_cmd_arg *nca)
{
	struct ncsi_cmd_svf_pkt *cmd;

	if (!nca)
		return 4;

	cmd = (struct ncsi_cmd_svf_pkt *)skb_put(skb, sizeof(*cmd));
	memset(cmd, 0, sizeof(*cmd));
	cmd->vlan = htons(nca->nca_words[0]);
	cmd->index = nca->nca_bytes[2];
	cmd->enable = nca->nca_bytes[3];
	return ncsi_cmd_build_header(&cmd->cmd.common, nca);
}

static int ncsi_cmd_handler_ev(struct sk_buff *skb,
			       struct ncsi_cmd_arg *nca)
{
	struct ncsi_cmd_ev_pkt *cmd;

	if (!nca)
		return 4;

	cmd = (struct ncsi_cmd_ev_pkt *)skb_put(skb, sizeof(*cmd));
	memset(cmd, 0, sizeof(*cmd));
	cmd->mode = nca->nca_bytes[0];
	return ncsi_cmd_build_header(&cmd->cmd.common, nca);
}

static int ncsi_cmd_handler_sma(struct sk_buff *skb,
				struct ncsi_cmd_arg *nca)
{
	struct ncsi_cmd_sma_pkt *cmd;
	int i;

	if (!nca)
		return 8;

	cmd = (struct ncsi_cmd_sma_pkt *)skb_put(skb, sizeof(*cmd));
	memset(cmd, 0, sizeof(*cmd));
	for (i = 0; i < 6; i++)
		cmd->mac[i] = nca->nca_bytes[i];
	cmd->index = nca->nca_bytes[6];
	cmd->at_e = nca->nca_bytes[7];
	return ncsi_cmd_build_header(&cmd->cmd.common, nca);
}

static int ncsi_cmd_handler_ebf(struct sk_buff *skb,
				struct ncsi_cmd_arg *nca)
{
	struct ncsi_cmd_ebf_pkt *cmd;

	if (!nca)
		return 4;

	cmd = (struct ncsi_cmd_ebf_pkt *)skb_put(skb, sizeof(*cmd));
	memset(cmd, 0, sizeof(*cmd));
	cmd->mode = htonl(nca->nca_dwords[0]);
	return ncsi_cmd_build_header(&cmd->cmd.common, nca);
}

static int ncsi_cmd_handler_egmf(struct sk_buff *skb,
				 struct ncsi_cmd_arg *nca)
{
	struct ncsi_cmd_egmf_pkt *cmd;

	if (!nca)
		return 4;

	cmd = (struct ncsi_cmd_egmf_pkt *)skb_put(skb, sizeof(*cmd));
	memset(cmd, 0, sizeof(*cmd));
	cmd->mode = htonl(nca->nca_dwords[0]);
	return ncsi_cmd_build_header(&cmd->cmd.common, nca);
}

static int ncsi_cmd_handler_snfc(struct sk_buff *skb,
				 struct ncsi_cmd_arg *nca)
{
	struct ncsi_cmd_snfc_pkt *cmd;

	if (!nca)
		return 4;

	cmd = (struct ncsi_cmd_snfc_pkt *)skb_put(skb, sizeof(*cmd));
	memset(cmd, 0, sizeof(*cmd));
	cmd->mode = nca->nca_bytes[0];
	return ncsi_cmd_build_header(&cmd->cmd.common, nca);
}

static struct ncsi_cmd_handler {
	unsigned char	nch_type;
	int		(*nch_handler)(struct sk_buff *skb,
				       struct ncsi_cmd_arg *nca);
} ncsi_cmd_handlers[] = {
	{ NCSI_PKT_CMD_CIS,   ncsi_cmd_handler_default },
	{ NCSI_PKT_CMD_SP,    ncsi_cmd_handler_sp	 },
	{ NCSI_PKT_CMD_DP,    ncsi_cmd_handler_default },
	{ NCSI_PKT_CMD_EC,    ncsi_cmd_handler_default },
	{ NCSI_PKT_CMD_DC,    ncsi_cmd_handler_dc      },
	{ NCSI_PKT_CMD_RC,    ncsi_cmd_handler_rc      },
	{ NCSI_PKT_CMD_ECNT,  ncsi_cmd_handler_default },
	{ NCSI_PKT_CMD_DCNT,  ncsi_cmd_handler_default },
	{ NCSI_PKT_CMD_AE,    ncsi_cmd_handler_ae      },
	{ NCSI_PKT_CMD_SL,    ncsi_cmd_handler_sl      },
	{ NCSI_PKT_CMD_GLS,   ncsi_cmd_handler_default },
	{ NCSI_PKT_CMD_SVF,   ncsi_cmd_handler_svf     },
	{ NCSI_PKT_CMD_EV,    ncsi_cmd_handler_ev      },
	{ NCSI_PKT_CMD_DV,    ncsi_cmd_handler_default },
	{ NCSI_PKT_CMD_SMA,   ncsi_cmd_handler_sma     },
	{ NCSI_PKT_CMD_EBF,   ncsi_cmd_handler_ebf     },
	{ NCSI_PKT_CMD_DBF,   ncsi_cmd_handler_default },
	{ NCSI_PKT_CMD_EGMF,  ncsi_cmd_handler_egmf    },
	{ NCSI_PKT_CMD_DGMF,  ncsi_cmd_handler_default },
	{ NCSI_PKT_CMD_SNFC,  ncsi_cmd_handler_snfc    },
	{ NCSI_PKT_CMD_GVI,   ncsi_cmd_handler_default },
	{ NCSI_PKT_CMD_GC,    ncsi_cmd_handler_default },
	{ NCSI_PKT_CMD_GP,    ncsi_cmd_handler_default },
	{ NCSI_PKT_CMD_GCPS,  ncsi_cmd_handler_default },
	{ NCSI_PKT_CMD_GNS,   ncsi_cmd_handler_default },
	{ NCSI_PKT_CMD_GNPTS, ncsi_cmd_handler_default },
	{ 0,                  NULL                     }
};

static struct ncsi_req *ncsi_alloc_cmd_req(struct ncsi_cmd_arg *nca)
{
	struct ncsi_dev_priv *ndp = nca->nca_ndp;
	struct ncsi_dev *nd = &ndp->ndp_ndev;
	struct net_device *dev = nd->nd_dev;
	int hlen = LL_RESERVED_SPACE(dev);
	int tlen = dev->needed_tailroom;
	int len = hlen + tlen;
	struct sk_buff *skb;
	struct ncsi_req *nr;

	nr = ncsi_alloc_req(ndp);
	if (!nr)
		return NULL;

	/* NCSI command packet has 16-bytes header, payload,
	 * 4-bytes checksum and optional padding.
	 */
	len += sizeof(struct ncsi_cmd_pkt_hdr);
	len += 4;
	if (nca->nca_payload < 26)
		len += 26;
	else
		len += nca->nca_payload;

	/* Allocate skb */
	skb = alloc_skb(len, GFP_ATOMIC);
	if (!skb) {
		ncsi_free_req(nr, false, false);
		return NULL;
	}

	nr->nr_cmd = skb;
	skb_reserve(skb, hlen);
	skb_reset_network_header(skb);

	skb->dev = dev;
	skb->protocol = htons(ETH_P_NCSI);

	if (nca->nca_nlh) {
		NCSI_CB(skb).nsp_valid = 1;
		memcpy(&NCSI_CB(skb).nsp_nlh, nca->nca_nlh,
		       nlmsg_total_size(sizeof(struct ncsi_msg)));
	} else {
		NCSI_CB(skb).nsp_valid = 0;
	}

	return nr;
}

int ncsi_xmit_cmd(struct ncsi_cmd_arg *nca)
{
	struct ncsi_req *nr;
	struct ethhdr *eh;
	struct ncsi_cmd_handler *nch;
	int i, ret;

	/* Search for the handler */
	nch = ncsi_cmd_handlers;
	while (nch->nch_handler) {
		if (nch->nch_type == nca->nca_type)
			break;
		nch++;
	}

	if (!nch->nch_handler) {
		pr_info("%s: Cannot send packet with type 0x%x\n",
			__func__, nca->nca_type);
		return -ENOENT;
	}

	/* Get packet payload length and allocate the request */
	nca->nca_payload = nch->nch_handler(NULL, NULL);
	nr = ncsi_alloc_cmd_req(nca);
	if (!nr)
		return -ENOMEM;

	/* Prepare the packet */
	nca->nca_id = nr->nr_id;
	ret = nch->nch_handler(nr->nr_cmd, nca);
	if (ret)
		goto out;

	/* Fill the ethernet header */
	eh = (struct ethhdr *)skb_push(nr->nr_cmd, sizeof(*eh));
	eh->h_proto = htons(ETH_P_NCSI);
	for (i = 0; i < ETH_ALEN; i++) {
		eh->h_dest[i] = 0xff;
		eh->h_source[i] = 0xff;
	}

	/* Send NCSI packet */
	skb_get(nr->nr_cmd);
	ret = dev_queue_xmit_sk(NULL, nr->nr_cmd);
	if (ret)
		goto out;

	/* Start the timer for the request that might not have
	 * corresponding response. I'm not sure 1 second delay
	 * here is enough. Anyway, NCSI is internal network, so
	 * the responsiveness should be as fast as enough.
	 */
	nr->nr_timer_enabled = true;
	mod_timer(&nr->nr_timer, jiffies + 1 * HZ);

	return 0;
out:
	ncsi_free_req(nr, false, false);
	return ret;
}

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

static int ncsi_validate_rsp_pkt(struct ncsi_req *nr,
				 unsigned short payload)
{
	struct ncsi_rsp_pkt_hdr *h;
	unsigned char *stream;
	__be32 *checksum, csum;
	__be32 high, low;
	int i;

	/*
	 * Check NCSI packet header. We don't need validate
	 * the packet type, which should have been checked
	 * before calling this function.
	 */
	h = (struct ncsi_rsp_pkt_hdr *)skb_network_header(nr->nr_rsp);
	if (h->common.revision != NCSI_PKT_REVISION)
		return -EINVAL;
	if (ntohs(h->common.length) != payload)
		return -EINVAL;

	/* Check on code and reason */
	if (ntohs(h->code) != NCSI_PKT_RSP_C_COMPLETED ||
	    ntohs(h->reason) != NCSI_PKT_RSP_R_NO_ERROR)
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

static int ncsi_rsp_handler_default(struct ncsi_req *nr)
{
	return ncsi_validate_rsp_pkt(nr, 0);
}

static int ncsi_rsp_handler_cis(struct ncsi_req *nr)
{
	struct ncsi_rsp_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_package *np;
	struct ncsi_channel *nc;
	int ret;

	ret = ncsi_validate_rsp_pkt( nr, 4);
	if (ret)
		return ret;

	rsp = (struct ncsi_rsp_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel, &np, &nc);
	if (!np)
		return -ENODEV;

	/* Add the channel if necessary */
	if (!nc)
		nc = ncsi_add_channel(np,
			NCSI_CHANNEL_INDEX(rsp->rsp.common.channel));
	else if (nc->nc_state == ncsi_channel_state_deselected_initial)
		nc->nc_state = ncsi_channel_state_deselected_ready;
	else if (nc->nc_state == ncsi_channel_state_selected_initial)
		nc->nc_state = ncsi_channel_state_selected_ready;

	return nc ? 0 : -ENODEV;
}

static int ncsi_rsp_handler_sp(struct ncsi_req *nr)
{
	struct ncsi_rsp_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_package *np;
	struct ncsi_channel *nc;
	int ret;

	ret = ncsi_validate_rsp_pkt( nr, 4);
	if (ret)
		return ret;

	/*
	 * Add the package if it's not existing. Otherwise,
	 * to change the state of its child channels.
	 */
	rsp = (struct ncsi_rsp_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      &np, NULL);
	if (!np) {
		np = ncsi_add_package(ndp,
			NCSI_PACKAGE_INDEX(rsp->rsp.common.channel));
		if (!np)
			return -ENODEV;
	}

	NCSI_FOR_EACH_CHANNEL(np, nc) {
		if (nc->nc_state == ncsi_channel_state_deselected_initial)
			nc->nc_state = ncsi_channel_state_selected_initial;
		else if (nc->nc_state == ncsi_channel_state_deselected_ready)
			nc->nc_state = ncsi_channel_state_selected_ready;
	}

	return 0;
}

static int ncsi_rsp_handler_dp(struct ncsi_req *nr)
{
	struct ncsi_rsp_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_package *np;
	struct ncsi_channel *nc;
	int ret;

	ret = ncsi_validate_rsp_pkt( nr, 4);
	if (ret)
		return ret;

	/* Find the package */
	rsp = (struct ncsi_rsp_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      &np, NULL);
	if (!np)
		return -ENODEV;

	/* Change state of all channels attached to the package */
	NCSI_FOR_EACH_CHANNEL(np, nc) {
		if (nc->nc_state == ncsi_channel_state_selected_initial)
			nc->nc_state = ncsi_channel_state_deselected_initial;
		else if (nc->nc_state == ncsi_channel_state_selected_ready)
			nc->nc_state = ncsi_channel_state_deselected_ready;
	}

	return 0;
}

static int ncsi_rsp_handler_ec(struct ncsi_req *nr)
{
	struct ncsi_rsp_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_channel *nc;
	struct ncsi_channel_mode *ncm;
	int ret;

	ret = ncsi_validate_rsp_pkt( nr, 4);
	if (ret)
		return ret;

	/* Find the package and channel */
	rsp = (struct ncsi_rsp_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      NULL, &nc);
	if (!nc)
		return -ENODEV;

	ncm = &nc->nc_modes[NCSI_MODE_ENABLE];
	if (ncm->ncm_enable)
		return -EBUSY;

	ncm->ncm_enable = 1;
	return 0;
}

static int ncsi_rsp_handler_dc(struct ncsi_req *nr)
{
	struct ncsi_rsp_pkt *rsp;
	struct ncsi_dev_priv *ndp= nr->nr_ndp;
	struct ncsi_channel *nc;
	struct ncsi_channel_mode *ncm;
	int ret;

	ret = ncsi_validate_rsp_pkt( nr, 4);
	if (ret)
		return ret;

	/* Find the package and channel */
	rsp = (struct ncsi_rsp_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      NULL, &nc);
	if (!nc)
		return -ENODEV;

	ncm = &nc->nc_modes[NCSI_MODE_ENABLE];
	if (!ncm->ncm_enable)
		return -EBUSY;

	ncm->ncm_enable = 0;;
	return 0;
}

static int ncsi_rsp_handler_rc(struct ncsi_req *nr)
{
	struct ncsi_rsp_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_channel *nc;
	int ret;

	ret = ncsi_validate_rsp_pkt( nr, 4);
	if (ret)
		return ret;

	/* Find the package and channel */
	rsp = (struct ncsi_rsp_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      NULL, &nc);
	if (!nc)
		return -ENODEV;

	/* Update state for the specified channel */
	if (nc->nc_state == ncsi_channel_state_deselected_ready)
		nc->nc_state = ncsi_channel_state_deselected_initial;
	else if (nc->nc_state == ncsi_channel_state_selected_ready)
		nc->nc_state = ncsi_channel_state_selected_initial;

	return 0;
}

static int ncsi_rsp_handler_ecnt(struct ncsi_req *nr)
{
	struct ncsi_rsp_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_channel *nc;
	struct ncsi_channel_mode *ncm;
	int ret;

	ret = ncsi_validate_rsp_pkt( nr, 4);
	if (ret)
		return ret;

	/* Find the package and channel */
	rsp = (struct ncsi_rsp_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      NULL, &nc);
	if (!nc)
		return -ENODEV;

	ncm = &nc->nc_modes[NCSI_MODE_TX_ENABLE];
	if (ncm->ncm_enable)
		return -EBUSY;

	ncm->ncm_enable = 1;
	return 0;
}

static int ncsi_rsp_handler_dcnt(struct ncsi_req *nr)
{
	struct ncsi_rsp_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_channel *nc;
	struct ncsi_channel_mode *ncm;
	int ret;

	ret = ncsi_validate_rsp_pkt( nr, 4);
	if (ret)
		return ret;

	/* Find the package and channel */
	rsp = (struct ncsi_rsp_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      NULL, &nc);
	if (!nc)
		return -ENODEV;

	ncm = &nc->nc_modes[NCSI_MODE_TX_ENABLE];
	if (!ncm->ncm_enable)
		return -EBUSY;

	ncm->ncm_enable = 1;
	return 0;
}

static int ncsi_rsp_handler_ae(struct ncsi_req *nr)
{
	struct ncsi_cmd_ae_pkt *cmd;
	struct ncsi_rsp_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_channel *nc;
	struct ncsi_channel_mode *ncm;
	int ret;

	ret = ncsi_validate_rsp_pkt( nr, 4);
	if (ret)
		return ret;

	/* Find the package and channel */
	rsp = (struct ncsi_rsp_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      NULL, &nc);
	if (!nc)
		return -ENODEV;

	/* Check if the AEN has been enabled */
	ncm = &nc->nc_modes[NCSI_MODE_AEN];
	if (ncm->ncm_enable)
		return -EBUSY;

	/* Update to AEN configuration */
	cmd = (struct ncsi_cmd_ae_pkt *)skb_network_header(nr->nr_cmd);
	ncm->ncm_enable = 1;
	ncm->ncm_data[0] = cmd->mc_id;
	ncm->ncm_data[1] = ntohl(cmd->mode);

	return 0;
}

static int ncsi_rsp_handler_sl(struct ncsi_req *nr)
{
	struct ncsi_cmd_sl_pkt *cmd;
	struct ncsi_rsp_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_channel *nc;
	struct ncsi_channel_mode *ncm;
	int ret;

	ret = ncsi_validate_rsp_pkt( nr, 4);
	if (ret)
		return ret;

	/* Find the package and channel */
	rsp = (struct ncsi_rsp_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      NULL, &nc);
	if (!nc)
		return -ENODEV;

	cmd = (struct ncsi_cmd_sl_pkt *)skb_network_header(nr->nr_cmd);
	ncm = &nc->nc_modes[NCSI_MODE_LINK];
	ncm->ncm_data[0] = ntohl(cmd->mode);
	ncm->ncm_data[1] = ntohl(cmd->oem_mode);

	return 0;
}

static int ncsi_rsp_handler_gls(struct ncsi_req *nr)
{
	struct ncsi_rsp_gls_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_channel *nc;
	struct ncsi_channel_mode *ncm;
	int ret;

	ret = ncsi_validate_rsp_pkt( nr, 16);
	if (ret)
		return ret;

	/* Find the package and channel */
	rsp = (struct ncsi_rsp_gls_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      NULL, &nc);
	if (!nc)
		return -ENODEV;

	ncm = &nc->nc_modes[NCSI_MODE_LINK];
	ncm->ncm_data[2] = ntohl(rsp->status);
	ncm->ncm_data[3] = ntohl(rsp->other);
	ncm->ncm_data[4] = ntohl(rsp->oem_status);

	return 0;
}

static int ncsi_rsp_handler_svf(struct ncsi_req *nr)
{
	struct ncsi_cmd_svf_pkt *cmd;
	struct ncsi_rsp_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_channel *nc;
	struct ncsi_channel_filter *ncf;
	unsigned short vlan;
	int ret;

	ret = ncsi_validate_rsp_pkt( nr, 4);
	if (ret)
		return ret;

	/* Find the package and channel */
	rsp = (struct ncsi_rsp_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      NULL, &nc);
	if (!nc)
		return -ENODEV;

	cmd = (struct ncsi_cmd_svf_pkt *)skb_network_header(nr->nr_cmd);
	ncf = nc->nc_filters[NCSI_FILTER_VLAN];
	if (!ncf)
		return -ENOENT;
	if (cmd->index >= ncf->ncf_total)
		return -ERANGE;

	/* Add or remove the VLAN filter */
	if (!(cmd->enable & 0x1)) {
		ret = ncsi_del_channel_filter(nc, NCSI_FILTER_VLAN, cmd->index);
	} else {
		vlan = ntohs(cmd->vlan);
		ret = ncsi_add_channel_filter(nc, NCSI_FILTER_VLAN, &vlan);
	}

	return ret;
}

static int ncsi_rsp_handler_ev(struct ncsi_req *nr)
{
	struct ncsi_cmd_ev_pkt *cmd;
	struct ncsi_rsp_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_channel *nc;
	struct ncsi_channel_mode *ncm;
	int ret;

	ret = ncsi_validate_rsp_pkt( nr, 4);
	if (ret)
		return ret;

	/* Find the package and channel */
	rsp = (struct ncsi_rsp_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      NULL, &nc);
	if (!nc)
		return -ENODEV;

	/* Check if VLAN mode has been enabled */
	ncm = &nc->nc_modes[NCSI_MODE_VLAN];
	if (ncm->ncm_enable)
		return -EBUSY;

	/* Update to VLAN mode */
	cmd = (struct ncsi_cmd_ev_pkt *)skb_network_header(nr->nr_cmd);
	ncm->ncm_enable = 1;
	ncm->ncm_data[0] = ntohl(cmd->mode);

	return 0;
}

static int ncsi_rsp_handler_dv(struct ncsi_req *nr)
{
	struct ncsi_rsp_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_channel *nc;
	struct ncsi_channel_mode *ncm;
	int ret;

	ret = ncsi_validate_rsp_pkt( nr, 4);
	if (ret)
		return ret;

	/* Find the package and channel */
	rsp = (struct ncsi_rsp_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      NULL, &nc);
	if (!nc)
		return -ENODEV;

	/* Check if VLAN mode has been enabled */
	ncm = &nc->nc_modes[NCSI_MODE_VLAN];
	if (!ncm->ncm_enable)
		return -EBUSY;

	/* Update to VLAN mode */
	ncm->ncm_enable = 0;
	return 0;
}

static int ncsi_rsp_handler_sma(struct ncsi_req *nr)
{
	struct ncsi_cmd_sma_pkt *cmd;
	struct ncsi_rsp_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_channel *nc;
	struct ncsi_channel_filter *ncf;
	void *bitmap;
	int ret;

	ret = ncsi_validate_rsp_pkt( nr, 4);
	if (ret)
		return ret;

	/* Find the package and channel */
	rsp = (struct ncsi_rsp_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      NULL, &nc);
	if (!nc)
		return -ENODEV;

	/* According to NCSI spec 1.01, the mixed filter table
	 * isn't supported yet.
	 */
	cmd = (struct ncsi_cmd_sma_pkt *)skb_network_header(nr->nr_cmd);
	switch (cmd->at_e >> 5) {
	case 0x0:	/* UC address */
		ncf = nc->nc_filters[NCSI_FILTER_UC];
		break;
	case 0x1:	/* MC address */
		ncf = nc->nc_filters[NCSI_FILTER_MC];
		break;
	default:
		return -EINVAL;
	}

	/* Sanity check on the filter */
	if (!ncf)
		return -ENOENT;
	else if (cmd->index >= ncf->ncf_total)
		return -ERANGE;

	bitmap = &ncf->ncf_bitmap;
	if (cmd->at_e & 0x1) {
		if (test_and_set_bit(cmd->index, bitmap))
			return -EBUSY;
		memcpy(ncf->ncf_data + 6 * cmd->index, cmd->mac, 6);
	} else {
		if (!test_and_clear_bit(cmd->index, bitmap))
			return -EBUSY;

		memset(ncf->ncf_data + 6 * cmd->index, 0, 6);
	}

	return 0;
}

static int ncsi_rsp_handler_ebf(struct ncsi_req *nr)
{
	struct ncsi_cmd_ebf_pkt *cmd;
	struct ncsi_rsp_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_channel *nc;
	struct ncsi_channel_mode *ncm;
	int ret;

	ret = ncsi_validate_rsp_pkt( nr, 4);
	if (ret)
		return ret;

	/* Find the package and channel */
	rsp = (struct ncsi_rsp_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel, NULL, &nc);
	if (!nc)
		return -ENODEV;

	/* Check if broadcast filter has been enabled */
	ncm = &nc->nc_modes[NCSI_MODE_BC];
	if (ncm->ncm_enable)
		return -EBUSY;

	/* Update to broadcast filter mode */
	cmd = (struct ncsi_cmd_ebf_pkt *)skb_network_header(nr->nr_cmd);
	ncm->ncm_enable = 1;
	ncm->ncm_data[0] = ntohl(cmd->mode);

	return 0;
}

static int ncsi_rsp_handler_dbf(struct ncsi_req *nr)
{
	struct ncsi_rsp_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_channel *nc;
	struct ncsi_channel_mode *ncm;
	int ret;

	ret = ncsi_validate_rsp_pkt( nr, 4);
	if (ret)
		return ret;

	rsp = (struct ncsi_rsp_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      NULL, &nc);
	if (!nc)
		return -ENODEV;

	/* Check if broadcast filter isn't enabled */
	ncm = &nc->nc_modes[NCSI_MODE_BC];
	if (!ncm->ncm_enable)
		return -EBUSY;

	/* Update to broadcast filter mode */
	ncm->ncm_enable = 0;
	ncm->ncm_data[0] = 0;

	return 0;
}

static int ncsi_rsp_handler_egmf(struct ncsi_req *nr)
{
	struct ncsi_cmd_egmf_pkt *cmd;
	struct ncsi_rsp_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_channel *nc;
	struct ncsi_channel_mode *ncm;
	int ret;

	ret = ncsi_validate_rsp_pkt( nr, 4);
	if (ret)
		return ret;

	/* Find the channel */
	rsp = (struct ncsi_rsp_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      NULL, &nc);
	if (!nc)
		return -ENODEV;

	/* Check if multicast filter has been enabled */
	ncm = &nc->nc_modes[NCSI_MODE_MC];
	if (ncm->ncm_enable)
		return -EBUSY;

	/* Update to multicast filter mode */
	cmd = (struct ncsi_cmd_egmf_pkt *)skb_network_header(nr->nr_cmd);
	ncm->ncm_enable = 1;
	ncm->ncm_data[0] = ntohl(cmd->mode);

	return 0;
}

static int ncsi_rsp_handler_dgmf(struct ncsi_req *nr)
{
	struct ncsi_rsp_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_channel *nc;
	struct ncsi_channel_mode *ncm;
	int ret;

	ret = ncsi_validate_rsp_pkt( nr, 4);
	if (ret)
		return ret;

	rsp = (struct ncsi_rsp_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      NULL, &nc);
	if (!nc)
		return -ENODEV;

	/* Check if multicast filter has been enabled */
	ncm = &nc->nc_modes[NCSI_MODE_MC];
	if (!ncm->ncm_enable)
		return -EBUSY;

	/* Update to multicast filter mode */
	ncm->ncm_enable = 0;
	ncm->ncm_data[0] = 0;

	return 0;
}

static int ncsi_rsp_handler_snfc(struct ncsi_req *nr)
{
	struct ncsi_cmd_snfc_pkt *cmd;
	struct ncsi_rsp_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_channel *nc;
	struct ncsi_channel_mode *ncm;
	int ret;

	ret = ncsi_validate_rsp_pkt( nr, 4);
	if (ret)
		return ret;

	/* Find the channel */
	rsp = (struct ncsi_rsp_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      NULL, &nc);
	if (!nc)
		return -ENODEV;

	/* Check if flow control has been enabled */
	ncm = &nc->nc_modes[NCSI_MODE_FC];
	if (ncm->ncm_enable)
		return -EBUSY;

	/* Update to flow control mode */
	cmd = (struct ncsi_cmd_snfc_pkt *)skb_network_header(nr->nr_cmd);
	ncm->ncm_enable = 1;
	ncm->ncm_data[0] = cmd->mode;

	return 0;
}

static int ncsi_rsp_handler_gvi(struct ncsi_req *nr)
{
	struct ncsi_rsp_gvi_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_channel *nc;
	struct ncsi_channel_version *ncv;
	int i, ret;

	ret = ncsi_validate_rsp_pkt( nr, 36);
	if (ret)
		return ret;

	/* Find the channel */
	rsp = (struct ncsi_rsp_gvi_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      NULL, &nc);
	if (!nc)
		return -ENODEV;

	/* Update to channel's version info */
	ncv = &nc->nc_version;
	ncv->ncv_version = ntohl(rsp->ncsi_version);
	ncv->ncv_alpha2 = rsp->alpha2;
	memcpy(ncv->ncv_fw_name, rsp->fw_name, 12);
	ncv->ncv_fw_version = ntohl(rsp->fw_version);
	for (i = 0; i < 4; i++)
		ncv->ncv_pci_ids[i] = ntohs(rsp->pci_ids[i]);
	ncv->ncv_mf_id = ntohl(rsp->mf_id);

	return 0;
}

static int ncsi_rsp_handler_gc(struct ncsi_req *nr)
{
	struct ncsi_rsp_gc_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_channel *nc;
	struct ncsi_channel_filter *ncf;
	size_t size, entry_size;
	int cnt, i, ret;

	ret = ncsi_validate_rsp_pkt( nr, 32);
	if (ret)
		return ret;

	/* Find the channel */
	rsp = (struct ncsi_rsp_gc_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      NULL, &nc);
	if (!nc)
		return -ENODEV;

	/* Update channel's capabilities */
	nc->nc_caps[NCSI_CAP_GENERIC].ncc_cap = ntohl(rsp->cap) &
						NCSI_CAP_GENERIC_MASK;
	nc->nc_caps[NCSI_CAP_BC].ncc_cap = ntohl(rsp->bc_cap) &
					   NCSI_CAP_BC_MASK;
	nc->nc_caps[NCSI_CAP_MC].ncc_cap = ntohl(rsp->mc_cap) &
					   NCSI_CAP_MC_MASK;
	nc->nc_caps[NCSI_CAP_BUFFER].ncc_cap = ntohl(rsp->buf_cap);
	nc->nc_caps[NCSI_CAP_AEN].ncc_cap = ntohl(rsp->aen_cap) &
					    NCSI_CAP_AEN_MASK;
	nc->nc_caps[NCSI_CAP_VLAN].ncc_cap = rsp->vlan_mode &
					     NCSI_CAP_VLAN_MASK;

	/* Build filters */
	for (i = 0; i < NCSI_FILTER_MAX; i++) {
		switch (i) {
		case NCSI_FILTER_VLAN:
			cnt = rsp->vlan_cnt;
			entry_size = 2;
			break;
		case NCSI_FILTER_MIXED:
			cnt = rsp->mixed_cnt;
			entry_size = 6;
			break;
		case NCSI_FILTER_MC:
			cnt = rsp->mc_cnt;
			entry_size = 6;
			break;
		case NCSI_FILTER_UC:
			cnt = rsp->uc_cnt;
			entry_size = 6;
			break;
		default:
			continue;
		}

		if (!cnt || nc->nc_filters[i])
			continue;

		size = sizeof(*ncf) + cnt * entry_size;
		ncf = kzalloc(size, GFP_ATOMIC);
		if (!ncf) {
			pr_warn("%s: Cannot alloc filter table (%d)\n",
				__func__, i);
			return -ENOMEM;
		}

		ncf->ncf_index = i;
		ncf->ncf_total = cnt;
		ncf->ncf_bitmap = 0x0ul;
		nc->nc_filters[i] = ncf;
	}

	return 0;
}

static int ncsi_rsp_handler_gp(struct ncsi_req *nr)
{
	struct ncsi_pkt_hdr *h;
	struct ncsi_rsp_gp_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_channel *nc;
	unsigned short length, enable, vlan;
	unsigned char *pdata;
	int table, i, ret;

	/*
	 * The get parameter response packet has variable length.
	 * The payload should be figured out from the packet
	 * header, instead of the fixed one we have for other types
	 * of packets.
	 */
	h = (struct ncsi_pkt_hdr *)skb_network_header(nr->nr_rsp);
	length = ntohs(h->length);
	if (length < 32)
		return -EINVAL;

	ret = ncsi_validate_rsp_pkt( nr, length);
	if (ret)
		return ret;

	/* Find the channel */
	rsp = (struct ncsi_rsp_gp_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      NULL, &nc);
	if (!nc)
		return -ENODEV;

	/* Modes with explicit enabled indications */
	if (ntohl(rsp->valid_modes) & 0x1) {	/* BC filter mode */
		nc->nc_modes[NCSI_MODE_BC].ncm_enable = 1;
		nc->nc_modes[NCSI_MODE_BC].ncm_data[0] = ntohl(rsp->bc_mode);
	}
	if (ntohl(rsp->valid_modes) & 0x2)	/* Channel enabled */
		nc->nc_modes[NCSI_MODE_ENABLE].ncm_enable = 1;
	if (ntohl(rsp->valid_modes) & 0x4)	/* Channel Tx enabled */
		nc->nc_modes[NCSI_MODE_TX_ENABLE].ncm_enable = 1;
	if (ntohl(rsp->valid_modes) & 0x8)	/* MC filter mode */
		nc->nc_modes[NCSI_MODE_MC].ncm_enable = 1;

	/* Modes without explicit enabled indications */
	nc->nc_modes[NCSI_MODE_LINK].ncm_enable = 1;
	nc->nc_modes[NCSI_MODE_LINK].ncm_data[0] = ntohl(rsp->link_mode);
	nc->nc_modes[NCSI_MODE_VLAN].ncm_enable = 1;
	nc->nc_modes[NCSI_MODE_VLAN].ncm_data[0] = rsp->vlan_mode;
	nc->nc_modes[NCSI_MODE_FC].ncm_enable = 1;
	nc->nc_modes[NCSI_MODE_FC].ncm_data[0] = rsp->fc_mode;
	nc->nc_modes[NCSI_MODE_AEN].ncm_enable = 1;
	nc->nc_modes[NCSI_MODE_AEN].ncm_data[0] = ntohl(rsp->aen_mode);

	/* MAC addresses filter table */
	pdata = (unsigned char *)rsp + 48;
	enable = rsp->mac_enable;
	for (i = 0; i < rsp->mac_cnt; i++, pdata += 6) {
		if (i >= (nc->nc_filters[NCSI_FILTER_UC]->ncf_total +
			  nc->nc_filters[NCSI_FILTER_MC]->ncf_total))
			table = NCSI_FILTER_MIXED;
		else if (i >= nc->nc_filters[NCSI_FILTER_UC]->ncf_total)
			table = NCSI_FILTER_MC;
		else
			table = NCSI_FILTER_UC;

		if (!(enable & (0x1 << i)))
			continue;

		if (ncsi_find_channel_filter(nc, table, pdata) >= 0)
			continue;

		ncsi_add_channel_filter(nc, table, pdata);
	}

	/* VLAN filter table */
	enable = ntohs(rsp->vlan_enable);
	for (i = 0; i < rsp->vlan_cnt; i++, pdata += 2) {
		if (!(enable & (0x1 << i)))
			continue;

		vlan = ntohs(*(__be16 *)pdata);
		if (ncsi_find_channel_filter(nc, NCSI_FILTER_VLAN, &vlan) >= 0)
			continue;

		ncsi_add_channel_filter(nc, NCSI_FILTER_VLAN, &vlan);
	}

	return 0;
}

static int ncsi_rsp_handler_gcps(struct ncsi_req *nr)
{
	struct ncsi_rsp_gcps_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_channel *nc;
	struct ncsi_channel_stats *ncs;
	int ret;

	ret = ncsi_validate_rsp_pkt( nr, 172);
	if (ret)
		return ret;

	/* Find the channel */
	rsp = (struct ncsi_rsp_gcps_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      NULL, &nc);
	if (!nc)
		return -ENODEV;

	/* Update HNC's statistics */
	ncs = &nc->nc_stats;
	ncs->ncs_hnc_cnt_hi         = ntohl(rsp->cnt_hi);
	ncs->ncs_hnc_cnt_lo         = ntohl(rsp->cnt_lo);
	ncs->ncs_hnc_rx_bytes       = ntohl(rsp->rx_bytes);
	ncs->ncs_hnc_tx_bytes       = ntohl(rsp->tx_bytes);
	ncs->ncs_hnc_rx_uc_pkts     = ntohl(rsp->rx_uc_pkts);
	ncs->ncs_hnc_rx_mc_pkts     = ntohl(rsp->rx_mc_pkts);
	ncs->ncs_hnc_rx_bc_pkts     = ntohl(rsp->rx_bc_pkts);
	ncs->ncs_hnc_tx_uc_pkts     = ntohl(rsp->tx_uc_pkts);
	ncs->ncs_hnc_tx_mc_pkts     = ntohl(rsp->tx_mc_pkts);
	ncs->ncs_hnc_tx_bc_pkts     = ntohl(rsp->tx_bc_pkts);
	ncs->ncs_hnc_fcs_err        = ntohl(rsp->fcs_err);
	ncs->ncs_hnc_align_err      = ntohl(rsp->align_err);
	ncs->ncs_hnc_false_carrier  = ntohl(rsp->false_carrier);
	ncs->ncs_hnc_runt_pkts      = ntohl(rsp->runt_pkts);
	ncs->ncs_hnc_jabber_pkts    = ntohl(rsp->jabber_pkts);
	ncs->ncs_hnc_rx_pause_xon   = ntohl(rsp->rx_pause_xon);
	ncs->ncs_hnc_rx_pause_xoff  = ntohl(rsp->rx_pause_xoff);
	ncs->ncs_hnc_tx_pause_xon   = ntohl(rsp->tx_pause_xon);
	ncs->ncs_hnc_tx_pause_xoff  = ntohl(rsp->tx_pause_xoff);
	ncs->ncs_hnc_tx_s_collision = ntohl(rsp->tx_s_collision);
	ncs->ncs_hnc_tx_m_collision = ntohl(rsp->tx_m_collision);
	ncs->ncs_hnc_l_collision    = ntohl(rsp->l_collision);
	ncs->ncs_hnc_e_collision    = ntohl(rsp->e_collision);
	ncs->ncs_hnc_rx_ctl_frames  = ntohl(rsp->rx_ctl_frames);
	ncs->ncs_hnc_rx_64_frames   = ntohl(rsp->rx_64_frames);
	ncs->ncs_hnc_rx_127_frames  = ntohl(rsp->rx_127_frames);
	ncs->ncs_hnc_rx_255_frames  = ntohl(rsp->rx_255_frames);
	ncs->ncs_hnc_rx_511_frames  = ntohl(rsp->rx_511_frames);
	ncs->ncs_hnc_rx_1023_frames = ntohl(rsp->rx_1023_frames);
	ncs->ncs_hnc_rx_1522_frames = ntohl(rsp->rx_1522_frames);
	ncs->ncs_hnc_rx_9022_frames = ntohl(rsp->rx_9022_frames);
	ncs->ncs_hnc_tx_64_frames   = ntohl(rsp->tx_64_frames);
	ncs->ncs_hnc_tx_127_frames  = ntohl(rsp->tx_127_frames);
	ncs->ncs_hnc_tx_255_frames  = ntohl(rsp->tx_255_frames);
	ncs->ncs_hnc_tx_511_frames  = ntohl(rsp->tx_511_frames);
	ncs->ncs_hnc_tx_1023_frames = ntohl(rsp->tx_1023_frames);
	ncs->ncs_hnc_tx_1522_frames = ntohl(rsp->tx_1522_frames);
	ncs->ncs_hnc_tx_9022_frames = ntohl(rsp->tx_9022_frames);
	ncs->ncs_hnc_rx_valid_bytes = ntohl(rsp->rx_valid_bytes);
	ncs->ncs_hnc_rx_runt_pkts   = ntohl(rsp->rx_runt_pkts);
	ncs->ncs_hnc_rx_jabber_pkts = ntohl(rsp->rx_jabber_pkts);

	return 0;
}

static int ncsi_rsp_handler_gns(struct ncsi_req *nr)
{
	struct ncsi_rsp_gns_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_channel *nc;
	struct ncsi_channel_stats *ncs;
	int ret;

	ret = ncsi_validate_rsp_pkt( nr, 172);
	if (ret)
		return ret;

	/* Find the channel */
	rsp = (struct ncsi_rsp_gns_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      NULL, &nc);
	if (!nc)
		return -ENODEV;

	/* Update HNC's statistics */
	ncs = &nc->nc_stats;
	ncs->ncs_ncsi_rx_cmds       = ntohl(rsp->rx_cmds);
	ncs->ncs_ncsi_dropped_cmds  = ntohl(rsp->dropped_cmds);
	ncs->ncs_ncsi_cmd_type_errs = ntohl(rsp->cmd_type_errs);
	ncs->ncs_ncsi_cmd_csum_errs = ntohl(rsp->cmd_csum_errs);
	ncs->ncs_ncsi_rx_pkts       = ntohl(rsp->rx_pkts);
	ncs->ncs_ncsi_tx_pkts       = ntohl(rsp->tx_pkts);
	ncs->ncs_ncsi_tx_aen_pkts   = ntohl(rsp->tx_aen_pkts);

	return 0;
}

static int ncsi_rsp_handler_gnpts(struct ncsi_req *nr)
{
	struct ncsi_rsp_gnpts_pkt *rsp;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct ncsi_channel *nc;
	struct ncsi_channel_stats *ncs;
	int ret;

	ret = ncsi_validate_rsp_pkt( nr, 172);
	if (ret)
		return ret;

	/* Find the channel */
	rsp = (struct ncsi_rsp_gnpts_pkt *)skb_network_header(nr->nr_rsp);
	ncsi_find_package_and_channel(ndp, rsp->rsp.common.channel,
				      NULL, &nc);
	if (!nc)
		return -ENODEV;

	/* Update HNC's statistics */
	ncs = &nc->nc_stats;
	ncs->ncs_pt_tx_pkts        = ntohl(rsp->tx_pkts);
	ncs->ncs_pt_tx_dropped     = ntohl(rsp->tx_dropped);
	ncs->ncs_pt_tx_channel_err = ntohl(rsp->tx_channel_err);
	ncs->ncs_pt_tx_us_err      = ntohl(rsp->tx_us_err);
	ncs->ncs_pt_rx_pkts        = ntohl(rsp->rx_pkts);
	ncs->ncs_pt_rx_dropped     = ntohl(rsp->rx_dropped);
	ncs->ncs_pt_rx_channel_err = ntohl(rsp->rx_channel_err);
	ncs->ncs_pt_rx_us_err      = ntohl(rsp->rx_us_err);
	ncs->ncs_pt_rx_os_err      = ntohl(rsp->rx_os_err);

	return 0;
}

static struct ncsi_rsp_handler {
	unsigned char	nrh_type;
	int		(*nrh_handler)(struct ncsi_req *nr);
} ncsi_rsp_handlers[] = {
	{ NCSI_PKT_RSP_CIS,   ncsi_rsp_handler_cis     },
	{ NCSI_PKT_RSP_SP,    ncsi_rsp_handler_sp      },
	{ NCSI_PKT_RSP_DP,    ncsi_rsp_handler_dp      },
	{ NCSI_PKT_RSP_EC,    ncsi_rsp_handler_ec      },
	{ NCSI_PKT_RSP_DC,    ncsi_rsp_handler_dc      },
	{ NCSI_PKT_RSP_RC,    ncsi_rsp_handler_rc      },
	{ NCSI_PKT_RSP_ECNT,  ncsi_rsp_handler_ecnt    },
	{ NCSI_PKT_RSP_DCNT,  ncsi_rsp_handler_dcnt    },
	{ NCSI_PKT_RSP_AE,    ncsi_rsp_handler_ae      },
	{ NCSI_PKT_RSP_SL,    ncsi_rsp_handler_sl      },
	{ NCSI_PKT_RSP_GLS,   ncsi_rsp_handler_gls     },
	{ NCSI_PKT_RSP_SVF,   ncsi_rsp_handler_svf     },
	{ NCSI_PKT_RSP_EV,    ncsi_rsp_handler_ev      },
	{ NCSI_PKT_RSP_DV,    ncsi_rsp_handler_dv      },
	{ NCSI_PKT_RSP_SMA,   ncsi_rsp_handler_sma     },
	{ NCSI_PKT_RSP_EBF,   ncsi_rsp_handler_ebf     },
	{ NCSI_PKT_RSP_DBF,   ncsi_rsp_handler_dbf     },
	{ NCSI_PKT_RSP_EGMF,  ncsi_rsp_handler_egmf    },
	{ NCSI_PKT_RSP_DGMF,  ncsi_rsp_handler_dgmf    },
	{ NCSI_PKT_RSP_SNFC,  ncsi_rsp_handler_snfc    },
	{ NCSI_PKT_RSP_GVI,   ncsi_rsp_handler_gvi     },
	{ NCSI_PKT_RSP_GC,    ncsi_rsp_handler_gc      },
	{ NCSI_PKT_RSP_GP,    ncsi_rsp_handler_gp      },
	{ NCSI_PKT_RSP_GCPS,  ncsi_rsp_handler_gcps    },
	{ NCSI_PKT_RSP_GNS,   ncsi_rsp_handler_gns     },
	{ NCSI_PKT_RSP_GNPTS, ncsi_rsp_handler_gnpts   },
	{ NCSI_PKT_RSP_OEM,   ncsi_rsp_handler_default },
	{ 0,                  NULL                     }
};

#if 0
void ncsi_pkt_dump(struct sk_buff *skb)
{
	struct skb_shared_info *info = skb_shinfo(skb);
	skb_frag_t *frag;
	char *data;
	int limit, i;

	pr_info("head: 0x%p data: 0x%p tail: 0x%p end: 0x%p\n",
		skb->head, skb->data,
		skb_tail_pointer(skb), skb_end_pointer(skb));
	pr_info("mac_header: 0x%p network_header: 0x%p\n",
		skb_mac_header(skb), skb_network_header(skb));
	pr_info("len: 0x%x data_len: 0x%x truesize: 0x%x\n",
		skb->len, skb->data_len, skb->truesize);

	for (i = 0; i < info->nr_frags; i++) {
		frag = &info->frags[i];
		pr_info("FRAG[%d]: 0x%p offset: 0x%x size: 0x%x\n",
			i, frag->page.p, frag->page_offset, frag->size);
	}

	data = skb_mac_header(skb);
	limit = skb->len + sizeof(struct ethhdr);
	for (i = 0; i < limit; data++, i++) {
		if (i % 16 == 0)
			printk("\n%02x ", *data);
		else
			printk("%02x ", *data);
	}
}
#endif

int ncsi_rcv_rsp(struct sk_buff *skb, struct net_device *dev,
		 struct packet_type *pt, struct net_device *orig_dev)
{
	struct ncsi_rsp_handler *nrh = NULL;
	struct ncsi_dev *nd;
	struct ncsi_dev_priv *ndp;
	struct ncsi_req *nr;
	struct ncsi_pkt_hdr *hdr;
	int ret;

	/* Find the NCSI device */
	nd = ncsi_find_dev(dev);
	ndp = nd ? TO_NCSI_DEV_PRIV(nd) : NULL;
	if (!ndp)
		return -ENODEV;

	/* Check if it's AEN packet */
	hdr = (struct ncsi_pkt_hdr *)skb_network_header(skb);
	if (hdr->type == NCSI_PKT_AEN)
		return ncsi_aen_handler(ndp, skb);

	/* Find the handler */
	nrh = ncsi_rsp_handlers;
	while (nrh->nrh_handler) {
		if (nrh->nrh_type == hdr->type)
			break;

		nrh++;
	}

	if (!nrh->nrh_handler) {
		pr_warn("NCSI: Received unrecognized packet (0x%x)\n",
			hdr->type);
		return -ENOENT;
	}

	/* Associate with the request */
	nr = &ndp->ndp_reqs[hdr->id];
	spin_lock(&ndp->ndp_req_lock);
	if (!nr->nr_used) {
		spin_unlock(&ndp->ndp_req_lock);
		return -ENODEV;
	}

	nr->nr_rsp = skb;
	if (!nr->nr_timer_enabled) {
		spin_unlock(&ndp->ndp_req_lock);
		ret = -ENOENT;
		goto out;
	}

	/* Process the response */
	ret = nrh->nrh_handler(nr);

out:
	ncsi_free_req(nr, true, false);
	return ret;
}

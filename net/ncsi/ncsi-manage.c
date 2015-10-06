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
#include <linux/netlink.h>

#include <net/ncsi.h>
#include <net/net_namespace.h>
#include <net/sock.h>

#include "ncsi-pkt.h"
#include "internal.h"

LIST_HEAD(ncsi_dev_list);
DEFINE_SPINLOCK(ncsi_dev_lock);

int ncsi_find_channel_filter(struct ncsi_channel *nc, int table, void *data)
{
	struct ncsi_channel_filter *ncf;
	int idx, entry_size;
	void *bitmap;

	switch (table) {
	case NCSI_FILTER_VLAN:
		entry_size = 2;
		break;
	case NCSI_FILTER_UC:
	case NCSI_FILTER_MC:
	case NCSI_FILTER_MIXED:
		entry_size = 6;
		break;
	default:
		return -EINVAL;
	}

	/* Check if the filter table has been initialized */
	ncf = nc->nc_filters[table];
	if (!ncf)
		return -ENODEV;

	/* Check the valid entries one by one */
	bitmap = (void *)&ncf->ncf_bitmap;
	idx = -1;
	while ((idx = find_next_bit(bitmap, ncf->ncf_total, idx+1))
		< ncf->ncf_total) {
		if (!memcmp(ncf->ncf_data + entry_size * idx, data, entry_size))
			return idx;
	}

	return -ENOENT;
}

int ncsi_add_channel_filter(struct ncsi_channel *nc, int table, void *data)
{
	struct ncsi_channel_filter *ncf;
	int idx, entry_size;
	void *bitmap;

	/* Needn't add it if it's already existing */
	idx = ncsi_find_channel_filter(nc, table, data);
	if (idx >= 0)
		return idx;

	switch (table) {
	case NCSI_FILTER_VLAN:
		entry_size = 2;
		break;
	case NCSI_FILTER_UC:
	case NCSI_FILTER_MC:
	case NCSI_FILTER_MIXED:
		entry_size = 6;
		break;
	default:
		return -EINVAL;
	}

	/* Check if the filter table has been initialized */
	ncf = nc->nc_filters[table];
	if (!ncf)
		return -ENODEV;

	/* Propagate the filter */
	bitmap = (void *)&ncf->ncf_bitmap;
	do {
		idx = find_next_zero_bit(bitmap, ncf->ncf_total, 0);
		if (idx >= ncf->ncf_total)
			return -ENOSPC;
	} while (test_and_set_bit(idx, bitmap));

	memcpy(ncf->ncf_data + entry_size * idx, data, entry_size);
	return idx;
}

int ncsi_del_channel_filter(struct ncsi_channel *nc, int table, int index)
{
	struct ncsi_channel_filter *ncf;
	int entry_size;
	void *bitmap;

	switch (table) {
	case NCSI_FILTER_VLAN:
		entry_size = 2;
		break;
	case NCSI_FILTER_UC:
	case NCSI_FILTER_MC:
	case NCSI_FILTER_MIXED:
		entry_size = 6;
		break;
	default:
		return -EINVAL;
	}

	/* Check if the filter table has been initialized */
	ncf = nc->nc_filters[table];
	if (!ncf || index >= ncf->ncf_total)
		return -ENODEV;

	/* Check if the entry is valid */
	bitmap = (void *)&ncf->ncf_bitmap;
	if (test_and_clear_bit(index, bitmap))
		memset(ncf->ncf_data + entry_size * index, 0, entry_size);

	return 0;
}

struct ncsi_channel *ncsi_add_channel(struct ncsi_package *np, unsigned char id)
{
	struct ncsi_channel *nc, *tmp;
	int index;

	nc = kzalloc(sizeof(*nc), GFP_ATOMIC);
	if (!nc) {
		pr_warn("%s: Out of memory !\n", __func__);
		return NULL;
	}

	nc->nc_package = np;
	nc->nc_id = id;
	for (index = 0; index < NCSI_CAP_MAX; index++)
		nc->nc_caps[index].ncc_index = index;
	for (index = 0; index < NCSI_MODE_MAX; index++)
		nc->nc_modes[index].ncm_index = index;

	spin_lock(&np->np_channel_lock);
	tmp = ncsi_find_channel(np, id);
	if (tmp) {
		spin_unlock(&np->np_channel_lock);
		kfree(nc);
		return tmp;
	}
	list_add_tail_rcu(&nc->nc_node, &np->np_channels);
	spin_unlock(&np->np_channel_lock);

	atomic_inc(&np->np_channel_num);
	return nc;
}

struct ncsi_channel *ncsi_find_channel(struct ncsi_package *np,
				       unsigned char id)
{
	struct ncsi_channel *nc;

	NCSI_FOR_EACH_CHANNEL(np, nc) {
		if (nc->nc_id == id)
			return nc;
	}

	return NULL;
}

static void ncsi_release_channel(struct ncsi_channel *nc)
{
	struct ncsi_dev_priv *ndp = nc->nc_package->np_ndp;
	struct ncsi_package *np = nc->nc_package;
	struct ncsi_channel_filter *ncf;
	int i;

	/* Release filters */
	for (i = 0; i < NCSI_FILTER_MAX; i++) {
		ncf = nc->nc_filters[i];
		if (!ncf)
			continue;

		nc->nc_filters[i] = NULL;
		kfree(ncf);
	}

	/* Update active channel if necessary */
	if (ndp->ndp_active_channel == nc) {
		ndp->ndp_active_package = NULL;
		ndp->ndp_active_channel = NULL;
	}

	/* Remove and free channel */
	list_del_rcu(&nc->nc_node);
	kfree(nc);
	BUG_ON(atomic_dec_return(&np->np_channel_num) < 0);
}

struct ncsi_package *ncsi_add_package(struct ncsi_dev_priv *ndp,
				      unsigned char id)
{
	struct ncsi_package *np, *tmp;

	np = kzalloc(sizeof(*np), GFP_ATOMIC);
	if (!np) {
		pr_warn("%s: Out of memory !\n", __func__);
		return NULL;
	}

	np->np_id = id;
	np->np_ndp = ndp;
	spin_lock_init(&np->np_channel_lock);
	INIT_LIST_HEAD(&np->np_channels);

	spin_lock(&ndp->ndp_package_lock);
	tmp = ncsi_find_package(ndp, id);
	if (tmp) {
		spin_unlock(&ndp->ndp_package_lock);
		kfree(np);
		return tmp;
	}
	list_add_tail_rcu(&np->np_node, &ndp->ndp_packages);
	spin_unlock(&ndp->ndp_package_lock);

	atomic_inc(&ndp->ndp_package_num);
	return np;
}

struct ncsi_package *ncsi_find_package(struct ncsi_dev_priv *ndp,
				       unsigned char id)
{
	struct ncsi_package *np;

	NCSI_FOR_EACH_PACKAGE(ndp, np) {
		if (np->np_id == id)
			return np;
	}

	return NULL;
}

void ncsi_release_package(struct ncsi_package *np)
{
	struct ncsi_dev_priv *ndp = np->np_ndp;
	struct ncsi_channel *nc, *tmp;

	/* Release all child channels */
	spin_lock(&np->np_channel_lock);
	list_for_each_entry_safe(nc, tmp, &np->np_channels, nc_node)
		ncsi_release_channel(nc);
	spin_unlock(&np->np_channel_lock);

	/* Clear active package if necessary */
	if (ndp->ndp_active_package == np) {
		ndp->ndp_active_package = NULL;
		ndp->ndp_active_channel = NULL;
	}

	/* Remove and free package */
	list_del_rcu(&np->np_node);
	kfree(np);

	/* Decrease number of packages */
	BUG_ON(atomic_dec_return(&ndp->ndp_package_num) < 0);
}

void ncsi_find_package_and_channel(struct ncsi_dev_priv *ndp,
				   unsigned char id,
				   struct ncsi_package **np,
				   struct ncsi_channel **nc)
{
	struct ncsi_package *p;
	struct ncsi_channel *c;

	p = ncsi_find_package(ndp, NCSI_PACKAGE_INDEX(id));
	c = p ? ncsi_find_channel(p, NCSI_CHANNEL_INDEX(id)) : NULL;

	if (np)
		*np = p;
	if (nc)
		*nc = c;
}

/*
 * For two consective NCSI commands, the packet IDs shouldn't be
 * same. Otherwise, the bogus response might be replied. So the
 * available IDs are allocated in round-robin fasion.
 */
struct ncsi_req *ncsi_alloc_req(struct ncsi_dev_priv *ndp)
{
	struct ncsi_req *nr = NULL;
	int idx, limit = 256;

	spin_lock(&ndp->ndp_req_lock);

	/* Check if there is one available request until the ceiling */
	for (idx = atomic_read(&ndp->ndp_last_req_idx);
	     !nr && idx < limit; idx++) {
		if (ndp->ndp_reqs[idx].nr_used)
			continue;

		ndp->ndp_reqs[idx].nr_used = true;
		nr = &ndp->ndp_reqs[idx];
		atomic_inc(&ndp->ndp_last_req_idx);
		if (atomic_read(&ndp->ndp_last_req_idx) >= limit)
			atomic_set(&ndp->ndp_last_req_idx, 0);
	}

	/* Fail back to check from the starting cursor */
	for (idx = 0; !nr && idx < atomic_read(&ndp->ndp_last_req_idx); idx++) {
		if (ndp->ndp_reqs[idx].nr_used)
			continue;

		ndp->ndp_reqs[idx].nr_used = true;
		nr = &ndp->ndp_reqs[idx];
		atomic_inc(&ndp->ndp_last_req_idx);
		if (atomic_read(&ndp->ndp_last_req_idx) >= limit)
			atomic_set(&ndp->ndp_last_req_idx, 0);
	}

	spin_unlock(&ndp->ndp_req_lock);
	return nr;
}

void ncsi_free_req(struct ncsi_req *nr, bool check, bool timeout)
{
	struct ncsi_dev_priv *ndp = nr->nr_ndp;
	struct sk_buff *cmd, *rsp;

	if (nr->nr_timer_enabled) {
		nr->nr_timer_enabled = false;
		del_timer_sync(&nr->nr_timer);
	}

	spin_lock(&ndp->ndp_req_lock);
	cmd = nr->nr_cmd;
	rsp = nr->nr_rsp;
	nr->nr_cmd = NULL;
	nr->nr_rsp = NULL;
	nr->nr_used = false;
	spin_unlock(&ndp->ndp_req_lock);

	/* If the NCSI command was sent because of netlink
	 * messages, we need reply with the result or error.
	 */
	if (check && cmd && NCSI_CB(cmd).nsp_valid)
		ncsi_netlink_reply(&NCSI_CB(cmd).nsp_nlh,
				   NCSI_CB(cmd).nsp_portid, timeout);

	if (check && cmd && atomic_dec_return(&ndp->ndp_pending_reqs) == 0)
		schedule_work(&ndp->ndp_work);
	/* Release command and response */
	consume_skb(cmd);
	consume_skb(rsp);
}

struct ncsi_dev *ncsi_find_dev(struct net_device *dev)
{
	struct ncsi_dev_priv *ndp;

	NCSI_FOR_EACH_DEV(ndp) {
		if (ndp->ndp_ndev.nd_dev == dev)
			return &ndp->ndp_ndev;
	}

	return NULL;
}

static int ncsi_select_active_channel(struct ncsi_dev_priv *ndp)
{
	struct ncsi_package *np;
	struct ncsi_channel *nc;

	/* For now, we simply choose the first valid channel as active one.
	 * There might be more factors, like the channel's capacity, can
	 * be considered to pick the active channel in future.
	 */
	NCSI_FOR_EACH_PACKAGE(ndp, np) {
		NCSI_FOR_EACH_CHANNEL(np, nc) {
			ndp->ndp_active_package = np;
			ndp->ndp_active_channel = nc;
			return 0;
		}
	}

	return -ENXIO;
}

static void ncsi_dev_config(struct ncsi_dev_priv *ndp)
{
	struct ncsi_dev *nd = &ndp->ndp_ndev;
	struct net_device *dev = nd->nd_dev;
	struct ncsi_package *np = ndp->ndp_active_package;
	struct ncsi_channel *nc = ndp->ndp_active_channel;
	struct ncsi_cmd_arg nca;
	unsigned char index;
	int ret;

	nca.nca_ndp = ndp;
	nca.nca_nlh = NULL;

	/* When we're reconfiguring the active channel, the active package
	 * should be selected and the old setting on the active channel
	 * should be cleared.
	 */
	switch (nd->nd_state) {
	case ncsi_dev_state_config:
	case ncsi_dev_state_config_sp:
		atomic_set(&ndp->ndp_pending_reqs, 1);

		/* Select the specific package */
		nca.nca_type = NCSI_PKT_CMD_SP;
		nca.nca_bytes[0] = 1;
		nca.nca_package = np->np_id;
		nca.nca_channel = 0x1f;
		ret = ncsi_xmit_cmd(&nca);
		if (ret)
			goto error;

		nd->nd_state = ncsi_dev_state_config_cis;
		break;
	case ncsi_dev_state_config_cis:
		atomic_set(&ndp->ndp_pending_reqs, 1);

		/* Clear initial state */
		nca.nca_type = NCSI_PKT_CMD_CIS;
		nca.nca_package = np->np_id;
		nca.nca_channel = nc->nc_id;
		ret = ncsi_xmit_cmd(&nca);
		if (ret)
			goto error;

		nd->nd_state = ncsi_dev_state_config_sma;
		break;
	case ncsi_dev_state_config_sma:
	case ncsi_dev_state_config_ebf:
	case ncsi_dev_state_config_ecnt:
	case ncsi_dev_state_config_ec:
	case ncsi_dev_state_config_gls:
		atomic_set(&ndp->ndp_pending_reqs, 1);

		nca.nca_package = np->np_id;
		nca.nca_channel = nc->nc_id;

		/* Use first entry in unicast filter table. Note that
		 * the MAC filter table starts from entry 1 instead of
		 * 0.
		 */
		if (nd->nd_state == ncsi_dev_state_config_sma) {
			nca.nca_type = NCSI_PKT_CMD_SMA;
			for (index = 0; index < 6; index++)
				nca.nca_bytes[index] = dev->dev_addr[index];
			nca.nca_bytes[6] = 0x1;
			nca.nca_bytes[7] = 0x1;
			nd->nd_state = ncsi_dev_state_config_ebf;
		} else if (nd->nd_state == ncsi_dev_state_config_ebf) {
			nca.nca_type = NCSI_PKT_CMD_EBF;
			nca.nca_dwords[0] = nc->nc_caps[NCSI_CAP_BC].ncc_cap;
			nd->nd_state = ncsi_dev_state_config_ecnt;
		} else if (nd->nd_state == ncsi_dev_state_config_ecnt) {
			nca.nca_type = NCSI_PKT_CMD_ECNT;
			nd->nd_state = ncsi_dev_state_config_ec;
		} else if (nd->nd_state == ncsi_dev_state_config_ec) {
			nca.nca_type = NCSI_PKT_CMD_EC;
			nd->nd_state = ncsi_dev_state_config_gls;
		} else if (nd->nd_state == ncsi_dev_state_config_gls) {
			nca.nca_type = NCSI_PKT_CMD_GLS;
			nd->nd_state = ncsi_dev_state_config_done;
		}

		ret = ncsi_xmit_cmd(&nca);
		if (ret)
			goto error;

		break;
	case ncsi_dev_state_config_done:
		nd->nd_state = ncsi_dev_state_functional;
		nd->nd_link_up = 0;
		if (nc->nc_modes[NCSI_MODE_LINK].ncm_data[2] & 0x1)
			nd->nd_link_up = 1;

		if (!(ndp->ndp_flags & NCSI_DEV_PRIV_FLAG_CHANGE_ACTIVE))
			nd->nd_handler(nd);
		ndp->ndp_flags &= ~NCSI_DEV_PRIV_FLAG_CHANGE_ACTIVE;

		break;
	default:
		pr_debug("%s: Unrecognized NCSI dev state 0x%x\n",
			 __func__, nd->nd_state);
		return;
	}

	return;

error:
	nd->nd_state = ncsi_dev_state_functional;
	nd->nd_link_up = 0;
	ndp->ndp_flags &= ~NCSI_DEV_PRIV_FLAG_CHANGE_ACTIVE;
	nd->nd_handler(nd);
}

static void ncsi_dev_start(struct ncsi_dev_priv *ndp)
{
	struct ncsi_dev *nd = &ndp->ndp_ndev;
	struct ncsi_package *np;
	struct ncsi_channel *nc;
	struct ncsi_cmd_arg nca;
	unsigned char index;
	int ret;

	nca.nca_ndp = ndp;
	nca.nca_nlh = NULL;
	switch (nd->nd_state) {
	case ncsi_dev_state_start:
		nd->nd_state = ncsi_dev_state_start_deselect;
		/* Fall through */
	case ncsi_dev_state_start_deselect:
		atomic_set(&ndp->ndp_pending_reqs, 8);

		/* Deselect all possible packages */
		nca.nca_type = NCSI_PKT_CMD_DP;
		nca.nca_channel = 0x1f;
		for (index = 0; index < 8; index++) {
			nca.nca_package = index;
			ret = ncsi_xmit_cmd(&nca);
			if (ret)
				goto error;
		}

		nd->nd_state = ncsi_dev_state_start_package;
		break;
	case ncsi_dev_state_start_package:
		atomic_set(&ndp->ndp_pending_reqs, 16);

		/* Select all possible packages */
		nca.nca_type = NCSI_PKT_CMD_SP;
		nca.nca_bytes[0] = 1;
		nca.nca_channel = 0x1f;
		for (index = 0; index < 8; index++) {
			nca.nca_package = index;
			ret = ncsi_xmit_cmd(&nca);
			if (ret)
				goto error;
		}

		/* Disable all possible packages */
		nca.nca_type = NCSI_PKT_CMD_DP;
		for (index = 0; index < 8; index++) {
			nca.nca_package = index;
			ret = ncsi_xmit_cmd(&nca);
			if (ret)
				goto error;
		}

		nd->nd_state = ncsi_dev_state_start_channel;
		break;
	case ncsi_dev_state_start_channel:
		/* The available packages should have been detected. To
		 * iterate every package to probe its channels.
		 */
		if (!ndp->ndp_active_package) {
			ndp->ndp_active_package = list_first_or_null_rcu(
				&ndp->ndp_packages, struct ncsi_package,
				np_node);
			if (!ndp->ndp_active_package)
				goto error;
		} else {
			if (list_is_last(&ndp->ndp_active_package->np_node,
					 &ndp->ndp_packages)) {
				nd->nd_state = ncsi_dev_state_start_active;
				goto choose_active_channel;
			}

			ndp->ndp_active_package = list_entry_rcu(
				ndp->ndp_active_package->np_node.next,
				struct ncsi_package, np_node);
		}
		/* Fall through */
	case ncsi_dev_state_start_sp:
		atomic_set(&ndp->ndp_pending_reqs, 1);

		/* Select the specific package */
		nca.nca_type = NCSI_PKT_CMD_SP;
		nca.nca_bytes[0] = 1;
		nca.nca_package = ndp->ndp_active_package->np_id;
		nca.nca_channel = 0x1f;
		ret = ncsi_xmit_cmd(&nca);
		if (ret)
			goto error;

		nd->nd_state = ncsi_dev_state_start_cis;
		break;
	case ncsi_dev_state_start_cis:
		atomic_set(&ndp->ndp_pending_reqs, 0x20);

		/* Clear initial state */
		nca.nca_type = NCSI_PKT_CMD_CIS;
		nca.nca_package = ndp->ndp_active_package->np_id;
		for (index = 0; index < 0x20; index++) {
			nca.nca_channel = index;
			ret = ncsi_xmit_cmd(&nca);
			if (ret)
				goto error;
		}

		nd->nd_state = ncsi_dev_state_start_gvi;
		break;
	case ncsi_dev_state_start_gvi:
	case ncsi_dev_state_start_gc:
		/* The available channels of the active package should have
		 * been populated.
		 */
		np = ndp->ndp_active_package;
		atomic_set(&ndp->ndp_pending_reqs,
			   atomic_read(&np->np_channel_num));

		/* Get version information or get capacity */
		if (nd->nd_state == ncsi_dev_state_start_gvi)
			nca.nca_type = NCSI_PKT_CMD_GVI;
		else
			nca.nca_type = NCSI_PKT_CMD_GC;

		nca.nca_package = np->np_id;
		NCSI_FOR_EACH_CHANNEL(np, nc) {
			nca.nca_channel = nc->nc_id;
			ret = ncsi_xmit_cmd(&nca);
			if (ret)
				goto error;
		}

		if (nd->nd_state == ncsi_dev_state_start_gvi)
			nd->nd_state = ncsi_dev_state_start_gc;
		else
			nd->nd_state = ncsi_dev_state_start_dp;
		break;
	case ncsi_dev_state_start_dp:
		atomic_set(&ndp->ndp_pending_reqs, 1);

		/* Deselect the active package */
		nca.nca_type = NCSI_PKT_CMD_DP;
		nca.nca_package = ndp->ndp_active_package->np_id;
		nca.nca_channel = 0x1f;
		ret = ncsi_xmit_cmd(&nca);
		if (ret)
			goto error;

		nd->nd_state = ncsi_dev_state_start_channel;
		break;
	case ncsi_dev_state_start_active:
choose_active_channel:
		/* All packages and channels should have been populated. Also,
		 * the information for all channels should have been retrieved.
		 */
		ndp->ndp_active_package = NULL;
		ncsi_select_active_channel(ndp);
		if (!ndp->ndp_active_package ||
		    !ndp->ndp_active_channel)
			goto error;

		/* To configure the active channel */
		nd->nd_state = ncsi_dev_state_config_sma;
		ncsi_dev_config(ndp);
	default:
		pr_debug("%s: Unrecognized NCSI dev state 0x%x\n",
			 __func__, nd->nd_state);
	}

	return;

error:
	ndp->ndp_flags &= ~NCSI_DEV_PRIV_FLAG_CHANGE_ACTIVE;
	nd->nd_state = ncsi_dev_state_functional;
	nd->nd_link_up = 0;
	nd->nd_handler(nd);
}

static void ncsi_dev_stop(struct ncsi_dev_priv *ndp)
{
	struct ncsi_dev *nd = &ndp->ndp_ndev;
	struct ncsi_package *np, *tmp;
	struct ncsi_channel *nc;
	struct ncsi_cmd_arg nca;
	int ret;

	nca.nca_ndp = ndp;
	nca.nca_nlh = NULL;
	switch (nd->nd_state) {
	case ncsi_dev_state_stop:
		/* If there're no active channel, we're done */
		if (!ndp->ndp_active_channel) {
			nd->nd_state = ncsi_dev_state_stop_done;
			goto done;
		}

		nd->nd_state = ncsi_dev_state_stop_select;
		/* Fall through */
	case ncsi_dev_state_stop_select:
	case ncsi_dev_state_stop_dcnt:
	case ncsi_dev_state_stop_dc:
	case ncsi_dev_state_stop_deselect:
		atomic_set(&ndp->ndp_pending_reqs, 1);

		np = ndp->ndp_active_package;
		nc = ndp->ndp_active_channel;
		nca.nca_package = np->np_id;
		if (nd->nd_state == ncsi_dev_state_stop_select) {
			nca.nca_type = NCSI_PKT_CMD_SP;
			nca.nca_channel = 0x1f;
			nca.nca_bytes[0] = 1;
			nd->nd_state = ncsi_dev_state_stop_dcnt;
		} else if (nd->nd_state == ncsi_dev_state_stop_dcnt) {
			nca.nca_type = NCSI_PKT_CMD_DCNT;
			nca.nca_channel = nc->nc_id;
			nd->nd_state = ncsi_dev_state_stop_dc;
		} else if (nd->nd_state == ncsi_dev_state_stop_dc) {
			nca.nca_type = NCSI_PKT_CMD_DC;
			nca.nca_channel = nc->nc_id;
			nca.nca_bytes[0] = 1;
			nd->nd_state = ncsi_dev_state_stop_deselect;
		} else if (nd->nd_state == ncsi_dev_state_stop_deselect) {
			nca.nca_type = NCSI_PKT_CMD_DP;
			nca.nca_channel = 0x1f;
			nd->nd_state = ncsi_dev_state_stop_done;
		}

		ret = ncsi_xmit_cmd(&nca);
		if (ret) {
			nd->nd_state = ncsi_dev_state_stop_done;
			goto done;
		}

		break;
	case ncsi_dev_state_stop_done:
done:
		spin_lock(&ndp->ndp_package_lock);
		list_for_each_entry_safe(np, tmp, &ndp->ndp_packages, np_node)
			ncsi_release_package(np);
		spin_unlock(&ndp->ndp_package_lock);

		if (!(ndp->ndp_flags & NCSI_DEV_PRIV_FLAG_CHANGE_ACTIVE)) {
			nd->nd_state = ncsi_dev_state_functional;
			nd->nd_link_up = 0;
			nd->nd_handler(nd);
		} else {
			nd->nd_state = ncsi_dev_state_start;
			ncsi_dev_start(ndp);
		}

		break;
	default:
		pr_warn("%s: Unsupported NCSI dev state 0x%x\n",
			__func__, nd->nd_state);
	}
}

static void ncsi_dev_work(struct work_struct *work)
{
	struct ncsi_dev_priv *ndp = container_of(work, struct ncsi_dev_priv,
						 ndp_work);
	struct ncsi_dev *nd = &ndp->ndp_ndev;

	switch (nd->nd_state & ncsi_dev_state_major) {
	case ncsi_dev_state_start:
		ncsi_dev_start(ndp);
		break;
	case ncsi_dev_state_stop:
		ncsi_dev_stop(ndp);
		break;
	case ncsi_dev_state_config:
		ncsi_dev_config(ndp);
		break;
	default:
		pr_warn("%s: Unsupported NCSI dev state 0x%x\n",
			__func__, nd->nd_state);
	}
}

static void ncsi_req_timeout(unsigned long data)
{
	struct ncsi_req *nr = (struct ncsi_req *)data;
	struct ncsi_dev_priv *ndp = nr->nr_ndp;

	/* If the request already had associated response,
	 * let the response handler to release it.
	 */
	spin_lock(&ndp->ndp_req_lock);
	nr->nr_timer_enabled = false;
	if (nr->nr_rsp || !nr->nr_cmd) {
		spin_unlock(&ndp->ndp_req_lock);
		return;
	}
	spin_unlock(&ndp->ndp_req_lock);

	/* Release the request */
	ncsi_free_req(nr, true, true);
}

struct ncsi_dev *ncsi_register_dev(struct net_device *dev,
				   void (*handler)(struct ncsi_dev *ndev))
{
	struct ncsi_dev_priv *ndp;
	struct ncsi_dev *nd;
	int idx;

	/* Check if the device has been registered or not */
	nd = ncsi_find_dev(dev);
	if (nd)
		return nd;

	/* Create NCSI device */
	ndp = kzalloc(sizeof(*ndp), GFP_ATOMIC);
	if (!ndp) {
		pr_warn("%s: Out of memory !\n", __func__);
		return NULL;
	}

	nd = &ndp->ndp_ndev;
	nd->nd_state = ncsi_dev_state_registered;
	nd->nd_dev = dev;
	nd->nd_handler = handler;

	/* Initialize private NCSI device */
	spin_lock_init(&ndp->ndp_package_lock);
	INIT_LIST_HEAD(&ndp->ndp_packages);
	INIT_WORK(&ndp->ndp_work, ncsi_dev_work);
	spin_lock_init(&ndp->ndp_req_lock);
	atomic_set(&ndp->ndp_last_req_idx, 0);
	for (idx = 0; idx < 256; idx++) {
		ndp->ndp_reqs[idx].nr_id = idx;
		ndp->ndp_reqs[idx].nr_ndp = ndp;
		setup_timer(&ndp->ndp_reqs[idx].nr_timer, ncsi_req_timeout,
			    (unsigned long)&ndp->ndp_reqs[idx]);
	}

	spin_lock(&ncsi_dev_lock);
	list_add_tail_rcu(&ndp->ndp_node, &ncsi_dev_list);
	spin_unlock(&ncsi_dev_lock);

	/* Register NCSI packet receiption handler */
	ndp->ndp_ptype.type = cpu_to_be16(ETH_P_NCSI);
	ndp->ndp_ptype.func = ncsi_rcv_rsp;
	ndp->ndp_ptype.dev = dev;
	dev_add_pack(&ndp->ndp_ptype);

	return nd;
}
EXPORT_SYMBOL_GPL(ncsi_register_dev);

int ncsi_start_dev(struct ncsi_dev *nd)
{
	struct ncsi_dev_priv *ndp = TO_NCSI_DEV_PRIV(nd);

	if (nd->nd_state != ncsi_dev_state_registered &&
	    nd->nd_state != ncsi_dev_state_functional)
		return -ENOTTY;

	nd->nd_state = ncsi_dev_state_start;
	schedule_work(&ndp->ndp_work);

	return 0;
}
EXPORT_SYMBOL_GPL(ncsi_start_dev);

int ncsi_config_dev(struct ncsi_dev *nd)
{
	struct ncsi_dev_priv *ndp = TO_NCSI_DEV_PRIV(nd);

	if (nd->nd_state != ncsi_dev_state_functional)
		return -ENOTTY;

	nd->nd_state = ncsi_dev_state_config;
	schedule_work(&ndp->ndp_work);

	return 0;
}

int ncsi_stop_dev(struct ncsi_dev *nd)
{
	struct ncsi_dev_priv *ndp = TO_NCSI_DEV_PRIV(nd);

	if (nd->nd_state != ncsi_dev_state_functional)
		return -ENOTTY;

	nd->nd_state = ncsi_dev_state_stop;
	schedule_work(&ndp->ndp_work);

	return 0;
}
EXPORT_SYMBOL_GPL(ncsi_stop_dev);

void ncsi_unregister_dev(struct ncsi_dev *nd)
{
	struct ncsi_dev_priv *ndp = TO_NCSI_DEV_PRIV(nd);
	struct ncsi_package *np, *tmp;

	dev_remove_pack(&ndp->ndp_ptype);

	spin_lock(&ndp->ndp_package_lock);
	list_for_each_entry_safe(np, tmp, &ndp->ndp_packages, np_node)
		ncsi_release_package(np);
	spin_unlock(&ndp->ndp_package_lock);
}
EXPORT_SYMBOL_GPL(ncsi_unregister_dev);

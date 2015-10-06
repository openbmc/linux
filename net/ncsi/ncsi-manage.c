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

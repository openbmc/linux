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
#include "ncsi-pkt.h"

static struct sock *ncsi_sock;

static void ncsi_netlink_error(struct nlmsghdr *h,
			       unsigned int portid,
			       int errcode)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	struct ncsi_msg *src, *dst;
	int ret;

	skb = nlmsg_new(sizeof(*dst), GFP_ATOMIC);
	if (!skb)
		return;

	/* The request might not have the common data instance */
	nlh = nlmsg_put(skb, h->nlmsg_pid, h->nlmsg_seq,
			h->nlmsg_type, sizeof(*dst), 0);
	dst = nlmsg_data(nlh);
	if (nlmsg_len(h) >= sizeof(*src)) {
		src = nlmsg_data(h);
		memcpy(dst, src, sizeof(*dst));
		dst->nm_flag &= ~NCSI_FLAG_REQUEST;
		dst->nm_flag |= NCSI_FLAG_RESPONSE;
		dst->nm_errcode = errcode;
	} else {
		memset(dst, 0, sizeof(*dst));
		dst->nm_flag = NCSI_FLAG_RESPONSE;
		dst->nm_errcode = errcode;
	}

	nlmsg_end(skb, nlh);
	ret = nlmsg_notify(ncsi_sock, skb, portid, 0, 1, 0);
	if (ret) {
		pr_warn("%s: Error %d sending message (%d)\n",
			__func__, ret, errcode);
		nlmsg_free(skb);
	}
}

static struct sk_buff *ncsi_netlink_get_layout(struct nlmsghdr *h,
					       int *errcode)
{
	struct net_device *dev;
	struct ncsi_dev *nd;
	struct ncsi_dev_priv *ndp;
	struct ncsi_package *np;
	struct ncsi_channel *nc;
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	struct ncsi_msg nm, *src, *dst;
	size_t size = sizeof(*dst);;

	/* Find the NCSI device */
	src = nlmsg_data(h);
	dev = dev_get_by_index(ncsi_net, src->nm_ifindex);
	nd = dev ? ncsi_find_dev(dev) : NULL;
	ndp = nd ? TO_NCSI_DEV_PRIV(nd) : NULL;
	if (!ndp) {
		*errcode = NCSI_ERR_NO_DEV;
		return NULL;
	}

	/* Allocate response */
	NCSI_FOR_EACH_PACKAGE(ndp, np)
		size += nla_total_size(sizeof(nm)) *
			atomic_read(&np->np_channel_num);
	if (size <= sizeof(nm)) {
		*errcode = NCSI_ERR_NO_DEV;
		return NULL;
	}
	skb = nlmsg_new(size, GFP_KERNEL);
	if (!skb) {
		*errcode = NCSI_ERR_NO_MEM;
		return NULL;
	}

	/* Fill header */
	nlh = nlmsg_put(skb, h->nlmsg_pid, h->nlmsg_seq,
			h->nlmsg_type, sizeof(nm), 0);
	dst = nlmsg_data(nlh);
	memcpy(dst, src, sizeof(*dst));
	dst->nm_flag &= ~NCSI_FLAG_REQUEST;
	dst->nm_flag |= NCSI_FLAG_RESPONSE;

	/* All available channels */
	NCSI_FOR_EACH_PACKAGE(ndp, np) {
		NCSI_FOR_EACH_CHANNEL(np, nc) {
			nm.nm_flag = 0;
			if (nc == ndp->ndp_active_channel)
				nm.nm_flag = NCSI_FLAG_ACTIVE_CHANNEL;
			nm.nm_ifindex = src->nm_ifindex;
			nm.nm_package_id = np->np_id;
			nm.nm_channel_id = nc->nc_id;
			nm.nm_index = 0;
			nm.nm_index = NCSI_SUCCESS;

			if (nla_put(skb, 0, sizeof(nm), &nm)) {
				*errcode = NCSI_ERR_INTERNAL;
				nlmsg_free(skb);
				return NULL;
			}
		}
	}

	nlmsg_end(skb, nlh);
	return skb;
}

static struct sk_buff *ncsi_netlink_get_version(struct nlmsghdr *h,
						int *errcode)
{
	struct net_device *dev;
	struct ncsi_dev *nd;
	struct ncsi_dev_priv *ndp;
	struct ncsi_package *np;
	struct ncsi_channel *nc;
	struct ncsi_channel_version *ncv;
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	struct ncsi_msg *src, *dst;
	size_t size = sizeof(*dst);;

	/* Find the NCSI device */
	src = nlmsg_data(h);
	dev = dev_get_by_index(ncsi_net, src->nm_ifindex);
	nd = dev ? ncsi_find_dev(dev) : NULL;
	ndp = nd ? TO_NCSI_DEV_PRIV(nd) : NULL;
	np = ndp ? ncsi_find_package(ndp, src->nm_package_id) : NULL;
	nc = np ? ncsi_find_channel(np, src->nm_channel_id) : NULL;
	if (!nc) {
		*errcode = NCSI_ERR_NO_DEV;
		return NULL;
	}

	/* Allocate response */
	size += nla_total_size(sizeof(*ncv));
	skb = nlmsg_new(size, GFP_KERNEL);
	if (!skb) {
		*errcode = NCSI_ERR_NO_MEM;
		return NULL;
	}

	/* Fill header */
	nlh = nlmsg_put(skb, h->nlmsg_pid, h->nlmsg_seq,
			h->nlmsg_type, sizeof(*dst), 0);
	dst = nlmsg_data(nlh);
	memcpy(dst, src, sizeof(*dst));
	dst->nm_flag &= ~NCSI_FLAG_REQUEST;
	dst->nm_flag |= NCSI_FLAG_RESPONSE;

	/* Fill channel version */
	ncv = &nc->nc_version;
	if (nla_put(skb, 0, sizeof(*ncv), ncv)) {
		*errcode = NCSI_ERR_INTERNAL;
		nlmsg_free(skb);
		return NULL;
	}

	nlmsg_end(skb, nlh);
	return skb;
}

static struct sk_buff *ncsi_netlink_get_cap(struct nlmsghdr *h,
					    int *errcode)
{
	struct net_device *dev;
	struct ncsi_dev *nd;
	struct ncsi_dev_priv *ndp;
	struct ncsi_package *np;
	struct ncsi_channel *nc;
	struct ncsi_channel_cap *ncc;
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	struct ncsi_msg *src, *dst;
	size_t size = sizeof(*dst);
	int index;

	/* Find the NCSI device */
	src = nlmsg_data(h);
	dev = dev_get_by_index(ncsi_net, src->nm_ifindex);
	nd = dev ? ncsi_find_dev(dev) : NULL;
	ndp = nd ? TO_NCSI_DEV_PRIV(nd) : NULL;
	np = ndp ? ncsi_find_package(ndp, src->nm_package_id) : NULL;
	nc = np ? ncsi_find_channel(np, src->nm_channel_id) : NULL;
	if (!nc) {
		*errcode = NCSI_ERR_NO_DEV;
		return NULL;
	}

	/* Allocate response */
	if (src->nm_index > NCSI_CAP_MAX) {
		*errcode = NCSI_ERR_PARAM;
		return NULL;
	} else if (src->nm_index == NCSI_CAP_MAX) {
		size += nla_total_size(sizeof(*ncc)) * NCSI_CAP_MAX;
	} else {
		size += nla_total_size(sizeof(*ncc));
	}
	skb = nlmsg_new(size, GFP_KERNEL);
	if (!skb) {
		*errcode = NCSI_ERR_NO_MEM;
		return NULL;
	}

	/* Fill header */
	nlh = nlmsg_put(skb, h->nlmsg_pid, h->nlmsg_seq,
			h->nlmsg_type, sizeof(*dst), 0);
	dst = nlmsg_data(nlh);
	memcpy(dst, src, sizeof(*dst));
	dst->nm_flag &= ~NCSI_FLAG_REQUEST;
	dst->nm_flag |= NCSI_FLAG_RESPONSE;

	/* Fill one or all capabilities */
	for (index = 0; index < NCSI_CAP_MAX; index++) {
		ncc = &nc->nc_caps[index];
		if (src->nm_index == NCSI_CAP_MAX ||
		    src->nm_index == index) {
			if (nla_put(skb, 0, sizeof(*ncc), ncc)) {
				*errcode = NCSI_ERR_INTERNAL;
				nlmsg_free(skb);
				return NULL;
			}
		}
	}

	nlmsg_end(skb, nlh);
	return skb;
}

static struct sk_buff *ncsi_netlink_get_mode(struct nlmsghdr *h,
					     int *errcode)
{
	struct net_device *dev;
	struct ncsi_dev *nd;
	struct ncsi_dev_priv *ndp;
	struct ncsi_package *np;
	struct ncsi_channel *nc;
	struct ncsi_cmd_arg nca;
	struct ncsi_msg *src;

	/* Find the NCSI device */
	src = nlmsg_data(h);
	dev = dev_get_by_index(ncsi_net, src->nm_ifindex);
	nd = dev ? ncsi_find_dev(dev) : NULL;
	ndp = nd ? TO_NCSI_DEV_PRIV(nd) : NULL;
	np = ndp ? ncsi_find_package(ndp, src->nm_package_id) : NULL;
	nc = np ? ncsi_find_channel(np, src->nm_channel_id) : NULL;
	if (!nc) {
		*errcode = NCSI_ERR_NO_DEV;
		return NULL;
	}
	if (nd->nd_state != ncsi_dev_state_functional ||
	    ndp->ndp_active_channel != nc) {
		*errcode = NCSI_ERR_NOT_ACTIVE;
		return NULL;
	}
	if (src->nm_index > NCSI_MODE_MAX) {
		*errcode = NCSI_ERR_PARAM;
		return NULL;
	}

	/* Send NCSI GP command */
	nca.nca_ndp = ndp;
	nca.nca_nlh = h;
	nca.nca_type = NCSI_PKT_CMD_GP;
	nca.nca_package = np->np_id;
	nca.nca_channel = nc->nc_id;
	if (ncsi_xmit_cmd(&nca)) {
		*errcode = NCSI_ERR_INTERNAL;
		return NULL;
	}

	*errcode = NCSI_SUCCESS;
	return NULL;
}

static struct sk_buff *ncsi_netlink_get_filter(struct nlmsghdr *h,
					       int *errcode)
{
	struct net_device *dev;
	struct ncsi_dev *nd;
	struct ncsi_dev_priv *ndp;
	struct ncsi_package *np;
	struct ncsi_channel *nc;
	struct ncsi_cmd_arg nca;
	struct ncsi_msg *src;

	/* Find the NCSI device */
	src = nlmsg_data(h);
	dev = dev_get_by_index(ncsi_net, src->nm_ifindex);
	nd = dev ? ncsi_find_dev(dev) : NULL;
	ndp = nd ? TO_NCSI_DEV_PRIV(nd) : NULL;
	np = ndp ? ncsi_find_package(ndp, src->nm_package_id) : NULL;
	nc = np ? ncsi_find_channel(np, src->nm_channel_id) : NULL;
	if (!nc) {
		*errcode = NCSI_ERR_NO_DEV;
		return NULL;
	}
	if (nd->nd_state != ncsi_dev_state_functional ||
	    ndp->ndp_active_channel != nc) {
		*errcode = NCSI_ERR_NOT_ACTIVE;
		return NULL;
	}
	if (src->nm_index > NCSI_FILTER_MAX) {
		*errcode = NCSI_ERR_PARAM;
		return NULL;
	}

	/* Send NCSI GP command */
	nca.nca_ndp = ndp;
	nca.nca_nlh = h;
	nca.nca_type = NCSI_PKT_CMD_GP;
	nca.nca_package = np->np_id;
	nca.nca_channel = nc->nc_id;
	if (ncsi_xmit_cmd(&nca)) {
		*errcode = NCSI_ERR_INTERNAL;
		return NULL;
	}

	*errcode = NCSI_SUCCESS;
	return NULL;
}

struct sk_buff *ncsi_netlink_get_stats(struct nlmsghdr *h, int *errcode)
{
	struct net_device *dev;
	struct ncsi_dev *nd;
	struct ncsi_dev_priv *ndp;
	struct ncsi_package *np;
	struct ncsi_channel *nc;
	struct ncsi_msg *src;
	struct ncsi_cmd_arg nca;
	unsigned char cmd;

	/* Find the NCSI device */
	src = nlmsg_data(h);
	dev = dev_get_by_index(ncsi_net, src->nm_ifindex);
	nd = dev ? ncsi_find_dev(dev) : NULL;
	ndp = nd ? TO_NCSI_DEV_PRIV(nd) : NULL;
	np = ndp ? ncsi_find_package(ndp, src->nm_package_id) : NULL;
	nc = np ? ncsi_find_channel(np, src->nm_channel_id) : NULL;
	if (!nc) {
		*errcode = NCSI_ERR_NO_DEV;
		return NULL;
	}
	if (nd->nd_state != ncsi_dev_state_functional ||
	    ndp->ndp_active_channel != nc) {
		*errcode = NCSI_ERR_NOT_ACTIVE;
		return NULL;
	}
	if (src->nm_index > NCSI_FILTER_MAX) {
		*errcode = NCSI_ERR_PARAM;
		return NULL;
	}

	/* Get NIC statistics */
	nca.nca_ndp = ndp;
	nca.nca_nlh = NULL;
	nca.nca_type = NCSI_PKT_CMD_GCPS;
	nca.nca_package = np->np_id;
	nca.nca_channel = nc->nc_id;
	for (cmd = NCSI_PKT_CMD_GCPS; cmd <= NCSI_PKT_CMD_GNPTS; cmd++) {
		nca.nca_type = cmd;
		if (cmd == NCSI_PKT_CMD_GNPTS)
			nca.nca_nlh = h;
		if (ncsi_xmit_cmd(&nca)) {
			*errcode = NCSI_ERR_INTERNAL;
			return NULL;
		}
	}

	*errcode = NCSI_SUCCESS;
	return NULL;
}

struct sk_buff *ncsi_netlink_set_mode(struct nlmsghdr *h, int *errcode)
{
	struct net_device *dev;
	struct ncsi_dev *nd;
	struct ncsi_dev_priv *ndp;
	struct ncsi_package *np;
	struct ncsi_channel *nc;
	struct ncsi_channel_mode *ncm;
	struct ncsi_msg *src;
	struct nlattr *nla;
	struct ncsi_cmd_arg nca;

	/* Find the NCSI device */
	src = nlmsg_data(h);
	dev = dev_get_by_index(ncsi_net, src->nm_ifindex);
	nd = dev ? ncsi_find_dev(dev) : NULL;
	ndp = nd ? TO_NCSI_DEV_PRIV(nd) : NULL;
	np = ndp ? ncsi_find_package(ndp, src->nm_package_id) : NULL;
	nc = np ? ncsi_find_channel(np, src->nm_channel_id) : NULL;
	if (!nc) {
		*errcode = NCSI_ERR_NO_DEV;
		return NULL;
	}
	if (nd->nd_state != ncsi_dev_state_functional ||
	    ndp->ndp_active_channel != nc) {
		*errcode = NCSI_ERR_NOT_ACTIVE;
		return NULL;
	}

	/* Get NIC statistics */
	nla = nlmsg_attrdata(h, sizeof(*src));
	ncm = nla_data(nla);

	nca.nca_ndp = ndp;
	nca.nca_nlh = h;
	nca.nca_package = np->np_id;
	nca.nca_channel = nc->nc_id;
	switch (src->nm_index) {
	case NCSI_MODE_ENABLE:
		if (ncm->ncm_enable) {
			nca.nca_type = NCSI_PKT_CMD_EC;
		} else {
			nca.nca_type = NCSI_PKT_CMD_DC;
			nca.nca_bytes[0] = ncm->ncm_data[0];
		}
		break;
	case NCSI_MODE_TX_ENABLE:
		if (ncm->ncm_enable)
			nca.nca_type = NCSI_PKT_CMD_ECNT;
		else
			nca.nca_type = NCSI_PKT_CMD_DCNT;
		break;
	case NCSI_MODE_LINK:
		nca.nca_type = NCSI_PKT_CMD_SL;
		nca.nca_dwords[0] = ncm->ncm_data[0];
		nca.nca_dwords[1] = ncm->ncm_data[1];
		break;
	case NCSI_MODE_VLAN:
		if (ncm->ncm_enable) {
			nca.nca_type = NCSI_PKT_CMD_EV;
			nca.nca_bytes[0] = ncm->ncm_data[0];
		} else {
			nca.nca_type = NCSI_PKT_CMD_DV;
		}
		break;
	case NCSI_MODE_BC:
		if (ncm->ncm_enable) {
			nca.nca_type = NCSI_PKT_CMD_EBF;
			nca.nca_dwords[0] = ncm->ncm_data[0];
		} else {
			nca.nca_type = NCSI_PKT_CMD_DBF;
		}
		break;
	case NCSI_MODE_MC:
		if (ncm->ncm_enable) {
			nca.nca_type = NCSI_PKT_CMD_EGMF;
			nca.nca_dwords[0] = ncm->ncm_data[0];
		} else {
			nca.nca_type = NCSI_PKT_CMD_DGMF;
		}
		break;
	case NCSI_MODE_AEN:
		nca.nca_type = NCSI_PKT_CMD_AE;
		nca.nca_bytes[0] = ncm->ncm_data[0];
		nca.nca_dwords[1] = ncm->ncm_data[1];
		break;
	case NCSI_MODE_FC:
		nca.nca_type = NCSI_PKT_CMD_SNFC;
		nca.nca_bytes[0] = ncm->ncm_data[0];
		break;
	default:
		*errcode = NCSI_ERR_PARAM;
		return NULL;
	}

	if (ncsi_xmit_cmd(&nca)) {
		*errcode = NCSI_ERR_INTERNAL;
		return NULL;
	}

	*errcode = NCSI_SUCCESS;
	return NULL;
}

struct sk_buff *ncsi_netlink_set_filter(struct nlmsghdr *h, int *errcode)
{
	struct net_device *dev;
	struct ncsi_dev *nd;
	struct ncsi_dev_priv *ndp;
	struct ncsi_package *np;
	struct ncsi_channel *nc;
	struct ncsi_channel_filter *ncf;
	struct ncsi_msg *src;
	struct nlattr *nla;
	struct ncsi_cmd_arg nca;

	/* Find the NCSI device */
	src = nlmsg_data(h);
	dev = dev_get_by_index(ncsi_net, src->nm_ifindex);
	nd = dev ? ncsi_find_dev(dev) : NULL;
	ndp = nd ? TO_NCSI_DEV_PRIV(nd) : NULL;
	np = ndp ? ncsi_find_package(ndp, src->nm_package_id) : NULL;
	nc = np ? ncsi_find_channel(np, src->nm_channel_id) : NULL;
	if (!nc) {
		*errcode = NCSI_ERR_NO_DEV;
		return NULL;
	}
	if (nd->nd_state != ncsi_dev_state_functional ||
	    ndp->ndp_active_channel != nc) {
		*errcode = NCSI_ERR_NOT_ACTIVE;
		return NULL;
	}

	/* Get NIC statistics */
	nla = nlmsg_attrdata(h, sizeof(*src));
	ncf = nla_data(nla);

	nca.nca_ndp = ndp;
	nca.nca_nlh = h;
	nca.nca_package = np->np_id;
	nca.nca_channel = nc->nc_id;
	switch (src->nm_index) {
	case NCSI_FILTER_VLAN:
		nca.nca_type = NCSI_PKT_CMD_SVF;
		memcpy(nca.nca_bytes, ncf->ncf_data, 4);
		break;
	case NCSI_FILTER_UC:
	case NCSI_FILTER_MC:
		memcpy(nca.nca_bytes, ncf->ncf_data, 8);
		nca.nca_bytes[7] &= 0x1f;
		if (src->nm_index == NCSI_FILTER_MC)
			nca.nca_bytes[7] |= 0x20;
		break;
	default:
		*errcode = NCSI_ERR_PARAM;
		return NULL;
	}

	if (ncsi_xmit_cmd(&nca)) {
		*errcode = NCSI_ERR_INTERNAL;
		return NULL;
	}

	*errcode = NCSI_SUCCESS;
	return NULL;
}

static struct sk_buff *ncsi_netlink_get_mode_reply(struct nlmsghdr *h,
						   int *errcode)
{
	struct net_device *dev;
	struct ncsi_dev *nd;
	struct ncsi_dev_priv *ndp;
	struct ncsi_package *np;
	struct ncsi_channel *nc;
	struct ncsi_channel_mode *ncm;
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	struct ncsi_msg *src, *dst;
	size_t size = sizeof(*dst);
	int index;

	/* Find the NCSI device */
	src = nlmsg_data(h);
	dev = dev_get_by_index(ncsi_net, src->nm_ifindex);
	nd = dev ? ncsi_find_dev(dev) : NULL;
	ndp = nd ? TO_NCSI_DEV_PRIV(nd) : NULL;
	np = ndp ? ncsi_find_package(ndp, src->nm_package_id) : NULL;
	nc = np ? ncsi_find_channel(np, src->nm_channel_id) : NULL;
	if (!nc) {
		*errcode = NCSI_ERR_NO_DEV;
		return NULL;
	}

	/* Allocate response */
	if (src->nm_index > NCSI_MODE_MAX) {
		*errcode = NCSI_ERR_PARAM;
		return NULL;
	} else if (src->nm_index == NCSI_MODE_MAX) {
		size += nla_total_size(sizeof(*ncm)) * NCSI_MODE_MAX;
	} else {
		size += nla_total_size(sizeof(*ncm));
	}
	skb = nlmsg_new(size, GFP_ATOMIC);
	if (!skb) {
		*errcode = NCSI_ERR_NO_MEM;
		return NULL;
	}

	/* Fill header */
	nlh = nlmsg_put(skb, h->nlmsg_pid, h->nlmsg_seq,
			h->nlmsg_type, sizeof(*dst), 0);
	dst = nlmsg_data(nlh);
	memcpy(dst, src, sizeof(*dst));
	dst->nm_flag &= ~NCSI_FLAG_REQUEST;
	dst->nm_flag |= NCSI_FLAG_RESPONSE;

	/* Fill one or all modes */
	for (index = 0; index < NCSI_MODE_MAX; index++) {
		ncm = &nc->nc_modes[index];
		if (src->nm_index == NCSI_MODE_MAX ||
		    src->nm_index == index) {
			if (nla_put(skb, 0, sizeof(*ncm), ncm)) {
				*errcode = NCSI_ERR_INTERNAL;
				nlmsg_free(skb);
				return NULL;
			}
		}
	}

	nlmsg_end(skb, nlh);
	return skb;
}

static struct sk_buff *ncsi_netlink_get_filter_reply(struct nlmsghdr *h,
						     int *errcode)
{
	struct net_device *dev;
	struct ncsi_dev *nd;
	struct ncsi_dev_priv *ndp;
	struct ncsi_package *np;
	struct ncsi_channel *nc;
	struct ncsi_channel_filter *ncf;
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	struct ncsi_msg *src, *dst;
	size_t entry_size, size = sizeof(*dst);
	int index;

	/* Find the NCSI device */
	src = nlmsg_data(h);
	dev = dev_get_by_index(ncsi_net, src->nm_ifindex);
	nd = dev ? ncsi_find_dev(dev) : NULL;
	ndp = nd ? TO_NCSI_DEV_PRIV(nd) : NULL;
	np = ndp ? ncsi_find_package(ndp, src->nm_package_id) : NULL;
	nc = np ? ncsi_find_channel(np, src->nm_channel_id) : NULL;
	if (!nc) {
		*errcode = NCSI_ERR_NO_DEV;
		return NULL;
	}

	/* Allocate response */
	if (src->nm_index > NCSI_FILTER_MAX) {
		*errcode = NCSI_ERR_PARAM;
		return NULL;
	}

	for (index = 0; index < NCSI_FILTER_MAX; index++) {
		ncf = nc->nc_filters[index];
		switch (index) {
		case NCSI_FILTER_VLAN:
			entry_size = 2;
			break;
		case NCSI_FILTER_UC:
		case NCSI_FILTER_MC:
		case NCSI_FILTER_MIXED:
			entry_size = 6;
			break;
		default:
			continue;
		}

		if (src->nm_index == NCSI_FILTER_MAX ||
		    src->nm_index == index) {
			if (!ncf)
				continue;

			size += nla_total_size(sizeof(*ncf) +
					       entry_size * ncf->ncf_total);
		}
	}

	skb = nlmsg_new(size, GFP_ATOMIC);
	if (!skb) {
		*errcode = NCSI_ERR_NO_MEM;
		return NULL;
	}

	/* Fill header */
	nlh = nlmsg_put(skb, h->nlmsg_pid, h->nlmsg_seq,
			h->nlmsg_type, sizeof(*dst), 0);
	dst = nlmsg_data(nlh);
	memcpy(dst, src, sizeof(*dst));
	dst->nm_flag &= ~NCSI_FLAG_REQUEST;
	dst->nm_flag |= NCSI_FLAG_RESPONSE;

	/* Fill one or all filters */
	for (index = 0; index < NCSI_FILTER_MAX; index++) {
		ncf = nc->nc_filters[index];
		switch (index) {
		case NCSI_FILTER_VLAN:
			entry_size = 2;
			break;
		case NCSI_FILTER_UC:
		case NCSI_FILTER_MC:
		case NCSI_FILTER_MIXED:
			entry_size = 6;
			break;
		default:
			continue;
		}

		if (src->nm_index == NCSI_FILTER_MAX ||
		    src->nm_index == index) {
			if (!ncf)
				continue;

			size = sizeof(*ncf) + entry_size * ncf->ncf_total;
			if (nla_put(skb, 0, size, ncf)) {
				nlmsg_free(skb);
				*errcode = NCSI_ERR_INTERNAL;
				return NULL;
			}
		}
	}

	nlmsg_end(skb, nlh);
	return skb;
}

struct sk_buff *ncsi_netlink_get_stats_reply(struct nlmsghdr *h, int *errcode)
{
	struct net_device *dev;
	struct ncsi_dev *nd;
	struct ncsi_dev_priv *ndp;
	struct ncsi_package *np;
	struct ncsi_channel *nc;
	struct ncsi_channel_stats *ncs;
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	struct ncsi_msg *src, *dst;
	size_t size = sizeof(*dst);

	/* Find the NCSI device */
	src = nlmsg_data(h);
	dev = dev_get_by_index(ncsi_net, src->nm_ifindex);
	nd = dev ? ncsi_find_dev(dev) : NULL;
	ndp = nd ? TO_NCSI_DEV_PRIV(nd) : NULL;
	np = ndp ? ncsi_find_package(ndp, src->nm_package_id) : NULL;
	nc = np ? ncsi_find_channel(np, src->nm_channel_id) : NULL;
	if (!nc) {
		*errcode = NCSI_ERR_NO_DEV;
		return NULL;
	}

	/* Allocate response */
	size += nla_total_size(sizeof(*ncs));
	skb = nlmsg_new(size, GFP_ATOMIC);
	if (!skb) {
		*errcode = NCSI_ERR_NO_MEM;
		return NULL;
	}

	/* Fill header */
	nlh = nlmsg_put(skb, h->nlmsg_pid, h->nlmsg_seq,
			h->nlmsg_type, sizeof(*dst), 0);
	dst = nlmsg_data(nlh);
	memcpy(dst, src, sizeof(*dst));
	dst->nm_flag &= ~NCSI_FLAG_REQUEST;
	dst->nm_flag |= NCSI_FLAG_RESPONSE;

	/* Fill one or all filters */
	ncs = &nc->nc_stats;
	if (nla_put(skb, 0, sizeof(*ncs), ncs)) {
		nlmsg_free(skb);
		*errcode = NCSI_ERR_INTERNAL;
		return NULL;
	}

	nlmsg_end(skb, nlh);
	return skb;
}

struct sk_buff *ncsi_netlink_set_mode_reply(struct nlmsghdr *h, int *errcode)
{
	struct net_device *dev;
	struct ncsi_dev *nd;
	struct ncsi_dev_priv *ndp;
	struct ncsi_package *np;
	struct ncsi_channel *nc;
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	struct nlattr *nla;
	struct ncsi_msg *src, *dst;
	struct ncsi_channel_mode *ncm;
	size_t size = sizeof(*dst);

	/* Find the NCSI device */
	src = nlmsg_data(h);
	dev = dev_get_by_index(ncsi_net, src->nm_ifindex);
	nd = dev ? ncsi_find_dev(dev) : NULL;
	ndp = nd ? TO_NCSI_DEV_PRIV(nd) : NULL;
	np = ndp ? ncsi_find_package(ndp, src->nm_package_id) : NULL;
	nc = np ? ncsi_find_channel(np, src->nm_channel_id) : NULL;
	if (!nc) {
		*errcode = NCSI_ERR_NO_DEV;
		return NULL;
	}

	/* Allocate response */
	size += nla_total_size(sizeof(*ncm));
	skb = nlmsg_new(size, GFP_ATOMIC);
	if (!skb) {
		*errcode = NCSI_ERR_NO_MEM;
		return NULL;
	}

	/* Fill header */
	nlh = nlmsg_put(skb, h->nlmsg_pid, h->nlmsg_seq,
			h->nlmsg_type, sizeof(*dst), 0);
	dst = nlmsg_data(nlh);
	memcpy(dst, src, sizeof(*dst));
	dst->nm_flag &= ~NCSI_FLAG_REQUEST;
	dst->nm_flag |= NCSI_FLAG_RESPONSE;

	/* Fill one or all filters */
	nla = nlmsg_attrdata(h, sizeof(*dst));
	ncm = nla_data(nla);
	if (nla_put(skb, 0, sizeof(*ncm), ncm)) {
		*errcode = NCSI_ERR_INTERNAL;
		return NULL;
	}

	nlmsg_end(skb, nlh);
	return skb;
}

struct sk_buff *ncsi_netlink_set_filter_reply(struct nlmsghdr *h, int *errcode)
{
	struct net_device *dev;
	struct ncsi_dev *nd;
	struct ncsi_dev_priv *ndp;
	struct ncsi_package *np;
	struct ncsi_channel *nc;
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	struct nlattr *nla;
	struct ncsi_msg *src, *dst;
	struct ncsi_channel_filter *ncf;
	size_t extra, size = sizeof(*dst);

	/* Find the NCSI device */
	src = nlmsg_data(h);
	dev = dev_get_by_index(ncsi_net, src->nm_ifindex);
	nd = dev ? ncsi_find_dev(dev) : NULL;
	ndp = nd ? TO_NCSI_DEV_PRIV(nd) : NULL;
	np = ndp ? ncsi_find_package(ndp, src->nm_package_id) : NULL;
	nc = np ? ncsi_find_channel(np, src->nm_channel_id) : NULL;
	if (!nc) {
		*errcode = NCSI_ERR_NO_DEV;
		return NULL;
	}

	/* Allocate response */
	size += nla_total_size(sizeof(*ncf) + 16);
	skb = nlmsg_new(size, GFP_ATOMIC);
	if (!skb) {
		*errcode = NCSI_ERR_NO_MEM;
		return NULL;
	}

	/* Fill header */
	nlh = nlmsg_put(skb, h->nlmsg_pid, h->nlmsg_seq,
			h->nlmsg_type, sizeof(*dst), 0);
	dst = nlmsg_data(nlh);
	memcpy(dst, src, sizeof(*dst));
	dst->nm_flag &= ~NCSI_FLAG_REQUEST;
	dst->nm_flag |= NCSI_FLAG_RESPONSE;

	/* Fill one or all filters */
	nla = nlmsg_attrdata(h, sizeof(*dst));
	ncf = nla_data(nla);
	switch(src->nm_index) {
	case NCSI_FILTER_VLAN:
		extra = 2;
		break;
	case NCSI_FILTER_UC:
	case NCSI_FILTER_MC:
		extra = 6;
		break;
	default:
		nlmsg_free(skb);
		*errcode = NCSI_ERR_PARAM;
		return NULL;
	}

	if (nla_put(skb, 0, sizeof(*ncf) + extra, ncf)) {
		*errcode = NCSI_ERR_INTERNAL;
		return NULL;
	}

	nlmsg_end(skb, nlh);
	return skb;
}

void ncsi_netlink_reply(struct nlmsghdr *h, unsigned int portid, bool timeout)
{
	struct sk_buff *skb;
	int errcode = NCSI_SUCCESS;
	struct sk_buff *(*func)(struct nlmsghdr *, int *) = NULL;

	if (timeout) {
		ncsi_netlink_error(h, portid, NCSI_ERR_INTERNAL);
		return;
	}

	switch (h->nlmsg_type) {
	case NCSI_MSG_GET_MODE:
		func = ncsi_netlink_get_mode_reply;
		break;
	case NCSI_MSG_GET_FILTER:
		func = ncsi_netlink_get_filter_reply;
		break;
	case NCSI_MSG_GET_STATS:
		func = ncsi_netlink_get_stats_reply;
		break;
	case NCSI_MSG_SET_MODE:
		func = ncsi_netlink_set_mode_reply;
		break;
	case NCSI_MSG_SET_FILTER:
		func = ncsi_netlink_set_filter_reply;
		break;
	default:
		errcode = NCSI_ERR_PARAM;
		goto out;
	}

	skb = func(h, &errcode);
	if (!skb)
		goto out;

	nlmsg_notify(ncsi_sock, skb, portid, 0, 0, GFP_ATOMIC);
	return;
out:
	ncsi_netlink_error(h, portid, errcode);
}

static int ncsi_netlink_rcv_msg(struct sk_buff *cmd, struct nlmsghdr *h)
{
	struct ncsi_msg *nm;
	struct sk_buff *skb;
	int errcode = NCSI_SUCCESS;
	unsigned int portid = NETLINK_CB(cmd).portid;
	struct sk_buff *(*func)(struct nlmsghdr *, int *) = NULL;

	if (h->nlmsg_type >= NCSI_MSG_MAX ||
	    nlmsg_len(h) < sizeof(*nm)) {
		errcode = NCSI_ERR_PARAM;
		goto out;
	}

	nm = nlmsg_data(h);
	if (!(nm->nm_flag & NCSI_FLAG_REQUEST)) {
		errcode = NCSI_ERR_PARAM;
		goto out;
	}

	switch (h->nlmsg_type) {
	case NCSI_MSG_GET_LAYOUT:
		func = ncsi_netlink_get_layout;
		break;
	case NCSI_MSG_GET_VERSION:
		func = ncsi_netlink_get_version;
		break;
	case NCSI_MSG_GET_CAP:
		func = ncsi_netlink_get_cap;
		break;
	case NCSI_MSG_GET_MODE:
		func = ncsi_netlink_get_mode;
		break;
	case NCSI_MSG_GET_FILTER:
		func = ncsi_netlink_get_filter;
		break;
	case NCSI_MSG_GET_STATS:
		func = ncsi_netlink_get_stats;
		break;
	case NCSI_MSG_SET_MODE:
		func = ncsi_netlink_set_mode;
		break;
	case NCSI_MSG_SET_FILTER:
		func = ncsi_netlink_set_filter;
		break;
	default:
		goto out;
	}

	skb = func(h, &errcode);
	if (errcode != NCSI_SUCCESS)
		goto out;

	/* Split transactions */
	if (!skb)
		return 0;

	return nlmsg_notify(ncsi_sock, skb, portid, 0, 0, GFP_KERNEL);
out:
	ncsi_netlink_error(h, portid, errcode);
	return -EINTR;
}

static void ncsi_netlink_rcv(struct sk_buff *skb)
{
	netlink_rcv_skb(skb, &ncsi_netlink_rcv_msg);
}

int __net_init ncsi_netlink_init(struct net *net)
{
	struct netlink_kernel_cfg cfg = {
		.groups   = 0,
		.flags    = NL_CFG_F_NONROOT_RECV |
			    NL_CFG_F_NONROOT_SEND,
		.input    = ncsi_netlink_rcv,
	};

	ncsi_sock = netlink_kernel_create(net, NETLINK_NCSI, &cfg);
	return ncsi_sock ? 0 : -ENOMEM;
}

void __net_exit ncsi_netlink_exit(struct net *net)
{
	netlink_kernel_release(ncsi_sock);
	ncsi_sock = NULL;
}

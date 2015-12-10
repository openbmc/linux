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

struct net *ncsi_net = NULL;

static int __net_init ncsi_net_init(struct net *net)
{
	ncsi_net = net;
	return ncsi_netlink_init(net);
}

static void __net_exit ncsi_net_exit(struct net *net)
{
	ncsi_netlink_exit(net);
	ncsi_net = NULL;
}

static struct pernet_operations ncsi_net_ops = {
	.init = ncsi_net_init,
	.exit = ncsi_net_exit,
};

static int __init ncsi_init(void)
{
	if (ncsi_net)
		return -EEXIST;

	return register_pernet_subsys(&ncsi_net_ops);
}

fs_initcall_sync(ncsi_init);

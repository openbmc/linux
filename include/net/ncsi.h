#ifndef __NET_NCSI_H
#define __NET_NCSI_H

#include <uapi/linux/ncsi.h>

enum {
	ncsi_dev_state_registered	= 0x0000,
	ncsi_dev_state_functional	= 0x0100,
	ncsi_dev_state_start		= 0x0200,
	ncsi_dev_state_config		= 0x0300,
	ncsi_dev_state_stop		= 0x0400
};

struct ncsi_dev {
	int			nd_state;
	int			nd_link_up;
	struct net_device	*nd_dev;
	void			(*nd_handler)(struct ncsi_dev *ndev);
};

#endif /* __NET_NCSI_H */

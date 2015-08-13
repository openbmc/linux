#ifndef __NET_NCSI_H
#define __NET_NCSI_H

#include <uapi/linux/ncsi.h>

enum {
	ncsi_dev_state_registered	= 0x0000,
	ncsi_dev_state_functional	= 0x0100,
	ncsi_dev_state_init		= 0x0200,
	ncsi_dev_state_stop		= 0x0300
};

struct ncsi_dev {
	int			nd_state;
	int			nd_link_up;
	struct net_device	*nd_dev;
	void			(*nd_handler)(struct ncsi_dev *ndev);
};

struct ncsi_dev *ncsi_register_dev(struct net_device *dev,
				   void (*notifier)(struct ncsi_dev *ndev));
struct ncsi_dev *ncsi_find_dev(struct net_device *dev);
int ncsi_start_dev(struct ncsi_dev *nd);
int ncsi_stop_dev(struct ncsi_dev *nd);
void ncsi_unregister_dev(struct ncsi_dev *nd);

#endif /* __NET_NCSI_H */

#ifndef __NET_NCSI_H
#define __NET_NCSI_H

#include <uapi/linux/ncsi.h>

/*
 * The NCSI device states seen from external. More NCSI device states are
 * only visible internally (in net/ncsi/internal.h). When the NCSI device
 * is registered, it's in ncsi_dev_state_registered state. The state
 * ncsi_dev_state_start is used to drive to choose active package and
 * channel. After that, its state is changed to ncsi_dev_state_functional.
 *
 * The state ncsi_dev_state_stop helps to shut down the currently active
 * package and channel while ncsi_dev_state_config helps to reconfigure
 * them.
 */
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

#ifdef CONFIG_NET_NCSI
struct ncsi_dev *ncsi_register_dev(struct net_device *dev,
				   void (*notifier)(struct ncsi_dev *nd));
int ncsi_start_dev(struct ncsi_dev *nd);
int ncsi_stop_dev(struct ncsi_dev *nd);
void ncsi_unregister_dev(struct ncsi_dev *nd);
#else /* !CONFIG_NET_NCSI */
static inline struct ncsi_dev *ncsi_register_dev(struct net_device *dev,
						 void (*notifier)(struct ncsi_dev *nd))
{
	return NULL;
}

static inline int ncsi_start_dev(struct ncsi_dev *nd)
{
	return -ENOTTY;
}

static inline int ncsi_stop_dev(struct ncsi_dev *nd)
{
	return -ENOTTY;
}

void inline ncsi_unregister_dev(struct ncsi_dev *nd)
{
}
#endif /* CONFIG_NET_NCSI */
#endif /* __NET_NCSI_H */

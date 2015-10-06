#ifndef __NCSI_INTERNAL_H__
#define __NCSI_INTERNAL_H__

struct ncsi_dev_priv;
struct ncsi_package;

#define NCSI_PACKAGE_INDEX(c)	(((c) >> 5) & 0x7)
#define NCSI_CHANNEL_INDEX(c)	((c) & 0x1ffff)
#define NCSI_TO_CHANNEL(p, c)	((((p) & 0x7) << 5) | ((c) & 0x1ffff))

/* Channel state */
enum {
	ncsi_channel_state_deselected_initial,
	ncsi_channel_state_selected_initial,
	ncsi_channel_state_deselected_ready,
	ncsi_channel_state_selected_ready,
};

struct ncsi_channel {
	unsigned char			nc_id;
	int				nc_state;
	struct ncsi_package		*nc_package;
	struct ncsi_channel_version	nc_version;
	struct ncsi_channel_cap		nc_caps[NCSI_CAP_MAX];
	struct ncsi_channel_mode	nc_modes[NCSI_MODE_MAX];
	struct ncsi_channel_filter	*nc_filters[NCSI_FILTER_MAX];
	struct ncsi_channel_stats	nc_stats;
	struct list_head		nc_node;
};

struct ncsi_package {
	unsigned char		np_id;
	struct ncsi_dev_priv	*np_ndp;
	atomic_t		np_channel_num;
	spinlock_t		np_channel_lock;
	struct list_head	np_channels;
	struct list_head	np_node;
};

struct ncsi_skb_parms {
	unsigned int		nsp_valid;
	unsigned int		nsp_portid;
	struct nlmsghdr		nsp_nlh;
};

#define NCSI_CB(skb)	(*(struct ncsi_skb_parms*)&((skb)->cb))

struct ncsi_req {
	unsigned char		nr_id;
	bool			nr_used;
	struct ncsi_dev_priv	*nr_ndp;
	struct sk_buff		*nr_cmd;
	struct sk_buff		*nr_rsp;
	struct timer_list	nr_timer;
	bool			nr_timer_enabled;
};

enum {
	ncsi_dev_state_major		= 0xff00,
	ncsi_dev_state_minor		= 0x00ff,
	ncsi_dev_state_start_deselect	= 0x0201,
	ncsi_dev_state_start_package,
	ncsi_dev_state_start_channel,
	ncsi_dev_state_start_sp,
	ncsi_dev_state_start_cis,
	ncsi_dev_state_start_gvi,
	ncsi_dev_state_start_gc,
	ncsi_dev_state_start_dp,
	ncsi_dev_state_start_active,
	ncsi_dev_state_config_sp	= 0x0301,
	ncsi_dev_state_config_cis,
	ncsi_dev_state_config_sma,
	ncsi_dev_state_config_ebf,
	ncsi_dev_state_config_ecnt,
	ncsi_dev_state_config_ec,
	ncsi_dev_state_config_gls,
	ncsi_dev_state_config_done,
	ncsi_dev_state_stop_select	= 0x0401,
	ncsi_dev_state_stop_dcnt,
	ncsi_dev_state_stop_dc,
	ncsi_dev_state_stop_deselect,
	ncsi_dev_state_stop_done
};

struct ncsi_dev_priv {
	struct ncsi_dev		ndp_ndev;
	int			ndp_flags;
#define NCSI_DEV_PRIV_FLAG_CHANGE_ACTIVE	0x1
	struct ncsi_package	*ndp_active_package;
	struct ncsi_channel	*ndp_active_channel;
	atomic_t		ndp_package_num;
	spinlock_t		ndp_package_lock;
	struct list_head	ndp_packages;
	atomic_t		ndp_pending_reqs;
	atomic_t		ndp_last_req_idx;
	spinlock_t		ndp_req_lock;
	struct ncsi_req		ndp_reqs[256];
	struct work_struct	ndp_work;
	struct packet_type	ndp_ptype;
	struct list_head	ndp_node;
};

struct ncsi_cmd_arg {
	struct ncsi_dev_priv	*nca_ndp;
	unsigned char		nca_type;
	unsigned char		nca_id;
	unsigned char		nca_package;
	unsigned char		nca_channel;
	unsigned short		nca_payload;
	struct nlmsghdr		*nca_nlh;
	unsigned int		nca_portid;
	union {
		unsigned char	nca_bytes[16];
		unsigned short	nca_words[8];
		unsigned int	nca_dwords[4];
	};
};

extern struct net *ncsi_net;
extern struct list_head ncsi_dev_list;
extern spinlock_t ncsi_dev_lock;

#define TO_NCSI_DEV_PRIV(nd) \
	container_of(nd, struct ncsi_dev_priv, ndp_ndev)
#define NCSI_FOR_EACH_DEV(ndp) \
	list_for_each_entry_rcu(ndp, &ncsi_dev_list, ndp_node)
#define NCSI_FOR_EACH_PACKAGE(ndp, np) \
	list_for_each_entry_rcu(np, &ndp->ndp_packages, np_node)
#define NCSI_FOR_EACH_CHANNEL(np, nc) \
	list_for_each_entry_rcu(nc, &np->np_channels, nc_node)

/* Resources */
int ncsi_find_channel_filter(struct ncsi_channel *nc, int table, void *data);
int ncsi_add_channel_filter(struct ncsi_channel *nc, int table, void *data);
int ncsi_del_channel_filter(struct ncsi_channel *nc, int table, int index);
struct ncsi_channel *ncsi_add_channel(struct ncsi_package *np,
				      unsigned char id);
struct ncsi_channel *ncsi_find_channel(struct ncsi_package *np,
				       unsigned char id);
struct ncsi_package *ncsi_add_package(struct ncsi_dev_priv *ndp,
				      unsigned char id);
struct ncsi_package *ncsi_find_package(struct ncsi_dev_priv *ndp,
				       unsigned char id);
void ncsi_release_package(struct ncsi_package *np);
void ncsi_find_package_and_channel(struct ncsi_dev_priv *ndp,
				   unsigned char id,
				   struct ncsi_package **np,
				   struct ncsi_channel **nc);
struct ncsi_req *ncsi_alloc_req(struct ncsi_dev_priv *ndp);
void ncsi_free_req(struct ncsi_req *nr, bool check, bool timeout);
struct ncsi_dev *ncsi_find_dev(struct net_device *dev);

/* Packet handlers */
int ncsi_xmit_cmd(struct ncsi_cmd_arg *nca);
int ncsi_rcv_rsp(struct sk_buff *skb, struct net_device *dev,
		 struct packet_type *pt, struct net_device *orig_dev);
int ncsi_aen_handler(struct ncsi_dev_priv *ndp, struct sk_buff *skb);

#endif /* __NCSI_INTERNAL_H__ */

#ifndef _UAPI_LINUX_NCSI_H
#define _UAPI_LINUX_NCSI_H

/* NCSI netlink message type */
enum {
	NCSI_MSG_BASE		= 16,
	NCSI_MSG_GET_LAYOUT	= 16,
	NCSI_MSG_GET_VERSION,
	NCSI_MSG_GET_CAP,
	NCSI_MSG_GET_MODE,
	NCSI_MSG_GET_FILTER,
	NCSI_MSG_GET_STATS,
	NCSI_MSG_SET_MODE,
	NCSI_MSG_SET_FILTER,
	NCSI_MSG_MAX
};


/* NCSI channel capabilities */
enum {
	NCSI_CAP_BASE		= 0,
	NCSI_CAP_GENERIC	= 0,
	NCSI_CAP_BC,
	NCSI_CAP_MC,
	NCSI_CAP_BUFFER,
	NCSI_CAP_AEN,
	NCSI_CAP_VLAN,
	NCSI_CAP_MAX
};

enum {
	NCSI_CAP_GENERIC_HWA	= 0x01,	/* HW arbitration             */
	NCSI_CAP_GENERIC_HDS	= 0x02,	/* HNC driver status change   */
	NCSI_CAP_GENERIC_FC	= 0x04,	/* HNC to MC flow control     */
	NCSI_CAP_GENERIC_FC1	= 0x08,	/* MC to HNC flow control     */
	NCSI_CAP_GENERIC_MC	= 0x10,	/* Global multicast filtering */
	NCSI_CAP_GENERIC_MASK	= 0x1f,
	NCSI_CAP_BC_ARP		= 0x01,	/* ARP packet filtering       */
	NCSI_CAP_BC_DHCPC	= 0x02,	/* DHCP client filtering      */
	NCSI_CAP_BC_DHCPS	= 0x04,	/* DHCP server filtering      */
	NCSI_CAP_BC_NETBIOS	= 0x08,	/* NetBIOS packet filtering   */
	NCSI_CAP_BC_MASK	= 0x0f,
	NCSI_CAP_MC_NEIGHBOR	= 0x01,	/* IPv6 neighbor filtering    */
	NCSI_CAP_MC_ROUTER	= 0x02,	/* IPv6 router filering       */
	NCSI_CAP_MC_DHCPv6	= 0x04,	/* DHCPv6 filtering           */
	NCSI_CAP_MC_MASK	= 0x07,
	NCSI_CAP_AEN_LSC	= 0x01,	/* Link status change AEN     */
	NCSI_CAP_AEN_CR		= 0x02,	/* Configuration required AEN */
	NCSI_CAP_AEN_HDS	= 0x04,	/* HNC driver status AEN      */
	NCSI_CAP_AEN_MASK	= 0x07,
	NCSI_CAP_VLAN_ONLY	= 0x01,	/* VLAN is supported          */
	NCSI_CAP_VLAN_NO	= 0x02,	/* Filter VLAN and non-VLAN   */
	NCSI_CAP_VLAN_ANY	= 0x04,	/* Filter Any-and-non-VLAN    */
	NCSI_CAP_VLAN_MASK	= 0x07
};

/* NCSI channel mode */
enum {
	NCSI_MODE_BASE		= 0,
	NCSI_MODE_ENABLE	= 0,
	NCSI_MODE_TX_ENABLE,
	NCSI_MODE_LINK,
	NCSI_MODE_VLAN,
	NCSI_MODE_BC,
	NCSI_MODE_MC,
	NCSI_MODE_AEN,
	NCSI_MODE_FC,
	NCSI_MODE_MAX
};

/* NCSI channel filters */
enum {
	NCSI_FILTER_BASE	= 0,
	NCSI_FILTER_VLAN	= 0,
	NCSI_FILTER_UC,
	NCSI_FILTER_MC,
	NCSI_FILTER_MIXED,
	NCSI_FILTER_MAX
};

/*
 * It's put right after netlink message header. Also, it's
 * used to convey NCSI topology layout.
 */
struct ncsi_msg {
	__u32	nm_flag;
#define NCSI_FLAG_REQUEST		0x1
#define NCSI_FLAG_RESPONSE		0x2
#define NCSI_FLAG_ACTIVE_CHANNEL	0x4

	__u32	nm_ifindex;		/* ID of network device               */
	__u32	nm_package_id;		/* ID of NCSI package                 */
	__u32	nm_channel_id;		/* ID of NCSI channel                 */
	__u32	nm_index;		/* ID of mode, capability or filter   */
	__u32	nm_errcode;		/* Error code                         */
};

enum {
	NCSI_SUCCESS,
	NCSI_ERR_PARAM,
	NCSI_ERR_NO_MEM,
	NCSI_ERR_NO_DEV,
	NCSI_ERR_NOT_ACTIVE,
	NCSI_ERR_INTERNAL,
};

/* NCSI channel version */
struct ncsi_channel_version {
	__u32	ncv_version;		/* Supported BCD encoded NCSI version */
	__u32	ncv_alpha2;		/* Supported BCD encoded NCSI version */
	__u8	ncv_fw_name[12];	/* Firware name string                */
	__u32	ncv_fw_version;		/* Firmware version                   */
	__u16	ncv_pci_ids[4];		/* PCI identification                 */
	__u32	ncv_mf_id;		/* Manufacture ID                     */
};

/* NCSI channel capability */
struct ncsi_channel_cap {
	__u32	ncc_index;		/* Index of channel capabilities     */
	__u32	ncc_cap;		/* NCSI channel capability           */
};

/* NCSI channel mode */
struct ncsi_channel_mode {
	__u32	ncm_index;		/* Index of channel modes            */
	__u32	ncm_enable;		/* Enabled or disabled               */
	__u32	ncm_size;		/* Valid entries in ncm_data[]       */
	__u32	ncm_data[8];		/* Data entries                      */
};

/* NCSI channel filter */
struct ncsi_channel_filter {
	__u32	ncf_index;		/* Index of channel filters          */
	__u32	ncf_total;		/* Total entries in the filter table */
	__u64	ncf_bitmap;		/* Bitmap of valid entries           */
	__u8	ncf_data[];		/* Data for the valid entries        */
};

/* NCSI channel statistics */
struct ncsi_channel_stats {
	__u32	ncs_hnc_cnt_hi;			/* Counter cleared            */
	__u32	ncs_hnc_cnt_lo;			/* Counter cleared            */
	__u32	ncs_hnc_rx_bytes;		/* Rx bytes                   */
	__u32	ncs_hnc_tx_bytes;		/* Tx bytes                   */
	__u32	ncs_hnc_rx_uc_pkts;		/* Rx UC packets              */
	__u32	ncs_hnc_rx_mc_pkts;		/* Rx MC packets              */
	__u32	ncs_hnc_rx_bc_pkts;		/* Rx BC packets              */
	__u32	ncs_hnc_tx_uc_pkts;		/* Tx UC packets              */
	__u32	ncs_hnc_tx_mc_pkts;		/* Tx MC packets              */
	__u32	ncs_hnc_tx_bc_pkts;		/* Tx BC packets              */
	__u32	ncs_hnc_fcs_err;		/* FCS errors                 */
	__u32	ncs_hnc_align_err;		/* Alignment errors           */
	__u32	ncs_hnc_false_carrier;		/* False carrier detection    */
	__u32	ncs_hnc_runt_pkts;		/* Rx runt packets            */
	__u32	ncs_hnc_jabber_pkts;		/* Rx jabber packets          */
	__u32	ncs_hnc_rx_pause_xon;		/* Rx pause XON frames        */
	__u32	ncs_hnc_rx_pause_xoff;		/* Rx XOFF frames             */
	__u32	ncs_hnc_tx_pause_xon;		/* Tx XON frames              */
	__u32	ncs_hnc_tx_pause_xoff;		/* Tx XOFF frames             */
	__u32	ncs_hnc_tx_s_collision;		/* Single collision frames    */
	__u32	ncs_hnc_tx_m_collision;		/* Multiple collision frames  */
	__u32	ncs_hnc_l_collision;		/* Late collision frames      */
	__u32	ncs_hnc_e_collision;		/* Excessive collision frames */
	__u32	ncs_hnc_rx_ctl_frames;		/* Rx control frames          */
	__u32	ncs_hnc_rx_64_frames;		/* Rx 64-bytes frames         */
	__u32	ncs_hnc_rx_127_frames;		/* Rx 65-127 bytes frames     */
	__u32	ncs_hnc_rx_255_frames;		/* Rx 128-255 bytes frames    */
	__u32	ncs_hnc_rx_511_frames;		/* Rx 256-511 bytes frames    */
	__u32	ncs_hnc_rx_1023_frames;		/* Rx 512-1023 bytes frames   */
	__u32	ncs_hnc_rx_1522_frames;		/* Rx 1024-1522 bytes frames  */
	__u32	ncs_hnc_rx_9022_frames;		/* Rx 1523-9022 bytes frames  */
	__u32	ncs_hnc_tx_64_frames;		/* Tx 64-bytes frames         */
	__u32	ncs_hnc_tx_127_frames;		/* Tx 65-127 bytes frames     */
	__u32	ncs_hnc_tx_255_frames;		/* Tx 128-255 bytes frames    */
	__u32	ncs_hnc_tx_511_frames;		/* Tx 256-511 bytes frames    */
	__u32	ncs_hnc_tx_1023_frames;		/* Tx 512-1023 bytes frames   */
	__u32	ncs_hnc_tx_1522_frames;		/* Tx 1024-1522 bytes frames  */
	__u32	ncs_hnc_tx_9022_frames;		/* Tx 1523-9022 bytes frames  */
	__u32	ncs_hnc_rx_valid_bytes;		/* Rx valid bytes             */
	__u32	ncs_hnc_rx_runt_pkts;		/* Rx error runt packets      */
	__u32	ncs_hnc_rx_jabber_pkts;		/* Rx error jabber packets    */
	__u32	ncs_ncsi_rx_cmds;		/* Rx NCSI commands           */
	__u32	ncs_ncsi_dropped_cmds;		/* Dropped commands           */
	__u32	ncs_ncsi_cmd_type_errs;		/* Command type errors        */
	__u32	ncs_ncsi_cmd_csum_errs;		/* Command checksum errors    */
	__u32	ncs_ncsi_rx_pkts;		/* Rx NCSI packets            */
	__u32	ncs_ncsi_tx_pkts;		/* Tx NCSI packets            */
	__u32	ncs_ncsi_tx_aen_pkts;		/* Tx AEN packets             */
	__u32	ncs_pt_tx_pkts;			/* Tx packets                 */
	__u32	ncs_pt_tx_dropped;		/* Tx dropped packets         */
	__u32	ncs_pt_tx_channel_err;		/* Tx channel errors          */
	__u32	ncs_pt_tx_us_err;		/* Tx undersize errors        */
	__u32	ncs_pt_rx_pkts;			/* Rx packets                 */
	__u32	ncs_pt_rx_dropped;		/* Rx dropped packets         */
	__u32	ncs_pt_rx_channel_err;		/* Rx channel errors          */
	__u32	ncs_pt_rx_us_err;		/* Rx undersize errors        */
	__u32	ncs_pt_rx_os_err;		/* Rx oversize errors         */
};

#endif /* _UAPI_LINUX_NCSI_H */

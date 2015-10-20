/*
 * include/linux/bt-host.h
 *
 */

#ifndef _UAPI_LINUX_BT_HOST_H
#define _UAPI_LINUX_BT_HOST_H

#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/irqnr.h>

#define BT_HOST_IOCTL_MAGIC	0xb1
#define BT_HOST_IOCTL_SMS_ATN	_IO( BT_HOST_IOCTL_MAGIC, 0x00 )

#endif /* _UAPI_LINUX_BT_HOST_H */

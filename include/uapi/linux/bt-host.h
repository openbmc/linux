/*
 * Copyright 2015 IBM Corp.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#ifndef _UAPI_LINUX_BT_HOST_H
#define _UAPI_LINUX_BT_HOST_H

#include <linux/ioctl.h>

#define __BT_HOST_IOCTL_MAGIC	0xb1
#define BT_HOST_IOCTL_SMS_ATN	_IO(__BT_HOST_IOCTL_MAGIC, 0x00)

#endif /* _UAPI_LINUX_BT_HOST_H */

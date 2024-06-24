/* SPDX-License-Identifier: GPL-2.0-only */
/* SPDX-FileCopyrightText: Copyright (c) 2009-2024 NVIDIA CORPORATION & AFFILIATES. All Rights Reserved. */

#ifndef __LINUX_NVHOST_H
#define __LINUX_NVHOST_H

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/host1x.h>
#include <linux/platform_device.h>

struct nvhost_device_data {
	void *private_data;		/* private platform data */

	/* Information related to engine-side synchronization */
	void *syncpt_unit_interface;
};

/* public api to return platform_device ptr to the default host1x instance */
struct platform_device *nvhost_get_default_device(void);

/* common device management APIs */
int nvhost_client_device_release(struct platform_device *dev);
int nvhost_client_device_init(struct platform_device *dev);

/* public host1x sync-point management APIs */
u32 nvhost_get_syncpt_client_managed(struct platform_device *pdev,
				     const char *syncpt_name);
void nvhost_syncpt_put_ref_ext(struct platform_device *pdev, u32 id);
int nvhost_syncpt_read_ext_check(struct platform_device *dev, u32 id, u32 *val);
dma_addr_t nvhost_syncpt_address(struct platform_device *engine_pdev, u32 id);
int nvhost_syncpt_unit_interface_init(struct platform_device *pdev);

/* public host1x sync-point management APIs */
struct host1x *nvhost_get_host1x(struct platform_device *pdev);

#endif

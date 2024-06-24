// SPDX-License-Identifier: GPL-2.0-only
/*
 * Capture support for syncpoint and GoS management
 *
 * Copyright (c) 2017-2022, NVIDIA Corporation.  All rights reserved.
 */

#include "capture-support.h"
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/nvhost.h>

int capture_alloc_syncpt(struct platform_device *pdev,
			const char *name,
			uint32_t *syncpt_id)
{
	uint32_t id;

	if (syncpt_id == NULL) {
		dev_err(&pdev->dev, "%s: null argument\n", __func__);
		return -EINVAL;
	}

	id = nvhost_get_syncpt_client_managed(pdev, name);
	if (id == 0) {
		dev_err(&pdev->dev, "%s: syncpt allocation failed\n", __func__);
		return -ENODEV;
	}

	*syncpt_id = id;

	return 0;
}
EXPORT_SYMBOL_GPL(capture_alloc_syncpt);

void capture_release_syncpt(struct platform_device *pdev, uint32_t id)
{
	dev_dbg(&pdev->dev, "%s: id=%u\n", __func__, id);
	nvhost_syncpt_put_ref_ext(pdev, id);
}
EXPORT_SYMBOL_GPL(capture_release_syncpt);

int capture_get_syncpt_gos_backing(struct platform_device *pdev,
			uint32_t id,
			dma_addr_t *syncpt_addr)
{
	dma_addr_t addr;

	if (id == 0) {
		dev_err(&pdev->dev, "%s: syncpt id is invalid\n", __func__);
		return -EINVAL;
	}

	if (syncpt_addr == NULL) {
		dev_err(&pdev->dev, "%s: null arguments\n", __func__);
		return -EINVAL;
	}

	addr = nvhost_syncpt_address(pdev, id);

	*syncpt_addr = addr;

	dev_dbg(&pdev->dev, "%s: id=%u addr=0x%llx\n",
		__func__, id, addr);

	return 0;
}
EXPORT_SYMBOL_GPL(capture_get_syncpt_gos_backing);

MODULE_LICENSE("GPL");

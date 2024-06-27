// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES. All rights reserved.

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/tegra-camera-rtcpu.h>
#include <linux/tegra-rtcpu-monitor.h>

struct tegra_camrtc_mon {
	struct device *rce_dev;
	int wdt_irq;
};

static irqreturn_t tegra_camrtc_mon_wdt_remote_isr(int irq, void *data)
{
	struct tegra_camrtc_mon *cam_rtcpu_mon = data;

	disable_irq_nosync(irq);

	tegra_camrtc_reboot(cam_rtcpu_mon->rce_dev);

	enable_irq(cam_rtcpu_mon->wdt_irq);

	return IRQ_HANDLED;
}

struct tegra_camrtc_mon *tegra_camrtc_mon_create(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_camrtc_mon *cam_rtcpu_mon;
	int ret;

	cam_rtcpu_mon = devm_kzalloc(dev, sizeof(*cam_rtcpu_mon), GFP_KERNEL);
	if (unlikely(cam_rtcpu_mon == NULL))
		return ERR_PTR(-ENOMEM);

	cam_rtcpu_mon->rce_dev = dev;

	ret = platform_get_irq_byname(pdev, "wdt-remote");
	if (ret < 0) {
		dev_err(dev, "Failed to find wdt-remote IRQ: %d\n", ret);
		return ERR_PTR(ret);
	}

	cam_rtcpu_mon->wdt_irq = ret;

	ret = devm_request_threaded_irq(dev, cam_rtcpu_mon->wdt_irq,
					NULL, tegra_camrtc_mon_wdt_remote_isr,
					IRQF_ONESHOT, dev_name(dev),
					cam_rtcpu_mon);
	if (ret)
		return ERR_PTR(ret);

	return cam_rtcpu_mon;
}
EXPORT_SYMBOL(tegra_camrtc_mon_create);

int tegra_cam_rtcpu_mon_destroy(struct tegra_camrtc_mon *cam_rtcpu_mon)
{
	devm_kfree(cam_rtcpu_mon->rce_dev, cam_rtcpu_mon);

	return 0;
}
EXPORT_SYMBOL(tegra_cam_rtcpu_mon_destroy);

MODULE_DESCRIPTION("CAMERA RTCPU monitor driver");
MODULE_AUTHOR("Sudhir Vyas <svyas@nvidia.com>");
MODULE_LICENSE("GPL v2");

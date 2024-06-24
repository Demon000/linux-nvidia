// SPDX-License-Identifier: GPL-2.0-only
// SPDX-FileCopyrightText: Copyright (c) 2017-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/interconnect.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <media/fusa-capture/capture-vi-channel.h>
#include <media/tegra_camera_platform.h>

#include "capture/capture-support.h"

/* HW capability, pixels per clock */
#define NUM_PPC		8
/* 15% bus protocol overhead */
/* + 5% SW overhead */
#define VI_OVERHEAD	20

#define VI_CLASS_ID 0x30

struct host_vi5 {
	struct icc_path *icc_write;
	struct clk *clk;
};

static struct vi_channel_drv_ops vi5_channel_drv_ops = {
	.alloc_syncpt = capture_alloc_syncpt,
	.release_syncpt = capture_release_syncpt,
	.get_syncpt_gos_backing = capture_get_syncpt_gos_backing,
};

static int vi5_priv_early_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct host_vi5 *vi5;
	int err = 0;

	vi5 = devm_kzalloc(dev, sizeof(*vi5), GFP_KERNEL);
	if (!vi5)
		return -ENOMEM;

	platform_set_drvdata(pdev, vi5);

	err = vi_channel_drv_fops_register(&vi5_channel_drv_ops);
	if (err) {
		dev_warn(&pdev->dev, "syncpt fops register failed, defer probe\n");
		return err;
	}

	(void) dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(39));

#ifdef CONFIG_DMABUF_DEFERRED_UNMAPPING
	if (dma_buf_defer_unmapping(dev, true) < 0)
		dev_warn(dev, "Failed to set deferred dma buffer unmapping\n");
#endif

	return 0;
}

static int vi5_set_rate(struct tegra_camera_dev_info *cdev_info, unsigned long rate)
{
	struct host_vi5 *vi5 = platform_get_drvdata(cdev_info->pdev);

	return clk_set_rate(vi5->clk, rate);
}

static struct tegra_camera_dev_ops vi5_cdev_ops = {
	.set_rate = vi5_set_rate,
};

static int vi5_priv_late_probe(struct platform_device *pdev)
{
	struct tegra_camera_dev_info vi_info;
	struct host_vi5 *vi5 = platform_get_drvdata(pdev);
	int err;

	memset(&vi_info, 0, sizeof(vi_info));
	vi_info.pdev = pdev;
	vi_info.hw_type = HWTYPE_VI;
	vi_info.ppc = NUM_PPC;
	vi_info.overhead = VI_OVERHEAD;
	vi_info.ops = &vi5_cdev_ops;

	err = tegra_camera_device_register(&vi_info, vi5);
	if (err)
		return err;

	return 0;
}

static int vi5_probe(struct platform_device *pdev)
{
	struct reset_control *reset_control;
	struct device *dev = &pdev->dev;
	struct host_vi5 *vi5;
	int err;

	reset_control = devm_reset_control_get_exclusive_released(dev, NULL);
	if (IS_ERR(reset_control)) {
		dev_err(dev, "failed to get reset\n");
		return PTR_ERR(reset_control);
	}

	err = reset_control_acquire(reset_control);
	if (err) {
		dev_err(dev, "failed to acquire reset: %d\n", err);
		return err;
	}

	reset_control_reset(reset_control);
	reset_control_release(reset_control);

	err = vi5_priv_early_probe(pdev);
	if (err)
		return err;

	vi5 = platform_get_drvdata(pdev);

	vi5->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(vi5->clk)) {
		dev_err(dev, "failed to get clock\n");
		return PTR_ERR(vi5->clk);
	}

	vi5->icc_write = devm_of_icc_get(dev, "write");
	if (IS_ERR(vi5->icc_write)) {
		dev_err(dev, "failed to get icc write handle\n");
		return PTR_ERR(vi5->icc_write);
	}

	err = vi5_priv_late_probe(pdev);
	if (err)
		return err;

	return 0;
}

static int vi5_remove(struct platform_device *pdev)
{
	struct host_vi5 *vi5 = platform_get_drvdata(pdev);

	tegra_camera_device_unregister(vi5);

	return 0;
}

static const struct of_device_id tegra_vi5_of_match[] = {
	{
		.name = "vi",
		.compatible = "nvidia,tegra194-vi",
	},
	{
		.name = "vi0",
		.compatible = "nvidia,tegra234-vi",
	},
	{
		.name = "vi1",
		.compatible = "nvidia,tegra234-vi",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_vi5_of_match);

static int vi_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct host_vi5 *vi5 = platform_get_drvdata(pdev);
	int err;

	if (vi5->icc_write) {
		err = icc_set_bw(vi5->icc_write, 0, 0);
		if (err)
			dev_warn(dev,
				 "failed to set icc_write bw: %d\n", err);
		return 0;
	}

	return -EOPNOTSUPP;
}

static int vi_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct host_vi5 *vi5 = platform_get_drvdata(pdev);
	int err;

	if (vi5->icc_write) {
		err = icc_set_bw(vi5->icc_write, 0, UINT_MAX);
		if (err)
			dev_warn(dev,
				 "failed to set icc_write bw: %d\n", err);
		return 0;
	}

	return -EOPNOTSUPP;
}

const struct dev_pm_ops vi_pm_ops = {
	SET_RUNTIME_PM_OPS(vi_runtime_suspend, vi_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static struct platform_driver vi5_driver = {
	.probe = vi5_probe,
	.remove = vi5_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "tegra194-vi5",
#ifdef CONFIG_OF
		.of_match_table = tegra_vi5_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &vi_pm_ops,
#endif
	},
};

module_platform_driver(vi5_driver);
MODULE_LICENSE("GPL");

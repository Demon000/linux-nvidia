// SPDX-License-Identifier: GPL-2.0-only
/*
 * SPDX-FileCopyrightText: Copyright (C) 2017-2023 NVIDIA CORPORATION.  All rights reserved.
 */

#include <linux/tegra-camera-rtcpu.h>

#include <asm/ioctls.h>
#include <linux/device.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>

#include <media/csi.h>
#include <media/mc_common.h>
#include <media/tegra_camera_platform.h>
#include "camera/nvcsi/csi5_fops.h"

/* width of interface between VI and CSI */
#define CSI_BUS_WIDTH	64
/* number of lanes per brick */
#define NUM_LANES	4

struct t194_nvcsi {
	struct tegra_csi_device csi;
	struct clk *clk;
};

static const struct of_device_id tegra194_nvcsi_of_match[] = {
	{
		.compatible = "nvidia,tegra194-nvcsi",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, tegra194_nvcsi_of_match);

static int t194_nvcsi_set_rate(struct tegra_camera_dev_info *cdev_info, unsigned long rate)
{
	struct t194_nvcsi *nvcsi = platform_get_drvdata(cdev_info->pdev);

	return clk_set_rate(nvcsi->clk, rate);
}

static struct tegra_camera_dev_ops t194_nvcsi_cdev_ops = {
	.set_rate = t194_nvcsi_set_rate,
};

static int t194_nvcsi_probe(struct platform_device *pdev)
{
	struct tegra_camera_dev_info csi_info;
	struct device *dev = &pdev->dev;
	struct t194_nvcsi *nvcsi;
	int err;

	nvcsi = devm_kzalloc(&pdev->dev, sizeof(*nvcsi), GFP_KERNEL);
	if (!nvcsi)
		return -ENOMEM;

	platform_set_drvdata(pdev, nvcsi);

	nvcsi->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(nvcsi->clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		return PTR_ERR(nvcsi->clk);
	}

	memset(&csi_info, 0, sizeof(csi_info));
	csi_info.pdev = pdev;
	csi_info.hw_type = HWTYPE_CSI;
	csi_info.use_max = true;
	csi_info.bus_width = CSI_BUS_WIDTH;
	csi_info.lane_num = NUM_LANES;
	csi_info.ops = &t194_nvcsi_cdev_ops;

	err = tegra_camera_device_register(&csi_info, nvcsi);
	if (err)
		return err;

	nvcsi->csi.fops = &csi5_fops;
	err = tegra_csi_media_controller_init(&nvcsi->csi, pdev);
	if (err) {
		tegra_camera_device_unregister(nvcsi);
		return err;
	}

	return 0;
}

static int __exit t194_nvcsi_remove(struct platform_device *dev)
{
	struct t194_nvcsi *nvcsi = platform_get_drvdata(dev);

	tegra_camera_device_unregister(nvcsi);
	tegra_csi_media_controller_remove(&nvcsi->csi);

	return 0;
}

static struct platform_driver t194_nvcsi_driver = {
	.probe = t194_nvcsi_probe,
	.remove = __exit_p(t194_nvcsi_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "t194-nvcsi",
#ifdef CONFIG_OF
		.of_match_table = tegra194_nvcsi_of_match,
#endif
	},
};

module_platform_driver(t194_nvcsi_driver);
MODULE_LICENSE("GPL");

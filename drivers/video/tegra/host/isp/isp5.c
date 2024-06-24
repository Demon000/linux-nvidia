// SPDX-License-Identifier: GPL-2.0-only
// SPDX-FileCopyrightText: Copyright (c) 2017-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/reset.h>
#include <media/fusa-capture/capture-isp-channel.h>
#include <media/tegra_camera_platform.h>

#include "capture/capture-support.h"

#define ISP_PPC		2
/* 20% overhead */
#define ISP_OVERHEAD	20

struct host_isp5 {
	struct clk *clk;
	struct miscdevice misc;
};

static struct isp_channel_drv_ops isp5_channel_drv_ops = {
	.alloc_syncpt = capture_alloc_syncpt,
	.release_syncpt = capture_release_syncpt,
	.get_syncpt_gos_backing = capture_get_syncpt_gos_backing,
};

static int isp5_priv_early_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct host_isp5 *isp5;

	isp5 = devm_kzalloc(dev, sizeof(*isp5), GFP_KERNEL);
	if (!isp5)
		return -ENOMEM;

	platform_set_drvdata(pdev, isp5);

	/* A bit was stolen */
	(void) dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(39));

#ifdef CONFIG_DMABUF_DEFERRED_UNMAPPING
	if (dma_buf_defer_unmapping(dev, true) < 0)
		dev_warn(dev, "Failed to set deferred dma buffer unmapping\n");
#endif

	return 0;
}

static int isp5_set_rate(struct tegra_camera_dev_info *cdev_info, unsigned long rate)
{
	struct host_isp5 *isp5 = platform_get_drvdata(cdev_info->pdev);

	return clk_set_rate(isp5->clk, rate);
}

static struct tegra_camera_dev_ops isp5_cdev_ops = {
	.set_rate = isp5_set_rate,
};

static int isp5_priv_late_probe(struct platform_device *pdev)
{
	struct tegra_camera_dev_info isp_info;
	struct host_isp5 *isp5 = platform_get_drvdata(pdev);
	int err;

	memset(&isp_info, 0, sizeof(isp_info));
	isp_info.overhead = ISP_OVERHEAD;
	isp_info.ppc = ISP_PPC;
	isp_info.hw_type = HWTYPE_ISPA;
	isp_info.pdev = pdev;
	isp_info.ops = &isp5_cdev_ops;

	err = tegra_camera_device_register(&isp_info, isp5);
	if (err)
		return err;

	err = isp_channel_drv_register(pdev, &isp5_channel_drv_ops);
	if (err)
		return err;

	return 0;
}

static long isp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return 0;
}

static int isp_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int isp_release(struct inode *inode, struct file *file)
{
	return 0;
}

const struct file_operations tegra194_isp5_ctrl_ops = {
	.owner = THIS_MODULE,
	.open = isp_open,
	.unlocked_ioctl = isp_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = isp_ioctl,
#endif
	.release = isp_release,
};

static int isp5_probe(struct platform_device *pdev)
{
	struct reset_control *reset_control;
	struct device *dev = &pdev->dev;
	struct host_isp5 *isp5;
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

	err = isp5_priv_early_probe(pdev);
	if (err)
		return err;

	isp5 = platform_get_drvdata(pdev);

	isp5->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(isp5->clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		return PTR_ERR(isp5->clk);
	}

	isp5->misc = (struct miscdevice) {
		.minor	= MISC_DYNAMIC_MINOR,
		.name	= "nvhost-ctrl-isp",
		.fops	= &tegra194_isp5_ctrl_ops,
		.mode	= 0666,
	};

	err = misc_register(&isp5->misc);
	if (err) {
		dev_err(dev, "failed to register misc device: %d\n", err);
		return err;
	}

	err = isp5_priv_late_probe(pdev);
	if (err)
		return err;

	return 0;
}

static int isp5_remove(struct platform_device *pdev)
{
	struct host_isp5 *isp5 = platform_get_drvdata(pdev);

	tegra_camera_device_unregister(isp5);

	isp_channel_drv_unregister(&pdev->dev);

	misc_deregister(&isp5->misc);

	return 0;
}

static const struct of_device_id tegra_isp5_of_match[] = {
	{
		.compatible = "nvidia,tegra194-isp",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_isp5_of_match);

static struct platform_driver isp5_driver = {
	.probe = isp5_probe,
	.remove = isp5_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "tegra194-isp5",
#ifdef CONFIG_OF
		.of_match_table = tegra_isp5_of_match,
#endif
	},
};

static int __init capture_isp_init(void)
{
	int err;

	err = isp_channel_drv_init();
	if (err)
		return err;

	err = platform_driver_register(&isp5_driver);
	if (err) {
		isp_channel_drv_exit();
		return err;
	}

	return 0;
}
static void __exit capture_isp_exit(void)
{
	isp_channel_drv_exit();
	platform_driver_unregister(&isp5_driver);
}

module_init(capture_isp_init);
module_exit(capture_isp_exit);
MODULE_LICENSE("GPL");

/*
 * ISP5 driver for T194
 *
 * Copyright (c) 2017, NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <asm/ioctls.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/isp_channel.h>
#include <soc/tegra/camrtc-capture.h>
#include <soc/tegra/chip-id.h>

#include "isp5.h"
#include "capture/capture-support.h"
#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "nvhost_syncpt_unit_interface.h"
#include "t194/t194.h"
#include <linux/nvhost_isp_ioctl.h>

struct host_isp5 {
	struct platform_device *pdev;
	struct platform_device *isp_thi;
};

static int isp5_alloc_syncpt(struct platform_device *pdev,
			const char *name,
			uint32_t *syncpt_id)
{
	struct host_isp5 *isp5 = nvhost_get_private_data(pdev);

	return t194_capture_alloc_syncpt(isp5->isp_thi, name, syncpt_id);
}

static void isp5_release_syncpt(struct platform_device *pdev, uint32_t id)
{
	struct host_isp5 *isp5 = nvhost_get_private_data(pdev);

	t194_capture_release_syncpt(isp5->isp_thi, id);
}

static int isp5_get_syncpt_gos_backing(struct platform_device *pdev,
			uint32_t id,
			dma_addr_t *syncpt_addr,
			uint32_t *gos_index,
			uint32_t *gos_offset)
{
	struct host_isp5 *isp5 = nvhost_get_private_data(pdev);

	return t194_capture_get_syncpt_gos_backing(isp5->isp_thi, id,
				syncpt_addr, gos_index, gos_offset);

}

static uint32_t isp5_get_gos_table(struct platform_device *pdev,
			const dma_addr_t **table)
{
	struct host_isp5 *isp5 = nvhost_get_private_data(pdev);
	uint32_t count;

	t194_capture_get_gos_table(isp5->isp_thi, &count, table);

	return count;
}

static struct isp_channel_drv_ops isp5_channel_drv_ops = {
	.alloc_syncpt = isp5_alloc_syncpt,
	.release_syncpt = isp5_release_syncpt,
	.get_gos_table = isp5_get_gos_table,
	.get_syncpt_gos_backing = isp5_get_syncpt_gos_backing,
};

static int isp5_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvhost_device_data *info;
	struct device_node *thi_np;
	struct platform_device *thi = NULL;
	struct host_isp5 *isp5;
	int err = 0;

	info = (void *)of_device_get_match_data(dev);
	if (unlikely(info == NULL)) {
		dev_WARN(dev, "no platform data\n");
		return -ENODATA;
	}

	thi_np = of_parse_phandle(dev->of_node, "nvidia,isp-falcon-device", 0);
	if (thi_np == NULL) {
		dev_WARN(dev, "missing %s handle\n",
			"nvidia,isp-falcon-device");
		err = -ENODEV;
		goto error;
	}

	thi = of_find_device_by_node(thi_np);
	of_node_put(thi_np);

	if (thi == NULL) {
		err = -ENODEV;
		goto error;
	}

	if (thi->dev.driver == NULL) {
		platform_device_put(thi);
		return -EPROBE_DEFER;
	}

	isp5 = devm_kzalloc(dev, sizeof(*isp5), GFP_KERNEL);
	if (!isp5)
		return -ENOMEM;

	isp5->isp_thi = thi;
	isp5->pdev = pdev;
	info->pdev = pdev;
	mutex_init(&info->lock);
	platform_set_drvdata(pdev, info);
	info->private_data = isp5;

	err = nvhost_client_device_get_resources(pdev);
	if (err)
		goto put_thi;

	err = nvhost_module_init(pdev);
	if (err)
		goto put_thi;

	err = nvhost_client_device_init(pdev);
	if (err)
		goto deinit;

#if defined(CONFIG_TEGRA_CAMERA_RTCPU)
	err = isp_channel_drv_register(pdev, &isp5_channel_drv_ops);
	if (err)
		goto device_release;
#endif

	return 0;

device_release:
	nvhost_client_device_release(pdev);
deinit:
	nvhost_module_deinit(pdev);
put_thi:
	platform_device_put(thi);
error:
	if (err != -EPROBE_DEFER)
		dev_err(dev, "probe failed: %d\n", err);
	return err;
}

static long isp_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	struct t194_isp5_file_private *filepriv = file->private_data;
	struct platform_device *pdev = filepriv->pdev;

	if (_IOC_TYPE(cmd) != NVHOST_ISP_IOCTL_MAGIC)
		return -EFAULT;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(NVHOST_ISP_IOCTL_GET_ISP_CLK): {
		int ret;
		u64 isp_clk_rate = 0;

		ret = nvhost_module_get_rate(pdev,
			(unsigned long *)&isp_clk_rate, 0);
		if (ret) {
			dev_err(&pdev->dev,
			"%s: failed to get isp clk\n",
			__func__);
			return ret;
		}

		if (copy_to_user((void __user *)arg,
			&isp_clk_rate, sizeof(isp_clk_rate))) {
			dev_err(&pdev->dev,
			"%s:Failed to copy isp clk rate to user\n",
			__func__);
			return -EFAULT;
		}

		return 0;
	}
	case _IOC_NR(NVHOST_ISP_IOCTL_SET_ISP_CLK): {
		long isp_clk_rate = 0;

		if (copy_from_user(&isp_clk_rate,
			(const void __user *)arg, sizeof(long))) {
			dev_err(&pdev->dev,
				"%s: Failed to copy arg from user\n", __func__);
			return -EFAULT;
		}

		return nvhost_module_set_rate(pdev,
				filepriv, isp_clk_rate, 0, NVHOST_CLOCK);
	}
	case _IOC_NR(NVHOST_ISP_IOCTL_SET_ISP_LA_BW): {
		/* No BW control needed. Return without error. */
		return 0;
	}
	default:
		dev_err(&pdev->dev,
		"%s: Unknown ISP ioctl.\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int isp_open(struct inode *inode, struct file *file)
{
	struct nvhost_device_data *pdata = container_of(inode->i_cdev,
					struct nvhost_device_data, ctrl_cdev);
	struct platform_device *pdev = pdata->pdev;
	struct t194_isp5_file_private *filepriv;

	filepriv = kzalloc(sizeof(*filepriv), GFP_KERNEL);
	if (unlikely(filepriv == NULL))
		return -ENOMEM;

	filepriv->pdev = pdev;

	if (nvhost_module_add_client(pdev, filepriv)) {
		kfree(filepriv);
		return -ENOMEM;
	}

	file->private_data = filepriv;

	return nonseekable_open(inode, file);
}

static int isp_release(struct inode *inode, struct file *file)
{
	struct t194_isp5_file_private *filepriv = file->private_data;
	struct platform_device *pdev = filepriv->pdev;

	nvhost_module_remove_client(pdev, filepriv);
	kfree(filepriv);

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

static int isp5_remove(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct host_isp5 *isp5 = (struct host_isp5 *)pdata->private_data;

#if defined(CONFIG_TEGRA_CAMERA_RTCPU)
	isp_channel_drv_unregister(&pdev->dev);
#endif

	platform_device_put(isp5->isp_thi);

	return 0;
}

static const struct of_device_id tegra_isp5_of_match[] = {
	{
		.compatible = "nvidia,tegra194-isp",
		.data = &t19_isp5_info,
	},
	{ },
};

static struct platform_driver isp5_driver = {
	.probe = isp5_probe,
	.remove = isp5_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "tegra194-isp5",
#ifdef CONFIG_OF
		.of_match_table = tegra_isp5_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
	},
};

module_platform_driver(isp5_driver);

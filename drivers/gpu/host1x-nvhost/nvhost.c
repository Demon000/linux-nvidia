// SPDX-License-Identifier: GPL-2.0-only
// SPDX-FileCopyrightText: Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All Rights Reserved.

#include <nvidia/conftest.h>

#include <linux/clk.h>
#include <linux/dma-fence.h>
#include <linux/dma-mapping.h>
#include <linux/host1x-next.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/nvhost.h>
#include <linux/nvhost_t194.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/version.h>

#include "falcon.h"

#define TEGRA194_SYNCPT_PAGE_SIZE 0x1000
#define TEGRA194_SYNCPT_SHIM_BASE 0x60000000
#define TEGRA194_SYNCPT_SHIM_SIZE 0x00400000
#define TEGRA234_SYNCPT_PAGE_SIZE 0x10000
#define TEGRA234_SYNCPT_SHIM_BASE 0x60000000
#define TEGRA234_SYNCPT_SHIM_SIZE 0x04000000

#define THI_STREAMID0	0x00000030
#define THI_STREAMID1	0x00000034

#define NVHOST_NUM_CDEV 1

struct nvhost_syncpt_interface {
	dma_addr_t base;
	size_t size;
	uint32_t page_size;
};

static const struct of_device_id host1x_match[] = {
	{ .compatible = "nvidia,tegra194-host1x", },
	{ .compatible = "nvidia,tegra234-host1x", },
	{},
};

struct platform_device *nvhost_get_default_device(void)
{
	struct platform_device *host1x_pdev;
	struct device_node *np;

	np = of_find_matching_node(NULL, host1x_match);
	if (!np)
		return NULL;

	host1x_pdev = of_find_device_by_node(np);
	if (!host1x_pdev)
		return NULL;

	return host1x_pdev;
}
EXPORT_SYMBOL(nvhost_get_default_device);

struct host1x *nvhost_get_host1x(struct platform_device *pdev)
{
	struct platform_device *host1x_pdev;
	struct host1x *host1x;

	host1x_pdev = nvhost_get_default_device();
	if (!host1x_pdev) {
		dev_dbg(&pdev->dev, "host1x device not available\n");
		return NULL;
	}

	host1x = platform_get_drvdata(host1x_pdev);
	if (!host1x) {
		dev_warn(&pdev->dev, "No platform data for host1x!\n");
		return NULL;
	}

	return host1x;
}
EXPORT_SYMBOL(nvhost_get_host1x);

u32 nvhost_get_syncpt_client_managed(struct platform_device *pdev,
				     const char *syncpt_name)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct host1x_syncpt *sp;

	sp = host1x_syncpt_alloc(pdata->host1x, HOST1X_SYNCPT_CLIENT_MANAGED,
				 syncpt_name ? syncpt_name :
						     dev_name(&pdev->dev));
	if (!sp)
		return 0;

	return host1x_syncpt_id(sp);
}
EXPORT_SYMBOL_GPL(nvhost_get_syncpt_client_managed);

void nvhost_syncpt_put_ref_ext(struct platform_device *pdev, u32 id)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct host1x_syncpt *sp;

	sp = host1x_syncpt_get_by_id_noref(pdata->host1x, id);
	if (WARN_ON(!sp))
		return;

	host1x_syncpt_put(sp);
}
EXPORT_SYMBOL(nvhost_syncpt_put_ref_ext);

int nvhost_syncpt_read_ext_check(struct platform_device *pdev, u32 id, u32 *val)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct host1x_syncpt *sp;

	sp = host1x_syncpt_get_by_id_noref(pdata->host1x, id);
	if (!sp)
		return -EINVAL;

	*val = host1x_syncpt_read(sp);
	return 0;
}
EXPORT_SYMBOL(nvhost_syncpt_read_ext_check);

static int nvhost_syncpt_get_aperture(struct device_node *np, u64 *base,
				      size_t *size)
{
	if (of_device_is_compatible(np, "nvidia,tegra194-host1x")) {
		*base = TEGRA194_SYNCPT_SHIM_BASE;
		*size = TEGRA194_SYNCPT_SHIM_SIZE;
		return 0;
	}

	if (of_device_is_compatible(np, "nvidia,tegra234-host1x")) {
		*base = TEGRA234_SYNCPT_SHIM_BASE;
		*size = TEGRA234_SYNCPT_SHIM_SIZE;
		return 0;
	}

	return -ENODEV;
}

static int nvhost_syncpt_get_page_size(struct device_node *np, uint32_t *size)
{
	if (of_device_is_compatible(np, "nvidia,tegra194-host1x")) {
		*size = TEGRA194_SYNCPT_PAGE_SIZE;
		return 0;
	}

	if (of_device_is_compatible(np, "nvidia,tegra234-host1x")) {
		*size = TEGRA234_SYNCPT_PAGE_SIZE;
		return 0;
	}

	return -ENODEV;
}

u32 nvhost_syncpt_unit_interface_get_byte_offset_ext(struct platform_device *pdev,
						     u32 syncpt_id)
{
	uint32_t size;
	int err;

	err = nvhost_syncpt_get_page_size(pdev->dev.of_node, &size);
	if (WARN_ON(err < 0))
		return 0;

	return syncpt_id * size;
}
EXPORT_SYMBOL(nvhost_syncpt_unit_interface_get_byte_offset_ext);

u32 nvhost_syncpt_unit_interface_get_byte_offset(u32 syncpt_id)
{
	struct platform_device *host1x_pdev;

	host1x_pdev = nvhost_get_default_device();
	if (WARN_ON(!host1x_pdev))
		return 0;

	return nvhost_syncpt_unit_interface_get_byte_offset_ext(host1x_pdev,
								syncpt_id);
}
EXPORT_SYMBOL(nvhost_syncpt_unit_interface_get_byte_offset);

int nvhost_syncpt_unit_interface_get_aperture(struct platform_device *pdev,
					      u64 *base, size_t *size)
{
	return nvhost_syncpt_get_aperture(pdev->dev.of_node, base, size);
}
EXPORT_SYMBOL(nvhost_syncpt_unit_interface_get_aperture);

int nvhost_syncpt_unit_interface_init(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_syncpt_interface *syncpt_if;
	u64 base;
	int err;

	syncpt_if = devm_kzalloc(&pdev->dev, sizeof(*syncpt_if), GFP_KERNEL);
	if (!syncpt_if)
		return -ENOMEM;

	err = nvhost_syncpt_get_aperture(pdev->dev.parent->of_node, &base,
					 &syncpt_if->size);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to get syncpt aperture\n");
		return err;
	}

	err = nvhost_syncpt_get_page_size(pdev->dev.parent->of_node,
					  &syncpt_if->page_size);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to get syncpt page size\n");
		return err;
	}

	/* If IOMMU is enabled, map it into the device memory */
	if (iommu_get_domain_for_dev(&pdev->dev)) {
		syncpt_if->base = dma_map_resource(&pdev->dev, base,
						   syncpt_if->size,
						   DMA_BIDIRECTIONAL,
						   DMA_ATTR_SKIP_CPU_SYNC);
		if (dma_mapping_error(&pdev->dev, syncpt_if->base))
			return -ENOMEM;
	} else {
		syncpt_if->base = base;
	}

	pdata->syncpt_unit_interface = syncpt_if;

	dev_info(&pdev->dev,
		 "syncpt_unit_base %llx syncpt_unit_size %zx size %x\n",
		 base, syncpt_if->size, syncpt_if->page_size);

	return 0;
}
EXPORT_SYMBOL(nvhost_syncpt_unit_interface_init);

dma_addr_t nvhost_syncpt_address(struct platform_device *pdev, u32 id)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_syncpt_interface *syncpt_if = pdata->syncpt_unit_interface;

	return syncpt_if->base + syncpt_if->page_size * id;
}
EXPORT_SYMBOL(nvhost_syncpt_address);

void nvhost_module_deinit(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
}
EXPORT_SYMBOL(nvhost_module_deinit);

int nvhost_module_init(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	unsigned int i;
	int err;

	err = devm_clk_bulk_get_all(&pdev->dev, &pdata->clks);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to get clocks %d\n", err);
		return err;
	}
	pdata->num_clks = err;

	for (i = 0; i < pdata->num_clks; i++) {
		err = clk_set_rate(pdata->clks[i].clk, ULONG_MAX);
		if (err < 0) {
			dev_err(&pdev->dev, "failed to set clock rate!\n");
			return err;
		}
	}

	pdata->reset_control = devm_reset_control_get_exclusive_released(
					&pdev->dev, NULL);
	if (IS_ERR(pdata->reset_control)) {
		dev_err(&pdev->dev, "failed to get reset\n");
		return PTR_ERR(pdata->reset_control);
	}

	reset_control_acquire(pdata->reset_control);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to acquire reset: %d\n", err);
		return err;
	}

	err = clk_bulk_prepare_enable(pdata->num_clks, pdata->clks);
	if (err < 0) {
		reset_control_release(pdata->reset_control);
		dev_err(&pdev->dev, "failed to enabled clocks: %d\n", err);
		return err;
	}

	reset_control_reset(pdata->reset_control);
	clk_bulk_disable_unprepare(pdata->num_clks, pdata->clks);
	reset_control_release(pdata->reset_control);

	if (pdata->autosuspend_delay) {
		pm_runtime_set_autosuspend_delay(&pdev->dev,
			pdata->autosuspend_delay);
		pm_runtime_use_autosuspend(&pdev->dev);
	}

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev))
		return -EOPNOTSUPP;

	return 0;
}
EXPORT_SYMBOL(nvhost_module_init);

int nvhost_module_busy(struct platform_device *dev)
{
	int err;

	err = pm_runtime_get_sync(&dev->dev);
	if (err < 0) {
		pm_runtime_put_noidle(&dev->dev);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(nvhost_module_busy);

void nvhost_module_idle_mult(struct platform_device *pdev, int refs)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);

	while (refs--) {
		pm_runtime_mark_last_busy(&pdev->dev);
		if (pdata->autosuspend_delay)
			pm_runtime_put_autosuspend(&pdev->dev);
		else
			pm_runtime_put(&pdev->dev);
	}
}
EXPORT_SYMBOL(nvhost_module_idle_mult);

inline void nvhost_module_idle(struct platform_device *pdev)
{
	nvhost_module_idle_mult(pdev, 1);
}
EXPORT_SYMBOL(nvhost_module_idle);

static int nvhost_module_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct nvhost_device_data *pdata = dev_get_drvdata(dev);
	int err;

	err = clk_bulk_prepare_enable(pdata->num_clks, pdata->clks);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to enabled clocks: %d\n", err);
		return err;
	}

	return err;
}

static int nvhost_module_runtime_suspend(struct device *dev)
{
	struct nvhost_device_data *pdata = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(pdata->num_clks, pdata->clks);

	return 0;
}

const struct dev_pm_ops nvhost_module_pm_ops = {
	SET_RUNTIME_PM_OPS(nvhost_module_runtime_suspend,
			   nvhost_module_runtime_resume, NULL)
};
EXPORT_SYMBOL(nvhost_module_pm_ops);

static struct platform_driver nvhost_driver = {
	.driver = {
		.name = "host1x-nvhost",
	},
};

module_platform_driver(nvhost_driver);
MODULE_LICENSE("GPL v2");

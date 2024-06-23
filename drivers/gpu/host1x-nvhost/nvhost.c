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

static struct device *nvhost_client_device_create(struct platform_device *pdev,
						  struct cdev *cdev,
						  const char *cdev_name,
						  dev_t devno,
						  const struct file_operations *ops)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct device *dev;
	int err;

#if defined(NV_CLASS_CREATE_HAS_NO_OWNER_ARG) /* Linux v6.4 */
	pdata->nvhost_class = class_create(pdev->dev.of_node->name);
#else
	pdata->nvhost_class = class_create(THIS_MODULE, pdev->dev.of_node->name);
#endif
	if (IS_ERR(pdata->nvhost_class)) {
		dev_err(&pdev->dev, "failed to create class\n");
		return ERR_CAST(pdata->nvhost_class);
	}

	cdev_init(cdev, ops);
	cdev->owner = THIS_MODULE;

	err = cdev_add(cdev, devno, 1);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to add cdev\n");
		class_destroy(pdata->nvhost_class);
		return ERR_PTR(err);
	}

	dev = device_create(pdata->nvhost_class, &pdev->dev, devno, NULL,
			    (pdev->id <= 0) ? "nvhost-%s%s" : "nvhost-%s%s.%d",
			    cdev_name, pdev->dev.of_node->name, pdev->id);

	if (IS_ERR(dev)) {
		dev_err(&pdev->dev, "failed to create %s device\n", cdev_name);
		class_destroy(pdata->nvhost_class);
		cdev_del(cdev);
	}

	return dev;
}

int nvhost_client_device_get_resources(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);

	pdata->host1x = nvhost_get_host1x(pdev);
	if (!pdata->host1x) {
		dev_warn(&pdev->dev, "No platform data for host1x!\n");
		return -ENODEV;
	}

	return 0;
}
EXPORT_SYMBOL(nvhost_client_device_get_resources);

int nvhost_client_device_init(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	dev_t devno;
	int err;

	err = alloc_chrdev_region(&devno, 0, NVHOST_NUM_CDEV, "nvhost");
	if (err < 0) {
		dev_err(&pdev->dev, "failed to reserve chrdev region\n");
		return err;
	}

	pdata->ctrl_node = nvhost_client_device_create(pdev, &pdata->ctrl_cdev,
						       "ctrl-", devno,
						       pdata->ctrl_ops);
	if (IS_ERR(pdata->ctrl_node))
		return PTR_ERR(pdata->ctrl_node);

	pdata->cdev_region = devno;

	return 0;
}
EXPORT_SYMBOL(nvhost_client_device_init);

int nvhost_client_device_release(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);

	if (!IS_ERR_OR_NULL(pdata->ctrl_node)) {
		device_destroy(pdata->nvhost_class, pdata->ctrl_cdev.dev);
		cdev_del(&pdata->ctrl_cdev);
		class_destroy(pdata->nvhost_class);
	}

	unregister_chrdev_region(pdata->cdev_region, NVHOST_NUM_CDEV);

	return 0;
}
EXPORT_SYMBOL(nvhost_client_device_release);

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
	int err;

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

	reset_control_reset(pdata->reset_control);
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

static struct platform_driver nvhost_driver = {
	.driver = {
		.name = "host1x-nvhost",
	},
};

module_platform_driver(nvhost_driver);
MODULE_LICENSE("GPL v2");

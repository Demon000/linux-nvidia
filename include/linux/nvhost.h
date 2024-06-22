/* SPDX-License-Identifier: GPL-2.0-only */
/* SPDX-FileCopyrightText: Copyright (c) 2009-2024 NVIDIA CORPORATION & AFFILIATES. All Rights Reserved. */

#ifndef __LINUX_NVHOST_H
#define __LINUX_NVHOST_H

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/host1x.h>
#include <linux/platform_device.h>

struct nvhost_ctrl_sync_fence_info;
struct nvhost_fence;
struct nvhost_job;

#define NVHOST_MODULE_MAX_IORESOURCE_MEM	5

struct nvhost_notification {
	struct {			/* 0000- */
		__u32 nanoseconds[2];	/* nanoseconds since Jan. 1, 1970 */
	} time_stamp;			/* -0007 */
	__u32 info32;	/* info returned depends on method 0008-000b */
	__u16 info16;	/* info returned depends on method 000c-000d */
	__u16 status;	/* user sets bit 15, NV sets status 000e-000f */
};

struct nvhost_gating_register {
	u64 addr;
	u32 prod;
	u32 disable;
};

enum tegra_emc_request_type {
	TEGRA_SET_EMC_FLOOR,		/* lower bound */
};

struct nvhost_clock {
	char *name;
	unsigned long default_rate;
	enum tegra_emc_request_type request_type;
	bool disable_scaling;
	unsigned long devfreq_rate;
};

struct nvhost_vm_hwid {
	u64 addr;
	bool dynamic;
	u32 shift;
};

/*
 * Defines HW and SW class identifiers.
 *
 * This is module ID mapping between userspace and kernelspace.
 * The values of enum entries' are referred from NvRmModuleID enum defined
 * in below userspace file:
 * $TOP/vendor/nvidia/tegra/core/include/nvrm_module.h
 * Please make sure each entry below has same value as set in above file.
 */
enum nvhost_module_identifier {
	/* Specifies external memory (DDR RAM, etc) */
	NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER = 75,
};

struct nvhost_device_data {
	/* interrupt ISR routine for falcon based engines */
	int irq;

	u32		class;		/* Device class */

	int		autosuspend_delay;/* Delay before power gated */

	int		num_clks;	/* Number of clocks opened for dev */
	struct clk_bulk_data *clks;
	struct mutex	lock;		/* Power management lock */

	int		num_channels;	/* Max num of channel supported */
	dev_t cdev_region;

	/* device node for ctrl block */
	struct class *nvhost_class;
	struct device *ctrl_node;
	struct cdev ctrl_cdev;
	const struct file_operations *ctrl_ops;    /* ctrl ops for the module */

	struct dentry *debugfs;		/* debugfs directory */

	void *private_data;		/* private platform data */
	struct platform_device *pdev;	/* owner platform_device */
	struct host1x *host1x;		/* host1x device */

	/* Information related to engine-side synchronization */
	void *syncpt_unit_interface;

	/* reset control for this device */
	struct reset_control *reset_control;
};

/* public api to return platform_device ptr to the default host1x instance */
struct platform_device *nvhost_get_default_device(void);

/* common runtime pm and power domain APIs */
int nvhost_module_init(struct platform_device *ndev);
void nvhost_module_deinit(struct platform_device *dev);
void nvhost_module_idle(struct platform_device *dev);
void nvhost_module_idle_mult(struct platform_device *pdev, int refs);
int nvhost_module_busy(struct platform_device *dev);
extern const struct dev_pm_ops nvhost_module_pm_ops;

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

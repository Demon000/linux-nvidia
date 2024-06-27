/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 */

#ifndef _LINUX_TEGRA_CAMERA_RTCPU_H_
#define _LINUX_TEGRA_CAMERA_RTCPU_H_

#include <linux/types.h>

struct device;

int tegra_camrtc_iovm_setup(struct device *dev, dma_addr_t iova);
int tegra_camrtc_reboot(struct device *dev);

#endif

/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * NVIDIA Tegra CSI Device Header
 *
 * Copyright (c) 2015-2022, NVIDIA CORPORATION.  All rights reserved.
 */

#ifndef __CSI_H_
#define __CSI_H_

#include <linux/minmax.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/camera_common.h>
#include <media/csi_common.h>
#include <linux/platform_device.h>

#define MAX_CSI_BLOCK_LANES					4

struct tegra_csi_port {
	void __iomem *pixel_parser;
	void __iomem *cil;

	u32 csi_port;
	u32 stream_id;
	u32 virtual_channel_id;

	/* One pair of sink/source pad has one format */
	struct v4l2_mbus_framefmt format;
	const struct tegra_video_format *core_format;
	unsigned int lanes;
	unsigned int framerate;
	unsigned int h_blank;
	unsigned int v_blank;
};

struct tegra_csi_device;
struct tegra_csi_channel;

struct tegra_csi_fops {
	int (*csi_power_on)(struct tegra_csi_device *csi);
	int (*csi_power_off)(struct tegra_csi_device *csi);
	int (*csi_start_streaming)(struct tegra_csi_channel *chan,
		int port_idx);
	void (*csi_stop_streaming)(struct tegra_csi_channel *chan,
		int port_idx);
	int (*hw_init)(struct tegra_csi_device *csi);
};

struct tegra_csi_device {
	struct device *dev;
	struct platform_device *pdev;
	char devname[32];
	void __iomem *iomem_base;
	void __iomem *iomem[3];
	struct clk *plld_dsi;
	struct clk *plld;

	struct camera_common_data s_data[6];
	struct tegra_csi_port *ports;
	struct media_pad *pads;

	unsigned int clk_freq;
	int num_ports;
	int num_channels;
	struct list_head csi_chans;
	const struct tegra_csi_fops *fops;
	atomic_t power_ref;

	struct dentry *debugdir;
	struct mutex source_update;
	int sensor_active;
};

/*
 * subdev: channel subdev
 * numports: Number of CSI ports in use for this channel
 * numlanes: Number of CIL lanes in use
 */
struct tegra_csi_channel {
	struct list_head list;
	struct v4l2_subdev subdev;
	struct media_pad *pads;
	struct media_pipeline pipe;
	struct v4l2_subdev *sensor_sd;

	struct tegra_csi_device *csi;
	struct tegra_csi_port *ports;
	unsigned char port[TEGRA_CSI_BLOCKS];
	struct mutex format_lock;
	unsigned int numports;
	unsigned int numlanes;
	struct camera_common_data *s_data;
	unsigned int id;
	atomic_t is_streaming;

	struct device_node *of_node;
};

static inline struct tegra_csi_channel *to_csi_chan(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct tegra_csi_channel, subdev);
}

static inline struct tegra_csi_device *to_csi(struct v4l2_subdev *subdev)
{
	struct tegra_csi_channel *chan = to_csi_chan(subdev);

	return chan->csi;
}

u32 read_phy_mode_from_dt(struct tegra_csi_channel *chan);
u64 read_mipi_clk_from_dt(struct tegra_csi_channel *chan);
int tegra_csi_media_controller_init(struct tegra_csi_device *csi,
				struct platform_device *pdev);
int tegra_csi_media_controller_remove(struct tegra_csi_device *csi);

#endif

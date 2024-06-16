// SPDX-License-Identifier: GPL-2.0
/*
 * NVIDIA Tegra CSI Device
 *
 * Copyright (c) 2015-2024, NVIDIA CORPORATION.  All rights reserved.
 */

#include <nvidia/conftest.h>

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/property.h>
#include <linux/nospec.h>

#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/camera_common.h>
#include <media/mc_common.h>
#include <media/csi.h>
#include <trace/events/camera_common.h>
#include <linux/nvhost.h>
#include <asm/barrier.h>
#include "soc/tegra/camrtc-capture.h"
#include <uapi/linux/nvhost_nvcsi_ioctl.h>
#include "nvcsi/deskew.h"

/*
 * deskew should be run when the sensor data rate is >= 1.5 gbps
 * data is sent on both rising/falling edges of clock, so /2
 */
#define CLK_HZ_FOR_DESKEW ((1500*1000*1000)/2)

static struct tegra_csi_device *mc_csi;

static struct sensor_mode_properties*
read_mode_from_dt(struct camera_common_data *s_data)
{
	struct sensor_mode_properties *mode = NULL;

	if (s_data) {
		int idx = s_data->mode_prop_idx;

		if (idx < s_data->sensor_props.num_modes)
			mode = &s_data->sensor_props.sensor_modes[idx];
	}

	return mode;
}

u32 read_phy_mode_from_dt(struct tegra_csi_channel *chan)
{
	struct camera_common_data *s_data = chan->s_data;
	struct sensor_mode_properties *mode = read_mode_from_dt(s_data);
	struct device *dev = chan->csi->dev;
	u32 phy_mode = 0;

	if (mode) {
		dev_dbg(dev, "settle time reading from props\n");
		phy_mode = mode->signal_properties.phy_mode;
	} else {
		dev_dbg(dev, "phy mode unavailable in props, use default\n");
		phy_mode = CSI_PHY_MODE_DPHY;
	}

	return phy_mode;
}

u64 read_mipi_clk_from_dt(struct tegra_csi_channel *chan)
{
	struct sensor_signal_properties *sig_props;
	struct sensor_properties *props;
	u64 mipi_clk = 0;
	int mode_idx;

	if (chan && chan->s_data) {
		mode_idx = chan->s_data->mode_prop_idx;
		props =  &chan->s_data->sensor_props;
		sig_props = &props->sensor_modes[mode_idx].signal_properties;
		mipi_clk = sig_props->mipi_clock.val;
	}

	return mipi_clk;
}

static int tegra_csi_power(struct tegra_csi_device *csi,
			struct tegra_csi_channel *chan, int enable)
{
	int err = 0;

	trace_csi_s_power("enable", enable);
	if (enable) {
		err = csi->fops->csi_power_on(csi);
		if (!err)
			atomic_inc(&csi->power_ref);
	} else {
		err = csi->fops->csi_power_off(csi);
		if (!err)
			atomic_dec(&csi->power_ref);
	}
	return err;
}

static int tegra_csi_s_power(struct v4l2_subdev *subdev, int enable)
{
	int err = 0;
	struct tegra_csi_device *csi = to_csi(subdev);
	struct tegra_csi_channel *chan = to_csi_chan(subdev);

	err = tegra_csi_power(csi, chan, enable);

	return err;
}

/*
 * -----------------------------------------------------------------------------
 * CSI Subdevice Video Operations
 * -----------------------------------------------------------------------------
 */

static int tegra_csi_start_streaming(struct tegra_csi_channel *chan, int port_idx)
{
	struct tegra_csi_device *csi = chan->csi;

	return csi->fops->csi_start_streaming(chan, port_idx);
}

static void tegra_csi_stop_streaming(struct tegra_csi_channel *chan, int port_idx)
{
	struct tegra_csi_device *csi = chan->csi;

	csi->fops->csi_stop_streaming(chan, port_idx);
}

static int update_video_source(struct tegra_csi_device *csi, int on)
{
	mutex_lock(&csi->source_update);

	if (!on) {
		csi->sensor_active--;
		WARN_ON(csi->sensor_active < 0);
		goto stream_okay;
	}

	if (csi->sensor_active >= 0) {
		csi->sensor_active++;
		goto stream_okay;
	}

	mutex_unlock(&csi->source_update);
	dev_err(csi->dev, "Request rejected for new sensor stream\n");
	dev_err(csi->dev, "Active sensor streams %d\n", csi->sensor_active);

	return -EINVAL;

stream_okay:
	mutex_unlock(&csi->source_update);

	return 0;
}

static void deskew_setup(struct tegra_csi_channel *chan,
				struct nvcsi_deskew_context *deskew_ctx)
{
	struct sensor_signal_properties *sig_props;
	struct sensor_properties *props;
	int i;
	int mode_idx = -1;
	u64 pix_clk_hz = 0;
	u32 deskew_enable = 0;
	unsigned int csi_lane_start = 0;
	unsigned int csi_port, csi_lanes;

	if (chan->s_data == NULL)
		return;

	mode_idx = chan->s_data->mode_prop_idx;
	props =  &chan->s_data->sensor_props;
	sig_props = &props->sensor_modes[mode_idx].signal_properties;
	if (sig_props->serdes_pixel_clock.val != 0ULL)
		pix_clk_hz = sig_props->serdes_pixel_clock.val;
	else
		pix_clk_hz = sig_props->pixel_clock.val;
	deskew_enable = sig_props->deskew_initial_enable;

	if (pix_clk_hz >= CLK_HZ_FOR_DESKEW && deskew_enable) {
		csi_port = chan->ports[0].csi_port;
		csi_lanes = chan->ports[0].lanes;
		switch (csi_port) {
		case NVCSI_PORT_A:
			csi_lane_start = NVCSI_PHY_0_NVCSI_CIL_A_IO0;
			break;
		case NVCSI_PORT_B:
			csi_lane_start = NVCSI_PHY_0_NVCSI_CIL_B_IO0;
			break;
		case NVCSI_PORT_C:
			csi_lane_start = NVCSI_PHY_1_NVCSI_CIL_A_IO0;
			break;
		case NVCSI_PORT_D:
			csi_lane_start = NVCSI_PHY_1_NVCSI_CIL_B_IO0;
			break;
		case NVCSI_PORT_E:
			csi_lane_start = NVCSI_PHY_2_NVCSI_CIL_A_IO0;
			break;
		case NVCSI_PORT_F:
			csi_lane_start = NVCSI_PHY_2_NVCSI_CIL_B_IO0;
			break;
		case NVCSI_PORT_G:
			csi_lane_start = NVCSI_PHY_3_NVCSI_CIL_A_IO0;
			break;
		case NVCSI_PORT_H:
			csi_lane_start = NVCSI_PHY_3_NVCSI_CIL_B_IO0;
			break;
		default:
			break;
		}
		deskew_ctx->deskew_lanes = 0;
		for (i = 0; i < csi_lanes; ++i)
			deskew_ctx->deskew_lanes |= csi_lane_start << i;
		nvcsi_deskew_setup(deskew_ctx);
	}

}

static int tegra_csi_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct tegra_csi_device *csi;
	struct tegra_csi_channel *chan = to_csi_chan(subdev);
	struct tegra_channel *tegra_chan = v4l2_get_subdev_hostdata(subdev);
	int i, ret = 0;

	if (atomic_read(&chan->is_streaming) == enable)
		return 0;
	trace_csi_s_stream("enable", enable);
	csi = to_csi(subdev);
	if (!csi)
		return -EINVAL;
	ret = update_video_source(csi, enable);
	if (ret)
		return ret;

	/* if it is bypass and real sensor, return here
	 * else let tegra_csi_start_streaming handle it
	 * depending on bypass flag
	 */
	if (tegra_chan->bypass) {
		atomic_set(&chan->is_streaming, enable);
		return 0;
	}
	for (i = 0; i < tegra_chan->valid_ports; i++) {
		if (enable) {
				ret = tegra_csi_start_streaming(chan, i);
				if (ret)
					goto start_fail;
				if (!tegra_chan->bypass)
					deskew_setup(chan,
						tegra_chan->deskew_ctx);
		} else
			tegra_csi_stop_streaming(chan, i);
	}
	atomic_set(&chan->is_streaming, enable);
	return ret;
start_fail:
	update_video_source(csi, 0);
	/* Reverse sequence to stop streaming on all valid_ports
	 * i is the current failing port, need to stop ports 0 ~ (i-1)
	 */
	for (i = i - 1; i >= 0; i--)
		tegra_csi_stop_streaming(chan, i);
	return ret;
}

static int csi_is_power_on(struct tegra_csi_device *csi)
{
	return atomic_read(&csi->power_ref);
}
static int tegra_csi_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct tegra_csi_device *csi = to_csi(sd);

	/* Set status to 0 if power is on
	 * Set status to 1 if power is off
	 */
	*status = !csi_is_power_on(csi);

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Operations
 */
static struct v4l2_subdev_video_ops tegra_csi_video_ops = {
	.s_stream	= tegra_csi_s_stream,
	.g_input_status = tegra_csi_g_input_status,
};

static struct v4l2_subdev_core_ops tegra_csi_core_ops = {
	.s_power	= tegra_csi_s_power,
};

static struct v4l2_subdev_ops tegra_csi_ops = {
	.core	= &tegra_csi_core_ops,
	.video  = &tegra_csi_video_ops,
};

/* -----------------------------------------------------------------------------
 * Media Operations
 */

static const struct media_entity_operations tegra_csi_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* -----------------------------------------------------------------------------
 * Platform Device Driver
 */

static int tegra_csi_get_port_info(struct tegra_csi_channel *chan,
				struct device_node *node, unsigned int index)
{
	struct device_node *ep = NULL;
	struct device_node *ports;
	struct device_node *port;
	struct device_node *chan_dt;

	int value = 0xFFFF;
	int ret = 0;
	u32 i = 0;

	memset(&chan->port[0], INVALID_CSI_PORT, TEGRA_CSI_BLOCKS);
	for_each_child_of_node(node, chan_dt) {
		if (!chan_dt->name || of_node_cmp(chan_dt->name, "channel"))
			continue;
		ret = of_property_read_u32(chan_dt, "reg", &value);
		if (ret < 0)
			return -EINVAL;
		chan->of_node = chan_dt;
		if (value == index)
			break;
	}

	chan->subdev.fwnode = of_fwnode_handle(chan_dt);
	ports = of_get_child_by_name(chan_dt, "ports");
	if (ports == NULL)
		return -EINVAL;

	for_each_child_of_node(ports, port) {
		if (!port->name || of_node_cmp(port->name, "port"))
			continue;
		ret = of_property_read_u32(port, "reg", &value);
		if (ret < 0)
			continue;
		if (value != 0)
			continue;
		for_each_child_of_node(port, ep) {
			if (!ep->name || of_node_cmp(ep->name, "endpoint"))
				continue;
			ret = of_property_read_u32(ep, "port-index", &value);
			if (ret < 0)
				dev_err(chan->csi->dev, "No port index info\n");
			chan->port[0] = value;

			ret = of_property_read_u32(ep, "bus-width", &value);
			if (ret < 0)
				dev_err(chan->csi->dev, "No bus width info\n");
			chan->numlanes = value;
			if (value > 12) {
				dev_err(chan->csi->dev, "Invalid num lanes\n");
				return -EINVAL;
			}
			/*
			 * for numlanes greater than 4 multiple CSI bricks
			 * are needed to capture the image, the logic below
			 * checks for numlanes > 4 and add a new CSI brick
			 * as a valid port. Loops around the three CSI
			 * bricks to add as many ports necessary.
			 */
			value -= 4;
			for (i = 1; value > 0 && i < TEGRA_CSI_BLOCKS; i++, value -= 4) {
				int next_port = chan->port[i-1] + 2;

				next_port = (next_port % (NVCSI_PORT_H + 1));
				chan->port[i] = next_port;
			}
		}
	}

	for (i = 0; csi_port_is_valid(chan->port[i]); i++)
		chan->numports++;

	return 0;
}

static int tegra_csi_init(struct tegra_csi_device *csi,
		struct platform_device *pdev)
{
	int err = 0;

	csi->dev = &pdev->dev;
	csi->fops->hw_init(csi);

	return err;
}

static int tegra_csi_channel_init_one(struct tegra_csi_channel *chan)
{
	struct v4l2_subdev *sd;
	int numlanes = 0;
	struct tegra_csi_device *csi = chan->csi;
	int i, ret;
	const struct tegra_video_format *vf;

	vf = tegra_core_get_default_format();
	if (vf == NULL) {
		dev_err(csi->dev, "Fail to find tegra video fmt");
		return -EINVAL;
	}

	atomic_set(&chan->is_streaming, 0);
	sd = &chan->subdev;
	/* Initialize V4L2 subdevice and media entity */
	v4l2_subdev_init(sd, &tegra_csi_ops);
	sd->dev = chan->csi->dev;
	v4l2_set_subdevdata(sd, csi);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.ops = &tegra_csi_media_ops;
	chan->ports = devm_kzalloc(csi->dev,
			chan->numports * sizeof(struct tegra_csi_port),
			GFP_KERNEL);
	if (!chan->ports)
		return -ENOMEM;

	/* Initialize the default format */
	for (i = 0; i < chan->numports; i++) {
		chan->ports[i].format.code = vf->vf_code;
		chan->ports[i].format.field = V4L2_FIELD_NONE;
		chan->ports[i].format.colorspace = V4L2_COLORSPACE_SRGB;
		chan->ports[i].format.width = TEGRA_DEF_WIDTH;
		chan->ports[i].format.height = TEGRA_DEF_HEIGHT;
		chan->ports[i].core_format = vf;
	}

	chan->pads = devm_kzalloc(csi->dev, 2 * sizeof(*chan->pads),
		GFP_KERNEL);
	if (!chan->pads)
		return -ENOMEM;
	chan->pads[0].flags = MEDIA_PAD_FL_SINK;
	chan->pads[1].flags = MEDIA_PAD_FL_SOURCE;

	ret = snprintf(sd->name, sizeof(sd->name), "%s-%d",
			 (strlen(csi->devname) == 0 ?
			  dev_name(csi->dev) : csi->devname),
			  (chan->id - csi->num_channels));
	if (ret < 0)
		return -EINVAL;

	/* Initialize media entity */
	ret = tegra_media_entity_init(&sd->entity, 2,
				chan->pads, true, false);
	if (ret < 0)
		return ret;

	for (i = 0; i < chan->numports; i++) {
		numlanes = chan->numlanes - (i * MAX_CSI_BLOCK_LANES);
		WARN_ON(numlanes < 0);
		numlanes = numlanes > MAX_CSI_BLOCK_LANES ?
			MAX_CSI_BLOCK_LANES : numlanes;
		chan->ports[i].lanes = numlanes;
		chan->ports[i].csi_port = chan->port[i];
	}

#if defined(CONFIG_V4L2_ASYNC)
	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		dev_err(csi->dev, "failed to register subdev\n");
		media_entity_cleanup(&sd->entity);
	}

	return ret;
#else
	dev_err(csi->dev, "CONFIG_V4L2_ASYNC not enabled!\n");

	return -ENOTSUPP;
#endif
}

static int tegra_csi_channels_init(struct tegra_csi_device *csi)
{
	int ret;
	struct tegra_csi_channel *it;

	list_for_each_entry(it, &csi->csi_chans, list) {
		ret = tegra_csi_channel_init_one(it);
		if (ret)
			return ret;
	}

	return 0;
}

static int csi_parse_dt(struct tegra_csi_device *csi,
			struct platform_device *pdev)
{
	int err = 0, i;
	int num_channels = 0;
	struct device_node *node = pdev->dev.of_node;
	struct tegra_csi_channel *item;

	if (strncmp(node->name, "nvcsi", 5)) {
		node = of_find_node_by_name(node, "nvcsi");
		strncpy(csi->devname, "nvcsi", 6);
	}

	if (node) {
		err = of_property_read_u32(node, "num-channels", &num_channels);
		if (err) {
			dev_dbg(csi->dev, " Failed to find num of channels, set to 0\n");
			num_channels = 0;
		}
	}

	csi->num_channels = num_channels;
	for (i = 0; i < num_channels; i++) {
		item = devm_kzalloc(csi->dev, sizeof(*item), GFP_KERNEL);
		if (!item)
			return -ENOMEM;
		list_add_tail(&item->list, &csi->csi_chans);
		item->csi = csi;
		item->id = i;
		err = tegra_csi_get_port_info(item, node, item->id);
		if (err)
			return err;
	}

	return 0;
}

int tegra_csi_media_controller_init(struct tegra_csi_device *csi,
				    struct platform_device *pdev)
{
	int ret;

	if (!csi)
		return -EINVAL;
	mc_csi = csi;

	csi->dev = &pdev->dev;
	csi->pdev = pdev;
	csi->sensor_active = 0;
	atomic_set(&csi->power_ref, 0);
	mutex_init(&csi->source_update);
	INIT_LIST_HEAD(&csi->csi_chans);
	ret = csi_parse_dt(csi, pdev);
	if (ret < 0)
		return ret;

	/*
	 * if there is no csi channels listed in DT,
	 * no need to init the channel and graph
	 */
	if (csi->num_channels > 0) {
		ret = tegra_csi_channels_init(csi);
		if (ret < 0)
			dev_err(&pdev->dev, "Failed to init csi channel\n");
	}

	ret = tegra_csi_init(csi, pdev);
	if (ret < 0)
		dev_err(&pdev->dev, "Failed to init csi property,clks\n");

	return 0;
}
EXPORT_SYMBOL(tegra_csi_media_controller_init);

int tegra_csi_media_controller_remove(struct tegra_csi_device *csi)
{
	struct tegra_csi_channel *chan;
	struct v4l2_subdev *sd;

	list_for_each_entry(chan, &csi->csi_chans, list) {
		sd = &chan->subdev;
#if defined(CONFIG_V4L2_ASYNC)
		v4l2_async_unregister_subdev(sd);
#endif
		media_entity_cleanup(&sd->entity);
	}
	return 0;
}
EXPORT_SYMBOL(tegra_csi_media_controller_remove);


void tegra_csi_channel_sd_set_sensor_sd(struct v4l2_subdev *subdev,
					struct v4l2_subdev *sensor_sd)
{
	struct tegra_csi_channel *chan = to_csi_chan(subdev);

	chan->s_data = to_camera_common_data(sensor_sd->dev);
	chan->sensor_sd = sensor_sd;
}
EXPORT_SYMBOL(tegra_csi_channel_sd_set_sensor_sd);


uint32_t tegra_csi_channel_sd_get_vi_csi_port(struct v4l2_subdev *subdev,
					      uint32_t vi_port)
{
	struct tegra_csi_channel *chan = to_csi_chan(subdev);

	return chan->ports[vi_port].csi_port;
}
EXPORT_SYMBOL(tegra_csi_channel_sd_get_vi_csi_port);

// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2023, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 *
 * Tegra CSI5 device common APIs
 */

#include <media/csi.h>
#include <media/mc_common.h>
#include "csi5_fops.h"
#include <linux/nospec.h>
#include <linux/nvhost.h>
#include <linux/tegra-capture-ivc.h>
#include "soc/tegra/camrtc-capture-messages.h"
#include <media/fusa-capture/capture-vi.h>

/* Referred from capture-scheduler.c defined in rtcpu-fw */
#define NUM_CAPTURE_CHANNELS 64

/* Temporary ids for the clients whose channel-id is not yet allocated */
#define NUM_CAPTURE_TRANSACTION_IDS 64

#define TOTAL_CHANNELS (NUM_CAPTURE_CHANNELS + NUM_CAPTURE_TRANSACTION_IDS)

static inline u32 csi5_port_to_stream(u32 csi_port)
{
	return (csi_port < NVCSI_PORT_E) ?
		csi_port : (((csi_port - NVCSI_PORT_E) >> 1U) + NVCSI_PORT_E);
}

static int csi5_power_on(struct tegra_csi_device *csi)
{
	int err = 0;

	dev_dbg(csi->dev, "%s\n", __func__);

	err = nvhost_module_busy(csi->pdev);
	if (err)
		dev_err(csi->dev, "%s:cannot enable csi\n", __func__);

	return err;
}

static int csi5_power_off(struct tegra_csi_device *csi)
{
	dev_dbg(csi->dev, "%s\n", __func__);

	nvhost_module_idle(csi->pdev);

	return 0;
}

static int verify_capture_control_response(const uint32_t result)
{
	int err = 0;

	switch (result) {
	case CAPTURE_OK:
	{
		err = 0;
		break;
	}
	case CAPTURE_ERROR_INVALID_PARAMETER:
	{
		err = -EINVAL;
		break;
	}
	case CAPTURE_ERROR_NO_MEMORY:
	{
		err = -ENOMEM;
		break;
	}
	case CAPTURE_ERROR_BUSY:
	{
		err = -EBUSY;
		break;
	}
	case CAPTURE_ERROR_NOT_SUPPORTED:
	case CAPTURE_ERROR_NOT_INITIALIZED:
	{
		err = -EPERM;
		break;
	}
	case CAPTURE_ERROR_OVERFLOW:
	{
		err = -EOVERFLOW;
		break;
	}
	case CAPTURE_ERROR_NO_RESOURCES:
	{
		err = -ENODEV;
		break;
	}
	default:
	{
		err = -EINVAL;
		break;
	}
	}

	return err;
}

static int csi5_send_control_message(
	struct tegra_vi_channel *chan,
	struct CAPTURE_CONTROL_MSG *msg,
	uint32_t *result)
{
	int err = 0;
	struct vi_capture_control_msg vi_msg;
	(void) memset(&vi_msg, 0, sizeof(vi_msg));
	vi_msg.ptr = (uint64_t)msg;
	vi_msg.size = sizeof(*msg);
	vi_msg.response = (uint64_t)msg;

	err = vi_capture_control_message(chan, &vi_msg);
	if (err < 0)
		return err;

	return verify_capture_control_response(*result);
}

static int csi5_stream_open(struct tegra_csi_channel *chan, u32 stream_id,
	u32 csi_port)
{

	struct tegra_csi_device *csi = chan->csi;
	struct tegra_channel *tegra_chan =
			v4l2_get_subdev_hostdata(&chan->subdev);
	struct CAPTURE_CONTROL_MSG msg;
	int vi_port = 0;

	/* Open NVCSI stream */
	memset(&msg, 0, sizeof(msg));
	msg.header.msg_id = CAPTURE_PHY_STREAM_OPEN_REQ;

	msg.phy_stream_open_req.stream_id = stream_id;
	msg.phy_stream_open_req.csi_port = csi_port;

	if (tegra_chan->valid_ports > 1)
		vi_port = (stream_id > 0) ? 1 : 0;

	return csi5_send_control_message(tegra_chan->tegra_vi_channel[vi_port], &msg,
							&msg.phy_stream_open_resp.result);
}

static void csi5_stream_close(struct tegra_csi_channel *chan, u32 stream_id,
	u32 csi_port)
{
	struct tegra_csi_device *csi = chan->csi;
	struct tegra_channel *tegra_chan =
			v4l2_get_subdev_hostdata(&chan->subdev);
	int err = 0;
	int vi_port = 0;

	struct CAPTURE_CONTROL_MSG msg;

	/* Close NVCSI stream */
	memset(&msg, 0, sizeof(msg));
	msg.header.msg_id = CAPTURE_PHY_STREAM_CLOSE_REQ;

	msg.phy_stream_close_req.stream_id = stream_id;
	msg.phy_stream_close_req.csi_port = csi_port;

	if (tegra_chan->valid_ports > 1)
		vi_port = (stream_id > 0) ? 1 : 0;

	err = csi5_send_control_message(tegra_chan->tegra_vi_channel[vi_port], &msg,
							&msg.phy_stream_open_resp.result);
	if (err < 0) {
		dev_err(csi->dev, "%s: Error in closing stream_id=%u, csi_port=%u\n",
			__func__, stream_id, csi_port);
	}

	return;
}

static int csi5_stream_set_config(struct tegra_csi_channel *chan, u32 stream_id,
	u32 csi_port, int csi_lanes)
{
	struct tegra_csi_device *csi = chan->csi;
	struct tegra_channel *tegra_chan =
			v4l2_get_subdev_hostdata(&chan->subdev);

	struct camera_common_data *s_data = chan->s_data;
	const struct sensor_mode_properties *mode = NULL;

	unsigned int cil_settletime = 0;
	unsigned int lane_polarity = 0;
	int vi_port = 0;

	struct CAPTURE_CONTROL_MSG msg;
	struct nvcsi_brick_config brick_config;
	struct nvcsi_cil_config cil_config;
	u32 phy_mode = read_phy_mode_from_dt(chan);
	bool is_cphy = (phy_mode == CSI_PHY_MODE_CPHY);
	dev_dbg(csi->dev, "%s: stream_id=%u, csi_port=%u\n",
		__func__, stream_id, csi_port);

	/* Attempt to find the brick config from the device tree */
	if (s_data) {
		int idx = s_data->mode_prop_idx;

		dev_dbg(csi->dev, "cil_settingtime is pulled from device");
		if (idx < s_data->sensor_props.num_modes) {
			mode = &s_data->sensor_props.sensor_modes[idx];
			cil_settletime = mode->signal_properties.cil_settletime;
			lane_polarity = mode->signal_properties.lane_polarity;
		} else {
			dev_dbg(csi->dev, "mode not listed in DT, use default");
			cil_settletime = 0;
			lane_polarity = 0;
		}
	} else if (chan->of_node) {
		int err = 0;
		const char *str = NULL;

		dev_dbg(csi->dev,
			"cil_settletime is pulled from device of_node");
		err = of_property_read_string(chan->of_node, "cil_settletime",
			&str);
		if (!err) {
			err = kstrtou32(str, 10, &cil_settletime);
			if (err) {
				dev_dbg(csi->dev,
					"no cil_settletime in of_node");
				cil_settletime = 0;
			}
		}
		/* Reset string pointer for the next property */
		str = NULL;
		err = of_property_read_string(chan->of_node, "lane_polarity",
			&str);
		if (!err) {
			err = kstrtou32(str, 10, &lane_polarity);
			if (err) {
				dev_dbg(csi->dev,
					"no lane_polarity in of_node");
				lane_polarity = 0;
			}
		}
	}

	/* Brick config */
	memset(&brick_config, 0, sizeof(brick_config));
	brick_config.phy_mode = (!is_cphy) ?
		NVCSI_PHY_TYPE_DPHY : NVCSI_PHY_TYPE_CPHY;

	/* Lane polarity */
	if (!is_cphy) {
		unsigned int index = 0;
		for (index = 0; index < NVCSI_BRICK_NUM_LANES; index++)
			brick_config.lane_polarity[index] = (lane_polarity >> index) & (0x1);
	}

	/* CIL config */
	memset(&cil_config, 0, sizeof(cil_config));
	cil_config.num_lanes = csi_lanes;
	cil_config.lp_bypass_mode = is_cphy ? 0 : 1;
	cil_config.t_hs_settle = cil_settletime;

	if (s_data)
		cil_config.mipi_clock_rate = read_mipi_clk_from_dt(chan) / 1000;
	else
		cil_config.mipi_clock_rate = TEGRA_CLOCK_CSI_PORT_MAX / 1000;

	/* Set NVCSI stream config */
	memset(&msg, 0, sizeof(msg));
	msg.header.msg_id = CAPTURE_CSI_STREAM_SET_CONFIG_REQ;

	msg.csi_stream_set_config_req.stream_id = stream_id;
	msg.csi_stream_set_config_req.csi_port = csi_port;
	msg.csi_stream_set_config_req.brick_config = brick_config;
	msg.csi_stream_set_config_req.cil_config = cil_config;

	if (tegra_chan->valid_ports > 1)
		vi_port = (stream_id > 0) ? 1 : 0;

	return csi5_send_control_message(tegra_chan->tegra_vi_channel[vi_port], &msg,
							&msg.csi_stream_set_config_resp.result);
}

static int csi5_start_streaming(struct tegra_csi_channel *chan, int port_idx)
{
	int err = 0, num_lanes;
	struct tegra_csi_device *csi = chan->csi;
	struct tegra_csi_port *port = &chan->ports[port_idx];
	u32 csi_pt, st_id, vc_id;

	csi_pt = port->csi_port;
	st_id = csi5_port_to_stream(port->csi_port);
	vc_id = port->virtual_channel_id;
	num_lanes = port->lanes;

	dev_dbg(csi->dev, "%s: csi_pt=%u, st_id=%u, vc_id=%u\n",
		__func__, csi_pt, st_id, vc_id);

	csi5_stream_set_config(chan, st_id, csi_pt, num_lanes);
	csi5_stream_open(chan, st_id, csi_pt);

	return err;
}

static void csi5_stop_streaming(struct tegra_csi_channel *chan, int port_idx)
{
	struct tegra_csi_device *csi = chan->csi;
	struct tegra_csi_port *port = &chan->ports[port_idx];
	u32 csi_pt, st_id, vc_id;

	csi_pt = port->csi_port;
	st_id = csi5_port_to_stream(port->csi_port);
	vc_id = port->virtual_channel_id;

	dev_dbg(csi->dev, "%s: csi_pt=%u, st_id=%u, vc_id=%u\n",
		__func__, csi_pt, st_id, vc_id);

	csi5_stream_close(chan, st_id, csi_pt);
}

struct tegra_csi_fops csi5_fops = {
	.csi_power_on = csi5_power_on,
	.csi_power_off = csi5_power_off,
	.csi_start_streaming = csi5_start_streaming,
	.csi_stop_streaming = csi5_stop_streaming,
};
EXPORT_SYMBOL(csi5_fops);

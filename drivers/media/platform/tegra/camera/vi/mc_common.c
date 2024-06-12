// SPDX-License-Identifier: GPL-2.0-only
// SPDX-FileCopyrightText: Copyright (c) 2015-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
/*
 * Tegra Video Input device common APIs
 */

#include <linux/init.h>
#include <linux/export.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <media/tegra_v4l2_camera.h>
#include <media/camera_common.h>
#include <media/v4l2-event.h>
#include <media/tegra_camera_platform.h>
#include <media/mc_common.h>
#include <linux/nvhost.h>

/* -----------------------------------------------------------------------------
 * Media Controller and V4L2
 */

static void tegra_vi_v4l2_cleanup(struct tegra_mc_vi *vi)
{
	v4l2_device_unregister(&vi->v4l2_dev);
	media_device_unregister(&vi->media_dev);
}

static void tegra_vi_notify(struct v4l2_subdev *sd,
					  unsigned int notification, void *arg)
{
	struct tegra_mc_vi *vi = container_of(sd->v4l2_dev,
			struct tegra_mc_vi, v4l2_dev);
	const struct v4l2_event *ev = arg;
	unsigned i;
	struct tegra_channel *chan;

	if (notification != V4L2_DEVICE_NOTIFY_EVENT)
		return;

	list_for_each_entry(chan, &vi->vi_chans, list) {
		for (i = 0; i < chan->num_subdevs; i++)
			if (sd == chan->subdev[i]) {
				v4l2_event_queue(chan->video, arg);
				if (ev->type == V4L2_EVENT_SOURCE_CHANGE &&
						vb2_is_streaming(&chan->queue))
					vb2_queue_error(&chan->queue);
			}
	}
}

static int tegra_vi_v4l2_init(struct tegra_mc_vi *vi)
{
	ssize_t len;
	int ret;

	vi->media_dev.dev = vi->dev;
	len = strscpy(vi->media_dev.model, "NVIDIA Tegra Video Input Device",
		sizeof(vi->media_dev.model));
	if (len < 0)
		return -ENAMETOOLONG;

	vi->media_dev.hw_revision = 3;

	media_device_init(&vi->media_dev);

	ret = media_device_register(&vi->media_dev);
	if (ret < 0) {
		dev_err(vi->dev,
			"media device registration failed (%d)\n",
			ret);
		return ret;
	}

	mutex_init(&vi->bw_update_lock);
	vi->v4l2_dev.mdev = &vi->media_dev;
	vi->v4l2_dev.notify = tegra_vi_notify;
	ret = v4l2_device_register(vi->dev, &vi->v4l2_dev);
	if (ret < 0) {
		dev_err(vi->dev, "V4L2 device registration failed (%d)\n",
			ret);
		goto register_error;
	}

	return 0;

register_error:
	media_device_cleanup(&vi->media_dev);
	media_device_unregister(&vi->media_dev);
	return ret;
}

static int vi_parse_dt(struct tegra_mc_vi *vi, struct platform_device *dev)
{
	int err = 0;
	int num_channels = 0;
	struct tegra_channel *item;
	struct device_node *node = dev->dev.of_node;
	struct device_node *ports;
	struct device_node *port;
	int value = 0xFF;
	int ret = 0;

	err = of_property_read_u32(node, "num-channels", &num_channels);
	if (err) {
		dev_dbg(&dev->dev,
			"Failed to find num of channels, set to 0\n");
		num_channels = 0;
	}
	vi->num_channels = num_channels;

	ports = of_get_child_by_name(node, "ports");
	if (ports == NULL)
		ports = node;

	for_each_child_of_node(ports, port) {
		if (!port->name || of_node_cmp(port->name, "port"))
			continue;

		ret = of_property_read_u32(port, "reg", &value);
		if (ret < 0)
			continue;

		item = devm_kzalloc(vi->dev, sizeof(*item), GFP_KERNEL);
		if (!item)
			return -ENOMEM;

		item->id = value;
		list_add_tail(&item->list, &vi->vi_chans);
	}

	return 0;
}

static int tegra_vi_media_controller_init_int(struct tegra_mc_vi *mc_vi,
				struct platform_device *pdev)
{
	int err = 0;
	mc_vi->ndev = pdev;
	mc_vi->dev = &pdev->dev;
	INIT_LIST_HEAD(&mc_vi->vi_chans);
	mutex_init(&mc_vi->mipical_lock);

	err = vi_parse_dt(mc_vi, pdev);
	if (err)
		goto mc_init_fail;

	err = tegra_vi_v4l2_init(mc_vi);
	if (err < 0)
		goto mc_init_fail;

	/*
	 * if there is no vi channels listed in DT,
	 * no need to init the channel and graph
	 */
	if (mc_vi->num_channels == 0)
		return 0;

	/* Init Tegra VI channels */
	err = tegra_vi_channels_init(mc_vi);
	if (err < 0) {
		dev_err(&pdev->dev, "Init channel failed\n");
		goto channels_error;
	}

	/* Setup media links between VI and external sensor subdev. */
	err = tegra_vi_graph_init(mc_vi);
	if (err < 0)
		goto graph_error;

	return 0;

graph_error:
	tegra_vi_channels_cleanup(mc_vi);
channels_error:
	tegra_vi_v4l2_cleanup(mc_vi);
mc_init_fail:
	dev_err(&pdev->dev, "%s: failed\n", __func__);
	return err;
}

int tegra_capture_vi_media_controller_init(struct tegra_mc_vi *mc_vi,
				   struct platform_device *pdev)
{
	return tegra_vi_media_controller_init_int(mc_vi, pdev);
}
EXPORT_SYMBOL(tegra_capture_vi_media_controller_init);

void tegra_vi_media_controller_cleanup(struct tegra_mc_vi *mc_vi)
{
	tegra_vi_channels_unregister(mc_vi);
	tegra_vi_graph_cleanup(mc_vi);
	tegra_vi_channels_cleanup(mc_vi);
	tegra_vi_v4l2_cleanup(mc_vi);
}
EXPORT_SYMBOL(tegra_vi_media_controller_cleanup);

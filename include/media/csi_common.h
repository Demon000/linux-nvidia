/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __CSI_COMMON_H_
#define __CSI_COMMON_H_

#include <media/v4l2-subdev.h>

#include "soc/tegra/camrtc-capture.h"

#define INVALID_CSI_PORT					0xFF
#define TEGRA_CSI_BLOCKS					3
#define TEGRA_CLOCK_CSI_PORT_MAX            102000000
#define TEGRA_SURFACE_ALIGNMENT				64

#define csi_port_is_valid(port) (port > NVCSI_PORT_H ? 0 : 1)

enum camera_gang_mode {
	CAMERA_NO_GANG_MODE = 0,
	CAMERA_GANG_L_R = 1,
	CAMERA_GANG_T_B,
	CAMERA_GANG_R_L,
	CAMERA_GANG_B_T
};

void tegra_csi_channel_sd_set_sensor_sd(struct v4l2_subdev *subdev,
					struct v4l2_subdev *sensor_sd);

uint32_t tegra_csi_channel_sd_get_vi_csi_port(struct v4l2_subdev *subdev,
					      uint32_t vi_port);

#endif

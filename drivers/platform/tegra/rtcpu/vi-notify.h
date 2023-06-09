/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2016-2022, NVIDIA CORPORATION.  All rights reserved.
 */

#ifndef INCLUDE_VI_NOTIFY_H
#define INCLUDE_VI_NOTIFY_H

/* Extended message types */
#define VI_NOTIFY_MSG_INVALID	0x00000000
#define VI_NOTIFY_MSG_ACK	0x00000002
#define VI_NOTIFY_MSG_STATUS	0x00000004

/* This must match libnvvi API header and vi-notifier enum in FW */
enum {
	VI_CAPTURE_STATUS_NONE,
	VI_CAPTURE_STATUS_SUCCESS,
	VI_CAPTURE_STATUS_CSIMUX_FRAME,
	VI_CAPTURE_STATUS_CSIMUX_STREAM,
	VI_CAPTURE_STATUS_CHANSEL_FAULT,
	VI_CAPTURE_STATUS_CHANSEL_FAULT_FE,
	VI_CAPTURE_STATUS_CHANSEL_COLLISION,
	VI_CAPTURE_STATUS_CHANSEL_SHORT_FRAME,
	VI_CAPTURE_STATUS_ATOMP_PACKER_OVERFLOW,
	VI_CAPTURE_STATUS_ATOMP_FRAME_TRUNCATED,
	VI_CAPTURE_STATUS_ATOMP_FRAME_TOSSED,
	VI_CAPTURE_STATUS_ISPBUF_FIFO_OVERFLOW,
	VI_CAPTURE_STATUS_SYNC_FAILURE,
	VI_CAPTURE_STATUS_NOTIFIER_BACKEND_DOWN,
};

#endif /* INCLUDE_VI_NOTIFY_H */

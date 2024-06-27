/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022-2023, NVIDIA CORPORATION & AFFILIATES.  All rights reserved.
 */

/**
 * @file camrtc-commands.h
 *
 * @brief Commands used with "nvidia,tegra-camrtc-hsp-vm" & "nvidia,tegra-hsp-mailbox"
 * protocol
 */

#ifndef INCLUDE_CAMRTC_COMMANDS_H
#define INCLUDE_CAMRTC_COMMANDS_H

#include "camrtc-common.h"

/**
 * @defgroup HspVmMsgs Definitions for "nvidia,tegra-camrtc-hsp-vm" protocol
 * @{
 */
#define CAMRTC_HSP_MSG(_id, _param) ( \
	((uint32_t)(_id) << MK_U32(24)) | \
	((uint32_t)(_param) & MK_U32(0xffffff)))
#define CAMRTC_HSP_MSG_ID(_msg) \
	(((_msg) >> MK_U32(24)) & MK_U32(0x7f))
#define CAMRTC_HSP_MSG_PARAM(_msg) \
	((uint32_t)(_msg) & MK_U32(0xffffff))

/**
 * The IRQ message is sent when no other HSP-VM protocol message is being sent
 * (i.e. the messages for higher level protocols implementing HSP such as IVC
 * channel protocol) and the sender has updated its shared semaphore bits.
 */
#define CAMRTC_HSP_IRQ			MK_U32(0x00)

/**
 * The HELLO messages are exchanged at the beginning of VM and RCE FW session.
 * The HELLO message exchange ensures there are no unprocessed messages
 * in transit within VM or RCE FW.
 */
#define CAMRTC_HSP_HELLO		MK_U32(0x40)
/**
 * VM session close in indicated using BYE message,
 * RCE FW reclaims the resources assigned to given VM.
 * It must be sent before the Camera VM shuts down self.
 */
#define CAMRTC_HSP_BYE			MK_U32(0x41)
/**
 * The RESUME message is sent when VM wants to activate the RCE FW
 * and access the camera hardware through it.
 */
#define CAMRTC_HSP_RESUME		MK_U32(0x42)
/**
 * Power off camera HW, switch to idle state. VM initiates it during runtime suspend or SC7.
 */
#define CAMRTC_HSP_SUSPEND		MK_U32(0x43)
/**
 * Used to set up a shared memory area (such as IVC channels, trace buffer etc)
 * between Camera VM and RCE FW.
 */
#define CAMRTC_HSP_CH_SETUP		MK_U32(0x44)

/**
 * The VM includes its protocol version as a parameter to PROTOCOL message.
 * FW responds with its protocol version, or RTCPU_FW_INVALID_VERSION
 * if the VM protocol is not supported.
 */
#define CAMRTC_HSP_PROTOCOL		MK_U32(0x47)
#define CAMRTC_HSP_UNKNOWN		MK_U32(0x7F)
#define RTCPU_DRIVER_SM6_VERSION	MK_U32(6)
#define RTCPU_FW_INVALID_VERSION	MK_U32(0xFFFFFF)

/** Shared semaphore bits (FW->VM) */
#define CAMRTC_HSP_SS_FW_MASK		MK_U32(0xFFFF)
#define CAMRTC_HSP_SS_FW_SHIFT		MK_U32(0)

/** Bits used by IVC channels */
#define CAMRTC_HSP_SS_IVC_MASK		MK_U32(0xFF)

/** @} */

#endif /* INCLUDE_CAMRTC_COMMANDS_H */

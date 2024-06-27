// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES. All rights reserved.

#include "linux/tegra-hsp-combo.h"

#include <linux/version.h>

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/mailbox_client.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/err.h>

#include "soc/tegra/camrtc-commands.h"

struct camrtc_hsp_mbox {
	struct mbox_client client;
	struct mbox_chan *chan;
};

struct camrtc_hsp {
	struct camrtc_hsp_mbox rx;
	struct camrtc_hsp_mbox tx;
	u32 cookie;
	spinlock_t sendlock;
	void (*group_notify)(struct device *dev, u16 group);
	struct device dev;
	struct mutex mutex;
	struct completion emptied;
	wait_queue_head_t response_waitq;
	atomic_t response;
	long timeout;
};

static int camrtc_hsp_send(struct camrtc_hsp *camhsp, int request)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&camhsp->sendlock, flags);
	atomic_set(&camhsp->response, -1);
	ret = mbox_send_message(camhsp->tx.chan, (void *)(unsigned long) request);
	spin_unlock_irqrestore(&camhsp->sendlock, flags);

	if (ret < 0)
		dev_err(&camhsp->dev, "send 0x%08x failed: %d\n", request, ret);

	return ret < 0 ? ret : 0;
}

static int camrtc_hsp_recv(struct camrtc_hsp *camhsp,  int *response, long *timeout)
{
	*timeout = wait_event_timeout(camhsp->response_waitq,
				      (*response = atomic_xchg(&camhsp->response, -1)) >= 0,
				      *timeout);

	return *timeout == 0 ? -ETIMEDOUT : 0;
}

static int camrtc_hsp_sendrecv(struct camrtc_hsp *camhsp, u8 id, u32 param)
{
	long timeout = camhsp->timeout;
	int request = CAMRTC_HSP_MSG(id, param);
	int response;
	int ret;

	ret = camrtc_hsp_send(camhsp, request);
	if (ret)
		return ret;

	ret = camrtc_hsp_recv(camhsp, &response, &timeout);
	if (ret)
		return ret;

	if (CAMRTC_HSP_MSG_ID(request) != CAMRTC_HSP_MSG_ID(response)) {
		dev_err(&camhsp->dev,
			"request 0x%08x mismatch with response 0x%08x\n",
			request, response);
		return -EIO;
	}

	return 0;
}

static void camrtc_hsp_vm_send_irqmsg(struct camrtc_hsp *camhsp)
{
	int irqmsg = CAMRTC_HSP_MSG(CAMRTC_HSP_IRQ, 1);
	unsigned long flags;

	spin_lock_irqsave(&camhsp->sendlock, flags);
	mbox_send_message(camhsp->tx.chan, (void *)(unsigned long) irqmsg);
	spin_unlock_irqrestore(&camhsp->sendlock, flags);
}

static u32 camrtc_hsp_vm_cookie(void)
{
	u32 value = CAMRTC_HSP_MSG_PARAM(sched_clock() >> 5U);

	if (value == 0)
		value = 1;

	return value;
}

static int camrtc_hsp_vm_hello(struct camrtc_hsp *camhsp)
{
	u32 cookie = camrtc_hsp_vm_cookie();
	int request = CAMRTC_HSP_MSG(CAMRTC_HSP_HELLO, cookie);
	long timeout = camhsp->timeout;
	int response;
	int ret;

	ret = camrtc_hsp_send(camhsp, request);
	if (ret)
		return ret;

	for (;;) {
		ret = camrtc_hsp_recv(camhsp, &response, &timeout);
		if (ret)
			return ret;

		/* Wait until we get the HELLO message we sent */
		if (response == request)
			break;
	}

	camhsp->cookie = cookie;

	return 0;
}

static struct device_node *hsp_vm_get_available(const struct device_node *parent)
{
	struct device_node *child;

	for_each_child_of_node(parent, child)
		if (of_device_is_compatible(child, "nvidia,tegra-camrtc-hsp-vm") &&
			of_device_is_available(child))
			break;

	return child;
}

static int camrtc_hsp_vm_probe(struct camrtc_hsp *camhsp)
{
	struct device_node *np;
	int err;

	np = hsp_vm_get_available(camhsp->dev.parent->of_node);
	if (!np)
		return -ENOTSUPP;

	camhsp->dev.of_node = np;

	camhsp->rx.chan = mbox_request_channel_byname(&camhsp->rx.client, "vm-rx");
	if (IS_ERR(camhsp->rx.chan)) {
		err = PTR_ERR(camhsp->rx.chan);
		goto fail;
	}

	camhsp->tx.chan = mbox_request_channel_byname(&camhsp->tx.client, "vm-tx");
	if (IS_ERR(camhsp->tx.chan)) {
		err = PTR_ERR(camhsp->tx.chan);
		goto fail;
	}

	dev_set_name(&camhsp->dev, "%s:%s",
		dev_name(camhsp->dev.parent), camhsp->dev.of_node->name);

	return 0;

fail:
	of_node_put(np);

	return err;
}

void camrtc_hsp_group_ring(struct camrtc_hsp *camhsp)
{
	camrtc_hsp_vm_send_irqmsg(camhsp);
}
EXPORT_SYMBOL(camrtc_hsp_group_ring);

int camrtc_hsp_sync(struct camrtc_hsp *camhsp)
{
	int ret;

	mutex_lock(&camhsp->mutex);
	ret = camrtc_hsp_vm_hello(camhsp);
	if (ret)
		goto out;

	ret = camrtc_hsp_sendrecv(camhsp, CAMRTC_HSP_PROTOCOL,
				  RTCPU_DRIVER_SM6_VERSION);

out:
	mutex_unlock(&camhsp->mutex);

	return ret;
}
EXPORT_SYMBOL(camrtc_hsp_sync);

int camrtc_hsp_resume(struct camrtc_hsp *camhsp)
{
	int ret;

	mutex_lock(&camhsp->mutex);
	ret = camrtc_hsp_sendrecv(camhsp, CAMRTC_HSP_RESUME, camhsp->cookie);
	mutex_unlock(&camhsp->mutex);

	return ret;
}
EXPORT_SYMBOL(camrtc_hsp_resume);

int camrtc_hsp_suspend(struct camrtc_hsp *camhsp)
{
	int ret;

	mutex_lock(&camhsp->mutex);
	ret = camrtc_hsp_sendrecv(camhsp, CAMRTC_HSP_SUSPEND, 0);
	mutex_unlock(&camhsp->mutex);

	return ret;
}
EXPORT_SYMBOL(camrtc_hsp_suspend);

int camrtc_hsp_bye(struct camrtc_hsp *camhsp)
{
	int ret;

	mutex_lock(&camhsp->mutex);
	camhsp->cookie = 0U;
	ret = camrtc_hsp_sendrecv(camhsp, CAMRTC_HSP_BYE, 0);
	mutex_unlock(&camhsp->mutex);

	return ret;
}
EXPORT_SYMBOL(camrtc_hsp_bye);

int camrtc_hsp_ch_setup(struct camrtc_hsp *camhsp, dma_addr_t iova)
{
	int ret;

	mutex_lock(&camhsp->mutex);
	ret = camrtc_hsp_sendrecv(camhsp, CAMRTC_HSP_CH_SETUP, iova >> 8);
	mutex_unlock(&camhsp->mutex);

	return ret;
}
EXPORT_SYMBOL(camrtc_hsp_ch_setup);

static void camrtc_hsp_rx_full_notify(struct mbox_client *cl, void *data)
{
	struct camrtc_hsp *camhsp = dev_get_drvdata(cl->dev);
	u32 status, group;

	u32 msg = (u32) (unsigned long) data;
	u8 id = CAMRTC_HSP_MSG_ID(msg);
	status = CAMRTC_HSP_SS_FW_MASK;
	status >>= CAMRTC_HSP_SS_FW_SHIFT;
	group = status & CAMRTC_HSP_SS_IVC_MASK;

	if (group != 0) {
		dev_err(&camhsp->dev, "group notify msg: 0x%08x, id: %02x\n", msg, id);
		camhsp->group_notify(camhsp->dev.parent, (u16)group);
	}

	if (id == CAMRTC_HSP_IRQ)
		return;

	if (id < CAMRTC_HSP_HELLO) {
		/* Rest of the unidirectional messages are now ignored */
		dev_info(&camhsp->dev, "unknown message 0x%08x\n", msg);
		return;
	}

	atomic_set(&camhsp->response, msg);
	wake_up(&camhsp->response_waitq);
}

static const struct device_type camrtc_hsp_combo_dev_type = {
	.name	= "camrtc-hsp-protocol",
};

static void camrtc_hsp_combo_dev_release(struct device *dev)
{
	struct camrtc_hsp *camhsp = container_of(dev, struct camrtc_hsp, dev);

	mbox_free_channel(camhsp->rx.chan);
	mbox_free_channel(camhsp->tx.chan);
	of_node_put(dev->of_node);
	kfree(camhsp);
}

struct camrtc_hsp *camrtc_hsp_create(
	struct device *dev,
	void (*group_notify)(struct device *dev, u16 group),
	long cmd_timeout)
{
	struct camrtc_hsp *camhsp;
	int ret = -EINVAL;

	camhsp = kzalloc(sizeof(*camhsp), GFP_KERNEL);
	if (camhsp == NULL)
		return ERR_PTR(-ENOMEM);

	camhsp->dev.parent = dev;
	camhsp->group_notify = group_notify;
	camhsp->timeout = cmd_timeout;
	mutex_init(&camhsp->mutex);
	spin_lock_init(&camhsp->sendlock);
	init_waitqueue_head(&camhsp->response_waitq);
	init_completion(&camhsp->emptied);
	atomic_set(&camhsp->response, -1);

	camhsp->dev.type = &camrtc_hsp_combo_dev_type;
	camhsp->dev.release = camrtc_hsp_combo_dev_release;
	device_initialize(&camhsp->dev);

	camhsp->tx.client.tx_block = false;
	camhsp->tx.client.dev = &camhsp->dev;
	camhsp->rx.client.rx_callback = camrtc_hsp_rx_full_notify;
	camhsp->rx.client.dev = &camhsp->dev;
	dev_set_drvdata(&camhsp->dev, camhsp);
	dev_set_name(&camhsp->dev, "%s:%s", dev_name(dev), "hsp");

	ret = camrtc_hsp_vm_probe(camhsp);
	if (ret)
		goto fail;

	ret = device_add(&camhsp->dev);
	if (ret)
		goto fail;

	return camhsp;

fail:
	camrtc_hsp_free(camhsp);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(camrtc_hsp_create);

void camrtc_hsp_free(struct camrtc_hsp *camhsp)
{
	if (dev_get_drvdata(&camhsp->dev) != NULL)
		device_unregister(&camhsp->dev);
}
EXPORT_SYMBOL(camrtc_hsp_free);
MODULE_LICENSE("GPL v2");

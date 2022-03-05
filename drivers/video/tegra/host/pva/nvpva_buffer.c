/*
 * NVPVA buffer management for T194 and T234
 *
 * Copyright (c) 2016-2022, NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/dma-buf.h>
#include <linux/cvnas.h>

#include "dev.h"
#include "nvpva_buffer.h"

/**
 * nvpva_vm_buffer - Virtual mapping information for a buffer
 *
 * @attach:		Pointer to dma_buf_attachment struct
 * @dmabuf:		Pointer to dma_buf struct
 * @sgt:		Pointer to sg_table struct
 * @addr:		Physical address of the buffer
 * @size:		Size of the buffer
 * @user_map_count:	Buffer reference count from user space
 * @submit_map_count:	Buffer reference count from task submit
 * @rb_node:		pinned buffer node
 * @list_head:		List entry
 *
 */
struct nvpva_vm_buffer {
	struct dma_buf_attachment *attach;
	struct dma_buf *dmabuf;
	struct sg_table *sgt;

	dma_addr_t addr;
	size_t size;
	enum nvpva_buffers_heap heap;

	s32 user_map_count;
	s32 submit_map_count;

	struct rb_node rb_node;
	struct list_head list_head;
};

static struct nvpva_vm_buffer *nvpva_find_map_buffer(
		struct nvpva_buffers *nvpva_buffers, struct dma_buf *dmabuf)
{
	struct rb_root *root = &nvpva_buffers->rb_root;
	struct rb_node *node = root->rb_node;
	struct nvpva_vm_buffer *vm;

	/* check in a sorted tree */
	while (node) {
		vm = rb_entry(node, struct nvpva_vm_buffer,
						rb_node);

		if (vm->dmabuf > dmabuf)
			node = node->rb_left;
		else if (vm->dmabuf != dmabuf)
			node = node->rb_right;
		else
			return vm;
	}

	return NULL;
}

static void nvpva_buffer_insert_map_buffer(
				struct nvpva_buffers *nvpva_buffers,
				struct nvpva_vm_buffer *new_vm)
{
	struct rb_node **new_node = &(nvpva_buffers->rb_root.rb_node);
	struct rb_node *parent = NULL;

	/* Figure out where to put the new node */
	while (*new_node) {
		struct nvpva_vm_buffer *vm =
			rb_entry(*new_node, struct nvpva_vm_buffer,
						rb_node);
		parent = *new_node;

		if (vm->dmabuf > new_vm->dmabuf)
			new_node = &((*new_node)->rb_left);
		else
			new_node = &((*new_node)->rb_right);
	}

	/* Add new node and rebalance tree */
	rb_link_node(&new_vm->rb_node, parent, new_node);
	rb_insert_color(&new_vm->rb_node, &nvpva_buffers->rb_root);

	/* Add the node into a list  */
	list_add_tail(&new_vm->list_head, &nvpva_buffers->list_head);
}

int nvpva_get_iova_addr(struct nvpva_buffers *nvpva_buffers,
			struct dma_buf *dmabuf, dma_addr_t *addr)
{
	struct nvpva_vm_buffer *vm;
	int err = -EINVAL;

	mutex_lock(&nvpva_buffers->mutex);

	vm = nvpva_find_map_buffer(nvpva_buffers, dmabuf);
	if (vm) {
		*addr = vm->addr;
		err = 0;
	}

	mutex_unlock(&nvpva_buffers->mutex);

	return err;
}

static int nvpva_buffer_map(struct platform_device *pdev,
				struct dma_buf *dmabuf,
				struct nvpva_vm_buffer *vm)
{

	const dma_addr_t cvnas_begin = nvcvnas_get_cvsram_base();
	const dma_addr_t cvnas_end = cvnas_begin + nvcvnas_get_cvsram_size();
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	dma_addr_t dma_addr;
	dma_addr_t phys_addr;
	int err = 0;

	get_dma_buf(dmabuf);

	attach = dma_buf_attach(dmabuf, &pdev->dev);
	if (IS_ERR_OR_NULL(attach)) {
		err = PTR_ERR(dmabuf);
		dev_err(&pdev->dev, "dma_attach failed: %d\n", err);
		goto buf_attach_err;
	}

	sgt = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR_OR_NULL(sgt)) {
		err = PTR_ERR(sgt);
		dev_err(&pdev->dev, "dma mapping failed: %d\n", err);
		goto buf_map_err;
	}

	phys_addr = sg_phys(sgt->sgl);
	dma_addr = sg_dma_address(sgt->sgl);

	/* Determine the heap */
	if (phys_addr >= cvnas_begin && phys_addr < cvnas_end)
		vm->heap = NVPVA_BUFFERS_HEAP_CVNAS;
	else
		vm->heap = NVPVA_BUFFERS_HEAP_DRAM;

	/*
	 * If dma address is not available or heap is in CVNAS, use the
	 * physical address.
	 */
	if (!dma_addr || vm->heap == NVPVA_BUFFERS_HEAP_CVNAS)
		dma_addr = phys_addr;

	vm->sgt = sgt;
	vm->attach = attach;
	vm->dmabuf = dmabuf;
	vm->size = dmabuf->size;
	vm->addr = dma_addr;
	vm->user_map_count = 1;

	return err;

buf_map_err:
	dma_buf_detach(dmabuf, attach);
buf_attach_err:
	dma_buf_put(dmabuf);
	return err;
}

static void nvpva_free_buffers(struct kref *kref)
{
	struct nvpva_buffers *nvpva_buffers =
		container_of(kref, struct nvpva_buffers, kref);

	kfree(nvpva_buffers);
}

static void nvpva_buffer_unmap(struct nvpva_buffers *nvpva_buffers,
				struct nvpva_vm_buffer *vm)
{
	nvhost_dbg_fn("");

	if ((vm->user_map_count != 0) || (vm->submit_map_count != 0))
		return;

	dma_buf_unmap_attachment(vm->attach, vm->sgt, DMA_BIDIRECTIONAL);
	dma_buf_detach(vm->dmabuf, vm->attach);
	dma_buf_put(vm->dmabuf);

	rb_erase(&vm->rb_node, &nvpva_buffers->rb_root);
	list_del(&vm->list_head);

	kfree(vm);
}

struct nvpva_buffers *nvpva_buffer_init(struct platform_device *pdev)
{
	struct nvpva_buffers *nvpva_buffers;
	int err = 0;

	nvpva_buffers = kzalloc(sizeof(struct nvpva_buffers), GFP_KERNEL);
	if (!nvpva_buffers) {
		err = -ENOMEM;
		goto nvpva_buffer_init_err;
	}

	nvpva_buffers->pdev = pdev;
	mutex_init(&nvpva_buffers->mutex);
	nvpva_buffers->rb_root = RB_ROOT;
	INIT_LIST_HEAD(&nvpva_buffers->list_head);
	kref_init(&nvpva_buffers->kref);

	return nvpva_buffers;

nvpva_buffer_init_err:
	return ERR_PTR(err);
}

int nvpva_buffer_submit_pin(struct nvpva_buffers *nvpva_buffers,
			     struct dma_buf **dmabufs, u32 count,
			     dma_addr_t *paddr, size_t *psize,
			     enum nvpva_buffers_heap *heap)
{
	struct nvpva_vm_buffer *vm;
	int i = 0;

	kref_get(&nvpva_buffers->kref);

	mutex_lock(&nvpva_buffers->mutex);

	for (i = 0; i < count; i++) {
		vm = nvpva_find_map_buffer(nvpva_buffers, dmabufs[i]);
		if (vm == NULL)
			goto submit_err;

		vm->submit_map_count++;
		paddr[i] = vm->addr;
		psize[i] = vm->size;

		/* Return heap only if requested */
		if (heap != NULL)
			heap[i] = vm->heap;
	}

	mutex_unlock(&nvpva_buffers->mutex);
	return 0;

submit_err:
	mutex_unlock(&nvpva_buffers->mutex);

	count = i;

	nvpva_buffer_submit_unpin(nvpva_buffers, dmabufs, count);

	return -EINVAL;
}

int nvpva_buffer_pin(struct nvpva_buffers *nvpva_buffers,
			struct dma_buf **dmabufs,
			u32 count)
{
	struct nvpva_vm_buffer *vm;
	int i = 0;
	int err = 0;

	mutex_lock(&nvpva_buffers->mutex);

	for (i = 0; i < count; i++) {
		vm = nvpva_find_map_buffer(nvpva_buffers, dmabufs[i]);
		if (vm) {
			vm->user_map_count++;
			continue;
		}

		vm = kzalloc(sizeof(struct nvpva_vm_buffer), GFP_KERNEL);
		if (!vm) {
			nvhost_err(NULL, "could not allocate vm_buffer");
			err = -ENOMEM;
			goto unpin;
		}

		err = nvpva_buffer_map(nvpva_buffers->pdev, dmabufs[i], vm);
		if (err)
			goto free_vm;

		nvpva_buffer_insert_map_buffer(nvpva_buffers, vm);
	}

	mutex_unlock(&nvpva_buffers->mutex);
	return err;

free_vm:
	kfree(vm);
unpin:
	mutex_unlock(&nvpva_buffers->mutex);

	/* free pinned buffers */
	count = i;
	nvpva_buffer_unpin(nvpva_buffers, dmabufs, count);

	return err;
}

void nvpva_buffer_submit_unpin(struct nvpva_buffers *nvpva_buffers,
				struct dma_buf **dmabufs, u32 count)
{
	struct nvpva_vm_buffer *vm;
	int i = 0;

	mutex_lock(&nvpva_buffers->mutex);

	for (i = 0; i < count; i++) {

		vm = nvpva_find_map_buffer(nvpva_buffers, dmabufs[i]);
		if (vm == NULL)
			continue;

		if (vm->submit_map_count-- < 0)
			vm->submit_map_count = 0;
		nvpva_buffer_unmap(nvpva_buffers, vm);
	}

	mutex_unlock(&nvpva_buffers->mutex);

	kref_put(&nvpva_buffers->kref, nvpva_free_buffers);
}

void nvpva_buffer_unpin(struct nvpva_buffers *nvpva_buffers,
			 struct dma_buf **dmabufs, u32 count)
{
	int i = 0;

	mutex_lock(&nvpva_buffers->mutex);

	for (i = 0; i < count; i++) {
		struct nvpva_vm_buffer *vm = NULL;

		vm = nvpva_find_map_buffer(nvpva_buffers, dmabufs[i]);
		if (vm == NULL)
			continue;

		if (vm->user_map_count-- < 0)
			vm->user_map_count = 0;
		nvpva_buffer_unmap(nvpva_buffers, vm);
	}

	mutex_unlock(&nvpva_buffers->mutex);
}

void nvpva_buffer_release(struct nvpva_buffers *nvpva_buffers)
{
	struct nvpva_vm_buffer *vm, *n;

	/* Go through each entry and remove it safely */
	mutex_lock(&nvpva_buffers->mutex);
	list_for_each_entry_safe(vm, n, &nvpva_buffers->list_head,
				 list_head) {
		vm->user_map_count = 0;
		nvpva_buffer_unmap(nvpva_buffers, vm);
	}
	mutex_unlock(&nvpva_buffers->mutex);

	kref_put(&nvpva_buffers->kref, nvpva_free_buffers);
}

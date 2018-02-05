/*
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/clk/tegra.h>
#include <linux/genalloc.h>
#include <linux/mailbox_client.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/semaphore.h>
#include <linux/sched/clock.h>

#include <soc/tegra/bpmp.h>
#include <soc/tegra/bpmp-abi.h>
#include <soc/tegra/ivc.h>

#define MSG_ACK		BIT(0)
#define MSG_RING	BIT(1)

static inline struct tegra_bpmp *
mbox_client_to_bpmp(struct mbox_client *client)
{
	return container_of(client, struct tegra_bpmp, mbox.client);
}

struct tegra_bpmp *tegra_bpmp_get(struct device *dev)
{
	struct platform_device *pdev;
	struct tegra_bpmp *bpmp;
	struct device_node *np;

	np = of_parse_phandle(dev->of_node, "nvidia,bpmp", 0);
	if (!np)
		return ERR_PTR(-ENOENT);

	pdev = of_find_device_by_node(np);
	if (!pdev) {
		bpmp = ERR_PTR(-ENODEV);
		goto put;
	}

	bpmp = platform_get_drvdata(pdev);
	if (!bpmp) {
		bpmp = ERR_PTR(-EPROBE_DEFER);
		put_device(&pdev->dev);
		goto put;
	}

put:
	of_node_put(np);
	return bpmp;
}
EXPORT_SYMBOL_GPL(tegra_bpmp_get);

void tegra_bpmp_put(struct tegra_bpmp *bpmp)
{
	if (bpmp)
		put_device(bpmp->dev);
}
EXPORT_SYMBOL_GPL(tegra_bpmp_put);

static int tegra_bpmp_channel_get_index(struct tegra_bpmp_channel *channel)
{
	return channel - channel->bpmp->channels;
}

static int
tegra_bpmp_channel_get_thread_index(struct tegra_bpmp_channel *channel)
{
	struct tegra_bpmp *bpmp = channel->bpmp;
	unsigned int offset, count;
	int index;

	offset = bpmp->soc->channels.thread.offset;
	count = bpmp->soc->channels.thread.count;

	index = tegra_bpmp_channel_get_index(channel);
	if (index < 0)
		return index;

	if (index < offset || index >= offset + count)
		return -EINVAL;

	return index - offset;
}

static struct tegra_bpmp_channel *
tegra_bpmp_channel_get_thread(struct tegra_bpmp *bpmp, unsigned int index)
{
	unsigned int offset = bpmp->soc->channels.thread.offset;
	unsigned int count = bpmp->soc->channels.thread.count;

	if (index >= count)
		return NULL;

	return &bpmp->channels[offset + index];
}

static struct tegra_bpmp_channel *
tegra_bpmp_channel_get_tx(struct tegra_bpmp *bpmp)
{
	unsigned int offset = bpmp->soc->channels.cpu_tx.offset;

	return &bpmp->channels[offset + smp_processor_id()];
}

static struct tegra_bpmp_channel *
tegra_bpmp_channel_get_rx(struct tegra_bpmp *bpmp)
{
	unsigned int offset = bpmp->soc->channels.cpu_rx.offset;

	return &bpmp->channels[offset];
}

static bool tegra_bpmp_message_valid(const struct tegra_bpmp_message *msg)
{
	return (msg->tx.size <= MSG_DATA_MIN_SZ) &&
	       (msg->rx.size <= MSG_DATA_MIN_SZ) &&
	       (msg->tx.size == 0 || msg->tx.data) &&
	       (msg->rx.size == 0 || msg->rx.data);
}

static bool tegra_bpmp_master_acked(struct tegra_bpmp_channel *channel)
{
	void *frame;

	frame = tegra_ivc_read_get_next_frame(channel->ivc);
	if (IS_ERR(frame)) {
		channel->ib = NULL;
		return false;
	}

	channel->ib = frame;

	return true;
}

static int tegra_bpmp_wait_ack(struct tegra_bpmp_channel *channel)
{
	unsigned long timeout = channel->bpmp->soc->channels.cpu_tx.timeout;
	ktime_t end;

	end = ktime_add_us(ktime_get(), timeout);

	do {
		if (tegra_bpmp_master_acked(channel))
			return 0;
	} while (ktime_before(ktime_get(), end));

	return -ETIMEDOUT;
}

static bool tegra_bpmp_master_free(struct tegra_bpmp_channel *channel)
{
	void *frame;

	frame = tegra_ivc_write_get_next_frame(channel->ivc);
	if (IS_ERR(frame)) {
		channel->ob = NULL;
		return false;
	}

	channel->ob = frame;

	return true;
}

static int tegra_bpmp_wait_master_free(struct tegra_bpmp_channel *channel)
{
	unsigned long timeout = channel->bpmp->soc->channels.cpu_tx.timeout;
	ktime_t start, now;

	start = ns_to_ktime(local_clock());

	do {
		if (tegra_bpmp_master_free(channel))
			return 0;

		now = ns_to_ktime(local_clock());
	} while (ktime_us_delta(now, start) < timeout);

	return -ETIMEDOUT;
}

static ssize_t __tegra_bpmp_channel_read(struct tegra_bpmp_channel *channel,
					 void *data, size_t size, int *ret)
{
	int err;

	if (data && size > 0)
		memcpy(data, channel->ib->data, size);

	err = tegra_ivc_read_advance(channel->ivc);
	if (err < 0)
		return err;

	*ret = channel->ib->code;

	return 0;
}

static ssize_t tegra_bpmp_channel_read(struct tegra_bpmp_channel *channel,
				       void *data, size_t size, int *ret)
{
	struct tegra_bpmp *bpmp = channel->bpmp;
	unsigned long flags;
	ssize_t err;
	int index;

	index = tegra_bpmp_channel_get_thread_index(channel);
	if (index < 0) {
		err = index;
		goto unlock;
	}

	spin_lock_irqsave(&bpmp->lock, flags);
	err = __tegra_bpmp_channel_read(channel, data, size, ret);
	clear_bit(index, bpmp->threaded.allocated);
	spin_unlock_irqrestore(&bpmp->lock, flags);

unlock:
	up(&bpmp->threaded.lock);

	return err;
}

static ssize_t __tegra_bpmp_channel_write(struct tegra_bpmp_channel *channel,
					  unsigned int mrq, unsigned long flags,
					  const void *data, size_t size)
{
	channel->ob->code = mrq;
	channel->ob->flags = flags;

	if (data && size > 0)
		memcpy(channel->ob->data, data, size);

	return tegra_ivc_write_advance(channel->ivc);
}

static struct tegra_bpmp_channel *
tegra_bpmp_write_threaded(struct tegra_bpmp *bpmp, unsigned int mrq,
			  const void *data, size_t size)
{
	unsigned long timeout = bpmp->soc->channels.thread.timeout;
	unsigned int count = bpmp->soc->channels.thread.count;
	struct tegra_bpmp_channel *channel;
	unsigned long flags;
	unsigned int index;
	int err;

	err = down_timeout(&bpmp->threaded.lock, usecs_to_jiffies(timeout));
	if (err < 0)
		return ERR_PTR(err);

	spin_lock_irqsave(&bpmp->lock, flags);

	index = find_first_zero_bit(bpmp->threaded.allocated, count);
	if (index == count) {
		err = -EBUSY;
		goto unlock;
	}

	channel = tegra_bpmp_channel_get_thread(bpmp, index);
	if (!channel) {
		err = -EINVAL;
		goto unlock;
	}

	if (!tegra_bpmp_master_free(channel)) {
		err = -EBUSY;
		goto unlock;
	}

	set_bit(index, bpmp->threaded.allocated);

	err = __tegra_bpmp_channel_write(channel, mrq, MSG_ACK | MSG_RING,
					 data, size);
	if (err < 0)
		goto clear_allocated;

	set_bit(index, bpmp->threaded.busy);

	spin_unlock_irqrestore(&bpmp->lock, flags);
	return channel;

clear_allocated:
	clear_bit(index, bpmp->threaded.allocated);
unlock:
	spin_unlock_irqrestore(&bpmp->lock, flags);
	up(&bpmp->threaded.lock);

	return ERR_PTR(err);
}

static ssize_t tegra_bpmp_channel_write(struct tegra_bpmp_channel *channel,
					unsigned int mrq, unsigned long flags,
					const void *data, size_t size)
{
	int err;

	err = tegra_bpmp_wait_master_free(channel);
	if (err < 0)
		return err;

	return __tegra_bpmp_channel_write(channel, mrq, flags, data, size);
}

int tegra_bpmp_transfer_atomic(struct tegra_bpmp *bpmp,
			       struct tegra_bpmp_message *msg)
{
	struct tegra_bpmp_channel *channel;
	int err;

	if (WARN_ON(!irqs_disabled()))
		return -EPERM;

	if (!tegra_bpmp_message_valid(msg))
		return -EINVAL;

	channel = tegra_bpmp_channel_get_tx(bpmp);

	err = tegra_bpmp_channel_write(channel, msg->mrq, MSG_ACK,
				       msg->tx.data, msg->tx.size);
	if (err < 0)
		return err;

	err = mbox_send_message(bpmp->mbox.channel, NULL);
	if (err < 0)
		return err;

	mbox_client_txdone(bpmp->mbox.channel, 0);

	err = tegra_bpmp_wait_ack(channel);
	if (err < 0)
		return err;

	return __tegra_bpmp_channel_read(channel, msg->rx.data, msg->rx.size,
					 &msg->rx.ret);
}
EXPORT_SYMBOL_GPL(tegra_bpmp_transfer_atomic);

int tegra_bpmp_transfer(struct tegra_bpmp *bpmp,
			struct tegra_bpmp_message *msg)
{
	struct tegra_bpmp_channel *channel;
	unsigned long timeout;
	int err;

	if (WARN_ON(irqs_disabled()))
		return -EPERM;

	if (!tegra_bpmp_message_valid(msg))
		return -EINVAL;

	channel = tegra_bpmp_write_threaded(bpmp, msg->mrq, msg->tx.data,
					    msg->tx.size);
	if (IS_ERR(channel))
		return PTR_ERR(channel);

	err = mbox_send_message(bpmp->mbox.channel, NULL);
	if (err < 0)
		return err;

	mbox_client_txdone(bpmp->mbox.channel, 0);

	timeout = usecs_to_jiffies(bpmp->soc->channels.thread.timeout);

	err = wait_for_completion_timeout(&channel->completion, timeout);
	if (err == 0)
		return -ETIMEDOUT;

	return tegra_bpmp_channel_read(channel, msg->rx.data, msg->rx.size,
				       &msg->rx.ret);
}
EXPORT_SYMBOL_GPL(tegra_bpmp_transfer);

static struct tegra_bpmp_mrq *tegra_bpmp_find_mrq(struct tegra_bpmp *bpmp,
						  unsigned int mrq)
{
	struct tegra_bpmp_mrq *entry;

	list_for_each_entry(entry, &bpmp->mrqs, list)
		if (entry->mrq == mrq)
			return entry;

	return NULL;
}

void tegra_bpmp_mrq_return(struct tegra_bpmp_channel *channel, int code,
			   const void *data, size_t size)
{
	unsigned long flags = channel->ib->flags;
	struct tegra_bpmp *bpmp = channel->bpmp;
	struct tegra_bpmp_mb_data *frame;
	int err;

	if (WARN_ON(size > MSG_DATA_MIN_SZ))
		return;

	err = tegra_ivc_read_advance(channel->ivc);
	if (WARN_ON(err < 0))
		return;

	if ((flags & MSG_ACK) == 0)
		return;

	frame = tegra_ivc_write_get_next_frame(channel->ivc);
	if (WARN_ON(IS_ERR(frame)))
		return;

	frame->code = code;

	if (data && size > 0)
		memcpy(frame->data, data, size);

	err = tegra_ivc_write_advance(channel->ivc);
	if (WARN_ON(err < 0))
		return;

	if (flags & MSG_RING) {
		err = mbox_send_message(bpmp->mbox.channel, NULL);
		if (WARN_ON(err < 0))
			return;

		mbox_client_txdone(bpmp->mbox.channel, 0);
	}
}
EXPORT_SYMBOL_GPL(tegra_bpmp_mrq_return);

static void tegra_bpmp_handle_mrq(struct tegra_bpmp *bpmp,
				  unsigned int mrq,
				  struct tegra_bpmp_channel *channel)
{
	struct tegra_bpmp_mrq *entry;
	u32 zero = 0;

	spin_lock(&bpmp->lock);

	entry = tegra_bpmp_find_mrq(bpmp, mrq);
	if (!entry) {
		spin_unlock(&bpmp->lock);
		tegra_bpmp_mrq_return(channel, -EINVAL, &zero, sizeof(zero));
		return;
	}

	entry->handler(mrq, channel, entry->data);

	spin_unlock(&bpmp->lock);
}

int tegra_bpmp_request_mrq(struct tegra_bpmp *bpmp, unsigned int mrq,
			   tegra_bpmp_mrq_handler_t handler, void *data)
{
	struct tegra_bpmp_mrq *entry;
	unsigned long flags;

	if (!handler)
		return -EINVAL;

	entry = devm_kzalloc(bpmp->dev, sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	spin_lock_irqsave(&bpmp->lock, flags);

	entry->mrq = mrq;
	entry->handler = handler;
	entry->data = data;
	list_add(&entry->list, &bpmp->mrqs);

	spin_unlock_irqrestore(&bpmp->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_bpmp_request_mrq);

void tegra_bpmp_free_mrq(struct tegra_bpmp *bpmp, unsigned int mrq, void *data)
{
	struct tegra_bpmp_mrq *entry;
	unsigned long flags;

	spin_lock_irqsave(&bpmp->lock, flags);

	entry = tegra_bpmp_find_mrq(bpmp, mrq);
	if (!entry)
		goto unlock;

	list_del(&entry->list);
	devm_kfree(bpmp->dev, entry);

unlock:
	spin_unlock_irqrestore(&bpmp->lock, flags);
}
EXPORT_SYMBOL_GPL(tegra_bpmp_free_mrq);

static void tegra_bpmp_mrq_handle_ping(unsigned int mrq,
				       struct tegra_bpmp_channel *channel,
				       void *data)
{
	struct mrq_ping_request *request;
	struct mrq_ping_response response;

	request = (struct mrq_ping_request *)channel->ib->data;

	memset(&response, 0, sizeof(response));
	response.reply = request->challenge << 1;

	tegra_bpmp_mrq_return(channel, 0, &response, sizeof(response));
}

static int tegra_bpmp_ping(struct tegra_bpmp *bpmp)
{
	struct mrq_ping_response response;
	struct mrq_ping_request request;
	struct tegra_bpmp_message msg;
	unsigned long flags;
	ktime_t start, end;
	int err;

	memset(&request, 0, sizeof(request));
	request.challenge = 1;

	memset(&response, 0, sizeof(response));

	memset(&msg, 0, sizeof(msg));
	msg.mrq = MRQ_PING;
	msg.tx.data = &request;
	msg.tx.size = sizeof(request);
	msg.rx.data = &response;
	msg.rx.size = sizeof(response);

	local_irq_save(flags);
	start = ktime_get();
	err = tegra_bpmp_transfer_atomic(bpmp, &msg);
	end = ktime_get();
	local_irq_restore(flags);

	if (!err)
		dev_dbg(bpmp->dev,
			"ping ok: challenge: %u, response: %u, time: %lld\n",
			request.challenge, response.reply,
			ktime_to_us(ktime_sub(end, start)));

	return err;
}

static int tegra_bpmp_get_firmware_tag(struct tegra_bpmp *bpmp, char *tag,
				       size_t size)
{
	struct mrq_query_tag_request request;
	struct tegra_bpmp_message msg;
	unsigned long flags;
	dma_addr_t phys;
	void *virt;
	int err;

	virt = dma_alloc_coherent(bpmp->dev, MSG_DATA_MIN_SZ, &phys,
				  GFP_KERNEL | GFP_DMA32);
	if (!virt)
		return -ENOMEM;

	memset(&request, 0, sizeof(request));
	request.addr = phys;

	memset(&msg, 0, sizeof(msg));
	msg.mrq = MRQ_QUERY_TAG;
	msg.tx.data = &request;
	msg.tx.size = sizeof(request);

	local_irq_save(flags);
	err = tegra_bpmp_transfer_atomic(bpmp, &msg);
	local_irq_restore(flags);

	if (err == 0)
		strlcpy(tag, virt, size);

	dma_free_coherent(bpmp->dev, MSG_DATA_MIN_SZ, virt, phys);

	return err;
}

static void tegra_bpmp_channel_signal(struct tegra_bpmp_channel *channel)
{
	unsigned long flags = channel->ob->flags;

	if ((flags & MSG_RING) == 0)
		return;

	complete(&channel->completion);
}

static void tegra_bpmp_handle_rx(struct mbox_client *client, void *data)
{
	struct tegra_bpmp *bpmp = mbox_client_to_bpmp(client);
	struct tegra_bpmp_channel *channel;
	unsigned int i, count;
	unsigned long *busy;

	channel = tegra_bpmp_channel_get_rx(bpmp);
	count = bpmp->soc->channels.thread.count;
	busy = bpmp->threaded.busy;

	if (tegra_bpmp_master_acked(channel))
		tegra_bpmp_handle_mrq(bpmp, channel->ib->code, channel);

	spin_lock(&bpmp->lock);

	for_each_set_bit(i, busy, count) {
		struct tegra_bpmp_channel *channel;

		channel = tegra_bpmp_channel_get_thread(bpmp, i);
		if (!channel)
			continue;

		if (tegra_bpmp_master_acked(channel)) {
			tegra_bpmp_channel_signal(channel);
			clear_bit(i, busy);
		}
	}

	spin_unlock(&bpmp->lock);
}

static void tegra_bpmp_ivc_notify(struct tegra_ivc *ivc, void *data)
{
	struct tegra_bpmp *bpmp = data;
	int err;

	if (WARN_ON(bpmp->mbox.channel == NULL))
		return;

	err = mbox_send_message(bpmp->mbox.channel, NULL);
	if (err < 0)
		return;

	mbox_client_txdone(bpmp->mbox.channel, 0);
}

static int tegra_bpmp_channel_init(struct tegra_bpmp_channel *channel,
				   struct tegra_bpmp *bpmp,
				   unsigned int index)
{
	size_t message_size, queue_size;
	unsigned int offset;
	int err;

	channel->ivc = devm_kzalloc(bpmp->dev, sizeof(*channel->ivc),
				    GFP_KERNEL);
	if (!channel->ivc)
		return -ENOMEM;

	message_size = tegra_ivc_align(MSG_MIN_SZ);
	queue_size = tegra_ivc_total_queue_size(message_size);
	offset = queue_size * index;

	err = tegra_ivc_init(channel->ivc, NULL,
			     bpmp->rx.virt + offset, bpmp->rx.phys + offset,
			     bpmp->tx.virt + offset, bpmp->tx.phys + offset,
			     1, message_size, tegra_bpmp_ivc_notify,
			     bpmp);
	if (err < 0) {
		dev_err(bpmp->dev, "failed to setup IVC for channel %u: %d\n",
			index, err);
		return err;
	}

	init_completion(&channel->completion);
	channel->bpmp = bpmp;

	return 0;
}

static void tegra_bpmp_channel_reset(struct tegra_bpmp_channel *channel)
{
	/* reset the channel state */
	tegra_ivc_reset(channel->ivc);

	/* sync the channel state with BPMP */
	while (tegra_ivc_notified(channel->ivc))
		;
}

static void tegra_bpmp_channel_cleanup(struct tegra_bpmp_channel *channel)
{
	tegra_ivc_cleanup(channel->ivc);
}

static int tegra_bpmp_probe(struct platform_device *pdev)
{
	struct tegra_bpmp_channel *channel;
	struct tegra_bpmp *bpmp;
	unsigned int i;
	char tag[32];
	size_t size;
	int err;

	bpmp = devm_kzalloc(&pdev->dev, sizeof(*bpmp), GFP_KERNEL);
	if (!bpmp)
		return -ENOMEM;

	bpmp->soc = of_device_get_match_data(&pdev->dev);
	bpmp->dev = &pdev->dev;

	bpmp->tx.pool = of_gen_pool_get(pdev->dev.of_node, "shmem", 0);
	if (!bpmp->tx.pool) {
		dev_err(&pdev->dev, "TX shmem pool not found\n");
		return -ENOMEM;
	}

	bpmp->tx.virt = gen_pool_dma_alloc(bpmp->tx.pool, 4096, &bpmp->tx.phys);
	if (!bpmp->tx.virt) {
		dev_err(&pdev->dev, "failed to allocate from TX pool\n");
		return -ENOMEM;
	}

	bpmp->rx.pool = of_gen_pool_get(pdev->dev.of_node, "shmem", 1);
	if (!bpmp->rx.pool) {
		dev_err(&pdev->dev, "RX shmem pool not found\n");
		err = -ENOMEM;
		goto free_tx;
	}

	bpmp->rx.virt = gen_pool_dma_alloc(bpmp->rx.pool, 4096, &bpmp->rx.phys);
	if (!bpmp->rx.pool) {
		dev_err(&pdev->dev, "failed to allocate from RX pool\n");
		err = -ENOMEM;
		goto free_tx;
	}

	INIT_LIST_HEAD(&bpmp->mrqs);
	spin_lock_init(&bpmp->lock);

	bpmp->threaded.count = bpmp->soc->channels.thread.count;
	sema_init(&bpmp->threaded.lock, bpmp->threaded.count);

	size = BITS_TO_LONGS(bpmp->threaded.count) * sizeof(long);

	bpmp->threaded.allocated = devm_kzalloc(&pdev->dev, size, GFP_KERNEL);
	if (!bpmp->threaded.allocated) {
		err = -ENOMEM;
		goto free_rx;
	}

	bpmp->threaded.busy = devm_kzalloc(&pdev->dev, size, GFP_KERNEL);
	if (!bpmp->threaded.busy) {
		err = -ENOMEM;
		goto free_rx;
	}

	bpmp->num_channels = bpmp->soc->channels.cpu_tx.count +
			     bpmp->soc->channels.thread.count +
			     bpmp->soc->channels.cpu_rx.count;

	bpmp->channels = devm_kcalloc(&pdev->dev, bpmp->num_channels,
				      sizeof(*channel), GFP_KERNEL);
	if (!bpmp->channels) {
		err = -ENOMEM;
		goto free_rx;
	}

	/* message channel initialization */
	for (i = 0; i < bpmp->num_channels; i++) {
		struct tegra_bpmp_channel *channel = &bpmp->channels[i];

		err = tegra_bpmp_channel_init(channel, bpmp, i);
		if (err < 0)
			goto cleanup_channels;
	}

	/* mbox registration */
	bpmp->mbox.client.dev = &pdev->dev;
	bpmp->mbox.client.rx_callback = tegra_bpmp_handle_rx;
	bpmp->mbox.client.tx_block = false;
	bpmp->mbox.client.knows_txdone = false;

	bpmp->mbox.channel = mbox_request_channel(&bpmp->mbox.client, 0);
	if (IS_ERR(bpmp->mbox.channel)) {
		err = PTR_ERR(bpmp->mbox.channel);
		dev_err(&pdev->dev, "failed to get HSP mailbox: %d\n", err);
		goto cleanup_channels;
	}

	/* reset message channels */
	for (i = 0; i < bpmp->num_channels; i++) {
		struct tegra_bpmp_channel *channel = &bpmp->channels[i];

		tegra_bpmp_channel_reset(channel);
	}

	err = tegra_bpmp_request_mrq(bpmp, MRQ_PING,
				     tegra_bpmp_mrq_handle_ping, bpmp);
	if (err < 0)
		goto free_mbox;

	err = tegra_bpmp_ping(bpmp);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to ping BPMP: %d\n", err);
		goto free_mrq;
	}

	err = tegra_bpmp_get_firmware_tag(bpmp, tag, sizeof(tag) - 1);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to get firmware tag: %d\n", err);
		goto free_mrq;
	}

	dev_info(&pdev->dev, "firmware: %s\n", tag);

	platform_set_drvdata(pdev, bpmp);

	err = of_platform_default_populate(pdev->dev.of_node, NULL, &pdev->dev);
	if (err < 0)
		goto free_mrq;

	err = tegra_bpmp_init_clocks(bpmp);
	if (err < 0)
		goto free_mrq;

	err = tegra_bpmp_init_resets(bpmp);
	if (err < 0)
		goto free_mrq;

	err = tegra_bpmp_init_powergates(bpmp);
	if (err < 0)
		goto free_mrq;

	err = tegra_bpmp_init_debugfs(bpmp);
	if (err < 0)
		dev_err(&pdev->dev, "debugfs initialization failed: %d\n", err);

	return 0;

free_mrq:
	tegra_bpmp_free_mrq(bpmp, MRQ_PING, bpmp);
free_mbox:
	mbox_free_channel(bpmp->mbox.channel);
cleanup_channels:
	while (i--)
		tegra_bpmp_channel_cleanup(&bpmp->channels[i]);
free_rx:
	gen_pool_free(bpmp->rx.pool, (unsigned long)bpmp->rx.virt, 4096);
free_tx:
	gen_pool_free(bpmp->tx.pool, (unsigned long)bpmp->tx.virt, 4096);
	return err;
}

static const struct tegra_bpmp_soc tegra186_soc = {
	.channels = {
		.cpu_tx = {
			.offset = 0,
			.count = 6,
			.timeout = 60 * USEC_PER_SEC,
		},
		.thread = {
			.offset = 6,
			.count = 7,
			.timeout = 600 * USEC_PER_SEC,
		},
		.cpu_rx = {
			.offset = 13,
			.count = 1,
			.timeout = 0,
		},
	},
	.num_resets = 193,
};

static const struct of_device_id tegra_bpmp_match[] = {
	{ .compatible = "nvidia,tegra186-bpmp", .data = &tegra186_soc },
	{ }
};

static struct platform_driver tegra_bpmp_driver = {
	.driver = {
		.name = "tegra-bpmp",
		.of_match_table = tegra_bpmp_match,
	},
	.probe = tegra_bpmp_probe,
};

static int __init tegra_bpmp_init(void)
{
	return platform_driver_register(&tegra_bpmp_driver);
}
core_initcall(tegra_bpmp_init);

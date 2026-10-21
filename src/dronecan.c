/*
 * Copyright (c) 2026 3DR, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_dronecan

#include <dronecan.h>

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/slist.h>

#include <canard.h>

LOG_MODULE_REGISTER(dronecan, CONFIG_DRONECAN_LOG_LEVEL);

#define DRONECAN_MAX_NODES 128 // node IDs 0..127

struct dronecan_config {
	const struct device *can_bus[1]; // one bus for now; index = interface id
	uint8_t num_ifaces;
	uint8_t node_id;
	uint8_t *arena;
	size_t arena_size;
	struct k_msgq *rx_msgq;
	k_thread_stack_t *rx_stack;
	k_thread_stack_t *hk_stack;
};

/*
 * Lock order: canard_mtx -> table_mtx, canard_mtx -> sub_mtx.
 * Never call a subscription callback or take canard_mtx while holding sub_mtx.
 */
struct dronecan_data {
	CanardInstance canard;
	struct k_mutex canard_mtx;
	struct k_mutex table_mtx;
	struct k_mutex sub_mtx;
	struct k_condvar sub_cv;
	sys_slist_t subs;
	struct dronecan_subscription *in_flight; // callback running, guarded by sub_mtx

	struct k_thread rx_thread;
	struct k_thread hk_thread;

	bool started;
	uint8_t node_id;
	uint8_t health;
	uint8_t mode;
	uint16_t vendor_status;

	struct dronecan_stats stats;
#ifdef CONFIG_DRONECAN_NODE_TABLE
	struct dronecan_node_entry node_table[DRONECAN_MAX_NODES];
#endif
};

static int dronecan_init(const struct device *dev)
{
	const struct dronecan_config *cfg = dev->config;
	struct dronecan_data *data = dev->data;

	k_mutex_init(&data->canard_mtx);
	k_mutex_init(&data->table_mtx);
	k_mutex_init(&data->sub_mtx);
	k_condvar_init(&data->sub_cv);
	sys_slist_init(&data->subs);

	data->node_id = cfg->node_id;
	data->health = DRONECAN_HEALTH_OK;
	data->mode = DRONECAN_MODE_INITIALIZATION;

	LOG_DBG("%s: node %u on %s, arena %zu bytes", dev->name, data->node_id,
		cfg->can_bus[0]->name, cfg->arena_size);

	return 0;
}


uint8_t dronecan_get_node_id(const struct device *dev)
{
	const struct dronecan_data *data = dev->data;

	return data->node_id;
}

int dronecan_set_health(const struct device *dev, uint8_t health)
{
	struct dronecan_data *data = dev->data;

	if (health > DRONECAN_HEALTH_CRITICAL) {
		return -EINVAL;
	}
	data->health = health;
	return 0;
}

int dronecan_set_mode(const struct device *dev, uint8_t mode)
{
	struct dronecan_data *data = dev->data;

	if (mode > DRONECAN_MODE_OFFLINE) {
		return -EINVAL;
	}
	data->mode = mode;
	return 0;
}

int dronecan_set_vendor_status(const struct device *dev, uint16_t status)
{
	struct dronecan_data *data = dev->data;

	data->vendor_status = status;
	return 0;
}

int dronecan_get_node(const struct device *dev, uint8_t id, struct dronecan_node_entry *out)
{
#ifdef CONFIG_DRONECAN_NODE_TABLE
	struct dronecan_data *data = dev->data;

	if (out == NULL || id >= DRONECAN_MAX_NODES) {
		return -EINVAL;
	}
	k_mutex_lock(&data->table_mtx, K_FOREVER);
	*out = data->node_table[id];
	k_mutex_unlock(&data->table_mtx);
	return 0;
#else
	ARG_UNUSED(dev);
	ARG_UNUSED(id);
	ARG_UNUSED(out);
	return -ENOTSUP;
#endif
}

void dronecan_get_stats(const struct device *dev, struct dronecan_stats *out)
{
	const struct dronecan_data *data = dev->data;

	*out = data->stats; // TODO add libcanard pool stats once initialized
}

int dronecan_start(const struct device *dev)
{
	ARG_UNUSED(dev);
	return -ENOSYS;
}

int dronecan_stop(const struct device *dev)
{
	ARG_UNUSED(dev);
	return -ENOSYS;
}

int dronecan_broadcast(const struct device *dev, const struct dronecan_msg_desc *desc,
		       enum dronecan_priority p, const void *payload, uint16_t len)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(desc);
	ARG_UNUSED(p);
	ARG_UNUSED(payload);
	ARG_UNUSED(len);
	return -ENOSYS;
}

int dronecan_subscribe(const struct device *dev, struct dronecan_subscription *sub,
		       const struct dronecan_msg_desc *desc, dronecan_rx_cb_t cb, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sub);
	ARG_UNUSED(desc);
	ARG_UNUSED(cb);
	ARG_UNUSED(user_data);
	return -ENOSYS;
}

int dronecan_unsubscribe(const struct device *dev, struct dronecan_subscription *sub)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sub);
	return -ENOSYS;
}


#define DRONECAN_CAN_BUS(node_id, prop, idx) DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx))

#define DRONECAN_DEFINE(n)                                                                         \
	BUILD_ASSERT(DT_INST_PROP_LEN(n, can_buses) == 1,                                          \
		     "only one CAN bus per DroneCAN node is supported for now");                   \
	BUILD_ASSERT(DT_INST_PROP(n, node_id) <= 127, "node-id must be 0..127");                   \
	BUILD_ASSERT(DT_INST_PROP(n, mem_blocks) > 0, "mem-blocks must be > 0");                   \
                                                                                                   \
	/* 8-byte aligned: CanardRxState holds a uint64_t, unaligned faults on Cortex-M7 */        \
	static uint8_t dronecan_arena_##n[DT_INST_PROP(n, mem_blocks) * CANARD_MEM_BLOCK_SIZE]     \
		__aligned(8);                                                                      \
	CAN_MSGQ_DEFINE(dronecan_rx_msgq_##n, CONFIG_DRONECAN_RX_MSGQ_SIZE);                       \
	static K_THREAD_STACK_DEFINE(dronecan_rx_stack_##n, CONFIG_DRONECAN_RX_STACK_SIZE);        \
	static K_THREAD_STACK_DEFINE(dronecan_hk_stack_##n,                                        \
				     CONFIG_DRONECAN_HOUSEKEEPING_STACK_SIZE);                     \
                                                                                                   \
	static struct dronecan_data dronecan_data_##n;                                             \
	static const struct dronecan_config dronecan_config_##n = {                                \
		.can_bus = {DT_INST_FOREACH_PROP_ELEM_SEP(n, can_buses, DRONECAN_CAN_BUS, (,))},  \
		.num_ifaces = DT_INST_PROP_LEN(n, can_buses),                                      \
		.node_id = DT_INST_PROP(n, node_id),                                               \
		.arena = dronecan_arena_##n,                                                       \
		.arena_size = sizeof(dronecan_arena_##n),                                          \
		.rx_msgq = &dronecan_rx_msgq_##n,                                                  \
		.rx_stack = dronecan_rx_stack_##n,                                                 \
		.hk_stack = dronecan_hk_stack_##n,                                                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, dronecan_init, NULL, &dronecan_data_##n, &dronecan_config_##n,    \
			      POST_KERNEL, CONFIG_DRONECAN_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(DRONECAN_DEFINE)

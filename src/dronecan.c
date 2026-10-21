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
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/slist.h>

#include <canard.h>
#include <uavcan.protocol.NodeStatus.h>

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
	size_t rx_stack_size;
	k_thread_stack_t *tx_stack;
	size_t tx_stack_size;
	k_thread_stack_t *hk_stack;
	size_t hk_stack_size;
};

struct dronecan_pub {
	uint16_t id;
	uint8_t transfer_id;
};

/*
 * Lock order: lifecycle_mtx -> canard_mtx -> table_mtx, canard_mtx -> sub_mtx.
 * Never call a subscription callback or take canard_mtx while holding sub_mtx.
 * Never log or call can_send() with canard_mtx held.
 */
struct dronecan_data {
	const struct device *dev;
	CanardInstance canard;
	struct k_mutex lifecycle_mtx;
	struct k_mutex canard_mtx;
	struct k_mutex table_mtx;
	struct k_mutex sub_mtx;
	struct k_condvar sub_cv;
	sys_slist_t subs;
	struct dronecan_subscription *in_flight; // callback running, guarded by sub_mtx

	struct k_thread rx_thread;
	struct k_work_q tx_workq; // one sender, so frames leave in queue order
	struct k_work tx_work;
	struct k_sem tx_done;
	int tx_status;
	bool tx_stuck; // only used by the TX work
	struct k_thread hk_thread;

	atomic_t started;
	int rx_filter_id;
	bool owns_controller; // we called can_start(), so we may call can_stop()

	uint8_t health;
	uint8_t mode;
	uint16_t vendor_status;

	struct dronecan_pub pub[CONFIG_DRONECAN_MAX_PUBLISHED_TYPES];
	uint8_t num_pub;

	struct dronecan_stats stats;
#ifdef CONFIG_DRONECAN_NODE_TABLE
	struct dronecan_node_entry node_table[DRONECAN_MAX_NODES];
#endif
};

// libcanard callbacks

static bool dronecan_should_accept(const CanardInstance *ins, uint64_t *out_signature,
				   uint16_t data_type_id, CanardTransferType transfer_type,
				   uint8_t source_node_id)
{
	ARG_UNUSED(ins);
	ARG_UNUSED(out_signature);
	ARG_UNUSED(data_type_id);
	ARG_UNUSED(transfer_type);
	ARG_UNUSED(source_node_id);

	return false;
}

static void dronecan_on_reception(CanardInstance *ins, CanardRxTransfer *transfer)
{
	ARG_UNUSED(ins);
	ARG_UNUSED(transfer);
}

// TX path

// Move queued libcanard frames into Zephyr frames. Call with canard_mtx held.
static int drain_tx(struct dronecan_data *data, struct can_frame *out, int max)
{
	int n = 0;

	while (n < max) {
		const CanardCANFrame *cf = canardPeekTxQueue(&data->canard);

		if (cf == NULL) {
			break;
		}
		out[n] = (struct can_frame){
			.id = cf->id & CANARD_CAN_EXT_ID_MASK,
			.flags = CAN_FRAME_IDE,
			.dlc = can_bytes_to_dlc(cf->data_len),
		};
		memcpy(out[n].data, cf->data, cf->data_len);
		canardPopTxQueue(&data->canard);
		n++;
	}
	return n;
}

static void tx_done_cb(const struct device *can, int error, void *user_data)
{
	struct dronecan_data *data = user_data;

	ARG_UNUSED(can);
	data->tx_status = error;
	k_sem_give(&data->tx_done);
}

// Drop the frames libcanard has queued. Call with canard_mtx held.
static int drop_tx(struct dronecan_data *data)
{
	int n = 0;

	while (canardPeekTxQueue(&data->canard) != NULL) {
		canardPopTxQueue(&data->canard);
		n++;
	}
	return n;
}

// One frame at a time. Without an ACK the frame is dropped after a timeout instead of blocking.
static void tx_work_handler(struct k_work *work)
{
	struct dronecan_data *data = CONTAINER_OF(work, struct dronecan_data, tx_work);
	const struct dronecan_config *cfg = data->dev->config;
	struct can_frame frames[CONFIG_DRONECAN_TX_DRAIN_MAX];
	int n;

	if (data->tx_stuck && k_sem_take(&data->tx_done, K_NO_WAIT) == 0) {
		data->tx_stuck = false;
		if (data->tx_status == 0) {
			data->stats.tx++;
		} else {
			data->stats.tx_err++;
		}
		LOG_INF("%s: the CAN bus answers again", data->dev->name);
	}

	do {
		k_mutex_lock(&data->canard_mtx, K_FOREVER);
		if (data->tx_stuck) {
			data->stats.tx_err += drop_tx(data);
			k_mutex_unlock(&data->canard_mtx);
			return;
		}
		n = drain_tx(data, frames, ARRAY_SIZE(frames));
		k_mutex_unlock(&data->canard_mtx);

		for (int i = 0; i < n; i++) {
			int ret = can_send(cfg->can_bus[0], &frames[i],
					   K_MSEC(CONFIG_DRONECAN_TX_TIMEOUT_MS), tx_done_cb, data);

			if (ret != 0) {
				data->stats.tx_err++;
				LOG_DBG("can_send failed: %d", ret);
				continue;
			}

			if (k_sem_take(&data->tx_done, K_MSEC(CONFIG_DRONECAN_TX_ACK_TIMEOUT_MS)) != 0) {
				data->tx_stuck = true;
				data->stats.tx_err += n - i - 1;
				k_mutex_lock(&data->canard_mtx, K_FOREVER);
				data->stats.tx_err += drop_tx(data);
				k_mutex_unlock(&data->canard_mtx);
				LOG_WRN("%s: no ACK on the CAN bus, dropping frames until it answers",
					data->dev->name);
				return;
			}

			if (data->tx_status == 0) {
				data->stats.tx++;
			} else {
				data->stats.tx_err++;
			}
		}
	} while (n == ARRAY_SIZE(frames));
}

// RX path

static void rx_thread_fn(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	const struct dronecan_config *cfg = dev->config;
	struct dronecan_data *data = dev->data;
	struct can_frame frame;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		// Fails (-ENOMSG) when dronecan_stop() purges the queue while we wait.
		if (k_msgq_get(cfg->rx_msgq, &frame, K_FOREVER) != 0) {
			continue;
		}

		if (!atomic_get(&data->started) || (frame.flags & CAN_FRAME_RTR) != 0U) {
			continue;
		}

		CanardCANFrame cf = {
			.id = frame.id | CANARD_CAN_FRAME_EFF,
			.data_len = can_dlc_to_bytes(frame.dlc),
			.iface_id = 0, // TODO: interface index when multi-bus lands
		};

		memcpy(cf.data, frame.data, cf.data_len);
		LOG_DBG("rx id=0x%08x len=%u tail=0x%02x", frame.id, cf.data_len,
			cf.data_len > 0 ? cf.data[cf.data_len - 1] : 0);
		LOG_HEXDUMP_DBG(cf.data, cf.data_len, "rx data");

		k_mutex_lock(&data->canard_mtx, K_FOREVER);
		canardHandleRxFrame(&data->canard, &cf, k_uptime_get() * 1000ULL);
		bool tx_pending = canardPeekTxQueue(&data->canard) != NULL;

		k_mutex_unlock(&data->canard_mtx);

		if (tx_pending) {
			k_work_submit_to_queue(&data->tx_workq, &data->tx_work);
		}
	}
}

// Housekeeping

static const struct dronecan_msg_desc node_status_desc = {
	.id = UAVCAN_PROTOCOL_NODESTATUS_ID,
	.signature = UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
	.max_size = UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE,
};

// Broadcasts NodeStatus and drops stale partial transfers.
static void hk_thread_fn(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct dronecan_data *data = dev->data;
	int64_t next = k_uptime_get();

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		next += CONFIG_DRONECAN_NODE_STATUS_PERIOD_MS;
		k_sleep(K_TIMEOUT_ABS_MS(next));

		if (!atomic_get(&data->started)) {
			continue;
		}

		struct uavcan_protocol_NodeStatus ns = {0};
		uint8_t buf[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
		uint8_t node_id;

		// One critical section, so health, mode and vendor status are consistent.
		k_mutex_lock(&data->canard_mtx, K_FOREVER);
		node_id = canardGetLocalNodeID(&data->canard);
		ns.health = data->health;
		ns.mode = data->mode;
		ns.vendor_specific_status_code = data->vendor_status;
		canardCleanupStaleTransfers(&data->canard, k_uptime_get() * 1000ULL);
		k_mutex_unlock(&data->canard_mtx);

		if (node_id == DRONECAN_NODE_ID_ANON) {
			continue; // anonymous nodes must not broadcast NodeStatus
		}

		ns.uptime_sec = k_uptime_get_32() / 1000U;
		uint32_t len = uavcan_protocol_NodeStatus_encode(&ns, buf);
		int ret = dronecan_broadcast(dev, &node_status_desc, DRONECAN_PRIORITY_LOW, buf,
					     len);

		if (ret != 0) {
			LOG_DBG("NodeStatus broadcast failed: %d", ret); // e.g. stopped meanwhile
		}
	}
}

// Init

static int dronecan_init(const struct device *dev)
{
	const struct dronecan_config *cfg = dev->config;
	struct dronecan_data *data = dev->data;
	const struct k_work_queue_config qcfg = {.name = "dronecan_tx"};

	data->dev = dev;

	k_mutex_init(&data->lifecycle_mtx);
	k_mutex_init(&data->canard_mtx);
	k_mutex_init(&data->table_mtx);
	k_mutex_init(&data->sub_mtx);
	k_condvar_init(&data->sub_cv);
	sys_slist_init(&data->subs);
	k_sem_init(&data->tx_done, 0, 1);

	data->health = DRONECAN_HEALTH_OK;
	data->mode = DRONECAN_MODE_INITIALIZATION;

	// The threads live as long as the device: start/stop only gate the traffic.
	k_work_init(&data->tx_work, tx_work_handler);
	k_work_queue_init(&data->tx_workq);
	k_work_queue_start(&data->tx_workq, cfg->tx_stack, cfg->tx_stack_size,
			   CONFIG_DRONECAN_THREAD_PRIORITY, &qcfg);

	k_thread_create(&data->rx_thread, cfg->rx_stack, cfg->rx_stack_size, rx_thread_fn,
			(void *)dev, NULL, NULL, CONFIG_DRONECAN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&data->rx_thread, "dronecan_rx");

	k_thread_create(&data->hk_thread, cfg->hk_stack, cfg->hk_stack_size, hk_thread_fn,
			(void *)dev, NULL, NULL, CONFIG_DRONECAN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&data->hk_thread, "dronecan_hk");

	LOG_DBG("%s: node %u on %s, arena %zu bytes", dev->name, cfg->node_id,
		cfg->can_bus[0]->name, cfg->arena_size);

	return 0;
}

// Lifecycle

int dronecan_start(const struct device *dev)
{
	const struct dronecan_config *cfg = dev->config;
	struct dronecan_data *data = dev->data;
	const struct device *can = cfg->can_bus[0];
	static const struct can_filter filter = {.id = 0, .mask = 0, .flags = CAN_FILTER_IDE};
	int ret;

	k_mutex_lock(&data->lifecycle_mtx, K_FOREVER);

	if (atomic_get(&data->started)) {
		ret = -EALREADY;
		goto out;
	}
	if (!device_is_ready(can)) {
		ret = -ENODEV;
		goto out;
	}

	// The bitrate comes from the controller's devicetree node, classic CAN only.
	ret = can_set_mode(can, IS_ENABLED(CONFIG_DRONECAN_LOOPBACK_MODE) ? CAN_MODE_LOOPBACK
									 : CAN_MODE_NORMAL);
	if (ret != 0 && ret != -EBUSY) { // -EBUSY: someone else already started it
		LOG_ERR("can_set_mode failed: %d", ret);
		goto out;
	}

	ret = can_add_rx_filter_msgq(can, cfg->rx_msgq, &filter);
	if (ret < 0) {
		LOG_ERR("can_add_rx_filter_msgq failed: %d", ret);
		goto out;
	}
	data->rx_filter_id = ret;

	ret = can_start(can);
	if (ret == 0) {
		data->owns_controller = true;
	} else if (ret == -EALREADY) {
		data->owns_controller = false;
	} else {
		LOG_ERR("can_start failed: %d", ret);
		can_remove_rx_filter(can, data->rx_filter_id);
		goto out;
	}

	memset(&data->stats, 0, sizeof(data->stats));
	data->tx_stuck = false;
	k_sem_reset(&data->tx_done);

	k_mutex_lock(&data->canard_mtx, K_FOREVER);
	canardInit(&data->canard, cfg->arena, cfg->arena_size, dronecan_on_reception,
		   dronecan_should_accept, data);
	if (cfg->node_id != DRONECAN_NODE_ID_ANON) {
		canardSetLocalNodeID(&data->canard, cfg->node_id);
	}
	memset(data->pub, 0, sizeof(data->pub));
	data->num_pub = 0;
	data->mode = DRONECAN_MODE_OPERATIONAL;
	k_mutex_unlock(&data->canard_mtx);

	atomic_set(&data->started, 1);

	if (cfg->node_id == DRONECAN_NODE_ID_ANON) {
		LOG_WRN("%s: node ID 0 (anonymous): no NodeStatus is sent and dynamic node ID "
			"allocation is not implemented yet",
			dev->name);
	}
	ret = 0;
out:
	k_mutex_unlock(&data->lifecycle_mtx);
	return ret;
}

int dronecan_stop(const struct device *dev)
{
	const struct dronecan_config *cfg = dev->config;
	struct dronecan_data *data = dev->data;
	const struct device *can = cfg->can_bus[0];
	struct k_work_sync sync;
	int ret = 0;

	k_mutex_lock(&data->lifecycle_mtx, K_FOREVER);

	if (!atomic_get(&data->started)) {
		ret = -EALREADY;
		goto out;
	}

	atomic_set(&data->started, 0);
	can_remove_rx_filter(can, data->rx_filter_id);
	k_msgq_purge(cfg->rx_msgq);
	k_work_flush(&data->tx_work, &sync); // let queued frames go out first

	if (data->owns_controller) {
		ret = can_stop(can);
		if (ret == -EALREADY) {
			ret = 0;
		}
		data->owns_controller = false;
	}
	k_mutex_lock(&data->canard_mtx, K_FOREVER);
	data->mode = DRONECAN_MODE_OFFLINE;
	k_mutex_unlock(&data->canard_mtx);
out:
	k_mutex_unlock(&data->lifecycle_mtx);
	return ret;
}

uint8_t dronecan_get_node_id(const struct device *dev)
{
	const struct dronecan_config *cfg = dev->config;
	struct dronecan_data *data = dev->data;
	uint8_t id = cfg->node_id;

	if (atomic_get(&data->started)) {
		k_mutex_lock(&data->canard_mtx, K_FOREVER);
		id = canardGetLocalNodeID(&data->canard);
		k_mutex_unlock(&data->canard_mtx);
	}
	return id;
}

// Node status

int dronecan_set_health(const struct device *dev, uint8_t health)
{
	struct dronecan_data *data = dev->data;

	if (health > DRONECAN_HEALTH_CRITICAL) {
		return -EINVAL;
	}
	k_mutex_lock(&data->canard_mtx, K_FOREVER);
	data->health = health;
	k_mutex_unlock(&data->canard_mtx);
	return 0;
}

int dronecan_set_mode(const struct device *dev, uint8_t mode)
{
	struct dronecan_data *data = dev->data;

	if (mode > DRONECAN_MODE_SOFTWARE_UPDATE && mode != DRONECAN_MODE_OFFLINE) {
		return -EINVAL; // the DSDL defines 0-3 and 7
	}
	k_mutex_lock(&data->canard_mtx, K_FOREVER);
	data->mode = mode;
	k_mutex_unlock(&data->canard_mtx);
	return 0;
}

int dronecan_set_vendor_status(const struct device *dev, uint16_t status)
{
	struct dronecan_data *data = dev->data;

	k_mutex_lock(&data->canard_mtx, K_FOREVER);
	data->vendor_status = status;
	k_mutex_unlock(&data->canard_mtx);
	return 0;
}

// Publish

// Persistent transfer ID for a message type. Call with canard_mtx held.
static uint8_t *transfer_id_for(struct dronecan_data *data, uint16_t id)
{
	for (uint8_t i = 0; i < data->num_pub; i++) {
		if (data->pub[i].id == id) {
			return &data->pub[i].transfer_id;
		}
	}
	if (data->num_pub >= ARRAY_SIZE(data->pub)) {
		return NULL;
	}
	data->pub[data->num_pub].id = id;
	data->pub[data->num_pub].transfer_id = 0;
	return &data->pub[data->num_pub++].transfer_id;
}

int dronecan_broadcast(const struct device *dev, const struct dronecan_msg_desc *desc,
		       enum dronecan_priority p, const void *payload, uint16_t len)
{
	struct dronecan_data *data = dev->data;
	uint8_t *tid;
	int16_t ret;

	if (desc == NULL || (payload == NULL && len > 0) || len > desc->max_size) {
		return -EINVAL;
	}
	if (!atomic_get(&data->started)) {
		return -ENETDOWN;
	}

	k_mutex_lock(&data->canard_mtx, K_FOREVER);
	tid = transfer_id_for(data, desc->id);
	if (tid == NULL) {
		ret = -CANARD_ERROR_OUT_OF_MEMORY;
	} else {
		ret = canardBroadcast(&data->canard, desc->signature, desc->id, tid, (uint8_t)p,
				      payload, len);
	}
	k_mutex_unlock(&data->canard_mtx);

	if (ret < 0) {
		switch (-ret) {
		case CANARD_ERROR_INVALID_ARGUMENT:
			return -EINVAL;
		case CANARD_ERROR_OUT_OF_MEMORY:
			return -ENOMEM;
		case CANARD_ERROR_NODE_ID_NOT_SET:
			return -EPERM;
		default:
			return -EIO;
		}
	}

	k_work_submit_to_queue(&data->tx_workq, &data->tx_work);
	return 0; // queued: transmission errors show up in the statistics
}

// Queries

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
	struct dronecan_data *data = dev->data;

	*out = data->stats; // counters are not read atomically

	if (atomic_get(&data->started)) {
		k_mutex_lock(&data->canard_mtx, K_FOREVER);
		CanardPoolAllocatorStatistics s = canardGetPoolAllocatorStatistics(&data->canard);

		k_mutex_unlock(&data->canard_mtx);

		out->mem_capacity = s.capacity_blocks;
		out->mem_current = s.current_usage_blocks;
		out->mem_peak = s.peak_usage_blocks;
	}
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

// Instances

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
	static K_THREAD_STACK_DEFINE(dronecan_tx_stack_##n, CONFIG_DRONECAN_TX_STACK_SIZE);        \
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
		.rx_stack_size = K_THREAD_STACK_SIZEOF(dronecan_rx_stack_##n),                     \
		.tx_stack = dronecan_tx_stack_##n,                                                 \
		.tx_stack_size = K_THREAD_STACK_SIZEOF(dronecan_tx_stack_##n),                     \
		.hk_stack = dronecan_hk_stack_##n,                                                 \
		.hk_stack_size = K_THREAD_STACK_SIZEOF(dronecan_hk_stack_##n),                     \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, dronecan_init, NULL, &dronecan_data_##n, &dronecan_config_##n,    \
			      POST_KERNEL, CONFIG_DRONECAN_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(DRONECAN_DEFINE)

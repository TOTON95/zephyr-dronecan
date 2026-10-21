/*
 * Copyright (c) 2026 3DR, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <dronecan.h>
#include <uavcan.protocol.NodeStatus.h>

LOG_MODULE_REGISTER(dronecan_sample, LOG_LEVEL_INF);

static const struct device *const dc = DEVICE_DT_GET(DT_NODELABEL(dronecan0));

static void show_stats(const char *when)
{
	struct dronecan_stats s;

	dronecan_get_stats(dc, &s);
	LOG_INF("%s: tx=%u tx_err=%u pool %u/%u (peak %u)", when, s.tx, s.tx_err, s.mem_current,
		s.mem_capacity, s.mem_peak);
}

int main(void)
{
	static const struct dronecan_msg_desc node_status = {
		.id = UAVCAN_PROTOCOL_NODESTATUS_ID,
		.signature = UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
		.max_size = UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE,
	};

	static const struct dronecan_msg_desc big = {.id = 1000, .signature = 0x1234, .max_size = 64};
	uint8_t payload[20] = {0};

	LOG_INF("dronecan sample");

	if (!device_is_ready(dc)) {
		LOG_ERR("%s is not ready", dc->name);
		return -ENODEV;
	}

	LOG_INF("broadcast before start: %d", dronecan_broadcast(dc, &node_status,
		DRONECAN_PRIORITY_LOW, payload, 7));
	LOG_INF("stop before start: %d", dronecan_stop(dc));

	LOG_INF("start: %d", dronecan_start(dc));
	LOG_INF("second start: %d", dronecan_start(dc));
	LOG_INF("node id: %u", dronecan_get_node_id(dc));

	LOG_INF("broadcast NodeStatus-sized: %d",
		dronecan_broadcast(dc, &node_status, DRONECAN_PRIORITY_LOW, payload, 7));
	LOG_INF("broadcast 20 bytes: %d",
		dronecan_broadcast(dc, &big, DRONECAN_PRIORITY_LOW, payload, sizeof(payload)));
	LOG_INF("broadcast too long: %d",
		dronecan_broadcast(dc, &big, DRONECAN_PRIORITY_LOW, payload, 65));

	k_sleep(K_MSEC(300));
	show_stats("after traffic");

	LOG_INF("stop: %d", dronecan_stop(dc));
	LOG_INF("broadcast after stop: %d",
		dronecan_broadcast(dc, &node_status, DRONECAN_PRIORITY_LOW, payload, 7));
	LOG_INF("restart: %d", dronecan_start(dc));
	LOG_INF("broadcast after restart: %d",
		dronecan_broadcast(dc, &node_status, DRONECAN_PRIORITY_LOW, payload, 7));
	k_sleep(K_MSEC(300));
	show_stats("after restart");

	while (1) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}

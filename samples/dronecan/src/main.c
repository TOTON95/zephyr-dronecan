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

BUILD_ASSERT(DT_NODE_HAS_STATUS(DT_NODELABEL(dronecan0), okay),
	     "dronecan0 node missing or disabled");

/* Stand-in handle until Step 2 defines the dronecan device. */
static const struct device *const dc = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

static struct dronecan_subscription node_status_sub;

static void on_node_status(const struct device *dev, const struct dronecan_transfer *t,
			   void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	LOG_INF("NodeStatus from node %u (%u bytes)", t->source_node_id, t->payload_len);
}

int main(void)
{
	static const struct dronecan_msg_desc node_status = {
		.id = UAVCAN_PROTOCOL_NODESTATUS_ID,
		.signature = UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
		.max_size = UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE,
	};
	int ret;

	LOG_INF("dronecan sample");

	ret = dronecan_start(dc);
	LOG_INF("dronecan_start: %d", ret);

	ret = dronecan_subscribe(dc, &node_status_sub, &node_status, on_node_status, NULL);
	LOG_INF("dronecan_subscribe: %d", ret);

	while (1) {
		dronecan_set_health(dc, DRONECAN_HEALTH_OK);
		LOG_INF("node id: %u", dronecan_get_node_id(dc));
		k_sleep(K_SECONDS(1));
	}

	return 0;
}

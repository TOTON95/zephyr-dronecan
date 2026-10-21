/*
 * Copyright (c) 2026 3DR, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include <dronecan.h>
#include <uavcan.protocol.NodeStatus.h>

LOG_MODULE_REGISTER(dronecan_sample, LOG_LEVEL_INF);

static const struct device *const dc = DEVICE_DT_GET(DT_NODELABEL(dronecan0));

#if DT_HAS_CHOSEN(zephyr_console) && DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart)
#define CONSOLE_IS_CDC_ACM 1
#endif

// A USB console shows nothing until the host opens the port: wait for it (at most 15 s).
static void wait_for_console(void)
{
#ifdef CONSOLE_IS_CDC_ACM
	const struct device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t dtr = 0;

	for (int i = 0; i < 150 && dtr == 0; i++) {
		uart_line_ctrl_get(console, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(100));
	}
	k_sleep(K_MSEC(200)); // let the terminal settle
#endif
}

static void show_stats(const char *when)
{
	struct dronecan_stats s;

	dronecan_get_stats(dc, &s);
	LOG_INF("%s: tx=%u tx_err=%u pool %u/%u (peak %u), IDs granted %u (last %u)", when, s.tx,
		s.tx_err, s.mem_current, s.mem_capacity, s.mem_peak, s.alloc_grant, s.last_grant_id);
}

static void show_nodes(void)
{
	struct dronecan_node_entry e;

	for (int id = 1; id < 128; id++) {
		if (dronecan_get_node(dc, id, &e) == 0 && e.active) {
			LOG_INF("node %d: up %u s, health %u, mode %u, vendor 0x%04x", id, e.uptime_sec,
				e.health, e.mode, e.vendor_status);
		}
	}
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

	wait_for_console();
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

	// NodeStatus keeps going out once a second: tx should grow by one each time.
	while (1) {
		show_stats("running");
		show_nodes();
		k_sleep(K_SECONDS(2));
	}

	return 0;
}

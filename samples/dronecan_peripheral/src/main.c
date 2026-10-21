/*
 * Copyright (c) 2026 3DR, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include <dronecan.h>

LOG_MODULE_REGISTER(dronecan_peripheral_sample, LOG_LEVEL_INF);

static const struct device *const dc = DEVICE_DT_GET(DT_NODELABEL(dronecan0));

#if DT_NODE_EXISTS(DT_NODELABEL(dronecan_fc))
// On native_sim a second node plays the flight controller that grants the ID
#define HAS_ALLOCATOR 1
static const struct device *const fc = DEVICE_DT_GET(DT_NODELABEL(dronecan_fc));
#endif

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

static const char *can_state_name(enum can_state state)
{
	static const char *const names[] = {
		[CAN_STATE_ERROR_ACTIVE] = "error-active",
		[CAN_STATE_ERROR_WARNING] = "error-warning",
		[CAN_STATE_ERROR_PASSIVE] = "error-passive",
		[CAN_STATE_BUS_OFF] = "bus-off",
		[CAN_STATE_STOPPED] = "stopped",
	};

	return (state < ARRAY_SIZE(names) && names[state] != NULL) ? names[state] : "unknown";
}

int main(void)
{
	struct dronecan_stats s;
	int ret;

	wait_for_console();
	LOG_INF("dronecan peripheral sample");

	if (!device_is_ready(dc)) {
		LOG_ERR("%s is not ready", dc->name);
		return -ENODEV;
	}

#ifdef HAS_ALLOCATOR
	LOG_INF("allocator start: %d", dronecan_start(fc));
#endif
	LOG_INF("start: %d, node id %u", dronecan_start(dc), dronecan_get_node_id(dc));

	// The node is anonymous and silent until an allocator on the bus grants it an ID
	while ((ret = dronecan_dnia_request(dc, K_SECONDS(10))) != 0) {
		LOG_WRN("no node ID yet: %d", ret);
	}
	LOG_INF("node id: %u", dronecan_get_node_id(dc));

	while (1) {
		dronecan_get_stats(dc, &s);
		LOG_INF("tx=%u tx_err=%u, controller %s (errors tx %u rx %u)", s.tx, s.tx_err,
			can_state_name(s.can_state), s.can_tx_errors, s.can_rx_errors);
		k_sleep(K_SECONDS(2));
	}

	return 0;
}

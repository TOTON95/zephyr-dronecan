/*
 * Copyright (c) 2026 3DR, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <dronecan.h>

LOG_MODULE_REGISTER(dronecan_shell_sample, LOG_LEVEL_INF);

static const struct device *const dc = DEVICE_DT_GET(DT_NODELABEL(dronecan0));

int main(void)
{
	int ret;

	if (!device_is_ready(dc)) {
		LOG_ERR("%s is not ready", dc->name);
		return -ENODEV;
	}

	ret = dronecan_start(dc);
	if (ret != 0) {
		LOG_ERR("start: %d", ret);
		return ret;
	}
	LOG_INF("node %u started, type 'dc' for the commands", dronecan_get_node_id(dc));

	return 0;
}

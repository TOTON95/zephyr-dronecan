/*
 * Copyright (c) 2026 3DR, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <dronecan.h>

#include <errno.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include <canard.h>
#include <dronecan_msgs.h>

LOG_MODULE_REGISTER(dronecan, CONFIG_DRONECAN_LOG_LEVEL);

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

uint8_t dronecan_get_node_id(const struct device *dev)
{
	ARG_UNUSED(dev);
	return DRONECAN_NODE_ID_ANON;
}

int dronecan_set_health(const struct device *dev, uint8_t health)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(health);
	return -ENOSYS;
}

int dronecan_set_mode(const struct device *dev, uint8_t mode)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(mode);
	return -ENOSYS;
}

int dronecan_set_vendor_status(const struct device *dev, uint16_t status)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(status);
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

int dronecan_get_node(const struct device *dev, uint8_t id, struct dronecan_node_entry *out)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(id);
	ARG_UNUSED(out);
	return -ENOSYS;
}

void dronecan_get_stats(const struct device *dev, struct dronecan_stats *out)
{
	ARG_UNUSED(dev);
	memset(out, 0, sizeof(*out));
}

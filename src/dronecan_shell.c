/*
 * Copyright (c) 2026 3DR, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#include <dronecan.h>

#include "dronecan_msg_table.h"

#define DC_DEV(node) DEVICE_DT_GET(node),
static const struct device *const dc_devs[] = {
	DT_FOREACH_STATUS_OKAY(zephyr_dronecan, DC_DEV)
};

static const struct device *find_dev(const struct shell *sh, const char *name)
{
	for (size_t i = 0; i < ARRAY_SIZE(dc_devs); i++) {
		if (strcmp(dc_devs[i]->name, name) == 0) {
			return dc_devs[i];
		}
	}
	shell_error(sh, "%s is not a DroneCAN device (see 'dc list')", name);
	return NULL;
}

static const struct dronecan_msg_entry *find_msg(const struct shell *sh, const char *name)
{
	for (size_t i = 0; i < dronecan_msg_table_len; i++) {
		if (strcmp(dronecan_msg_table[i].name, name) == 0) {
			return &dronecan_msg_table[i];
		}
	}
	shell_error(sh, "message %s is not enabled (see 'dc msgs')", name);
	return NULL;
}

static int cmd_list(const struct shell *sh, size_t argc, char **argv)
{
	for (size_t i = 0; i < ARRAY_SIZE(dc_devs); i++) {
		shell_print(sh, "%s: node id %u", dc_devs[i]->name, dronecan_get_node_id(dc_devs[i]));
	}
	return 0;
}

static int cmd_msgs(const struct shell *sh, size_t argc, char **argv)
{
	for (size_t i = 0; i < dronecan_msg_table_len; i++) {
		shell_print(sh, "%5u  %s", dronecan_msg_table[i].desc.id, dronecan_msg_table[i].name);
	}
	return 0;
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = find_dev(sh, argv[1]);
	struct dronecan_stats s;

	if (dev == NULL) {
		return -ENODEV;
	}
	dronecan_get_stats(dev, &s);
	shell_print(sh, "node id    %u", dronecan_get_node_id(dev));
	shell_print(sh, "tx         %u (errors %u)", s.tx, s.tx_err);
	shell_print(sh, "rx status  %u", s.rx_ns);
	shell_print(sh, "alloc      rx %u, stage 1/2/3 %u/%u/%u, grants %u (last %u)", s.alloc_rx,
		    s.alloc_s1, s.alloc_s2, s.alloc_s3, s.alloc_grant, s.last_grant_id);
	shell_print(sh, "pool       %u/%u blocks (peak %u)", s.mem_current, s.mem_capacity,
		    s.mem_peak);
	return 0;
}

static int cmd_nodes(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = find_dev(sh, argv[1]);
	struct dronecan_node_entry e;
	uint32_t now = k_uptime_get_32();
	int found = 0;

	if (dev == NULL) {
		return -ENODEV;
	}
	for (uint8_t id = 1; dronecan_get_node(dev, id, &e) == 0; id++) {
		if (!e.active) {
			continue;
		}
		shell_print(sh, "node %3u: up %u s, health %u, mode %u, vendor 0x%04x, seen %u ms ago",
			    id, e.uptime_sec, e.health, e.mode, e.vendor_status, now - e.last_seen_ms);
		found++;
	}
	if (found == 0) {
		shell_print(sh, "no nodes (is DRONECAN_NODE_TABLE enabled?)");
	}
	return 0;
}

#ifdef CONFIG_DRONECAN_SHELL_PUBLISH
static int cmd_publish(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = find_dev(sh, argv[1]);
	const struct dronecan_msg_entry *m = find_msg(sh, argv[2]);
	uint8_t payload[CONFIG_DRONECAN_SHELL_MAX_PAYLOAD];
	char hex[2 * sizeof(payload) + 1] = "";
	size_t len;
	int ret;

	if (dev == NULL || m == NULL) {
		return -EINVAL;
	}
	// The bytes can be given as one string or as several arguments
	for (size_t i = 3; i < argc; i++) {
		if (strlen(hex) + strlen(argv[i]) >= sizeof(hex)) {
			shell_error(sh, "payload is longer than %zu bytes", sizeof(payload));
			return -EINVAL;
		}
		strcat(hex, argv[i]);
	}
	len = strlen(hex);
	if (len % 2 != 0 || hex2bin(hex, len, payload, sizeof(payload)) != len / 2) {
		shell_error(sh, "payload must be hex digits, two per byte");
		return -EINVAL;
	}
	ret = dronecan_broadcast(dev, &m->desc, DRONECAN_PRIORITY_MEDIUM, payload, len / 2);
	if (ret != 0) {
		shell_error(sh, "broadcast failed: %d", ret);
	}
	return ret;
}
#endif // CONFIG_DRONECAN_SHELL_PUBLISH

#ifdef CONFIG_DRONECAN_SHELL_SUBSCRIBE
// Receive callbacks run in the RX thread: the shell instance that asked is used to print
static const struct shell *rx_sh;

struct dc_sub {
	struct dronecan_subscription sub;
	const struct device *dev;
	const struct dronecan_msg_entry *msg;
};

static struct dc_sub dc_subs[CONFIG_DRONECAN_SHELL_MAX_SUBSCRIPTIONS];

static void sub_cb(const struct device *dev, const struct dronecan_transfer *t, void *user_data)
{
	const struct dc_sub *s = user_data;
	char hex[2 * CONFIG_DRONECAN_SHELL_MAX_PAYLOAD + 1];
	size_t n = MIN(t->payload_len, CONFIG_DRONECAN_SHELL_MAX_PAYLOAD);

	bin2hex(t->payload, n, hex, sizeof(hex));
	shell_print(rx_sh, "%s from %u (%u bytes): %s%s", s->msg->name, t->source_node_id,
		    t->payload_len, hex, n < t->payload_len ? "..." : "");
}

static struct dc_sub *find_sub(const struct device *dev, const struct dronecan_msg_entry *m)
{
	for (size_t i = 0; i < ARRAY_SIZE(dc_subs); i++) {
		if (dc_subs[i].msg == m && dc_subs[i].dev == dev) {
			return &dc_subs[i];
		}
	}
	return NULL;
}

static int cmd_subscribe(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = find_dev(sh, argv[1]);
	const struct dronecan_msg_entry *m = find_msg(sh, argv[2]);
	struct dc_sub *s;
	int ret;

	if (dev == NULL || m == NULL) {
		return -EINVAL;
	}
	if (find_sub(dev, m) != NULL) {
		shell_error(sh, "already subscribed");
		return -EALREADY;
	}
	s = find_sub(NULL, NULL);
	if (s == NULL) {
		shell_error(sh, "at most %d subscriptions", CONFIG_DRONECAN_SHELL_MAX_SUBSCRIPTIONS);
		return -ENOMEM;
	}
	rx_sh = sh;
	ret = dronecan_subscribe(dev, &s->sub, &m->desc, sub_cb, s);
	if (ret == 0) {
		s->dev = dev;
		s->msg = m;
	} else {
		shell_error(sh, "subscribe failed: %d", ret);
	}
	return ret;
}

static int cmd_unsubscribe(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = find_dev(sh, argv[1]);
	const struct dronecan_msg_entry *m = find_msg(sh, argv[2]);
	struct dc_sub *s;
	int ret;

	if (dev == NULL || m == NULL) {
		return -EINVAL;
	}
	s = find_sub(dev, m);
	if (s == NULL) {
		shell_error(sh, "not subscribed");
		return -ENOENT;
	}
	ret = dronecan_unsubscribe(dev, &s->sub);
	if (ret == 0) {
		s->dev = NULL;
		s->msg = NULL;
	}
	return ret;
}
#endif // CONFIG_DRONECAN_SHELL_SUBSCRIBE

#ifdef CONFIG_DRONECAN_SHELL_MONITOR
// One entry per message type: how many arrived and the last one
struct dc_mon {
	struct dronecan_subscription sub;
	uint32_t count;
	uint32_t last_ms;
	uint16_t last_len;
	uint8_t last_src;
};

static struct dc_mon dc_mons[CONFIG_DRONECAN_SHELL_MONITOR_TYPES];
static const struct device *mon_dev;
static const struct dronecan_msg_entry *mon_msgs[ARRAY_SIZE(dc_mons)];
static size_t mon_n;

static void mon_cb(const struct device *dev, const struct dronecan_transfer *t, void *user_data)
{
	struct dc_mon *m = user_data;

	m->count++;
	m->last_ms = k_uptime_get_32();
	m->last_len = t->payload_len;
	m->last_src = t->source_node_id;
}

static void mon_stop(void)
{
	for (size_t i = 0; i < mon_n; i++) {
		dronecan_unsubscribe(mon_dev, &dc_mons[i].sub);
	}
	mon_n = 0;
	mon_dev = NULL;
}

static int cmd_monitor(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = find_dev(sh, argv[1]);
	const char *op = argv[2];

	if (dev == NULL) {
		return -ENODEV;
	}
	if (strcmp(op, "off") == 0) {
		mon_stop();
		return 0;
	}
	if (strcmp(op, "on") == 0) {
		if (mon_dev != NULL) {
			mon_stop();
		}
		memset(dc_mons, 0, sizeof(dc_mons));
		for (size_t i = 0; i < dronecan_msg_table_len && mon_n < ARRAY_SIZE(dc_mons); i++) {
			if (dronecan_subscribe(dev, &dc_mons[mon_n].sub, &dronecan_msg_table[i].desc,
					       mon_cb, &dc_mons[mon_n]) == 0) {
				mon_msgs[mon_n++] = &dronecan_msg_table[i];
			}
		}
		mon_dev = dev;
		if (mon_n < dronecan_msg_table_len) {
			shell_warn(sh, "monitoring %zu of %zu enabled messages "
				   "(DRONECAN_SHELL_MONITOR_TYPES)", mon_n, dronecan_msg_table_len);
		}
		return 0;
	}
	if (strcmp(op, "show") == 0) {
		uint32_t now = k_uptime_get_32();

		if (mon_dev != dev) {
			shell_error(sh, "not monitoring %s, use 'dc monitor %s on'", dev->name,
				    dev->name);
			return -EINVAL;
		}
		for (size_t i = 0; i < mon_n; i++) {
			if (dc_mons[i].count == 0) {
				continue;
			}
			shell_print(sh, "%5u %-44s count %u, last from %u (%u bytes) %u ms ago",
				    mon_msgs[i]->desc.id, mon_msgs[i]->name, dc_mons[i].count,
				    dc_mons[i].last_src, dc_mons[i].last_len,
				    now - dc_mons[i].last_ms);
		}
		return 0;
	}
	shell_error(sh, "use on, off or show");
	return -EINVAL;
}
#endif // CONFIG_DRONECAN_SHELL_MONITOR

SHELL_STATIC_SUBCMD_SET_CREATE(dc_cmds,
	SHELL_CMD_ARG(list, NULL, "List the DroneCAN devices", cmd_list, 1, 0),
	SHELL_CMD_ARG(msgs, NULL, "List the enabled messages", cmd_msgs, 1, 0),
	SHELL_CMD_ARG(status, NULL, "Node id, counters and memory pool\n"
		      "Usage: dc status <device>", cmd_status, 2, 0),
	SHELL_CMD_ARG(nodes, NULL, "Nodes seen on the bus\n"
		      "Usage: dc nodes <device>", cmd_nodes, 2, 0),
#ifdef CONFIG_DRONECAN_SHELL_MONITOR
	SHELL_CMD_ARG(monitor, NULL, "Count the messages of every enabled type\n"
		      "Usage: dc monitor <device> on|off|show", cmd_monitor, 3, 0),
#endif
#ifdef CONFIG_DRONECAN_SHELL_SUBSCRIBE
	SHELL_CMD_ARG(subscribe, NULL, "Print each received message as hex\n"
		      "Usage: dc subscribe <device> <message>", cmd_subscribe, 3, 0),
	SHELL_CMD_ARG(unsubscribe, NULL, "Stop printing a message\n"
		      "Usage: dc unsubscribe <device> <message>", cmd_unsubscribe, 3, 0),
#endif
#ifdef CONFIG_DRONECAN_SHELL_PUBLISH
	SHELL_CMD_ARG(publish, NULL, "Broadcast a message from hex bytes\n"
		      "Usage: dc publish <device> <message> <hex bytes>", cmd_publish, 4, 8),
#endif
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(dc, &dc_cmds, "DroneCAN commands", NULL);

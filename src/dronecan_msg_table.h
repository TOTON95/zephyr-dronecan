/*
 * Copyright (c) 2026 3DR, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_MODULES_DRONECAN_SRC_DRONECAN_MSG_TABLE_H_
#define ZEPHYR_MODULES_DRONECAN_SRC_DRONECAN_MSG_TABLE_H_

#include <stddef.h>

#include <dronecan.h>

// Enabled messages, by full DSDL name. Generated, see scripts/gen_kconfig_messages.py
struct dronecan_msg_entry {
	const char *name;
	struct dronecan_msg_desc desc;
};

extern const struct dronecan_msg_entry dronecan_msg_table[];
extern const size_t dronecan_msg_table_len;

#endif // ZEPHYR_MODULES_DRONECAN_SRC_DRONECAN_MSG_TABLE_H_

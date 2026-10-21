/*
 * Copyright (c) 2026 3DR, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_MODULES_DRONECAN_DRONECAN_H_
#define ZEPHYR_MODULES_DRONECAN_DRONECAN_H_

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/slist.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// Constants
#define DRONECAN_NODE_ID_ANON		0
#define DRONECAN_HEALTH_OK		0
#define DRONECAN_HEALTH_WARNING		1
#define DRONECAN_HEALTH_ERROR		2
#define DRONECAN_HEALTH_CRITICAL	3
#define DRONECAN_MODE_OPERATIONAL	0
#define DRONECAN_MODE_INITIALIZATION	1
#define DRONECAN_MODE_MAINTENANCE	2
#define DRONECAN_MODE_SOFTWARE_UPDATE	3
#define DRONECAN_MODE_OFFLINE		7

// DroneCAN Transfer priority (libcanard)
enum dronecan_priority {
	DRONECAN_PRIORITY_HIGHEST = 0,
	DRONECAN_PRIORITY_HIGH    = 8,
	DRONECAN_PRIORITY_MEDIUM  = 16,
	DRONECAN_PRIORITY_LOW     = 24,
	DRONECAN_PRIORITY_LOWEST  = 31,
};

// Message Descriptor
struct dronecan_msg_desc {
	uint16_t id;
	uint64_t signature;
	uint16_t max_size;
};

// Abstraction from CanardRxTransfer
struct dronecan_transfer {
	uint8_t source_node_id;
	uint16_t data_type_id;
	uint8_t transfer_id;
	uint8_t priority;
	const uint8_t *payload;
	uint16_t payload_len;
	int64_t timestamp_ms;
};

// Callback to receive packages.
// Runs in the RX thread (not an ISR) with no canard lock held: keep it short,
// it may call dronecan_broadcast() but not dronecan_stop()/(un)subscribe().
// t->payload is read-only and valid only during the callback: copy to keep it.
typedef void (*dronecan_rx_cb_t)(const struct device *dev,
				 const struct dronecan_transfer *t,
				 void *user_data);

// DroneCAN subscription
struct dronecan_subscription {
	sys_snode_t node;
	uint16_t data_type_id;
	uint64_t signature;
	dronecan_rx_cb_t cb;
	void *user_data;
};

struct dronecan_stats {
	uint32_t tx;		// NodeStatus frames sent
	uint32_t tx_err;	// can_send() failures
	uint32_t rx_ns;		// NodeStatus frames received
	uint32_t alloc_rx;	// allocation frames received
	uint32_t alloc_s1;	// handshakes stage 1 handled
	uint32_t alloc_s2;	// handshakes stage 2 handled
	uint32_t alloc_s3;	// handshakes stage 3 handled (final)
	uint32_t alloc_grant;	// grants sent
	uint8_t  last_grant_id;

	// libcanard memory pool
	uint32_t mem_capacity;	// total blocks in arena
	uint32_t mem_current;	// blocks in use right now
	uint32_t mem_peak;	// peak blocks ever used

	// CAN controller, read when asked
	enum can_state can_state;
	uint8_t can_tx_errors;	// transmit error counter
	uint8_t can_rx_errors;	// receive error counter
};

struct dronecan_node_entry {
	bool active;
	uint32_t last_seen_ms;
	uint32_t uptime_sec;
	uint8_t health;
	uint8_t mode;
	uint16_t vendor_status;
};

// Lifecycle related functions
int dronecan_start(const struct device *dev);
int dronecan_stop(const struct device *dev);
uint8_t dronecan_get_node_id(const struct device *dev);

// NodeStatus related functions
int dronecan_set_health(const struct device *dev, uint8_t health);
int dronecan_set_mode(const struct device *dev, uint8_t mode);
int dronecan_set_vendor_status(const struct device *dev, uint16_t status);

// Publish into CAN Bus
int dronecan_broadcast(const struct device *dev, const struct dronecan_msg_desc *desc,
		       enum dronecan_priority p, const void *payload, uint16_t len);

// Un/Subscribe for a topic from DroneCAN node.
// The caller owns *sub: keep it valid until dronecan_unsubscribe() returns,
// after which the callback is no longer running. Several subscriptions per
// message are allowed. Calls are thread-safe, not ISR-safe; return -errno.
int dronecan_subscribe(const struct device *dev, struct dronecan_subscription *sub,
		       const struct dronecan_msg_desc *desc, dronecan_rx_cb_t cb, void *user_data);
int dronecan_unsubscribe(const struct device *dev, struct dronecan_subscription *sub);

// Node table and statistics
int dronecan_get_node(const struct device *dev, uint8_t id, struct dronecan_node_entry *out);
void dronecan_get_stats(const struct device *dev, struct dronecan_stats *out);

// DNIA Request, for clients
// int dronecan_dnia_request(const struct device *dev, k_timeout_t timeout);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // ZEPHYR_MODULES_DRONECAN_DRONECAN_H_

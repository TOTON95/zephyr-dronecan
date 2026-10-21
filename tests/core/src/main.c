/*
 * Copyright (c) 2026 3DR, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <dronecan.h>
#include <uavcan.protocol.NodeStatus.h>

// Nodes 42 and 43 share the loopback controller and hear each other
static const struct device *const a = DEVICE_DT_GET(DT_NODELABEL(dronecan0));
static const struct device *const b = DEVICE_DT_GET(DT_NODELABEL(dronecan1));

static const struct dronecan_msg_desc small_desc = {1001, 0x5678, 8};
static const struct dronecan_msg_desc big_desc = {1000, 0x1234, 64};
static const struct dronecan_msg_desc echo_desc = {1002, 0x9abc, 8};

struct record {
	struct k_sem sem;
	atomic_t count;
	uint8_t source;
	uint16_t type;
	uint16_t len;
	uint8_t data[64];
};

static struct record rec1, rec2;
static struct dronecan_subscription sub1, sub2;

static void record_cb(const struct device *dev, const struct dronecan_transfer *t, void *user_data)
{
	struct record *r = user_data;

	r->source = t->source_node_id;
	r->type = t->data_type_id;
	r->len = t->payload_len;
	memcpy(r->data, t->payload, MIN(t->payload_len, sizeof(r->data)));
	atomic_inc(&r->count);
	k_sem_give(&r->sem);
}

static void record_reset(struct record *r)
{
	memset(r, 0, sizeof(*r));
	k_sem_init(&r->sem, 0, K_SEM_MAX_LIMIT);
}

static void before(void *fixture)
{
	record_reset(&rec1);
	record_reset(&rec2);
	zassert_ok(dronecan_start(a));
	zassert_ok(dronecan_start(b));
}

// Reverse start order: the first node started owns the controller
static void after(void *fixture)
{
	dronecan_unsubscribe(a, &sub1);
	dronecan_unsubscribe(a, &sub2);
	dronecan_unsubscribe(b, &sub1);
	dronecan_stop(b);
	dronecan_stop(a);
}

ZTEST_SUITE(dronecan_core, NULL, NULL, before, after, NULL);

ZTEST(dronecan_core, test_lifecycle)
{
	uint8_t p[4] = {0};

	dronecan_stop(b);
	dronecan_stop(a);

	zassert_equal(dronecan_broadcast(a, &small_desc, DRONECAN_PRIORITY_LOW, p, 4), -ENETDOWN);
	zassert_equal(dronecan_stop(a), -EALREADY);
	zassert_ok(dronecan_start(a));
	zassert_equal(dronecan_start(a), -EALREADY);
	zassert_equal(dronecan_get_node_id(a), 42);
	zassert_ok(dronecan_broadcast(a, &small_desc, DRONECAN_PRIORITY_LOW, p, 4));
	zassert_ok(dronecan_stop(a));
	zassert_equal(dronecan_broadcast(a, &small_desc, DRONECAN_PRIORITY_LOW, p, 4), -ENETDOWN);

	zassert_ok(dronecan_start(a)); // after() stops what the hook started
	zassert_ok(dronecan_start(b));
}

ZTEST(dronecan_core, test_broadcast_arguments)
{
	uint8_t p[16] = {0};

	zassert_equal(dronecan_broadcast(a, NULL, DRONECAN_PRIORITY_LOW, p, 4), -EINVAL);
	zassert_equal(dronecan_broadcast(a, &small_desc, DRONECAN_PRIORITY_LOW, p, 9), -EINVAL,
		      "longer than the descriptor allows");
	zassert_equal(dronecan_broadcast(a, &small_desc, DRONECAN_PRIORITY_LOW, NULL, 4), -EINVAL);
}

ZTEST(dronecan_core, test_subscribe_arguments)
{
	zassert_equal(dronecan_subscribe(a, NULL, &small_desc, record_cb, &rec1), -EINVAL);
	zassert_equal(dronecan_subscribe(a, &sub1, NULL, record_cb, &rec1), -EINVAL);
	zassert_equal(dronecan_subscribe(a, &sub1, &small_desc, NULL, &rec1), -EINVAL);

	zassert_ok(dronecan_subscribe(a, &sub1, &small_desc, record_cb, &rec1));
	zassert_equal(dronecan_subscribe(a, &sub1, &small_desc, record_cb, &rec1), -EALREADY);
	zassert_ok(dronecan_unsubscribe(a, &sub1));
	zassert_equal(dronecan_unsubscribe(a, &sub1), -ENOENT);
}

ZTEST(dronecan_core, test_single_frame_delivery)
{
	const uint8_t p[4] = {1, 2, 3, 4};

	zassert_ok(dronecan_subscribe(a, &sub1, &small_desc, record_cb, &rec1));
	zassert_ok(dronecan_broadcast(b, &small_desc, DRONECAN_PRIORITY_LOW, p, sizeof(p)));
	zassert_ok(k_sem_take(&rec1.sem, K_MSEC(500)), "not delivered");

	zassert_equal(rec1.source, 43);
	zassert_equal(rec1.type, small_desc.id);
	zassert_equal(rec1.len, sizeof(p));
	zassert_mem_equal(rec1.data, p, sizeof(p));
}

ZTEST(dronecan_core, test_multi_frame_delivery)
{
	uint8_t p[64];

	for (int i = 0; i < sizeof(p); i++) {
		p[i] = i;
	}

	zassert_ok(dronecan_subscribe(a, &sub1, &big_desc, record_cb, &rec1));
	for (uint16_t len = 8; len <= sizeof(p); len += 28) { // 8, 36, 64 bytes
		zassert_ok(dronecan_broadcast(b, &big_desc, DRONECAN_PRIORITY_LOW, p, len));
		zassert_ok(k_sem_take(&rec1.sem, K_MSEC(500)), "%u bytes not delivered", len);
		zassert_equal(rec1.len, len);
		zassert_mem_equal(rec1.data, p, len, "%u bytes arrived changed", len);
	}
}

static void echo_cb(const struct device *dev, const struct dronecan_transfer *t, void *user_data)
{
	dronecan_broadcast(dev, &echo_desc, DRONECAN_PRIORITY_LOW, t->payload, MIN(t->payload_len, 8));
}

ZTEST(dronecan_core, test_callback_can_broadcast)
{
	const uint8_t p[2] = {7, 8};

	zassert_ok(dronecan_subscribe(a, &sub1, &small_desc, echo_cb, NULL));
	zassert_ok(dronecan_subscribe(b, &sub2, &echo_desc, record_cb, &rec2));
	zassert_ok(dronecan_broadcast(b, &small_desc, DRONECAN_PRIORITY_LOW, p, sizeof(p)));
	zassert_ok(k_sem_take(&rec2.sem, K_MSEC(500)), "the callback's message is not delivered");
	zassert_equal(rec2.source, 42);
	zassert_mem_equal(rec2.data, p, sizeof(p));
}

ZTEST(dronecan_core, test_several_subscribers)
{
	const uint8_t p[1] = {9};
	struct dronecan_subscription extra;

	zassert_ok(dronecan_subscribe(a, &sub1, &small_desc, record_cb, &rec1));
	zassert_ok(dronecan_subscribe(a, &extra, &small_desc, record_cb, &rec2));
	zassert_ok(dronecan_broadcast(b, &small_desc, DRONECAN_PRIORITY_LOW, p, sizeof(p)));
	zassert_ok(k_sem_take(&rec1.sem, K_MSEC(500)));
	zassert_ok(k_sem_take(&rec2.sem, K_MSEC(500)));
	zassert_ok(dronecan_unsubscribe(a, &extra));
}

ZTEST(dronecan_core, test_no_callback_after_unsubscribe)
{
	const uint8_t p[1] = {1};

	zassert_ok(dronecan_subscribe(a, &sub1, &small_desc, record_cb, &rec1));
	zassert_ok(dronecan_broadcast(b, &small_desc, DRONECAN_PRIORITY_LOW, p, sizeof(p)));
	zassert_ok(k_sem_take(&rec1.sem, K_MSEC(500)));

	zassert_ok(dronecan_unsubscribe(a, &sub1));
	zassert_ok(dronecan_broadcast(b, &small_desc, DRONECAN_PRIORITY_LOW, p, sizeof(p)));
	k_sleep(K_MSEC(200));
	zassert_equal(atomic_get(&rec1.count), 1, "delivered after unsubscribe");
}

static K_SEM_DEFINE(slow_started, 0, 1);
static volatile bool slow_done;

static void slow_cb(const struct device *dev, const struct dronecan_transfer *t, void *user_data)
{
	k_sem_give(&slow_started);
	k_msleep(300);
	slow_done = true;
}

ZTEST(dronecan_core, test_unsubscribe_waits_for_running_callback)
{
	const uint8_t p[1] = {1};

	slow_done = false;
	k_sem_reset(&slow_started);
	zassert_ok(dronecan_subscribe(a, &sub1, &small_desc, slow_cb, NULL));
	zassert_ok(dronecan_broadcast(b, &small_desc, DRONECAN_PRIORITY_LOW, p, sizeof(p)));
	zassert_ok(k_sem_take(&slow_started, K_MSEC(500)));

	zassert_false(slow_done);
	zassert_ok(dronecan_unsubscribe(a, &sub1));
	zassert_true(slow_done, "unsubscribe returned while the callback was still running");
}

ZTEST(dronecan_core, test_node_status_reaches_the_table)
{
	struct dronecan_node_entry e = {0};
	int64_t end = k_uptime_get() + 2000;

	zassert_ok(dronecan_set_health(a, DRONECAN_HEALTH_WARNING));
	zassert_ok(dronecan_set_mode(a, DRONECAN_MODE_MAINTENANCE));
	zassert_ok(dronecan_set_vendor_status(a, 0x1234));

	// Node b hears node a's NodeStatus
	while (k_uptime_get() < end) {
		zassert_ok(dronecan_get_node(b, 42, &e));
		if (e.active && e.health == DRONECAN_HEALTH_WARNING) {
			break;
		}
		k_sleep(K_MSEC(50));
	}
	zassert_true(e.active, "node 42 never appeared");
	zassert_equal(e.health, DRONECAN_HEALTH_WARNING);
	zassert_equal(e.mode, DRONECAN_MODE_MAINTENANCE);
	zassert_equal(e.vendor_status, 0x1234);

	zassert_ok(dronecan_get_node(b, 7, &e));
	zassert_false(e.active, "an unknown node is active");
}

ZTEST(dronecan_core, test_status_setters_validate)
{
	zassert_equal(dronecan_set_health(a, 4), -EINVAL);
	zassert_equal(dronecan_set_mode(a, 4), -EINVAL);
	zassert_ok(dronecan_set_mode(a, DRONECAN_MODE_OFFLINE));
	zassert_ok(dronecan_set_mode(a, DRONECAN_MODE_OPERATIONAL));
}

ZTEST(dronecan_core, test_stale_node_expires)
{
	struct dronecan_node_entry e = {0};
	int64_t end = k_uptime_get() + 2000;

	while (k_uptime_get() < end && !e.active) {
		zassert_ok(dronecan_get_node(a, 43, &e));
		k_sleep(K_MSEC(50));
	}
	zassert_true(e.active, "node 43 never appeared");

	zassert_ok(dronecan_stop(b)); // b does not own the controller: a keeps running
	end = k_uptime_get() + 3000;
	while (k_uptime_get() < end && e.active) {
		k_sleep(K_MSEC(50));
		zassert_ok(dronecan_get_node(a, 43, &e));
	}
	zassert_false(e.active, "node 43 never expired");
	zassert_ok(dronecan_start(b)); // for after()
}

ZTEST(dronecan_core, test_statistics)
{
	struct dronecan_stats s;
	const uint8_t p[1] = {1};

	zassert_ok(dronecan_broadcast(a, &small_desc, DRONECAN_PRIORITY_LOW, p, sizeof(p)));
	k_sleep(K_MSEC(300));
	dronecan_get_stats(a, &s);

	zassert_true(s.tx >= 1, "no frame counted");
	zassert_equal(s.tx_err, 0);
	zassert_true(s.rx_ns >= 1, "no NodeStatus counted");
	zassert_true(s.mem_capacity > 0);
	zassert_equal(s.can_state, CAN_STATE_ERROR_ACTIVE);
	zassert_equal(s.can_tx_errors, 0);
}

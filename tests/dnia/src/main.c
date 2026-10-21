/*
 * Copyright (c) 2026 3DR, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <dronecan.h>
#include <uavcan.protocol.dynamic_node_id.Allocation.h>

static const struct device *const alloc = DEVICE_DT_GET(DT_NODELABEL(dronecan0));  // 10, allocator
static const struct device *const lib = DEVICE_DT_GET(DT_NODELABEL(dronecan1));    // anonymous, library client
static const struct device *const raw = DEVICE_DT_GET(DT_NODELABEL(dronecan2));    // anonymous, driven here

static const struct dronecan_msg_desc alloc_desc = {
	UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
	UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
	UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_SIZE,
};

// The nodes sharing the controller are started in this order, so the first one owns it
static const struct device *const all[] = {alloc, lib, raw};

static void start_nodes(const struct device *const *nodes, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		zassert_ok(dronecan_start(nodes[i]));
	}
}

static void before(void *fixture)
{
}

// Reverse order: the first node started owns the controller
static void after(void *fixture)
{
	for (int i = ARRAY_SIZE(all) - 1; i >= 0; i--) {
		dronecan_stop(all[i]);
	}
}

ZTEST_SUITE(dronecan_dnia, NULL, NULL, before, after, NULL);

// The client side of the handshake, by hand, on the "raw" node
static struct dronecan_subscription raw_sub;
static const uint8_t *raw_uid;
static volatile int raw_granted;
static K_SEM_DEFINE(raw_done, 0, 1);

static void raw_send(const uint8_t *uid, int offset, bool first)
{
	struct uavcan_protocol_dynamic_node_id_Allocation req = {0};
	uint8_t buf[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_SIZE];
	int n = MIN(6, 16 - offset);

	req.first_part_of_unique_id = first;
	req.unique_id.len = n;
	memcpy(req.unique_id.data, uid + offset, n);
	zassert_ok(dronecan_broadcast(raw, &alloc_desc, DRONECAN_PRIORITY_LOW, buf,
				      uavcan_protocol_dynamic_node_id_Allocation_encode(&req, buf)));
}

static void raw_cb(const struct device *dev, const struct dronecan_transfer *t, void *user_data)
{
	uint16_t got = t->payload_len - 1; // the node ID byte comes first

	if (t->source_node_id == 0 || raw_uid == NULL || got > 16 ||
	    memcmp(t->payload + 1, raw_uid, got) != 0) {
		return; // a request, or an answer for somebody else
	}
	if (got == 16) {
		raw_granted = t->payload[0] >> 1;
		k_sem_give(&raw_done);
		return;
	}
	raw_send(raw_uid, got, false);
}

static int raw_request(const uint8_t *uid)
{
	raw_uid = uid;
	raw_granted = 0;
	k_sem_reset(&raw_done);
	raw_send(uid, 0, true);
	return k_sem_take(&raw_done, K_SECONDS(2)) == 0 ? raw_granted : -1;
}

static void uid_fill(uint8_t *uid, uint8_t base)
{
	for (int i = 0; i < 16; i++) {
		uid[i] = base + i;
	}
}

ZTEST(dronecan_dnia, test_server_grants_and_remembers)
{
	uint8_t uid1[16], uid2[16];
	struct dronecan_stats s;

	uid_fill(uid1, 0x10);
	uid_fill(uid2, 0x80);
	start_nodes(all, 3);
	zassert_ok(dronecan_subscribe(raw, &raw_sub, &alloc_desc, raw_cb, NULL));

	// The allocator is node 10, so the first ID it can hand out is 11
	zassert_equal(raw_request(uid1), 11, "first unique ID");
	zassert_equal(raw_request(uid1), 11, "the same unique ID gets the same node ID");
	zassert_equal(raw_request(uid2), 12, "another unique ID gets the next one");

	dronecan_get_stats(alloc, &s);
	zassert_equal(s.alloc_s1, 3);
	zassert_equal(s.alloc_s2, 3);
	zassert_equal(s.alloc_s3, 3);
	zassert_equal(s.alloc_grant, 3);
	zassert_equal(s.last_grant_id, 12);
	zassert_ok(dronecan_unsubscribe(raw, &raw_sub));
}

ZTEST(dronecan_dnia, test_client_gets_an_id)
{
	int id;

	start_nodes(all, 3);
	zassert_equal(dronecan_get_node_id(lib), 0);
	zassert_ok(dronecan_dnia_request(lib, K_SECONDS(5)));

	id = dronecan_get_node_id(lib);
	zassert_true(id >= 11 && id <= 125, "node ID %d", id);
	zassert_equal(dronecan_dnia_request(lib, K_SECONDS(1)), -EALREADY);

	// A restarted node asks again and gets the same ID
	zassert_ok(dronecan_stop(lib));
	zassert_ok(dronecan_start(lib));
	zassert_equal(dronecan_get_node_id(lib), 0);
	zassert_ok(dronecan_dnia_request(lib, K_SECONDS(5)));
	zassert_equal(dronecan_get_node_id(lib), id, "a different ID the second time");
}

ZTEST(dronecan_dnia, test_client_needs_a_started_node)
{
	zassert_equal(dronecan_dnia_request(lib, K_SECONDS(1)), -ENETDOWN);
}

ZTEST(dronecan_dnia, test_client_times_out_without_an_allocator)
{
	const struct device *const nodes[] = {lib, raw};
	struct dronecan_stats s;
	int64_t t0;
	int ret;

	start_nodes(nodes, 2);
	t0 = k_uptime_get();
	ret = dronecan_dnia_request(lib, K_SECONDS(2));
	t0 = k_uptime_get() - t0;

	zassert_equal(ret, -ETIMEDOUT);
	zassert_true(t0 >= 2000 && t0 < 4500, "took %lld ms", t0);
	zassert_equal(dronecan_get_node_id(lib), 0);
	dronecan_get_stats(lib, &s);
	zassert_true(s.tx >= 2, "no attempts made: tx=%u", s.tx);
}

static K_THREAD_STACK_DEFINE(req_stack, 2048);
static struct k_thread req_thread;
static volatile int req_ret;

static void req_fn(void *a, void *b, void *c)
{
	req_ret = dronecan_dnia_request(lib, K_FOREVER);
}

ZTEST(dronecan_dnia, test_client_busy_and_cancelled_by_stop)
{
	const struct device *const nodes[] = {lib, raw};

	start_nodes(nodes, 2); // no allocator, so the request stays pending
	req_ret = 12345;
	k_thread_create(&req_thread, req_stack, K_THREAD_STACK_SIZEOF(req_stack), req_fn, NULL, NULL,
			NULL, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
	k_sleep(K_MSEC(300));

	zassert_equal(dronecan_dnia_request(lib, K_SECONDS(1)), -EBUSY);
	zassert_ok(dronecan_stop(lib));
	zassert_ok(k_thread_join(&req_thread, K_SECONDS(3)), "the request did not end");
	zassert_equal(req_ret, -ECANCELED);
}

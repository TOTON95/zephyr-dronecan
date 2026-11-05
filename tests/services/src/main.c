/*
 * Copyright (c) 2026 3DR, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <canard.h>
#include <dronecan.h>

static const struct device *const dc = DEVICE_DT_GET(DT_NODELABEL(dronecan0)); // node 42
static const struct device *const can = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

#define CLIENT_ID 99

static const struct dronecan_msg_desc echo_desc = {200, 0x1122334455667788ULL, 8};
// Service 1 has the number of the allocation message, which is a different thing
static const struct dronecan_msg_desc echo1_desc = {1, 0x8877665544332211ULL, 8};
static const uint64_t wrong_signature = 0x0102030405060708ULL;

// A client that is not part of the module, like a GUI tool: libcanard on the same controller
static CanardInstance client;
static uint8_t client_arena[64 * CANARD_MEM_BLOCK_SIZE] __aligned(8);
CAN_MSGQ_DEFINE(client_msgq, 32);

static struct {
	bool got;
	uint8_t source;
	uint8_t priority;
	uint8_t transfer_id;
	uint16_t len;
	uint8_t data[CONFIG_DRONECAN_MAX_SERVICE_RESPONSE];
} resp;

// The first response, kept for the test that sends two requests
static int resp_count;
static uint16_t first_len;
static uint8_t first_data[8];

static bool client_should_accept(const CanardInstance *ins, uint64_t *sig, uint16_t id,
				 CanardTransferType type, uint8_t source)
{
	if (type != CanardTransferTypeResponse) {
		return false;
	}
	if (id == echo_desc.id) {
		*sig = echo_desc.signature;
		return true;
	}
	if (id == echo1_desc.id) {
		*sig = echo1_desc.signature;
		return true;
	}
	return false;
}

static void client_on_reception(CanardInstance *ins, CanardRxTransfer *t)
{
	if (resp_count == 0) {
		first_len = MIN(t->payload_len, sizeof(first_data));
		for (uint16_t i = 0; i < first_len; i++) {
			canardDecodeScalar(t, i * 8U, 8, false, &first_data[i]);
		}
	}
	resp_count++;
	resp.got = true;
	resp.source = t->source_node_id;
	resp.priority = t->priority;
	resp.transfer_id = t->transfer_id;
	resp.len = MIN(t->payload_len, sizeof(resp.data));
	for (uint16_t i = 0; i < resp.len; i++) {
		canardDecodeScalar(t, i * 8U, 8, false, &resp.data[i]);
	}
}

static uint8_t client_tid;
static uint8_t last_request_tid;

static void client_send(uint8_t dest, uint64_t signature, uint8_t id, const void *payload,
			uint16_t len)
{
	const CanardCANFrame *cf;

	last_request_tid = client_tid;
	zassert_true(canardRequestOrRespond(&client, dest, signature, id, &client_tid,
					    CANARD_TRANSFER_PRIORITY_MEDIUM, CanardRequest, payload,
					    len) > 0);
	while ((cf = canardPeekTxQueue(&client)) != NULL) {
		struct can_frame f = {
			.id = cf->id & CANARD_CAN_EXT_ID_MASK,
			.flags = CAN_FRAME_IDE,
			.dlc = can_bytes_to_dlc(cf->data_len),
		};

		memcpy(f.data, cf->data, cf->data_len);
		zassert_ok(can_send(can, &f, K_MSEC(100), NULL, NULL));
		canardPopTxQueue(&client);
	}
}

// Feed the client what arrives on the controller until a response shows up or the time is up
static bool client_wait(k_timeout_t timeout)
{
	k_timepoint_t end = sys_timepoint_calc(timeout);
	struct can_frame f;

	while (!resp.got && k_msgq_get(&client_msgq, &f, sys_timepoint_timeout(end)) == 0) {
		CanardCANFrame cf = {
			.id = f.id | CANARD_CAN_FRAME_EFF,
			.data_len = can_dlc_to_bytes(f.dlc),
		};

		memcpy(cf.data, f.data, cf.data_len);
		canardHandleRxFrame(&client, &cf, k_ticks_to_us_floor64(k_uptime_ticks()) + 1U);
	}
	return resp.got;
}

static bool request_id(uint8_t dest, uint8_t id, uint64_t signature, const void *payload,
		       uint16_t len, k_timeout_t timeout)
{
	memset(&resp, 0, sizeof(resp));
	resp_count = 0;
	client_send(dest, signature, id, payload, len);
	return client_wait(timeout);
}

static bool request(uint8_t dest, uint64_t signature, const void *payload, uint16_t len,
		    k_timeout_t timeout)
{
	return request_id(dest, echo_desc.id, signature, payload, len, timeout);
}

static int filter_id;

static void *setup(void)
{
	canardInit(&client, client_arena, sizeof(client_arena), client_on_reception,
		   client_should_accept, NULL);
	canardSetLocalNodeID(&client, CLIENT_ID);
	return NULL;
}

static void before(void *fixture)
{
	static const struct can_filter all = {.id = 0, .mask = 0, .flags = CAN_FILTER_IDE};

	zassert_ok(dronecan_start(dc));
	filter_id = can_add_rx_filter_msgq(can, &client_msgq, &all);
	zassert_true(filter_id >= 0);
	k_msgq_purge(&client_msgq);
}

static struct dronecan_service svc, svc2;
static atomic_t handler_calls;
static atomic_t handler_response_size;

static void after(void *fixture)
{
	dronecan_unregister_service(dc, &svc);
	dronecan_unregister_service(dc, &svc2);
	can_remove_rx_filter(can, filter_id);
	dronecan_stop(dc);
}

ZTEST_SUITE(dronecan_services, NULL, setup, before, after, NULL);

static int echo_cb(const struct device *dev, const struct dronecan_transfer *req, void *response,
		   uint16_t response_size, void *user_data)
{
	atomic_inc(&handler_calls);
	atomic_set(&handler_response_size, response_size);
	zassert_equal(req->source_node_id, CLIENT_ID);
	memcpy(response, req->payload, req->payload_len);
	return req->payload_len;
}

ZTEST(dronecan_services, test_register_arguments)
{
	struct dronecan_msg_desc big_id = {256, 1, 8};

	zassert_equal(dronecan_register_service(dc, NULL, &echo_desc, echo_cb, NULL), -EINVAL);
	zassert_equal(dronecan_register_service(dc, &svc, NULL, echo_cb, NULL), -EINVAL);
	zassert_equal(dronecan_register_service(dc, &svc, &echo_desc, NULL, NULL), -EINVAL);
	zassert_equal(dronecan_register_service(dc, &svc, &big_id, echo_cb, NULL), -EINVAL,
		      "service IDs are 8 bit");

	zassert_ok(dronecan_register_service(dc, &svc, &echo_desc, echo_cb, NULL));
	zassert_equal(dronecan_register_service(dc, &svc, &echo_desc, echo_cb, NULL), -EALREADY);
	zassert_equal(dronecan_register_service(dc, &svc2, &echo_desc, echo_cb, NULL), -EEXIST,
		      "one handler per service");
	zassert_ok(dronecan_unregister_service(dc, &svc));
	zassert_equal(dronecan_unregister_service(dc, &svc), -ENOENT);
	zassert_ok(dronecan_register_service(dc, &svc2, &echo_desc, echo_cb, NULL),
		   "the ID is free again");
}

ZTEST(dronecan_services, test_echo_single_and_multi_frame)
{
	uint8_t p[100];

	for (int i = 0; i < sizeof(p); i++) {
		p[i] = i;
	}
	zassert_ok(dronecan_register_service(dc, &svc, &echo_desc, echo_cb, NULL));

	for (uint16_t len = 1; len <= sizeof(p); len += (len < 8 ? 3 : 46)) { // 1, 4, 7, 53, 99
		zassert_true(request(42, echo_desc.signature, p, len, K_MSEC(500)),
			     "no response to %u bytes", len);
		zassert_equal(resp.source, 42);
		zassert_equal(resp.len, len);
		zassert_mem_equal(resp.data, p, len);
		zassert_equal(resp.transfer_id, last_request_tid,
			      "the response must carry the request's transfer ID");
	}
	zassert_equal(atomic_get(&handler_response_size), CONFIG_DRONECAN_MAX_SERVICE_RESPONSE);
}

static int failing_cb(const struct device *dev, const struct dronecan_transfer *req,
		      void *response, uint16_t response_size, void *user_data)
{
	atomic_inc(&handler_calls);
	return -EIO;
}

ZTEST(dronecan_services, test_handler_error_sends_no_response)
{
	const uint8_t p[2] = {1, 2};
	struct dronecan_stats s;

	atomic_set(&handler_calls, 0);
	zassert_ok(dronecan_register_service(dc, &svc, &echo_desc, failing_cb, NULL));
	zassert_false(request(42, echo_desc.signature, p, sizeof(p), K_MSEC(300)));
	zassert_equal(atomic_get(&handler_calls), 1);

	dronecan_get_stats(dc, &s);
	zassert_equal(s.svc_requests, 1);
	zassert_equal(s.svc_errors, 1);
	zassert_equal(s.svc_responses, 0);
}

ZTEST(dronecan_services, test_requests_that_are_not_for_us)
{
	uint8_t p[20] = {0};

	atomic_set(&handler_calls, 0);
	zassert_ok(dronecan_register_service(dc, &svc, &echo_desc, echo_cb, NULL));

	zassert_false(request(77, echo_desc.signature, p, 4, K_MSEC(300)), "another node");
	zassert_false(request(42, wrong_signature, p, sizeof(p), K_MSEC(300)),
		      "a multi-frame request with another signature fails its CRC");
	zassert_equal(atomic_get(&handler_calls), 0);

	zassert_ok(dronecan_unregister_service(dc, &svc));
	zassert_false(request(42, echo_desc.signature, p, 4, K_MSEC(300)), "no handler");
	zassert_equal(atomic_get(&handler_calls), 0);
}

static K_SEM_DEFINE(slow_started, 0, 1);
static atomic_t slow_done;

static int slow_cb(const struct device *dev, const struct dronecan_transfer *req, void *response,
		   uint16_t response_size, void *user_data)
{
	k_sem_give(&slow_started);
	k_msleep(300);
	atomic_set(&slow_done, 1);
	return 0;
}

ZTEST(dronecan_services, test_unregister_waits_for_running_handler)
{
	const uint8_t p[1] = {1};

	atomic_set(&slow_done, 0);
	k_sem_reset(&slow_started);
	zassert_ok(dronecan_register_service(dc, &svc, &echo_desc, slow_cb, NULL));
	memset(&resp, 0, sizeof(resp));
	client_send(42, echo_desc.signature, echo_desc.id, p, sizeof(p));
	zassert_ok(k_sem_take(&slow_started, K_MSEC(500)));

	zassert_false(atomic_get(&slow_done));
	zassert_ok(dronecan_unregister_service(dc, &svc));
	zassert_true(atomic_get(&slow_done),
		     "unregister returned while the handler was still running");
}

ZTEST(dronecan_services, test_empty_response)
{
	const uint8_t p[1] = {1};

	zassert_ok(dronecan_register_service(dc, &svc, &echo_desc, slow_cb, NULL));
	zassert_true(request(42, echo_desc.signature, p, sizeof(p), K_SECONDS(1)));
	zassert_equal(resp.len, 0, "a handler may answer with no payload");
}

ZTEST(dronecan_services, test_registration_survives_restart)
{
	const uint8_t p[2] = {5, 6};

	zassert_ok(dronecan_register_service(dc, &svc, &echo_desc, echo_cb, NULL));
	can_remove_rx_filter(can, filter_id);
	zassert_ok(dronecan_stop(dc));
	zassert_ok(dronecan_start(dc));
	static const struct can_filter all = {.id = 0, .mask = 0, .flags = CAN_FILTER_IDE};

	filter_id = can_add_rx_filter_msgq(can, &client_msgq, &all);
	zassert_true(filter_id >= 0);

	zassert_true(request(42, echo_desc.signature, p, sizeof(p), K_MSEC(500)));
	zassert_mem_equal(resp.data, p, sizeof(p));
}

static int too_big_cb(const struct device *dev, const struct dronecan_transfer *req, void *response,
		      uint16_t response_size, void *user_data)
{
	return response_size + 1;
}

ZTEST(dronecan_services, test_result_larger_than_the_buffer_sends_nothing)
{
	const uint8_t p[1] = {1};
	struct dronecan_stats s;

	zassert_ok(dronecan_register_service(dc, &svc, &echo_desc, too_big_cb, NULL));
	zassert_false(request(42, echo_desc.signature, p, sizeof(p), K_MSEC(300)));

	dronecan_get_stats(dc, &s);
	zassert_equal(s.svc_requests, 1);
	zassert_equal(s.svc_responses, 0);
	zassert_equal(s.svc_errors, 1);
}

ZTEST(dronecan_services, test_response_keeps_the_priority)
{
	const uint8_t p[1] = {1};

	zassert_ok(dronecan_register_service(dc, &svc, &echo_desc, echo_cb, NULL));
	zassert_true(request(42, echo_desc.signature, p, sizeof(p), K_MSEC(500)));
	zassert_equal(resp.priority, CANARD_TRANSFER_PRIORITY_MEDIUM,
		      "the client asked with MEDIUM");
}

ZTEST(dronecan_services, test_requests_back_to_back)
{
	const uint8_t a[2] = {1, 2};
	const uint8_t b[3] = {3, 4, 5};
	struct dronecan_stats s;

	zassert_ok(dronecan_register_service(dc, &svc, &echo_desc, echo_cb, NULL));
	memset(&resp, 0, sizeof(resp));
	resp_count = 0;
	client_send(42, echo_desc.signature, echo_desc.id, a, sizeof(a));
	client_send(42, echo_desc.signature, echo_desc.id, b, sizeof(b));

	// Both are answered, in order
	k_timepoint_t end = sys_timepoint_calc(K_SECONDS(1));

	while (resp_count < 2 && !sys_timepoint_expired(end)) {
		resp.got = false;
		client_wait(sys_timepoint_timeout(end));
	}
	zassert_equal(resp_count, 2, "got %d responses", resp_count);
	zassert_equal(first_len, sizeof(a));
	zassert_mem_equal(first_data, a, sizeof(a));
	zassert_equal(resp.len, sizeof(b));
	zassert_mem_equal(resp.data, b, sizeof(b));

	dronecan_get_stats(dc, &s);
	zassert_equal(s.svc_requests, 2);
	zassert_equal(s.svc_responses, 2);
}

// With the allocation code built in, a service numbered 1 is still just a service
ZTEST(dronecan_services, test_service_id_one_next_to_the_allocation_message)
{
	const uint8_t p[3] = {7, 8, 9};
	struct dronecan_stats s;

	zassert_ok(dronecan_register_service(dc, &svc, &echo1_desc, echo_cb, NULL));
	zassert_true(request_id(42, echo1_desc.id, echo1_desc.signature, p, sizeof(p),
				K_MSEC(500)));
	zassert_equal(resp.len, sizeof(p));
	zassert_mem_equal(resp.data, p, sizeof(p));

	dronecan_get_stats(dc, &s);
	zassert_equal(s.alloc_rx, 0, "the request reached the allocation handler");
}

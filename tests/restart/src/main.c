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
#include <uavcan.protocol.RestartNode.h>

static const struct device *const dc = DEVICE_DT_GET(DT_NODELABEL(dronecan0)); // node 42
static const struct device *const can = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

#define CLIENT_ID 99
#define DELAY_MS CONFIG_DRONECAN_RESTART_DELAY_MS

// The magic number, least significant byte first
static const uint8_t magic[5] = {0x1E, 0x1B, 0x55, 0xCE, 0xAC};

// A client that is not part of the module, like a GUI tool: libcanard on the same controller
static CanardInstance client;
static uint8_t client_arena[64 * CANARD_MEM_BLOCK_SIZE] __aligned(8);
CAN_MSGQ_DEFINE(client_msgq, 32);

static struct uavcan_protocol_RestartNodeResponse response;
static bool got;
static bool decode_failed;
static int64_t answer_ms;
static uint8_t client_tid;
static int filter_id;

// Register what the node did
static K_SEM_DEFINE(restarted, 0, 8);
static int restart_count;
static int64_t restart_ms;
static int cb_count;
static int cb_ret;
static void *cb_data;

void dronecan_restart_now(const struct device *dev)
{
	restart_count++;
	restart_ms = k_uptime_get();
	k_sem_give(&restarted);
}

static int restart_cb(const struct device *dev, void *user_data)
{
	cb_count++;
	cb_data = user_data;
	return cb_ret;
}

static bool client_should_accept(const CanardInstance *ins, uint64_t *sig, uint16_t id,
				 CanardTransferType type, uint8_t source)
{
	if (type != CanardTransferTypeResponse || id != UAVCAN_PROTOCOL_RESTARTNODE_ID) {
		return false;
	}
	*sig = UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE;
	return true;
}

static void client_on_reception(CanardInstance *ins, CanardRxTransfer *t)
{
	decode_failed = uavcan_protocol_RestartNodeResponse_decode(t, &response);
	answer_ms = k_uptime_get();
	got = true;
}

static void send_request(const uint8_t *payload, uint16_t len)
{
	const CanardCANFrame *cf;

	zassert_true(canardRequestOrRespond(&client, 42, UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE,
					    UAVCAN_PROTOCOL_RESTARTNODE_ID, &client_tid,
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

// Send a request and wait for a response
static void restart_request(const uint8_t *payload, uint16_t len)
{
	k_timepoint_t end = sys_timepoint_calc(K_SECONDS(1));
	struct can_frame f;

	got = false;
	memset(&response, 0, sizeof(response));
	send_request(payload, len);

	while (!got && k_msgq_get(&client_msgq, &f, sys_timepoint_timeout(end)) == 0) {
		CanardCANFrame cf = {
			.id = f.id | CANARD_CAN_FRAME_EFF,
			.data_len = can_dlc_to_bytes(f.dlc),
		};

		memcpy(cf.data, f.data, cf.data_len);
		canardHandleRxFrame(&client, &cf, k_ticks_to_us_floor64(k_uptime_ticks()) + 1U);
	}
	zassert_true(got, "no response");
	zassert_false(decode_failed, "the response is not decoding");
}

// True if the node restarts within the time, for the tests that expect no restart
static bool restarts_within(int ms)
{
	return k_sem_take(&restarted, K_MSEC(ms)) == 0;
}

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

	restart_count = 0;
	cb_count = 0;
	cb_ret = 0;
	cb_data = NULL;
	k_sem_reset(&restarted);
	dronecan_set_restart_handler(dc, NULL, NULL);

	zassert_ok(dronecan_start(dc));
	filter_id = can_add_rx_filter_msgq(can, &client_msgq, &all);
	zassert_true(filter_id >= 0);
	k_msgq_purge(&client_msgq);
}

static void after(void *fixture)
{
	can_remove_rx_filter(can, filter_id);
	dronecan_stop(dc);
}

ZTEST_SUITE(dronecan_restart, NULL, setup, before, after, NULL);

ZTEST(dronecan_restart, test_accepted_without_a_callback)
{
	restart_request(magic, sizeof(magic));
	zassert_true(response.ok);

	// The answer leaves the bus first, and the restart comes after the delay
	zassert_equal(restart_count, 0, "restarted before the delay");
	zassert_true(restarts_within(DELAY_MS * 3), "no restart");
	zassert_equal(restart_count, 1);
	zassert_true(restart_ms - answer_ms >= DELAY_MS - 10, "restart %lld ms after the answer",
		     restart_ms - answer_ms);
}

ZTEST(dronecan_restart, test_callback_accepts)
{
	int marker;

	dronecan_set_restart_handler(dc, restart_cb, &marker);
	restart_request(magic, sizeof(magic));

	zassert_true(response.ok);
	zassert_equal(cb_count, 1);
	zassert_equal_ptr(cb_data, &marker, "the callback gets its user data");
	zassert_true(restarts_within(DELAY_MS * 3), "no restart");
}

ZTEST(dronecan_restart, test_callback_refuses)
{
	cb_ret = -EBUSY;
	dronecan_set_restart_handler(dc, restart_cb, NULL);
	restart_request(magic, sizeof(magic));

	zassert_false(response.ok);
	zassert_equal(cb_count, 1);
	zassert_false(restarts_within(DELAY_MS * 3), "restarted after a refusal");
}

ZTEST(dronecan_restart, test_wrong_magic_number)
{
	uint8_t wrong[sizeof(magic)];

	memcpy(wrong, magic, sizeof(wrong));
	wrong[4] ^= 0x01;
	dronecan_set_restart_handler(dc, restart_cb, NULL);
	restart_request(wrong, sizeof(wrong));

	zassert_false(response.ok);
	zassert_equal(cb_count, 0, "the callback is asked only with the right magic number");
	zassert_false(restarts_within(DELAY_MS * 3), "restarted with a wrong magic number");
}

ZTEST(dronecan_restart, test_short_request)
{
	dronecan_set_restart_handler(dc, restart_cb, NULL);
	restart_request(magic, sizeof(magic) - 1);

	zassert_false(response.ok);
	zassert_equal(cb_count, 0);
	zassert_false(restarts_within(DELAY_MS * 3), "restarted with a short request");
}

ZTEST(dronecan_restart, test_two_requests_one_restart)
{
	restart_request(magic, sizeof(magic));
	zassert_true(response.ok);
	restart_request(magic, sizeof(magic));
	zassert_true(response.ok);

	zassert_true(restarts_within(DELAY_MS * 3), "no restart");
	zassert_false(restarts_within(DELAY_MS * 3), "restarted twice");
	zassert_equal(restart_count, 1);
}

ZTEST(dronecan_restart, test_stop_does_not_cancel)
{
	restart_request(magic, sizeof(magic));
	zassert_true(response.ok);

	zassert_ok(dronecan_stop(dc));
	zassert_true(restarts_within(DELAY_MS * 3), "stopping the node cancelled the restart");

	// Leave the node started, as before() found it stopped
	zassert_ok(dronecan_start(dc));
}

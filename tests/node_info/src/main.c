/*
 * Copyright (c) 2026 3DR, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/drivers/can.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <canard.h>
#include <dronecan.h>
#include <uavcan.protocol.GetNodeInfo.h>

static const struct device *const dc = DEVICE_DT_GET(DT_NODELABEL(dronecan0)); // node 42
static const struct device *const can = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

#define CLIENT_ID 99

// A client that is not part of the module, like a GUI tool: libcanard on the same controller
static CanardInstance client;
static uint8_t client_arena[64 * CANARD_MEM_BLOCK_SIZE] __aligned(8);
CAN_MSGQ_DEFINE(client_msgq, 32);

static struct uavcan_protocol_GetNodeInfoResponse info;
static bool got;
static bool decode_failed;
static uint8_t client_tid;
static int filter_id;

static bool client_should_accept(const CanardInstance *ins, uint64_t *sig, uint16_t id,
				 CanardTransferType type, uint8_t source)
{
	if (type != CanardTransferTypeResponse || id != UAVCAN_PROTOCOL_GETNODEINFO_ID) {
		return false;
	}
	*sig = UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;
	return true;
}

static void client_on_reception(CanardInstance *ins, CanardRxTransfer *t)
{
	decode_failed = uavcan_protocol_GetNodeInfoResponse_decode(t, &info);
	got = true;
}

static void send_request(void)
{
	const CanardCANFrame *cf;
	uint8_t none = 0;

	zassert_true(canardRequestOrRespond(&client, 42, UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
					    UAVCAN_PROTOCOL_GETNODEINFO_ID, &client_tid,
					    CANARD_TRANSFER_PRIORITY_MEDIUM, CanardRequest, &none,
					    0) > 0);
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

// Ask the node about its information and wait for the reply
static void get_node_info(void)
{
	k_timepoint_t end = sys_timepoint_calc(K_SECONDS(1));
	struct can_frame f;

	got = false;
	memset(&info, 0, sizeof(info));
	send_request();

	while (!got && k_msgq_get(&client_msgq, &f, sys_timepoint_timeout(end)) == 0) {
		CanardCANFrame cf = {
			.id = f.id | CANARD_CAN_FRAME_EFF,
			.data_len = can_dlc_to_bytes(f.dlc),
		};

		memcpy(cf.data, f.data, cf.data_len);
		canardHandleRxFrame(&client, &cf, k_ticks_to_us_floor64(k_uptime_ticks()) + 1U);
	}
	zassert_true(got, "no response");
	zassert_false(decode_failed, "the response does not decode");
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

ZTEST_SUITE(dronecan_node_info, NULL, setup, before, after, NULL);

ZTEST(dronecan_node_info, test_name_and_versions)
{
	uint8_t uid[16] = {0};

	get_node_info();

	zassert_equal(info.name.len, strlen(CONFIG_DRONECAN_NODE_NAME));
	zassert_mem_equal(info.name.data, CONFIG_DRONECAN_NODE_NAME, info.name.len);

	// The hardware version is the product ID: 0x0102
	zassert_equal(info.hardware_version.major, 0x01);
	zassert_equal(info.hardware_version.minor, 0x02);

	zassert_equal(info.hardware_version.certificate_of_authenticity.len, 0);

	// data given by git
	if (info.software_version.optional_field_flags &
	    UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_VCS_COMMIT) {
		zassert_not_equal(info.software_version.vcs_commit, 0);
	} else {
		zassert_equal(info.software_version.vcs_commit, 0);
	}
	zassert_equal(info.software_version.image_crc, 0);

	zassert_true(hwinfo_get_device_id(uid, sizeof(uid)) > 0);
	zassert_mem_equal(info.hardware_version.unique_id, uid, sizeof(uid),
			  "the unique ID is not the MCU serial number");
}

ZTEST(dronecan_node_info, test_status_follows_the_setters)
{
	get_node_info();
	zassert_equal(info.status.health, DRONECAN_HEALTH_OK);
	zassert_equal(info.status.mode, DRONECAN_MODE_OPERATIONAL);
	zassert_equal(info.status.vendor_specific_status_code, 0);

	zassert_ok(dronecan_set_health(dc, DRONECAN_HEALTH_WARNING));
	zassert_ok(dronecan_set_mode(dc, DRONECAN_MODE_MAINTENANCE));
	zassert_ok(dronecan_set_vendor_status(dc, 0x1234));

	get_node_info();
	zassert_equal(info.status.health, DRONECAN_HEALTH_WARNING);
	zassert_equal(info.status.mode, DRONECAN_MODE_MAINTENANCE);
	zassert_equal(info.status.vendor_specific_status_code, 0x1234);
}

ZTEST(dronecan_node_info, test_uptime_advances)
{
	uint32_t first;

	get_node_info();
	first = info.status.uptime_sec;
	k_sleep(K_MSEC(1200));
	get_node_info();
	zassert_true(info.status.uptime_sec > first, "uptime %u then %u", first,
		     info.status.uptime_sec);
}

ZTEST(dronecan_node_info, test_service_is_taken)
{
	struct dronecan_service svc;
	const struct dronecan_msg_desc desc = {UAVCAN_PROTOCOL_GETNODEINFO_ID, 1, 0};

	zassert_equal(dronecan_register_service(dc, &svc, &desc, NULL, NULL), -EINVAL);
	zassert_equal(dronecan_register_service(dc, &svc, &desc,
						(dronecan_service_cb_t)0x1, NULL), -EEXIST,
		      "the node's own GetNodeInfo is registered");
}

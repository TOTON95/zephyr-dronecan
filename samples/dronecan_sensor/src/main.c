/*
 * Copyright (c) 2026 3DR, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include <dronecan.h>
#include <uavcan.equipment.air_data.RawAirData.h>
#include <uavcan.equipment.power.BatteryInfo.h>
#include <uavcan.equipment.range_sensor.Measurement.h>

LOG_MODULE_REGISTER(dronecan_sensor_sample, LOG_LEVEL_INF);

static const struct device *const dc = DEVICE_DT_GET(DT_NODELABEL(dronecan0));

#define TICK_MS 50 // air data every tick (20 Hz), battery and range every other tick (10 Hz)

static const struct dronecan_msg_desc battery_desc = {
	.id = UAVCAN_EQUIPMENT_POWER_BATTERYINFO_ID,
	.signature = UAVCAN_EQUIPMENT_POWER_BATTERYINFO_SIGNATURE,
	.max_size = UAVCAN_EQUIPMENT_POWER_BATTERYINFO_MAX_SIZE,
};
static const struct dronecan_msg_desc air_desc = {
	.id = UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_ID,
	.signature = UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_SIGNATURE,
	.max_size = UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_MAX_SIZE,
};
static const struct dronecan_msg_desc range_desc = {
	.id = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_ID,
	.signature = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SIGNATURE,
	.max_size = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_MAX_SIZE,
};

#define BATT_FULL_WH  74.0f
#define BATT_V_FULL   16.8f   // 4S pack
#define BATT_V_EMPTY  14.0f

// A 4S battery that is slowly discharged (~35 min), with a load that varies
static void send_battery(float t)
{
	struct uavcan_equipment_power_BatteryInfo msg = {0};
	uint8_t buf[UAVCAN_EQUIPMENT_POWER_BATTERYINFO_MAX_SIZE];

	// Load: 8 A with a +/- 2 A ripple
	float current = 8.0f + 2.0f * sinf(0.5f * t);

	// Charge drawn so far
	float charge_as = 8.0f * t + 4.0f * (1.0f - cosf(0.5f * t));

	// V = V_FULL - k * used_wh, so dE/dt = V * I / 3600
	float k = (BATT_V_FULL - BATT_V_EMPTY) / BATT_FULL_WH;
	float used_wh = (BATT_V_FULL / k) * (1.0f - expf(-k * charge_as / 3600.0f));

	float soc = fmaxf(100.0f * (1.0f - used_wh / BATT_FULL_WH), 0.0f);
	float voltage = BATT_V_EMPTY + (BATT_V_FULL - BATT_V_EMPTY) * soc / 100.0f;

	msg.temperature = 305.0f; // fixed temperature in Kelvin
	msg.voltage = voltage;
	msg.current = current;
	msg.average_power_10sec = voltage * current;
	msg.full_charge_capacity_wh = BATT_FULL_WH;
	msg.remaining_capacity_wh = BATT_FULL_WH * soc / 100.0f;
	msg.status_flags = UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_IN_USE;
	msg.state_of_health_pct = 100;
	msg.state_of_charge_pct = (uint8_t)(soc + 0.5f);
	msg.battery_id = 0; // unique per battery
	msg.model_instance_id = 0x3d5;
	msg.model_name.len = sizeof("Emulated Batt") - 1;
	memcpy(msg.model_name.data, "Emulated Batt", msg.model_name.len);

	dronecan_broadcast(dc, &battery_desc, DRONECAN_PRIORITY_LOW, buf,
			   uavcan_equipment_power_BatteryInfo_encode(&msg, buf));
}

// Sea level, with a dynamic pressure that goes up and down around 14 m/s
static void send_air_data(float t)
{
	struct uavcan_equipment_air_data_RawAirData msg = {0};
	uint8_t buf[UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_MAX_SIZE];

	msg.static_pressure = 101325.0f;
	msg.differential_pressure = 120.0f + 60.0f * sinf(0.3f * t);
	msg.static_pressure_sensor_temperature = 295.0f;
	msg.differential_pressure_sensor_temperature = 295.0f;
	msg.static_air_temperature = 288.15f;
	msg.pitot_temperature = 288.15f;

	dronecan_broadcast(dc, &air_desc, DRONECAN_PRIORITY_MEDIUM, buf,
			   uavcan_equipment_air_data_RawAirData_encode(&msg, buf));
}

// A lidar looking down: the range goes between 0.5 and 3.5 m
static void send_range(float t)
{
	struct uavcan_equipment_range_sensor_Measurement msg = {0};
	uint8_t buf[UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_MAX_SIZE];

	msg.field_of_view = 0.1f;
	msg.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_LIDAR;
	msg.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE;
	msg.range = 2.0f + 1.5f * sinf(0.2f * t);

	dronecan_broadcast(dc, &range_desc, DRONECAN_PRIORITY_LOW, buf,
			   uavcan_equipment_range_sensor_Measurement_encode(&msg, buf));
}

#if DT_NODE_EXISTS(DT_NODELABEL(dronecan_fc))
// On native_sim a second node plays the flight controller: it grants the ID and counts what arrives
#define HAS_ALLOCATOR 1
static const struct device *const fc = DEVICE_DT_GET(DT_NODELABEL(dronecan_fc));
static struct dronecan_subscription fc_subs[3];
static const struct dronecan_msg_desc *const fc_descs[] = {&battery_desc, &air_desc, &range_desc};
static atomic_t fc_count[3];

static void fc_cb(const struct device *dev, const struct dronecan_transfer *t, void *user_data)
{
	atomic_inc(user_data);
}

static void allocator_start(void)
{
	LOG_INF("allocator start: %d", dronecan_start(fc));
	for (int i = 0; i < 3; i++) {
		dronecan_subscribe(fc, &fc_subs[i], fc_descs[i], fc_cb, &fc_count[i]);
	}
}

static void allocator_report(void)
{
	LOG_INF("allocator received: battery %ld, air data %ld, range %ld", (long)atomic_set(&fc_count[0], 0),
		(long)atomic_set(&fc_count[1], 0), (long)atomic_set(&fc_count[2], 0));
}
#endif

#if DT_HAS_CHOSEN(zephyr_console) && DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart)
#define CONSOLE_IS_CDC_ACM 1
#endif

// A USB console shows nothing until the host opens the port: wait for it (at most 15 s).
static void wait_for_console(void)
{
#ifdef CONSOLE_IS_CDC_ACM
	const struct device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t dtr = 0;

	for (int i = 0; i < 150 && dtr == 0; i++) {
		uart_line_ctrl_get(console, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(100));
	}
	k_sleep(K_MSEC(200)); // let the terminal settle
#endif
}

int main(void)
{
	struct dronecan_stats s;
	int64_t next;
	int ret;

	wait_for_console();
	LOG_INF("dronecan sensor sample");

	if (!device_is_ready(dc)) {
		LOG_ERR("%s is not ready", dc->name);
		return -ENODEV;
	}

#ifdef HAS_ALLOCATOR
	allocator_start();
#endif
	LOG_INF("start: %d", dronecan_start(dc));

	// With node ID 0 the node stays silent until an allocator on the bus grants it an ID
	while (dronecan_get_node_id(dc) == DRONECAN_NODE_ID_ANON &&
	       (ret = dronecan_dnia_request(dc, K_SECONDS(10))) != 0) {
		LOG_WRN("no node ID yet: %d", ret);
	}
	LOG_INF("node id: %u", dronecan_get_node_id(dc));

	next = k_uptime_get();
	for (uint32_t tick = 0;; tick++) {
		float t = (float)k_uptime_get() / 1000.0f;

		send_air_data(t);
		if (tick % 2 == 0) {
			send_battery(t);
		} else {
			send_range(t);
		}

		if (tick % 40 == 39) { // every 2 s
			dronecan_get_stats(dc, &s);
			LOG_INF("tx=%u tx_err=%u pool %u/%u (peak %u)", s.tx, s.tx_err, s.mem_current,
				s.mem_capacity, s.mem_peak);
#ifdef HAS_ALLOCATOR
			allocator_report();
#endif
		}

		next += TICK_MS;
		k_sleep(K_TIMEOUT_ABS_MS(next));
	}

	return 0;
}

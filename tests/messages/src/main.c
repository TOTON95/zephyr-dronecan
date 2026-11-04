/*
 * Copyright (c) 2026 3DR, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "dronecan_msg_table.h"
#include "expected_includes.h"

// ID, signature and size that dronecan_dsdlc generated for each message in the table.
// expected.inc is written by tests/messages/CMakeLists.txt from the names in the table.
static const struct {
	const char *name;
	uint16_t id;
	uint64_t signature;
	uint16_t max_size;
} expected[] = {
#include "expected.inc"
};

static const struct dronecan_msg_entry *find(const char *name)
{
	for (size_t i = 0; i < dronecan_msg_table_len; i++) {
		if (strcmp(dronecan_msg_table[i].name, name) == 0) {
			return &dronecan_msg_table[i];
		}
	}
	return NULL;
}

ZTEST(dronecan_messages, test_table_matches_the_generated_headers)
{
	int wrong = 0;

	zassert_true(ARRAY_SIZE(expected) > 0);
	zassert_equal(dronecan_msg_table_len, ARRAY_SIZE(expected),
		      "table has %zu messages, %zu generated", dronecan_msg_table_len,
		      ARRAY_SIZE(expected));

	for (size_t i = 0; i < ARRAY_SIZE(expected); i++) {
		const struct dronecan_msg_entry *e = find(expected[i].name);
		bool same;

		if (e == NULL) {
			TC_PRINT("%s: not in the table\n", expected[i].name);
			wrong++;
			continue;
		}

		same = e->desc.id == expected[i].id && e->desc.signature == expected[i].signature &&
		       e->desc.max_size == expected[i].max_size;
		if (!same) {
			unsigned long long table_sig = e->desc.signature;
			unsigned long long gen_sig = expected[i].signature;

			TC_PRINT("%s: table (%u, 0x%llx, %u), generated (%u, 0x%llx, %u)\n",
				 expected[i].name, e->desc.id, table_sig, e->desc.max_size,
				 expected[i].id, gen_sig, expected[i].max_size);
			wrong++;
		}
	}
	zassert_equal(wrong, 0, "%d messages differ, run scripts/gen_kconfig_messages.py", wrong);
}

ZTEST_SUITE(dronecan_messages, NULL, NULL, NULL, NULL, NULL);

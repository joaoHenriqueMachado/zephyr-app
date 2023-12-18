/*
 * Copyright (c) 2022 Rodrigo Peixoto <rodrigopex@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */
#include "messages.h"

#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
LOG_MODULE_DECLARE(zbus, CONFIG_ZBUS_LOG_LEVEL);

ZBUS_CHAN_DEFINE(raw_data_chan, struct sensor_msg, NULL, NULL, ZBUS_OBSERVERS(producer_lis, consumer_sub),
		 ZBUS_MSG_INIT(.x = 0, .y = 0, .z = 0));

static void producer_listener_cb(const struct zbus_channel *chan)
{
	const struct sensor_msg *acc = zbus_chan_const_msg(chan);

	LOG_INF("From listener -> Acc x=%d, y=%d, z=%d", acc->x, acc->y, acc->z);
}

ZBUS_LISTENER_DEFINE(producer_lis, producer_listener_cb);

static void producer_thread(void)
{
	struct sensor_msg acc = {0};
	uint32_t count = 0;

	while (1) {
		++acc.x;
		++acc.y;
		++acc.z;

		LOG_INF(" >-- Raw data fetched");

		zbus_chan_pub(&raw_data_chan, &acc, K_MSEC(200));

		k_msleep(500);

		++count;
	}
}

K_THREAD_DEFINE(produce_thread_id, 1024, producer_thread, NULL, NULL, NULL, 3, 0, 0);

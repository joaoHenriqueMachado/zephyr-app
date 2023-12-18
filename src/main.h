// Zephyr includes
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

// Standard includes
#include <inttypes.h>
#include <string.h>
#include <stdint.h>

#include "messages.h"

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#define STACKSIZE 1024
#define PRIORITY 7
#define SLEEP_TIME_MS 200
#define DEBOUNCE_TIME_MS 50
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <inttypes.h>
#include <string.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include "messages.h"

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define SW0_NODE DT_ALIAS(sw0)

#define SLEEP_TIME_MS 200
#define DEBOUNCE_TIME_MS 50

#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif


#if !DT_NODE_HAS_STATUS(LED1_NODE, okay)
#error "Unsupported board: led1 devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

struct led {
	struct gpio_dt_spec spec;
	uint8_t num;
};

struct sw {
	struct gpio_dt_spec spec;
	struct gpio_callback cb;
};

static const struct led led0 = {
	.spec = GPIO_DT_SPEC_GET_OR(LED0_NODE, gpios, {0}),
	.num = 0,
};

static const struct led led1 = {
	.spec = GPIO_DT_SPEC_GET_OR(LED1_NODE, gpios, {0}),
	.num = 0,
};

struct k_timer led_timer;

struct sw sw0 = {
	.spec = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0}),
	.cb = NULL,
};

void blinky_callback(struct k_timer *timer)
{
	gpio_pin_toggle(led0.spec.port, led0.spec.pin);
}

void blink(const struct led *led, uint32_t sleep_ms, uint32_t id)
{
	const struct gpio_dt_spec *spec = &led->spec;
	int ret;

	if (!device_is_ready(spec->port)) {
		printk("Error: %s device is not ready\n", spec->port->name);
		return;
	}

	ret = gpio_pin_configure_dt(spec, GPIO_OUTPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure pin %d (LED '%d')\n",
			ret, spec->pin, led->num);
		return;
	}

	k_timer_init(&led_timer, blinky_callback, NULL);
	k_timer_start(&led_timer, K_MSEC(sleep_ms), K_MSEC(sleep_ms));

	k_sleep(K_FOREVER);
}

static int64_t previous_cb_time = 0;

void button_pressed(struct sw *sw, unsigned int pins)
{
	int64_t current_cb_time = k_uptime_get();
	if(current_cb_time - previous_cb_time > DEBOUNCE_TIME_MS)
	{
		printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
		previous_cb_time = current_cb_time;
	}
}

void button(const struct sw *sw, const struct led *led0, uint32_t sleep_ms)
{
	const struct gpio_dt_spec *spec = &sw->spec;
	struct gpio_callback *cb = &sw->cb;

	struct gpio_dt_spec *led = &led0->spec;
	int ret;

	if(!gpio_is_ready_dt(spec))
	{
		printk("Error: button device %s is not ready\n", spec->port->name);
		return;
	}

	ret = gpio_pin_configure_dt(spec, GPIO_INPUT);
	if(ret != 0)
	{
		printk("Error %d: failed to configure %s pin %d\n", ret, spec->port->name, spec->pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(spec, GPIO_INT_EDGE_TO_ACTIVE);
	if(ret != 0)
	{
		printk("Error %d: failed to configure interrupt on %s pin %d\n", ret, spec->port->name, spec->pin);
		return;
	}

	gpio_init_callback(cb, button_pressed, BIT(spec->pin));
	gpio_add_callback(spec->port, cb);
	printk("Set up button at %s pin %d\n", spec->port->name, spec->pin);

	if (led->port && !gpio_is_ready_dt(led)) {
		printk("Error %d: LED device %s is not ready; ignoring it\n",
		       ret, led->port->name);
		led->port = NULL;
	}
	if (led->port) {
		ret = gpio_pin_configure_dt(led, GPIO_OUTPUT);
		if (ret != 0) {
			printk("Error %d: failed to configure LED device %s pin %d\n",
			       ret, led->port->name, led->pin);
			led->port = NULL;
		} else {
			printk("Set up LED at %s pin %d\n", led->port->name, led->pin);
		}
	}

	printk("Press the button\n");
	if (led->port) {
		while (1) {
			int val = gpio_pin_get_dt(spec);

			if (val >= 0) {
				gpio_pin_set_dt(led, val);
			}
			k_msleep(SLEEP_TIME_MS);
		}
	}
}

void button0(void)
{
	button(&sw0, &led1, 200);
}

void blink0(void)
{
	blink(&led0, 200, 0);	
}

K_THREAD_DEFINE(blink0_id, STACKSIZE, blink0, NULL, NULL, NULL,PRIORITY, 0, 0);
K_THREAD_DEFINE(button0_id, STACKSIZE, button0, NULL, NULL, NULL,PRIORITY, 0, 0);

int main(void)
{
	return 0;
}

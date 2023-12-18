#include "led.h"

static const struct led led0 = {
	.spec = GPIO_DT_SPEC_GET_OR(LED0_NODE, gpios, {0}),
	.num = 0,
};

struct k_timer led_timer;

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

void blink0(void)
{
	blink(&led0, 200, 0);	
}

K_THREAD_DEFINE(blink0_id, STACKSIZE, blink0, NULL, NULL, NULL,PRIORITY, 0, 0);
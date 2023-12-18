#include "button.h"
#include "led.h"

static const struct led led1 = {
	.spec = GPIO_DT_SPEC_GET_OR(LED1_NODE, gpios, {0}),
	.num = 0,
};

struct sw sw0 = {
	.spec = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0}),
	.cb = NULL,
};

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

K_THREAD_DEFINE(button0_id, STACKSIZE, button0, NULL, NULL, NULL,PRIORITY, 0, 0);

#include "dac.h"

static const struct device *const dac_dev = DEVICE_DT_GET_OR_NULL(DAC_DEVICE_NODE);

static const struct dac_channel_cfg dac_ch_cfg = {
	.channel_id  = DAC_CHANNEL_ID,
	.resolution  = DAC_RESOLUTION
};

void dac(struct device *device)
{
    if (!device_is_ready(dac_dev)) {
		printk("DAC device %s is not ready\n", dac_dev->name);
		return 0;
	}

	int ret = dac_channel_setup(dac_dev, &dac_ch_cfg);

	if (ret != 0) {
		printk("Setting up of DAC channel failed with code %d\n", ret);
		return 0;
	}

	printk("Generating sawtooth signal at DAC channel %d.\n",
		DAC_CHANNEL_ID);
	while (1) {
		const int dac_values = 1U << DAC_RESOLUTION;

		const int sleep_time = 4096 / dac_values > 0 ?
			4096 / dac_values : 1;

		for (int i = 0; i < dac_values; i++) {
			ret = dac_write_value(dac_dev, DAC_CHANNEL_ID, i);
			if (ret != 0) {
				printk("dac_write_value() failed with code %d\n", ret);
				return 0;
			}
			k_sleep(K_MSEC(sleep_time));
		}
	}
}

void dac0(void)
{
	dac(dac_dev);
}

K_THREAD_DEFINE(dac0_id, STACKSIZE, dac0, NULL, NULL, NULL,PRIORITY, 0, 0);

#include "main.h"

#define SW0_NODE DT_ALIAS(sw0)

#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

struct sw {
	struct gpio_dt_spec spec;
	struct gpio_callback cb;
};
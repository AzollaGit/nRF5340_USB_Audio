#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>


#define SLEEP_TIME_MS	1000

/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */
#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button_cb_data;
 
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led1), gpios, {0});
 
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printk("Button[0x%x] pressed at %" PRIu32 "\n", pins, k_cycle_get_32());
}

void led0_toggle(void)
{
    gpio_pin_toggle_dt(&led0);
    gpio_pin_toggle_dt(&led1);
}
 


int hal_gpio_init(void)
{
	int ret = 0;

	if (!gpio_is_ready_dt(&button)) {
		printk("Error: button device %s is not ready\n", button.port->name);
		return ret;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n", ret, button.port->name, button.pin);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n", ret, button.port->name, button.pin);
		return ret;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);

    const struct gpio_dt_spec leds[2] = {led0, led1};
    for (uint8_t i = 0; i < 2; i++) {
        if (!gpio_is_ready_dt(&leds[i])) {
            printk("Error %d: LED device %s is not ready; ignoring it\n", ret, leds[i].port->name);
            continue;
        }
        ret = gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT);
        if (ret != 0) {
            printk("Error %d: failed to configure LED device %s pin %d\n", ret, leds[i].port->name, leds[i].pin);
        } else {
            printk("Set up LED at %s pin %d\n", leds[i].port->name, leds[i].pin);
        }
    }
 
#if 0 // test...    
 
	printk("Press the button\n");
    while (1) {
        /* If we have an LED, match its state to the button's. */
        int val = gpio_pin_get_dt(&button);
        if (val >= 0) {
            gpio_pin_set_dt(&led1, val);
        }

        gpio_pin_toggle_dt(&led0);
        k_msleep(SLEEP_TIME_MS);
    }
#endif

	return ret;
}

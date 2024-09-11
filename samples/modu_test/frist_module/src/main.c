/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>

#define LED0_MODE DT_ALIAS(led0)


#if DT_NODE_HAS_STATUS(LED0_MODE, okay)
#else
#error "LED0 devicetree alias is not defined"
#endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_MODE, gpios);
int main(void)
{
    int ret;
    int vale = 0;

    if (!gpio_is_ready_dt(&led)) {
        return 0;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 0;
    }

    while (1) {
//        ret = gpio_pin_toggle_dt(&led);
        ret = gpio_pin_set_dt(&led, vale);
        if (ret < 0) {
            return 0;
        }
        vale = !vale;
        k_msleep(1000);
    }
    return 0;

	/*printf("Hello World! %s\n", CONFIG_BOARD);
	while(1) {
		printf("hello time num:%d\r\n",TestNum);
		k_sleep(K_MSEC(1000));
        TestNum++;
	}*/
	return 0;
}

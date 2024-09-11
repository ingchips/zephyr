/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#define LED_ENABLE (0)
#define KEY_ENABLE (0)
#define ADC_ENABLE (1)

#if LED_ENABLE
#include <zephyr/drivers/gpio.h>

#define LED0_MODE DT_ALIAS(led1)
#if DT_NODE_HAS_STATUS(LED0_MODE, okay)
#else
#error "LED0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_MODE, gpios);
#endif

#if KEY_ENABLE
#include <zephyr/drivers/gpio.h>

#define BUTTON_NODE DT_ALIAS(key0)

#if (!DT_NODE_HAS_STATUS(BUTTON_NODE, okay))
#error "Unsupported board: button_0 devicetree alias is not defined"
#endif

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);

/* Button pressed callback function */
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printf("Button pressed at pin %d\n", button.pin);
}

/* GPIO callback structure */
static struct gpio_callback button_cb_data;
#endif

#if ADC_ENABLE
#include <zephyr/drivers/adc.h>

#define ADC_RESOLUTION 12
#define ADC_CHANNEL_ID 1

#endif

int main(void)
{
#if LED_ENABLE
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
#endif

#if KEY_ENABLE
    int ret;

    /* Check if the GPIO device is ready */
    if (!device_is_ready(button.port)) {
        printf("Error: button device %s is not ready\n", button.port->name);
        return 0;
    }

    /* Configure the GPIO pin as input with pull-up/pull-down (if necessary) */
    ret = gpio_pin_configure_dt(&button, GPIO_INPUT|GPIO_PULL_UP);
    if (ret != 0) {
        printf("Error %d: failed to configure %s pin %d\n", ret, button.port->name, button.pin);
        return 0;
    }

    /* Configure the interrupt callback for button press */
    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_LEVEL_LOW);
    if (ret != 0) {
        printf("Error %d: failed to configure interrupt on %s pin %d\n", ret, button.port->name, button.pin);
        return 0;
    }

    /* Initialize the GPIO callback structure */
    gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);

    /* Main loop */
    while (1) {
        /* Read the GPIO pin value */
        int val = gpio_pin_get_dt(&button);
        printf("Button state: %d\n", val);
        k_sleep(K_MSEC(1000));  // Delay for 1 second
    }
    return 0;
#endif

#if ADC_ENABLE

    const struct device *adc_dev;
    struct adc_channel_cfg channel_cfg;
    struct adc_sequence sequence;
    uint16_t sample_buffer[1];
    int err;

//    adc_dev = device_get_binding(DT_LABEL(DT_NODELABEL(adc0)));
    if (!adc_dev) {
        printf("ADC device not found\n");
        return 0;
    }

    channel_cfg.gain = ADC_GAIN_1;
    channel_cfg.reference = ADC_REF_INTERNAL;
    channel_cfg.acquisition_time = ADC_ACQ_TIME_DEFAULT;
    channel_cfg.channel_id = ADC_CHANNEL_ID;
    channel_cfg.differential = 0;

    err = adc_channel_setup(adc_dev, &channel_cfg);
    if (err) {
        printf("Failed to setup ADC channel (err %d)\n", err);
        return 0;
    }

    sequence.options = NULL;
    sequence.channels = BIT(ADC_CHANNEL_ID);
    sequence.buffer = sample_buffer;
    sequence.buffer_size = sizeof(sample_buffer);
    sequence.resolution = ADC_RESOLUTION;

    err = adc_read(adc_dev, &sequence);
    if (err) {
        printf("ADC read failed (err %d)\n", err);
        return 0;
    }

    printf("ADC value: %d\n", sample_buffer[0]);
#endif
}

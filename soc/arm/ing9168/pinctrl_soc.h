/*
 * Copyright (c) 2020 Linaro Ltd.
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * STM32 SoC specific helpers for pinctrl driver
 */

#ifndef ZEPHYR_SOC_ARM_ST_STM32_COMMON_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ARM_ST_STM32_COMMON_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>


#ifdef __cplusplus
extern "C" {
#endif

/** @cond INTERNAL_HIDDEN */

/** Type for STM32 pin. */
typedef struct pinctrl_soc_pin {
	/** Pinmux settings (port, pin and function). */
	uint32_t pin;
	/** Pin configuration (bias, drive and slew rate). */
	uint32_t pinmux;

	uint32_t pull;
} pinctrl_soc_pin_t;

/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_ARM_ST_STM32_COMMON_PINCTRL_SOC_H_ */

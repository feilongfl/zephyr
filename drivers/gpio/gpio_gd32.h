/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_GPIO_GPIO_GD32_H_
#define ZEPHYR_DRIVERS_GPIO_GPIO_GD32_H_

/**
 * @file header for GD32 GPIO
 */

#include <drivers/clock_control/gd32_clock_control.h>
#include <pinmux/pinmux_gd32.h>
#include <drivers/gpio.h>

/* GPIO buses definitions */

#define GD32_PORT_NOT_AVAILABLE 0xFFFFFFFF

/**
 * @brief configuration of GPIO device
 */
struct gpio_gd32_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	/* port base address */
	uint32_t *base;
	/* IO port */
	int port;
	struct gd32_pclken pclken;
};

/**
 * @brief driver data
 */
struct gpio_gd32_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	/* device's owner of this data */
	const struct device *dev;
	/* user ISR cb */
	sys_slist_t cb;
#ifdef CONFIG_PM_DEVICE
	/* device power state */
	enum pm_device_state power_state;
#endif
};

/**
 * @brief helper for configuration of GPIO pin
 *
 * @param dev GPIO port device pointer
 * @param pin IO pin
 * @param conf GPIO mode
 * @param altf Alternate function
 */
int gpio_gd32_configure(const struct device *dev, int pin, int conf, int altf);

/**
 * @brief Enable / disable GPIO port clock.
 *
 * @param dev GPIO port device pointer
 * @param on boolean for on/off clock request
 */
int gpio_gd32_clock_request(const struct device *dev, bool on);

#endif /* ZEPHYR_DRIVERS_GPIO_GPIO_GD32_H_ */

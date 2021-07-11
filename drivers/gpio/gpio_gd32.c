/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gd_gd32_gpio

#include <errno.h>

#include <kernel.h>
#include <device.h>
#include <soc.h>
#include <drivers/gpio.h>
#include <drivers/clock_control/gd32_clock_control.h>
#include <pinmux/pinmux_gd32.h>
#include <drivers/pinmux.h>
#include <sys/util.h>
#include <pm/device.h>
#include <pm/device_runtime.h>

#include "gpio_gd32.h"
#include "gpio_utils.h"

static int gpio_gd32_flags_to_conf(int flags, int *pincfg)
{

	if ((flags & GPIO_OUTPUT) != 0) {
		/* Output only or Output/Input */

		*pincfg = GD32_PINCFG_MODE_OUTPUT;

		if ((flags & GPIO_SINGLE_ENDED) != 0) {
			if (flags & GPIO_LINE_OPEN_DRAIN) {
				*pincfg |= GD32_PINCFG_OPEN_DRAIN;
			} else  {
				/* Output can't be open source */
				return -ENOTSUP;
			}
		} else {
			*pincfg |= GD32_PINCFG_PUSH_PULL;
		}

		if ((flags & GPIO_PULL_UP) != 0) {
			*pincfg |= GD32_PINCFG_PULL_UP;
		} else if ((flags & GPIO_PULL_DOWN) != 0) {
			*pincfg |= GD32_PINCFG_PULL_DOWN;
		}

	} else if  ((flags & GPIO_INPUT) != 0) {
		/* Input */

		*pincfg = GD32_PINCFG_MODE_INPUT;

		if ((flags & GPIO_PULL_UP) != 0) {
			*pincfg |= GD32_PINCFG_PULL_UP;
		} else if ((flags & GPIO_PULL_DOWN) != 0) {
			*pincfg |= GD32_PINCFG_PULL_DOWN;
		} else {
			*pincfg |= GD32_PINCFG_FLOATING;
		}
	} else {
		/* Desactivated: Analog */
		*pincfg = GD32_PINCFG_MODE_ANALOG;
	}

	return 0;
}

/**
 * @brief Configure the hardware.
 */
int gpio_gd32_configure(const struct device *dev, int pin, int conf, int altf)
{
	const struct gpio_gd32_config *cfg = dev->config;
	uint32_t speed = GPIO_OSPEED_10MHZ;
	uint32_t temp = conf & (GD32_MODE_OSPEED_MASK << GD32_MODE_OSPEED_SHIFT);

	ARG_UNUSED(altf);

	switch (temp)
	{
	case GD32_MODE_OUTPUT_MAX_2:
		speed = GPIO_OSPEED_2MHZ;
		break;

	case GD32_MODE_OUTPUT_MAX_10:
		speed = GPIO_OSPEED_10MHZ;
		break;

	case GD32_MODE_OUTPUT_MAX_50:
		speed = GPIO_OSPEED_50MHZ;
		break;

	default:
		speed = GPIO_OSPEED_MAX;
		break;
	}

	gpio_init((uint32_t)cfg->base, conf, speed, BIT(pin));

	return 0;
}

/**
 * @brief GPIO port clock handling
 */
int gpio_gd32_clock_request(const struct device *dev, bool on)
{
	const struct gpio_gd32_config *cfg = dev->config;
	int ret = 0;

	__ASSERT_NO_MSG(dev != NULL);

	/* enable clock for subsystem */
	const struct device *clk = DEVICE_DT_GET(GD32_CLOCK_CONTROL_NODE);

	if (on) {
		ret = clock_control_on(clk,
					(clock_control_subsys_t *)&cfg->pclken);
	} else {
		ret = clock_control_off(clk,
					(clock_control_subsys_t *)&cfg->pclken);
	}

	if (ret != 0) {
		return ret;
	}

	return ret;
}

static inline uint32_t gpio_gd32_pin_to_exti_line(int pin)
{
	return 0;
}

static int gpio_gd32_port_get_raw(const struct device *dev, uint32_t *value)
{
	const struct gpio_gd32_config *cfg = dev->config;

	*value = (uint32_t)gpio_input_port_get((uint32_t)cfg->base);

	return 0;
}

static int gpio_gd32_port_set_masked_raw(const struct device *dev,
					  gpio_port_pins_t mask,
					  gpio_port_value_t value)
{
	return 0;
}

static int gpio_gd32_port_set_bits_raw(const struct device *dev,
					gpio_port_pins_t pins)
{
	const struct gpio_gd32_config *cfg = dev->config;
        gpio_bit_set((uint32_t)cfg->base, pins);

	return 0;
}

static int gpio_gd32_port_clear_bits_raw(const struct device *dev,
					  gpio_port_pins_t pins)
{
	const struct gpio_gd32_config *cfg = dev->config;
        gpio_bit_reset((uint32_t)cfg->base, pins);

	return 0;
}

static int gpio_gd32_port_toggle_bits(const struct device *dev,
				       gpio_port_pins_t pins)
{

	return 0;
}



/**
 * @brief Configure pin or port
 */
static int gpio_gd32_config(const struct device *dev,
			     gpio_pin_t pin, gpio_flags_t flags)
{
	//todo: add pm device
	int err = 0;
	int pincfg = 0;

	/* figure out if we can map the requested GPIO
	 * configuration
	 */
	err = gpio_gd32_flags_to_conf(flags, &pincfg);
	if (err != 0) {
		goto exit;
	}

	if ((flags & GPIO_OUTPUT) != 0) {
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			gpio_gd32_port_set_bits_raw(dev, BIT(pin));
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			gpio_gd32_port_clear_bits_raw(dev, BIT(pin));
		}
	}

	gpio_gd32_configure(dev, pin, pincfg, 0);

exit:
	return err;
}

static int gpio_gd32_pin_interrupt_configure(const struct device *dev,
					      gpio_pin_t pin,
					      enum gpio_int_mode mode,
					      enum gpio_int_trig trig)
{
	return 0;
}

static int gpio_gd32_manage_callback(const struct device *dev,
				      struct gpio_callback *callback,
				      bool set)
{
	return 0;
}

static const struct gpio_driver_api gpio_gd32_driver = {
	.pin_configure = gpio_gd32_config,
	.port_get_raw = gpio_gd32_port_get_raw,
	.port_set_masked_raw = gpio_gd32_port_set_masked_raw,
	.port_set_bits_raw = gpio_gd32_port_set_bits_raw,
	.port_clear_bits_raw = gpio_gd32_port_clear_bits_raw,
	.port_toggle_bits = gpio_gd32_port_toggle_bits,
	.pin_interrupt_configure = gpio_gd32_pin_interrupt_configure,
	.manage_callback = gpio_gd32_manage_callback,
};

/**
 * @brief Initialize GPIO port
 *
 * Perform basic initialization of a GPIO port. The code will
 * enable the clock for corresponding peripheral.
 *
 * @param dev GPIO device struct
 *
 * @return 0
 */
static int gpio_gd32_init(const struct device *dev)
{
	struct gpio_gd32_data *data = dev->data;

	data->dev = dev;

	return gpio_gd32_clock_request(dev, true);
}

#ifdef CONFIG_PM_DEVICE
static int gpio_gd32_pm_device_ctrl(const struct device *dev,
				     uint32_t ctrl_command,
				     enum pm_device_state *state, pm_device_cb cb, void *arg)
{
	return 0;
}
#endif /* CONFIG_PM_DEVICE */


#define GPIO_DEVICE_INIT(__node, __suffix, __base_addr, __port, __cenr, __bus) \
	static const struct gpio_gd32_config gpio_gd32_cfg_## __suffix = {     \
		.common = {						       \
			 .port_pin_mask = GPIO_PORT_PIN_MASK_FROM_NGPIOS(16U), \
		},							       \
		.base = (uint32_t *)__base_addr,			       \
		.port = __port,						       \
		.pclken = { .bus = __bus, .enr = __cenr }		       \
	};								       \
	static struct gpio_gd32_data gpio_gd32_data_## __suffix;	       \
	DEVICE_DT_DEFINE(__node,					       \
			    gpio_gd32_init,				       \
			    gpio_gd32_pm_device_ctrl,			       \
			    &gpio_gd32_data_## __suffix,		       \
			    &gpio_gd32_cfg_## __suffix,		               \
			    POST_KERNEL,				       \
			    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,	       \
			    &gpio_gd32_driver)

#define GPIO_DEVICE_INIT_GD32(__suffix, __SUFFIX)			    \
	GPIO_DEVICE_INIT(DT_NODELABEL(gpio##__suffix),	                    \
			 __suffix,					    \
			 DT_REG_ADDR(DT_NODELABEL(gpio##__suffix)),	    \
			 GD32_PORT##__SUFFIX,				    \
			 DT_CLOCKS_CELL(DT_NODELABEL(gpio##__suffix), bits),\
			 DT_CLOCKS_CELL(DT_NODELABEL(gpio##__suffix), bus))

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioa), okay)
GPIO_DEVICE_INIT_GD32(a, A);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpioa), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpiob), okay)
GPIO_DEVICE_INIT_GD32(b, B);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpiob), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioc), okay)
GPIO_DEVICE_INIT_GD32(c, C);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpioc), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpiod), okay)
GPIO_DEVICE_INIT_GD32(d, D);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpiod), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioe), okay)
GPIO_DEVICE_INIT_GD32(e, E);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpioe), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpiof), okay)
GPIO_DEVICE_INIT_GD32(f, F);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpiof), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpiog), okay)
GPIO_DEVICE_INIT_GD32(g, G);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpiog), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioh), okay)
GPIO_DEVICE_INIT_GD32(h, H);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpioh), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioi), okay)
GPIO_DEVICE_INIT_GD32(i, I);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpioi), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioj), okay)
GPIO_DEVICE_INIT_GD32(j, J);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpioj), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpiok), okay)
GPIO_DEVICE_INIT_GD32(k, K);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpiok), okay) */

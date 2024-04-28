/*
 * Copyright (c) 2020 Andreas Sandberg
 * Copyright (c) 2020 Grinn
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_LLCC68_COMMON_H_
#define ZEPHYR_DRIVERS_LLCC68_COMMON_H_

#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/device.h>

#include <sx126x/sx126x.h>
#include <sx126x-board.h>

#if DT_HAS_COMPAT_STATUS_OKAY(semtech_llcc68)
#define DT_DRV_COMPAT semtech_llcc68
#define LLCC68_DEVICE_ID SX1262
#else
#error No LLCC68 instance in device tree.
#endif

#define HAVE_GPIO_ANTENNA_ENABLE			\
	DT_INST_NODE_HAS_PROP(0, antenna_enable_gpios)
#define HAVE_GPIO_TX_ENABLE	DT_INST_NODE_HAS_PROP(0, tx_enable_gpios)
#define HAVE_GPIO_RX_ENABLE	DT_INST_NODE_HAS_PROP(0, rx_enable_gpios)

struct llcc68_config {
    struct spi_dt_spec bus;
#if HAVE_GPIO_ANTENNA_ENABLE
    struct gpio_dt_spec antenna_enable;
#endif
#if HAVE_GPIO_TX_ENABLE
    struct gpio_dt_spec tx_enable;
#endif
#if HAVE_GPIO_RX_ENABLE
    struct gpio_dt_spec rx_enable;
#endif
};

struct llcc68_data {
    struct gpio_callback dio1_irq_callback;
    struct k_work dio1_irq_work;
    DioIrqHandler *radio_dio_irq;
    RadioOperatingModes_t mode;
};

int __llcc68_configure_pin(const struct gpio_dt_spec *gpio, gpio_flags_t flags);

#define llcc68_configure_pin(_name, _flags)				\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(0, _name##_gpios),		\
		    (__llcc68_configure_pin(&dev_config._name, _flags)),\
		    (0))

int llcc68_lora_send(const struct device *dev, uint8_t *data,
		     uint32_t data_len);

int llcc68_lora_send_async(const struct device *dev, uint8_t *data,
			   uint32_t data_len, struct k_poll_signal *async);

int llcc68_lora_recv(const struct device *dev, uint8_t *data, uint8_t size,
		     k_timeout_t timeout, int16_t *rssi, int8_t *snr);

int llcc68_lora_recv_async(const struct device *dev, lora_recv_cb cb);

int llcc68_lora_config(const struct device *dev,
		       struct lora_modem_config *config);

int llcc68_lora_test_cw(const struct device *dev, uint32_t frequency,
			int8_t tx_power,
			uint16_t duration);

int llcc68_init(const struct device *dev);

void llcc68_reset(struct llcc68_data *dev_data);

bool llcc68_is_busy(struct llcc68_data *dev_data);

uint32_t llcc68_get_dio1_pin_state(struct llcc68_data *dev_data);

void llcc68_dio1_irq_enable(struct llcc68_data *dev_data);

void llcc68_dio1_irq_disable(struct llcc68_data *dev_data);

void llcc68_set_tx_params(int8_t power, RadioRampTimes_t ramp_time);

int llcc68_variant_init(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_LLCC68_COMMON_H_ */

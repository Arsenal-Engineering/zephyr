/*
 * Copyright (c) 2025 Gabriele Zampieri <opensource@arsenaling.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ADC_MAX2253X_H_
#define ZEPHYR_INCLUDE_DRIVERS_ADC_MAX2253X_H_

#include <zephyr/device.h>

enum max2253x_co_mode {
	MAX2253X_DIGITAL_INPUT_MODE = 0,
	MAX2253X_STATUS_INPUT_MODE,
};

enum max2253x_co_mode_source {
	MAX2253X_CO_SOURCE_ADC = 0, /* Unfiltered ADC data is used */
	MAX2253X_CO_SOURCE_FADC,    /* Filtered ADC data is used */
};

enum max2253x_co_interrupt {
	MAX2253X_CO_INTERRUPT_NONE = 0,
	MAX2253X_CO_INTERRUPT_RISING = BIT(0),
	MAX2253X_CO_INTERRUPT_FALLING = BIT(1),
};

/**
 * @brief Enable or disable the interrupt on the specified CO channel.
 *
 * @param dev ADC device handle.
 * @param channel Channel to configure, in range 0 to 3.
 * @param irq Interrupts to enable (bitwise OR of enum max2253x_co_interrupt).
 * @return 0 on success, negative error code on failure.
 */
int max2253x_co_enable(const struct device *dev, uint8_t channel, enum max2253x_co_interrupt irq);

/**
 * @brief Configure the specified CO channel mode and thresholds.
 *
 * @param dev ADC device handle.
 * @param channel Channel to configure, in range 0 to 3.
 * @param mode Mode to set.
 * @param souce Source of data for threshold comparison.
 * @param th_low Threshold low value in LSBs.
 * @param th_high Threshold high value in LSBs.
 * @return 0 on success, negative error code on failure.
 */
int max2253x_co_set_mode(const struct device *dev, uint8_t channel, enum max2253x_co_mode mode,
			 enum max2253x_co_mode_source source, uint16_t th_low, uint16_t th_high);

/**
 * @brief Enable or disable common thresholds for all CO channels. This results in all enabled
 * channels to use the channel 0 thresholds.
 *
 * @param dev Device handle.
 * @param enable Enable or disable common thresholds.
 * @return 0 on success, negative error code on failure.
 */
int max2253x_enable_common_thresholds(const struct device *dev, bool enable);

/**
 * @brief Clear the moving average filter for the specified channel.
 *
 * @param dev Device handle.
 * @param channel Channel to clear the filter for, in range 0 to 3.
 * @return 0 on success, negative error code on failure.
 */
int max2253x_clear_filter(const struct device *dev, uint8_t channel);

#endif

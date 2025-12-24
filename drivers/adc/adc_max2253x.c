/*
 * Copyright (c) 2025 Gabriele Zampieri <opensource@arsenaling.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/adc/max2253x.h>

#define DT_DRV_COMPAT maxim_max2253x

LOG_MODULE_REGISTER(ADC_MAX2253x, CONFIG_ADC_LOG_LEVEL);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#define MAX2253X_VREF_MV         1800
#define MAX2253X_CH_COUNT        4
#define MAX2253X_RESOLUTION_BITS 12

/* MAX2253x header fields */
#define MAX2253X_HDR_ADDRESS_MASK GENMASK(7, 2)
#define MAX2253X_HDR_WNR_MASK     BIT(1)
#define MAX2253X_HDR_BURST_MASK   BIT(0)

#define MAX2253X_WRITE    1
#define MAX2253X_READ     0
#define MAX2253X_BURST    1
#define MAX2253X_NO_BURST 0

#define MAX2253X_BUILD_HDR(addr, wnr, burst)                                                       \
	(FIELD_PREP(MAX2253X_HDR_ADDRESS_MASK, (addr)) |                                           \
	 FIELD_PREP(MAX2253X_HDR_WNR_MASK, (wnr)) | FIELD_PREP(MAX2253X_HDR_BURST_MASK, burst))

#define MAX2253X_HDR_WRITE(addr)      MAX2253X_BUILD_HDR(addr, MAX2253X_WRITE, MAX2253X_NO_BURST)
#define MAX2253X_HDR_READ(addr)       MAX2253X_BUILD_HDR(addr, MAX2253X_READ, MAX2253X_NO_BURST)
#define MAX2253X_HDR_READ_BURST(addr) MAX2253X_BUILD_HDR(addr, MAX2253X_READ, MAX2253X_BURST)

/* MAX2253x registers*/
#define MAX2253X_ADC1             0x01
#define MAX2253X_COUTHI1          0x09
#define MAX2253X_COUTLO1          0x0D
#define MAX2253X_INTERRUPT_STATUS 0x12
#define MAX2253X_INTERRUPT_ENABLE 0x13
#define MAX2253X_CONTROL          0x14

/* MAX2253X_ADCx Bit Definitions */
#define MAX2253X_ADCx_ADCS BIT(15)
#define MAX2253X_ADCx_ADC  GENMASK(11, 0)

/* MAX2253X_COUTHIx Bit Definitions */
#define MAX2253X_COUTHI_CO_MODE   BIT(15)
#define MAX2253X_COUTHI_CO_IN_SEL BIT(14)
#define MAX2253X_COUTHI_COTHI     GENMASK(11, 0)

/* MAX2253X_COUTLOx Bit Definitions */
#define MAX2253X_COUTLO_COTLO GENMASK(11, 0)

/* MAX2253X_INTERRUPT_STATUS Bit Definitions */
#define MAX2253X_INTERRUPT_STATUS_EOC BIT(12)

/* MAX2253X_INTERRUPT_ENABLE Bit Definitions */
#define MAX2253X_INTERRUPT_ENABLE_EEOC      BIT(12)
#define MAX2253X_INTERRUPT_ENABLE_ECO_POS_1 BIT(4)
#define MAX2253X_INTERRUPT_ENABLE_ECO_NEG_1 BIT(0)

/* MAX2253X_CONTROL Bit Definitions */
#define MAX2253X_CONTROL_ECOM      BIT(14) /* Enable common thresholds */
#define MAX2253X_CONTROL_FLT_CLR_1 BIT(4)  /* Clear ADC1 Moving avg filter */
#define MAX2253X_CONTROL_SRES      BIT(1)  /* Soft reset */

/* Per-channel helpers */
#define MAX2253X_COUTHI(ch)         (MAX2253X_COUTHI1 + (ch))
#define MAX2253X_COUTLO(ch)         (MAX2253X_COUTLO1 + (ch))
#define MAX2253X_INT_EN_ECO_POS(ch) (MAX2253X_INTERRUPT_ENABLE_ECO_POS_1 << (ch))
#define MAX2253X_INT_EN_ECO_NEG(ch) (MAX2253X_INTERRUPT_ENABLE_ECO_NEG_1 << (ch))
#define MAX2253X_FLT_CLR(ch)        (MAX2253X_CONTROL_FLT_CLR_1 << (ch))

struct max2253x_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec int_gpio;

	const struct adc_dt_spec spec;
};

struct max2253x_data {
	struct adc_context ctx;
	const struct device *dev;
	struct gpio_callback int_callback;

	uint32_t channels;
	uint16_t *buffer;
	uint16_t *repeat_buffer;

	struct k_sem sem;
	struct k_thread thread;
	K_KERNEL_STACK_MEMBER(stack, CONFIG_ADC_MAX2253X_ACQUISITION_THREAD_STACK_SIZE);
};

static int max2253x_read_burst(const struct device *dev, uint8_t address, uint8_t *buffer,
			       size_t count)
{
	const struct max2253x_config *config = dev->config;
	uint8_t hdr = MAX2253X_HDR_READ_BURST(address);
	const struct spi_buf hdr_buf[1] = {{.buf = &hdr, .len = 1}};
	const struct spi_buf_set tx = {.buffers = hdr_buf, .count = ARRAY_SIZE(hdr_buf)};
	const struct spi_buf rx_buf[2] = {
		{.buf = NULL, .len = 1}, /* Dummy byte for header */
		{.buf = buffer, .len = count},
	};
	const struct spi_buf_set rx = {.buffers = rx_buf, .count = ARRAY_SIZE(rx_buf)};

	return spi_transceive_dt(&config->spi, &tx, &rx);
}

static int max2253x_read_reg(const struct device *dev, uint8_t reg, uint16_t *value)
{
	uint8_t value_buf[2];
	int r;

	r = max2253x_read_burst(dev, reg, value_buf, sizeof(value_buf));
	if (r) {
		return r;
	}

	*value = sys_get_be16(value_buf);
	return 0;
}

static int max2253x_write_reg(const struct device *dev, uint8_t reg, uint16_t value)
{
	const struct max2253x_config *config = dev->config;
	uint8_t hdr = MAX2253X_HDR_WRITE(reg);
	uint8_t value_buf[2];
	const struct spi_buf tx_buf[2] = {
		{.buf = &hdr, .len = 1},
		{.buf = value_buf, .len = 2},
	};
	const struct spi_buf_set tx = {.buffers = tx_buf, .count = ARRAY_SIZE(tx_buf)};

	sys_put_be16(value, value_buf);

	return spi_transceive_dt(&config->spi, &tx, NULL);
}

static int max2253x_update_reg(const struct device *dev, uint8_t reg, uint16_t mask, uint16_t field)
{
	int ret;
	uint16_t reg_val;

	ret = max2253x_read_reg(dev, reg, &reg_val);
	if (ret < 0) {
		return ret;
	}

	reg_val = (reg_val & ~mask) | field;
	return max2253x_write_reg(dev, reg, reg_val);
}

static int max2253x_read_raw_adc(const struct device *dev, uint16_t *buffer, size_t count)
{
	uint8_t raw_buffer[MAX2253X_CH_COUNT * 2];
	int ret;

	ret = max2253x_read_burst(dev, MAX2253X_ADC1, raw_buffer, sizeof(raw_buffer));
	if (ret) {
		return ret;
	}

	for (int i = 0; i < MAX2253X_CH_COUNT; i++) {
		buffer[i] = FIELD_GET(MAX2253X_ADCx_ADC, sys_get_be16(&raw_buffer[i * 2]));
	}

	return 0;
}

int max2253x_co_enable(const struct device *dev, uint8_t channel, enum max2253x_co_interrupt irq)
{
	if (channel >= MAX2253X_CH_COUNT) {
		return -EINVAL;
	}

	uint16_t mask = MAX2253X_INT_EN_ECO_POS(channel) | MAX2253X_INT_EN_ECO_NEG(channel);
	uint16_t field = FIELD_PREP(MAX2253X_INT_EN_ECO_POS(channel),
				    !!(irq & MAX2253X_CO_INTERRUPT_RISING));
	field |= FIELD_PREP(MAX2253X_INT_EN_ECO_NEG(channel),
			    !!(irq & MAX2253X_CO_INTERRUPT_FALLING));
	return max2253x_update_reg(dev, MAX2253X_INTERRUPT_ENABLE, mask, field);
}

int max2253x_co_set_mode(const struct device *dev, uint8_t channel, enum max2253x_co_mode mode,
			 enum max2253x_co_mode_source source, uint16_t th_low, uint16_t th_high)
{
	int ret;

	if (channel >= MAX2253X_CH_COUNT) {
		return -EINVAL;
	}

	ret = max2253x_write_reg(dev, MAX2253X_COUTHI(channel),
				 FIELD_PREP(MAX2253X_COUTHI_CO_MODE, mode) |
					 FIELD_PREP(MAX2253X_COUTHI_CO_IN_SEL, source) |
					 FIELD_PREP(MAX2253X_COUTHI_COTHI, th_high));

	if (ret) {
		return ret;
	}

	return max2253x_write_reg(dev, MAX2253X_COUTLO(channel),
				  FIELD_PREP(MAX2253X_COUTLO_COTLO, th_low));
}

int max2253x_enable_common_thresholds(const struct device *dev, bool enable)
{
	return max2253x_update_reg(dev, MAX2253X_CONTROL, MAX2253X_CONTROL_ECOM,
				   FIELD_PREP(MAX2253X_CONTROL_ECOM, !!enable));
}

int max2253x_clear_filter(const struct device *dev, uint8_t channel)
{
	if (channel >= MAX2253X_CH_COUNT) {
		return -EINVAL;
	}

	return max2253x_update_reg(dev, MAX2253X_CONTROL, MAX2253X_FLT_CLR(channel),
				   FIELD_PREP(MAX2253X_FLT_CLR(channel), 0));
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct max2253x_data *data = CONTAINER_OF(ctx, struct max2253x_data, ctx);

	data->channels = ctx->sequence.channels;
	data->repeat_buffer = data->buffer;

	k_sem_give(&data->sem);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct max2253x_data *data = CONTAINER_OF(ctx, struct max2253x_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static int adc_max2253x_channel_setup(const struct device *dev,
				      const struct adc_channel_cfg *channel_cfg)
{
	if (channel_cfg->channel_id > (MAX2253X_CH_COUNT - 1)) {
		LOG_ERR("Invalid channel %d", channel_cfg->channel_id);
		return -EINVAL;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("only x1 gain is supported");
		return -EINVAL;
	}

	if (channel_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("only internal reference is supported");
		return -EINVAL;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("only default acquisition time is supported");
		return -EINVAL;
	}

	if (channel_cfg->differential) {
		LOG_ERR("differential channel not supported");
		return -EINVAL;
	}

	return 0;
}

static int adc_max2253x_validate_buffer_size(const struct device *dev,
					     const struct adc_sequence *sequence)
{
	uint8_t channels;
	size_t needed;

	channels = POPCOUNT(sequence->channels);
	needed = channels * sizeof(uint16_t);

	if (sequence->options) {
		needed *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed) {
		return -ENOMEM;
	}

	return 0;
}

static int adc_max2253x_start_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct max2253x_data *data = dev->data;
	int ret;

	if (sequence->resolution != MAX2253X_RESOLUTION_BITS) {
		LOG_ERR("invalid resolution %d", sequence->resolution);
		return -EINVAL;
	}

	ret = adc_max2253x_validate_buffer_size(dev, sequence);
	if (ret < 0) {
		LOG_ERR("insufficient buffer size");
		return ret;
	}

	data->buffer = sequence->buffer;
	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

static int adc_max2253x_read_async(const struct device *dev, const struct adc_sequence *sequence,
				   struct k_poll_signal *async)
{
	struct max2253x_data *data = dev->data;
	int ret;

	adc_context_lock(&data->ctx, async ? true : false, async);
	ret = adc_max2253x_start_read(dev, sequence);
	adc_context_release(&data->ctx, ret);

	return ret;
}

static int adc_max2253x_read(const struct device *dev, const struct adc_sequence *sequence)
{
	return adc_max2253x_read_async(dev, sequence, NULL);
}

static DEVICE_API(adc, max2253x_api) = {
	.channel_setup = adc_max2253x_channel_setup,
	.read = adc_max2253x_read,
	.ref_internal = MAX2253X_VREF_MV,
#ifdef CONFIG_ADC_ASYNC
	.read_async = adc_max2253x_read_async,
#endif /* CONFIG_ADC_ASYNC */
};

#if CONFIG_MAX2253X_TRIGGER
static void max2253x_int_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct max2253x_data *data = dev->data;

	k_sem_give(&data->sem);
}

int max2253x_init_interrupt(const struct device *dev)
{
	const struct max2253x_config *cfg = dev->config;
	struct max2253x_data *data = dev->data;
	int ret;

	if (!gpio_is_ready_dt(&cfg->int_gpio)) {
		LOG_ERR("GPIO port %s not ready", cfg->int_gpio.port->name);
		return -EINVAL;
	}

	ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT | GPIO_PULL_UP);
	if (ret) {
		return ret;
	}

	gpio_init_callback(&data->int_callback, max2253x_int_callback, BIT(cfg->int_gpio.pin));

	ret = gpio_add_callback_dt(&cfg->int_gpio, &data->int_callback);
	if (ret) {
		LOG_ERR("Failed to set gpio callback!");
		return ret;
	}

#warning enable irq

	return gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}
#endif

static void max2253x_acquisition_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct max2253x_data *data = p1;
	uint16_t adc_raw[MAX2253X_CH_COUNT];
	int ret;

	while (true) {
		k_sem_take(&data->sem, K_FOREVER);

		ret = max2253x_read_raw_adc(data->dev, adc_raw, sizeof(adc_raw));
		if (ret) {
			LOG_ERR("Failed to read raw samples (err %d)", ret);
			adc_context_complete(&data->ctx, ret);
			break;
		}

		for (int i = 0; i < MAX2253X_CH_COUNT; i++) {
			if (data->channels & BIT(i)) {
				*data->buffer++ = adc_raw[i];
			}
		}

		adc_context_on_sampling_done(&data->ctx, data->dev);
	}
}

static int max2253x_init(const struct device *dev)
{
	const struct max2253x_config *cfg = dev->config;
	struct max2253x_data *data = dev->data;
	int ret;

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI not ready");
		return -ENODEV;
	}

	adc_context_init(&data->ctx);

	k_sem_init(&data->sem, 0, 1);

	k_tid_t tid =
		k_thread_create(&data->thread, data->stack, K_KERNEL_STACK_SIZEOF(data->stack),
				max2253x_acquisition_thread, data, NULL, NULL,
				CONFIG_ADC_MAX2253X_ACQUISITION_THREAD_PRIORITY, 0, K_NO_WAIT);

	k_thread_name_set(tid, "adc_max2253x");

	/* Reset */
	ret = max2253x_update_reg(dev, MAX2253X_CONTROL, MAX2253X_CONTROL_SRES,
				  FIELD_PREP(MAX2253X_CONTROL_SRES, 1));
	if (ret < 0) {
		LOG_ERR("Failed to reset device %s", dev->name);
		return ret;
	}

#if CONFIG_MAX2253X_TRIGGER
	ret = max2253x_init_interrupt(dev);
	if (ret != 0) {
		LOG_ERR("Failed to initialize interrupt!");
		return -EIO;
	}
#endif

	adc_context_unlock_unconditionally(&data->ctx);
	data->dev = dev;
	return ret;
}

#define MAX2253X_INIT(inst)                                                                        \
	static const struct max2253x_config max2253x_config_##inst = {                             \
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |          \
							  SPI_WORD_SET(8)),                        \
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),                        \
	};                                                                                         \
                                                                                                   \
	static struct max2253x_data max2253x_data_##inst = {};                                     \
	DEVICE_DT_INST_DEFINE(inst, max2253x_init, NULL, &max2253x_data_##inst,                    \
			      &max2253x_config_##inst, POST_KERNEL,                                \
			      CONFIG_ADC_MAX2253X_INIT_PRIORITY, &max2253x_api);

DT_INST_FOREACH_STATUS_OKAY(MAX2253X_INIT)

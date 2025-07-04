/* 
 * Azoteq TPS43 capacitive track-pad – ZMK edition
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tps43, LOG_LEVEL_INF);

/* -------------------------------------------------------------------------- */

#define DT_DRV_COMPAT azoteq_tps43

/* TPS43 register map (subset) */
#define REG_X_LSB         0x30
#define REG_GESTURE_EVENT 0x3E

/* -------------------------------------------------------------------------- */
/* Build-time constants pulled from DTS                                       */

struct tps43_config {
	struct i2c_dt_spec bus;
	struct gpio_dt_spec int_gpio;
	struct gpio_dt_spec rst_gpio;
};

/* -------------------------------------------------------------------------- */
/* Runtime state                                                              */

struct tps43_data {
	const struct device *dev;      /* back-pointer for input_report_*     */
	struct gpio_callback int_cb;
	struct k_work work;
};

/* -------------------------------------------------------------------------- */

static void tps43_process(struct k_work *work)
{
	struct tps43_data    *data = CONTAINER_OF(work, struct tps43_data, work);
	const struct tps43_config *cfg  = data->dev->config;

	uint8_t buf[4];

	/* 12-bit X / Y, little-endian */
	if (i2c_reg_read_buf_dt(&cfg->bus, REG_X_LSB, buf, sizeof(buf))) {
		LOG_DBG("I²C read failed");
		return;
	}

	uint16_t x = buf[0] | (buf[1] << 8);
	uint16_t y = buf[2] | (buf[3] << 8);

	input_report_abs(data->dev, INPUT_ABS_X, x, false, K_NO_WAIT);
	input_report_abs(data->dev, INPUT_ABS_Y, y, false, K_NO_WAIT);
	input_report_key(data->dev, INPUT_BTN_TOUCH, 1, true, K_NO_WAIT);
	input_sync(data->dev);
}

static void tps43_isr(const struct device *port,
		      struct gpio_callback *cb,
		      uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	struct tps43_data *data = CONTAINER_OF(cb, struct tps43_data, int_cb);
	k_work_submit(&data->work);
}

/* -------------------------------------------------------------------------- */

static int tps43_init(const struct device *dev)
{
	const struct tps43_config *cfg = dev->config;
	struct tps43_data        *data = dev->data;
	int ret;

	data->dev = dev;
	k_work_init(&data->work, tps43_process);

	/* --- Reset pin ------------------------------------------------------- */
	ret = gpio_pin_configure_dt(&cfg->rst_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		LOG_ERR("RST GPIO config failed (%d)", ret);
		return ret;
	}
	/* 10 ms low-pulse hardware reset */
	gpio_pin_set_dt(&cfg->rst_gpio, 0);
	k_sleep(K_MSEC(1));
	gpio_pin_set_dt(&cfg->rst_gpio, 1);
	k_sleep(K_MSEC(10));

	/* --- INT pin --------------------------------------------------------- */
	ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
	if (ret) {
		LOG_ERR("INT GPIO config failed (%d)", ret);
		return ret;
	}

	gpio_init_callback(&data->int_cb, tps43_isr,
			   BIT(cfg->int_gpio.pin));
	gpio_add_callback(cfg->int_gpio.port, &data->int_cb);
	gpio_pin_interrupt_configure_dt(&cfg->int_gpio,
					GPIO_INT_EDGE_TO_ACTIVE);

	LOG_INF("TPS43 initialised");
	return 0;
}

/* Zephyr input driver shim (we only need the generic helper table) */
static const struct input_driver_api tps43_api = { 0 };

/* -------------------------------------------------------------------------- */
/* One instance per DT node with status = "okay"                              */

#define TPS43_INST(idx)                                                         \
	static struct tps43_data   tps43_data_##idx;                            \
	static const struct tps43_config tps43_config_##idx = {                 \
		.bus       = I2C_DT_SPEC_INST_GET(idx),                         \
		.int_gpio  = GPIO_DT_SPEC_INST_GET(idx, int_gpios),             \
		.rst_gpio  = GPIO_DT_SPEC_INST_GET(idx, reset_gpios),           \
	};                                                                       \
	DEVICE_DT_INST_DEFINE(idx,                                                \
			      tps43_init, NULL,                                 \
			      &tps43_data_##idx, &tps43_config_##idx,           \
			      POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY,         \
			      &tps43_api);

DT_INST_FOREACH_STATUS_OKAY(TPS43_INST)


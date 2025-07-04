/*
 * Azoteq TPS43 capacitive track-pad — ZMK edition
 *
 * Works on both Zephyr stand-alone and ZMK because the public
 * Zephyr input API is unchanged. The only tweak is to keep the
 * device pointer when reporting events so the ZMK input-listener
 * can route them correctly.
 */

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tps43, CONFIG_INPUT_LOG_LEVEL);

#define DT_DRV_COMPAT azoteq_tps43
#define TPS43_I2C_ADDR DT_INST_REG_ADDR(0)

#define REG_SYS_INFO      0x00
#define REG_X_LSB         0x30
#define REG_GESTURE_EVENT 0x3E
#define REG_SYS_CTRL      0x56

struct tps43_data {
	const struct device *dev;     /* <-- keep for input_report_* */
	const struct device *i2c;
	const struct gpio_dt_spec int_gpio;
	const struct gpio_dt_spec rst_gpio;
	struct gpio_callback int_cb;
	struct k_work work;
};

static void tps43_process(struct k_work *w)
{
	struct tps43_data *data = CONTAINER_OF(w, struct tps43_data, work);
	uint8_t buf[6];

	if (i2c_burst_read_dt(&data->i2c->config,
			      TPS43_I2C_ADDR, REG_X_LSB, buf, sizeof(buf))) {
		LOG_DBG("I2C read failed");
		return;
	}

	uint16_t x = buf[0] | (buf[1] << 8);
	uint16_t y = buf[2] | (buf[3] << 8);

	input_report_abs(data->dev, INPUT_ABS_X, x, false, K_NO_WAIT);
	input_report_abs(data->dev, INPUT_ABS_Y, y, false, K_NO_WAIT);
	input_report_key(data->dev, INPUT_BTN_TOUCH, 1, true,  K_NO_WAIT);
	input_sync(data->dev);
}

static void tps43_isr(const struct device *port,
		      struct gpio_callback *cb,
		      uint32_t pins)
{
	struct tps43_data *data = CONTAINER_OF(cb, struct tps43_data, int_cb);
	k_work_submit(&data->work);
}

static int tps43_init(const struct device *dev)
{
	struct tps43_data *data = dev->data;
	int ret;

	data->dev = dev; /* save self */

	/* Hardware reset */
	gpio_pin_configure_dt(&data->rst_gpio, GPIO_OUTPUT_ACTIVE);
	k_sleep(K_MSEC(1));
	gpio_pin_set_dt(&data->rst_gpio, 1);
	k_sleep(K_MSEC(10));

	/* INT line */
	ret = gpio_pin_configure_dt(&data->int_gpio, GPIO_INPUT);
	if (ret) {
		LOG_ERR("INT config failed");
		return ret;
	}
	gpio_init_callback(&data->int_cb, tps43_isr,
			   BIT(data->int_gpio.pin));
	gpio_add_callback(data->int_gpio.port, &data->int_cb);
	gpio_pin_interrupt_configure_dt(&data->int_gpio,
					GPIO_INT_EDGE_TO_ACTIVE);

	k_work_init(&data->work, tps43_process);

	LOG_INF("TPS43 ready");
	return 0;
}

static const struct input_driver_api tps43_api = { 0 };

#define TPS43_INST(idx)                                                         \
	static struct tps43_data tps43_data_##idx;                              \
	DEVICE_DT_INST_DEFINE(idx,                                              \
			      tps43_init, NULL,                                 \
			      &tps43_data_##idx, NULL,                          \
			      POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY,         \
			      &tps43_api);

DT_INST_FOREACH_STATUS_OKAY(TPS43_INST)

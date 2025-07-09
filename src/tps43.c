// tps43.cpp – Azoteq TPS43 touch‑pad driver (C++17 + Zephyr 3.6 / ZMK)
// SPDX‑License‑Identifier: Apache‑2.0

// ──────────────────────────────────────────────────────────────────────────────
// NOTE ❶: this file is meant to be compiled as **C++** inside a ZMK module.
//         Add to your CMakeLists.txt with `zephyr_library_sources(tps43.cpp)`.
//
// NOTE ❷: set these Kconfig options so the Zephyr build picks up C++:
//         CONFIG_CPLUSPLUS=y
//         CONFIG_LIB_CPLUSPLUS=y   # pulls in the minimal libstdc++ stub
// ──────────────────────────────────────────────────────────────────────────────

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/input/input.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tps43, LOG_LEVEL_INF);

// ──────────────────────────────────────────────────────────────────────────────
// Device‑tree interface
// ──────────────────────────────────────────────────────────────────────────────
#define DT_DRV_COMPAT azoteq_tps43

/* TPS43 register map (subset) */
static constexpr uint8_t REG_X_LSB         = 0x30;
static constexpr uint8_t REG_GESTURE_EVENT = 0x3E;

// ──────────────────────────────────────────────────────────────────────────────
// Build‑time configuration that lives in flash/rodata
// ──────────────────────────────────────────────────────────────────────────────
struct tps43_config {
    i2c_dt_spec bus;          // I2C peripheral + address
    gpio_dt_spec int_gpio;    // RDY/INT (active‑low)
    gpio_dt_spec rst_gpio;    // Hardware reset (active‑low)
};

// ──────────────────────────────────────────────────────────────────────────────
// Runtime state (RAM)
// ──────────────────────────────────────────────────────────────────────────────
struct tps43_data {
    const device *dev{};      // back‑pointer – needed by input API
    gpio_callback int_cb{};   // ISR hook structure
    k_work work{};            // deferred worker
};

// ──────────────────────────────────────────────────────────────────────────────
// Helper: read XY and push to ZMK
// ──────────────────────────────────────────────────────────────────────────────
static void tps43_process(k_work *work)
{
    auto *data = CONTAINER_OF(work, tps43_data, work);
    auto *cfg  = static_cast<const tps43_config *>(data->dev->config);

    uint8_t buf[4];  // X LSB/MSB, Y LSB/MSB

    if (i2c_reg_read_buf_dt(&cfg->bus, REG_X_LSB, buf, sizeof(buf)) < 0) {
        LOG_DBG("I²C read failed");
        return;
    }

    uint16_t x = static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);
    uint16_t y = static_cast<uint16_t>(buf[2]) | (static_cast<uint16_t>(buf[3]) << 8);

    input_report_abs(data->dev, INPUT_ABS_X, x, false, K_NO_WAIT);
    input_report_abs(data->dev, INPUT_ABS_Y, y, false, K_NO_WAIT);
    input_report_key(data->dev, INPUT_BTN_TOUCH, 1, true, K_NO_WAIT);
    input_sync(data->dev);
}

// ──────────────────────────────────────────────────────────────────────────────
// GPIO interrupt shim: schedule the worker
// ──────────────────────────────────────────────────────────────────────────────
static void tps43_isr(const device * /*port*/, gpio_callback *cb, uint32_t /*pins*/)
{
    auto *data = CONTAINER_OF(cb, tps43_data, int_cb);
    k_work_submit(&data->work);
}

// ──────────────────────────────────────────────────────────────────────────────
// Driver initialisation
// ──────────────────────────────────────────────────────────────────────────────
static int tps43_init(const device *dev)
{
    auto *cfg  = static_cast<const tps43_config *>(dev->config);
    auto *data = static_cast<tps43_data *>(dev->data);

    data->dev = dev;
    k_work_init(&data->work, tps43_process);

    /* Reset sequence – pull RST low 1 ms, then high, wait 10 ms */
    if (gpio_pin_configure_dt(&cfg->rst_gpio, GPIO_OUTPUT_INACTIVE) < 0) {
        LOG_ERR("RST GPIO config failed");
        return -EINVAL;
    }
    k_sleep(K_MSEC(1));
    gpio_pin_set_dt(&cfg->rst_gpio, 1);
    k_sleep(K_MSEC(10));

    /* INT line */
    if (gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT) < 0) {
        LOG_ERR("INT GPIO config failed");
        return -EINVAL;
    }
    gpio_init_callback(&data->int_cb, tps43_isr, BIT(cfg->int_gpio.pin));
    gpio_add_callback(cfg->int_gpio.port, &data->int_cb);
    gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);

    LOG_INF("TPS43 ready – driver initialised");
    return 0;
}

// ──────────────────────────────────────────────────────────────────────────────
// Zephyr boiler‑plate to create each instance listed in the devicetree
// ──────────────────────────────────────────────────────────────────────────────
static constexpr input_driver_api tps43_api{}; // empty – we only need the struct

#define TPS43_INST(idx)                                                                  \
    static tps43_data   tps43_data_##idx{};                                              \
    static constexpr tps43_config tps43_config_##idx = {                                 \
        I2C_DT_SPEC_INST_GET(idx),       /* bus */                                       \
        GPIO_DT_SPEC_INST_GET(idx, int_gpios),                                           \
        GPIO_DT_SPEC_INST_GET(idx, reset_gpios)                                          \
    };                                                                                   \
    DEVICE_DT_INST_DEFINE(idx,                                                           \
                          tps43_init,                                                    \
                          nullptr,            /* PM control fn – none */                 \
                          &tps43_data_##idx,                                            \
                          &tps43_config_##idx,                                          \
                          POST_KERNEL,                                                   \
                          CONFIG_INPUT_INIT_PRIORITY,                                    \
                          &tps43_api);

DT_INST_FOREACH_STATUS_OKAY(TPS43_INST)


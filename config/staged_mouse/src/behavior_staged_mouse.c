/*
 * SPDX-License-Identifier: MIT
 *
 * Precision movement: 5 pixels on press, then 5-pixel steps on a held key
 * before a smooth acceleration to cruising speed. Braking is optional.
 */
#define DT_DRV_COMPAT adv360_behavior_staged_mouse

#include <errno.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/sys/util.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define UPDATE_MS 4
#define PRECISION_REPEAT_MS 90
#define BRAKE_SPEED_CAP 200

struct staged_config {
    int32_t x_speed;
    int32_t y_speed;
    uint16_t crawl_speed;
    uint16_t crawl_ms;
    uint16_t ramp_ms;
    uint16_t brake_ms;
    uint8_t nudge_px;
};

struct staged_data {
    struct k_work_delayable tick;
    const struct device *dev;
    int64_t start_ms;
    int64_t release_ms;
    int32_t remainder;
    int64_t last_tick_ms;
    uint16_t precision_steps;
    int32_t brake_speed;
    bool down;
    bool braking;
};

static struct staged_data *all_states[DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT)];

static int32_t abs_speed(int32_t speed) { return speed < 0 ? -speed : speed; }
static int32_t sign_speed(int32_t speed) { return (speed > 0) - (speed < 0); }

static void send_delta(const struct device *dev, int32_t pixels) {
    const struct staged_config *cfg = dev->config;
    int32_t axis = cfg->x_speed ? cfg->x_speed : cfg->y_speed;
    int code = cfg->x_speed ? INPUT_REL_X : INPUT_REL_Y;
    if (pixels != 0 && axis != 0) {
        input_report_rel(dev, code, pixels * sign_speed(axis), true, K_NO_WAIT);
    }
}

static int32_t staged_speed(const struct staged_config *cfg, int64_t held_ms) {
    int32_t max = abs_speed(cfg->x_speed ? cfg->x_speed : cfg->y_speed);
    // Held precision steps are handled by staged_tick before the ramp.
    if (held_ms < cfg->crawl_ms) {
        return 0;
    }
    int64_t t = held_ms - cfg->crawl_ms;
    if (cfg->ramp_ms == 0 || t >= cfg->ramp_ms) {
        return max;
    }
    // Smoothstep acceleration avoids a sudden jump in speed at either end.
    int64_t ramp = cfg->ramp_ms;
    int64_t blend = t * t * (3 * ramp - 2 * t);
    return cfg->crawl_speed + (int32_t)(((int64_t)(max - cfg->crawl_speed) * blend) /
                                         (ramp * ramp * ramp));
}

static void staged_tick(struct k_work *work) {
    struct staged_data *data =
        CONTAINER_OF(k_work_delayable_from_work(work), struct staged_data, tick);
    const struct staged_config *cfg = data->dev->config;
    int64_t now = k_uptime_get();
    int32_t speed;
    int64_t elapsed_ms = now - data->last_tick_ms;
    data->last_tick_ms = now;
    // Bound catch-up movement if the scheduler was delayed.
    elapsed_ms = CLAMP(elapsed_ms, 0, 32);

    if (data->down && now - data->start_ms < cfg->crawl_ms) {
        uint16_t steps_due = (uint16_t)((now - data->start_ms) / PRECISION_REPEAT_MS);
        while (data->precision_steps <= steps_due) {
            send_delta(data->dev, cfg->nudge_px);
            data->precision_steps++;
        }
        k_work_reschedule(&data->tick, K_MSEC(UPDATE_MS));
        return;
    }

    if (data->down) {
        speed = staged_speed(cfg, now - data->start_ms);
    } else if (data->braking) {
        int64_t elapsed = now - data->release_ms;
        if (elapsed >= cfg->brake_ms || cfg->brake_ms == 0) {
            data->braking = false;
            data->remainder = 0;
            return;
        }
        speed = (int32_t)(data->brake_speed * (cfg->brake_ms - elapsed) / cfg->brake_ms);
    } else {
        return;
    }

    data->remainder += speed * (int32_t)elapsed_ms;
    int32_t pixels = data->remainder / 1000;
    data->remainder -= pixels * 1000;
    send_delta(data->dev, pixels);
    k_work_reschedule(&data->tick, K_MSEC(UPDATE_MS));
}

static int staged_pressed(struct zmk_behavior_binding *binding,
                          struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct staged_data *data = dev->data;
    const struct staged_config *cfg = dev->config;
    if (data->down) {
        return -EBUSY;
    }

    for (size_t i = 0; i < ARRAY_SIZE(all_states); i++) {
        struct staged_data *state = all_states[i];
        if (state && state->braking) {
            state->braking = false;
            state->remainder = 0;
            k_work_cancel_delayable(&state->tick);
        }
    }

    k_work_cancel_delayable(&data->tick);
    data->down = true;
    data->braking = false;
    data->start_ms = k_uptime_get();
    data->last_tick_ms = data->start_ms;
    data->precision_steps = 1;
    data->remainder = 0;
    send_delta(dev, cfg->nudge_px);
    k_work_reschedule(&data->tick, K_MSEC(UPDATE_MS));
    return 0;
}

static int staged_released(struct zmk_behavior_binding *binding,
                           struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct staged_data *data = dev->data;
    const struct staged_config *cfg = dev->config;

    if (!data->down) {
        return -EINVAL;
    }
    int64_t now = k_uptime_get();
    int64_t held_ms = now - data->start_ms;
    data->down = false;
    data->remainder = 0;
    k_work_cancel_delayable(&data->tick);

    /* Never add inertia to a precision tap or slow/accelerating movement. */
    if (cfg->brake_ms && held_ms >= cfg->crawl_ms + cfg->ramp_ms) {
        data->braking = true;
        data->release_ms = now;
        data->brake_speed = MIN(staged_speed(cfg, held_ms) / 4, BRAKE_SPEED_CAP);
        k_work_reschedule(&data->tick, K_MSEC(UPDATE_MS));
    }
    return 0;
}

static const struct behavior_driver_api staged_api = {
    .binding_pressed = staged_pressed,
    .binding_released = staged_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif
};

static int staged_init(const struct device *dev) {
    struct staged_data *data = dev->data;
    data->dev = dev;
    k_work_init_delayable(&data->tick, staged_tick);
    return 0;
}

#define STAGED_INSTANCE(n)                                                       \
    static const struct staged_config staged_cfg_##n = {                        \
        .x_speed = DT_INST_PROP(n, x_speed),                                     \
        .y_speed = DT_INST_PROP(n, y_speed),                                     \
        .crawl_speed = DT_INST_PROP(n, crawl_speed),                             \
        .crawl_ms = DT_INST_PROP(n, crawl_ms),                                   \
        .ramp_ms = DT_INST_PROP(n, ramp_ms),                                     \
        .brake_ms = DT_INST_PROP(n, brake_ms),                                   \
        .nudge_px = DT_INST_PROP(n, nudge_px),                                   \
    };                                                                           \
    static struct staged_data staged_data_##n;                                  \
    static int staged_init_##n(const struct device *dev) {                      \
        all_states[n] = &staged_data_##n;                                        \
        return staged_init(dev);                                                \
    }                                                                            \
    BEHAVIOR_DT_INST_DEFINE(n, staged_init_##n, NULL, &staged_data_##n,          \
                            &staged_cfg_##n, POST_KERNEL,                        \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &staged_api);

DT_INST_FOREACH_STATUS_OKAY(STAGED_INSTANCE)
#endif

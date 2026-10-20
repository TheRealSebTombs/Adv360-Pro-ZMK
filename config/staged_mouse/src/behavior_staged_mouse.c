/*
 * SPDX-License-Identifier: MIT
 *
 * Immediate one-pixel nudge, followed by continuous, smoothly accelerating
 * movement. A shared timer combines X/Y into synchronised reports.
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
    const struct device *dev;
    int64_t start_ms;
    int64_t release_ms;
    int32_t remainder;
    int64_t last_tick_ms;
    int32_t brake_speed;
    bool down;
    bool braking;
};

static struct staged_data *all_states[DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT)];
static struct k_work_delayable shared_tick;
static bool shared_tick_ready;
static bool precision_held;
#define PRECISION_PX_PER_SECOND 100

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
    // Precision is constant-speed; it does not restart the normal ramp.
    if (precision_held) {
        return PRECISION_PX_PER_SECOND;
    }
    int32_t max = abs_speed(cfg->x_speed ? cfg->x_speed : cfg->y_speed);
    int32_t min = MIN((int32_t)cfg->crawl_speed, max);
    // Use one continuous S-shaped ramp, with no separate step-repeat phase.
    int64_t ramp = (int64_t)cfg->crawl_ms + cfg->ramp_ms;
    if (ramp == 0 || held_ms >= ramp) {
        return max;
    }
    int64_t t = MAX(held_ms, 0);
    int64_t blend = t * t * (3 * ramp - 2 * t);
    return min + (int32_t)(((int64_t)(max - min) * blend) /
                           (ramp * ramp * ramp));
}

static void staged_tick(struct k_work *work) {
    int64_t now = k_uptime_get();
    int32_t dx = 0, dy = 0;
    bool active = false;
    bool held_x = false, held_y = false;
    ARG_UNUSED(work);

    // Identify simultaneous X/Y movement before calculating individual deltas.
    for (size_t i = 0; i < ARRAY_SIZE(all_states); i++) {
        struct staged_data *state = all_states[i];
        if (!state || !state->down) {
            continue;
        }
        const struct staged_config *direction = state->dev->config;
        held_x |= direction->x_speed != 0;
        held_y |= direction->y_speed != 0;
    }

    // Compute all held directions on one clock and coalesce the axes.
    for (size_t i = 0; i < ARRAY_SIZE(all_states); i++) {
        struct staged_data *data = all_states[i];
        if (!data || (!data->down && !data->braking)) {
            continue;
        }
        const struct staged_config *cfg = data->dev->config;
        if (cfg->x_speed == 0 && cfg->y_speed == 0) {
            continue; // precision clutch does not generate mouse movement
        }
        int64_t elapsed_ms = CLAMP(now - data->last_tick_ms, 0, 32);
        data->last_tick_ms = now;
        int32_t speed;
        if (data->down) {
            speed = staged_speed(cfg, now - data->start_ms);
        } else {
            int64_t elapsed = now - data->release_ms;
            if (cfg->brake_ms == 0 || elapsed >= cfg->brake_ms) {
                data->braking = false;
                data->remainder = 0;
                continue;
            }
            speed = (int32_t)((int64_t)data->brake_speed *
                              (cfg->brake_ms - elapsed) / cfg->brake_ms);
        }
        active = true;
        // Scale each component by 1/sqrt(2), preserving total diagonal speed.
        // Apply before fractional accumulation to avoid alternating large steps.
        if (held_x && held_y) {
            speed = (int32_t)(((int64_t)speed * 707107 + 500000) / 1000000);
        }
        data->remainder += speed * (int32_t)elapsed_ms;
        int32_t pixels = data->remainder / 1000;
        data->remainder -= pixels * 1000;
        dx += pixels * sign_speed(cfg->x_speed);
        dy += pixels * sign_speed(cfg->y_speed);
    }

    // Use one listener/device and one report frame for both axes.
    if (all_states[0] && (dx || dy)) {
        const struct device *report_dev = all_states[0]->dev;
        if (dx) {
            input_report_rel(report_dev, INPUT_REL_X, dx, dy == 0, K_NO_WAIT);
        }
        if (dy) {
            input_report_rel(report_dev, INPUT_REL_Y, dy, true, K_NO_WAIT);
        }
    }
    if (active) {
        k_work_reschedule(&shared_tick, K_MSEC(UPDATE_MS));
    }
}

static int staged_pressed(struct zmk_behavior_binding *binding,
                          struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct staged_data *data = dev->data;
    const struct staged_config *cfg = dev->config;
    if (data->down) {
        return -EBUSY;
    }

    // A zero-axis instance is the momentary precision clutch.
    if (cfg->x_speed == 0 && cfg->y_speed == 0) {
        data->down = true;
        precision_held = true;
        return 0;
    }

    for (size_t i = 0; i < ARRAY_SIZE(all_states); i++) {
        struct staged_data *state = all_states[i];
        if (state && state->braking) {
            state->braking = false;
            state->remainder = 0;
        }
    }

    data->down = true;
    data->braking = false;
    data->start_ms = k_uptime_get();
    data->last_tick_ms = data->start_ms;
    data->remainder = 0;

    // When adding a perpendicular direction, align its acceleration clock
    // and fractional-pixel phase to the already-held axis. Otherwise two
    // slightly staggered keypresses alternate X and Y movement counts.
    // Keep the one-pixel press nudge for each key unchanged.
    for (size_t i = 0; i < ARRAY_SIZE(all_states); i++) {
        struct staged_data *other = all_states[i];
        if (!other || other == data || !other->down) {
            continue;
        }
        const struct staged_config *other_cfg = other->dev->config;
        bool this_is_x = cfg->x_speed != 0;
        bool other_is_x = other_cfg->x_speed != 0;
        if (this_is_x != other_is_x) {
            data->start_ms = other->start_ms;
            data->last_tick_ms = other->last_tick_ms;
            data->remainder = other->remainder;
            break;
        }
    }

    send_delta(dev, cfg->nudge_px);
    k_work_reschedule(&shared_tick, K_MSEC(UPDATE_MS));
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
    if (cfg->x_speed == 0 && cfg->y_speed == 0) {
        data->down = false;
        precision_held = false;
        return 0;
    }
    int64_t now = k_uptime_get();
    int64_t held_ms = now - data->start_ms;
    data->down = false;
    data->remainder = 0;
    /* Never add inertia to a precision tap or slow/accelerating movement. */
    if (cfg->brake_ms && held_ms >= cfg->crawl_ms + cfg->ramp_ms) {
        data->braking = true;
        data->release_ms = now;
        data->brake_speed = MIN(staged_speed(cfg, held_ms) / 4, BRAKE_SPEED_CAP);
        k_work_reschedule(&shared_tick, K_MSEC(UPDATE_MS));
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
    if (!shared_tick_ready) {
        k_work_init_delayable(&shared_tick, staged_tick);
        shared_tick_ready = true;
    }
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

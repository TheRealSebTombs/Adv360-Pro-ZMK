/*
 * SPDX-License-Identifier: MIT
 *
 * Dedicated Cmd+Option+Enter behavior. Do not nest mod-morph instances:
 * their global modifier masks overwrite one another in this ZMK fork.
 */
#define DT_DRV_COMPAT adv360_behavior_enter_chord

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/hid.h>
#include <dt-bindings/zmk/keys.h>
#include <dt-bindings/zmk/modifiers.h>

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define GUI_MODS (MOD_LGUI | MOD_RGUI)
#define ALT_MODS (MOD_LALT | MOD_RALT)
#define CHORD_MODS (GUI_MODS | ALT_MODS)

struct enter_chord_config {
    uint32_t tapping_term_ms;
};

struct enter_chord_data {
    struct k_work_delayable hold_work;
    struct zmk_behavior_binding_event event;
    bool pressed;
    bool chord;
    bool hold;
    bool enter_down;
};

static struct zmk_behavior_binding kp_binding(uint32_t key) {
    return (struct zmk_behavior_binding) {
        .behavior_dev = DEVICE_DT_NAME(DT_NODELABEL(kp)),
        .param1 = key,
        .param2 = 0,
    };
}

static int send_key(uint32_t key, struct zmk_behavior_binding_event event, bool down) {
    struct zmk_behavior_binding binding = kp_binding(key);
    return zmk_behavior_invoke_binding(&binding, event, down);
}

static void enter_hold_timeout(struct k_work *work) {
    struct enter_chord_data *data =
        CONTAINER_OF(work, struct enter_chord_data, hold_work.work);

    if (!data->pressed || !data->chord || data->hold) {
        return;
    }

    data->hold = true;
    /* F18 must be unmodified even while Command and Option are physically held. */
    zmk_hid_masked_modifiers_set(CHORD_MODS);
    send_key(F18, data->event, true);
}

static int enter_chord_pressed(struct zmk_behavior_binding *binding,
                               struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct enter_chord_data *data = dev->data;
    const struct enter_chord_config *cfg = dev->config;

    if (data->pressed) {
        return -EBUSY;
    }

    zmk_mod_flags_t mods = zmk_hid_get_explicit_mods();
    data->pressed = true;
    data->chord = (mods & GUI_MODS) && (mods & ALT_MODS);
    data->hold = false;
    data->enter_down = false;
    data->event = event;

    if (data->chord) {
        k_work_reschedule(&data->hold_work, K_MSEC(cfg->tapping_term_ms));
        return ZMK_BEHAVIOR_OPAQUE;
    }

    data->enter_down = true;
    return send_key(ENTER, event, true);
}

static int enter_chord_released(struct zmk_behavior_binding *binding,
                                struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct enter_chord_data *data = dev->data;

    if (!data->pressed) {
        return -EINVAL;
    }

    data->pressed = false;
    k_work_cancel_delayable(&data->hold_work);

    if (data->hold) {
        int ret = send_key(F18, data->event, false);
        zmk_hid_masked_modifiers_clear();
        data->hold = false;
        return ret;
    }

    if (data->chord) {
        /* On a tap, let the physical Command+Option modifiers reach the host. */
        int ret = send_key(ENTER, event, true);
        if (ret < 0) {
            return ret;
        }
        return send_key(ENTER, event, false);
    }

    return data->enter_down ? send_key(ENTER, event, false) : 0;
}

static const struct behavior_driver_api enter_chord_api = {
    .binding_pressed = enter_chord_pressed,
    .binding_released = enter_chord_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif
};

static int enter_chord_init(const struct device *dev) {
    struct enter_chord_data *data = dev->data;
    k_work_init_delayable(&data->hold_work, enter_hold_timeout);
    return 0;
}

#define ENTER_CHORD_INST(n)                                                        \
    static const struct enter_chord_config enter_chord_cfg_##n = {                 \
        .tapping_term_ms = DT_INST_PROP(n, tapping_term_ms),                        \
    };                                                                              \
    static struct enter_chord_data enter_chord_data_##n;                           \
    BEHAVIOR_DT_INST_DEFINE(n, enter_chord_init, NULL, &enter_chord_data_##n,       \
                            &enter_chord_cfg_##n, POST_KERNEL,                      \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &enter_chord_api);

DT_INST_FOREACH_STATUS_OKAY(ENTER_CHORD_INST)

#endif

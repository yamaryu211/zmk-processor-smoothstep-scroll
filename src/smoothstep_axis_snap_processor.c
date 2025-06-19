
/*
 * ZMK用Smoothstep軸スナップ入力プロセッサー
 * スクロール専用：XY動きを滑らかに軸方向にスナップ
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zmk/input_processor.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// 固定小数点演算のためのスケール係数（1000倍精度）
#define SCALE_FACTOR 1000
#define SCALE_HALF 500

struct smoothstep_axis_snap_config {
    int32_t threshold_start;  // 開始閾値（SCALE_FACTOR倍）
    int32_t threshold_end;    // 終了閾値（SCALE_FACTOR倍）
};

struct smoothstep_axis_snap_data {
    // 状態管理（必要に応じて）
};

// 固定小数点クランプ関数
static int32_t clamp_fixed(int32_t value, int32_t min_val, int32_t max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// 固定小数点Smoothstep関数
static int32_t smoothstep_fixed(int32_t edge0, int32_t edge1, int32_t x) {
    // edge0からedge1の範囲を0～SCALE_FACTORに正規化
    int32_t range = edge1 - edge0;
    if (range <= 0) return 0;

    int32_t t = clamp_fixed(((x - edge0) * SCALE_FACTOR) / range, 0, SCALE_FACTOR);

    // 3t² - 2t³ の計算（固定小数点）
    int32_t t2 = (t * t) / SCALE_FACTOR;
    int32_t t3 = (t2 * t) / SCALE_FACTOR;

    return (3 * t2 - 2 * t3) / SCALE_FACTOR;
}

static int smoothstep_axis_snap_handle_event(const struct device *dev,
                                           struct input_event *event,
                                           uint32_t param1, uint32_t param2,
                                           struct input_event **result) {
    const struct smoothstep_axis_snap_config *cfg = dev->config;

    // REL_WHEEL（縦スクロール）とREL_HWHEEL（横スクロール）のみ処理
    if (event->type != INPUT_EV_REL || 
        (event->code != INPUT_REL_WHEEL && event->code != INPUT_REL_HWHEEL)) {
        return 0; // このイベントは処理しない
    }

    static int32_t accumulated_wheel = 0;
    static int32_t accumulated_hwheel = 0;

    // 現在のイベントを蓄積
    if (event->code == INPUT_REL_WHEEL) {
        accumulated_wheel += event->value;
    } else if (event->code == INPUT_REL_HWHEEL) {
        accumulated_hwheel += event->value;
    }

    // 軸スナップ処理
    int32_t abs_wheel = accumulated_wheel < 0 ? -accumulated_wheel : accumulated_wheel;
    int32_t abs_hwheel = accumulated_hwheel < 0 ? -accumulated_hwheel : accumulated_hwheel;

    if (abs_wheel == 0 && abs_hwheel == 0) {
        return 0; // 動きがない場合は何もしない
    }

    // 主軸の判定と減衰処理
    if (abs_wheel > abs_hwheel) {
        // 縦スクロールが主軸、横スクロールを減衰
        if (abs_wheel > 0) {
            int32_t ratio = (abs_hwheel * SCALE_FACTOR) / abs_wheel;
            int32_t smoothstep_val = smoothstep_fixed(cfg->threshold_start, 
                                                    cfg->threshold_end, ratio);
            int32_t attenuation = SCALE_FACTOR - smoothstep_val;

            accumulated_hwheel = (accumulated_hwheel * attenuation) / SCALE_FACTOR;
        }
    } else {
        // 横スクロールが主軸、縦スクロールを減衰
        if (abs_hwheel > 0) {
            int32_t ratio = (abs_wheel * SCALE_FACTOR) / abs_hwheel;
            int32_t smoothstep_val = smoothstep_fixed(cfg->threshold_start, 
                                                    cfg->threshold_end, ratio);
            int32_t attenuation = SCALE_FACTOR - smoothstep_val;

            accumulated_wheel = (accumulated_wheel * attenuation) / SCALE_FACTOR;
        }
    }

    // 処理済みの値で新しいイベントを作成
    if (event->code == INPUT_REL_WHEEL) {
        event->value = accumulated_wheel;
        accumulated_wheel = 0; // リセット
    } else if (event->code == INPUT_REL_HWHEEL) {
        event->value = accumulated_hwheel;
        accumulated_hwheel = 0; // リセット
    }

    *result = event;
    return 1; // イベントを処理済み
}

static const struct input_processor_driver_api smoothstep_axis_snap_driver_api = {
    .handle_event = smoothstep_axis_snap_handle_event,
};

static int smoothstep_axis_snap_init(const struct device *dev) {
    return 0;
}

// デバイスツリー設定マクロ
#define SMOOTHSTEP_AXIS_SNAP_INST(n)                                             static const struct smoothstep_axis_snap_config                                   smoothstep_axis_snap_config_##n = {                                               .threshold_start = DT_INST_PROP(n, threshold_start) * SCALE_FACTOR / 1000,             .threshold_end = DT_INST_PROP(n, threshold_end) * SCALE_FACTOR / 1000,         };                                                                             static struct smoothstep_axis_snap_data smoothstep_axis_snap_data_##n;        DEVICE_DT_INST_DEFINE(n, smoothstep_axis_snap_init, NULL,                                           &smoothstep_axis_snap_data_##n,                                                &smoothstep_axis_snap_config_##n, POST_KERNEL,                                 CONFIG_INPUT_INIT_PRIORITY,                                                    &smoothstep_axis_snap_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SMOOTHSTEP_AXIS_SNAP_INST)
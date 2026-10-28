/*
 * Cirque Pinnacle (絶対座標モード) の出力を受け取り、
 *   ・通常の移動         -> REL_X / REL_Y
 *   ・タップ             -> BTN_0 (左クリック)。2回タップすればダブルクリック
 *   ・タップ直後に触れて動かす / 押さえ続ける -> BTN_0 を押したままドラッグ
 *   ・外周をなぞる       -> REL_WHEEL (時計回りで下スクロール)
 * に変換して自分自身のinputデバイスとして出力する。
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT sosho_trackpad_gestures

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(trackpad_gestures, CONFIG_INPUT_LOG_LEVEL);

/* 正規化座標の範囲 (中心 0、端 ±NORM_MAX) */
#define NORM_MAX 1000
/* 中心付近は角度が不安定なので、スクロール計算に使わない半径 */
#define SCROLL_MIN_RADIUS 200
/* rad -> ミリ度 */
#define MDEG_PER_RAD 57296
/* 同時に溜めておけるクリック数 */
#define MAX_PENDING_CLICKS 3

enum tpg_state {
    TPG_IDLE,    /* 指が離れている */
    TPG_PENDING, /* 触れた直後。タップ / 移動 / スクロール / ドラッグのどれか未確定 */
    TPG_MOVE,    /* ポインタ移動 */
    TPG_SCROLL,  /* 外周スクロール */
    TPG_DRAG,    /* ボタンを押したままポインタ移動 */
};

struct tpg_config {
    int32_t x_min, x_max, y_min, y_max;
    int32_t tap_term_ms;
    int32_t tap_move_threshold;
    int32_t tap_press_ms;
    int32_t drag_window_ms;
    int32_t move_scale_mul;
    int32_t move_scale_div;
    int32_t scroll_edge_percent;
    int32_t scroll_tick_mdeg;
    bool scroll_invert;
};

struct tpg_data {
    const struct device *dev;
    bool ready;

    /* 受信中のサンプル */
    int32_t x, y, z;

    /* ジェスチャ判定の状態 (input スレッドからのみ触る) */
    enum tpg_state state;
    int64_t down_time;
    int32_t start_x, start_y;
    int32_t prev_x, prev_y;
    int32_t scroll_acc_mdeg;
    /* 移動量を縮小したときの端数 */
    int32_t rem_x, rem_y;
    bool start_on_edge;
    /* タップ直後の2回目のタッチ */
    bool after_tap;

    /* スレッド間で共有する値 (lock で保護) */
    struct k_spinlock lock;
    int32_t acc_dx, acc_dy, acc_wheel;
    uint8_t pending_clicks;
    /* タップ後、2回目のタッチを待っている */
    bool tap_waiting;
    /* ドラッグのためにボタンを押し続けたい */
    bool want_drag;

    /* 出力はシステムワークキューで行う */
    struct k_work motion_work;
    struct k_work_delayable button_work;
    struct k_work_delayable tap_timeout_work;
    /* 以下はシステムワークキューからのみ触る */
    bool button_down;
    bool button_holding;
};

static void tpg_normalize(const struct tpg_config *cfg, int32_t x, int32_t y, int32_t *nx,
                          int32_t *ny) {
    int32_t hx = (cfg->x_max - cfg->x_min) / 2;
    int32_t hy = (cfg->y_max - cfg->y_min) / 2;

    *nx = (x - (cfg->x_min + hx)) * NORM_MAX / hx;
    *ny = (y - (cfg->y_min + hy)) * NORM_MAX / hy;
}

static bool tpg_on_edge(const struct tpg_config *cfg, int32_t nx, int32_t ny) {
    int32_t r = NORM_MAX * cfg->scroll_edge_percent / 100;

    return nx * nx + ny * ny >= r * r;
}

/* 外周の起点から見て、動きが円周方向 (接線方向) 優位なら true */
static bool tpg_is_tangential(const struct tpg_config *cfg, struct tpg_data *data, int32_t x,
                              int32_t y) {
    int32_t sx, sy, cx, cy;

    tpg_normalize(cfg, data->start_x, data->start_y, &sx, &sy);
    tpg_normalize(cfg, x, y, &cx, &cy);

    int64_t dx = cx - sx;
    int64_t dy = cy - sy;
    int64_t radial = dx * sx + dy * sy;
    int64_t tangential = dx * sy - dy * sx;

    return llabs(tangential) > llabs(radial);
}

/* 前回位置から今回位置までの中心まわりの回転角 (ミリ度、時計回りが正) */
static int32_t tpg_angle_delta_mdeg(const struct tpg_config *cfg, int32_t px, int32_t py,
                                    int32_t qx, int32_t qy) {
    int32_t ax, ay, bx, by;

    tpg_normalize(cfg, px, py, &ax, &ay);
    tpg_normalize(cfg, qx, qy, &bx, &by);

    if (ax * ax + ay * ay < SCROLL_MIN_RADIUS * SCROLL_MIN_RADIUS ||
        bx * bx + by * by < SCROLL_MIN_RADIUS * SCROLL_MIN_RADIUS) {
        return 0;
    }

    /* Y は下向きが正なので、外積が正なら見た目で時計回り */
    int64_t cross = (int64_t)ax * by - (int64_t)ay * bx;
    int64_t dot = (int64_t)ax * bx + (int64_t)ay * by;

    /* 1サンプルで 45 度以上の変化はノイズとして捨てる */
    if (dot <= 0 || llabs(cross) > dot) {
        return 0;
    }

    /* 小さい角度なので atan(cross/dot) ≒ cross/dot */
    return (int32_t)(cross * MDEG_PER_RAD / dot);
}

static void tpg_queue_motion(struct tpg_data *data, int32_t dx, int32_t dy, int32_t wheel) {
    k_spinlock_key_t key = k_spin_lock(&data->lock);

    data->acc_dx += dx;
    data->acc_dy += dy;
    data->acc_wheel += wheel;

    k_spin_unlock(&data->lock, key);

    k_work_submit(&data->motion_work);
}

static void tpg_queue_clicks(struct tpg_data *data, uint8_t count) {
    k_spinlock_key_t key = k_spin_lock(&data->lock);

    data->pending_clicks = MIN(data->pending_clicks + count, MAX_PENDING_CLICKS);

    k_spin_unlock(&data->lock, key);

    /* 押下中ならリリース予定時刻はそのまま */
    k_work_schedule(&data->button_work, K_NO_WAIT);
}

static void tpg_set_drag(struct tpg_data *data, bool drag) {
    k_spinlock_key_t key = k_spin_lock(&data->lock);

    data->want_drag = drag;

    k_spin_unlock(&data->lock, key);

    k_work_schedule(&data->button_work, K_NO_WAIT);
}

static void tpg_start_drag(struct tpg_data *data) {
    data->state = TPG_DRAG;
    tpg_set_drag(data, true);
}

static void tpg_handle_touch(const struct tpg_config *cfg, struct tpg_data *data) {
    int32_t x = data->x;
    int32_t y = data->y;

    if (data->state == TPG_IDLE) {
        int32_t nx, ny;
        k_spinlock_key_t key = k_spin_lock(&data->lock);

        /* 1回目のタップのクリックは、2回目のタッチの結果に合わせて出す */
        data->after_tap = data->tap_waiting;
        data->tap_waiting = false;
        k_spin_unlock(&data->lock, key);

        tpg_normalize(cfg, x, y, &nx, &ny);
        data->state = TPG_PENDING;
        data->down_time = k_uptime_get();
        data->start_x = data->prev_x = x;
        data->start_y = data->prev_y = y;
        data->start_on_edge = tpg_on_edge(cfg, nx, ny);
        data->scroll_acc_mdeg = 0;
        data->rem_x = data->rem_y = 0;
        return;
    }

    if (data->state == TPG_PENDING) {
        int32_t sdx = x - data->start_x;
        int32_t sdy = y - data->start_y;

        if (sdx * sdx + sdy * sdy <= cfg->tap_move_threshold * cfg->tap_move_threshold) {
            /* タップ後に押さえ続けたらその場でドラッグ開始 */
            if (data->after_tap && k_uptime_get() - data->down_time > cfg->tap_term_ms) {
                tpg_start_drag(data);
            }
            /* まだタップの範囲内。ポインタは動かさない */
            return;
        }

        if (data->after_tap) {
            tpg_start_drag(data);
        } else if (data->start_on_edge && tpg_is_tangential(cfg, data, x, y)) {
            data->state = TPG_SCROLL;
        } else {
            data->state = TPG_MOVE;
        }
    }

    switch (data->state) {
    case TPG_MOVE:
    case TPG_DRAG: {
        int32_t sx = (x - data->prev_x) * cfg->move_scale_mul + data->rem_x;
        int32_t sy = (y - data->prev_y) * cfg->move_scale_mul + data->rem_y;

        /* 端数は次回に持ち越して、ゆっくり動かしても止まらないようにする */
        data->rem_x = sx % cfg->move_scale_div;
        data->rem_y = sy % cfg->move_scale_div;
        tpg_queue_motion(data, sx / cfg->move_scale_div, sy / cfg->move_scale_div, 0);
        break;
    }
    case TPG_SCROLL: {
        data->scroll_acc_mdeg +=
            tpg_angle_delta_mdeg(cfg, data->prev_x, data->prev_y, x, y);

        int32_t ticks = data->scroll_acc_mdeg / cfg->scroll_tick_mdeg;

        if (ticks != 0) {
            data->scroll_acc_mdeg -= ticks * cfg->scroll_tick_mdeg;
            /* 時計回り = 下スクロール (WHEEL は上が正) */
            tpg_queue_motion(data, 0, 0, cfg->scroll_invert ? ticks : -ticks);
        }
        break;
    }
    default:
        break;
    }

    data->prev_x = x;
    data->prev_y = y;
}

static void tpg_handle_release(const struct tpg_config *cfg, struct tpg_data *data) {
    switch (data->state) {
    case TPG_PENDING:
        if (k_uptime_get() - data->down_time > cfg->tap_term_ms) {
            break;
        }
        if (data->after_tap) {
            /* 1回目 + 2回目のタップ = ダブルクリック */
            tpg_queue_clicks(data, 2);
        } else if (cfg->drag_window_ms <= 0) {
            tpg_queue_clicks(data, 1);
        } else {
            /* 続けて触れたらドラッグになるので、クリックは少し待ってから出す */
            k_spinlock_key_t key = k_spin_lock(&data->lock);

            data->tap_waiting = true;
            k_spin_unlock(&data->lock, key);
            k_work_reschedule(&data->tap_timeout_work, K_MSEC(cfg->drag_window_ms));
        }
        break;
    case TPG_DRAG:
        tpg_set_drag(data, false);
        break;
    default:
        break;
    }

    data->state = TPG_IDLE;
}

static void tpg_input_cb(struct input_event *evt, void *user_data) {
    const struct device *dev = user_data;
    const struct tpg_config *cfg = dev->config;
    struct tpg_data *data = dev->data;

    if (!data->ready || evt->type != INPUT_EV_ABS) {
        return;
    }

    switch (evt->code) {
    case INPUT_ABS_X:
        data->x = evt->value;
        break;
    case INPUT_ABS_Y:
        data->y = evt->value;
        break;
    case INPUT_ABS_Z:
        data->z = evt->value;
        break;
    default:
        return;
    }

    if (!evt->sync) {
        return;
    }

    /* idle パケット (x=y=z=0) で指が離れたことを知る */
    if (data->z == 0) {
        tpg_handle_release(cfg, data);
    } else {
        tpg_handle_touch(cfg, data);
    }
}

static void tpg_motion_work_cb(struct k_work *work) {
    struct tpg_data *data = CONTAINER_OF(work, struct tpg_data, motion_work);
    k_spinlock_key_t key = k_spin_lock(&data->lock);
    int32_t dx = data->acc_dx;
    int32_t dy = data->acc_dy;
    int32_t wheel = data->acc_wheel;

    data->acc_dx = data->acc_dy = data->acc_wheel = 0;
    k_spin_unlock(&data->lock, key);

    if (dx != 0 || dy != 0) {
        input_report_rel(data->dev, INPUT_REL_X, dx, false, K_FOREVER);
        input_report_rel(data->dev, INPUT_REL_Y, dy, true, K_FOREVER);
    }
    if (wheel != 0) {
        input_report_rel(data->dev, INPUT_REL_WHEEL, wheel, true, K_FOREVER);
    }
}

/* 2回目のタッチが来なかったので、溜めていたタップをクリックとして出す */
static void tpg_tap_timeout_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct tpg_data *data = CONTAINER_OF(dwork, struct tpg_data, tap_timeout_work);
    k_spinlock_key_t key = k_spin_lock(&data->lock);
    bool fire = data->tap_waiting;

    data->tap_waiting = false;
    k_spin_unlock(&data->lock, key);

    if (fire) {
        tpg_queue_clicks(data, 1);
    }
}

/* pending_clicks / want_drag に合わせて BTN_0 を押したり離したりする */
static void tpg_button_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct tpg_data *data = CONTAINER_OF(dwork, struct tpg_data, button_work);
    const struct tpg_config *cfg = data->dev->config;
    k_spinlock_key_t key = k_spin_lock(&data->lock);
    bool click = data->pending_clicks > 0;
    bool drag = data->want_drag;

    if (!data->button_down && click) {
        data->pending_clicks--;
    }
    k_spin_unlock(&data->lock, key);

    if (data->button_down) {
        if (data->button_holding && drag) {
            return;
        }

        input_report_key(data->dev, INPUT_BTN_0, 0, true, K_FOREVER);
        data->button_down = false;
        data->button_holding = false;

        if (click || drag) {
            /* 次の押下まで少し間を空ける (ダブルクリック用) */
            k_work_reschedule(dwork, K_MSEC(cfg->tap_press_ms));
        }
        return;
    }

    if (click) {
        input_report_key(data->dev, INPUT_BTN_0, 1, true, K_FOREVER);
        data->button_down = true;
        k_work_reschedule(dwork, K_MSEC(cfg->tap_press_ms));
    } else if (drag) {
        input_report_key(data->dev, INPUT_BTN_0, 1, true, K_FOREVER);
        data->button_down = true;
        data->button_holding = true;
    }
}

static int tpg_init(const struct device *dev) {
    struct tpg_data *data = dev->data;

    data->dev = dev;
    data->state = TPG_IDLE;
    k_work_init(&data->motion_work, tpg_motion_work_cb);
    k_work_init_delayable(&data->button_work, tpg_button_work_cb);
    k_work_init_delayable(&data->tap_timeout_work, tpg_tap_timeout_work_cb);
    data->ready = true;

    return 0;
}

#define TPG_INST(n)                                                                                \
    BUILD_ASSERT(DT_INST_PROP(n, x_max) > DT_INST_PROP(n, x_min), "x-max must be > x-min");      \
    BUILD_ASSERT(DT_INST_PROP(n, y_max) > DT_INST_PROP(n, y_min), "y-max must be > y-min");      \
    BUILD_ASSERT(DT_INST_PROP(n, scroll_tick_degrees) > 0, "scroll-tick-degrees must be > 0");   \
    BUILD_ASSERT(DT_INST_PROP_BY_IDX(n, move_scale, 1) > 0, "move-scale divisor must be > 0");   \
    static struct tpg_data tpg_data_##n;                                                           \
    static const struct tpg_config tpg_config_##n = {                                              \
        .x_min = DT_INST_PROP(n, x_min),                                                           \
        .x_max = DT_INST_PROP(n, x_max),                                                           \
        .y_min = DT_INST_PROP(n, y_min),                                                           \
        .y_max = DT_INST_PROP(n, y_max),                                                           \
        .tap_term_ms = DT_INST_PROP(n, tap_term_ms),                                               \
        .tap_move_threshold = DT_INST_PROP(n, tap_move_threshold),                                 \
        .tap_press_ms = DT_INST_PROP(n, tap_press_ms),                                             \
        .drag_window_ms = DT_INST_PROP(n, drag_window_ms),                                         \
        .move_scale_mul = DT_INST_PROP_BY_IDX(n, move_scale, 0),                                   \
        .move_scale_div = DT_INST_PROP_BY_IDX(n, move_scale, 1),                                   \
        .scroll_edge_percent = DT_INST_PROP(n, scroll_edge_percent),                               \
        .scroll_tick_mdeg = DT_INST_PROP(n, scroll_tick_degrees) * 1000,                           \
        .scroll_invert = DT_INST_PROP(n, scroll_invert),                                           \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, tpg_init, NULL, &tpg_data_##n, &tpg_config_##n, POST_KERNEL,          \
                          CONFIG_SOSHO_TRACKPAD_GESTURES_INIT_PRIORITY, NULL);                     \
    INPUT_CALLBACK_DEFINE_NAMED(DEVICE_DT_GET(DT_INST_PHANDLE(n, device)), tpg_input_cb,           \
                                (void *)DEVICE_DT_INST_GET(n), tpg_input_cb_##n);

DT_INST_FOREACH_STATUS_OKAY(TPG_INST)

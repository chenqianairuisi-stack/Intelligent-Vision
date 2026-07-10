#include "Storage.h"
#include "tuning_config.h"
#include "zf_common_headfile.h"
#include <string.h>

#define FLASH_SECTION_INDEX       (127)
#define FLASH_PAGE_INDEX          (FLASH_PAGE_7)
// 注意：每次改动 TuningConfig 的内存布局都要 bump 此 magic，
// 否则旧 flash 数据会被裸 memcpy 读成错位/垃圾值（本结构无版本号）。
// 0xAA55CC56: foundation 阶段为 latency 结构追加拐点延时估计字段
// 0xAA55CC57: 运动控制阶段追加 tracker.corner_pause_speed / stanley / bomb 结构
// 注：2026-07-04 在 TuningConfig **末尾**追加 feel{brake_hold_gain,corner_turn_acc}，
//     属"只在尾部追加"——旧 flash 读出的尾部区由 sanitize 兜回默认，故**不 bump magic**、
//     保留用户已调参数。只有在结构体中段插字段/改布局时才必须 bump。
#define CONFIG_MAGIC_WORD         (0xAA55CC57)

void Storage::init() {
    flash_init();

    if (flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX)) {
        if (load_params()) {
            save_params();
        }
    } else {
        (void)TuningDefaults::sanitize(tune);
        save_params();
    }
}

void Storage::save_params() {
    (void)TuningDefaults::sanitize(tune);
    flash_buffer_clear();

    flash_union_buffer[0].uint32_type = CONFIG_MAGIC_WORD;
    memcpy(&flash_union_buffer[1], &tune, sizeof(TuningConfig));

    flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
}

bool Storage::load_params() {
    bool changed = false;

    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);

    if (flash_union_buffer[0].uint32_type != CONFIG_MAGIC_WORD) {
        (void)TuningDefaults::sanitize(tune);
        return true;
    }

    memcpy(&tune, &flash_union_buffer[1], sizeof(TuningConfig));

    changed = TuningDefaults::sanitize(tune) || changed;
    if (tune.dynamics.max_vel == 60.0f) {
        tune.dynamics.max_vel = TuningDefaults::DEFAULT_DYNAMICS_MAX_VEL;
        changed = true;
    }

    if (tune.latency.encoder_latency_gain == 1.00f &&
        tune.latency.vision_latency_ms == 300.0f) {
        tune.latency.encoder_latency_gain = TuningDefaults::DEFAULT_ENCODER_LATENCY_GAIN;
        tune.latency.vision_latency_ms = TuningDefaults::DEFAULT_VISION_LATENCY_MS;
        changed = true;
    }

    // 旧默认 brake_limit=0.85 会让规划层过晚刹车，实车轮速环跟不上时表现为打滑、过冲和停前晃。
    if (tune.dynamics.brake_limit > 0.80f) {
        tune.dynamics.brake_limit = TuningDefaults::DEFAULT_BRAKE_LIMIT;
        changed = true;
    }

    // Turn_V/end_speed 保持 0：连贯性靠提前切段和切向限幅，不靠带末速斜切。
    // 旧 flash 若残留非零 Turn_V 或 Stanley 开，会导致拐点带速磨圆/抖，这里刷回安全默认。
    if (tune.tracker.corner_pass_speed > 0.0f || tune.stanley.enable) {
        tune.tracker.corner_pass_speed = TuningDefaults::DEFAULT_CORNER_PASS_SPEED;
        tune.stanley.enable = false;
        changed = true;
    }

    // explosion_wait_ms 兜底：早期默认给了 100ms 占位（墙炸开约~1s），会导致车没等墙炸穿就穿墙。
    // flash 里若残留过短值(<500ms，任何真墙都炸不开)就抬到安全默认；>=500 视为用户已标定，保留。
    if (tune.bomb.explosion_wait_ms < 500.0f) {
        tune.bomb.explosion_wait_ms = TuningDefaults::DEFAULT_EXPLOSION_WAIT_MS;
        changed = true;
    }

    // yaw 控制链融合(2026-07-10)：新的 sqrt/线性/陀螺阻尼 yaw 控制器复用 dynamics.max_ang_*
    // 与 tracker.ang_tolerance，但它们是旧 Master 为纯 P 控制器整定的(3.0/4.6/0.006)。
    // 精确匹配旧默认值一次性迁到 Branch 值，避免旧 flash 让新控制器跑得过钝；
    // 用户已改过(值≠旧默认)则不动，不与现场整定打架。
    if (tune.dynamics.max_ang_vel == 3.0f && tune.dynamics.max_ang_acc == 4.6f) {
        tune.dynamics.max_ang_vel = 10.0f;
        tune.dynamics.max_ang_acc = 40.0f;
        changed = true;
    }
    if (tune.tracker.ang_tolerance == 0.006f) {
        tune.tracker.ang_tolerance = 0.010f;
        changed = true;
    }

    return changed;
}

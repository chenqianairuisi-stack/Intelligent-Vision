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
// 注：2026-07-14 在 TuningConfig **最末尾**（wheels[] 之后）追加 stop_approach_band_cm，
//     同为"尾部追加"，wheels/yaw/feel 等偏移不变、旧 flash 已调参数照常读出，新字段由
//     sanitize 兜回默认，故同样**不 bump magic**。
// 注：2026-07-15 在 TuningConfig **最末尾**（stop_approach_band_cm 之后）追加 stop_approach_brake_gain，
//     同为"尾部追加"、前面所有字段偏移不变，旧 flash 读出的该尾部区（0/垃圾）由 sanitize 兜回 1.0
//     = 旧行为，故仍**不 bump magic**。
// 注：2026-07-15 继续尾部追加 short_seg_len_cm/short_seg_accel（长短段起步加速度切换，默认 60/300）
//     与 approach_zone_cm/approach_zone_ratio/approach_brake_acc/approach_enable（停车接近区双段提前
//     刹车，默认启用 40/0.25/15/1）。均由 sanitize 兜回默认（**默认启用**，系用户实车拍板的目标行为），
//     故仍**不 bump magic**。approach_enable 为 float 存的开关（0=关/1=开），clamp 到 [0,1]。
// 注：2026-07-16 在 TuningConfig 最末尾追加 kinematics.strafe_decouple（X 轴横向指令侧解耦，默认 0.03），
//     旧 flash 无此尾部字段时由 sanitize 兜回默认；此次 bump magic 到 0xAA55CC59，旧 flash 一律取默认。
// 兼容读取现有同布局版本并只迁移四轮 ka，避免清空其他已调参数
#define CONFIG_MAGIC_WORD_PRE_WHEEL_KA_ZERO_V1 (0xAA55CC59)
#define CONFIG_MAGIC_WORD_PRE_WHEEL_KA_ZERO_V2 (0xAA55CC5A)
#define CONFIG_MAGIC_WORD_PRE_CORNER_SPEED_V3  (0xAA55CC5B)
#define CONFIG_MAGIC_WORD                      (0xAA55CC5C)

static_assert(sizeof(TuningConfig) + sizeof(flash_data_union) <= FLASH_PAGE_SIZE,
              "TuningConfig does not fit in one flash page");

void Storage::init() {
    flash_init();

    if (flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX)) {
        if (load_params()) {
            (void)save_params();
        }
    } else {
        (void)TuningDefaults::sanitize(tune);
        (void)save_params();
    }
}

bool Storage::save_params() {
    (void)TuningDefaults::sanitize(tune);
    flash_buffer_clear();

    flash_union_buffer[0].uint32_type = CONFIG_MAGIC_WORD;
    memcpy(&flash_union_buffer[1], &tune, sizeof(TuningConfig));

    if (flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX) != 0U) {
        return false;
    }
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);

    // 底层 buffer 写接口不透传写入状态，因此回读并逐字节确认实际持久化结果
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
    return flash_union_buffer[0].uint32_type == CONFIG_MAGIC_WORD &&
           memcmp(&flash_union_buffer[1], &tune, sizeof(TuningConfig)) == 0;
}

bool Storage::load_params() {
    bool changed = false;

    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);

    uint32_t stored_magic = flash_union_buffer[0].uint32_type;
    bool needs_wheel_ka_migration =
        stored_magic == CONFIG_MAGIC_WORD_PRE_WHEEL_KA_ZERO_V1 ||
        stored_magic == CONFIG_MAGIC_WORD_PRE_WHEEL_KA_ZERO_V2;
    bool needs_corner_speed_migration =
        stored_magic == CONFIG_MAGIC_WORD_PRE_CORNER_SPEED_V3;
    if (stored_magic != CONFIG_MAGIC_WORD &&
        !needs_wheel_ka_migration &&
        !needs_corner_speed_migration) {
        (void)TuningDefaults::sanitize(tune);
        return true;
    }

    memcpy(&tune, &flash_union_buffer[1], sizeof(TuningConfig));

    changed = TuningDefaults::sanitize(tune) || changed;
    if (needs_wheel_ka_migration) {
        // 首次运行新版固件时只清四轮加速前馈，其他已保存参数保持不变
        for (int wheel = 0; wheel < 4; ++wheel) {
            tune.wheels[wheel].ka = 0.0f;
        }
        changed = true;
    }

    if (needs_corner_speed_migration &&
        tune.tracker.corner_pass_speed <= 26.0f) {
        // 5B 固件的默认值为 26，升级后改为固定带速切向默认值 120
        tune.tracker.corner_pass_speed = TuningDefaults::DEFAULT_CORNER_PASS_SPEED;
        changed = true;
    }

    // 以下"值等式/阈值"迁移只应对"从旧固件 flash(V1/V2 magic) 首次升级"跑一次。
    // 当前 magic(V3) 存回的 flash 一律不再触发——否则每次上电都把用户在菜单 Save 的
    // 合法值(brake_limit>0.80、corner_pass_speed>0、max_vel==60 等)强抹回默认，
    // 表现为"调好存了、重启又变回默认"。init() 首启会在迁移后 save_params() 写回 V3 magic，
    // 下次启动 needs_wheel_ka_migration=false 即整块跳过。
    if (!needs_wheel_ka_migration) {
        return changed;
    }

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

    // 旧版本的 Turn_V 由默认 0 改为固定带速切向，旧 flash 一律迁移到新默认。
    if (tune.tracker.corner_pass_speed != TuningDefaults::DEFAULT_CORNER_PASS_SPEED ||
        tune.stanley.enable) {
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

    // 视觉纠偏幅度加大(2026-07-14)：FULL_MAX_STEP_CM 步长 1.5→5，配套把接受阈值默认 3→5。
    // flash 里若残留旧默认 3.0 就抬到 5.0，让"改默认重烧"即生效；用户已现场调过(!S G，值≠3.0)则保留。
    if (tune.tracker.vision_reject_dist == 3.0f) {
        tune.tracker.vision_reject_dist = TuningDefaults::DEFAULT_VISION_REJECT_DIST;
        changed = true;
    }

    return changed;
}

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
    if (tune.latency.encoder_latency_gain == 1.00f &&
        tune.latency.vision_latency_ms == 300.0f) {
        tune.latency.encoder_latency_gain = TuningDefaults::DEFAULT_ENCODER_LATENCY_GAIN;
        tune.latency.vision_latency_ms = TuningDefaults::DEFAULT_VISION_LATENCY_MS;
        changed = true;
    }

    // 一次性迁移：过弯保留速度(Turn_V)历史上被旧 sanitize 钳成 0，旧 flash 普遍存的是
    // 0=每个拐点都停。检测到 <=0 视为"旧的未迁移 flash"，在此一次性把运动手感新默认刷上：
    //   - Turn_V 拉到默认值，让"过弯不停顿"在仅刷固件、不动其它已调参数下直接生效；
    //   - Stanley 关闭（本底盘横移耦合，持续横纠会抖）。
    // 一旦 Turn_V 被设为 >0（本块写入后即如此）就不再触发，后续可自由调 Turn_V/Stanley 并持久化。
    if (tune.tracker.corner_pass_speed <= 0.0f) {
        tune.tracker.corner_pass_speed = TuningDefaults::DEFAULT_CORNER_PASS_SPEED;
        tune.stanley.enable = false;
        changed = true;
    }

    return changed;
}

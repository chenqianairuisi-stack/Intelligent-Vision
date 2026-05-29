#include "Storage.h"
#include "tuning_config.h"
#include "zf_common_headfile.h"
#include <string.h>

#define FLASH_SECTION_INDEX       (127)
#define FLASH_PAGE_INDEX          (FLASH_PAGE_7)
#define CONFIG_MAGIC_WORD         (0xAA55CC55)

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

    return changed;
}

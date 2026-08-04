/// \file storage.cpp
/// \brief 带版本和 CRC 校验的参数 Flash 持久化

#include "Storage.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "tuning_config.h"
#include "zf_common_headfile.h"

namespace {

constexpr std::uint32_t FLASH_SECTION_INDEX = 127u;
constexpr flash_page_enum FLASH_PAGE_INDEX = FLASH_PAGE_7;
constexpr std::uint32_t CONFIG_MAGIC = 0x49565444u; // IVTC
constexpr std::uint16_t CONFIG_VERSION = 1u;

struct ConfigHeader {
    std::uint32_t magic;
    std::uint16_t version;
    std::uint16_t payload_size;
    std::uint32_t payload_crc32;
};

static_assert(sizeof(ConfigHeader) + sizeof(TuningConfig) <= FLASH_PAGE_SIZE,
              "TuningConfig exceeds one Flash page");

std::uint32_t crc32(const void* data, std::size_t size) {
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    std::uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t index = 0u; index < size; ++index) {
        crc ^= bytes[index];
        for (std::uint8_t bit = 0u; bit < 8u; ++bit) {
            const std::uint32_t mask = 0u - (crc & 1u);
            crc = (crc >> 1u) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

void apply_config(const TuningConfig& config) {
    const std::uint32_t primask = interrupt_global_disable();
    tune = config;
    interrupt_global_enable(primask);
}

} // namespace

void Storage::init() {
    flash_init();
    if (!load_params()) {
        reset_params();
        (void)save_params();
    }
}

bool Storage::save_params() {
    TuningConfig snapshot;
    const std::uint32_t primask = interrupt_global_disable();
    snapshot = tune;
    interrupt_global_enable(primask);

    if (!TuningRegistry::config_valid(snapshot)) return false;

    const ConfigHeader header = {
        CONFIG_MAGIC,
        CONFIG_VERSION,
        static_cast<std::uint16_t>(sizeof(TuningConfig)),
        crc32(&snapshot, sizeof(snapshot)),
    };

    flash_buffer_clear();
    auto* output = reinterpret_cast<std::uint8_t*>(flash_union_buffer);
    std::memcpy(output, &header, sizeof(header));
    std::memcpy(output + sizeof(header), &snapshot, sizeof(snapshot));

    if (flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX) != 0u) {
        return false;
    }
    const std::size_t record_size = sizeof(header) + sizeof(snapshot);
    const std::uint16_t word_count = static_cast<std::uint16_t>((record_size + 3u) / 4u);
    return flash_write_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX,
                            reinterpret_cast<const uint32*>(flash_union_buffer),
                            word_count) == 0u;
}

bool Storage::load_params() {
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
    const auto* input = reinterpret_cast<const std::uint8_t*>(flash_union_buffer);

    ConfigHeader header{};
    std::memcpy(&header, input, sizeof(header));
    if (header.magic != CONFIG_MAGIC || header.version != CONFIG_VERSION ||
        header.payload_size != sizeof(TuningConfig)) {
        return false;
    }

    TuningConfig candidate{};
    std::memcpy(&candidate, input + sizeof(header), sizeof(candidate));
    if (crc32(&candidate, sizeof(candidate)) != header.payload_crc32 ||
        !TuningRegistry::config_valid(candidate)) {
        return false;
    }

    apply_config(candidate);
    return true;
}

void Storage::reset_params() {
    apply_config(DEFAULT_TUNE_CONFIG);
}

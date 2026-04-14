#pragma once
#include <cstdint>
#include "system_config.h"


// 地图库定义：每个地图由一个名称和一个字符串数组组成，字符串数组表示地图的网格布局
struct MockMapDef {
    const char* name;
    const char* layout[SystemConfig::MAP_MAX_HEIGHT];
};

extern const MockMapDef mock_map_library[7];    // 静态只读地图库
constexpr uint8_t MOCK_MAP_COUNT = 7;           // 地图库中地图的总数

namespace TestMap {
    uint8_t get_mock_map_count();                // 获取地图库中地图的总数
    const char* get_mock_map_name(uint8_t idx);  // 获取地图库中指定地图的名称
    void load_mock_map(uint8_t map_idx);         // 注入本地脱机测试地图数据
}


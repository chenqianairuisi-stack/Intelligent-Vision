#include "TestMap.h"
#include "Vision.h"
#include "UartComm.h"
#include "RobotState.h"

// 静态只读地图库
const MockMapDef mock_map_library[6] = {
    {
        " Map0_1",
        {
            "############",
            "#----------#",
            "#----------#",
            "#----------#",
            "#--.--$----#",
            "#----------#",
            "#----------#",
            "#----------#",
            "#---$---.--#",
            "#----------#",
            "#--.-------#",
            "#----------#",
            "#--$-------#",
            "#----------#",
            "#----------#",
            "############"
        }
    },
    {
        " Map1_1",
        {
            "############",
            "#----------#",
            "#-######---#",
            "#-#----#---#",
            "#-#-##$----#",
            "#-#..------#",
            "#-####$--#-#",
            "#----#---#-#",
            "#----#---#-#",
            "#----#---#-#",
            "#--###-----#",
            "#--#-------#",
            "#--#-------#",
            "#-.$-------#",
            "##-#-------#",
            "############"
        }
    },
    {
        " Map1_2",
        {
            "############",
            "####-------#",
            "#.------#-.#",
            "#####-###--#",
            "#----------#",
            "#-----###--#",
            "#----------#",
            "#--#-#-#-#-#",
            "##--$-$-$--#",
            "#--#-#-#-#-#",
            "#-------#-.#",
            "#---###-#--#",
            "#----------#",
            "#---#-#-#--#",
            "#----------#",
            "############"
        }
    },
    {
        " Map2_1",
        {
            "############",
            "#----------#",
            "#-####-----#",
            "#-#--#-----#",
            "#-#.-#-----#",
            "#-####-----#",
            "#--------#-#",
            "#------$-#-#",
            "#--*-----#-#",
            "#--------#-#",
            "#-------.--#",
            "#-$--------#",
            "#-------$--#",
            "#-.--------#",
            "##-#-------#",
            "############"
        }
    },
    {
        " Map2_2",
        {
            "############",
            "#.-$-------#",
            "#-#---####-#",
            "#-#---#-$*-#",
            "#-#----*-#-#",
            "#####----#-#",
            "#--------#-#",
            "#-###----#-#",
            "##-------#.#",
            "#--###---###",
            "#-##-------#",
            "#--####----#",
            "##--.#-*-$-#",
            "#--#-------#",
            "#--##------#",
            "############"
        }
    },
    {
        " Map2_3",
        {
            "############",
            "#-----.-$--#",
            "#-####--.--#",
            "#-#--#-----#",
            "#-#.-#--$--#",
            "#-####-----#",
            "#--------#-#",
            "#------$-#-#",
            "#--*-----#-#",
            "#--------#-#",
            "#-------.--#",
            "#-$--------#",
            "#-------$--#",
            "#-.--------#",
            "##-#-------#",
            "############"
        }
    },
};

namespace TestMap { 

// 获取地图库中地图的总数
uint8_t get_mock_map_count() {return MOCK_MAP_COUNT;}

// 获取地图库中指定地图的名称
const char* get_mock_map_name(uint8_t idx) {
    if (idx < MOCK_MAP_COUNT) return mock_map_library[idx].name;
    return "Unknown";
}

// 注入本地脱机测试地图数据
void load_mock_map(uint8_t map_idx) {
    if (map_idx >= MOCK_MAP_COUNT) return; // 越界保护
    auto& vision_data = App::g_state.vision;
    // 获取选中的地图布局
    const auto& map_layout = mock_map_library[map_idx].layout;

    vision_data.box_count = 0;
    vision_data.bomb_count = 0;
    uint8_t target_count = 0;

    // 遍历解析字符矩阵
    for (int y = 0; y < SystemConfig::MAP_MAX_HEIGHT; y++) {
        for (int x = 0; x < SystemConfig::MAP_MAX_WIDTH; x++) {
            
            char ch = map_layout[SystemConfig::MAP_MAX_HEIGHT - 1 - y][x]; 
    
            vision_data.map[y][x] = 0;
            switch(ch) {
                case '#': 
                    vision_data.map[y][x] = 1; 
                    break;
                case '.': 
                    if (target_count < SystemConfig::MAX_BOXES) {
                        vision_data.targets[target_count] = {(int8_t)x, (int8_t)y};
                        target_count++;
                    }
                    break;
                case '$': 
                    if (vision_data.box_count < SystemConfig::MAX_BOXES) {
                        vision_data.boxes[vision_data.box_count] = {(int8_t)x, (int8_t)y};
                        vision_data.box_count++;
                    }
                    break;
                case '*': 
                    if (vision_data.bomb_count < SystemConfig::MAX_BOMBS) {
                        vision_data.bombs[vision_data.bomb_count] = {(int8_t)x, (int8_t)y};
                        vision_data.bomb_count++;
                    }
                    break;
                default: 
                    break;
            }
        }
    }
    
    vision_data.art1_map_ready = true; 
}


}

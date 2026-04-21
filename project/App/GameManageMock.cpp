#include "GameManage.h"
#include "tuning_config.h"
#include <cmath>
#include <cstring>

namespace App::GameEngine {

// =========================================================
// 本地地图库 (位于 Flash ROM，供 MockGameManager 加载使用)
// =========================================================
struct MockMapDef {
    std::string_view name;
    std::array<std::string_view, SystemConfig::MAP_MAX_HEIGHT> layout;
};

constexpr std::array<MockMapDef, 8> mock_map_library = {{
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
            "#.---------#",
            "#----------#",
            "#--$--####-#",
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
            "############"
        }
    },
    {
        " Map2_3",
        {
            "############",
            "##.--------#",
            "#----------#",
            "#-#-#-##*#-#",
            "#-#---#-$#-#",
            "#-#----*-#-#",
            "#####----#-#",
            "#--------#-#",
            "#-###----#-#",
            "##-------#.#",
            "#--###---###",
            "#-##-------#",
            "#--####-$--#",
            "##--.#-*-$-#",
            "#--#-------#",
            "############"
        }
    },
    {
        " Map2_4",
        {
            "############",
            "#.---------#",
            "#----------#",
            "#-########-#",
            "#-#---#--#.#",
            "#-#-----*#-#",
            "##########$#",
            "#--------#-#",
            "#-##-*--$#-#",
            "##-------#-#",
            "#---##--####",
            "#--#-------#",
            "#--####----#",
            "##--.#-*-$-#",
            "#----------#",
            "############"
        }
    },
    {
        " Map3_1",
        {
            "############",
            "#----------#",
            "#-------.--#",
            "#-###------#",
            "#-#.#---$--#",
            "#-###------#",
            "#---$---.--#",
            "#------$---#",
            "#--*-------#",
            "#-$--.-----#",
            "#-------.--#",
            "#-$--------#",
            "#-------$--#",
            "#-.--------#",
            "#----------#",
            "############"
        }
    }
}};

uint8_t get_mock_map_count() { 
    return mock_map_library.size(); 
}

const char* get_mock_map_name(uint8_t idx) { 
    if (idx >= mock_map_library.size()) return "Unknown";
    return mock_map_library[idx].name.data(); 
}

void load_mock_map(uint8_t map_idx) {
    if (map_idx >= mock_map_library.size()) return;
    
    auto& vision_data = App::g_state.vision;
    const auto& map_layout = mock_map_library[map_idx].layout;

    vision_data.box_count = 0;
    vision_data.bomb_count = 0;
    uint8_t target_count = 0;

    for (int y = 0; y < SystemConfig::MAP_MAX_HEIGHT; y++) {
        for (int x = 0; x < SystemConfig::MAP_MAX_WIDTH; x++) {
            char ch = map_layout[SystemConfig::MAP_MAX_HEIGHT - 1 - y][x]; 
            vision_data.map[y][x] = 0;
            switch(ch) {
                case '#': vision_data.map[y][x] = 1; break;
                case '.': 
                    if (target_count < SystemConfig::MAX_BOXES)
                        vision_data.targets[target_count++] = {(int8_t)x, (int8_t)y};
                    break;
                case '$': 
                    if (vision_data.box_count < SystemConfig::MAX_BOXES)
                        vision_data.boxes[vision_data.box_count++] = {(int8_t)x, (int8_t)y};
                    break;
                case '*': 
                    if (vision_data.bomb_count < SystemConfig::MAX_BOMBS)
                        vision_data.bombs[vision_data.bomb_count++] = {(int8_t)x, (int8_t)y};
                    break;
            }
        }
    }
    vision_data.art1_map_ready = true; 
}

void MockGameManager::inject_mock_semantics() {
    std::memset(mock_truth_labels, -1, sizeof(mock_truth_labels));

    // 自动构造一组一一对应标签：前 N 个箱子 ↔ 后 N 个目标
    // for(int i = 0; i < num; i++) {
    //     mock_truth_labels[i] = i + 1;          // 箱子 ID
    //     mock_truth_labels[num + i] = i + 1;    // 目标 ID 起始偏移 = BOXE_COUNT，确保不与箱子 ID 冲突
    // }

    // 手动指定标签进行测试
    mock_truth_labels[0] = 1; 
    mock_truth_labels[1] = 3; 
    mock_truth_labels[2] = 2;    
    mock_truth_labels[logical_level.box_count] = 1;     
    mock_truth_labels[logical_level.box_count + 1] = 2;  
    mock_truth_labels[logical_level.box_count + 2] = 3;   
}

__attribute__((section(".ramfunc"))) void MockGameManager::update() {
    auto& game = App::g_state.game;
    auto& pos = App::g_state.physical.pose;
    auto& ctrl = App::g_state.control;

    // 【多态拦截】：专挑需要 Mock 的阶段截流，其他放行给基类
    switch (game.phase) {
        case GamePhase::EXIT_START_ZONE: {
            // 监控是否到达出库点
            float dist = std::sqrt((OUT_TARGET_X - pos.x)*(OUT_TARGET_X - pos.x) + 
                (OUT_TARGET_Y - pos.y)*(OUT_TARGET_Y - pos.y));
            
            // 到位后进入地图输入阶段
            if (dist < tune.tracker.reach_radius_min) {
                load_mock_map(App::g_state.game.selected_map_id);  // 直接加载本地测试地图，绕过视觉输入
                game.phase = GamePhase::WAIT_FOR_VISION;
            }
            break;
        }
        case GamePhase::WAIT_FOR_VISION: {
            // 在这里拦截，并注入语义
            GameManager::update(); // 调用基类处理，它会发现 art1_map_ready==true 并转储逻辑
            if (game.phase == GamePhase::PLAN_PATROL) {
                inject_mock_semantics();
            }
            break;
        }
        case GamePhase::EXEC_ALIGN_YAW: {
            // 到位后原地对齐朝向，准备触发 ART2 抓拍
            ctrl.current_target.yaw = patrol_actions[game.action_idx].obs.target_yaw;  
            ctrl.mode = ControlMode::MANUAL_DEBUG; // 停止循迹，仅执行角度对齐

            // 检查 Yaw 角度误差是否小于 5 度
            float current_yaw = App::g_state.physical.pose.yaw;
            float err_yaw = std::abs(ctrl.current_target.yaw - current_yaw);
            if (err_yaw > 180.0f) err_yaw = 360.0f - err_yaw;

            // 朝向收敛后触发 ART2 捕捉
            if (err_yaw < 5.0f) { 
                uint8_t entity_id = patrol_actions[game.action_idx].obs.entity_id;
                App::g_state.vision.semantic_labels[entity_id] = mock_truth_labels[entity_id];
                
                logical_level.player_start = patrol_actions[game.action_idx].obs.pos; 
                game.action_idx++;

                game.phase = GamePhase::EXEC_ACTION_DISPATCH;
            }
            break;
        }
        default:
            // 未被拦截的阶段，回退使用正赛物理主轴
            GameManager::update();
            break;
    }
}

} // namespace App::GameEngine
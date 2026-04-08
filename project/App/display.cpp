#include "display.h"
#include "task_control.h"
#include "tuning_config.h"
#include "task_manage.h"
#include "odometry.h"
#include "encoder.h"
#include "imu_process.h"
#include "storage.h"
#include "telemetry.h"
#include "test_loadmap.h"


// ===================================================== 全局常量与参数字典 =====================================================

TftMenu sys_menu;

// 128x160 的屏幕，6x8 的字体，留出少量间距后可用 16 行，21 列
static constexpr int UI_COL_W = 6; 
static constexpr int UI_ROW_H = 10;

// ============================= 全局参数字典 ===========================
// 结构：{ "屏幕显示名称",  变量的内存地址,  按键单次加减步长 }
struct ParamItem { const char* name; float* val_ptr; float step; };

static ParamItem tune_dict[] = {
    {"Yaw_Kp  ",   &tune.pid_yaw.kp,                  0.1f  },
    {"Yaw_Kd  ",   &tune.pid_yaw.kd,                  0.01f },
    {"Spd_Kp  ",   &tune.pid_speed.kp,                0.1f  },
    {"Spd_Ki  ",   &tune.pid_speed.ki,                0.01f },
    {"Max_Spd ",   &tune.dynamics.max_speed,          1.0f  },
    {"Max_Acc ",   &tune.dynamics.max_acc,            10.0f },
    {"MaxJerk ",   &tune.dynamics.max_jerk,           100.0f},
    {"MaxASpd ",   &tune.dynamics.max_ang_speed,      0.1f  },
    {"ReachRad",   &tune.tracker.reach_radius,        1.0f  },
    {"ReachMin",   &tune.tracker.reach_radius_min,    0.5f  },
    {"LF_Spd  ",   &tune.motors.lf_speed,             2.0f  },
    {"LB_Spd  ",   &tune.motors.lb_speed,             2.0f  },
    {"RF_Spd  ",   &tune.motors.rf_speed,             2.0f  },
    {"RB_Spd  ",   &tune.motors.rb_speed,             2.0f  }
};

static constexpr int DICT_SIZE = sizeof(tune_dict) / sizeof(tune_dict[0]);     // 自动计算字典大小
static constexpr int PARAMS_PER_PAGE = 12;                                     // 每页显示的参数数量，超过会自动滚动
// ======================================================================



void TftMenu::init() {
    current_page = MenuPage::MAIN_MENU;
    cursor_idx = 0;
    scroll_offset = 0;
    map_cursor_idx = 0;
    map_scroll_offset = 0;
    is_editing = false;
    need_full_redraw = true;  
    ui_dirty = true;           
    is_closed = false;
    last_k1 = true; last_k2 = true; last_k3 = true; last_k4 = true;

    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);

    tft180_set_dir(TFT180_PORTAIT);                  // 竖屏
    tft180_set_font(TFT180_6X8_FONT);                // 设置字库
    tft180_set_color(RGB565_BLACK, RGB565_WHITE);    // 白底黑字
    tft180_init();
    
    tft180_full(RGB565_WHITE);
    system_delay_ms(50);                             // 延时确保初始化完成
}


// ===================================================== 主循环与核心功能 =====================================================

// 主循环：扫描按键、处理逻辑、渲染 UI
void TftMenu::run() { 
    scan_keys(); 
    if (is_closed) {
        if (key_back_pressed) {     // 按返回键唤醒
            is_closed = false;
            need_full_redraw = true;
            current_page = MenuPage::MAIN_MENU;
            cursor_idx = 0;
            ui_dirty = true;
        }
        return;  // 拦截后续逻辑与渲染
    }
    
    process_logic(); 
    render_ui(); 
}

// 按键扫描函数：检测按键边缘，更新按键状态变量，并标记 UI 需要刷新
void TftMenu::scan_keys() { 
    bool cur_k1 = gpio_get_level(KEY1);
    bool cur_k2 = gpio_get_level(KEY2);
    bool cur_k3 = gpio_get_level(KEY3);
    bool cur_k4 = gpio_get_level(KEY4);

    // 边缘检测触发：只有在按键从 高电平(true) 变到 低电平(false) 的瞬间，才触发一次 true
    key_down_pressed  = (last_k1 && !cur_k1);
    key_enter_pressed = (last_k2 && !cur_k2);
    key_up_pressed    = (last_k3 && !cur_k3);
    key_back_pressed  = (last_k4 && !cur_k4);

    last_k1 = cur_k1; last_k2 = cur_k2; 
    last_k3 = cur_k3; last_k4 = cur_k4;

    if (key_down_pressed || key_enter_pressed || key_up_pressed || key_back_pressed) {
        ui_dirty = true;
    }
}

// 核心逻辑控制器：状态跳转与参数修改
void TftMenu::process_logic() {
    // 高频刷新监控页面
    if (current_page == MenuPage::DASHBOARD || current_page == MenuPage::ODOMETRY_DATA) {
        ui_dirty = true;
    }

    if (!ui_dirty) return;

    switch (current_page) {
        case MenuPage::MAIN_MENU:
            if (key_down_pressed) cursor_idx = (cursor_idx + 1) % 7;    // 7个主菜单项
            if (key_up_pressed)   cursor_idx = (cursor_idx == 0) ? 6 : cursor_idx - 1;

            if (key_enter_pressed) {
                if (cursor_idx == 0) current_page = MenuPage::DASHBOARD;
                if (cursor_idx == 1) current_page = MenuPage::ODOMETRY_DATA;
                if (cursor_idx == 2) { current_page = MenuPage::TUNE_PARAMS; cursor_idx = 0; scroll_offset = 0; }
                // --- 运行模式切换 ---
                if (cursor_idx == 3) {
                    is_debug_mode = !is_debug_mode;
                    tft180_show_string(14 * UI_COL_W, 5 * UI_ROW_H, "[SWAP]");
                    system_delay_ms(300);
                }
                // --- Flash 存储触发 ---
                if (cursor_idx == 4) {
                    Storage::save_params();
                    // 在屏幕右侧打个 [OK] 提示，延时 300ms 让人眼能看清
                    tft180_show_string(15 * UI_COL_W, 6 * UI_ROW_H, "[OK]");
                    system_delay_ms(300);
                }
                if (cursor_idx == 5) {
                    Storage::load_params();
                    tft180_show_string(15 * UI_COL_W, 7 * UI_ROW_H, "[OK]");
                    system_delay_ms(300);
                }
                // --- 息屏模式 ---
                if (cursor_idx == 6) {
                    is_closed = true;
                    tft180_full(RGB565_WHITE); 
                    return; 
                }

                // 页面跳转后触发重绘
                if (cursor_idx < 3) { cursor_idx = 0; need_full_redraw = true;} 
                else { need_full_redraw = true;}
            }
            break;

        case MenuPage::TUNE_PARAMS:
            if (!is_editing) {
                // 列表上下移动
                if (key_down_pressed) cursor_idx = (cursor_idx + 1) % DICT_SIZE;
                if (key_up_pressed)   cursor_idx = (cursor_idx == 0) ? (DICT_SIZE - 1) : cursor_idx - 1;
                
                // 自动计算滚动窗口，保证光标始终在屏幕内
                if (cursor_idx < scroll_offset) scroll_offset = cursor_idx;
                if (cursor_idx >= scroll_offset + PARAMS_PER_PAGE) scroll_offset = cursor_idx - PARAMS_PER_PAGE + 1;

                if (key_enter_pressed) is_editing = true;
                if (key_back_pressed) { current_page = MenuPage::MAIN_MENU; cursor_idx = 3; need_full_redraw = true; }
            } else {
                // 编辑模式：直接操作字典中的指针，一键修改全局黑板变量
                if (key_up_pressed)   *(tune_dict[cursor_idx].val_ptr) += tune_dict[cursor_idx].step;
                if (key_down_pressed) *(tune_dict[cursor_idx].val_ptr) -= tune_dict[cursor_idx].step;
                if (key_enter_pressed || key_back_pressed) is_editing = false;
            }
            break;

        case MenuPage::DASHBOARD:
            // 按确认键注入本地地图
            if (key_enter_pressed) {
                current_page = MenuPage::MAP_SELECT;
                map_cursor_idx = 0;       // 光标复位
                map_scroll_offset = 0;
                need_full_redraw = true;  
            }
            if (key_back_pressed) { 
                current_page = MenuPage::MAIN_MENU; 
                need_full_redraw = true; 
            }
            break;

        case MenuPage::MAP_SELECT:
            if (key_down_pressed) map_cursor_idx = (map_cursor_idx + 1) % TestMap::get_mock_map_count();
            if (key_up_pressed)   map_cursor_idx = (map_cursor_idx == 0) ? (TestMap::get_mock_map_count() - 1) : map_cursor_idx - 1;
            
            // 滚动窗口保护计算
            if (map_cursor_idx < map_scroll_offset) map_scroll_offset = map_cursor_idx;
            if (map_cursor_idx >= map_scroll_offset + PARAMS_PER_PAGE) map_scroll_offset = map_cursor_idx - PARAMS_PER_PAGE + 1;

            // 确认选择：加载对应的地图并跳回 VISION_DATA 监视结果
            if (key_enter_pressed) {
                debug_manager.set_phase(GamePhase::WAIT_FOR_VISION); // 直接跳过视觉模块，进入寻路阶段
                TestMap::load_mock_map(map_cursor_idx); // 载入选中的地图
                
                // 屏幕中间打个提示框
                tft180_full(RGB565_WHITE);
                tft180_show_string(10, 80, "[ Map Loaded ]");
                system_delay_ms(200);
                
                current_page = MenuPage::DASHBOARD; // 自动跳回视图层看地图
                need_full_redraw = true;
            }
            // 按返回键：取消选择，退回 Vision 层
            if (key_back_pressed) { 
                current_page = MenuPage::DASHBOARD; 
                need_full_redraw = true; 
            }
            break;

        default:
            if (key_back_pressed) { current_page = MenuPage::MAIN_MENU; need_full_redraw = true; }
            break;
    }
}



// ===================================================== UI 渲染 =====================================================

// UI 渲染器：根据 current_page 调用对应的绘制函数
void TftMenu::render_ui() {
    if (!ui_dirty) return;
    if (need_full_redraw) { 
        tft180_full(RGB565_WHITE); 
        debug_manager.force_bg_redraw = true;
        need_full_redraw = false; 
    }

    switch (current_page) {
        case MenuPage::MAIN_MENU:      draw_main_menu(); break;
        case MenuPage::DASHBOARD:      draw_dashboard(); break;
        case MenuPage::ODOMETRY_DATA:  draw_odometry_data(); break;
        case MenuPage::TUNE_PARAMS:    draw_tune_params(); break;
        case MenuPage::MAP_SELECT:     draw_map_select(); break;
    }
    ui_dirty = false; 
}

// 绘制主菜单
void TftMenu::draw_main_menu() {
    tft180_show_string(0, 0, "-- COMMAND MENU --");
    draw_item(2, "Dashboard",  cursor_idx == 0);
    draw_item(3, "Odometry",   cursor_idx == 1);
    draw_item(4, "Tuning",     cursor_idx == 2);
    draw_item(5, is_debug_mode ? "Mode: [DEBUG]" : "Mode: [PROD ]", cursor_idx == 3);
    draw_item(6, "Save Config",cursor_idx == 4);
    draw_item(7, "Load Config",cursor_idx == 5);
    draw_item(8, "Close Menu", cursor_idx == 6); 
}

void TftMenu::draw_map_select() {
    tft180_show_string(0, 0, "-- SELECT MAP --");
    
    uint8_t count = TestMap::get_mock_map_count();

    // 进度提示 (例如 1/3)，放在右上角
    tft180_show_int(14 * UI_COL_W, 0, map_cursor_idx + 1, 2);
    tft180_show_string(16 * UI_COL_W, 0, "/");
    tft180_show_int(17 * UI_COL_W, 0, count, 2);
    
    // 利用已有的 PARAMS_PER_PAGE 行数限制渲染滚动列表
    for (int i = 0; i < PARAMS_PER_PAGE; i++) {
        int item_idx = map_scroll_offset + i;
        if (item_idx >= count) {
            // 清理多余行，打印 21 个空格覆盖一整行
            tft180_show_string(0, (i + 1) * UI_ROW_H, "                     ");
            continue;
        }

        draw_item(i + 1, TestMap::get_mock_map_name(item_idx), map_cursor_idx == item_idx);
    }
}

// 绘制参数调节页面
void TftMenu::draw_tune_params() {
    tft180_show_string(0, 0, "PARAMETERS");
    
    // 进度提示 (例如 1/8)，放在右上角
    tft180_show_int(14 * UI_COL_W, 0, cursor_idx + 1, 2);
    tft180_show_string(16 * UI_COL_W, 0, "/");
    tft180_show_int(17 * UI_COL_W, 0, DICT_SIZE, 2);
    
    for (int i = 0; i < PARAMS_PER_PAGE; i++) {
        int item_idx = scroll_offset + i;
        if (item_idx >= DICT_SIZE) {
            // 清理多余行，打印 21 个空格正好覆盖一整行
            tft180_show_string(0, (i + 1) * UI_ROW_H, "                     ");
            continue;
        }

        draw_float_item(i + 1, 
            tune_dict[item_idx].name, 
            *(tune_dict[item_idx].val_ptr), 
            cursor_idx == item_idx, 
            is_editing && (cursor_idx == item_idx));
    }
}


// // 融合后的仪表盘绘制：集比赛状态、规划耗时、动态地图于一体
// void TftMenu::draw_dashboard() {
//     GamePhase phase = debug_manager.get_phase();
//     const DemoState& demo = debug_manager.get_demo_state();

//     // ========================================================================
//     // Row 0: 顶部状态展示
//     // ========================================================================
//     tft180_show_string(0, 0, "                    "); // 清行
//     if (is_debug_mode) {
//         // Debug 模式的简短提示
//         if (phase <= GamePhase::WAIT_FOR_VISION)           tft180_show_string(0, 0, "Phase: WAITING MAP");
//         else if (phase == GamePhase::PLAN_PATROL)          tft180_show_string(0, 0, "Phase: PLAN PATROL");
//         else if (phase == GamePhase::ANIMATE_PATROL_DEMO)  tft180_show_string(0, 0, "Phase: DEMO PATROL");
//         else if (phase == GamePhase::BIND_SEMANTICS)       tft180_show_string(0, 0, "Phase: BINDING... ");
//         else if (phase == GamePhase::PLAN_SOKOBAN)         tft180_show_string(0, 0, "Phase: PLAN SOKO  ");
//         else if (phase == GamePhase::ANIMATE_DEMO)         tft180_show_string(0, 0, "Phase: DEMO PUSH  ");
//         else if (phase == GamePhase::FINISHED)             tft180_show_string(0, 0, "Phase: FINISHED   ");
//         else if (phase == GamePhase::ERROR_OCCURRED)       tft180_show_string(0, 0, "Phase: ERROR      ");
//     } else {
//         // Prod 模式的详尽物理机状态
//         tft180_show_string(0, 0, "P: ");
//         switch(phase) {
//             case GamePhase::INIT_CALIBRATE:        tft180_show_string(3 * UI_COL_W, 0, "INIT      "); break;
//             case GamePhase::EXIT_START_ZONE:       tft180_show_string(3 * UI_COL_W, 0, "EXIT_ZONE "); break;
//             case GamePhase::WAIT_FOR_VISION:       tft180_show_string(3 * UI_COL_W, 0, "WAIT_ART1 "); break;
//             case GamePhase::EXEC_ACTION_DISPATCH:  tft180_show_string(3 * UI_COL_W, 0, "ACT_DISP  "); break;
//             case GamePhase::EXEC_PATROL_MOVE:      tft180_show_string(3 * UI_COL_W, 0, "MOVE_PTRL "); break;
//             case GamePhase::EXEC_ALIGN_YAW:        tft180_show_string(3 * UI_COL_W, 0, "ALIGN_YAW "); break;
//             case GamePhase::WAIT_ART2_CAPTURE_ACK: tft180_show_string(3 * UI_COL_W, 0, "WAIT_ART2 "); break;
//             case GamePhase::EXEC_BOMB_PUSH:        tft180_show_string(3 * UI_COL_W, 0, "PUSH_BOMB "); break;
//             case GamePhase::EXEC_SOKOBAN:          tft180_show_string(3 * UI_COL_W, 0, "TRACKING  "); break;
//             case GamePhase::FINISHED:              tft180_show_string(3 * UI_COL_W, 0, "FINISHED  "); break;
//             case GamePhase::ERROR_OCCURRED:        tft180_show_string(3 * UI_COL_W, 0, "ERROR     "); break;
//             default:                               tft180_show_string(3 * UI_COL_W, 0, "COMPUTING "); break;
//         }
//     }

//     // ========================================================================
//     // Row 1: 比赛阶段 Stage
//     // ========================================================================
//     tft180_show_string(0, 1 * UI_ROW_H, "Stage: ");
//     tft180_show_int(7 * UI_COL_W, 1 * UI_ROW_H, debug_manager.get_stage(), 1); 

//     // ========================================================================
//     // Row 2: 耗时展示 (根据模式区分)
//     // ========================================================================
//     if (is_debug_mode) {
//         if (phase == GamePhase::ANIMATE_PATROL_DEMO || phase == GamePhase::BIND_SEMANTICS || phase == GamePhase::PLAN_SOKOBAN) {
//             tft180_show_string(0, 2 * UI_ROW_H, "Bm:    ms GT:    ms");
//             tft180_show_int(3 * UI_COL_W,  2 * UI_ROW_H, debug_manager.get_bomb_plan_time_ms(), 3);
//             tft180_show_int(13 * UI_COL_W, 2 * UI_ROW_H, debug_manager.get_patrol_plan_time_ms(), 3);
//         } else if (phase == GamePhase::ANIMATE_DEMO || phase == GamePhase::FINISHED) {
//             tft180_show_string(0, 2 * UI_ROW_H, "IDA* Time:       ms"); 
//             tft180_show_string(11 * UI_COL_W, 2 * UI_ROW_H, "    "); 
//             tft180_show_int(11 * UI_COL_W, 2 * UI_ROW_H, debug_manager.get_push_plan_time_ms(), 4);
//         } else {
//             tft180_show_string(0, 2 * UI_ROW_H, "Plan Time: --    ms");
//         }
//     } else {
//         // Prod 模式固定显示
//         tft180_show_string(0, 2 * UI_ROW_H, "Plan Time: --    ms");
//     }

//     // 地图未就绪拦截
//     bool is_anim_patrol = is_debug_mode && (phase == GamePhase::ANIMATE_PATROL_DEMO);
//     bool is_anim_push   = is_debug_mode && (phase == GamePhase::ANIMATE_DEMO);
//     bool is_animating   = is_anim_patrol || is_anim_push;

//     if (!vision_data.art1_map_ready && vision_data.box_count == 0 && !is_animating) return;

//     // ========================================================================
//     // Map Rendering
//     // ========================================================================
//     int map_start_y = 3 * UI_ROW_H + 4;  // 地图起始于 y = 34

//     const SokobanLevel& level = debug_manager.get_logical_level();

//     // --- 动态地图渲染 ---
//     if (debug_manager.force_bg_redraw) {
//         for (int map_x = 0; map_x < MAP_MAX_WIDTH; ++map_x) {
//             for (int map_y = 0; map_y < MAP_MAX_HEIGHT; ++map_y) {
//                 int screen_x = map_y * 8;
//                 int screen_y = map_x * 8 + map_start_y;
//                 // Debug 模式画被炸毁的地图，Prod 模式画原生视觉地图
//                 bool is_wall = is_debug_mode ? (demo.map_state.map[map_y][map_x] == 1) : (vision_data.map[map_y][map_x] == 1);
//                 fill_rect(screen_x+1, screen_y+1, 7, 7, is_wall ? RGB565_GRAY : RGB565_WHITE); 
//             }
//         }
//         // 网格线 (每8像素一条)，覆盖在地图上，增强视觉效果
//         for (int i = 0; i <= 12; ++i) tft180_draw_line(0, i * 8 + map_start_y, 127, i * 8 + map_start_y, RGB565_BLACK); 
//         for (int i = 1; i <= 15; ++i) tft180_draw_line(i * 8, map_start_y, i * 8,  96 + map_start_y, RGB565_BLACK); 
//         debug_manager.force_bg_redraw = false; 
//     }

//     // --- 数据源动态绑定 ---
//     const point* current_boxes = is_anim_push ? demo.boxes : level.boxes;
//     uint8_t current_box_count  = is_anim_push ? demo.box_count : level.box_count;
//     const point* current_bombs = is_animating ? demo.bombs : level.bombs;
//     uint8_t current_bomb_count = is_animating ? demo.bomb_count : level.bomb_count;
//     const point* current_tgts  = is_anim_push ? demo.targets : level.targets;
//     uint8_t current_tgt_count  = is_anim_push ? demo.target_count : level.target_count;

//     // --- 擦除逻辑：只擦除上一次出现但这次消失的元素，避免全图刷新导致的闪烁 ---
//     static point last_boxes[SystemConfig::MAX_BOXES];  
//     static point last_bombs[SystemConfig::MAX_BOMBS];
//     static bool init_done = false;
//     if (!init_done) {
//         for(int i=0; i<SystemConfig::MAX_BOXES; ++i) last_boxes[i] = {-1, -1};
//         for(int i=0; i<SystemConfig::MAX_BOMBS; ++i) last_bombs[i] = {-1, -1};
//         init_done = true;
//     }

//     auto erase_ghost = [&](point* last_arr, const point* curr_arr, uint8_t curr_count, int max_size) {
//         for (int i = 0; i < max_size; ++i) {
//             if (last_arr[i].x != -1) {
//                 bool moved_or_vanished = true;
//                 for (int j = 0; j < curr_count; ++j) {
//                     if (last_arr[i] == curr_arr[j]) { moved_or_vanished = false; break; }
//                 }
//                 if (moved_or_vanished) {
//                     fill_rect(last_arr[i].y * 8 + 1, map_start_y + last_arr[i].x * 8 + 1, 6, 6, RGB565_WHITE);
//                     last_arr[i] = {-1, -1}; // BUG FIX: 擦除后立刻标记为空，防止重复擦除导致花屏
//                 }
//             }
//         }
//     };
//     erase_ghost(last_boxes, current_boxes, current_box_count, SystemConfig::MAX_BOXES);
//     erase_ghost(last_bombs, current_bombs, current_bomb_count, SystemConfig::MAX_BOMBS);

//     // --- 绘制彩色炸弹匹配边框 ---
//     const auto& bomb_tasks = debug_manager.get_cached_bomb_tasks();
//     uint16_t b_colors[] = {RGB565_RED, RGB565_BLUE, RGB565_CYAN, RGB565_MAGENTA}; 

//     for (int i = 0; i < bomb_tasks.size(); ++i) {
//         uint16_t color = b_colors[i % 4];
//         point bs = bomb_tasks[i].bomb_start;
//         point tw = bomb_tasks[i].target_wall;

//         // 炸弹起点边框
//         int bs_sx = bs.y * 8, bs_sy = map_start_y + bs.x * 8;
//         tft180_draw_line(bs_sx, bs_sy, bs_sx + 7, bs_sy, color);
//         tft180_draw_line(bs_sx, bs_sy + 7, bs_sx + 7, bs_sy + 7, color);
//         tft180_draw_line(bs_sx, bs_sy, bs_sx, bs_sy + 7, color);
//         tft180_draw_line(bs_sx + 7, bs_sy, bs_sx + 7, bs_sy + 7, color);

//         // 墙壁目标边框
//         if (demo.map_state.map[tw.y][tw.x] == 1) {
//             int tw_sx = tw.y * 8, tw_sy = map_start_y + tw.x * 8;
//             tft180_draw_line(tw_sx, tw_sy, tw_sx + 7, tw_sy, color);
//             tft180_draw_line(tw_sx, tw_sy + 7, tw_sx + 7, tw_sy + 7, color);
//             tft180_draw_line(tw_sx, tw_sy, tw_sx, tw_sy + 7, color);
//             tft180_draw_line(tw_sx + 7, tw_sy, tw_sx + 7, tw_sy + 7, color);
//         }
//     }

//     // --- 绘制底层实体要素 [目标点(紫色) / 箱子(黄色) / 炸弹(黑+红心)] ---
//     for (int i = 0; i < current_tgt_count; ++i) {
//         fill_rect(current_tgts[i].y * 8 + 1, map_start_y + current_tgts[i].x * 8 + 1, 6, 6, RGB565_PURPLE);
//     }
//     for (int i = 0; i < current_box_count; ++i) {
//         last_boxes[i] = current_boxes[i]; 
//         fill_rect(current_boxes[i].y * 8 + 1, map_start_y + current_boxes[i].x * 8 + 1, 6, 6, RGB565_YELLOW);
//     }
//     for (int i = 0; i < current_bomb_count; ++i) {
//         if (current_bombs[i].x != -1) {
//             last_bombs[i] = current_bombs[i]; 
//             int sx = current_bombs[i].y * 8 + 1, sy = map_start_y + current_bombs[i].x * 8 + 1;
//             fill_rect(sx, sy, 6, 6, RGB565_BLACK); 
//             fill_rect(sx + 2, sy + 2, 2, 2, RGB565_RED); 
//         }
//     }

//     // --- PROD 与 DEBUG 模式的轨迹与观测点自适应渲染 ---
//     if (phase >= GamePhase::PLAN_PATROL) {
//         const auto& actions = debug_manager.get_patrol_actions();
        
//         // 绘制尚未到达的观测点大叉
//         // Prod 模式从 GameManager 获取真实进度，动画模式从 Demo 状态机获取虚拟进度
//         uint8_t cur_action_idx = is_animating ? demo.patrol_target_idx : debug_manager.get_action_idx(); 
        
//         for (size_t i = cur_action_idx; i < actions.size(); ++i) {
//             if (!actions[i].is_bomb_task) {
//                 int sx = actions[i].obs.pos.y * 8; 
//                 int sy = map_start_y + actions[i].obs.pos.x * 8;
//                 tft180_draw_line(sx + 2, sy + 2, sx + 6, sy + 6, RGB565_BLUE);
//                 tft180_draw_line(sx + 2, sy + 6, sx + 6, sy + 2, RGB565_BLUE);
//             }
//         }

//         // 绘制底层 Tracker 运行的网格路径豆子
//         if (is_animating) {
//             // 动画模式下，从虚拟内存读取
//             if (is_anim_patrol) {
//                 for (size_t i = demo.segment_idx; i < demo.segment_path.size(); ++i) 
//                     fill_rect(demo.segment_path[i].y * 8 + 3, map_start_y + demo.segment_path[i].x * 8 + 3, 2, 2, RGB565_BLUE);
//             } else if (is_anim_push) {
//                 const auto& push_path = solver.get_result_path();
//                 for (size_t i = demo.path_idx; i < push_path.size(); ++i) 
//                     fill_rect(push_path[i].y * 8 + 3, map_start_y + push_path[i].x * 8 + 3, 2, 2, RGB565_BLUE);
//             }
//         } else {
//             // PROD 模式下，直接抓取 Tracker 内部正在跟踪的坐标数据
//             if (phase == GamePhase::EXEC_PATROL_MOVE || phase == GamePhase::EXEC_BOMB_PUSH) {
//                 const auto& current_path = path_tracker.grid_path; 
//                 for (size_t i = 0; i < current_path.size(); ++i) {
//                     fill_rect(current_path[i].y * 8 + 3, map_start_y + current_path[i].x * 8 + 3, 2, 2, RGB565_BLUE);
//                 }
//             } else if (phase == GamePhase::EXEC_SOKOBAN) {
//                 const auto& current_path = solver.get_result_path(); 
//                 for (size_t i = 0; i < current_path.size(); ++i) {
//                     fill_rect(current_path[i].y * 8 + 3, map_start_y + current_path[i].x * 8 + 3, 2, 2, RGB565_BLUE);
//                 }
//             }
//         }
//     }

//     // --- 绘制小车位姿 ---
//     static point last_player_pos = {-1, -1}; // 上一帧小车位置，初始为无效坐标
//     point car_pos = {-1, -1};

//     if (is_debug_mode) {
//         // Debug 模式使用动画逻辑的虚拟小车坐标
//         if (is_animating) car_pos = demo.player;
//     } else {
//         // Prod 模式使用真实底盘里程计坐标并转为网格
//         Point2D real_pos = chassis_odometry.get_position();
//         int8_t grid_x = static_cast<int8_t>(std::round((real_pos.x - MAP_OFFSET_X) / GRID_SIZE_CM));
//         int8_t grid_y = static_cast<int8_t>(std::round((real_pos.y - MAP_OFFSET_Y) / GRID_SIZE_CM));
        
//         // 限幅约束
//         if (grid_x < 0) grid_x = 0; else if (grid_x >= MAP_MAX_WIDTH) grid_x = MAP_MAX_WIDTH - 1;
//         if (grid_y < 0) grid_y = 0; else if (grid_y >= MAP_MAX_HEIGHT) grid_y = MAP_MAX_HEIGHT - 1;
//         car_pos = {grid_x, grid_y};
//     }

//     // 渲染小车绿块并擦除上一帧
//     if (car_pos.x != -1) {
//         if (last_player_pos.x != -1 && !(last_player_pos == car_pos)) {
//             fill_rect(last_player_pos.y * 8 + 2, map_start_y + last_player_pos.x * 8 + 2, 4, 4, RGB565_WHITE);
//         }
//         fill_rect(car_pos.y * 8 + 2, map_start_y + car_pos.x * 8 + 2, 4, 4, RGB565_GREEN);
//         last_player_pos = car_pos;
//     }
// }

// 掩码定义（可放在 display.cpp 顶部）
constexpr uint8_t TL_WALL=1<<0, TL_TGT=1<<1, TL_BOX=1<<2, TL_BOMB=1<<3, TL_PATH=1<<4, TL_CRS=1<<5, TL_CAR=1<<6;
static uint8_t back_buffer[16][12] = {0}; // 显存缓冲，用于防闪烁

void TftMenu::draw_dashboard() {
    // 1. 获取上帝视角的“数据投影”
    RenderContext ctx = debug_manager.get_render_context();

    // ====================================================================
    // 2. 顶部 HUD 渲染 (带缓存的局部刷新，告别闪烁)
    // ====================================================================
    static char last_hud0[22] = {0}, last_hud1[22] = {0}, last_hud2[22] = {0};
    
    if (strncmp(last_hud0, ctx.hud_line0, 22) != 0) {
        tft180_show_string(0, 0, "                     "); // 先用空格擦除
        tft180_show_string(0, 0, ctx.hud_line0);
        strncpy(last_hud0, ctx.hud_line0, 22);
    }
    if (strncmp(last_hud1, ctx.hud_line1, 22) != 0) {
        tft180_show_string(0, 1 * UI_ROW_H, "                     ");
        tft180_show_string(0, 1 * UI_ROW_H, ctx.hud_line1);
        strncpy(last_hud1, ctx.hud_line1, 22);
    }
    if (strncmp(last_hud2, ctx.hud_line2, 22) != 0) {
        tft180_show_string(0, 2 * UI_ROW_H, "                     ");
        tft180_show_string(0, 2 * UI_ROW_H, ctx.hud_line2);
        strncpy(last_hud2, ctx.hud_line2, 22);
    }

    // ====================================================================
    // 3. 在内存中合成 192 字节的语义画布 (无任何 if-else 业务分支！)
    // ====================================================================
    uint8_t canvas[16][12] = {0};
    
    for(int y=0; y<MAP_MAX_HEIGHT; y++) for(int x=0; x<MAP_MAX_WIDTH; x++) if((*ctx.map)[y][x]) canvas[y][x] |= TL_WALL;
    for(int i=0; i<ctx.target_count; i++) if(ctx.targets[i].x != -1) canvas[ctx.targets[i].y][ctx.targets[i].x] |= TL_TGT;
    for(int i=0; i<ctx.box_count; i++)    if(ctx.boxes[i].x != -1)   canvas[ctx.boxes[i].y][ctx.boxes[i].x] |= TL_BOX;
    for(int i=0; i<ctx.bomb_count; i++)   if(ctx.bombs[i].x != -1)   canvas[ctx.bombs[i].y][ctx.bombs[i].x] |= TL_BOMB;
    
    if (ctx.path_ptr) {
        for(size_t i = ctx.path_start_idx; i < ctx.path_ptr->size(); i++) 
            canvas[(*ctx.path_ptr)[i].y][(*ctx.path_ptr)[i].x] |= TL_PATH;
    }
    if (ctx.actions_ptr) {
        for(size_t i = ctx.action_start_idx; i < ctx.actions_ptr->size(); i++) 
            if(!(*ctx.actions_ptr)[i].is_bomb_task) canvas[(*ctx.actions_ptr)[i].obs.pos.y][(*ctx.actions_ptr)[i].obs.pos.x] |= TL_CRS;
    }
    canvas[ctx.player_pos.y][ctx.player_pos.x] |= TL_CAR;

    // ====================================================================
    // 4. O(1) 脏矩形增量渲染 
    // ====================================================================
    int map_start_y = 3 * UI_ROW_H + 4;
    
    if (debug_manager.force_bg_redraw) { 
        memset(back_buffer, 0xFF, sizeof(back_buffer)); // 失效显存，强制全刷
        
        // --- 静态背景上的附加涂装：彩色炸弹框 ---
        if (ctx.bomb_tasks_ptr) {
            uint16_t b_colors[] = {RGB565_RED, RGB565_BLUE, RGB565_CYAN, RGB565_MAGENTA}; 
            for (int i = 0; i < ctx.bomb_tasks_ptr->size(); ++i) {
                uint16_t color = b_colors[i % 4];
                point bs = (*ctx.bomb_tasks_ptr)[i].bomb_start;
                point tw = (*ctx.bomb_tasks_ptr)[i].target_wall;

                int bs_sx = bs.y * 8, bs_sy = map_start_y + bs.x * 8;
                tft180_draw_line(bs_sx, bs_sy, bs_sx + 7, bs_sy, color);
                tft180_draw_line(bs_sx, bs_sy + 7, bs_sx + 7, bs_sy + 7, color);
                tft180_draw_line(bs_sx, bs_sy, bs_sx, bs_sy + 7, color);
                tft180_draw_line(bs_sx + 7, bs_sy, bs_sx + 7, bs_sy + 7, color);

                if ((*ctx.map)[tw.y][tw.x] == 1) { // 墙还没被炸毁的话，画框
                    int tw_sx = tw.y * 8, tw_sy = map_start_y + tw.x * 8;
                    tft180_draw_line(tw_sx, tw_sy, tw_sx + 7, tw_sy, color);
                    tft180_draw_line(tw_sx, tw_sy + 7, tw_sx + 7, tw_sy + 7, color);
                    tft180_draw_line(tw_sx, tw_sy, tw_sx, tw_sy + 7, color);
                    tft180_draw_line(tw_sx + 7, tw_sy, tw_sx + 7, tw_sy + 7, color);
                }
            }
        }
        debug_manager.force_bg_redraw = false; 
    }

    // 核心渲染循环：比较 canvas 与 back_buffer，谁变了就盖谁
    for(int y=0; y<16; y++) {
        for(int x=0; x<12; x++) {
            if (canvas[y][x] != back_buffer[y][x]) {  
                int sx = y * 8, sy = x * 8 + map_start_y;
                
                // 第一层：铺底色 (自带清空残影功能)
                fill_rect(sx + 1, sy + 1, 7, 7, (canvas[y][x] & TL_WALL) ? RGB565_GRAY : RGB565_WHITE);
                
                // 第二层：贴花 (紫框、蓝豆、蓝叉叉)
                if (canvas[y][x] & TL_TGT)  fill_rect(sx + 1, sy + 1, 6, 6, RGB565_PURPLE);
                if (canvas[y][x] & TL_PATH) fill_rect(sx + 3, sy + 3, 2, 2, RGB565_BLUE);
                if (canvas[y][x] & TL_CRS)  { tft180_draw_line(sx+2, sy+2, sx+6, sy+6, RGB565_BLUE); tft180_draw_line(sx+2, sy+6, sx+6, sy+2, RGB565_BLUE); }
                
                // 第三层：3D实体 (箱子、炸弹)
                if (canvas[y][x] & TL_BOX)  fill_rect(sx + 1, sy + 1, 6, 6, RGB565_YELLOW);
                if (canvas[y][x] & TL_BOMB) { fill_rect(sx + 1, sy + 1, 6, 6, RGB565_BLACK); fill_rect(sx + 3, sy + 3, 2, 2, RGB565_RED); }
                
                // 第四层：Actor小车本身 (强制最顶层展示)
                if (canvas[y][x] & TL_CAR)  fill_rect(sx + 2, sy + 2, 4, 4, RGB565_GREEN);

                // 更新显存记录
                back_buffer[y][x] = canvas[y][x]; 
            }
        }
    }
}

// 里程计和硬件监控页面
void TftMenu::draw_odometry_data() {
    tft180_show_string(0, 0, "-- ODO & HW --");
    Point2D pos = chassis_odometry.get_position();
    
    tft180_show_string(0, 2 * UI_ROW_H, "Global X: ");   tft180_show_float(10 * UI_COL_W, 2 * UI_ROW_H, pos.x, 3, 1);
    tft180_show_string(0, 3 * UI_ROW_H, "Global Y: ");   tft180_show_float(10 * UI_COL_W, 3 * UI_ROW_H, pos.y, 3, 1);

    char ui_buf[32]; 
    sprintf(ui_buf, "Yaw: %8.2f   ", imu_sensor.get_yaw());     tft180_show_string(0, 4 * UI_ROW_H, ui_buf);
    sprintf(ui_buf, "Spd: %8.2f   ", imu_sensor.get_gyro_z());  tft180_show_string(0, 5 * UI_ROW_H, ui_buf);

    tft180_show_string(0, 6 * UI_ROW_H, "Spd LF: ");     tft180_show_float(10 * UI_COL_W, 6 * UI_ROW_H, encoders.get_speed_cm_s(0), 3, 1);
    tft180_show_string(0, 7 * UI_ROW_H, "Spd LB: ");     tft180_show_float(10 * UI_COL_W, 7 * UI_ROW_H, encoders.get_speed_cm_s(1), 3, 1);
    tft180_show_string(0, 8 * UI_ROW_H, "Spd RF: ");     tft180_show_float(10 * UI_COL_W, 8 * UI_ROW_H, encoders.get_speed_cm_s(2), 3, 1);
    tft180_show_string(0, 9 * UI_ROW_H, "Spd RB: ");     tft180_show_float(10 * UI_COL_W, 9 * UI_ROW_H, encoders.get_speed_cm_s(3), 3, 1);
}



// ===================================================== 局部刷新辅助函数 =====================================================

void TftMenu::draw_item(uint8_t row, const char* name, bool is_selected) {
    if (is_selected) tft180_show_string(0, row * UI_ROW_H, ">"); 
    else tft180_show_string(0, row * UI_ROW_H, " "); 
    // 名称从第 2 列开始写
    tft180_show_string(1 * UI_COL_W, row * UI_ROW_H, (char*)name);
}

void TftMenu::draw_float_item(uint8_t row, const char* name, float val, bool is_selected, bool is_editing_this) {
    // 渲染光标和名称 (占用 0 ~ 9 列)
    draw_item(row, name, is_selected);
    
    // 渲染编辑标识（与数值区保持不重叠）
    if (is_selected && is_editing_this) {
        tft180_show_string(9 * UI_COL_W, row * UI_ROW_H, "[E]");
    } else {
        tft180_show_string(9 * UI_COL_W, row * UI_ROW_H, "   ");
    }

    // 固定从第 12 列开始显示，较原先左移一列，避免增加位数后越界。
    tft180_show_float(12 * UI_COL_W,
                     row * UI_ROW_H,
                     val,
                     5,
                     2);
}


// ===================================================== 基础绘图辅助函数 =====================================================

void TftMenu::fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color) {
    for (uint8_t i = 0; i < h; ++i) {
        tft180_draw_line(x, y + i, x + w - 1, y + i, color);
    }
}
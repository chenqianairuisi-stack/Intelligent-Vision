#include "display.h"
#include "task_control.h"
#include "tuning_config.h"
#include "game_manage.h"
#include "odometry.h"
#include "encoder.h"
#include "imu_process.h"
#include "storage.h"
#include "telemetry.h"


// ===================================================== 初始化与全局变量 =====================================================

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
    is_editing = false;
    need_full_redraw = true;  
    ui_dirty = true;           
    last_k1 = true; last_k2 = true; last_k3 = true; last_k4 = true;

    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);

    tft180_set_dir(TFT180_PORTAIT);                  // 竖屏
    tft180_set_font(TFT180_6X8_FONT);                // 设置字库
    tft180_set_color(RGB565_BLACK, RGB565_WHITE);     // 白底黑字
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

    // 只有在按键从 高电平(true) 变到 低电平(false) 的瞬间，才触发一次 true
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
    // 如果是监控页面，哪怕没有按键，也要刷新数据
    if (current_page == MenuPage::GAME_STATUS ||
        current_page == MenuPage::VISION_DATA ||
        current_page == MenuPage::ODOMETRY_DATA) {
        ui_dirty = true;
    }

    if (!ui_dirty) return;

    switch (current_page) {
        case MenuPage::MAIN_MENU:
            if (key_down_pressed) cursor_idx = (cursor_idx + 1) % 7;    // 7个主菜单项
            if (key_up_pressed)   cursor_idx = (cursor_idx == 0) ? 6 : cursor_idx - 1;

            if (key_enter_pressed) {
                if (cursor_idx == 0) current_page = MenuPage::GAME_STATUS;
                if (cursor_idx == 1) current_page = MenuPage::ODOMETRY_DATA;
                if (cursor_idx == 2) current_page = MenuPage::VISION_DATA;
                if (cursor_idx == 3) { current_page = MenuPage::TUNE_PARAMS; cursor_idx = 0; scroll_offset = 0; }
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
                if (cursor_idx == 6) {
                    is_closed = true;
                    tft180_full(RGB565_WHITE); 
                    return; 
                }

                // 前4个是页面跳转，需要重置光标和清屏
                if (cursor_idx < 4) {
                    cursor_idx = 0; need_full_redraw = true;
                } else {
                    // 第5和第6项执行完只需局部刷新把 [OK] 擦掉
                    need_full_redraw = true;
                }
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

        case MenuPage::VISION_DATA:
            // 按确认键注入本地地图
            if (key_enter_pressed) {
                debug_manager.set_phase(GamePhase::WAIT_FOR_VISION);  // 直接跳过视觉模块，进入寻路阶段，方便调试 UI 和算法
                vision_manager.load_mock_map();
                // 在屏幕中间打个提示，延时防抖
                tft180_show_string(12, 80, "Local Map Loaded");
                system_delay_ms(300);
                need_full_redraw = true;   
            }
            if (key_back_pressed) { 
                current_page = MenuPage::MAIN_MENU; 
                need_full_redraw = true; 
            }
            break;

        default:
            if (key_back_pressed) { current_page = MenuPage::MAIN_MENU; need_full_redraw = true; }
            break;
    }
}

// 硬件报错挂起
void TftMenu::halt_with_error(const char* err_msg) {
    tft180_full(RGB565_WHITE);
    tft180_show_string(0, 50, "!!! FATAL ERROR !!!");
    tft180_show_string(0, 70, (char*)err_msg);
    
    // 彻底死循环，避免主循环里的 sys_menu.run() 覆盖屏幕
    while (1) {
        // 可以在这里加个蜂鸣器报警代码
        system_delay_ms(100);
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
        case MenuPage::GAME_STATUS:    draw_game_status(); break;
        case MenuPage::ODOMETRY_DATA:  draw_odometry_data(); break;
        case MenuPage::VISION_DATA:    draw_vision_data(); break;
        case MenuPage::TUNE_PARAMS:    draw_tune_params(); break;
    }
    ui_dirty = false; 
}

// 绘制主菜单
void TftMenu::draw_main_menu() {
    tft180_show_string(0, 0, "-- COMMAND MENU --");
    draw_item(2, "Game State", cursor_idx == 0);
    draw_item(3, "Odometry",   cursor_idx == 1);
    draw_item(4, "Vision",     cursor_idx == 2);
    draw_item(5, "Tuning",     cursor_idx == 3);
    draw_item(6, "Save Config",cursor_idx == 4);
    draw_item(7, "Load Config",cursor_idx == 5);
    draw_item(8, "Close Menu", cursor_idx == 6); 
}

// 绘制参数调节页面
void TftMenu::draw_tune_params() {
    tft180_show_string(0, 0, "PARAMETERS");
    
    // 进度提示 (例如 1/8)，放在右上角第 15 列开始
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

// 绘制比赛状态监控页面
void TftMenu::draw_game_status() {
    tft180_show_string(0, 0, "-- GAME STATUS --");
    tft180_show_string(0, 2 * UI_ROW_H, "Phase: ");
    switch(game_manager.get_phase()) {
        case GamePhase::INIT_CALIBRATE:        tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "INIT_CALIB"); break;
        case GamePhase::EXIT_START_ZONE:       tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "EXIT_ZONE "); break;
        case GamePhase::WAIT_FOR_VISION:       tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "WAIT_VIS  "); break;
        case GamePhase::PLAN_PATROL:           tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "PLAN_PTRL "); break;
        case GamePhase::EXEC_ACTION_DISPATCH:  tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "ACT_DISP  "); break;
        case GamePhase::EXEC_PATROL_MOVE:      tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "MOVE_PTRL "); break;
        case GamePhase::EXEC_ALIGN_YAW:        tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "ALIGN_YAW "); break;
        case GamePhase::WAIT_ART2_CAPTURE_ACK: tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "WAIT_ART2 "); break;
        case GamePhase::EXEC_BOMB_PUSH:        tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "PUSH_BOMB "); break;
        case GamePhase::UPDATE_MAP:            tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "UPDATE_MAP"); break;
        case GamePhase::BIND_SEMANTICS:        tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "BIND_SEM  "); break;
        case GamePhase::PLAN_SOKOBAN:          tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "PLANNING  "); break;
        case GamePhase::EXEC_SOKOBAN:          tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "TRACKING  "); break;
        case GamePhase::FINISHED:              tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "FINISHED  "); break;
        case GamePhase::ERROR_OCCURRED:        tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "ERROR     "); break;
        case GamePhase::ANIMATE_PATROL_DEMO:   tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "DEMO_PTRL "); break;
        case GamePhase::ANIMATE_DEMO:          tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "DEMO_PUSH "); break;
        default:                               tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "UNKNOWN   "); break;
    }

    tft180_show_string(0, 4 * UI_ROW_H, "GAME STAGE:");
    tft180_show_int(16 * UI_COL_W, 4 * UI_ROW_H, game_manager.get_stage(), 1); 
    tft180_show_string(0, 6 * UI_ROW_H, "DEBUG STAGE:");
    tft180_show_int(16 * UI_COL_W, 6 * UI_ROW_H, debug_manager.get_stage(), 1); 
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


// 视觉数据监控页面：地图格子、箱子、目标、炸弹、小车位置等的实时绘制
void TftMenu::draw_vision_data() {
    
    GamePhase current_phase = debug_manager.get_phase();  
    bool is_animating = (current_phase == GamePhase::ANIMATE_PATROL_DEMO || current_phase == GamePhase::ANIMATE_DEMO);  
    const DemoState& demo = debug_manager.get_demo_state();  // 获取演示状态机数据，里面有动画模式下的箱子和目标位置

    // ========================================================================
    // Step 1: 顶部状态栏与规划时间展示
    // ========================================================================
    tft180_show_string(0, 0, "                     "); 
    if (current_phase <= GamePhase::WAIT_FOR_VISION)           tft180_show_string(0, 0, "Phase: WAITING MAP");
    else if (current_phase == GamePhase::PLAN_PATROL)          tft180_show_string(0, 0, "Phase: PLAN PATROL");
    else if (current_phase == GamePhase::ANIMATE_PATROL_DEMO)  tft180_show_string(0, 0, "Phase: DEMO PATROL");
    else if (current_phase == GamePhase::BIND_SEMANTICS)       tft180_show_string(0, 0, "Phase: BINDING... ");
    else if (current_phase == GamePhase::PLAN_SOKOBAN)         tft180_show_string(0, 0, "Phase: PLAN SOKO  ");
    else if (current_phase == GamePhase::ANIMATE_DEMO)         tft180_show_string(0, 0, "Phase: DEMO PUSH  ");
    else if (current_phase >= GamePhase::FINISHED)             tft180_show_string(0, 0, "Phase: FINISHED   ");

    // 耗时分类展示：巡图阶段展示 [炸弹分配] + [3D GTSP] 的耗时
    if (current_phase == GamePhase::ANIMATE_PATROL_DEMO || current_phase == GamePhase::BIND_SEMANTICS ||  current_phase == GamePhase::PLAN_SOKOBAN) {
        tft180_show_string(0, 1 * UI_ROW_H, "Bm:    ms GT:    ms");
        tft180_show_int(3 * UI_COL_W,  1 * UI_ROW_H, debug_manager.get_bomb_plan_time_ms(), 3);
        tft180_show_int(13 * UI_COL_W, 1 * UI_ROW_H, debug_manager.get_patrol_plan_time_ms(), 3);
    } else if (current_phase == GamePhase::ANIMATE_DEMO || current_phase == GamePhase::FINISHED) {
        tft180_show_string(0, 1 * UI_ROW_H, "IDA* Time:       ms"); 
        tft180_show_string(11 * UI_COL_W, 1 * UI_ROW_H, "    "); 
        tft180_show_int(11 * UI_COL_W, 1 * UI_ROW_H, debug_manager.get_push_plan_time_ms(), 4);
    } else {
        tft180_show_string(0, 1 * UI_ROW_H, "Plan Time: --  ms");
    }

    if (!vision_data.art1_map_ready && vision_data.box_count == 0 && !is_animating) return;   // 地图未就绪，退出


    // ========================================================================
    // Step 2: 动态背景绘制 (支持炸弹毁墙重绘)
    // ========================================================================
    int map_start_y = 2 * UI_ROW_H + 4;

    if (debug_manager.force_bg_redraw) {

        // 根据 demo 状态机里实时被炸弹破坏的地图来画墙壁
        for (int map_x = 0; map_x < MAP_MAX_WIDTH; ++map_x) {
            for (int map_y = 0; map_y < MAP_MAX_HEIGHT; ++map_y) {
                int screen_x = map_y * 8;
                int screen_y = map_x * 8 + map_start_y;
                uint16_t color = (demo.map_state.map[map_y][map_x] == 1) ? RGB565_GRAY : RGB565_WHITE;
                fill_rect(screen_x+1, screen_y+1, 7, 7, color); 
            }
        }

        // 画网格线
        for (int i = 0; i <= 12; ++i) tft180_draw_line(0, i * 8 + map_start_y, 127, i * 8 + map_start_y, RGB565_BLACK); 
        for (int i = 1; i <= 15; ++i)  tft180_draw_line(i * 8, map_start_y, i * 8,  96 + map_start_y, RGB565_BLACK); 

        debug_manager.force_bg_redraw = false; 
    }

    // ========================================================================
    // Step 3: 彩色炸弹匹配边框
    // ========================================================================
    const auto& bomb_tasks = debug_manager.get_cached_bomb_tasks();
    uint16_t b_colors[] = {RGB565_RED, RGB565_BLUE, RGB565_CYAN, RGB565_MAGENTA}; 

    for (int i = 0; i < bomb_tasks.size(); ++i) {
        uint16_t color = b_colors[i % 4];
        point bs = bomb_tasks[i].bomb_start;
        point tw = bomb_tasks[i].target_wall;

        // 炸弹起点边框 (严格遵循 screen_x = map.y, screen_y = map.x)
        int bs_sx = bs.y * 8, bs_sy = map_start_y + bs.x * 8;
        tft180_draw_line(bs_sx, bs_sy, bs_sx + 7, bs_sy, color);
        tft180_draw_line(bs_sx, bs_sy + 7, bs_sx + 7, bs_sy + 7, color);
        tft180_draw_line(bs_sx, bs_sy, bs_sx, bs_sy + 7, color);
        tft180_draw_line(bs_sx + 7, bs_sy, bs_sx + 7, bs_sy + 7, color);

        // 墙壁目标边框
        if (demo.map_state.map[tw.y][tw.x] == 1) {
            int tw_sx = tw.y * 8, tw_sy = map_start_y + tw.x * 8;
            tft180_draw_line(tw_sx, tw_sy, tw_sx + 7, tw_sy, color);
            tft180_draw_line(tw_sx, tw_sy + 7, tw_sx + 7, tw_sy + 7, color);
            tft180_draw_line(tw_sx, tw_sy, tw_sx, tw_sy + 7, color);
            tft180_draw_line(tw_sx + 7, tw_sy, tw_sx + 7, tw_sy + 7, color);
        }
    }

    // ========================================================================
    // Step 4: 动态图层擦除与重绘
    // ========================================================================
    static point last_player_pos = {-1, -1};  
    static point last_boxes[SystemConfig::MAX_BOXES] = {{-1, -1}};  
    static point last_bombs[SystemConfig::MAX_BOMBS] = {{-1, -1}};

    // 根据不同的物理阶段，切换渲染数据源
    const point* current_boxes = is_animating ? demo.boxes : vision_data.boxes;
    uint8_t current_box_count  = is_animating ? demo.box_count : vision_data.box_count;
    
    const point* current_bombs = is_animating ? demo.bombs : vision_data.bombs;
    uint8_t current_bomb_count = is_animating ? demo.bomb_count : vision_data.bomb_count;

    const point* current_tgts = is_animating ? demo.targets : vision_data.targets;
    uint8_t current_tgt_count = is_animating ? demo.target_count : vision_data.box_count;
    
    auto erase_ghost = [&](point* last_arr, const point* curr_arr, uint8_t curr_count, int max_size) {
        for (int i = 0; i < max_size; ++i) {
            if (last_arr[i].x != -1) {
                bool moved_or_vanished = true;
                for (int j = 0; j < curr_count; ++j) {
                    if (last_arr[i] == curr_arr[j]) { moved_or_vanished = false; break; }
                }
                if (moved_or_vanished) {
                    fill_rect(last_arr[i].y * 8 + 1, map_start_y + last_arr[i].x * 8 + 1, 6, 6, RGB565_WHITE);
                    last_arr[i] = {-1, -1}; // BUG FIX: 擦除后立刻标记为空，防止重复擦除导致花屏
                }
            }
        }
    };
    erase_ghost(last_boxes, current_boxes, current_box_count, SystemConfig::MAX_BOXES);
    erase_ghost(last_bombs, current_bombs, current_bomb_count, SystemConfig::MAX_BOMBS);

    if (is_animating && last_player_pos.x != -1 && !(last_player_pos == demo.player)) {
        fill_rect(last_player_pos.y * 8 + 2, map_start_y + last_player_pos.x * 8 + 2, 4, 4, RGB565_WHITE);
    }

    // 绘制固定目标点 (紫色)
    for (int i = 0; i < current_tgt_count; ++i) {
        fill_rect(current_tgts[i].y * 8 + 1, map_start_y + current_tgts[i].x * 8 + 1, 6, 6, RGB565_PURPLE);
    }

    // 绘制箱子 (黄色)
    for (int i = 0; i < current_box_count; ++i) {
        last_boxes[i] = current_boxes[i];
        fill_rect(current_boxes[i].y * 8 + 1, map_start_y + current_boxes[i].x * 8 + 1, 6, 6, RGB565_YELLOW);
    }

    // 绘制炸弹 (黑底红心)
    for (int i = 0; i < current_bomb_count; ++i) {
        if (current_bombs[i].x != -1) {
            last_bombs[i] = current_bombs[i];
            int sx = current_bombs[i].y * 8 + 1;
            int sy = map_start_y + current_bombs[i].x * 8 + 1;
            fill_rect(sx, sy, 6, 6, RGB565_BLACK);
            fill_rect(sx + 2, sy + 2, 2, 2, RGB565_RED); 
        }
    }
    
    // ========================================================================
    // Step 5: 小车与路径轨迹绘制
    // ========================================================================
    if (is_animating) {
        last_player_pos = demo.player;
        // 画小车
        fill_rect(demo.player.y * 8 + 2, map_start_y + demo.player.x * 8 + 2, 4, 4, RGB565_GREEN);

        if (current_phase == GamePhase::ANIMATE_PATROL_DEMO) {
            const auto& actions = debug_manager.get_patrol_actions();
            
            // 画所有未走过的观测点大叉
            for (size_t i = demo.patrol_target_idx; i < actions.size(); ++i) {
                if (!actions[i].is_bomb_task) {
                    int sx = actions[i].obs.pos.y * 8; 
                    int sy = map_start_y + actions[i].obs.pos.x * 8;
                    tft180_draw_line(sx + 2, sy + 2, sx + 6, sy + 6, RGB565_BLUE);
                    tft180_draw_line(sx + 2, sy + 6, sx + 6, sy + 2, RGB565_BLUE);
                }
            }

            // 画微观寻路网格点 (小豆子)
            for (size_t i = demo.segment_idx; i < demo.segment_path.size(); ++i) {
                fill_rect(demo.segment_path[i].y * 8 + 3, map_start_y + demo.segment_path[i].x * 8 + 3, 2, 2, RGB565_BLUE);
            }

            tft180_show_string(0, 140, "Macro Action: ");
            tft180_show_int(14 * UI_COL_W, 140, demo.patrol_target_idx, 2);
        } 
        else {
            tft180_show_string(0, 140, "IDA* Step:    ");
            tft180_show_int(11 * UI_COL_W, 140, demo.path_idx, 3);
        }
    }
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
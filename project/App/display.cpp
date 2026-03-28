#include "display.h"
#include "task_control.h"
#include "tuning_config.h"
#include "game_manage.h"
#include "odometry.h"
#include "encoder.h"
#include "imu.h"
#include "storage.h"
#include "telemetry.h"

TftMenu sys_menu;

// 128x160 的屏幕，6x8 的字体，留出少量间距后可用 16 行，21 列
static constexpr int UI_COL_W = 6; 
static constexpr int UI_ROW_H = 10;


// ============================== 全局参数字典 ==============================
// 结构：{ "屏幕显示名称",  变量的内存地址,  按键单次加减步长 }
struct ParamItem { const char* name; float* val_ptr; float step; };

static ParamItem tune_dict[] = {
    {"Yaw_Kp",   &tune.pid_yaw.kp,                  0.1f  },
    {"Yaw_Kd",   &tune.pid_yaw.kd,                  0.01f },
    {"Spd_Kp",   &tune.pid_speed.kp,                0.1f  },
    {"Spd_Ki",   &tune.pid_speed.ki,                0.01f },
    {"Max_Spd",  &tune.dynamics.max_speed,          1.0f  },
    {"Max_Acc",  &tune.dynamics.max_acc,            10.0f },
    {"MaxJerk",  &tune.dynamics.max_jerk,           100.0f},
    {"MaxASpd",  &tune.dynamics.max_ang_speed,      0.1f  },
    {"ReachRad", &tune.tracker.reach_radius,        1.0f  },
    {"ReachMin", &tune.tracker.reach_radius_min,    0.5f  },
};

static constexpr int DICT_SIZE = sizeof(tune_dict) / sizeof(tune_dict[0]);     // 自动计算字典大小
static constexpr int PARAMS_PER_PAGE = 12;                                     // 每页显示的参数数量，超过会自动滚动
// ==========================================================================


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


// --------------------- 核心功能函数实现 -------------------------

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
                vision_manager.load_mock_map();
                debug_manager.set_phase(GamePhase::WAIT_FOR_VISION);  // 直接跳过视觉模块，进入寻路阶段，方便调试 UI 和算法
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


// ------------------------- 页面绘制函数 -------------------------

// UI 渲染器：根据 current_page 调用对应的绘制函数
void TftMenu::render_ui() {
    if (!ui_dirty) return;
    if (need_full_redraw) { tft180_full(RGB565_WHITE); need_full_redraw = false; }

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
        case GamePhase::INIT_CALIBRATE:    tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "INIT_CALIB"); break;
        case GamePhase::EXIT_START_ZONE:   tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "EXIT_ZONE "); break;
        case GamePhase::WAIT_FOR_VISION:   tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "WAIT_VIS  "); break;
        case GamePhase::PLAN_SOKOBAN:      tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "PLANNING  "); break;
        case GamePhase::ANIMATE_DEMO:      tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "ANIMATING "); break;
        case GamePhase::EXEC_SOKOBAN:      tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "TRACKING  "); break;
        case GamePhase::FINISHED:          tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "FINISHED  "); break;
    }

    tft180_show_string(0, 6 * UI_ROW_H, "Last RX Cmd:");
    tft180_show_string(0, 7 * UI_ROW_H, last_rx_cmd); // 打印全局变量
}


// 里程计和硬件监控页面
void TftMenu::draw_odometry_data() {
    tft180_show_string(0, 0, "-- ODO & HW --");
    Point2D pos = chassis_odometry.get_position();
    
    tft180_show_string(0, 2 * UI_ROW_H, "Global X: ");   tft180_show_float(10 * UI_COL_W, 2 * UI_ROW_H, pos.x, 3, 1);
    tft180_show_string(0, 3 * UI_ROW_H, "Global Y: ");   tft180_show_float(10 * UI_COL_W, 3 * UI_ROW_H, pos.y, 3, 1);
    tft180_show_string(0, 4 * UI_ROW_H, "Yaw: ");        tft180_show_float(10 * UI_COL_W, 4 * UI_ROW_H, imu_sensor.get_yaw(), 3, 1); 

    tft180_show_string(0, 5 * UI_ROW_H, "Spd Yaw: ");    tft180_show_float(10 * UI_COL_W, 5 * UI_ROW_H, imu_sensor.get_gyro_z(), 3, 1);

    tft180_show_string(0, 6 * UI_ROW_H, "Spd LF: ");     tft180_show_float(10 * UI_COL_W, 6 * UI_ROW_H, encoders.get_speed_cm_s(0), 3, 1);
    tft180_show_string(0, 7 * UI_ROW_H, "Spd LB: ");     tft180_show_float(10 * UI_COL_W, 7 * UI_ROW_H, encoders.get_speed_cm_s(1), 3, 1);
    tft180_show_string(0, 8 * UI_ROW_H, "Spd RF: ");     tft180_show_float(10 * UI_COL_W, 8 * UI_ROW_H, encoders.get_speed_cm_s(2), 3, 1);
    tft180_show_string(0, 9 * UI_ROW_H, "Spd RB: ");     tft180_show_float(10 * UI_COL_W, 9 * UI_ROW_H, encoders.get_speed_cm_s(3), 3, 1);
}



// 视觉数据监控页面：地图格子、箱子、目标、炸弹、小车位置等的实时绘制
void TftMenu::draw_vision_data() {
    //------------------------------------------------------------------------------------
    // 1. 顶部状态与耗时信息栏
    //------------------------------------------------------------------------------------
    GamePhase current_phase = debug_manager.get_phase();  
    tft180_show_string(0, 0, "                     ");  // 清除顶部的旧文本(覆盖 21 个空格)
    
    // 显示当前进程运行到哪一步了
    if (current_phase <= GamePhase::WAIT_FOR_VISION) {
        tft180_show_string(0, 0, "Phase: WAITING MAP");
    } else if (current_phase == GamePhase::PLAN_SOKOBAN) {
        tft180_show_string(0, 0, "Phase: PLANNING...");
    } else if (current_phase == GamePhase::ANIMATE_DEMO) {
        tft180_show_string(0, 0, "Phase: DEMO PLAY  ");
    } else if (current_phase == GamePhase::EXEC_SOKOBAN) {
        tft180_show_string(0, 0, "Phase: EXECUTING  ");
    } else if (current_phase == GamePhase::FINISHED) {
        tft180_show_string(0, 0, "Phase: FINISHED   ");
    }

    // 显示寻路耗时
    if (current_phase >= GamePhase::ANIMATE_DEMO) {
        tft180_show_string(0, 1 * UI_ROW_H, "Plan Time: ");
        tft180_show_int(11 * UI_COL_W, 1 * UI_ROW_H, debug_manager.get_plan_time_ms(), 4);
        tft180_show_string(15 * UI_COL_W, 1 * UI_ROW_H, "ms");
    } else {
        tft180_show_string(0, 1 * UI_ROW_H, "Plan Time: --  ms");
    }

    if (!vision_data.art1_map_ready && current_phase <= GamePhase::WAIT_FOR_VISION) return;


    //------------------------------------------------------------------------------------
    // 2. 静态地图与网格线绘制
    //------------------------------------------------------------------------------------
    int map_start_y = 2 * UI_ROW_H + 4;

    for (int map_x = 0; map_x < 12; ++map_x) {
        for (int map_y = 0; map_y < 16; ++map_y) {
            int screen_x = map_y * 8;
            int screen_y = map_x * 8 + map_start_y;
            if (vision_data.map[map_y][map_x] == 1) {
                fill_rect(screen_x + 1, screen_y + 1, 7, 7, RGB565_GRAY);  
            } else {
                fill_rect(screen_x + 1, screen_y + 1, 6, 6, RGB565_WHITE);           
            }
        }
    }

    for (int i = 0; i <= 12; ++i) { tft180_draw_line(0, i * 8 + map_start_y, 127, i * 8 + map_start_y, RGB565_BLACK); }
    for (int i = 1; i <= 15; ++i) { tft180_draw_line(i * 8, map_start_y, i * 8, 96 + map_start_y, RGB565_BLACK); }

    //------------------------------------------------------------------------------------
    // 3. 根据当前阶段，决定是画 [动画数据] 还是 [真实数据]
    //------------------------------------------------------------------------------------
    if (current_phase == GamePhase::ANIMATE_DEMO) {
        const DemoState& demo = debug_manager.get_demo_state();
        
        for (int i = 0; i < demo.target_count; ++i) { fill_rect(demo.targets[i].y * 8 + 1, map_start_y + demo.targets[i].x * 8 + 1, 6, 6, RGB565_PINK);}
        for (int i = 0; i < demo.box_count; ++i) { fill_rect(demo.boxes[i].y * 8 + 1, map_start_y + demo.boxes[i].x * 8 + 1, 6, 6, RGB565_YELLOW);}
        fill_rect(demo.player.y * 8 + 2, map_start_y + demo.player.x * 8 + 2, 4, 4, RGB565_GREEN);

        // 底部显示播放进度
        tft180_show_string(0, 140, "Anim Step: ");
        tft180_show_int(11 * UI_COL_W, 140, demo.path_idx, 3);

    } else if (current_phase >= GamePhase::EXEC_SOKOBAN) {

        // 绘制箱子和终点目标
        for (int i = 0; i < vision_data.box_count; ++i) {
            fill_rect(vision_data.targets[i].y * 8 + 1, map_start_y + vision_data.targets[i].x * 8 + 1, 6, 6, RGB565_PINK);
            fill_rect(vision_data.boxes[i].y * 8 + 1, map_start_y + vision_data.boxes[i].x * 8 + 1, 6, 6, RGB565_YELLOW);
        }

        // 绘制 Tracker 提取的折线路径与目标点
        const auto& path = path_tracker.grid_path;
        uint16_t target_idx = path_tracker.current_wp_idx;
        
        if (path.size() > 0) {
            // A. 先画折线 (蓝色)
            for (size_t i = 0; i < path.size() - 1; ++i) {
                // + 4 是为了将线条画在 8x8 格子的正中心
                int cx1 = path[i].y * 8 + 4;
                int cy1 = map_start_y + path[i].x * 8 + 4;
                int cx2 = path[i+1].y * 8 + 4;
                int cy2 = map_start_y + path[i+1].x * 8 + 4;
                tft180_draw_line(cx1, cy1, cx2, cy2, RGB565_BLUE); 
            }

            // B. 再画节点
            for (size_t i = 0; i < path.size(); ++i) {
                int sx = path[i].y * 8;
                int sy = map_start_y + path[i].x * 8;
                
                if (path_tracker.state == TrackerState::TRACKING && i == target_idx) {
                    // 【当前目标点】：画一个显眼的 蓝底白心 的嵌套矩形
                    fill_rect(sx + 1, sy + 1, 6, 6, RGB565_BLUE);
                    fill_rect(sx + 2, sy + 2, 4, 4, RGB565_WHITE);
                } else if (i < target_idx) {
                    // 【已路过的点】：画个小灰点
                    fill_rect(sx + 3, sy + 3, 2, 2, RGB565_GRAY);
                } else {
                    // 【未来的点】：画个小蓝点
                    fill_rect(sx + 3, sy + 3, 2, 2, RGB565_BLUE);
                }
            }
        }

        // 换算真实物理坐标 -> 屏幕像素
        Point2D pos = chassis_odometry.get_position();
        float car_screen_x = pos.y  / GRID_SIZE_CM * 8.0f;
        float car_screen_y = map_start_y + pos.x / GRID_SIZE_CM * 8.0f;

        // 边界保护：确保小车不会画出 128x160 屏幕导致内存越界
        if (car_screen_x >= 2 && car_screen_x <= 126 && 
            car_screen_y >= map_start_y + 2 && car_screen_y <= map_start_y + 94) {
            
            // -2 是为了将 4x4 的车子放在 8x8 的格子中心
            fill_rect((int)car_screen_x - 2, (int)car_screen_y - 2, 4, 4, RGB565_RED); 
        }

        // 3. 底部显示运行进度与目标 WP 编号
        tft180_show_string(0, 140, "Status: TRACKING   ");
        tft180_show_string(0, 150, "Target WP: ");
        tft180_show_int(11 * UI_COL_W, 150, target_idx, 2);
        tft180_show_string(13 * UI_COL_W, 150, "/");
        tft180_show_int(14 * UI_COL_W, 150, path.size(), 2);
    }
}



// -------------------------局部刷新辅助函数 -------------------------

void TftMenu::draw_item(uint8_t row, const char* name, bool is_selected) {
    if (is_selected) tft180_show_string(0, row * UI_ROW_H, ">"); 
    else tft180_show_string(0, row * UI_ROW_H, " "); 
    // 名称从第 2 列开始写
    tft180_show_string(1 * UI_COL_W, row * UI_ROW_H, (char*)name);
}

void TftMenu::draw_float_item(uint8_t row, const char* name, float val, bool is_selected, bool is_editing_this) {
    // 渲染光标和名称 (占用 0 ~ 9 列)
    draw_item(row, name, is_selected);
    
    // 渲染编辑标识 (占用 10 ~ 12 列)
    if (is_selected && is_editing_this) {
        tft180_show_string(10 * UI_COL_W, row * UI_ROW_H, "[E]");
    } else {
        tft180_show_string(10 * UI_COL_W, row * UI_ROW_H, "   ");
    }

    // 渲染浮点数 (从第 14 列开始，最多占用 7 列)
    // 格式化占位：2位整数 + 1位符号 + 1位小数点 + 3位小数 = 7个字符。
    // 14 + 7 = 21列。完美容纳在 128 像素内。
    tft180_show_float(14 * UI_COL_W, row * UI_ROW_H, val, 2, 3);
}



// ------------------------- 基础绘图接口封装 -------------------------

void TftMenu::fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color) {
    for (uint8_t i = 0; i < h; ++i) {
        tft180_draw_line(x, y + i, x + w - 1, y + i, color);
    }
}
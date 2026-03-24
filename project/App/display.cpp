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
    {"Yaw_Kp",   &tune.pid_yaw.kp,                  0.1f },
    {"Yaw_Kd",   &tune.pid_yaw.kd,                  0.01f},
    {"Spd_Kp",   &tune.pid_speed.kp,                0.1f },
    {"Spd_Ki",   &tune.pid_speed.ki,                0.01f},
    {"Max_Spd",  &tune.tracker.max_speed,           1.0f },
    {"Max_Acc",  &tune.tracker.max_acc,             0.5f },
    {"MaxASpd",  &tune.tracker.max_ang_speed,       0.1f },
    {"ReachRad", &tune.tracker.reach_radius,        1.0f },
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
    tft180_set_color(RGB565_RED, RGB565_BLACK);      // 黑底红字
    tft180_init();

    tft180_full(RGB565_BLACK);
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
                    tft180_full(RGB565_BLACK); 
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

        default:
            if (key_back_pressed) { current_page = MenuPage::MAIN_MENU; need_full_redraw = true; }
            break;
    }
}

// 硬件报错挂起
void TftMenu::halt_with_error(const char* err_msg) {
    tft180_full(RGB565_BLACK);
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
    if (need_full_redraw) { tft180_full(RGB565_BLACK); need_full_redraw = false; }

    switch (current_page) {
        case MenuPage::MAIN_MENU:      draw_main_menu(); break;
        case MenuPage::GAME_STATUS:    draw_game_status(); break;
        case MenuPage::ODOMETRY_DATA:  draw_odometry_data(); break;
        case MenuPage::VISION_DATA:    draw_vision_data(); break;
        case MenuPage::TUNE_PARAMS:    draw_tune_params(); break;
    }
    ui_dirty = false; 
}


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

void TftMenu::draw_game_status() {
    tft180_show_string(0, 0, "-- GAME STATUS --");
    tft180_show_string(0, 2 * UI_ROW_H, "Phase: ");
    switch(game_manager.get_phase()) {
        case GamePhase::INIT_CALIBRATE:    tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "INIT_CALIB"); break;
        case GamePhase::EXIT_START_ZONE:   tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "EXIT_ZONE "); break;
        case GamePhase::WAIT_FOR_VISION:   tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "WAIT_VIS  "); break;
        case GamePhase::PLAN_SOKOBAN:      tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "PLANNING  "); break;
        case GamePhase::EXEC_SOKOBAN:      tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "TRACKING  "); break;
        case GamePhase::PLAN_RETURN:       tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "PLAN_RET  "); break;
        case GamePhase::EXEC_RETURN:       tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "RETURNING "); break;
        case GamePhase::ENTER_START_ZONE:  tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "ENTER_ZONE"); break;
        case GamePhase::FINISHED:          tft180_show_string(7 * UI_COL_W, 2 * UI_ROW_H, "FINISHED  "); break;
    }

    tft180_show_string(0, 6 * UI_ROW_H, "Last RX Cmd:");
    tft180_show_string(0, 7 * UI_ROW_H, last_rx_cmd); // 打印全局变量
}

// void TftMenu::draw_vision_data() {
//     tft180_show_string(0, 0, "-- VISION --");
//     tft180_show_string(0, 1 * UI_ROW_H, vision_data.art1_map_ready ? "Map: Ready  " : "Map: Waiting");

//     if (!vision_data.art1_map_ready) {
//         return; // 地图没就绪就不画下面了
//     }

//     // 地图原点设定 (屏幕宽度128, 地图宽16*5=80, 居中X=(128-80)/2=24)
//     int map_start_x = 24; 
//     int map_start_y = 2 * UI_ROW_H + 5; 

//     // 1. 绘制底层地图网格 (16x12)
//     // 根据你的按位解析逻辑，Y=0是下面(起点)，为了符合屏幕习惯，Y值需要倒置(11 - y)
//     for (int y = 0; y < 12; ++y) {
//         for (int x = 0; x < 16; ++x) {
//             uint16_t color = (vision_data.map[y][x] == 1) ? RGB565_GRAY : RGB565_BLUE;
//             // 调用你底层的画实心矩形函数
//             tft180_fill_rect(map_start_x + x * 5, map_start_y + (11 - y) * 5, 5, 5, color);
//         }
//     }

//     // 2. 绘制箱子 (黄色) 与 目标点 (紫色)
//     for (int i = 0; i < vision_data.box_count; i++) {
//         int tx = vision_data.targets[i].x;
//         int ty = vision_data.targets[i].y;
//         tft180_fill_rect(map_start_x + tx * 5, map_start_y + (11 - ty) * 5, 5, 5, RGB565_PURPLE);

//         int bx = vision_data.boxes[i].x;
//         int by = vision_data.boxes[i].y;
//         tft180_fill_rect(map_start_x + bx * 5, map_start_y + (11 - by) * 5, 5, 5, RGB565_YELLOW);
//     }

//     // 3. 绘制小车全局定位点 (红色)
//     // 通过里程计当前绝对坐标，除以每个网格对应的真实物理长度，映射回网格系
//     Point2D pos = chassis_odometry.get_position();
//     float car_grid_x = pos.x / CM_PER_GRID;
//     float car_grid_y = pos.y / CM_PER_GRID;
    
//     // 如果小车在地图范围内，就画一个半格大小 (3x3 像素) 的红点
//     if (car_grid_x >= 0 && car_grid_x <= 16.0f && car_grid_y >= 0 && car_grid_y <= 12.0f) {
//         int px = map_start_x + (int)(car_grid_x * 5) + 1; // +1为了居中到5像素格子内
//         int py = map_start_y + (11 * 5) - (int)(car_grid_y * 5) + 1;
//         tft180_fill_rect(px, py, 3, 3, RGB565_RED);
//     }

//     // 4. 在屏幕底部显示具体数值
//     int text_y = map_start_y + (12 * 5) + 6;
//     tft180_show_string(0, text_y, "Boxes: ");
//     tft180_show_int(8 * UI_COL_W, text_y, vision_data.box_count, 2);
    
//     tft180_show_string(0, text_y + UI_ROW_H, "Targets: ");
//     tft180_show_int(9 * UI_COL_W, text_y + UI_ROW_H, vision_data.box_count, 2); // 目标数一般与箱子相等
    
//     tft180_show_string(0, text_y + 2 * UI_ROW_H, "ART2 ID: ");
//     if (vision_data.art2_result_ready) {
//         tft180_show_int(9 * UI_COL_W, text_y + 2 * UI_ROW_H, vision_data.current_front_id, 2);
//     } else {
//         tft180_show_string(9 * UI_COL_W, text_y + 2 * UI_ROW_H, "--");
//     }
// }

void TftMenu::draw_vision_data() {
    tft180_show_string(0, 0, "-- VISION --");
    tft180_show_string(0, 2 * UI_ROW_H, vision_data.art1_map_ready ? "Map: Ready  " : "Map: Waiting");
    
    tft180_show_string(0, 3 * UI_ROW_H, "Boxes: ");
    tft180_show_int(8 * UI_COL_W, 3 * UI_ROW_H, vision_data.box_count, 2);
    
    tft180_show_string(0, 4 * UI_ROW_H, "Bombs: ");
    tft180_show_int(8 * UI_COL_W, 4 * UI_ROW_H, vision_data.bomb_count, 2);
    
    tft180_show_string(0, 5 * UI_ROW_H, "ART2 ID: ");
    if (vision_data.art2_result_ready) {
        tft180_show_int(9 * UI_COL_W, 5 * UI_ROW_H, vision_data.current_front_id, 2);
    } else {
        tft180_show_string(9 * UI_COL_W, 5 * UI_ROW_H, "--");
    }
}

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
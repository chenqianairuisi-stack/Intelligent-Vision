#include "display.h"
#include "task_control.h"
#include "tuning_config.h"
#include "game_manage.h"
#include "odometry.h"
#include "encoder.h"
#include "imu.h"
#include "storage.h"

TftMenu sys_menu;

// ================= 全局参数字典 (神级调参入口) =================
// 结构：{ "屏幕显示名称",  变量的内存地址,  按键单次加减步长 }
struct ParamItem {
    const char* name;
    float* val_ptr;
    float step;
};

static ParamItem tune_dict[] = {
    {"X_Kp",     &tune.pid_x.kp,              0.1f },
    {"X_Ki",     &tune.pid_x.ki,              0.01f},
    {"X_Kd",     &tune.pid_x.kd,              0.01f},
    {"Y_Kp",     &tune.pid_y.kp,              0.1f },
    {"Y_Ki",     &tune.pid_y.ki,              0.01f},
    {"Y_Kd",     &tune.pid_y.kd,              0.01f},
    {"Yaw_Kp",   &tune.pid_yaw.kp,            0.1f },
    {"Yaw_Kd",   &tune.pid_yaw.kd,            0.01f},
    {"Spd_Kp",   &tune.pid_speed.kp,          0.1f },
    {"Spd_Ki",   &tune.pid_speed.ki,          0.01f},
    {"MaxSpd",   &tune.tracker.max_speed,     5.0f },
    {"ReachRad", &tune.tracker.reach_radius,  1.0f },
};
// 自动计算字典大小
static constexpr int DICT_SIZE = sizeof(tune_dict) / sizeof(tune_dict[0]);
static constexpr int PARAMS_PER_PAGE = 7; // 一页最多显示7个参数 (128x160竖屏支持10行)
// ===============================================================

TftMenu::TftMenu() {}

void TftMenu::init() {
    current_page = MenuPage::MAIN_MENU;
    cursor_idx = 0;
    scroll_offset = 0;
    is_editing = false;
    need_full_redraw = true;   // 保证第一帧一定会全屏刷黑
    ui_dirty = true;           // 保证开机一定会渲染菜单
    last_k1 = true; last_k2 = true; last_k3 = true; last_k4 = true;

    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);

    tft180_set_dir(TFT180_PORTAIT);                  // 竖屏
    tft180_set_font(TFT180_8X16_FONT);               // 设置字库
    tft180_set_color(RGB565_RED, RGB565_BLACK);      // 黑底红字
    tft180_init();

    tft180_full(RGB565_BLACK);
    system_delay_ms(50);                             // 延时确保初始化完成
}

void TftMenu::run() { scan_keys(); process_logic(); render_ui(); }


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
        current_page == MenuPage::ODOMETRY_DATA || 
        current_page == MenuPage::HARDWARE_RAW) {
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
                if (cursor_idx == 2) current_page = MenuPage::HARDWARE_RAW;
                if (cursor_idx == 3) current_page = MenuPage::VISION_DATA;
                if (cursor_idx == 4) { current_page = MenuPage::TUNE_PARAMS; cursor_idx = 0; scroll_offset = 0; }
                // --- Flash 存储触发 ---
                if (cursor_idx == 5) {
                    Storage::save_params();
                    // 在屏幕右侧打个 [OK] 提示，延时 300ms 让人眼能看清
                    tft180_show_string(16*8, 6 * 16, "[OK]");
                    system_delay_ms(300); 
                }
                if (cursor_idx == 6) {
                    Storage::load_params();
                    tft180_show_string(16*8, 7 * 16, "[OK]");
                    system_delay_ms(300);
                }
                
                // 前5个是页面跳转，需要重置光标和清屏
                if (cursor_idx < 5) {
                    cursor_idx = 0; need_full_redraw = true;
                } else {
                    // 第6和第7项执行完只需局部刷新把 [OK] 擦掉
                    need_full_redraw = true; 
                }
            }
            break;

        case MenuPage::ODOMETRY_DATA:
            if (key_back_pressed) { current_page = MenuPage::MAIN_MENU; need_full_redraw = true; }
            // 按下确认键，一键将里程计 X,Y 清零，用于发车校准
            // if (key_enter_pressed) {
            //     chassis_odometry.calibrate_position(0.0f, 0.0f);  // TODO: 需要在 Odometry 类中添加此方法
            //     imu_sensor.reset_yaw(0.0f);                       // TODO: 需要在 Imu 类中添加此方法
            // }
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
                if (key_back_pressed) { current_page = MenuPage::MAIN_MENU; cursor_idx = 4; need_full_redraw = true; }
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

// UI 渲染器：根据 current_page 调用对应的绘制函数
void TftMenu::render_ui() {
    if (!ui_dirty) return;
    if (need_full_redraw) { tft180_full(RGB565_BLACK); need_full_redraw = false; }

    switch (current_page) {
        case MenuPage::MAIN_MENU:      draw_main_menu(); break;
        case MenuPage::GAME_STATUS:    draw_game_status(); break;
        case MenuPage::ODOMETRY_DATA:  draw_odometry_data(); break;
        case MenuPage::HARDWARE_RAW:   draw_hardware_raw(); break;
        case MenuPage::VISION_DATA:    draw_vision_data(); break;
        case MenuPage::TUNE_PARAMS:    draw_tune_params(); break;
    }
    ui_dirty = false; 
}


// ------------------------- 页面绘制函数 -------------------------

// 主菜单
void TftMenu::draw_main_menu() {
    tft180_show_string(0, 0, "-- COMMAND --");
    draw_item(1, "Game State", cursor_idx == 0);
    draw_item(2, "Odometry",   cursor_idx == 1);
    draw_item(3, "Hardware",   cursor_idx == 2);
    draw_item(4, "Vision",     cursor_idx == 3);
    draw_item(5, "Tuning",     cursor_idx == 4);
    // 新增：Flash 读写开关
    draw_item(6, "Save Config",cursor_idx == 5);
    draw_item(7, "Load Config",cursor_idx == 6);
}

// [滚动页面] 调参页面，展示 tune_dict 中的参数，支持上下滚动和编辑
void TftMenu::draw_tune_params() {
    tft180_show_string(0, 0, "-- PARAMETERS --");
    
    // 仅绘制当前滚动窗口内的参数
    for (int i = 0; i < PARAMS_PER_PAGE; i++) {
        int item_idx = scroll_offset + i;
        if (item_idx >= DICT_SIZE) {
            // 清除多余的空行残留
            tft180_show_string(0, (i + 1) * 16, "                    ");
            continue;
        }

        draw_float_item(i + 1, 
                        tune_dict[item_idx].name, 
                        *(tune_dict[item_idx].val_ptr), 
                        cursor_idx == item_idx, 
                        is_editing && (cursor_idx == item_idx));
    }
    
    // 右上角加个进度条提示 (如 1/13)
    tft180_show_int(16*7, 0, cursor_idx + 1, 2);
    tft180_show_string(16*8 + 8, 0, "/");
    tft180_show_int(16*9 + 8, 0, DICT_SIZE, 2);
}

// [静态页面] 显示比赛状态
void TftMenu::draw_game_status() {
    tft180_show_string(0, 0, "-- GAME STATUS --");
    tft180_show_string(0, 16, "Phase: ");
    switch(game_manager.get_phase()) {
        case GamePhase::INIT_CALIBRATE:   tft180_show_string(16*4, 16, "INIT_CALIB"); break;
        case GamePhase::EXIT_START_ZONE:  tft180_show_string(16*4, 16, "EXIT_ZONE "); break;
        case GamePhase::WAIT_FOR_VISION:  tft180_show_string(16*4, 16, "WAIT_VIS  "); break;
        case GamePhase::PLAN_SOKOBAN:     tft180_show_string(16*4, 16, "PLANNING  "); break;
        case GamePhase::EXEC_SOKOBAN:     tft180_show_string(16*4, 16, "TRACKING  "); break;
        case GamePhase::PLAN_RETURN:      tft180_show_string(16*4, 16, "PLAN_RET  "); break;
        case GamePhase::EXEC_RETURN:      tft180_show_string(16*4, 16, "RETURNING "); break;
        case GamePhase::ENTER_START_ZONE: tft180_show_string(16*4, 16, "ENTER_ZONE"); break;
        case GamePhase::FINISHED:         tft180_show_string(16*4, 16, "FINISHED  "); break;
    }
}

// [实时刷新页] 展示视觉系统数据
void TftMenu::draw_vision_data() {
    tft180_show_string(0, 0, "-- VISION --");
    tft180_show_string(0, 16, vision_data.art1_map_ready ? "Map: Ready  " : "Map: Waiting");
    
    tft180_show_string(0, 32, "Boxes: ");
    tft180_show_int(16*4, 32, vision_data.box_count, 2);
    
    tft180_show_string(0, 48, "Bombs: ");
    tft180_show_int(16*4, 48, vision_data.bomb_count, 2);
    
    tft180_show_string(0, 64, "ART2 ID: ");
    if (vision_data.art2_result_ready) {
        tft180_show_int(16*5, 64, vision_data.current_front_id, 2);
    } else {
        tft180_show_string(16*5, 64, "--");
    }
}

// [实时刷新页] 展示里程计坐标
void TftMenu::draw_odometry_data() {
    tft180_show_string(0, 0, "-- ODOMETRY --");
    Point2D pos = chassis_odometry.get_position();
    
    tft180_show_string(0, 16, "Global X: ");
    tft180_show_float(16*5, 16, pos.x, 3, 2);
    
    tft180_show_string(0, 32, "Global Y: ");
    tft180_show_float(16*5, 32, pos.y, 3, 2);

    tft180_show_string(0, 48, "Yaw(deg): ");
    tft180_show_float(16*5, 48, imu_sensor.get_yaw(), 3, 2);  // get_yaw() 已经是度

}

// [实时刷新页] 展示底层硬件速度 (用于检查编码器有没有反)
void TftMenu::draw_hardware_raw() {
    tft180_show_string(0, 0, "-- HW RAW --");
    tft180_show_string(0, 16, "Spd LF: "); tft180_show_float(16*4, 16, encoders.get_speed_cm_s(0), 2, 1);
    tft180_show_string(0, 32, "Spd LB: "); tft180_show_float(16*4, 32, encoders.get_speed_cm_s(1), 2, 1);
    tft180_show_string(0, 48, "Spd RF: "); tft180_show_float(16*4, 48, encoders.get_speed_cm_s(2), 2, 1);
    tft180_show_string(0, 64, "Spd RB: "); tft180_show_float(16*4, 64, encoders.get_speed_cm_s(3), 2, 1);
}


// -------------------------局部刷新辅助函数 -------------------------

void TftMenu::draw_item(uint8_t row, const char* name, bool is_selected) {
    if (is_selected) tft180_show_string(0, row * 16, ">"); 
    else tft180_show_string(0, row * 16, " "); 
    tft180_show_string(16, row * 16, (char*)name);
}

void TftMenu::draw_float_item(uint8_t row, const char* name, float val, bool is_selected, bool is_editing_this) {
    draw_item(row, name, is_selected);
    
    if (is_selected && is_editing_this) {
        tft180_show_string(16*5+8, row * 16, "[E]");
    } else {
        tft180_show_string(16*5+8, row * 16, "   ");
    }

    tft180_show_float(16*7+8, row * 16, val, 2, 3); // 微调了排版间距，防止文字重叠
}





// #pragma once

// #define KEY1 C13
// #define KEY2 C12
// #define KEY3 C14
// #define KEY4 C15


// class tft_menu {
// public:
//     tft_menu() : page(0),arrow(1),need_update(true) {}

//     void init();
//     void run();

// private:
//     uint8_t page,arrow;    // 页数，行数
//     bool need_update;    // 刷新标志

//     void input_process();
//     void switch_page();
//     void show_main_menu();
//     void show_page1();
//     void show_page2();
//     void show_page3();
//     void show_page4();
// };

// #include "zf_common_headfile.h"
// #include "display.h"

// // 菜单初始化
// void tft_menu::init() {

//     // 初始化 KEY1,KEY2,KEY3,KEY4,KEY5 输入 默认高电平 上拉输入 用于控制菜单
//     gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);
//     gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);
//     gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);
//     gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);

//     //初始化tft180屏幕
//     tft180_set_dir(TFT180_CROSSWISE);                 //横屏
//     tft180_set_color(RGB565_RED, RGB565_BLACK);       //设置背景颜色（黑）
//     tft180_init();
// }

// // 菜单运行主函数
// void tft_menu::run() {

//     input_process();   //按键输入处理

//     if(need_update) {
//         switch_page();   //选择菜单
//     }
// }

// //按键输入处理
// void tft_menu::input_process() {

//     //按键1：换行
//     if(!gpio_get_level(KEY1)) {
//         system_delay_ms(160);
//         need_update=1;
//         arrow++;
//         if(arrow>4) arrow=1;
//     }

//     //按键2：进入下一级菜单/返回上一级菜单
//     if(!gpio_get_level(KEY2)) {
//         system_delay_ms(160);
//         need_update=1;

//         if(page>0)  page=0;
//         else  page=arrow;
//     }

//     //按键3：参数增大
//     if(!gpio_get_level(KEY3)) {
//         system_delay_ms(120);
//         need_update=1;

//         switch(page) {
//             case 1:
//                 break;
//             case 2:
//                 break;
//             case 3:
//                 break;
//             case 4:
//                 break;
//             case 5:
//                 break;
//             default: break;
//         }

//     }

//     //按键4：参数减小
//     if(!gpio_get_level(KEY4)) {
//         system_delay_ms(120);
//         need_update=1;

//         switch(page) {
//             case 1:
//                 break;
//             case 2:
//                 break;
//             case 3:
//                 break;
//             case 4:
//                 break;
//             case 5:
//                 break;
//             default: break;
//         }
//     }
// }

// void tft_menu::switch_page() {

//     switch (page) {
//         case 0: show_main_menu();   break;
//         case 1: show_page1();  break;
//         case 2: show_page2();  break;
//         case 3: show_page3();  break;
//         case 4: show_page4();  break;
//     }
// }

// //显示主菜单
// void tft_menu::show_main_menu() {
//     tft180_full(RGB565_BLACK);
//     tft180_show_string( 16*0,  16*arrow,   ">");
//     tft180_show_string( 16*1,  16*0,   "MAIN_MENU");
//     tft180_show_string( 16*1,  16*1,   "Filter");
//     tft180_show_string( 16*1,  16*2,   "PID");
//     tft180_show_string( 16*1,  16*3,   "IMU");
//     tft180_show_string( 16*1,  16*4,   "Motor");

//     need_update=0;
// }

// //显示子菜单1（滤波）
// void tft_menu::show_page1() {
//     tft180_full(RGB565_BLACK);
//     tft180_show_string( 16*0,  16*arrow,   ">");
//     tft180_show_string( 16*1,  16*0,   "Filter");

//     need_update=0;
// }

// //显示子菜单2（PID）
// void tft_menu::show_page2() {

//     tft180_full(RGB565_BLACK);
//     tft180_show_string( 16*0,  16*arrow,   ">");
//     tft180_show_string( 16*1,  16*0,   "PID");
//     tft180_show_string( 16*1,  16*1,   "kp");
//     tft180_show_string( 16*1,  16*2,   "ki");
//     tft180_show_string( 16*1,  16*3,   "kd");

//     need_update=0;
// }

// //显示子菜单3（IMU）
// void tft_menu::show_page3() {

//     tft180_full(RGB565_BLACK);
//     tft180_show_string( 16*0,  16*arrow,   ">");
//     tft180_show_string( 16*1,  16*0,   "IMU");
//     tft180_show_string( 16*1,  16*1,   "yaw");
//     // tft180_show_float(16*6, 16*1, angle_yaw, 3, 2);

//     system_delay_ms(10);
// }

// //显示子菜单4（Motor）
// void tft_menu::show_page4() {

//     tft180_full(RGB565_BLACK);
//     tft180_show_string( 16*0,  16*arrow,   ">");
//     tft180_show_string( 16*1,  16*0,   "Motor");

//     system_delay_ms(10);
// }

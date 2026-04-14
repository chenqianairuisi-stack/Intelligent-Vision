#include "Display.h"
#include "RobotState.h"
#include "tuning_config.h"
#include "GameManage.h"
#include "TestMap.h"
#include "Storage.h"
#include "Encoder.h"

// 硬件引脚定义
#define KEY1 C14  // 下移 / 减小
#define KEY2 C12  // 确认 / 编辑模式
#define KEY3 C15  // 上移 / 增大
#define KEY4 C13  // 返回 / 退出编辑


namespace Subsystem::Display {

// ====================================================================
// 1. 私有数据结构与上下文 (Internal Context)
// ====================================================================

namespace { // 匿名命名空间，确保这些数据只在本文件可见

    // 所有的 UI 状态收敛进一个 Context 结构体
    struct UIContext {
        MenuPage current_page = MenuPage::DASHBOARD;   // 当前页面
        uint8_t  cursor_idx = 0;                       // 当前选中行
        uint8_t  scroll_offset = 0;                    // 用于参数列表的上下滚动翻页
        uint8_t  map_cursor_idx = 0;                   // 地图页面的光标位置
        uint8_t  map_scroll_offset = 0;                // 地图页面的滚动偏移
        bool     is_editing = false;                   // 是否处于编辑模式
        bool     need_full_redraw = true;              // 是否需要全屏重绘 
        bool     ui_dirty = true;                      // UI 是否需要更新
        bool     is_closed = false;                    // 息屏标志位

        // 按键状态
        bool key_up_pressed = false; bool key_down_pressed = false;
        bool key_enter_pressed = false; bool key_back_pressed = false;
        bool last_k1 = true, last_k2 = true, last_k3 = true, last_k4 = true;                                       
        
        // 显存缓冲
        uint8_t back_buffer[16][12] = {0}; 
    };

    UIContext ctx;

    // --- 页面布局常量 ---
    constexpr uint8_t TL_WALL=1<<0, TL_TGT=1<<1, TL_BOX=1<<2, TL_BOMB=1<<3, TL_PATH=1<<4, TL_CRS=1<<5, TL_CAR=1<<6;
    constexpr int UI_COL_W = 6; 
    constexpr int UI_ROW_H = 10;

    // --- 巨大的参数字典放到最后或者单独抽离 ---
    struct ParamItem { const char* name; float* val_ptr; float step; };
    ParamItem tune_dict[] = { 
    {"Yaw_Kp  ",   &tune.pid_yaw.kp,                  0.1f  },
    {"Yaw_Ki  ",   &tune.pid_yaw.ki,                  0.01f },
    {"Yaw_Kd  ",   &tune.pid_yaw.kd,                  0.01f },
    {"Spd_Kp  ",   &tune.pid_speed.kp,                0.01f },
    {"Spd_Ki  ",   &tune.pid_speed.ki,                0.01f },
    {"ff_Kv   ",   &tune.ff.kv,                       0.01f },
    {"ff_Ka   ",   &tune.ff.ka,                       1.0f  },
    {"Max_Duty",   &tune.dynamics.max_duty,           1.0f  },
    {"Max_Spd ",   &tune.dynamics.max_speed,          1.0f  },
    {"Max_Acc ",   &tune.dynamics.max_acc,            5.0f  },
    {"MaxJerk ",   &tune.dynamics.max_jerk,           50.0f },
    {"MaxASpd ",   &tune.dynamics.max_ang_speed,      0.1f  },
    {"Gain_X  ",   &tune.dynamics.kinematic_gain_x,   0.01f },
    {"Gain_Y  ",   &tune.dynamics.kinematic_gain_y,   0.01f },
    {"Reach_R ",   &tune.tracker.reach_radius,        0.1f  },
    {"Reach_M ",   &tune.tracker.reach_radius_min,    0.1f  }
     };
    constexpr int DICT_SIZE = sizeof(tune_dict) / sizeof(tune_dict[0]);   
    constexpr int PARAMS_PER_PAGE = 12; 
}


// ====================================================================
// 2. 内部辅助函数声明
// ====================================================================
static void scan_keys();
static void process_logic();
static void render_ui();
static void draw_main_menu();
static void draw_dashboard();
static void draw_map_select();
static void draw_odometry_data();
static void draw_tune_params();
static void draw_item(uint8_t row, const char* name, bool is_selected);
static void draw_float_item(uint8_t row, const char* name, float val, bool is_selected, bool is_editing);
static void fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);


// ====================================================================
// 3. 对外公开接口 (Facade) 
// ====================================================================
void init() {
    ctx = UIContext(); // 瞬间重置所有状态

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


void run() { 
    scan_keys(); 
    if (ctx.is_closed) {
        if (ctx.key_back_pressed) {     // 按返回键唤醒
            ctx.is_closed = false;
            ctx.need_full_redraw = true;
            ctx.current_page = MenuPage::MAIN_MENU;
            ctx.cursor_idx = 0;
            ctx.ui_dirty = true;
        }
        return;  // 拦截后续逻辑与渲染
    }
    
    process_logic(); 
    render_ui(); 
}


// ====================================================================
// 4. 私有业务逻辑实现细节
// ====================================================================

// 按键扫描函数：检测按键边缘，更新按键状态变量，并标记 UI 需要刷新
void scan_keys() { 
    bool cur_k1 = gpio_get_level(KEY1);
    bool cur_k2 = gpio_get_level(KEY2);
    bool cur_k3 = gpio_get_level(KEY3);
    bool cur_k4 = gpio_get_level(KEY4);

    // 边缘检测触发：只有在按键从 高电平(true) 变到 低电平(false) 的瞬间，才触发一次 true
    ctx.key_down_pressed  = (ctx.last_k1 && !cur_k1);
    ctx.key_enter_pressed = (ctx.last_k2 && !cur_k2);
    ctx.key_up_pressed    = (ctx.last_k3 && !cur_k3);
    ctx.key_back_pressed  = (ctx.last_k4 && !cur_k4);

    ctx.last_k1 = cur_k1; ctx.last_k2 = cur_k2; 
    ctx.last_k3 = cur_k3; ctx.last_k4 = cur_k4;

    if (ctx.key_down_pressed || ctx.key_enter_pressed || ctx.key_up_pressed || ctx.key_back_pressed) {
        ctx.ui_dirty = true;
    }
}

// 核心逻辑控制器：状态跳转与参数修改
void process_logic() {
    // 高频刷新监控页面
    if (ctx.current_page == MenuPage::DASHBOARD || ctx.current_page == MenuPage::ODOMETRY_DATA) {
        ctx.ui_dirty = true;
    }

    if (!ctx.ui_dirty) return;

    switch (ctx.current_page) {
        case MenuPage::MAIN_MENU:
            if (ctx.key_down_pressed) ctx.cursor_idx = (ctx.cursor_idx + 1) % 9;    // 9个主菜单项
            if (ctx.key_up_pressed)   ctx.cursor_idx = (ctx.cursor_idx == 0) ? 8 : ctx.cursor_idx - 1;

            if (ctx.key_enter_pressed) {
                if (ctx.cursor_idx == 0) { ctx.current_page = MenuPage::DASHBOARD;  App::g_state.debug.need_bg_redraw = true;}
                if (ctx.cursor_idx == 1) { ctx.current_page = MenuPage::ODOMETRY_DATA; }
                if (ctx.cursor_idx == 2) { ctx.current_page = MenuPage::TUNE_PARAMS; ctx.cursor_idx = 0; ctx.scroll_offset = 0; }
                // --- 运行模式切换 ---
                if (ctx.cursor_idx == 3) {
                    App::g_state.game.is_debug_mode = !App::g_state.game.is_debug_mode;
                    tft180_show_string(14 * UI_COL_W, 5 * UI_ROW_H, "[SWAP]");
                    system_delay_ms(300);
                }
                // --- Flash 存储触发 ---
                if (ctx.cursor_idx == 4) {
                    Storage::save_params();
                    tft180_show_string(15 * UI_COL_W, 6 * UI_ROW_H, "[OK]");
                    system_delay_ms(300);
                }
                if (ctx.cursor_idx == 5) {
                    Storage::load_params();
                    tft180_show_string(15 * UI_COL_W, 7 * UI_ROW_H, "[OK]");
                    system_delay_ms(300);
                }
                // --- 息屏模式 ---
                if (ctx.cursor_idx == 6) {
                    ctx.is_closed = true;
                    tft180_full(RGB565_WHITE); 
                    return; 
                }
                // --- 其他测试功能占位 ---
                if (ctx.cursor_idx == 7) {
                    App::g_state.control.current_target.y += 100;
                    return; 
                }
                if (ctx.cursor_idx == 8) {
                    App::g_state.control.current_target.x += 40;
                    return; 
                }

                // 页面跳转后触发重绘
                if (ctx.cursor_idx < 3) {ctx.need_full_redraw = true; ctx.cursor_idx = 0; ctx.ui_dirty = true;} 
                else { ctx.ui_dirty = true;}
            }
            break;

        case MenuPage::TUNE_PARAMS:
            if (!ctx.is_editing) {
                // 列表上下移动
                if (ctx.key_down_pressed) ctx.cursor_idx = (ctx.cursor_idx + 1) % DICT_SIZE;
                if (ctx.key_up_pressed)   ctx.cursor_idx = (ctx.cursor_idx == 0) ? (DICT_SIZE - 1) : ctx.cursor_idx - 1;
                
                // 自动计算滚动窗口，保证光标始终在屏幕内
                if (ctx.cursor_idx < ctx.scroll_offset) ctx.scroll_offset = ctx.cursor_idx;
                if (ctx.cursor_idx >= ctx.scroll_offset + PARAMS_PER_PAGE) ctx.scroll_offset = ctx.cursor_idx - PARAMS_PER_PAGE + 1;

                if (ctx.key_enter_pressed) ctx.is_editing = true;
                if (ctx.key_back_pressed) { ctx.current_page = MenuPage::MAIN_MENU; ctx.cursor_idx = 2; ctx.need_full_redraw = true; ctx.ui_dirty = true; }
            } else {
                // 编辑模式：直接操作字典中的指针，一键修改全局黑板变量
                if (ctx.key_up_pressed)   *(tune_dict[ctx.cursor_idx].val_ptr) += tune_dict[ctx.cursor_idx].step;
                if (ctx.key_down_pressed) *(tune_dict[ctx.cursor_idx].val_ptr) -= tune_dict[ctx.cursor_idx].step;
                if (ctx.key_enter_pressed || ctx.key_back_pressed) ctx.is_editing = false;
            }
            break;

        case MenuPage::DASHBOARD:
            // 按确认键注入本地地图
            if (ctx.key_enter_pressed) {
                ctx.current_page = MenuPage::MAP_SELECT;
                ctx.map_cursor_idx = 0;       // 光标复位
                ctx.map_scroll_offset = 0;
                ctx.need_full_redraw = true; 
                ctx.ui_dirty = true;  
            }
            if (ctx.key_back_pressed) { 
                ctx.current_page = MenuPage::MAIN_MENU; 
                ctx.need_full_redraw = true; 
                ctx.ui_dirty = true; 
            }
            break;

        case MenuPage::MAP_SELECT:
            if (ctx.key_down_pressed) ctx.map_cursor_idx = (ctx.map_cursor_idx + 1) % TestMap::get_mock_map_count();
            if (ctx.key_up_pressed)   ctx.map_cursor_idx = (ctx.map_cursor_idx == 0) ? (TestMap::get_mock_map_count() - 1) : ctx.map_cursor_idx - 1;
            
            // 滚动窗口保护计算
            if (ctx.map_cursor_idx < ctx.map_scroll_offset) ctx.map_scroll_offset = ctx.map_cursor_idx;
            if (ctx.map_cursor_idx >= ctx.map_scroll_offset + PARAMS_PER_PAGE) ctx.map_scroll_offset = ctx.map_cursor_idx - PARAMS_PER_PAGE + 1;

            // 确认选择：加载对应的地图并跳回 VISION_DATA 监视结果
            if (ctx.key_enter_pressed) {
                App::g_state.game.phase = GamePhase::WAIT_FOR_VISION; // 直接跳过视觉模块，进入寻路阶段
                TestMap::load_mock_map(ctx.map_cursor_idx); // 载入选中的地图
                
                // 屏幕中间打个提示框
                tft180_full(RGB565_WHITE);
                tft180_show_string(10, 80, "[ Map Loaded ]");
                system_delay_ms(200);
                
                ctx.current_page = MenuPage::DASHBOARD; // 自动跳回视图层看地图
                ctx.need_full_redraw = true; 
                App::g_state.debug.need_bg_redraw = true;
                ctx.ui_dirty = true;
            }
            // 按返回键：取消选择，退回 Vision 层
            if (ctx.key_back_pressed) { 
                ctx.current_page = MenuPage::DASHBOARD; 
                ctx.need_full_redraw = true;
                ctx.ui_dirty = true; 
            }
            break;

        default:
            if (ctx.key_back_pressed) { ctx.current_page = MenuPage::MAIN_MENU; ctx.need_full_redraw = true; ctx.ui_dirty = true; }
            break;
    }
}



// ===================================================== UI 渲染 =====================================================

// UI 渲染器：根据 current_page 调用对应的绘制函数
void render_ui() {
    if (!ctx.ui_dirty) return;
    if (ctx.need_full_redraw) { 
        tft180_full(RGB565_WHITE); 
        App::g_state.debug.need_bg_redraw = true;
        ctx.need_full_redraw = false; 
    }

    switch (ctx.current_page) {
        case MenuPage::MAIN_MENU:      draw_main_menu(); break;
        case MenuPage::DASHBOARD:      draw_dashboard(); break;
        case MenuPage::ODOMETRY_DATA:  draw_odometry_data(); break;
        case MenuPage::TUNE_PARAMS:    draw_tune_params(); break;
        case MenuPage::MAP_SELECT:     draw_map_select(); break;
    }
    ctx.ui_dirty = false; 
}

// 绘制主菜单
void draw_main_menu() {
    tft180_show_string(0, 0, "-- COMMAND MENU --");
    draw_item(2, "Dashboard",  ctx.cursor_idx == 0);
    draw_item(3, "Odometry",   ctx.cursor_idx == 1);
    draw_item(4, "Tuning",     ctx.cursor_idx == 2);
    draw_item(5, App::g_state.game.is_debug_mode ? "Mode: [Local ]" : "Mode: [VISION]", ctx.cursor_idx == 3);
    draw_item(6, "Save Config",ctx.cursor_idx == 4);
    draw_item(7, "Load Config",ctx.cursor_idx == 5);
    draw_item(8, "Close Menu", ctx.cursor_idx == 6); 
    draw_item(9, "forward",ctx.cursor_idx == 7);
    draw_item(10, "right", ctx.cursor_idx == 8); 
}

void draw_map_select() {
    tft180_show_string(0, 0, "-- SELECT MAP --");
    
    uint8_t count = TestMap::get_mock_map_count();

    // 进度提示 (例如 1/3)，放在右上角
    tft180_show_int(14 * UI_COL_W, 0, ctx.map_cursor_idx + 1, 2);
    tft180_show_string(16 * UI_COL_W, 0, "/");
    tft180_show_int(17 * UI_COL_W, 0, count, 2);
    
    // 利用已有的 PARAMS_PER_PAGE 行数限制渲染滚动列表
    for (int i = 0; i < PARAMS_PER_PAGE; i++) {
        int item_idx = ctx.map_scroll_offset + i;
        if (item_idx >= count) {
            // 清理多余行，打印 21 个空格覆盖一整行
            tft180_show_string(0, (i + 1) * UI_ROW_H, "                     ");
            continue;
        }

        draw_item(i + 1, TestMap::get_mock_map_name(item_idx), ctx.map_cursor_idx == item_idx);
    }
}

// 里程计和硬件监控页面
void draw_odometry_data() {
    tft180_show_string(0, 0, "-- ODO & HW --");
    auto pos = App::g_state.physical.pose;
    auto wheels = App::g_state.physical.current_wheel_speed;

    tft180_show_string(0, 2 * UI_ROW_H, "Global X: ");   tft180_show_float(10 * UI_COL_W, 2 * UI_ROW_H, pos.x, 3, 1);
    tft180_show_string(0, 3 * UI_ROW_H, "Global Y: ");   tft180_show_float(10 * UI_COL_W, 3 * UI_ROW_H, pos.y, 3, 1);
    char ui_buf[32]; sprintf(ui_buf, "Yaw: %8.2f   ", pos.yaw);     tft180_show_string(0, 4 * UI_ROW_H, ui_buf);

    tft180_show_string(0, 5 * UI_ROW_H, "Spd LF: ");     tft180_show_float(10 * UI_COL_W, 5 * UI_ROW_H, wheels.lf, 3, 1);
    tft180_show_string(0, 6 * UI_ROW_H, "Spd LB: ");     tft180_show_float(10 * UI_COL_W, 6 * UI_ROW_H, wheels.lb, 3, 1);
    tft180_show_string(0, 7 * UI_ROW_H, "Spd RF: ");     tft180_show_float(10 * UI_COL_W, 7 * UI_ROW_H, wheels.rf, 3, 1);
    tft180_show_string(0, 8 * UI_ROW_H, "Spd RB: ");     tft180_show_float(10 * UI_COL_W, 8 * UI_ROW_H, wheels.rb, 3, 1);
}

// 绘制参数调节页面
void draw_tune_params() {
    tft180_show_string(0, 0, "PARAMETERS");
    
    // 进度提示 (例如 1/8)，放在右上角
    tft180_show_int(14 * UI_COL_W, 0, ctx.cursor_idx + 1, 2);
    tft180_show_string(16 * UI_COL_W, 0, "/");
    tft180_show_int(17 * UI_COL_W, 0, DICT_SIZE, 2);
    
    for (int i = 0; i < PARAMS_PER_PAGE; i++) {
        int item_idx = ctx.scroll_offset + i;
        if (item_idx >= DICT_SIZE) {
            // 清理多余行，打印 21 个空格正好覆盖一整行
            tft180_show_string(0, (i + 1) * UI_ROW_H, "                     ");
            continue;
        }

        draw_float_item(i + 1, 
            tune_dict[item_idx].name, 
            *(tune_dict[item_idx].val_ptr), 
            ctx.cursor_idx == item_idx, 
            ctx.is_editing && (ctx.cursor_idx == item_idx));
    }
}

// 仪表盘页面：根据游戏状态动态显示信息
void draw_dashboard() {
    App::GameEngine::RenderContext render_ctx = App::GameEngine::get_render_context();
    auto& game = App::g_state.game;

    // 1. 局部组装字符串
    char hud_line0[22] = {0}, hud_line1[22] = {0}, hud_line2[22] = {0};
    snprintf(hud_line1, sizeof(hud_line1), "Stage: %d", game.is_advanced_stage ? 2 : 1);

    if (game.is_demo_mode) {
        switch(game.phase) {
            case GamePhase::WAIT_FOR_VISION:       snprintf(hud_line0, 22, "Phase: WAITING MAP"); break;
            case GamePhase::PLAN_PATROL:           snprintf(hud_line0, 22, "Phase: PLAN PATROL"); break;
            case GamePhase::ANIMATE_PATROL_DEMO:   snprintf(hud_line0, 22, "Phase: DEMO PATROL"); break;
            case GamePhase::BIND_SEMANTICS:        snprintf(hud_line0, 22, "Phase: BINDING    "); break;
            case GamePhase::PLAN_SOKOBAN:          snprintf(hud_line0, 22, "Phase: PLAN SOKO  "); break;
            case GamePhase::ANIMATE_DEMO:          snprintf(hud_line0, 22, "Phase: DEMO PUSH  "); break;
            case GamePhase::PLAN_RETURN_HOME:      snprintf(hud_line0, 22, "Phase: PLAN RTN   "); break;
            case GamePhase::ANIMATE_RETURN_DEMO:   snprintf(hud_line0, 22, "Phase: DEMO RTN   "); break;
            case GamePhase::FINISHED:              snprintf(hud_line0, 22, "Phase: FINISHED   "); break;
            case GamePhase::ERROR_OCCURRED:        snprintf(hud_line0, 22, "Phase: ERROR      "); break;
            default:                               snprintf(hud_line0, 22, "Phase: Other     "); break;
        }
        if (game.phase == GamePhase::ERROR_OCCURRED) {
            tft180_show_int (14 * UI_COL_W, 0, App::g_state.game.error_stage, 2);
        }
        if (game.phase == GamePhase::ANIMATE_PATROL_DEMO || game.phase == GamePhase::BIND_SEMANTICS || game.phase == GamePhase::PLAN_SOKOBAN) {
            snprintf(hud_line2, 22, "Bm:%3dms GT:%3dms", (int)render_ctx.bomb_plan_time_ms, (int)render_ctx.patrol_plan_time_ms);
        } else if (game.phase == GamePhase::ANIMATE_DEMO || game.phase == GamePhase::FINISHED) {
            snprintf(hud_line2, 22, "IDA* Time: %4dms", (int)render_ctx.push_plan_time_ms);
        } else {
            snprintf(hud_line2, 22, "Plan Time: --  ms");
        }
    } else {
        switch(game.phase) {
            case GamePhase::INIT_CALIBRATE:        snprintf(hud_line0, 22, "P: INIT      "); break;
            case GamePhase::EXIT_START_ZONE:       snprintf(hud_line0, 22, "P: EXIT 0    "); break;
            case GamePhase::WAIT_FOR_VISION:       snprintf(hud_line0, 22, "P: WAIT ART1 "); break;
            case GamePhase::EXEC_ACTION_DISPATCH:  snprintf(hud_line0, 22, "P: ACT_DISP  "); break;
            case GamePhase::EXEC_PATROL_MOVE:      snprintf(hud_line0, 22, "P: EXEC 2    "); break;
            case GamePhase::EXEC_ALIGN_YAW:        snprintf(hud_line0, 22, "P: EXEC YAW  "); break;
            case GamePhase::WAIT_ART2_CAPTURE_ACK: snprintf(hud_line0, 22, "P: WAIT ART2 "); break;
            case GamePhase::EXEC_BOMB_PUSH:        snprintf(hud_line0, 22, "P: EXEC 3    "); break;
            case GamePhase::EXEC_SOKOBAN:          snprintf(hud_line0, 22, "P: EXEC 1    "); break;
            case GamePhase::EXEC_RETURN_HOME:      snprintf(hud_line0, 22, "P: EXEC HOME "); break;
            case GamePhase::FINISHED:              snprintf(hud_line0, 22, "P: FINISHED  "); break;
            case GamePhase::ERROR_OCCURRED:        snprintf(hud_line0, 22, "P: ERROR     "); break;
            default:                               snprintf(hud_line0, 22, "P: COMPUTING "); break;
        }
        if (game.phase == GamePhase::ERROR_OCCURRED) {
            tft180_show_int (14 * UI_COL_W, 0, App::g_state.game.error_stage, 2);  // 错误阶段
        }
        snprintf(hud_line2, 22, "Plan Time: --  ms");
    }

    // 2. 顶部 HUD 防闪烁渲染
    static char last_hud0[22] = {0}, last_hud1[22] = {0}, last_hud2[22] = {0};
    
    // 如果外部触发了强制重绘，顺便把文字的记忆清空
    if (App::g_state.debug.need_bg_redraw) {
        last_hud0[0] = '\0'; last_hud1[0] = '\0'; last_hud2[0] = '\0';
    }

    if (strncmp(last_hud0, hud_line0, 22) != 0) {
        tft180_show_string(0, 0, "                     "); 
        tft180_show_string(0, 0, hud_line0);
        strncpy(last_hud0, hud_line0, 22);
    }
    if (strncmp(last_hud1, hud_line1, 22) != 0) {
        tft180_show_string(0, 1 * UI_ROW_H, "                     ");
        tft180_show_string(0, 1 * UI_ROW_H, hud_line1);
        strncpy(last_hud1, hud_line1, 22);
    }
    if (strncmp(last_hud2, hud_line2, 22) != 0) {
        tft180_show_string(0, 2 * UI_ROW_H, "                     ");
        tft180_show_string(0, 2 * UI_ROW_H, hud_line2);
        strncpy(last_hud2, hud_line2, 22);
    }

    // 3. 在内存中合成静态画布 （地图+目标+箱子+炸弹+路径+交叉点），一次性渲染到屏幕，避免多次调用绘制函数导致的闪烁
    uint8_t canvas[16][12] = {0};
    for(int y=0; y<MAP_MAX_HEIGHT; y++) for(int x=0; x<MAP_MAX_WIDTH; x++) if((*render_ctx.map)[y][x]) canvas[y][x] |= TL_WALL;
    for(int i=0; i<render_ctx.target_count; i++) if(render_ctx.targets[i].x != -1) canvas[render_ctx.targets[i].y][render_ctx.targets[i].x] |= TL_TGT;
    for(int i=0; i<render_ctx.box_count; i++)    if(render_ctx.boxes[i].x != -1)   canvas[render_ctx.boxes[i].y][render_ctx.boxes[i].x] |= TL_BOX;
    for(int i=0; i<render_ctx.bomb_count; i++)   if(render_ctx.bombs[i].x != -1)   canvas[render_ctx.bombs[i].y][render_ctx.bombs[i].x] |= TL_BOMB;
    
    if (render_ctx.path_ptr) {
        for(size_t i = render_ctx.path_start_idx; i < render_ctx.path_ptr->size(); i++) 
            canvas[(*render_ctx.path_ptr)[i].y][(*render_ctx.path_ptr)[i].x] |= TL_PATH;
    }
    if (render_ctx.actions_ptr) {
        for(size_t i = render_ctx.action_start_idx; i < render_ctx.actions_ptr->size(); i++) 
            if(!(*render_ctx.actions_ptr)[i].is_bomb_task) canvas[(*render_ctx.actions_ptr)[i].obs.pos.y][(*render_ctx.actions_ptr)[i].obs.pos.x] |= TL_CRS;
    }

    // 像素级小车独立计算系统 (Sprite Layer)
    int map_start_y = 3 * UI_ROW_H + 4;
    static float last_car_sx = -1.0f, last_car_sy = -1.0f;
    float current_car_sx = 0.0f, current_car_sy = 0.0f;
    
    // 只有在视觉模块完成，进入寻路阶段后才开始绘制小车。之前的阶段小车位置不确定，强行绘制反而会增加闪烁和视觉干扰
    bool should_draw_car = (game.phase > GamePhase::WAIT_FOR_VISION);

    if (should_draw_car) {
        if (game.is_demo_mode) {
            // 动画模式下，直接按网格坐标投射到屏幕像素 (8个像素一格)
            current_car_sx = render_ctx.player_pos.y * 8.0f;
            current_car_sy = render_ctx.player_pos.x * 8.0f + map_start_y;
        } else {
            auto pos = App::g_state.physical.pose;
            // X轴物理坐标对应屏幕上的 Y方向 (sx)，Y轴对应 X方向 (sy)
            // 物理网格 1格 = 20cm，屏幕上 1格 = 8像素。缩放系数为 8/20 = 0.4
            current_car_sx = (pos.y - SystemConfig::MAP_OFFSET_Y) * 0.4f;
            current_car_sy = (pos.x - SystemConfig::MAP_OFFSET_X) * 0.4f + map_start_y;
            
            // 安全限幅防越界
            if (current_car_sx < 0) current_car_sx = 0; else if (current_car_sx > 15*8) current_car_sx = 15*8;
            if (current_car_sy < map_start_y) current_car_sy = map_start_y; else if (current_car_sy > map_start_y + 11*8) current_car_sy = map_start_y + 11*8;
        }

        // 精髓：擦除小车的残影！强制小车上一帧所在的周边底图缓存失效，底图自动将其覆盖
        if (last_car_sx >= 0.0f) {
            fill_rect((int)last_car_sx + 2, (int)last_car_sy + 2, 4, 4, RGB565_WHITE);
            
            int old_gy = (int)last_car_sx / 8;
            int old_gx = (int)(last_car_sy - map_start_y) / 8;
            for(int dy = 0; dy <= 1; dy++) {
                for(int dx = 0; dx <= 1; dx++) {
                    if (old_gy + dy < 16 && old_gx + dx < 12) {
                        ctx.back_buffer[old_gy + dy][old_gx + dx] = 0xFF; // 强行设为无效，触发重绘
                    }
                }
            }
        }
    }

    // 4. O(1) 脏矩形底图增量渲染 
    if (App::g_state.debug.need_bg_redraw) { 
        memset(ctx.back_buffer, 0xFF, sizeof(ctx.back_buffer)); // 失效显存，强制全刷
        
        // 静态背景上的附加涂装：彩色炸弹框
        if (render_ctx.bomb_tasks_ptr) {
            uint16_t b_colors[] = {RGB565_RED, RGB565_BLUE, RGB565_CYAN, RGB565_MAGENTA}; 
            for (int i = 0; i < render_ctx.bomb_tasks_ptr->size(); ++i) {
                uint16_t color = b_colors[i % 4];
                point bs = (*render_ctx.bomb_tasks_ptr)[i].bomb_start;
                point tw = (*render_ctx.bomb_tasks_ptr)[i].target_wall;

                int bs_sx = bs.y * 8, bs_sy = map_start_y + bs.x * 8;
                tft180_draw_line(bs_sx, bs_sy, bs_sx + 7, bs_sy, color);
                tft180_draw_line(bs_sx, bs_sy + 7, bs_sx + 7, bs_sy + 7, color);
                tft180_draw_line(bs_sx, bs_sy, bs_sx, bs_sy + 7, color);
                tft180_draw_line(bs_sx + 7, bs_sy, bs_sx + 7, bs_sy + 7, color);

                if ((*render_ctx.map)[tw.y][tw.x] == 1) { // 墙还没被炸毁的话，画框
                    int tw_sx = tw.y * 8, tw_sy = map_start_y + tw.x * 8;
                    tft180_draw_line(tw_sx, tw_sy, tw_sx + 7, tw_sy, color);
                    tft180_draw_line(tw_sx, tw_sy + 7, tw_sx + 7, tw_sy + 7, color);
                    tft180_draw_line(tw_sx, tw_sy, tw_sx, tw_sy + 7, color);
                    tft180_draw_line(tw_sx + 7, tw_sy, tw_sx + 7, tw_sy + 7, color);
                }
            }
        }
        App::g_state.debug.need_bg_redraw = false; 
    }

    // 核心底图渲染循环
    for(int y=0; y<16; y++) {
        for(int x=0; x<12; x++) {
            if (canvas[y][x] != ctx.back_buffer[y][x]) {  
                int sx = y * 8, sy = x * 8 + map_start_y;
                
                fill_rect(sx + 1, sy + 1, 7, 7, (canvas[y][x] & TL_WALL) ? RGB565_GRAY : RGB565_WHITE);
                
                if (canvas[y][x] & TL_TGT)  fill_rect(sx + 1, sy + 1, 6, 6, RGB565_PURPLE);
                if (canvas[y][x] & TL_PATH) fill_rect(sx + 3, sy + 3, 2, 2, RGB565_BLUE);
                if (canvas[y][x] & TL_CRS)  { tft180_draw_line(sx+2, sy+2, sx+6, sy+6, RGB565_BLUE); tft180_draw_line(sx+2, sy+6, sx+6, sy+2, RGB565_BLUE); }
                
                if (canvas[y][x] & TL_BOX)  fill_rect(sx + 1, sy + 1, 6, 6, RGB565_YELLOW);
                if (canvas[y][x] & TL_BOMB) { fill_rect(sx + 1, sy + 1, 6, 6, RGB565_BLACK); fill_rect(sx + 3, sy + 3, 2, 2, RGB565_RED); }
                
                ctx.back_buffer[y][x] = canvas[y][x]; 
            }
        }
    }

    // 5. 最后在底图之上，绘制自由滑行的小车
    if (should_draw_car) {
        // float 转 int，+2 是为了让 4x4 的小车正好居中在一个 8x8 的格子里
        fill_rect((int)current_car_sx + 2, (int)current_car_sy + 2, 4, 4, RGB565_GREEN);
        
        // 记录历史位置，用于下一帧擦除
        last_car_sx = current_car_sx;
        last_car_sy = current_car_sy;
    } else {
        last_car_sx = -1.0f; // 重置小车状态
    }
}

// ================= 基础绘图辅助函数 ===================

void draw_item(uint8_t row, const char* name, bool is_selected) {
    if (is_selected) tft180_show_string(0, row * UI_ROW_H, ">"); 
    else tft180_show_string(0, row * UI_ROW_H, " "); 
    // 名称从第 2 列开始写
    tft180_show_string(1 * UI_COL_W, row * UI_ROW_H, (char*)name);
}

void draw_float_item(uint8_t row, const char* name, float val, bool is_selected, bool is_editing_this) {
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

// ================= 基础绘图辅助函数 ===================

void fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color) {
    for (uint8_t i = 0; i < h; ++i) {
        tft180_draw_line(x, y + i, x + w - 1, y + i, color);
    }
}

} // namespace Subsystem::Display
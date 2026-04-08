#pragma once
#include "zf_common_headfile.h"
#include <stdint.h>

#define KEY1 C14  // 下移 / 减小
#define KEY2 C12  // 确认 / 编辑模式
#define KEY3 C15  // 上移 / 增大
#define KEY4 C13  // 返回 / 退出编辑


// 页面枚举
enum class MenuPage {
    // ---菜单组 (可选)---
    MAIN_MENU,          // 主菜单
    MAP_SELECT,         // 地图选择

    // --- 状态监控组 (只读) ---
    DASHBOARD,          // 全局状态监控 (游戏进程、地图信息、规划展示等)
    ODOMETRY_DATA,      // 里程计+硬件监控 (全局位姿、编码器速度、IMU)

    // --- 参数调节组 (可编辑) ---
    TUNE_PARAMS
};

class TftMenu {
public:
    void init();
    void run(); 

private:
    MenuPage current_page;          // 当前页面
    uint8_t cursor_idx;             // 当前选中行
    uint8_t scroll_offset;          // 用于参数列表的上下滚动翻页
    uint8_t map_cursor_idx;
    uint8_t map_scroll_offset;
    bool is_editing;                // 是否处于编辑模式 (在调节页面中使用)
    bool need_full_redraw;          // 是否需要全屏重绘 (页面切换时为 true，局部刷新时为 false)
    bool ui_dirty;                  // UI 是否需要更新（按键触发或监控页面强制刷新）
    bool is_closed;                 // 息屏标志位

    bool key_up_pressed, key_down_pressed, key_enter_pressed, key_back_pressed;    // 按键状态
    bool last_k1, last_k2, last_k3, last_k4;                                       // 上一轮按键状态（用于边缘检测）

    // --- 页面逻辑函数 ---
    void scan_keys();               // 扫描按键状态，更新 key_xxx_pressed_ 变量
    void process_logic();           // 根据当前页面和按键状态处理逻辑，更新 current_page、cursor_idx、is_editing 等变量
    void render_ui();               // 根据 current_page 和相关状态变量绘制 UI

    // --- 页面绘制函数 ---
    void draw_main_menu();          // 主菜单
    void draw_dashboard();          // 全局状态监控页
    void draw_map_select();         // 绘制地图选择页
    void draw_odometry_data();      // 里程计+硬件数据监控页
    void draw_tune_params();        // 参数调节页

    // --- 局部刷新辅助 ---
    void draw_item(uint8_t row, const char* name, bool is_selected);  // 绘制菜单项，选中时高亮
    void draw_float_item(uint8_t row, const char* name, float val, bool is_selected, bool is_editing);  // 绘制带数值的菜单项，选中时高亮，编辑模式下数值闪烁提示

    // --- 基础绘图接口封装 ---
    void fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color); //  快速矩形填充
};

extern TftMenu sys_menu;
extern bool is_debug_mode; // 全局调试模式标志，控制是否启用测试功能和显示额外信息
#include "GameManage.h"
#include "tuning_config.h"
#include "ChassisControl.h"
#include "PoseEstimate.h"
#include "Vision.h"
#include "CoreScheduler.h"
#include "Tracker.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include "zf_common_headfile.h"

namespace App::GameEngine {

DTCM_DATA static DemoGameManager core_engine;
DTCM_DATA static GamePhase s_last_update_phase = GamePhase::NONE;

// 倒数计算宏区（利用编译期算好替代除法）
constexpr float INV_360 = 1.0f / 360.0f;
constexpr float INV_GRID_SIZE_CM = 1.0f / SystemConfig::GRID_SIZE_CM;

namespace {
    constexpr point RETURN_HOME_GRID = {
        static_cast<int8_t>((SystemConfig::ENTRY_X - SystemConfig::MAP_OFFSET_X) /
                            SystemConfig::GRID_SIZE_CM),
        static_cast<int8_t>((SystemConfig::ENTRY_Y - SystemConfig::MAP_OFFSET_Y) /
                            SystemConfig::GRID_SIZE_CM)
    };
    constexpr point RETURN_EXIT_GRID = {
        static_cast<int8_t>(SystemConfig::PLAN_START_X),
        static_cast<int8_t>(SystemConfig::PLAN_START_Y)
    };
    constexpr float RETURN_HOME_YAW = SystemConfig::ENTRY_YAW;

    // 三关模式轮次表：round 0 纯推箱，round 1/2 识别（巡图+推箱）。
    // 单关模式不使用 round 0 的纯推箱配置，而是在 init() 中直接设为 advanced。
    constexpr bool ROUND_ADVANCED_SEQ[] = {false, true, true};
    constexpr uint8_t MAX_ROUND_COUNT =
        static_cast<uint8_t>(sizeof(ROUND_ADVANCED_SEQ) / sizeof(ROUND_ADVANCED_SEQ[0]));
    constexpr float RETURN_FINAL_YAW_TOLERANCE_DEG = 2.0f;
    constexpr float RETURN_FINAL_SPIN_DEG = 360.0f;
    constexpr float RETURN_EXIT_ODOM_REACH_RADIUS_CM = 3.0f;
    constexpr float RETURN_EXIT_VISUAL_REACH_RADIUS_CM = 8.0f;
    constexpr uint32_t RETURN_POSE_RECENT_MS = 300U;
    constexpr uint32_t RETURN_POSE_REQUEST_INTERVAL_MS = 150U;
    constexpr uint32_t ART1_RETRY_INTERVAL_MS = 1000U;
    constexpr uint32_t ART1_MAP_CAPTURE_DELAY_MS = 200U;  // 到观测点停稳后等待画面切换
    constexpr uint32_t ART1_MAP_SETTLE_MS = 50U;
    constexpr uint32_t RETURN_HOME_DWELL_DEFAULT_MS = 120U;
    constexpr uint32_t RETURN_HOME_DWELL_LONG_MS = 4000U;

    DTCM_DATA uint32_t s_return_pose_request_tick_ms = 0U;
    DTCM_DATA bool s_return_final_align_started = false;
    DTCM_DATA bool s_return_final_spin_started = false;
    DTCM_DATA bool s_return_home_dwell_active = false;
    DTCM_DATA bool s_return_exit_started = false;
    DTCM_DATA uint32_t s_return_home_dwell_start_ms = 0U;
    DTCM_DATA uint32_t s_art1_request_tick_ms = 0U;
    DTCM_DATA uint32_t s_art1_map_settle_start_tick_ms = 0U;
    DTCM_DATA uint32_t s_return_route_pose_seq_start = 0U;
    DTCM_DATA uint32_t s_art1_capture_wait_start_ms = 0U;
    DTCM_DATA bool s_art1_map_request_sent = false;

    // 炸弹按需等待爆炸：记录"这次爆炸真正会清开的墙格集合"，下一条路径若踩到其中任一格才等爆炸。
    // 存被清开的墙格(而非 blast_wall 单点)有两个原因：
    //  1) 精确——3×3 里多数格本就是空地，只有原本是墙、靠这次爆炸清开的格才需要等；踩邻域空地不等；
    //  2) 时序——爆破 apply 会把这些墙当场清成 0，故必须在 apply 前捕获，之后无法再从地图反查。
    DTCM_DATA StaticArray<point, 9> s_pending_blast_cells;  // 空 = 无待处理炸弹
    DTCM_DATA bool s_explosion_wait_active = false;    // 当前是否正在等爆炸
    DTCM_DATA uint32_t s_explosion_wait_start_ms = 0U; // 等待起始时刻
    DTCM_DATA bool s_sokoban_solution_ready = false;   // 等爆炸期间复用已求出的最终路径

    // 在爆破 apply 之前调用：扫描 blast_wall 的 3×3，把其中当前仍是墙(map!=0)的格捕获进
    // s_pending_blast_cells。这些正是"下一步只有等这次爆炸炸开才能通过"的格子。
    [[gnu::always_inline]] inline void capture_blast_cells(point blast_wall, const SokobanLevel& level) {
        s_pending_blast_cells.clear();
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                int gx = blast_wall.x + dx;
                int gy = blast_wall.y + dy;
                if (gy < 0 || gy >= SystemConfig::MAP_MAX_HEIGHT ||
                    gx < 0 || gx >= SystemConfig::MAP_MAX_WIDTH) continue;
                if (level.map[gy][gx] != 0) {
                    s_pending_blast_cells.push_back({static_cast<int8_t>(gx), static_cast<int8_t>(gy)});
                }
            }
        }
    }

    // 路径是否踩到"这次爆炸才会清开的墙格"。只要路径任一格命中捕获的墙格集合即为真。
    // 车绕墙而行、只擦过墙的邻域空地时不命中 → 不需要等(见图示轨迹绕开墙却曾误等 1 秒)。
    [[gnu::always_inline]] inline bool segment_needs_blast(
        point start,
        const StaticArray<point, SystemConfig::MAX_PATH_LENGTH>& seg) {
        for (int j = 0; j < s_pending_blast_cells.size(); ++j) {
            if (PlanningCommon::path_crosses_cell(
                    start, seg, s_pending_blast_cells[j])) return true;
        }
        return false;
    }

    // 路径加载前调用：刚推的炸弹若炸开的墙挡住即将执行的路径段 seg，则先原地等爆炸。
    // 返回 true = 仍需等待（调用方应保持原地、暂不加载路径）；false = 可放行加载。
    [[gnu::always_inline]] inline bool gate_explosion_before_path(
        point start,
        const StaticArray<point, SystemConfig::MAX_PATH_LENGTH>& seg) {
        if (s_pending_blast_cells.size() == 0) return false;  // 无待处理炸弹

        if (!s_explosion_wait_active) {
            if (!segment_needs_blast(start, seg)) {
                s_pending_blast_cells.clear();         // 路径不靠这次爆炸开路：不等，直奔目标
                return false;
            }
            s_explosion_wait_active = true;            // 需穿墙：启动固定时长等待
            s_explosion_wait_start_ms = Core::Scheduler::get_sys_tick_ms();
            // 等待期间锁住当前位置，避免车继续往墙冲
            Algorithm::Tracker::track_point(
                {App::g_state.physical.pose.x, App::g_state.physical.pose.y, App::g_state.physical.pose.yaw});
            // 爆炸闪光会污染视觉坐标，等待窗口内屏蔽视觉修正，只靠锁位+里程计扛过
            Algorithm::Tracker::set_vision_correction_suppressed(true);
        }

        uint32_t wait_ms = (uint32_t)tune.wain_time.explosion_wait_ms;
        if (Core::Scheduler::get_sys_tick_ms() - s_explosion_wait_start_ms >= wait_ms) {
            s_explosion_wait_active = false;
            s_pending_blast_cells.clear();
            Algorithm::Tracker::set_vision_correction_suppressed(false);  // 闪光过去，恢复视觉修正
            return false;  // 等待结束，放行加载
        }
        return true;  // 仍在等待
    }

    [[gnu::always_inline]] inline float yaw_error_abs_deg(float target, float current) {
        float diff = target - current;
        return std::abs(diff - 360.0f * std::roundf(diff * INV_360));
    }

    [[gnu::always_inline]] inline void request_return_pose_sample(uint32_t now) {
        if (s_return_pose_request_tick_ms == 0U ||
            now - s_return_pose_request_tick_ms >= RETURN_POSE_REQUEST_INTERVAL_MS) {
            s_return_pose_request_tick_ms = now;
            Subsystem::Vision::schedule_pose_request_ART1();
        }
    }

    // 一次新的 ART1 采集必须丢弃上一关的地图和位姿帧，避免旧帧直接放行下一关
    [[gnu::always_inline]] inline void reset_art1_acquisition_state() {
        auto& vision = App::g_state.vision;
        vision.art1_map_ready = false;
        vision.art1_pose_updated = false;
        vision.art1_pose_applied = false;
        vision.art1_pose_request_pending = false;
        vision.art1_pose_seq = 0U;
        vision.art1_pose_tick_ms = 0U;
        vision.art1_pose_stable_count = 0U;
        s_art1_request_tick_ms = 0U;
        s_art1_capture_wait_start_ms = 0U;
        s_art1_map_request_sent = false;
        s_art1_map_settle_start_tick_ms = Core::Scheduler::get_sys_tick_ms();
    }

    // 校验 ART1 地图，拒绝空白帧、数量不完整帧和越界坐标帧
    [[gnu::always_inline]] inline bool art1_map_is_valid() {
        const auto& vision = App::g_state.vision;
        if (vision.box_count == 0U ||
            vision.box_count > SystemConfig::MAX_BOXES ||
            vision.target_count != vision.box_count ||
            vision.target_count > SystemConfig::MAX_BOXES ||
            vision.bomb_count > SystemConfig::MAX_BOMBS) {
            return false;
        }

        bool has_wall = false;
        bool has_floor = false;
        for (int y = 0; y < SystemConfig::MAP_MAX_HEIGHT; ++y) {
            for (int x = 0; x < SystemConfig::MAP_MAX_WIDTH; ++x) {
                const int8_t cell = vision.map[y][x];
                if (cell == 0) {
                    has_floor = true;
                } else if (cell == 1) {
                    has_wall = true;
                } else {
                    return false;
                }
            }
        }
        if (!has_wall || !has_floor) return false;

        for (int i = 0; i < vision.box_count; ++i) {
            const point box = vision.boxes[i];
            const point target = vision.targets[i];
            if (!PlanningCommon::in_bounds(box) ||
                !PlanningCommon::in_bounds(target) ||
                vision.map[box.y][box.x] != 0 ||
                vision.map[target.y][target.x] != 0) {
                return false;
            }
        }
        for (int i = 0; i < vision.bomb_count; ++i) {
            const point bomb = vision.bombs[i];
            if (!PlanningCommon::in_bounds(bomb) || vision.map[bomb.y][bomb.x] != 0) {
                return false;
            }
        }
        return true;
    }

    [[gnu::always_inline]] inline bool get_recent_stable_art1_pose(
        uint32_t now, Pose2D& out_pose) {
        const auto& vision = App::g_state.vision;
        if (vision.art1_pose_seq == 0U ||
            vision.art1_pose_seq <= s_return_route_pose_seq_start ||
            vision.art1_pose_stable_count < App::ART1_POSE_STABLE_REQUIRED_FRAMES ||
            now - vision.art1_pose_tick_ms > RETURN_POSE_RECENT_MS) {
            return false;
        }
        out_pose = vision.art1_pose_buffer[vision.art1_pose_publish_idx];
        return std::isfinite(out_pose.x) && std::isfinite(out_pose.y);
    }

    [[gnu::always_inline]] inline bool visual_reached_return_exit(uint32_t now) {
        Pose2D vision_pose;
        if (!get_recent_stable_art1_pose(now, vision_pose)) return false;
        const float dx = vision_pose.x - OUT_TARGET_X;
        const float dy = vision_pose.y - OUT_TARGET_Y;
        return dx * dx + dy * dy <=
               RETURN_EXIT_VISUAL_REACH_RADIUS_CM * RETURN_EXIT_VISUAL_REACH_RADIUS_CM;
    }

    // 返航终点不再重新下发追踪目标，只把当前路径锁死在当前位置等待视觉采集
    [[gnu::always_inline]] inline void hold_return_position() {
        auto& pos = App::g_state.physical.pose;
        auto& ctrl = App::g_state.control;
        ctrl.current_target = pos;
        ctrl.segment_end_speed = 0.0f;
        ctrl.tracker_state = TrackerState::FINISHED;
        ctrl.mode = ControlMode::AUTO_TRACKING;
        ctrl.hard_lock = true;
    }

    [[gnu::always_inline]] inline void request_art1_map_and_pose(uint32_t now) {
        Subsystem::Vision::request_map_ART1();
        Subsystem::Vision::request_pose_ART1();
        s_art1_request_tick_ms = now;
    }

    // 视觉帧稳定后只执行一次，XY 同时覆盖融合位姿和编码器历史；航向继续采用陀螺估计
    [[gnu::always_inline]] inline bool apply_stable_art1_pose() {
        auto& vision = App::g_state.vision;
        if (vision.art1_pose_applied ||
            vision.art1_pose_seq == 0U ||
            vision.art1_pose_stable_count < App::ART1_POSE_STABLE_REQUIRED_FRAMES) {
            return vision.art1_pose_applied;
        }

        const Pose2D visual_pose = vision.art1_pose_buffer[vision.art1_pose_publish_idx];
        if (!std::isfinite(visual_pose.x) || !std::isfinite(visual_pose.y)) {
            return false;
        }

        const float current_yaw = App::g_state.physical.pose.yaw;
        Subsystem::PoseEstimator::set_position(visual_pose.x, visual_pose.y, current_yaw);
        vision.art1_pose_applied = true;
        return true;
    }

    [[gnu::always_inline]] inline void start_return_home_route() {
        StaticArray<point, SystemConfig::MAX_PATH_LENGTH> route;
        route.push_back(RETURN_HOME_GRID);
        Algorithm::Tracker::load_path_with_vision_assist(route);
        App::g_state.control.current_target.yaw = RETURN_HOME_YAW;
    }

    [[gnu::always_inline]] inline void start_return_exit_route() {
        StaticArray<point, SystemConfig::MAX_PATH_LENGTH> route;
        route.push_back(RETURN_EXIT_GRID);
        Algorithm::Tracker::load_path_with_vision_assist(route);
        App::g_state.control.current_target.yaw = RETURN_HOME_YAW;
    }
}

//===================================================================
// GameEngine 模块对外接口实现
// ==================================================================

/// \brief 切换连续关卡返航后的停顿时长档位
void toggle_return_home_dwell() {
    const float current = tune.wain_time.return_home_dwell_ms;
    const uint32_t primask = interrupt_global_disable();
    tune.wain_time.return_home_dwell_ms =
        (std::isfinite(current) && current >= 2000.0f) ?
            static_cast<float>(RETURN_HOME_DWELL_DEFAULT_MS) :
            static_cast<float>(RETURN_HOME_DWELL_LONG_MS);
    interrupt_global_enable(primask);
}

/// \brief 获取当前返航停顿时长
/// \return 当前选择的停顿时长 ms
uint32_t get_return_home_dwell_ms() {
    float dwell = tune.wain_time.return_home_dwell_ms;
    if (!std::isfinite(dwell) || dwell < 0.0f) {
        dwell = DEFAULT_TUNE_CONFIG.wain_time.return_home_dwell_ms;
    }
    return static_cast<uint32_t>(dwell);
}

/// \brief 初始化比赛管理器入口状态
///
/// \details
/// 读取两路拨码开关，确定赛段模式和 Debug 模式
/// Debug 模式会停在 NONE，等待屏幕选图后再启动流程
///
void init() {
    gpio_init(C27, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(C26, GPI, GPIO_HIGH, GPI_PULL_UP);
    system_delay_ms(10); 

    bool sw2_on = !gpio_get_level(C27);
    bool sw1_on = !gpio_get_level(C26);

    // sw1 开：连续跑 3 关，按 [纯推箱, 识别, 识别] 轮次表执行
    // sw1 关：只跑 1 关，直接按 is_advanced_stage 识别关执行
    App::g_state.game.round_idx         = 0;
    App::g_state.game.round_count       = sw1_on ? MAX_ROUND_COUNT : 1U;
    App::g_state.game.is_advanced_stage = sw1_on ? ROUND_ADVANCED_SEQ[0] : true;
    App::g_state.game.is_debug_mode     = sw2_on;  // 调试模式：开-直接注入地图数据，绕过视觉输入；关-正常模式，等待视觉输入
    s_return_home_dwell_active = false;
    s_return_exit_started = false;
    s_return_home_dwell_start_ms = 0U;

    if (App::g_state.game.is_debug_mode) {
        App::g_state.game.phase = GamePhase::NONE;  // 直接进入初始状态，UI 选完地图后再切到正常流程
    }

    App::g_state.debug.need_bg_redraw = true;
}

/// \brief GameEngine 全局周期入口
///
/// \details
/// 根据运行模式把 update 分发给正式流程、Mock 流程或 Demo 动画流程
/// Debug 选图前保持挂起，避免真实控制流提前启动
///
__attribute__((section(".ramfunc"))) void update() {
    auto& game = App::g_state.game;

    // 【机制挂起】：如果是 Debug 模式，在 UI 选完地图前，强制挂起状态机，不进入物理控制流
    if (game.is_debug_mode && game.phase == GamePhase::NONE) {
        return; 
    }

    if (game.is_demo_mode) {
        core_engine.update();                   // 纯动画推演 (Demo)
    } else if (game.is_debug_mode) {
        core_engine.MockGameManager::update();  // 实体车+脱机地图+注入语义 (Mock)
    } else {
        core_engine.GameManager::update();      // 正常比赛流程 (Prod)
    }
}

// 获取当前渲染上下文 [Display 层只读取这个接口，不直接访问全局状态]
RenderContext get_render_context() { 
    return core_engine.get_render_context();
}


//===================================================================
// GameManager 基类实现
//===================================================================

/// \brief 正式比赛业务状态机
///
/// \details
/// 主状态机串起出库建图、巡图观测、语义绑定、推箱求解和返航
/// 底层移动由 RobotTask 队列拆分后交给 Tracker 和 Chassis 执行
///
__attribute__((section(".ramfunc"))) void GameManager::update() {
    auto& pos = App::g_state.physical.pose;
    auto& ctrl = App::g_state.control;       
    auto& game = App::g_state.game;          
    auto& vision_data = App::g_state.vision;
    const GamePhase phase_at_entry = game.phase;
    const bool phase_entered = (phase_at_entry != s_last_update_phase);
    
    switch (game.phase) {
        // =============================================================================
        // ---- 阶段一：启动出库与建图 ----
        // =============================================================================
        case GamePhase::INIT_CALIBRATE: {
            // 一次性复位，连续关卡返航后从发车区重新进入出库流程
            if (phase_entered) {
                reset_art1_acquisition_state();
                // 正常只在首轮进入，保留 round 分支供调试时从中间轮次手动重启
                const float start_x = (game.round_idx == 0) ? INITIAL_X : ENTRY_X;
                const float start_y = (game.round_idx == 0) ? INITIAL_Y : ENTRY_Y;
                Subsystem::PoseEstimator::set_position(start_x, start_y, ENTRY_YAW);
                Algorithm::Tracker::track_point({start_x, start_y, ENTRY_YAW});

                // 复位炸弹爆炸等待与视觉抑制，防止上一局异常中断后抑制标志卡死导致视觉永久失效
                s_pending_blast_cells.clear();
                s_explosion_wait_active = false;
                s_sokoban_solution_ready = false;
                Algorithm::Tracker::set_vision_correction_suppressed(false);
                Subsystem::Vision::finish_capture_ART2();
            }

            // 首轮正常发车去观测建图点
            Algorithm::Tracker::track_point({OUT_TARGET_X, OUT_TARGET_Y, ENTRY_YAW});
            game.phase = GamePhase::EXIT_START_ZONE;
            break;
        }

        case GamePhase::EXIT_START_ZONE: {
            // 到位停稳后先进入画面切换等待，再请求 ART1 地图和位姿
            if (Algorithm::Tracker::check_arrival({OUT_TARGET_X, OUT_TARGET_Y}, tune.tracker.reach_radius_min) &&
                App::g_state.physical.is_stopped) {
                // 到观测区后保持当前位置，不能在等待期间继续追点
                hold_return_position();
                reset_art1_acquisition_state();
                s_art1_capture_wait_start_ms = Core::Scheduler::get_sys_tick_ms();
                game.phase = GamePhase::WAIT_FOR_VISION;
            }
            break;
        }

        case GamePhase::WAIT_FOR_VISION: {
            // 等待地图和停稳后的新视觉位姿，两个条件缺一不可
            const uint32_t now = Core::Scheduler::get_sys_tick_ms();

            // 正式模式先丢弃等待窗口内到达的旧地图，满 100ms 后才发起本轮请求
            if (!game.is_debug_mode && !s_art1_map_request_sent) {
                vision_data.art1_map_ready = false;
                if (s_art1_capture_wait_start_ms == 0U) {
                    s_art1_capture_wait_start_ms = now;
                }
                if (now - s_art1_capture_wait_start_ms < ART1_MAP_CAPTURE_DELAY_MS) {
                    break;
                }
                request_art1_map_and_pose(now);
                s_art1_map_request_sent = true;
                break;
            }

            if (vision_data.art1_map_ready && !art1_map_is_valid()) {
                // 地图帧无效时释放首帧锁，按原重试周期重新取图
                vision_data.art1_map_ready = false;
            }

            const bool pose_recent = vision_data.art1_pose_seq != 0U &&
                now - vision_data.art1_pose_tick_ms <= RETURN_POSE_RECENT_MS;
            if (vision_data.art1_map_ready &&
                vision_data.art1_pose_seq != 0U &&
                vision_data.art1_pose_stable_count >= App::ART1_POSE_STABLE_REQUIRED_FRAMES &&
                pose_recent) {
                apply_stable_art1_pose();
                if (!vision_data.art1_pose_applied) {
                    break;
                }

                // 地图帧就绪，将视觉数据转换加载到逻辑地图结构中，供后续算法使用
                logical_level.map = vision_data.map;
                logical_level.player_start = {PLAN_START_X, PLAN_START_Y};
                logical_level.box_count = vision_data.box_count;
                logical_level.target_count = vision_data.target_count;
                logical_level.bomb_count = vision_data.bomb_count;

                for(int i=0; i<logical_level.box_count; ++i) {
                    logical_level.boxes[i] = vision_data.boxes[i];
                    logical_level.targets[i] = vision_data.targets[i];
                }
                for(int i=0; i<logical_level.bomb_count; ++i) {
                    logical_level.bombs[i] = vision_data.bombs[i];
                }

                // 刚开始先把箱子和目标点都归到同一语义组，如果是第一阶段不做改变；如果是第二阶段，后续会根据视觉识别结果重新绑定语义
                for(int i=0; i<logical_level.box_count; ++i) {
                    logical_level.box_semantics[i] = 0;
                }
                for(int i=0; i<logical_level.target_count; ++i) {
                    logical_level.target_semantics[i] = 0;
                }

                if (game.is_advanced_stage) {  
                    game.phase = GamePhase::PLAN_PATROL;       // 进入巡图
                } else {
                    if (!solver.load_from_vision(
                            logical_level,
                            nullptr,
                            0,
                            SokobanHeuristicMode::PURE) ||
                        !solver.bind_semantics()) {
                        game.error_stage = 6;
                        game.phase = GamePhase::ERROR_OCCURRED;
                        break;
                    }
                    s_sokoban_solution_ready = false;
                    game.phase = GamePhase::PLAN_SOKOBAN;      // 直接进入推箱子阶段
                }
            } else {
                // 到达观测点并完全停稳后，再给相机 50 ms 响应窗口后发起采集
                if (now - s_art1_map_settle_start_tick_ms >= ART1_MAP_SETTLE_MS &&
                    (s_art1_request_tick_ms == 0U ||
                     now - s_art1_request_tick_ms >= ART1_RETRY_INTERVAL_MS)) {
                    request_art1_map_and_pose(now);
                }
            }
            break;
        }

        // =============================================================================
        // ---- 阶段二：巡图与任务派发 ----
        // =============================================================================
        case GamePhase::PLAN_PATROL: {
            // 解算炸弹任务（无炸弹时自动退化）
            auto& bombs = App::g_state.planning.bomb_tasks;
            bombs = strategic_planner.plan_phase1_bombs(logical_level);

            // 规划巡图路径，得到宏观动作参考序列
            patrol_planner.load_level(logical_level);
            reference_patrol_actions = patrol_planner.plan_optimal_patrol(logical_level.player_start, bombs, ENTRY_YAW, 0);

            // 初始化实时规划器，加载参考序列，准备进入宏观动作调度阶段
            macro_planner.reset(logical_level);
            macro_planner.set_reference_plan(reference_patrol_actions);

            // 重置观测状态
            observed_mask = 0;
            current_observe_yaw = ENTRY_YAW;
            executed_patrol_actions.clear();
            game.action_idx = 0;

            Subsystem::Vision::reset_semantic_labels();   // 清空语义池，准备重新填充
            art2_capture_request_sent = false;
            game.phase = GamePhase::EXEC_ACTION_DISPATCH;
            break;
        }

        case GamePhase::EXEC_ACTION_DISPATCH: {
            // 每次派发新动作前，先同步一次视觉模块已经识别的语义标签，刷新算法内部的配对状态
            macro_planner.sync_semantics(vision_data.semantic_labels);

            // 如果参考序列完成且语义信息足够，进入推箱子阶段；否则继续派发下一个宏动作
            if (macro_planner.ready_for_sokoban(logical_level)) {
                game.phase = GamePhase::BIND_SEMANTICS;
            } else {
                MacroAction act;
                if (!plan_next_macro_action(act)) {
                    macro_planner.sync_semantics(vision_data.semantic_labels);
                    if (macro_planner.ready_for_sokoban(logical_level)) {
                        game.phase = GamePhase::BIND_SEMANTICS;
                        break;
                    }
                    game.error_stage = 1;
                    game.phase = GamePhase::ERROR_OCCURRED;
                    break;
                }

                // 将宏动作转换成底层任务序列并加载到调度器
                start_macro_action(act);
                game.phase = GamePhase::EXEC_TASK_QUEUE; // 转移到微观流水线
            }
            break;
        }

        case GamePhase::EXEC_TASK_QUEUE: {
            if (current_task_idx >= task_queue.size()) {
                game.phase = GamePhase::EXEC_ACTION_DISPATCH; // 当前宏任务完成，切回 DISPATCH
                break;
            }

            auto& task = task_queue[current_task_idx];
            bool task_done = false;

            switch (task.type) {
                case TaskType::LOAD_PATH_BOMB: {
                    StaticArray<point, MAX_PATH_LENGTH> segment;
                    BombTask bomb = make_bomb_task(task.param.bomb_push);
                    if (PlanningCommon::get_bomb_push_path(logical_level, logical_level.player_start, bomb, segment)) {
                        // 下一段路径需要穿过刚炸开的墙时先等待爆炸完成
                        if (gate_explosion_before_path(logical_level.player_start, segment)) break;
                        // 传入真实逻辑起点，避免路径首点缺失时 Tracker 误判第一段方向
                        Algorithm::Tracker::load_path(segment, logical_level.player_start);
                        task_done = true;
                    } else {
                        game.error_stage = 2; game.phase = GamePhase::ERROR_OCCURRED;
                    }
                    break;
                }
                case TaskType::LOAD_PATH_BOX: {
                    StaticArray<point, MAX_PATH_LENGTH> segment;
                    SokobanLevel probe = logical_level;
                    point probe_player = logical_level.player_start;
                    BoxPushTask box_push = make_box_push_task(task.param.box_push);

                    if (PlanningCommon::append_box_push_path(probe, probe_player, box_push, segment)) {
                        if (gate_explosion_before_path(logical_level.player_start, segment)) break;
                        // 推箱路径可能从下一格开始，逻辑起点用于 Tracker 压缩首段
                        Algorithm::Tracker::load_box_push_path(segment, logical_level.player_start,
                                                               box_push.box_start,
                                                               box_push.box_target,
                                                               logical_level);
                        task_done = true;
                    } else {
                        game.error_stage = 3; game.phase = GamePhase::ERROR_OCCURRED;
                    }
                    break;
                }
                case TaskType::LOAD_PATH_OBS: {
                    StaticArray<point, MAX_PATH_LENGTH> segment;

                    if (PlanningCommon::get_optimized_observe_path(
                            logical_level, logical_level.player_start,
                            task.param.target_grid, segment)) {
                        if (gate_explosion_before_path(logical_level.player_start, segment)) break;
                        // 观察移动同样保留逻辑起点，保证第一段航向和视觉校正基准一致
                        Algorithm::Tracker::load_path(segment, logical_level.player_start);
                        // 不再边跑边转：观测目标航向不在路径加载时写入，全程保持出发朝向平移，
                        // 转向留到到达观测点停稳后由 ALIGN_YAW 原地完成（用户拍板"到点再转"）。
                        task_done = true;
                    } else {
                        game.error_stage = 4; game.phase = GamePhase::ERROR_OCCURRED;
                    }
                    break;
                }
                case TaskType::WAIT_TRACKING_DONE: {
                    if (ctrl.tracker_state == TrackerState::FINISHED) task_done = true;
                    break;
                }
                case TaskType::ALIGN_YAW: {
                    // 到达观测点后在此原地转向（不再有提前转向预对齐）：把当前位置锁成目标点、
                    // 航向设为观测航向，交给底盘 YawProfiled 转，须转到容差内**且完全停稳**才放行。
                    // 关键：路径追踪到点时 Tracker 置了 hard_lock，底盘 hard_lock 分支会**绕过 yaw 规划**、
                    // 只把四轮清零 → 直接写 ctrl.mode=POINT_TRACKING 不解 hard_lock 的话车永远不转。
                    // 故首拍用 track_point 正规进入 POINT_TRACKING（它会清 hard_lock、锁死当前位置）。
                    if (ctrl.hard_lock || ctrl.mode != ControlMode::POINT_TRACKING) {
                        Algorithm::Tracker::track_point({pos.x, pos.y, task.param.target_yaw});
                    }
                    ctrl.current_target.yaw = task.param.target_yaw;  // 后续拍保持目标航向

                    // 无分支纯浮点 Yaw 包裹算法
                    float diff = ctrl.current_target.yaw - pos.yaw;
                    float err_yaw = std::abs(diff - 360.0f * std::roundf(diff * INV_360));

                    // 角度进容差且停稳才算对准完成（现转，故不再"不停稳就放行"）
                    constexpr float ALIGN_YAW_DONE_DEG = 2.0f;
                    if (err_yaw < ALIGN_YAW_DONE_DEG && App::g_state.physical.is_stopped) {
                        task_done = true;
                    }
                    break;
                }
                case TaskType::WAIT_ART2_CAPTURE: {
                    const uint32_t requested_mask = task.param.capture.active_mask;
                    if (!art2_capture_request_sent) {
                        if (!Subsystem::Vision::request_capture_ART2(
                                logical_level,
                                current_macro_action.observe.view.pos,
                                current_macro_action.observe.view.target_yaw,
                                requested_mask,
                                task.param.capture.use_new_protocol)) {
                            game.error_stage = 7;
                            game.phase = GamePhase::ERROR_OCCURRED;
                            break;
                        }
                        art2_capture_request_sent = true;
                    }

                    // 拍照 ACK 表示画面已经锁定，此时释放拍照通道并立即继续下一动作
                    // 所有已 ACK 请求的结果按 entity_id 异步写入，不阻塞后续观测请求
                    if (vision_data.capture_ack_received) {
                        logical_level.player_start = current_macro_action.observe.view.pos;
                        observed_mask |= requested_mask;
                        current_observe_yaw = current_macro_action.observe.view.target_yaw;
                        macro_planner.sync_semantics(vision_data.semantic_labels);
                        macro_planner.apply_observation(logical_level, requested_mask);
                        Subsystem::Vision::consume_capture_ack_ART2();
                        art2_capture_request_sent = false;
                        task_done = true;
                    }
                    break;
                }
                case TaskType::APPLY_BOMB_RESULT: {
                    // 必须在地图结算清墙前记录本次真正炸开的墙格
                    if (task.param.bomb_push.detonates) {
                        capture_blast_cells(task.param.bomb_push.blast_wall, logical_level);
                    } else {
                        s_pending_blast_cells.clear();
                    }

                    // 真实执行完成后，同时结算地图状态和剩余炸弹任务
                    PlanningCommon::apply_executed_bomb_push_result(
                        logical_level,
                        App::g_state.planning.bomb_tasks,
                        task.param.bomb_push
                    );

                    // 离散地图位置按本次规划终点结算，不再用带视觉微修正的物理 pose 反推格点
                    if (!App::g_state.planning.grid_path.empty()) {
                        logical_level.player_start = App::g_state.planning.grid_path.back();
                    }

                    s_explosion_wait_active = false;

                    task_done = true;
                    break;
                }
                case TaskType::UPDATE_BOX_LOGIC: {
                    PlanningCommon::apply_box_push_action_effect(logical_level, task.param.box_push);
                    if (!App::g_state.planning.grid_path.empty()) {
                        logical_level.player_start = App::g_state.planning.grid_path.back();
                    }
                    task_done = true;
                    break;
                }
            }

            if (task_done) {
                current_task_idx++;
            }
            break;
        }

        // =============================================================================
        // ---- 阶段三：语义绑定与推箱执行 ----
        // =============================================================================
        case GamePhase::BIND_SEMANTICS: {
            bool all_done = true;

            // 检查所有“观测动作”对应实体是否已写入语义标签
            for (uint8_t entity_id = 0; entity_id < SystemConfig::MAX_ENTITIES; ++entity_id) {
                if ((observed_mask & (1UL << entity_id)) && vision_data.semantic_labels[entity_id] == -1) {
                    all_done = false;
                    break;
                }
            }

            if (all_done) {
                macro_planner.sync_semantics(vision_data.semantic_labels);
                if (!macro_planner.apply_semantics_to_level(logical_level)) {
                    // 错误阶段 5：语义推断失败（观测结果无法形成完整箱子/目标点语义）
                    game.error_stage = 5; game.phase = GamePhase::ERROR_OCCURRED;  
                    break;
                }

                // 根据语义后的地图重新选择炸弹，并加载融合版 Sokoban
                if (!prepare_phase2_solver(false)) {
                    game.error_stage = 6;
                    game.phase = GamePhase::ERROR_OCCURRED;
                    break;
                }
                s_sokoban_solution_ready = false;
                game.phase = GamePhase::PLAN_SOKOBAN;
            }
            break;
        }
        
        case GamePhase::PLAN_SOKOBAN: {
            if (!s_sokoban_solution_ready) {
                bool success = solver.solve();
                if (!success && game.is_advanced_stage) {
                    success = prepare_phase2_solver(true) && solver.solve();
                }
                if (!success) {
                    game.error_stage = 6; // 错误阶段 6：推箱子路径求解失败
                    game.phase = GamePhase::ERROR_OCCURRED;
                    break;
                }
                s_sokoban_solution_ready = true;
            }

            const auto& result_path = solver.get_result_path();
            // 巡图最后一次爆破可能直接进入最终路径，此处补上跨阶段门控
            if (gate_explosion_before_path(logical_level.player_start, result_path)) break;

            Algorithm::Tracker::load_sokoban_path(
                result_path,
                logical_level.player_start,
                logical_level,
                solver.get_explosion_wait_indices());
            s_sokoban_solution_ready = false;
            ctrl.mode = ControlMode::AUTO_TRACKING;
            game.phase = GamePhase::EXEC_SOKOBAN;
            break;
        }

        case GamePhase::EXEC_SOKOBAN: {
            // 等待推箱路径执行完成
            if (ctrl.tracker_state == TrackerState::FINISHED) {

                // 同步逻辑终点
                if (!App::g_state.planning.grid_path.empty()) {
                    logical_level.player_start = App::g_state.planning.grid_path.back();  
                }

                game.phase = GamePhase::PLAN_RETURN_HOME;
            }
            break;
        }

        // =============================================================================
        // ---- 阶段四：返回起点 ----
        // =============================================================================
        case GamePhase::PLAN_RETURN_HOME: {
            s_return_pose_request_tick_ms = 0U;
            s_return_final_align_started = false;
            s_return_final_spin_started = false;
            s_return_home_dwell_active = false;
            s_return_exit_started = false;
            s_return_home_dwell_start_ms = 0U;
            s_return_route_pose_seq_start = App::g_state.vision.art1_pose_seq;
            // 爆炸屏蔽只作用于关卡内等待窗口，返航必须恢复动态视觉纠偏
            Algorithm::Tracker::set_vision_correction_suppressed(false);
            // 先单独返回发车区，连续关卡在此停稳等待 1 秒后再加载出库路径
            start_return_home_route();
            request_return_pose_sample(Core::Scheduler::get_sys_tick_ms());
            game.phase = GamePhase::EXEC_RETURN_HOME;
            break;
        }

        case GamePhase::EXEC_RETURN_HOME: {
            uint32_t now = Core::Scheduler::get_sys_tick_ms();
            bool has_next_round = game.round_idx + 1 < game.round_count;
            if (has_next_round) {
                // 连续关卡：发车区必须先停稳并保持 1 秒，再重新发车去观测区
                request_return_pose_sample(now);
                if (!s_return_exit_started) {
                    if (!s_return_home_dwell_active) {
                        if (ctrl.tracker_state != TrackerState::FINISHED ||
                            !App::g_state.physical.is_stopped) {
                            break;
                        }
                        s_return_home_dwell_active = true;
                        s_return_home_dwell_start_ms = now;
                    }

                    hold_return_position();
                    if (now - s_return_home_dwell_start_ms < get_return_home_dwell_ms()) {
                        break;
                    }

                    s_return_home_dwell_active = false;
                    s_return_exit_started = true;
                    // 只接受重新发车后采集的视觉帧作为观测区到达依据
                    s_return_route_pose_seq_start = App::g_state.vision.art1_pose_seq;
                    start_return_exit_route();
                    break;
                }

                bool odom_at_exit = Algorithm::Tracker::check_arrival(
                    {OUT_TARGET_X, OUT_TARGET_Y}, RETURN_EXIT_ODOM_REACH_RADIUS_CM);
                bool route_finished_at_exit = ctrl.tracker_state == TrackerState::FINISHED;
                bool visual_at_exit = visual_reached_return_exit(now);

                if (!odom_at_exit && !route_finished_at_exit && !visual_at_exit) {
                    break;
                }

                // 编码器有累计误差时，以视觉接近观测区作为到达依据，先硬锁当前位置等待完全停稳
                if (!App::g_state.physical.is_stopped) {
                    hold_return_position();
                    break;
                }

                // 到位且停稳后进入采集等待，等待期间绝不重新 track_point
                game.round_idx++;
                game.is_advanced_stage = ROUND_ADVANCED_SEQ[game.round_idx];
                s_return_exit_started = false;
                hold_return_position();
                reset_art1_acquisition_state();
                Algorithm::Tracker::set_vision_correction_suppressed(false);
                if (game.is_debug_mode) {
                    load_mock_map(game.selected_map_id);
                } else {
                    s_art1_capture_wait_start_ms = now;
                }
                game.phase = GamePhase::WAIT_FOR_VISION;
                break;
            }

            if (!s_return_final_align_started) {
                if (ctrl.tracker_state != TrackerState::FINISHED) {
                    break;
                }
                // AUTO 路径完成时会保持当前航向，这里只在最终端点原地收齐入口角
                Algorithm::Tracker::track_point({pos.x, pos.y, RETURN_HOME_YAW});
                s_return_final_align_started = true;
                break;
            }

            bool yaw_aligned =
                yaw_error_abs_deg(RETURN_HOME_YAW, pos.yaw) <= RETURN_FINAL_YAW_TOLERANCE_DEG;
            if (!App::g_state.physical.is_stopped || !yaw_aligned) {
                break;
            }

            if (game.round_count == MAX_ROUND_COUNT) {
                if (!s_return_final_spin_started) {
                    Subsystem::Chassis::start_continuous_spin(RETURN_FINAL_SPIN_DEG);
                    s_return_final_spin_started = true;
                    break;
                }

                if (!Subsystem::Chassis::is_continuous_spin_finished()) break;
            }

            game.phase = GamePhase::FINISHED;
            break;
        }

        case GamePhase::FINISHED: {
            // 完成态只在进入时锁住当前位姿，避免循环重置目标导致原地找零
            if (phase_entered || ctrl.mode != ControlMode::POINT_TRACKING) {
                Algorithm::Tracker::track_point({pos.x, pos.y, pos.yaw});
                ctrl.tracker_state = TrackerState::FINISHED;
            }
            break;
        }

        case GamePhase::ERROR_OCCURRED: {
            // 错误态同样保持当前姿态，不再强行转回入口角
            if (phase_entered || ctrl.mode != ControlMode::POINT_TRACKING) {
                Algorithm::Tracker::track_point({pos.x, pos.y, pos.yaw});
                ctrl.tracker_state = TrackerState::FINISHED;
            }
            break;
        }

        default:
            break;
    }

    s_last_update_phase = phase_at_entry;
}


//===================================================================
// GameManager 内部辅助函数
//===================================================================

/// \brief 准备第二阶段推箱求解器
/// \param dynamic_fallback 是否启用二阶段动态炸弹兜底策略
///
/// \details
/// 函数会根据当前 logical_level.box_semantics / target_semantics 恢复语义绑定，
/// 重新计算第二阶段炸弹任务，并把当前逻辑地图、语义关系、
/// 炸弹任务一起载入 Sokoban 求解器。求解失败重试时会传入 true，
/// 重试路径保留参数以维持状态机接口，新策略内部自行处理继承任务与补充任务。
bool GameManager::prepare_phase2_solver(bool dynamic_fallback) {
    (void)dynamic_fallback;
    auto& bombs = App::g_state.planning.bomb_tasks;
    bombs = strategic_planner.plan_phase2_bombs(logical_level, bombs);

    if (!solver.load_from_vision(
            logical_level,
            bombs.empty() ? nullptr : bombs.data(),
            bombs.size(),
            SokobanHeuristicMode::SEMANTIC)) return false;
    if (!solver.bind_semantics()) return false;
    return true;
}

/// \brief 规划下一条巡图宏动作
/// \param out_action 输出选中的宏动作
/// \return 成功得到宏动作时返回 true，MacroPlanner 和 fallback 都失败时返回 false
///
/// \details
/// 函数统一构造 MacroPlanContext，把 GameManager 持有的真实世界状态传给 MacroPlanner
bool GameManager::plan_next_macro_action(MacroAction& out_action) {
    MacroPlanContext ctx;
    ctx.level = logical_level;
    ctx.player = logical_level.player_start;
    ctx.yaw = current_observe_yaw;
    ctx.bomb_tasks = &App::g_state.planning.bomb_tasks;

    if (macro_planner.plan_next_action(ctx, out_action)) return true;
    if (!macro_planner.needs_exploration_replan()) return false;

    // 参考观测位被实际地图变化遮挡后，只重建未观测部分，不能清空已回传语义
    patrol_planner.load_level(logical_level);
    reference_patrol_actions = patrol_planner.plan_optimal_patrol(
        ctx.player,
        *ctx.bomb_tasks,
        ctx.yaw,
        macro_planner.observed_mask());
    macro_planner.set_reference_plan(reference_patrol_actions);

    // 单轮只重试一次，新的参考序列仍不可推进时交由上层错误分支收敛
    return macro_planner.plan_next_action(ctx, out_action);
}

/// \brief 启动一条宏动作
/// \param action 由 MacroPlanner 选出的宏动作
///
/// \details
/// 函数会记录当前宏动作、更新 UI 使用的动作序列索引，并将 PUSH_BOMB、PUSH_BOX、OBSERVE 三类宏动作转换为底层 RobotTask 队列。
/// 后续 EXEC_TASK_QUEUE 阶段只负责逐条执行这些微任务。
void GameManager::start_macro_action(const MacroAction& action) {
    // 更新当前宏动作
    current_macro_action = action;  

    // 将 action 记录到已执行动作序列 （仅供 UI 展示）
    executed_patrol_actions.push_back(action);
    App::g_state.game.action_idx = executed_patrol_actions.size() - 1;

    // 根据宏动作类型生成底层任务队列
    task_queue.clear();
    current_task_idx = 0;
    art2_capture_request_sent = false;

    if (action.kind == MacroActionKind::PUSH_BOMB) {
        task_queue.push_back(RobotTask::make_path_bomb(action.bomb_push));
        task_queue.push_back(RobotTask::make_wait_track());
        task_queue.push_back(RobotTask::make_apply_bomb_result(action.bomb_push));
    } else if (action.kind == MacroActionKind::PUSH_BOX) {
        task_queue.push_back(RobotTask::make_path_box(action.box_push));
        task_queue.push_back(RobotTask::make_wait_track());
        task_queue.push_back(RobotTask::make_update_box(action.box_push));
    } else {
        const ViewPose& view = action.observe.view;
        uint32_t mask = action.observe.active_mask;

        task_queue.push_back(RobotTask::make_path_obs(view.pos));
        task_queue.push_back(RobotTask::make_wait_track());
        task_queue.push_back(RobotTask::make_align(view.target_yaw));

        const bool use_new_protocol = Subsystem::Vision::use_new_art2_protocol();
        if (use_new_protocol) {
            // 新协议一个 OBSERVE 动作只发送一次批量请求
            task_queue.push_back(RobotTask::make_capture(mask, true));
        } else {
            // 旧协议每次只能请求一个实体，保持旧版任务拆分和处理顺序
            for (uint8_t entity_id = 0; entity_id < SystemConfig::MAX_ENTITIES; ++entity_id) {
                const uint32_t entity_bit = uint32_t{1u} << entity_id;
                if ((mask & entity_bit) != 0u) {
                    task_queue.push_back(RobotTask::make_capture(entity_bit, false));
                }
            }
        }
    }
}

/// \brief 根据物理坐标计算当前网格位置
/// \param pos 当前物理位姿
/// \return 合法网格坐标，越界时返回上一次逻辑位置
///
/// \details
/// 越界时直接进入错误态，而不是 clamp 成地图边界，避免后续规划基于假位置继续执行
///
point GameManager::current_grid_from_pose(const Pose2D& pos) const {
    int grid_x = static_cast<int>(std::lroundf((pos.x - MAP_OFFSET_X) * INV_GRID_SIZE_CM));
    int grid_y = static_cast<int>(std::lroundf((pos.y - MAP_OFFSET_Y) * INV_GRID_SIZE_CM));
    if (grid_x < 0 || grid_x >= MAP_MAX_WIDTH || grid_y < 0 || grid_y >= MAP_MAX_HEIGHT) {
        // 物理坐标已经越界时不再 clamp 成合法格点，避免规划层继续基于假位置行动
        App::g_state.game.error_stage = 9;
        App::g_state.game.phase = GamePhase::ERROR_OCCURRED;
        return logical_level.player_start;
    }
    return {static_cast<int8_t>(grid_x), static_cast<int8_t>(grid_y)};
}

/// \brief 获取观察动作的激活掩码
/// \param action 当前宏动作
/// \return OBSERVE 动作返回 active_mask，其他动作返回 0
///
uint32_t GameManager::observe_mask_of(const MacroAction& action) const {
    if (action.kind != MacroActionKind::OBSERVE) return 0;
    return action.observe.active_mask;
}

} // namespace App::GameEngine

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
    constexpr Point2D RETURN_HOME_TARGET = {SystemConfig::ENTRY_X, SystemConfig::ENTRY_Y};
    constexpr float RETURN_HOME_YAW = SystemConfig::ENTRY_YAW;

    // 连续发车轮次模式表：round 0 纯推箱，round 1/2 识别（巡图+推箱）。
    // 顺序 [pure, advanced, advanced] 保证没有陈旧巡图状态泄漏进 advanced 轮。
    constexpr bool ROUND_ADVANCED_SEQ[] = {false, true, true};
    constexpr uint8_t CONTINUOUS_ROUND_COUNT =
        static_cast<uint8_t>(sizeof(ROUND_ADVANCED_SEQ) / sizeof(ROUND_ADVANCED_SEQ[0]));
    // 返航到家判定半径：放宽到比小车停车散布更大一圈，避免在过严的点位上反复蹭/挪
    constexpr float RETURN_HOME_REACH_RADIUS_CM = 1.5f;
    constexpr float RETURN_FINAL_YAW_TOLERANCE_DEG = 2.0f;
    constexpr uint32_t RETURN_POSE_RECENT_MS = 300U;
    constexpr uint32_t RETURN_POSE_REQUEST_INTERVAL_MS = 150U;

    DTCM_DATA uint32_t s_return_pose_request_tick_ms = 0U;
    DTCM_DATA uint32_t s_return_pose_start_seq = 0U;
    DTCM_DATA bool s_return_tracking_started = false;

    // 发车前 home 起点重定位（仅 round>=1）：round1/2 是靠纯编码器返航回来的、home 基准已漂。
    // 故重进 INIT_CALIBRATE 后先**停在 home 别急着发车**，等一段固定时长让车/视觉完全稳，
    // 再吃一帧稳定视觉把里程计 x/y 校准到真实起点，定好了再发车去 OUT_TARGET。
    // 稳定闸比返航松：车此刻静止停在 home，视觉本就不动，几帧一致即可信（返航 10 帧太慢）。
    constexpr uint8_t HOME_RELOCATE_STABLE_REQUIRED_FRAMES = 3U;  // 发车前吃视觉所需稳定帧（<返航的10）
    constexpr uint32_t HOME_RELOCATE_SETTLE_MS = 800U;           // 到 home 后额外停这么久让车/视觉稳定再吃
    DTCM_DATA uint32_t s_home_relocate_request_tick_ms = 0U;
    DTCM_DATA uint32_t s_home_relocate_start_seq = 0U;
    DTCM_DATA uint32_t s_home_relocate_settle_start_ms = 0U;
    DTCM_DATA bool s_home_relocate_armed = false;

    // 炸弹按需等待爆炸：记录"这次爆炸真正会清开的墙格集合"，下一条路径若踩到其中任一格才等爆炸。
    // 存被清开的墙格(而非 blast_wall 单点)有两个原因：
    //  1) 精确——3×3 里多数格本就是空地，只有原本是墙、靠这次爆炸清开的格才需要等；踩邻域空地不等；
    //  2) 时序——爆破 apply 会把这些墙当场清成 0，故必须在 apply 前捕获，之后无法再从地图反查。
    DTCM_DATA StaticArray<point, 9> s_pending_blast_cells;  // 空 = 无待处理炸弹
    DTCM_DATA bool s_explosion_wait_active = false;    // 当前是否正在等爆炸
    DTCM_DATA uint32_t s_explosion_wait_start_ms = 0U; // 等待起始时刻

    [[maybe_unused]] [[gnu::always_inline]] inline bool segment_contains(
        const StaticArray<point, SystemConfig::MAX_PATH_LENGTH>& seg, point cell) {
        for (int i = 0; i < seg.size(); ++i) {
            if (seg[i] == cell) return true;
        }
        return false;
    }

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
        const StaticArray<point, SystemConfig::MAX_PATH_LENGTH>& seg) {
        for (int i = 0; i < seg.size(); ++i) {
            for (int j = 0; j < s_pending_blast_cells.size(); ++j) {
                if (seg[i] == s_pending_blast_cells[j]) return true;
            }
        }
        return false;
    }

    // 路径加载前调用：刚推的炸弹若炸开的墙挡住即将执行的路径段 seg，则先原地等爆炸。
    // 返回 true = 仍需等待（调用方应保持原地、暂不加载路径）；false = 可放行加载。
    [[gnu::always_inline]] inline bool gate_explosion_before_path(
        const StaticArray<point, SystemConfig::MAX_PATH_LENGTH>& seg) {
        if (s_pending_blast_cells.size() == 0) return false;  // 无待处理炸弹

        if (!s_explosion_wait_active) {
            if (!segment_needs_blast(seg)) {
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

        uint32_t wait_ms = (uint32_t)tune.bomb.explosion_wait_ms;
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

    [[gnu::always_inline]] inline bool get_recent_return_vision_pose(uint32_t now,
                                                                     uint32_t start_seq,
                                                                     Pose2D& out_pose) {
        const auto& vision = App::g_state.vision;
        if (vision.art1_pose_seq == 0U ||
            vision.art1_pose_seq == start_seq ||
            vision.art1_pose_stable_count < App::ART1_POSE_STABLE_REQUIRED_FRAMES ||
            now - vision.art1_pose_tick_ms > RETURN_POSE_RECENT_MS) {
            return false;
        }

        out_pose = vision.art1_pose_buffer[vision.art1_pose_publish_idx];
        return std::isfinite(out_pose.x) && std::isfinite(out_pose.y);
    }

    [[gnu::always_inline]] inline void start_return_home_tracking() {
        Algorithm::Tracker::track_point(
            {RETURN_HOME_TARGET.x, RETURN_HOME_TARGET.y, RETURN_HOME_YAW});
    }

    // 发车前 home 起点重定位：在 home 停稳后调用，非阻塞、阻塞发车直到定好。
    // 首拍 arm（快照 seq、发首帧请求、记 settle 起始时刻）；先停等 HOME_RELOCATE_SETTLE_MS 让车/视觉稳，
    // 期间只请求不吃；等够后再等一帧稳定近帧，拿到就 set_position 校准里程计 x/y（yaw 仍用陀螺）
    // 并返回 true；否则 false（继续停在 home 等）。成功后清 arm，供下一轮重新 arm。
    [[gnu::always_inline]] inline bool relocate_at_home() {
        uint32_t now = Core::Scheduler::get_sys_tick_ms();

        if (!s_home_relocate_armed) {
            s_home_relocate_start_seq = App::g_state.vision.art1_pose_seq;  // 请求前基准 seq
            s_home_relocate_request_tick_ms = 0U;                          // 强制首帧立即请求
            s_home_relocate_settle_start_ms = now;                          // settle 计时起点
            s_home_relocate_armed = true;
        }

        // 限流请求（settle 期间也请求，累积稳定帧计数，等够时刻立即有新鲜帧可吃）
        if (s_home_relocate_request_tick_ms == 0U ||
            now - s_home_relocate_request_tick_ms >= RETURN_POSE_REQUEST_INTERVAL_MS) {
            s_home_relocate_request_tick_ms = now;
            Subsystem::Vision::schedule_pose_request_ART1();
        }

        // 固定停等：到 home 后额外停 HOME_RELOCATE_SETTLE_MS 让车/视觉完全稳下来再吃
        if (now - s_home_relocate_settle_start_ms < HOME_RELOCATE_SETTLE_MS) {
            return false;
        }

        // 松稳定闸：车静止停在 home，几帧一致即可信（返航用 10 帧太慢，这里用 3）
        const auto& vision = App::g_state.vision;
        if (vision.art1_pose_seq == 0U ||
            vision.art1_pose_seq == s_home_relocate_start_seq ||
            vision.art1_pose_stable_count < HOME_RELOCATE_STABLE_REQUIRED_FRAMES ||
            now - vision.art1_pose_tick_ms > RETURN_POSE_RECENT_MS) {
            return false;  // 稳定近帧还没到，下一拍继续等
        }

        const Pose2D& vision_pose = vision.art1_pose_buffer[vision.art1_pose_publish_idx];
        if (!std::isfinite(vision_pose.x) || !std::isfinite(vision_pose.y)) {
            return false;
        }

        // 只重置 x/y，yaw 保留陀螺仪估计（视觉 yaw 不可信，全工程约定）
        Subsystem::PoseEstimator::set_position(
            vision_pose.x, vision_pose.y, App::g_state.physical.pose.yaw);
        s_home_relocate_armed = false;  // 本轮定好，解除 arm
        return true;
    }
}

//===================================================================
// GameEngine 模块对外接口实现
// ==================================================================

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

    // 连续发车：一次上电默认连打 3 次（round 0 纯推箱，round 1/2 识别，见 ROUND_ADVANCED_SEQ）。
    // sw1(C26) 不再决定赛段——赛段由轮次表在每轮返航到家时切换；C26 的 gpio_init 保留以便日后恢复该开关。
    App::g_state.game.round_idx         = 0;
    App::g_state.game.is_advanced_stage = ROUND_ADVANCED_SEQ[0];  // 第 0 轮固定纯推箱 (== false)
    App::g_state.game.is_debug_mode     = sw2_on;  // 调试模式：开-直接注入地图数据，绕过视觉输入；关-正常模式，等待视觉输入

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
            // 一次性复位（仅本 phase 首拍执行；round>=1 下面要在此停等重定位、会多拍停留，
            // 不能每拍重置视觉标志否则清掉正在累积的稳定帧）。
            if (phase_entered) {
                vision_data.art1_map_ready = false;
                vision_data.art1_pose_updated = false;
                vision_data.art1_pose_request_pending = false;
                vision_data.art1_pose_seq = 0U;
                vision_data.art1_pose_tick_ms = 0U;
                vision_data.art1_pose_stable_count = 0U;
                // 先把里程计设回名义 home（round>=1 随后由视觉重定位覆盖；round0 就用这个）
                Subsystem::PoseEstimator::set_position(ENTRY_X, ENTRY_Y, ENTRY_YAW);
                // 停在 home 锁位（round>=1 停等重定位期间不许动；round0 下面立刻改发 OUT_TARGET）
                Algorithm::Tracker::track_point({ENTRY_X, ENTRY_Y, ENTRY_YAW});

                // 复位炸弹爆炸等待与视觉抑制，防止上一局异常中断后抑制标志卡死导致视觉永久失效
                s_pending_blast_cells.clear();
                s_explosion_wait_active = false;
                Algorithm::Tracker::set_vision_correction_suppressed(false);

                // 复位发车前重定位 arm 标志，防止上一轮异常中断后卡在 armed 半途
                s_home_relocate_armed = false;
            }

            // 第 2、3 次发车（round>=1）：靠纯编码器返航回来、home 基准已漂——发车前先停在 home
            // 停等一段固定时长 + 吃一帧稳定视觉把里程计校准到真实起点，定好了再发车。
            // round 0 从真实 home 起步，里程计准，无需重定位、直接发车。
            if (game.round_idx >= 1 && !relocate_at_home()) {
                break;  // 还在停等/稳定帧没到，原地停在 home 等下一拍
            }

            // 定位已就绪（或 round0 无需定位）：发车去观测建图点
            Algorithm::Tracker::track_point({OUT_TARGET_X, OUT_TARGET_Y, ENTRY_YAW});
            game.phase = GamePhase::EXIT_START_ZONE;
            break;
        }

        case GamePhase::EXIT_START_ZONE: {
            // 到位后请求视觉模块 ART1 返回地图数据，进入等待状态。
            // （起点重定位已在 INIT_CALIBRATE 发车前完成，此处不再吃视觉。）
            if (Algorithm::Tracker::check_arrival({OUT_TARGET_X, OUT_TARGET_Y}, tune.tracker.reach_radius_min) &&
                App::g_state.physical.is_stopped) {
                Subsystem::Vision::request_map_ART1();   // 请求 ART1 地图
                game.phase = GamePhase::WAIT_FOR_VISION;
            }
            break;
        }

        case GamePhase::WAIT_FOR_VISION: {
            
            if (vision_data.art1_map_ready) {
                // 地图帧就绪，将视觉数据转换加载到逻辑地图结构中，供后续算法使用
                logical_level.map = vision_data.map;
                logical_level.player_start = {PLAN_START_X, PLAN_START_Y};
                logical_level.box_count = vision_data.box_count;
                logical_level.target_count = vision_data.box_count;
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

                // 异步请求位姿，用于后续全局定位校准
                Subsystem::Vision::request_pose_ART1(); 

                if (game.is_advanced_stage) {  
                    game.phase = GamePhase::PLAN_PATROL;       // 进入巡图
                } else {
                    if (!solver.load_from_vision(logical_level, nullptr, 0) || !solver.bind_semantics()) {
                        game.error_stage = 6;
                        game.phase = GamePhase::ERROR_OCCURRED;
                        break;
                    }
                    game.phase = GamePhase::PLAN_SOKOBAN;      // 直接进入推箱子阶段
                }
            } else {
                // 未接收到地图帧，超时重试请求地图数据
                static uint32_t last_request_tick = Core::Scheduler::get_sys_tick_ms();
                
                if (Core::Scheduler::get_sys_tick_ms() - last_request_tick > 1000) {
                    last_request_tick = Core::Scheduler::get_sys_tick_ms();
                    Subsystem::Vision::request_map_ART1();  
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
            bombs = strategic_planner.evaluate_and_assign_bombs<GameMode::PHASE1_ANY>(logical_level);

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
                        // 上一颗炸弹墙若在本段路径上，先原地等爆炸再加载
                        if (gate_explosion_before_path(segment)) break;
                        // 传入真实逻辑起点，避免路径首点缺失时 Tracker 误判第一段方向
                        Algorithm::Tracker::load_bomb_push_path(segment,
                                                               logical_level.player_start,
                                                               bomb.target_wall,
                                                               logical_level);
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
                        if (gate_explosion_before_path(segment)) break;
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

                    if (PlanningCommon::get_grid_time_path(logical_level, logical_level.player_start, task.param.target_grid, segment)) {
                        if (gate_explosion_before_path(segment)) break;
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
                    static bool req_sent = false;
                    static bool ack_seen = false;
                    uint8_t entity_id = task.param.capture.entity_id;
                    if (!req_sent) {
                        ack_seen = false;
                        Subsystem::Vision::request_capture_ART2(entity_id, task.param.capture.is_box);
                        req_sent = true;
                    }

                    if (vision_data.capture_ack_received) {
                        vision_data.capture_ack_received = false;
                        ack_seen = true;
                    }

                    // RESULT 和 ACK 都到齐后，才把该实体计入已观测
                    if (ack_seen && vision_data.semantic_labels[entity_id] != -1) {

                        logical_level.player_start = current_macro_action.observe.view.pos;
                        macro_planner.sync_semantics(vision_data.semantic_labels);
                        // 单个抓拍完成后立即更新观测位，便于 Macro 尽早触发完成式推箱
                        uint32_t entity_bit = (1UL << entity_id);
                        observed_mask |= entity_bit;
                        macro_planner.apply_observation(logical_level, entity_bit);
                        
                        req_sent = false;
                        ack_seen = false;
                        task_done = true;
                    }
                    break;
                }
                case TaskType::APPLY_BOMB_RESULT: {
                    // 记录被炸墙格：仅当本次推动确实引爆(detonates)才登记，下一条路径踩到被炸开的
                    // 墙格才等爆炸。非引爆的中间推动(detonates=false，只挪炸弹未炸墙)绝不能等待——
                    // 否则下一段常会经过 blast_wall(那还只是"未来"的墙)而误触发原地锁位等待+屏蔽视觉，
                    // 表现为"没有炸墙也在原地干等好久"。参见 [[bomb-wait-after-push]]。
                    // 必须在 apply 结算地图**之前**捕获——apply 会把这些墙当场清成空地，之后无法反查。
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
                if (current_task_idx >= task_queue.size()) {
                    if (current_macro_action.kind == MacroActionKind::OBSERVE) {
                        uint32_t mask = observe_mask_of(current_macro_action);
                        observed_mask |= mask;
                        current_observe_yaw = current_macro_action.observe.view.target_yaw;
                        macro_planner.sync_semantics(vision_data.semantic_labels);
                        macro_planner.apply_observation(logical_level, mask);
                    }
                }
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
                game.phase = GamePhase::PLAN_SOKOBAN;
            }
            break;
        }
        
        case GamePhase::PLAN_SOKOBAN: {
            bool success = solver.solve();
            if (!success && game.is_advanced_stage) {
                success = prepare_phase2_solver(true) && solver.solve();
            }

            if (success) {
                // 求解成功：加载路径并启动自动执行
                // Sokoban 结果路径由求解器生成，仍用当前逻辑起点修正首段压缩
                Algorithm::Tracker::load_sokoban_path(solver.get_result_path(),
                                                      logical_level.player_start,
                                                      logical_level);
                ctrl.mode = ControlMode::AUTO_TRACKING;
                game.phase = GamePhase::EXEC_SOKOBAN;
            } else {
                game.error_stage = 6; // 错误阶段 6：推箱子路径求解失败
                game.phase = GamePhase::ERROR_OCCURRED;
            }
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
            const auto& vision = App::g_state.vision;
            s_return_pose_request_tick_ms = 0U;
            s_return_pose_start_seq = vision.art1_pose_seq;
            s_return_tracking_started = false;
            request_return_pose_sample(Core::Scheduler::get_sys_tick_ms());
            game.phase = GamePhase::EXEC_RETURN_HOME;
            break;
        }

        case GamePhase::EXEC_RETURN_HOME: {
            uint32_t now = Core::Scheduler::get_sys_tick_ms();

            if (!s_return_tracking_started) {
                request_return_pose_sample(now);

                Pose2D vision_pose;
                if (!get_recent_return_vision_pose(now, s_return_pose_start_seq, vision_pose)) {
                    break;
                }

                Subsystem::PoseEstimator::set_position(vision_pose.x, vision_pose.y, pos.yaw);
                // 返航只在出发瞬间用一次视觉坐标校准里程计，之后全程靠编码器回出发点，
                // 不再让持续推流的视觉帧介入控制（避免临近终点被视觉反复拉扯/抖动）。
                // 回程误差交给"下一轮发车前在 home 停着等视觉定位好再发车"兜（见 INIT_CALIBRATE 起点重定位）。
                Algorithm::Tracker::set_vision_correction_suppressed(true);
                start_return_home_tracking();
                s_return_tracking_started = true;
                break;
            }

            bool yaw_aligned =
                yaw_error_abs_deg(RETURN_HOME_YAW, pos.yaw) <= RETURN_FINAL_YAW_TOLERANCE_DEG;
            bool odom_at_home = Algorithm::Tracker::check_arrival(
                RETURN_HOME_TARGET, RETURN_HOME_REACH_RADIUS_CM);

            if (odom_at_home && App::g_state.physical.is_stopped && yaw_aligned) {
                // 本轮已返航到家并停稳。若还有下一轮，推进轮次、切换赛段，重新进入
                // INIT_CALIBRATE——它会重新 arm 地图接收闸、把里程计原点设回 home、重发去观测点
                // 指令、清炸弹等待并解除视觉抑制，等价于一次完整的单轮复位（重新申请地图）。
                if (game.round_idx + 1 < CONTINUOUS_ROUND_COUNT) {
                    game.round_idx++;
                    game.is_advanced_stage = ROUND_ADVANCED_SEQ[game.round_idx];  // 下轮赛段：先设好再重进 INIT
                    game.phase = GamePhase::INIT_CALIBRATE;   // 重新发车
                } else {
                    game.phase = GamePhase::FINISHED;         // 第 3 轮完成，永久停止
                }
            }
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
/// 让 Strategy 使用更激进的动态兜底策略重新选择炸弹任务。
bool GameManager::prepare_phase2_solver(bool dynamic_fallback) {
    auto& bombs = App::g_state.planning.bomb_tasks;
    strategic_planner.set_phase2_dynamic_fallback(dynamic_fallback);
    bombs = strategic_planner.evaluate_and_assign_bombs<GameMode::PHASE2_SPECIFIC>(logical_level);
    strategic_planner.set_phase2_dynamic_fallback(false);

    if (!solver.load_from_vision(logical_level, bombs.empty() ? nullptr : bombs.data(), bombs.size())) return false;
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

        // 根据激活掩码为每个需要观测的实体生成一个捕获任务
        for (uint8_t entity_id = 0; entity_id < SystemConfig::MAX_ENTITIES; ++entity_id) {
            if (mask & (1UL << entity_id)) {
                task_queue.push_back(RobotTask::make_capture(entity_id, entity_id < logical_level.box_count));
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

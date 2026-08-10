import argparse
import itertools
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

'''
用法示例：
python simulation/test/run_regression.py --solver simulation/solver.exe --maps simulation/map/map_bomb
参数：
--show-bombs
--show-obs
--show-details 同时显示炸弹、观测点和 Strategy 清障摘要
--focus-regression 只对部分重点地图进行回归，并检查特定护栏，配合 --show-details 可人工复核观测点和清障诊断是否回退

python simulation/test/run_regression.py --solver simulation/solver.exe --maps simulation/map/map_box --show-obs
python simulation/test/run_regression.py --solver simulation/solver.exe --maps simulation/map/map_bomb --show-details
python simulation/test/run_regression.py --solver simulation/solver.exe --maps simulation/map/map_clear --show-details
python simulation/test/run_regression.py --solver simulation/solver.exe --maps simulation/map --show-details --focus-regression
'''

MAP_W = 12
MAP_H = 16
PLAN_START_X = 4
PLAN_START_Y = 1
TIMEOUT_SECONDS = 1.0
STAGE_TIMEOUT_MS = 50
MOVE_STEP_COST = 1
STOP_NODE_COST = 2
OBSERVE_EXTRA_COST = 1
TURN_EXTRA_COST = 2

# 重点观察地图只在 --focus-regression 模式下生效，普通批量回归不读取这组护栏
FOCUS_REGRESSION_CHECKS = {
    "map_bomb": [
        {
            "map": "map_bomb_01.txt",
            "kind": "contains_each_row_task",
            "needle": "7,2->5,2",
            "desc": "保留直接左推倾向，阶段1任务中出现 7,2->5,2",
        },
        {
            "map": "map_bomb_09.txt",
            "kind": "contains_each_row_tasks",
            "needles": ["8,3->3,1", "5,12->2,10"],
            "desc": "保留当前较优炸弹任务，阶段1任务中出现 8,3->3,1 和 5,12->2,10",
        },
        {
            "map": "map_bomb_38.txt",
            "kind": "all_solved",
            "desc": "不回退为失败或超时",
        },
    ],
    "map_clear": [
        {
            "map": "map_clear_13.txt",
            "kind": "all_solved",
            "desc": "不回退为失败或超时，并人工复核清障箱未被推到目标点导致消失",
        },
        {
            "map": "map_clear_08.txt",
            "kind": "manual_timeout_note",
            "desc": "若只是阶段超时先记录为性能债",
        },
        {
            "map": "map_clear_15.txt",
            "kind": "manual_timeout_note",
            "desc": "若只是阶段超时先记录为性能债",
        },
    ],
}


def parse_stage_ms(value):
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def has_stage_timeout(times):
    # PC 回归把四个阶段当作独立预算，任一阶段超限都标记为超时
    return any((ms is not None and ms > STAGE_TIMEOUT_MS) for ms in (parse_stage_ms(t) for t in times))


def natural_key(text):
    import re

    return [
        (0, int(part)) if part.isdigit() else (1, part.lower())
        for part in re.findall(r"[0-9]+|[^0-9]+", text)
    ]


def read_map(path, start_x=PLAN_START_X, start_y=PLAN_START_Y, use_map_start=False):
    lines = [
        line.strip()
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    rows = [(line + "-" * MAP_W)[:MAP_W] for line in lines[:MAP_H]]
    boxes = []
    targets = []
    bombs = []
    player = (start_x, start_y)

    for file_y, row in enumerate(rows):
        for x, ch in enumerate(row):
            grid_y = MAP_H - 1 - file_y
            if ch == "$":
                boxes.append((x, grid_y))
            elif ch == ".":
                targets.append((x, grid_y))
            elif ch == "*":
                bombs.append((x, grid_y))
            elif ch == "@" and use_map_start:
                player = (x, grid_y)

    return {
        "rows": rows,
        "boxes": boxes,
        "targets": targets,
        "bombs": bombs,
        "player": player,
    }


def write_map_input(work_dir, map_info, box_sem, target_sem):
    map_input = work_dir / "map_input.txt"
    with map_input.open("w", encoding="utf-8", newline="\n") as out:
        for row in map_info["rows"]:
            out.write(row.replace("@", "-") + "\n")
        out.write(f"START {map_info['player'][0]} {map_info['player'][1]}\n")
        out.write("SEMANTICS BOXES " + box_sem + "\n")
        out.write("TARGETS " + target_sem + "\n")


def run_git(args, root):
    try:
        return subprocess.check_output(
            ["git"] + args,
            cwd=root,
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def get_git_info(root):
    head = run_git(["rev-parse", "HEAD"], root)
    short_head = run_git(["rev-parse", "--short", "HEAD"], root)
    status = run_git(["status", "--porcelain"], root)
    return {
        "head": head,
        "short_head": short_head,
        "dirty": "1" if status else "0",
        "tree_state": "dirty" if status else "HEAD",
    }


def solver_stamp(path):
    try:
        stat = path.stat()
    except OSError:
        return ""
    return datetime.fromtimestamp(stat.st_mtime).isoformat(timespec="seconds")


def unlink_with_retry(path, attempts=6, delay=0.05):
    for attempt in range(attempts):
        try:
            path.unlink()
            return
        except PermissionError:
            if attempt + 1 >= attempts:
                raise
            # Windows 下子进程退出后文件句柄可能短暂滞留
            time.sleep(delay * (attempt + 1))


def first_tokens(lines, prefix):
    for line in lines:
        if line.startswith(prefix):
            return line.split()
    return []


def parse_bomb_tasks(lines, prefix):
    tasks = []
    i = 0
    while i < len(lines):
        line = lines[i]
        if line.startswith(prefix):
            parts = line.split()
            count = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 0
            for offset in range(1, count + 1):
                if i + offset >= len(lines):
                    break
                vals = lines[i + offset].split()
                if len(vals) >= 4:
                    push_count = vals[4] if len(vals) >= 5 else "0"
                    tasks.append(f"{vals[0]},{vals[1]}->{vals[2]},{vals[3]}:{push_count}")
            break
        i += 1
    return "|".join(tasks)


def parse_chosen_obs(lines):
    obs = []
    for i, line in enumerate(lines):
        if line.startswith("CHOSEN_OBS"):
            parts = line.split()
            count = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 0
            for offset in range(1, count + 1):
                if i + offset >= len(lines):
                    break
                vals = lines[i + offset].split()
                if len(vals) >= 2:
                    obs.append(f"{vals[0]},{vals[1]}")
            break
    return "|".join(obs)


def parse_strategy_clear_diag(lines):
    profile = first_tokens(lines, "STRATEGY_CLEAR_PROFILE ")
    total = profile[1] if len(profile) >= 2 else "0"
    dropped = profile[2] if len(profile) >= 3 else "0"

    pushes_by_clear = {}
    for line in lines:
        if not line.startswith("STRATEGY_CLEAR_PUSH "):
            continue
        parts = line.split()
        if len(parts) < 14:
            continue
        clear_idx = parts[1]
        pushes_by_clear.setdefault(clear_idx, []).append(
            f"b{parts[3]}:{parts[6]},{parts[7]}->{parts[8]},{parts[9]}"
            f":{parts[4]}:{parts[5]}:open{parts[11]}:safe{parts[12]}"
        )

    entries = []
    for line in lines:
        if not line.startswith("STRATEGY_CLEAR "):
            continue
        parts = line.split()
        if len(parts) < 16:
            continue
        idx = parts[1]
        push_desc = pushes_by_clear.get(idx, [])
        if parts[4] != "1" and not push_desc:
            continue
        if not push_desc:
            continue
        entries.append(
            f"e{parts[2]}p{parts[3]} {parts[8]},{parts[9]}->{parts[10]},{parts[11]}"
            f" {parts[5]} cost{parts[12]} route{parts[13]} block{parts[14]} "
            + ";".join(push_desc[:3])
        )
        if len(entries) >= 6:
            break

    if not profile and not entries:
        return ""

    summary = f"clear={total} drop={dropped}"
    if entries:
        summary += " | " + " || ".join(entries)
    return summary


def parse_strategy_hot_profile(lines):
    tokens = first_tokens(lines, "STRATEGY_HOT_PROFILE ")
    if len(tokens) < 24:
        return ""

    names = [
        "fbfs", "fbfs_us", "fbfs_reach", "fbfs_pop", "fbfs_q",
        "soft", "soft_us", "soft_pop", "soft_q",
        "clear", "clear_ok", "clear_us", "route", "route_ok",
        "boxchk", "boxok", "bombchk", "bombok", "playerchk",
        "real_nodes", "real_cand", "real_try", "real_depth",
    ]
    data = dict(zip(names, tokens[1:]))
    return (
        f"fbfs={data['fbfs']}/{data['fbfs_us']}us pop{data['fbfs_pop']} reach{data['fbfs_reach']} "
        f"soft={data['soft']}/{data['soft_us']}us pop{data['soft_pop']} "
        f"clear={data['clear']} ok{data['clear_ok']} {data['clear_us']}us route{data['route_ok']}/{data['route']} "
        f"path box{data['boxok']}/{data['boxchk']} bomb{data['bombok']}/{data['bombchk']} "
        f"real n{data['real_nodes']} cand{data['real_cand']} try{data['real_try']} d{data['real_depth']}"
    )


def parse_strategy_shadow_clear(lines):
    tokens = first_tokens(lines, "STRATEGY_SHADOW_CLEAR ")
    if len(tokens) < 29:
        return "", ""

    has_real_path_blocker = len(tokens) >= 42
    if has_real_path_blocker:
        names = [
            "route", "route_ok", "route_no_block",
            "blk_corr", "blk_real_path", "blk_stand", "blk_stand_exact", "blk_stand_near_only", "blk_near", "blk_rec", "blk_real",
            "acc_safe", "acc_theory", "acc_open", "acc_dead", "acc_exact", "acc_near",
            "real_nodes", "src_exact", "src_near", "src_far",
            "push_cand", "push_exec", "open_path",
            "park_chk", "park_safe", "park_theory", "park_dead", "park_rej",
        ]
        decision_start = 30
    else:
        names = [
            "route", "route_ok", "route_no_block",
            "blk_corr", "blk_stand", "blk_stand_exact", "blk_stand_near_only", "blk_near", "blk_rec", "blk_real",
            "acc_safe", "acc_theory", "acc_open", "acc_dead", "acc_exact", "acc_near",
            "real_nodes", "src_exact", "src_near", "src_far",
            "push_cand", "push_exec", "open_path",
            "park_chk", "park_safe", "park_theory", "park_dead", "park_rej",
        ]
        decision_start = 29
    decision_names = [
        "keep_exact", "deprior_near_stand", "deprior_route_near", "keep_recursive",
        "real_exact", "real_near", "real_far",
        "accept_safe", "need_theory", "need_open", "reject_dead", "reject_noblk",
    ]
    data = dict(zip(names, tokens[1:]))
    data.setdefault("blk_real_path", "0")
    shadow_summary = (
        f"route {data['route_ok']}/{data['route']} noblk{data['route_no_block']} "
        f"blk corr{data['blk_corr']} realPath{data['blk_real_path']} stand{data['blk_stand']} "
        f"stand0/{data['blk_stand_exact']} standNear/{data['blk_stand_near_only']} "
        f"near{data['blk_near']} real{data['blk_real']} "
        f"acc safe{data['acc_safe']} theory{data['acc_theory']} open{data['acc_open']} near{data['acc_near']}/{data['acc_exact']} "
        f"real src {data['src_exact']}/{data['src_near']}/{data['src_far']} "
        f"push {data['push_exec']}/{data['push_cand']} open{data['open_path']} "
        f"park safe{data['park_safe']} theory{data['park_theory']} dead{data['park_dead']} rej{data['park_rej']}"
    )
    decision_summary = ""
    if len(tokens) >= decision_start + len(decision_names):
        decision = dict(zip(decision_names, tokens[decision_start:]))
        decision_summary = (
            f"keep exact{decision['keep_exact']} rec{decision['keep_recursive']} "
            f"deprior nearStand{decision['deprior_near_stand']} routeNear{decision['deprior_route_near']} realFar{decision['real_far']} "
            f"real exact/near {decision['real_exact']}/{decision['real_near']} "
            f"accept safe{decision['accept_safe']} need theory{decision['need_theory']} open{decision['need_open']} "
            f"reject dead{decision['reject_dead']} noblk{decision['reject_noblk']}"
        )
    return shadow_summary, decision_summary


def parse_strategy_shadow_class(lines):
    tokens = first_tokens(lines, "STRATEGY_SHADOW_CLASS ")
    if len(tokens) < 14:
        return ""
    names = [
        "near_l", "real_l", "theory_l", "dead_l", "open_l", "noblk_l",
        "timeout_like", "risk",
        "near", "real", "theory", "dead", "open",
    ]
    data = dict(zip(names, tokens[1:]))
    return (
        f"risk{data['risk']} timeoutLike{data['timeout_like']} "
        f"L near/real/theory/dead/open/noblk="
        f"{data['near_l']}/{data['real_l']}/{data['theory_l']}/{data['dead_l']}/{data['open_l']}/{data['noblk_l']} "
        f"raw near{data['near']} real{data['real']} theory{data['theory']} dead{data['dead']} open{data['open']}"
    )


def from_output_coord(x, y):
    return [x, MAP_H - 1 - y]


def count_turns(path_coords):
    turn_count = 0
    prev_dir = None
    for i in range(1, len(path_coords)):
        p1, p2 = path_coords[i - 1], path_coords[i]
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        if dx == 0 and dy == 0:
            continue
        length = abs(dx) + abs(dy)
        curr_dir = (dx // length, dy // length) if length > 0 else (dx, dy)
        if prev_dir is not None and curr_dir != prev_dir:
            turn_count += 1
        prev_dir = curr_dir
    return turn_count


def path_length(path_coords):
    total = 0.0
    for i in range(1, len(path_coords)):
        dx = path_coords[i][0] - path_coords[i - 1][0]
        dy = path_coords[i][1] - path_coords[i - 1][1]
        total += (dx * dx + dy * dy) ** 0.5
    return int(round(total))


def count_patrol_rotations(patrol_events, map_info):
    rotations = 0
    current_yaw = 90
    virtual_pos = [map_info["player"][0], MAP_H - 1 - map_info["player"][1]]
    obs_group = []
    boxes_ui = [[x, MAP_H - 1 - y] for x, y in map_info["boxes"]]
    targets_ui = [[x, MAP_H - 1 - y] for x, y in map_info["targets"]]
    valid_fov = {
        0: [(1, 0), (2, 0), (1, 1), (1, -1), (2, 1), (2, -1)],
        90: [(0, 1), (0, 2), (-1, 1), (1, 1), (-1, 2), (1, 2)],
        180: [(-1, 0), (-2, 0), (-1, -1), (-1, 1), (-2, -1), (-2, 1)],
        270: [(0, -1), (0, -2), (1, -1), (-1, -1), (1, -2), (-1, -2)],
    }

    def process_obs_group(group, pos, curr_yaw):
        if not group:
            return curr_yaw, 0
        objects_rel = []
        for ent_id, is_box in group:
            if is_box:
                if ent_id >= len(boxes_ui):
                    continue
                ox, oy = boxes_ui[ent_id]
            else:
                target_idx = ent_id - len(boxes_ui) if ent_id >= len(boxes_ui) else ent_id
                if target_idx >= len(targets_ui):
                    continue
                ox, oy = targets_ui[target_idx]
            dx = ox - pos[0]
            dy = pos[1] - oy
            objects_rel.append((dx, dy))
        if not objects_rel:
            return curr_yaw, 0

        possible_yaws = []
        for yaw, fov in valid_fov.items():
            if all(obj in fov for obj in objects_rel):
                possible_yaws.append(yaw)
        if not possible_yaws:
            sum_dx = sum(o[0] for o in objects_rel)
            sum_dy = sum(o[1] for o in objects_rel)
            if abs(sum_dx) >= abs(sum_dy):
                possible_yaws = [0 if sum_dx > 0 else 180]
            else:
                possible_yaws = [90 if sum_dy > 0 else 270]
        if curr_yaw in possible_yaws:
            return curr_yaw, 0
        return possible_yaws[0], 1

    for event in patrol_events:
        if event[0] == "OBSERVE":
            observe_yaw = event[3] if len(event) >= 4 else None
            if observe_yaw is None:
                # 兼容旧输出：缺少真实 yaw 时才按同一驻留点的实体位置推断
                obs_group.append((event[1], event[2]))
                continue

            # 新输出逐次记录宏动作的 ALIGN_YAW，不能把同格连续观测合并
            if obs_group:
                current_yaw, added = process_obs_group(obs_group, virtual_pos, current_yaw)
                rotations += added
                obs_group = []
            observe_yaw %= 360
            if current_yaw != observe_yaw:
                rotations += 1
                current_yaw = observe_yaw
        elif event[0] == "MOVE":
            if obs_group:
                current_yaw, added = process_obs_group(obs_group, virtual_pos, current_yaw)
                rotations += added
                obs_group = []
            virtual_pos = event[1]
    if obs_group:
        current_yaw, added = process_obs_group(obs_group, virtual_pos, current_yaw)
        rotations += added
    return rotations


def parse_path_sections(lines, map_info):
    patrol_events = []
    sokoban_path = []
    patrol_observations = 0
    cost_model = (
        MOVE_STEP_COST,
        STOP_NODE_COST,
        OBSERVE_EXTRA_COST,
        TURN_EXTRA_COST,
    )
    failed = False

    i = 0
    while i < len(lines):
        line = lines[i]
        if line.startswith("CHOSEN_OBS "):
            parts = line.split()
            if len(parts) >= 2:
                patrol_observations = int(parts[1])
        elif line.startswith("COST_MODEL "):
            values = [int(v) for v in line.split()[1:5]]
            if len(values) == 4:
                cost_model = tuple(values)
        elif line == "PATROL":
            i += 1
            while i < len(lines) and lines[i] not in ("SOKOBAN", "SOKOBAN_REPLAY_BEGIN", "FAILED", "MCU_TRACE"):
                parts = lines[i].split()
                if parts and parts[0] == "OBSERVE" and len(parts) >= 3:
                    observe_yaw = int(parts[3]) if len(parts) >= 4 else None
                    patrol_events.append(
                        ["OBSERVE", int(parts[1]), parts[2] == "1", observe_yaw]
                    )
                elif len(parts) >= 2:
                    patrol_events.append(["MOVE", from_output_coord(int(parts[0]), int(parts[1]))])
                i += 1
            continue
        if line == "SOKOBAN":
            i += 1
            while i < len(lines) and lines[i] != "MCU_TRACE":
                if lines[i] == "FAILED":
                    failed = True
                else:
                    parts = lines[i].split()
                    if len(parts) >= 2:
                        sokoban_path.append(from_output_coord(int(parts[0]), int(parts[1])))
                i += 1
            continue
        if line == "MCU_TRACE":
            # MCU/ART2 调试事件不是坐标路径，跳过到下一个协议段
            i += 1
            while i < len(lines) and lines[i] not in ("PATROL", "SOKOBAN", "SOKOBAN_REPLAY_BEGIN"):
                i += 1
            continue
        i += 1

    start_ui = [map_info["player"][0], MAP_H - 1 - map_info["player"][1]]
    patrol_path = [start_ui]
    for event in patrol_events:
        if event[0] == "MOVE":
            patrol_path.append(event[1])

    sokoban_start = patrol_path[-1] if patrol_path else start_ui
    full_sokoban_path = [sokoban_start] + sokoban_path
    patrol_steps = path_length(patrol_path)
    sokoban_steps = len(sokoban_path)
    patrol_turns = count_turns(patrol_path)
    sokoban_turns = count_turns(full_sokoban_path)
    patrol_rotations = count_patrol_rotations(patrol_events, map_info)
    sokoban_rotations = 0
    sokoban_observations = 0
    total_steps = patrol_steps + sokoban_steps
    total_turns = patrol_turns + sokoban_turns
    total_rotations = patrol_rotations + sokoban_rotations
    total_observations = patrol_observations + sokoban_observations
    move_cost, stop_cost, observe_extra, turn_extra = cost_model
    total_cost = (
        total_steps * move_cost
        + total_turns * stop_cost
        + total_observations * (stop_cost + observe_extra)
        + total_rotations * turn_extra
    )

    return {
        "patrol_steps": patrol_steps,
        "sokoban_steps": sokoban_steps,
        "total_steps": total_steps,
        "patrol_turns": patrol_turns,
        "sokoban_turns": sokoban_turns,
        "total_turns": total_turns,
        "patrol_rotations": patrol_rotations,
        "sokoban_rotations": sokoban_rotations,
        "total_rotations": total_rotations,
        "patrol_observations": patrol_observations,
        "sokoban_observations": sokoban_observations,
        "total_observations": total_observations,
        "total_cost": total_cost,
        "failed": failed,
    }


def parse_replay_pushes(lines):
    push_count = ""
    complete_count = ""
    for line in lines:
        if line.startswith("REPLAY_SUMMARY"):
            parts = line.split()
            if len(parts) >= 3:
                push_count = parts[1]
                complete_count = parts[2]
            break
    return push_count, complete_count


def parse_output(path, map_info):
    if not path.exists():
        return {
            "failed": "1",
            "solver_build": "",
            "times": ["", "", "", ""],
            "profile": {},
            "patrol_steps": "",
            "sokoban_steps": "",
            "total_steps": "",
            "patrol_turns": "",
            "sokoban_turns": "",
            "total_turns": "",
            "patrol_rotations": "",
            "sokoban_rotations": "",
            "total_rotations": "",
            "patrol_observations": "",
            "sokoban_observations": "",
            "total_observations": "",
            "total_cost": "",
            "pushes": "",
            "completed": "",
            "bomb_tasks_1": "",
            "bomb_tasks_2": "",
            "chosen_obs": "",
            "strategy_hot_profile": "",
            "strategy_shadow_clear": "",
            "strategy_shadow_decision": "",
            "strategy_shadow_class": "",
            "strategy_clear_diag": "",
        }

    lines = [line.strip() for line in path.read_text(encoding="utf-8", errors="replace").splitlines() if line.strip()]
    time_tokens = first_tokens(lines, "TIMES ")
    times = time_tokens[1:5] if len(time_tokens) >= 5 else ["", "", "", ""]
    build_tokens = first_tokens(lines, "BUILD ")
    solver_build = " ".join(build_tokens[1:]) if build_tokens else ""
    profile_tokens = first_tokens(lines, "PROFILE ")
    profile_names = [
        "expanded_nodes",
        "generated_moves",
        "tt_hits",
        "heuristic_dead_prunes",
        "threshold_prunes",
        "path_cycle_prunes",
        "static_deadlock_prunes",
        "block_2x2_prunes",
        "max_depth",
        "threshold_iterations",
        "final_threshold",
        "nps",
    ]
    profile = {}
    for name, value in zip(profile_names, profile_tokens[1:]):
        profile[name] = value

    path_stats = parse_path_sections(lines, map_info)
    pushes, completed = parse_replay_pushes(lines)
    strategy_shadow_clear, strategy_shadow_decision = parse_strategy_shadow_clear(lines)
    strategy_shadow_class = parse_strategy_shadow_class(lines)

    return {
        "failed": "1" if path_stats["failed"] else "0",
        "solver_build": solver_build,
        "times": times,
        "profile": profile,
        "patrol_steps": str(path_stats["patrol_steps"]),
        "sokoban_steps": str(path_stats["sokoban_steps"]),
        "total_steps": str(path_stats["total_steps"]),
        "patrol_turns": str(path_stats["patrol_turns"]),
        "sokoban_turns": str(path_stats["sokoban_turns"]),
        "total_turns": str(path_stats["total_turns"]),
        "patrol_rotations": str(path_stats["patrol_rotations"]),
        "sokoban_rotations": str(path_stats["sokoban_rotations"]),
        "total_rotations": str(path_stats["total_rotations"]),
        "patrol_observations": str(path_stats["patrol_observations"]),
        "sokoban_observations": str(path_stats["sokoban_observations"]),
        "total_observations": str(path_stats["total_observations"]),
        "total_cost": str(path_stats["total_cost"]),
        "pushes": pushes,
        "completed": completed,
        "bomb_tasks_1": parse_bomb_tasks(lines, "BOMB_TASKS_1"),
        "bomb_tasks_2": parse_bomb_tasks(lines, "BOMB_TASKS_2"),
        "chosen_obs": parse_chosen_obs(lines),
        "strategy_hot_profile": parse_strategy_hot_profile(lines),
        "strategy_shadow_clear": strategy_shadow_clear,
        "strategy_shadow_decision": strategy_shadow_decision,
        "strategy_shadow_class": strategy_shadow_class,
        "strategy_clear_diag": parse_strategy_clear_diag(lines),
    }


def map_folder_label(path):
    return path.name + "_test"


def output_path_for(folder, output):
    if output:
        if output.suffix.lower() == ".csv":
            return output.with_suffix(".md")
        return output
    return Path("test") / (map_folder_label(folder) + ".md")


def focus_output_path_for(folder, output):
    if output:
        if output.suffix.lower() == ".csv":
            return output.with_suffix(".md")
        return output
    return Path("test") / (folder.name + "_focus_test.md")


def configured_focus_groups(folder):
    if folder.name in FOCUS_REGRESSION_CHECKS:
        return [folder.name]
    groups = [
        child.name
        for child in folder.iterdir()
        if child.is_dir() and child.name in FOCUS_REGRESSION_CHECKS
    ]
    return sorted(groups, key=natural_key)


def focus_map_paths(folder):
    paths = []
    for group in configured_focus_groups(folder):
        group_dir = folder if folder.name == group else folder / group
        focus_names = {
            check["map"]
            for check in FOCUS_REGRESSION_CHECKS.get(group, [])
        }
        for path in sorted(group_dir.glob("*.txt"), key=lambda p: natural_key(p.name)):
            if path.name in focus_names:
                paths.append(path)
    return paths


def build_semantic_cases(box_count):
    target_sem = "".join(str(i % 10) for i in range(box_count))
    cases = []
    for perm in itertools.permutations(range(box_count)):
        box_sem = "".join(str(i % 10) for i in perm)
        cases.append((box_sem, target_sem))
    return cases


def md_escape(value):
    return str(value).replace("\\", "\\\\").replace("|", "\\|").replace("\n", " ")


def phase1_task_count(task_text):
    if not task_text:
        return 0
    return len([part for part in task_text.split("|") if part])


def render_focus_regression_report(out, rows, map_folder):
    groups = configured_focus_groups(map_folder)
    if not groups:
        return

    rows_by_folder_map = {}
    for row in rows:
        key = (row.get("map_folder_name", ""), row.get("map_file", row.get("map", "")))
        rows_by_folder_map.setdefault(key, []).append(row)

    out.write("\n## 第一阶段重点图护栏\n\n")
    out.write("| 地图目录 | 地图 | 检查项 | 结果 | 说明 |\n")
    out.write("| --- | --- | --- | --- | --- |\n")
    for group in groups:
        for check in FOCUS_REGRESSION_CHECKS.get(group, []):
            map_rows = rows_by_folder_map.get((group, check["map"]), [])
            render_focus_check_row(out, group, check, map_rows)


def render_focus_check_row(out, group, check, map_rows):
    status = "缺少用例"
    note = "报告中没有该地图"

    if map_rows:
        statuses = [row.get("status", "") for row in map_rows]
        failed = [s for s in statuses if s not in ("ok", "timeout")]
        timeout = [s for s in statuses if s == "timeout"]

        if check["kind"] == "contains_each_row_task":
            needle = check["needle"]
            missing_rows = [row for row in map_rows if needle not in row.get("bomb_tasks_1", "")]
            if failed or timeout:
                status = "异常"
                note = f"失败 {len(failed)}，超时 {len(timeout)}"
            elif not missing_rows:
                status = "通过"
                note = f"全部用例找到 {needle}"
            else:
                status = "需复核"
                note = f"{len(missing_rows)} 个用例未找到 {needle}"
        elif check["kind"] == "contains_each_row_tasks":
            missing_rows = [
                row for row in map_rows
                if any(needle not in row.get("bomb_tasks_1", "") for needle in check["needles"])
            ]
            if failed or timeout:
                status = "异常"
                note = f"失败 {len(failed)}，超时 {len(timeout)}"
            elif not missing_rows:
                status = "通过"
                note = "全部用例关键任务均出现"
            else:
                status = "需复核"
                note = f"{len(missing_rows)} 个用例缺少关键任务"
        elif check["kind"] == "all_solved":
            max_phase1_tasks = max(phase1_task_count(row.get("bomb_tasks_1", "")) for row in map_rows)
            if failed:
                status = "异常"
                note = f"失败 {len(failed)}"
            elif timeout:
                status = "超时"
                note = f"超时 {len(timeout)}，最大阶段1炸弹数 {max_phase1_tasks}"
            else:
                status = "通过"
                note = f"全部 ok，最大阶段1炸弹数 {max_phase1_tasks}"
        elif check["kind"] == "manual_timeout_note":
            if failed:
                status = "异常"
                note = f"失败 {len(failed)}"
            elif timeout:
                status = "记录"
                note = f"超时 {len(timeout)}，按性能债跟踪"
            else:
                status = "通过"
                note = "没有失败或超时"

    out.write(
        f"| {md_escape(group)} | {md_escape(check['map'])} | {md_escape(check['desc'])} | "
        f"{md_escape(status)} | {md_escape(note)} |\n"
    )


def render_markdown_report(
    rows,
    md_path,
    map_folder,
    show_bombs=False,
    show_obs=False,
    show_strategy=False,
    focus_regression=False):
    md_path.parent.mkdir(parents=True, exist_ok=True)
    timeout_rows = [
        row for row in rows
        if row.get("timeout") == "1" or row.get("status") == "timeout"
    ]
    failed_rows = [
        row for row in rows
        if row.get("status") not in ("ok", "timeout")
    ]

    display_fields = [
        "mark",
        "map",
        "case_index",
        "box_semantics",
        "target_semantics",
        "status",
        "elapsed_ms",
        "phase1_bomb_ms",
        "patrol_ms",
        "phase2_bomb_ms",
        "sokoban_ms",
        "total_steps",
        "total_turns",
        "total_observations",
        "total_rotations",
        "total_cost",
    ]
    if show_obs:
        display_fields.append("chosen_obs")
    if show_bombs:
        display_fields.extend(["bomb_tasks_1", "bomb_tasks_2"])
    if show_strategy:
        display_fields.append("strategy_hot_profile")
        display_fields.append("strategy_shadow_clear")
        display_fields.append("strategy_shadow_decision")
        display_fields.append("strategy_shadow_class")
        display_fields.append("strategy_clear_diag")

    field_labels = {
        "mark": "标记",
        "map": "地图",
        "case_index": "用例",
        "box_semantics": "箱子语义",
        "target_semantics": "目标语义",
        "status": "状态",
        "elapsed_ms": "总耗时ms",
        "phase1_bomb_ms": "阶段1炸弹ms",
        "patrol_ms": "巡图ms",
        "phase2_bomb_ms": "阶段2炸弹ms",
        "sokoban_ms": "推箱ms",
        "total_steps": "总步数",
        "total_turns": "总拐弯",
        "total_observations": "总观测",
        "total_rotations": "总旋转",
        "total_cost": "总代价",
        "chosen_obs": "观测点",
        "bomb_tasks_1": "阶段1炸弹任务",
        "bomb_tasks_2": "阶段2炸弹任务",
        "strategy_hot_profile": "Strategy热点",
        "strategy_shadow_clear": "Shadow清障",
        "strategy_shadow_decision": "Shadow决策",
        "strategy_shadow_class": "Shadow分类",
        "strategy_clear_diag": "Strategy清障",
    }

    def row_mark(row):
        if row.get("timeout") == "1" or row.get("status") == "timeout":
            return "超时"
        if row.get("status") != "ok":
            return "失败"
        return ""

    def write_table(out, table_rows, fields):
        out.write("| " + " | ".join(field_labels.get(field, field) for field in fields) + " |\n")
        out.write("| " + " | ".join(["---"] * len(fields)) + " |\n")
        for row in table_rows:
            values = []
            for field in fields:
                values.append(row_mark(row) if field == "mark" else row.get(field, ""))
            out.write("| " + " | ".join(md_escape(value) for value in values) + " |\n")

    with md_path.open("w", encoding="utf-8", newline="\n") as out:
        out.write("# Sokoban 回归测试报告\n\n")
        out.write(f"- 地图目录：`{map_folder}`\n")
        out.write(f"- 总用例数：{len(rows)}\n")
        out.write(f"- 阶段超时阈值：任一阶段超过 {STAGE_TIMEOUT_MS}ms\n")
        out.write(f"- 进程失败阈值：总耗时超过 {int(TIMEOUT_SECONDS * 1000)}ms 未返回\n")
        out.write(f"- 超时用例数：{len(timeout_rows)}\n")
        out.write(f"- 失败用例数：{len(failed_rows)}\n")
        out.write(f"- 炸弹任务显示：{'开启' if show_bombs else '关闭'}\n")
        out.write(f"- 观测点显示：{'开启' if show_obs else '关闭'}\n")
        out.write(f"- Strategy 清障显示：{'开启' if show_strategy else '关闭'}\n\n")

        out.write("## 超时汇总\n\n")
        if timeout_rows:
            write_table(out, timeout_rows, display_fields)
        else:
            out.write("没有超时用例\n")
        out.write("\n## 失败汇总\n\n")
        if failed_rows:
            write_table(out, failed_rows, display_fields)
        else:
            out.write("没有失败用例\n")
        if focus_regression:
            render_focus_regression_report(out, rows, map_folder)
        out.write("\n## 全部用例\n\n")
        write_table(out, rows, display_fields)


def parse_args():
    parser = argparse.ArgumentParser(description="批量运行 Sokoban 地图回归测试，枚举 1 对 1 语义映射")
    parser.add_argument("--solver", required=True, type=Path, help="solver.exe 路径")
    parser.add_argument("--maps", required=True, type=Path, help="地图文件夹路径")
    parser.add_argument("--output", type=Path, help="输出 Markdown 报告路径，默认 test/<地图文件夹名>_test.md")
    parser.add_argument("--md-output", type=Path, help="输出 Markdown 报告路径，兼容旧参数")
    parser.add_argument("--timeout", type=float, default=TIMEOUT_SECONDS, help="单个语义用例进程超时时间，单位秒，最多按 1 秒执行")
    parser.add_argument("--max-cases", type=int, default=0, help="每张地图最多运行多少个语义用例，0 表示不限制")
    parser.add_argument("--start-x", type=int, default=PLAN_START_X, help="固定起点 X，默认 4")
    parser.add_argument("--start-y", type=int, default=PLAN_START_Y, help="固定起点 Y，默认 1")
    parser.add_argument("--use-map-start", action="store_true", help="兼容旧地图：使用地图文件里的 @ 作为起点")
    parser.add_argument("--show-bombs", action="store_true", help="Markdown 报告显示炸弹任务")
    parser.add_argument("--show-obs", action="store_true", help="Markdown 报告显示观测点")
    parser.add_argument("--show-details", action="store_true", help="Markdown 报告同时显示炸弹任务、观测点和 Strategy 清障摘要")
    parser.add_argument("--focus-regression", action="store_true", help="只运行文件开头配置的重点观察地图，并输出重点图护栏")
    return parser.parse_args()


def main():
    args = parse_args()
    root = Path(__file__).resolve().parents[1]
    solver = args.solver.resolve()
    maps_dir = args.maps.resolve()
    if args.md_output:
        md_output = args.md_output.resolve()
    elif args.focus_regression:
        md_output = (args.output.resolve() if args.output else
                     root / "test" / (maps_dir.name + "_focus_test.md"))
    else:
        md_output = (args.output.resolve() if args.output else
                     root / "test" / (map_folder_label(maps_dir) + ".md"))
    work_dir = root / "visualizer"
    path_output = work_dir / "path_output.txt"

    if not solver.exists():
        print(f"solver not found: {solver}", file=sys.stderr)
        return 2
    if not maps_dir.exists() or not maps_dir.is_dir():
        print(f"map folder not found: {maps_dir}", file=sys.stderr)
        return 3
    if not work_dir.exists():
        print(f"visualizer work dir not found: {work_dir}", file=sys.stderr)
        return 4

    git_info = get_git_info(root)
    run_start = datetime.now()
    run_id = run_start.strftime("%Y%m%d_%H%M%S")
    run_timestamp = run_start.isoformat(timespec="seconds")
    solver_mtime = solver_stamp(solver)
    process_timeout = min(args.timeout, TIMEOUT_SECONDS)
    wrote_run_header = False
    rows = []

    maps = sorted(maps_dir.glob("*.txt"), key=lambda p: natural_key(p.name))
    if args.focus_regression:
        maps = focus_map_paths(maps_dir)
        if not maps:
            print(f"focus maps not configured for: {maps_dir.name}", file=sys.stderr)
            return 5
    total_cases = 0
    print(f"run_id={run_id} maps={len(maps)} md={md_output}")

    for map_path in maps:
        map_folder_name = map_path.parent.name
        map_display_name = (
            str(map_path.relative_to(maps_dir)).replace("\\", "/")
            if args.focus_regression and map_path.parent != maps_dir
            else map_path.name
        )
        map_info = read_map(
            map_path,
            start_x=args.start_x,
            start_y=args.start_y,
            use_map_start=args.use_map_start,
        )
        box_count = len(map_info["boxes"])
        target_count = len(map_info["targets"])
        if box_count <= 0 or box_count != target_count or box_count > 10:
            continue

        cases = build_semantic_cases(box_count)
        if args.max_cases > 0:
            cases = cases[: args.max_cases]

        for case_index, (box_sem, target_sem) in enumerate(cases):
            total_cases += 1
            if path_output.exists():
                unlink_with_retry(path_output)
            write_map_input(work_dir, map_info, box_sem, target_sem)

            start = time.perf_counter()
            timeout = False
            status = "ok"
            try:
                proc = subprocess.run(
                    [str(solver)],
                    cwd=work_dir,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=process_timeout,
                )
                if proc.returncode != 0:
                    status = f"exit_{proc.returncode}"
            except subprocess.TimeoutExpired:
                status = "failed"
            elapsed_ms = int((time.perf_counter() - start) * 1000)

            parsed = parse_output(path_output, map_info) if not timeout else {
                "failed": "1",
                "solver_build": "",
                "times": ["", "", "", ""],
                "profile": {},
                "patrol_steps": "",
                "sokoban_steps": "",
                "total_steps": "",
                "patrol_turns": "",
                "sokoban_turns": "",
                "total_turns": "",
                "patrol_rotations": "",
                "sokoban_rotations": "",
                "total_rotations": "",
                "patrol_observations": "",
                "sokoban_observations": "",
                "total_observations": "",
                "total_cost": "",
                "pushes": "",
                "completed": "",
                "bomb_tasks_1": "",
                "bomb_tasks_2": "",
                "chosen_obs": "",
                "strategy_hot_profile": "",
                "strategy_shadow_clear": "",
                "strategy_shadow_decision": "",
                "strategy_shadow_class": "",
                "strategy_clear_diag": "",
            }
            if status == "ok" and parsed["failed"] == "1":
                status = "failed"

            times = parsed["times"]
            if status == "ok" and has_stage_timeout(times):
                timeout = True
                status = "timeout"
            profile = parsed["profile"]
            write_run_info = not wrote_run_header
            row = {
                "run_id": run_id if write_run_info else "",
                "timestamp": run_timestamp if write_run_info else "",
                "solver": str(solver) if write_run_info else "",
                "solver_mtime": solver_mtime if write_run_info else "",
                "solver_build": parsed["solver_build"] if write_run_info else "",
                "git_head": git_info["head"] if write_run_info else "",
                "git_short_head": git_info["short_head"] if write_run_info else "",
                "tree_state": git_info["tree_state"] if write_run_info else "",
                "dirty": git_info["dirty"] if write_run_info else "",
                "head_data": ("1" if git_info["dirty"] == "0" else "0") if write_run_info else "",
                "map_folder": str(maps_dir) if write_run_info else "",
                "start_x": str(args.start_x) if write_run_info else "",
                "start_y": str(args.start_y) if write_run_info else "",
                "use_map_start": "1" if args.use_map_start and write_run_info else ("0" if write_run_info else ""),
                "map": map_display_name,
                "map_file": map_path.name,
                "map_folder_name": map_folder_name,
                "box_count": str(box_count),
                "target_count": str(target_count),
                "bomb_count": str(len(map_info["bombs"])),
                "case_index": str(case_index),
                "box_semantics": box_sem,
                "target_semantics": target_sem,
                "status": status,
                "timeout": "1" if timeout else "0",
                "elapsed_ms": str(elapsed_ms),
                "phase1_bomb_ms": times[0],
                "patrol_ms": times[1],
                "phase2_bomb_ms": times[2],
                "sokoban_ms": times[3],
                "patrol_steps": parsed["patrol_steps"],
                "sokoban_steps": parsed["sokoban_steps"],
                "total_steps": parsed["total_steps"],
                "patrol_turns": parsed["patrol_turns"],
                "sokoban_turns": parsed["sokoban_turns"],
                "total_turns": parsed["total_turns"],
                "patrol_rotations": parsed["patrol_rotations"],
                "sokoban_rotations": parsed["sokoban_rotations"],
                "total_rotations": parsed["total_rotations"],
                "patrol_observations": parsed["patrol_observations"],
                "sokoban_observations": parsed["sokoban_observations"],
                "total_observations": parsed["total_observations"],
                "total_cost": parsed["total_cost"],
                "pushes": parsed["pushes"],
                "completed_targets": parsed["completed"],
                "expanded_nodes": profile.get("expanded_nodes", ""),
                "generated_moves": profile.get("generated_moves", ""),
                "tt_hits": profile.get("tt_hits", ""),
                "max_depth": profile.get("max_depth", ""),
                "threshold_iterations": profile.get("threshold_iterations", ""),
                "final_threshold": profile.get("final_threshold", ""),
                "nps": profile.get("nps", ""),
                "bomb_tasks_1": parsed["bomb_tasks_1"],
                "bomb_tasks_2": parsed["bomb_tasks_2"],
                "chosen_obs": parsed["chosen_obs"],
                "strategy_hot_profile": parsed["strategy_hot_profile"],
                "strategy_shadow_clear": parsed["strategy_shadow_clear"],
                "strategy_shadow_decision": parsed["strategy_shadow_decision"],
                "strategy_shadow_class": parsed["strategy_shadow_class"],
                "strategy_clear_diag": parsed["strategy_clear_diag"],
            }
            rows.append(row)
            wrote_run_header = True
            print(f"{map_display_name} case={case_index} sem={box_sem}->{target_sem} {status} {elapsed_ms}ms")

    render_markdown_report(
        rows,
        md_output,
        maps_dir,
        show_bombs=args.show_bombs or args.show_details,
        show_obs=args.show_obs or args.show_details,
        show_strategy=args.show_details,
        focus_regression=args.focus_regression,
    )
    print(f"done cases={total_cases} md={md_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


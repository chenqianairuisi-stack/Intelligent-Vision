#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "../../project/Core/system_config.h"

/* 
.\simulation\map\run_map_generator.ps1 -Build -Count 20 -Boxes 3 -OutDir map\map_generated
.\simulation\map\run_map_generator.ps1 -Build -Mode with-bomb -Bombs 3 -Count 20 -Boxes 3 -OutDir map\map_generated -Prefix bomb
.\simulation\map\run_map_generator.ps1 -Build -Style legacy -Mode with-bomb -Bombs 3 -Count 20 -Boxes 3 -WallDensity 0.4 -MinPairPushes 8 -QualityCandidates 100
.\simulation\map\run_map_generator.ps1 -Build -Mode with-bomb -Difficulty hard -Bombs 3 -Count 20 -Boxes 3 -WallDensity 0.30 -MinPairPushes 4 -QualityCandidates 4 -MaxAttempts 25000 -WriteMeta
*/


using namespace SystemConfig;

namespace {

struct Cell {
    int x = 0;
    int y = 0;
};

struct Dir {
    int dx = 0;
    int dy = 0;
};

constexpr Dir DIRS[4] = {
    {0, -1},
    {1, 0},
    {0, 1},
    {-1, 0},
};

constexpr double MAX_DISPERSED_WALL_DENSITY = 0.28;
constexpr int MAX_EFFECTIVE_QUALITY_CANDIDATES = 12;
constexpr int MAX_DISPERSED_MIN_PAIR_PUSHES = 6;
constexpr double HARD_MAX_WALL_DENSITY = 0.40;
constexpr int HARD_MAX_EFFECTIVE_QUALITY_CANDIDATES = 96;
constexpr int HARD_MAX_MIN_PAIR_PUSHES = 12;
constexpr double OFFICIAL_MAX_WALL_DENSITY = 0.18;
constexpr double OFFICIAL_HARD_MAX_WALL_DENSITY = 0.24;
constexpr int OFFICIAL_MAX_INTERNAL_WALLS = 23;
constexpr int OFFICIAL_HARD_MAX_INTERNAL_WALLS = 28;

enum class GeneratorMode {
    NO_BOMB,
    WITH_BOMB,
};

enum class DifficultyMode {
    NORMAL,
    HARD,
};

enum class GeneratorStyle {
    OFFICIAL,
    LEGACY,
};

struct GeneratorConfig {
    GeneratorMode mode = GeneratorMode::NO_BOMB;
    DifficultyMode difficulty = DifficultyMode::NORMAL;
    GeneratorStyle style = GeneratorStyle::OFFICIAL;
    int count = 10;
    int boxes = 3;
    int bombs = 0;
    int max_attempts = 50000;
    int quality_candidates = 56;
    int min_pair_pushes = 4;
    int min_bomb_required_pairs = 1;
    int min_entity_spacing = 2;
    double wall_density = 0.14;
    uint32_t seed = 0;
    bool write_meta = false;
    bool require_phase2_specific_bomb = false;
    std::string out_dir = "map/map_generated";
    std::string prefix = "gen";
};

struct Grid {
    bool wall[MAP_MAX_HEIGHT][MAP_MAX_WIDTH] = {};
};

struct GeneratedMap {
    Grid grid;
    Cell player;
    std::vector<Cell> boxes;
    std::vector<Cell> targets;
    std::vector<Cell> bombs;
    std::vector<Cell> bomb_targets;
    int wall_count = 0;
    int dead_end_count = 0;
    int corridor_count = 0;
    int pocket_count = 0;
    int wall_block_count = 0;
    int cul_de_sac_count = 0;
    int wall_component_count = 0;
    int largest_wall_component = 0;
    int thin_wall_count = 0;
    int thick_wall_count = 0;
    int entity_mix_score = 0;
    int bomb_required_pairs = 0;
    int bomb_improved_pairs = 0;
    int bomb_after_bad_pairs = 0;
    int bomb_after_matching = 0;
    int gateway_score = 0;
    int min_pair_pushes = 0;
    int max_pair_pushes = 0;
    double avg_pair_pushes = 0.0;
    double bomb_score = 0.0;
    double score = -1.0;
};

int cell_index(Cell c) {
    return c.y * MAP_MAX_WIDTH + c.x;
}

bool same_cell(Cell a, Cell b) {
    return a.x == b.x && a.y == b.y;
}

int manhattan(Cell a, Cell b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

bool in_bounds(int x, int y) {
    return x >= 0 && x < MAP_MAX_WIDTH && y >= 0 && y < MAP_MAX_HEIGHT;
}

bool is_floor(const Grid& grid, int x, int y) {
    return in_bounds(x, y) && !grid.wall[y][x];
}

Cell logical_to_file_cell(int x, int y) {
    return {x, MAP_MAX_HEIGHT - 1 - y};
}

bool is_reserved_start_cell(Cell c) {
    constexpr int RESERVED[][2] = {
        {5, 1}, {6, 1}, {5, 2}, {6, 2},
        {5, 13}, {6, 13}, {5, 14}, {6, 14},
    };
    for (const auto& item : RESERVED) {
        Cell file_cell = logical_to_file_cell(item[0], item[1]);
        if (same_cell(c, file_cell)) return true;
    }
    return false;
}

Cell fixed_player_start_file_cell() {
    return logical_to_file_cell(PLAN_START_X, PLAN_START_Y);
}

void open_reserved_start_zones(Grid& grid) {
    for (int logical_y : {1, 2, 13, 14}) {
        for (int x : {5, 6}) {
            Cell c = logical_to_file_cell(x, logical_y);
            if (in_bounds(c.x, c.y)) grid.wall[c.y][c.x] = false;
        }
    }
}

int count_walls(const Grid& grid) {
    int count = 0;
    for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
            if (grid.wall[y][x]) ++count;
        }
    }
    return count;
}

int count_floor_neighbors(const Grid& grid, Cell c) {
    int count = 0;
    for (const Dir& d : DIRS) {
        if (is_floor(grid, c.x + d.dx, c.y + d.dy)) ++count;
    }
    return count;
}

int count_dead_ends(const Grid& grid) {
    int count = 0;
    for (int y = 1; y < MAP_MAX_HEIGHT - 1; ++y) {
        for (int x = 1; x < MAP_MAX_WIDTH - 1; ++x) {
            if (!grid.wall[y][x] && count_floor_neighbors(grid, {x, y}) <= 1) ++count;
        }
    }
    return count;
}

int count_corridor_cells(const Grid& grid) {
    int count = 0;
    for (int y = 1; y < MAP_MAX_HEIGHT - 1; ++y) {
        for (int x = 1; x < MAP_MAX_WIDTH - 1; ++x) {
            if (grid.wall[y][x]) continue;
            bool up = is_floor(grid, x, y - 1);
            bool down = is_floor(grid, x, y + 1);
            bool left = is_floor(grid, x - 1, y);
            bool right = is_floor(grid, x + 1, y);
            if ((up && down && !left && !right) ||
                (left && right && !up && !down)) {
                ++count;
            }
        }
    }
    return count;
}

int count_corner_pockets(const Grid& grid) {
    int count = 0;
    for (int y = 1; y < MAP_MAX_HEIGHT - 1; ++y) {
        for (int x = 1; x < MAP_MAX_WIDTH - 1; ++x) {
            if (grid.wall[y][x]) continue;
            bool up_wall = grid.wall[y - 1][x];
            bool down_wall = grid.wall[y + 1][x];
            bool left_wall = grid.wall[y][x - 1];
            bool right_wall = grid.wall[y][x + 1];
            if ((up_wall && left_wall) || (up_wall && right_wall) ||
                (down_wall && left_wall) || (down_wall && right_wall)) {
                ++count;
            }
        }
    }
    return count;
}

int count_wall_blocks_2x2(const Grid& grid) {
    int count = 0;
    for (int y = 1; y < MAP_MAX_HEIGHT - 2; ++y) {
        for (int x = 1; x < MAP_MAX_WIDTH - 2; ++x) {
            if (grid.wall[y][x] && grid.wall[y + 1][x] &&
                grid.wall[y][x + 1] && grid.wall[y + 1][x + 1]) {
                ++count;
            }
        }
    }
    return count;
}

int count_thin_wall_cells(const Grid& grid) {
    int count = 0;
    for (int y = 1; y < MAP_MAX_HEIGHT - 1; ++y) {
        for (int x = 1; x < MAP_MAX_WIDTH - 1; ++x) {
            if (!grid.wall[y][x]) continue;
            bool up = grid.wall[y - 1][x];
            bool down = grid.wall[y + 1][x];
            bool left = grid.wall[y][x - 1];
            bool right = grid.wall[y][x + 1];
            if ((left || right) && !(up || down)) ++count;
            else if ((up || down) && !(left || right)) ++count;
        }
    }
    return count;
}

int count_thick_wall_cells(const Grid& grid) {
    int count = 0;
    for (int y = 1; y < MAP_MAX_HEIGHT - 1; ++y) {
        for (int x = 1; x < MAP_MAX_WIDTH - 1; ++x) {
            if (!grid.wall[y][x]) continue;
            int wall_neighbors = 0;
            for (const Dir& d : DIRS) {
                int nx = x + d.dx;
                int ny = y + d.dy;
                if (in_bounds(nx, ny) && grid.wall[ny][nx]) ++wall_neighbors;
            }
            if (wall_neighbors >= 3) ++count;
        }
    }
    return count;
}

void count_wall_components(const Grid& grid, int& component_count, int& largest_component) {
    bool visited[MAP_MAX_HEIGHT][MAP_MAX_WIDTH] = {};
    component_count = 0;
    largest_component = 0;

    for (int y = 1; y < MAP_MAX_HEIGHT - 1; ++y) {
        for (int x = 1; x < MAP_MAX_WIDTH - 1; ++x) {
            if (!grid.wall[y][x] || visited[y][x]) continue;
            ++component_count;
            int size = 0;
            std::queue<Cell> q;
            q.push({x, y});
            visited[y][x] = true;
            while (!q.empty()) {
                Cell cur = q.front();
                q.pop();
                ++size;
                for (const Dir& d : DIRS) {
                    Cell next{cur.x + d.dx, cur.y + d.dy};
                    if (!in_bounds(next.x, next.y) || visited[next.y][next.x] || !grid.wall[next.y][next.x]) continue;
                    visited[next.y][next.x] = true;
                    q.push(next);
                }
            }
            largest_component = std::max(largest_component, size);
        }
    }
}

int count_cul_de_sac_cells(const Grid& grid) {
    bool alive[MAP_MAX_HEIGHT][MAP_MAX_WIDTH] = {};
    int degree[MAP_MAX_HEIGHT][MAP_MAX_WIDTH] = {};
    std::queue<Cell> q;

    for (int y = 1; y < MAP_MAX_HEIGHT - 1; ++y) {
        for (int x = 1; x < MAP_MAX_WIDTH - 1; ++x) {
            if (grid.wall[y][x] || is_reserved_start_cell({x, y})) continue;
            alive[y][x] = true;
            degree[y][x] = count_floor_neighbors(grid, {x, y});
            if (degree[y][x] <= 1) q.push({x, y});
        }
    }

    int count = 0;
    while (!q.empty()) {
        Cell cur = q.front();
        q.pop();
        if (!alive[cur.y][cur.x]) continue;
        alive[cur.y][cur.x] = false;
        ++count;

        for (const Dir& d : DIRS) {
            Cell next{cur.x + d.dx, cur.y + d.dy};
            if (!in_bounds(next.x, next.y) || !alive[next.y][next.x]) continue;
            --degree[next.y][next.x];
            if (degree[next.y][next.x] <= 1) q.push(next);
        }
    }

    return count;
}

bool is_inner_wall(const Grid& grid, int x, int y) {
    return x > 0 && x < MAP_MAX_WIDTH - 1 &&
        y > 0 && y < MAP_MAX_HEIGHT - 1 &&
        grid.wall[y][x];
}

int count_blast_wall_mass(const Grid& grid, Cell wall) {
    int count = 0;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            int nx = wall.x + dx;
            int ny = wall.y + dy;
            if (is_inner_wall(grid, nx, ny)) ++count;
        }
    }
    return count;
}

int count_blast_floor_touch(const Grid& grid, Cell wall) {
    int count = 0;
    bool seen[MAP_MAX_HEIGHT][MAP_MAX_WIDTH] = {};
    for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
            int nx = wall.x + dx;
            int ny = wall.y + dy;
            if (!is_floor(grid, nx, ny) || seen[ny][nx]) continue;
            seen[ny][nx] = true;
            ++count;
        }
    }
    return count;
}

int score_blast_wall(const Grid& grid, Cell wall) {
    if (!is_inner_wall(grid, wall.x, wall.y)) return -1;
    int direct_floor_neighbors = count_floor_neighbors(grid, wall);
    if (direct_floor_neighbors <= 0) return -1;

    int wall_mass = count_blast_wall_mass(grid, wall);
    int floor_touch = count_blast_floor_touch(grid, wall);
    int score = wall_mass * 28 + floor_touch * 3 + direct_floor_neighbors * 12;
    if (wall_mass <= 1) score -= 30;
    if (direct_floor_neighbors >= 3) score += 18;
    return score;
}

void apply_blast(Grid& grid, Cell wall) {
    if (!is_inner_wall(grid, wall.x, wall.y)) return;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            int nx = wall.x + dx;
            int ny = wall.y + dy;
            if (nx <= 0 || nx >= MAP_MAX_WIDTH - 1 || ny <= 0 || ny >= MAP_MAX_HEIGHT - 1) continue;
            grid.wall[ny][nx] = false;
        }
    }
}

Grid blasted_grid(const GeneratedMap& map) {
    Grid grid = map.grid;
    for (Cell wall : map.bomb_targets) {
        apply_blast(grid, wall);
    }
    return grid;
}

std::vector<Cell> collect_floor_cells(const Grid& grid) {
    std::vector<Cell> cells;
    for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
            if (!grid.wall[y][x]) cells.push_back({x, y});
        }
    }
    return cells;
}

bool all_floor_connected(const Grid& grid) {
    std::vector<Cell> cells = collect_floor_cells(grid);
    if (cells.empty()) return false;

    bool visited[MAP_MAX_HEIGHT][MAP_MAX_WIDTH] = {};
    std::queue<Cell> q;
    q.push(cells.front());
    visited[cells.front().y][cells.front().x] = true;

    int reached = 0;
    while (!q.empty()) {
        Cell cur = q.front();
        q.pop();
        ++reached;
        for (const Dir& d : DIRS) {
            Cell next{cur.x + d.dx, cur.y + d.dy};
            if (!is_floor(grid, next.x, next.y) || visited[next.y][next.x]) continue;
            visited[next.y][next.x] = true;
            q.push(next);
        }
    }

    return reached == static_cast<int>(cells.size());
}

int reachable_floor_count(const Grid& grid, Cell start) {
    if (!is_floor(grid, start.x, start.y)) return 0;
    bool visited[MAP_MAX_HEIGHT][MAP_MAX_WIDTH] = {};
    std::queue<Cell> q;
    q.push(start);
    visited[start.y][start.x] = true;
    int count = 0;

    while (!q.empty()) {
        Cell cur = q.front();
        q.pop();
        ++count;
        for (const Dir& d : DIRS) {
            Cell next{cur.x + d.dx, cur.y + d.dy};
            if (!is_floor(grid, next.x, next.y) || visited[next.y][next.x]) continue;
            visited[next.y][next.x] = true;
            q.push(next);
        }
    }
    return count;
}

bool has_two_sided_push_axis(const Grid& grid, Cell c) {
    for (const Dir& d : DIRS) {
        if (is_floor(grid, c.x + d.dx, c.y + d.dy) &&
            is_floor(grid, c.x - d.dx, c.y - d.dy)) {
            return true;
        }
    }
    return false;
}

bool is_near_immutable_border(Cell c) {
    return c.x <= 1 || c.x >= MAP_MAX_WIDTH - 2 ||
        c.y <= 1 || c.y >= MAP_MAX_HEIGHT - 2;
}

void calc_player_reach(const Grid& grid, Cell player, Cell box, bool out[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]) {
    for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) out[y][x] = false;
    }
    if (!is_floor(grid, player.x, player.y) || same_cell(player, box)) return;

    std::queue<Cell> q;
    q.push(player);
    out[player.y][player.x] = true;

    while (!q.empty()) {
        Cell cur = q.front();
        q.pop();
        for (const Dir& d : DIRS) {
            Cell next{cur.x + d.dx, cur.y + d.dy};
            if (!is_floor(grid, next.x, next.y)) continue;
            if (same_cell(next, box) || out[next.y][next.x]) continue;
            out[next.y][next.x] = true;
            q.push(next);
        }
    }
}

int single_box_push_distance(const Grid& grid, Cell player_start, Cell box_start, Cell target) {
    if (!is_floor(grid, player_start.x, player_start.y)) return -1;
    if (!is_floor(grid, box_start.x, box_start.y)) return -1;
    if (!is_floor(grid, target.x, target.y)) return -1;
    if (same_cell(player_start, box_start)) return -1;
    if (same_cell(box_start, target)) return 0;

    constexpr int STATE_COUNT = MAP_CELL_COUNT * MAP_CELL_COUNT;
    std::vector<int16_t> dist(STATE_COUNT, -1);
    std::queue<std::pair<Cell, Cell>> q;

    int start_index = cell_index(box_start) * MAP_CELL_COUNT + cell_index(player_start);
    dist[start_index] = 0;
    q.push({box_start, player_start});

    bool reach[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    while (!q.empty()) {
        auto [box, player] = q.front();
        q.pop();
        int state_index = cell_index(box) * MAP_CELL_COUNT + cell_index(player);
        int cur_dist = dist[state_index];

        calc_player_reach(grid, player, box, reach);
        for (const Dir& d : DIRS) {
            Cell stand{box.x - d.dx, box.y - d.dy};
            Cell next_box{box.x + d.dx, box.y + d.dy};
            if (!is_floor(grid, stand.x, stand.y) || !is_floor(grid, next_box.x, next_box.y)) continue;
            if (!reach[stand.y][stand.x]) continue;

            Cell next_player = box;
            int next_index = cell_index(next_box) * MAP_CELL_COUNT + cell_index(next_player);
            if (dist[next_index] >= 0) continue;

            dist[next_index] = static_cast<int16_t>(cur_dist + 1);
            if (same_cell(next_box, target)) return cur_dist + 1;
            q.push({next_box, next_player});
        }
    }

    return -1;
}

int bomb_push_wall_distance(const Grid& grid, Cell player_start, Cell bomb_start, Cell target_wall) {
    if (!is_floor(grid, player_start.x, player_start.y)) return -1;
    if (!is_floor(grid, bomb_start.x, bomb_start.y)) return -1;
    if (!is_inner_wall(grid, target_wall.x, target_wall.y)) return -1;
    if (same_cell(player_start, bomb_start)) return -1;

    constexpr int STATE_COUNT = MAP_CELL_COUNT * MAP_CELL_COUNT;
    std::vector<int16_t> dist(STATE_COUNT, -1);
    std::queue<std::pair<Cell, Cell>> q;

    int start_index = cell_index(bomb_start) * MAP_CELL_COUNT + cell_index(player_start);
    dist[start_index] = 0;
    q.push({bomb_start, player_start});

    bool reach[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    while (!q.empty()) {
        auto [bomb, player] = q.front();
        q.pop();
        int state_index = cell_index(bomb) * MAP_CELL_COUNT + cell_index(player);
        int cur_dist = dist[state_index];

        calc_player_reach(grid, player, bomb, reach);
        for (const Dir& d : DIRS) {
            Cell stand{bomb.x - d.dx, bomb.y - d.dy};
            Cell next_bomb{bomb.x + d.dx, bomb.y + d.dy};
            if (!is_floor(grid, stand.x, stand.y) || !reach[stand.y][stand.x]) continue;
            if (same_cell(next_bomb, target_wall)) return cur_dist + 1;
            if (!is_floor(grid, next_bomb.x, next_bomb.y)) continue;

            Cell next_player = bomb;
            int next_index = cell_index(next_bomb) * MAP_CELL_COUNT + cell_index(next_player);
            if (dist[next_index] >= 0) continue;

            dist[next_index] = static_cast<int16_t>(cur_dist + 1);
            q.push({next_bomb, next_player});
        }
    }

    return -1;
}

bool contains_cell(const std::vector<Cell>& cells, Cell c) {
    for (Cell item : cells) {
        if (same_cell(item, c)) return true;
    }
    return false;
}

bool spacing_ok(const std::vector<Cell>& cells, Cell candidate, int min_spacing) {
    for (Cell existing : cells) {
        if (manhattan(existing, candidate) < min_spacing) return false;
    }
    return true;
}

bool add_wall_cell(Grid& grid, std::vector<Cell>& changed, int x, int y) {
    if (x <= 0 || x >= MAP_MAX_WIDTH - 1 || y <= 0 || y >= MAP_MAX_HEIGHT - 1) return false;
    if (is_reserved_start_cell({x, y})) return false;
    if (grid.wall[y][x]) return false;
    grid.wall[y][x] = true;
    changed.push_back({x, y});
    return true;
}

void add_wall_line(Grid& grid, std::vector<Cell>& changed, int x, int y, int dx, int dy, int len) {
    for (int i = 0; i < len; ++i) {
        add_wall_cell(grid, changed, x + dx * i, y + dy * i);
    }
}

void add_u_shape(Grid& grid, std::vector<Cell>& changed, int x, int y, int w, int h, int open_side) {
    for (int yy = 0; yy < h; ++yy) {
        for (int xx = 0; xx < w; ++xx) {
            bool top = yy == 0;
            bool bottom = yy == h - 1;
            bool left = xx == 0;
            bool right = xx == w - 1;
            bool boundary = top || bottom || left || right;
            if (!boundary) continue;
            if ((open_side == 0 && top) || (open_side == 1 && right) ||
                (open_side == 2 && bottom) || (open_side == 3 && left)) {
                continue;
            }
            add_wall_cell(grid, changed, x + xx, y + yy);
        }
    }
}

void add_comb_shape(Grid& grid, std::vector<Cell>& changed, int x, int y, bool horizontal, int len, int tooth_dir) {
    int dx = horizontal ? 1 : 0;
    int dy = horizontal ? 0 : 1;
    add_wall_line(grid, changed, x, y, dx, dy, len);
    for (int i = 1; i < len - 1; i += 2) {
        int bx = x + dx * i;
        int by = y + dy * i;
        int tx = horizontal ? 0 : tooth_dir;
        int ty = horizontal ? tooth_dir : 0;
        add_wall_line(grid, changed, bx + tx, by + ty, tx, ty, 2);
    }
}

bool wall_clumping_ok(const Grid& grid, const GeneratorConfig& cfg) {
    int component_count = 0;
    int largest_component = 0;
    count_wall_components(grid, component_count, largest_component);
    bool hard = cfg.difficulty == DifficultyMode::HARD;
    if (largest_component > (hard ? 34 : 18)) return false;
    if (count_thick_wall_cells(grid) > (hard ? 30 : 18)) return false;
    if (count_wall_blocks_2x2(grid) > (hard ? 18 : 10)) return false;
    return true;
}

bool grid_wall_layout_quality_ok(const Grid& grid, const GeneratorConfig& cfg) {
    int component_count = 0;
    int largest_component = 0;
    count_wall_components(grid, component_count, largest_component);
    bool hard = cfg.difficulty == DifficultyMode::HARD;
    if (cfg.style == GeneratorStyle::OFFICIAL) {
        // 正式赛样本的内部墙体保持稀疏，且普通地图不出现厚墙块
        const int border_walls = 2 * MAP_MAX_WIDTH + 2 * MAP_MAX_HEIGHT - 4;
        const int internal_walls = count_walls(grid) - border_walls;
        const int interior_count = (MAP_MAX_WIDTH - 2) * (MAP_MAX_HEIGHT - 2);
        const int min_internal = std::max(
            8, static_cast<int>(cfg.wall_density * interior_count * 0.70));
        if (internal_walls < min_internal) return false;
        if (count_wall_blocks_2x2(grid) > 0) return false;
        if (count_thick_wall_cells(grid) > (hard ? 8 : 2)) return false;
        if (largest_component > (hard ? 14 : 8)) return false;
        if (component_count < 3) return false;
        return true;
    }
    if (component_count < (hard ? 2 : 3)) return false;
    if (largest_component > (hard ? 34 : 18)) return false;
    if (count_thick_wall_cells(grid) > (hard ? 30 : 18)) return false;
    if (count_wall_blocks_2x2(grid) > (hard ? 18 : 10)) return false;
    return true;
}

Grid make_legacy_grid(std::mt19937& rng, const GeneratorConfig& cfg) {
    Grid grid;
    for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
            grid.wall[y][x] = (x == 0 || y == 0 || x == MAP_MAX_WIDTH - 1 || y == MAP_MAX_HEIGHT - 1);
        }
    }
    open_reserved_start_zones(grid);

    const int interior_count = (MAP_MAX_WIDTH - 2) * (MAP_MAX_HEIGHT - 2);
    const int target_internal_walls = std::max(0, static_cast<int>(interior_count * cfg.wall_density));
    std::uniform_int_distribution<int> x_dist(1, MAP_MAX_WIDTH - 2);
    std::uniform_int_distribution<int> y_dist(1, MAP_MAX_HEIGHT - 2);
    std::uniform_int_distribution<int> len_dist(
        cfg.difficulty == DifficultyMode::HARD ? 4 : 3,
        cfg.difficulty == DifficultyMode::HARD ? 8 : 6
    );
    std::uniform_int_distribution<int> orient_dist(0, 1);
    std::uniform_int_distribution<int> shape_dist(0, cfg.difficulty == DifficultyMode::HARD ? 9 : 13);
    std::uniform_int_distribution<int> u_w_dist(3, 4);
    std::uniform_int_distribution<int> u_h_dist(3, 4);
    std::uniform_int_distribution<int> dir_dist(0, 3);

    int attempts = 0;
    while (count_walls(grid) - (2 * MAP_MAX_WIDTH + 2 * MAP_MAX_HEIGHT - 4) < target_internal_walls &&
           attempts < target_internal_walls * 16 + 80) {
        ++attempts;
        int x = x_dist(rng);
        int y = y_dist(rng);
        int len = len_dist(rng);
        bool horizontal = orient_dist(rng) == 0;

        std::vector<Cell> changed;
        for (int i = 0; i < len; ++i) {
            int nx = x + (horizontal ? i : 0);
            int ny = y + (horizontal ? 0 : i);
            if (nx <= 0 || nx >= MAP_MAX_WIDTH - 1 || ny <= 0 || ny >= MAP_MAX_HEIGHT - 1) break;
            add_wall_cell(grid, changed, nx, ny);
        }

        if (!changed.empty() && shape_dist(rng) <= (cfg.difficulty == DifficultyMode::HARD ? 1 : 0)) {
            Cell anchor = changed[changed.size() / 2];
            int branch_dir = shape_dist(rng) % 2 == 0 ? 1 : -1;
            int branch_len = cfg.difficulty == DifficultyMode::HARD ? 2 : 1;
            for (int i = 1; i <= branch_len; ++i) {
                int nx = anchor.x + (horizontal ? 0 : branch_dir * i);
                int ny = anchor.y + (horizontal ? branch_dir * i : 0);
                if (nx <= 0 || nx >= MAP_MAX_WIDTH - 1 || ny <= 0 || ny >= MAP_MAX_HEIGHT - 1) break;
                add_wall_cell(grid, changed, nx, ny);
            }
        }

        int shape = shape_dist(rng);
        if (shape == 1) {
            int w = u_w_dist(rng);
            int h = u_h_dist(rng);
            add_u_shape(grid, changed, x, y, w, h, dir_dist(rng));
        } else if (shape == 2) {
            int tooth_dir = dir_dist(rng) % 2 == 0 ? 1 : -1;
            add_comb_shape(grid, changed, x, y, horizontal, std::max(4, len), tooth_dir);
        }

        const int min_floor = std::max(36, cfg.boxes * 16);
        // hard 模式允许弱连通隔断，后续由炸弹验收保证可修复
        bool needs_connected_floor = cfg.difficulty != DifficultyMode::HARD;
        if (changed.empty() ||
            static_cast<int>(collect_floor_cells(grid).size()) < min_floor ||
            (needs_connected_floor && !all_floor_connected(grid)) ||
            !wall_clumping_ok(grid, cfg)) {
            for (Cell c : changed) grid.wall[c.y][c.x] = false;
        }
    }

    return grid;
}

// 正式赛地图以分散单格墙和短墙段为主，普通模式不生成 2x2 厚墙
Grid make_official_grid(std::mt19937& rng, const GeneratorConfig& cfg) {
    Grid grid;
    for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
            grid.wall[y][x] = (x == 0 || y == 0 ||
                x == MAP_MAX_WIDTH - 1 || y == MAP_MAX_HEIGHT - 1);
        }
    }
    open_reserved_start_zones(grid);

    const int interior_count = (MAP_MAX_WIDTH - 2) * (MAP_MAX_HEIGHT - 2);
    const bool hard = cfg.difficulty == DifficultyMode::HARD;
    const int max_internal_walls = hard
        ? OFFICIAL_HARD_MAX_INTERNAL_WALLS : OFFICIAL_MAX_INTERNAL_WALLS;
    const int target_internal_walls = std::min(
        max_internal_walls,
        std::max(8, static_cast<int>(std::lround(interior_count * cfg.wall_density))));
    std::uniform_int_distribution<int> x_dist(1, MAP_MAX_WIDTH - 2);
    std::uniform_int_distribution<int> y_dist(1, MAP_MAX_HEIGHT - 2);
    std::uniform_int_distribution<int> shape_dist(0, 99);
    std::uniform_int_distribution<int> dir_dist(0, 1);

    const int max_attempts = target_internal_walls * 48 + 160;
    for (int attempt = 0;
         count_walls(grid) - (2 * MAP_MAX_WIDTH + 2 * MAP_MAX_HEIGHT - 4) < target_internal_walls &&
         attempt < max_attempts;
         ++attempt) {
        const int x = x_dist(rng);
        const int y = y_dist(rng);
        const int shape = shape_dist(rng);
        const bool horizontal = dir_dist(rng) == 0;
        const int length = shape < 72 ? 1 : (shape < 96 ? 2 : (hard ? 3 : 2));
        std::vector<Cell> changed;
        for (int i = 0; i < length; ++i) {
            const int nx = x + (horizontal ? i : 0);
            const int ny = y + (horizontal ? 0 : i);
            add_wall_cell(grid, changed, nx, ny);
        }

        int component_count = 0;
        int largest_component = 0;
        count_wall_components(grid, component_count, largest_component);
        const bool invalid = changed.empty() || !all_floor_connected(grid) ||
            count_wall_blocks_2x2(grid) > 0 ||
            count_thick_wall_cells(grid) > (hard ? 8 : 2) ||
            largest_component > (hard ? 14 : 8);
        if (invalid) {
            for (Cell c : changed) grid.wall[c.y][c.x] = false;
        }
    }

    return grid;
}

Grid make_random_grid(std::mt19937& rng, const GeneratorConfig& cfg) {
    return cfg.style == GeneratorStyle::OFFICIAL
        ? make_official_grid(rng, cfg)
        : make_legacy_grid(rng, cfg);
}

bool evaluate_pair_reachability(const Grid& grid,
                                Cell player,
                                const std::vector<Cell>& boxes,
                                const std::vector<Cell>& targets,
                                const GeneratorConfig& cfg,
                                GeneratedMap& out) {
    struct PairStats {
        int good_pairs = 0;
        int bad_pairs = 0;
        int min_push = std::numeric_limits<int>::max();
        int max_push = 0;
        int sum_push = 0;
    };

    PairStats stats;

    for (Cell box : boxes) {
        for (Cell target : targets) {
            int dist = single_box_push_distance(grid, player, box, target);
            if (dist < cfg.min_pair_pushes) {
                ++stats.bad_pairs;
                continue;
            }
            ++stats.good_pairs;
            stats.min_push = std::min(stats.min_push, dist);
            stats.max_push = std::max(stats.max_push, dist);
            stats.sum_push += dist;
        }
    }

    if (stats.bad_pairs > 0 || stats.good_pairs == 0) return false;

    out.min_pair_pushes = stats.min_push;
    out.max_pair_pushes = stats.max_push;
    out.avg_pair_pushes = static_cast<double>(stats.sum_push) / stats.good_pairs;
    return true;
}

struct PairReachStats {
    int good_pairs = 0;
    int bad_pairs = 0;
    int min_push = std::numeric_limits<int>::max();
    int max_push = 0;
    int sum_push = 0;
    double avg_push = 0.0;
};

int max_bipartite_matching_size(const Grid& grid,
                                Cell player,
                                const std::vector<Cell>& boxes,
                                const std::vector<Cell>& targets,
                                int min_pair_pushes) {
    int target_count = static_cast<int>(targets.size());
    int mask_limit = 1 << target_count;
    std::vector<int> dp(mask_limit, -1);
    dp[0] = 0;

    for (int b = 0; b < static_cast<int>(boxes.size()); ++b) {
        std::vector<int> next = dp;
        for (int mask = 0; mask < mask_limit; ++mask) {
            if (dp[mask] < 0) continue;
            for (int t = 0; t < target_count; ++t) {
                if (mask & (1 << t)) continue;
                int dist = single_box_push_distance(grid, player, boxes[b], targets[t]);
                if (dist < min_pair_pushes) continue;
                int next_mask = mask | (1 << t);
                next[next_mask] = std::max(next[next_mask], dp[mask] + 1);
            }
        }
        dp.swap(next);
    }

    int best = 0;
    for (int value : dp) best = std::max(best, value);
    return best;
}

PairReachStats compute_pair_reach_stats(const Grid& grid,
                                        Cell player,
                                        const std::vector<Cell>& boxes,
                                        const std::vector<Cell>& targets,
                                        int min_pair_pushes) {
    PairReachStats stats;
    for (Cell box : boxes) {
        for (Cell target : targets) {
            int dist = single_box_push_distance(grid, player, box, target);
            if (dist < min_pair_pushes) {
                ++stats.bad_pairs;
                continue;
            }
            ++stats.good_pairs;
            stats.min_push = std::min(stats.min_push, dist);
            stats.max_push = std::max(stats.max_push, dist);
            stats.sum_push += dist;
        }
    }
    if (stats.good_pairs == 0) {
        stats.min_push = 0;
        stats.max_push = 0;
        stats.avg_push = 0.0;
    } else {
        stats.avg_push = static_cast<double>(stats.sum_push) / stats.good_pairs;
    }
    return stats;
}

void apply_pair_stats(GeneratedMap& map, const PairReachStats& stats) {
    map.min_pair_pushes = stats.min_push;
    map.max_pair_pushes = stats.max_push;
    map.avg_pair_pushes = stats.avg_push;
}

int count_pair_improvements(const PairReachStats& before, const PairReachStats& after) {
    int repaired = before.bad_pairs - after.bad_pairs;
    int added_good = after.good_pairs - before.good_pairs;
    return std::max(repaired, added_good);
}

int score_gateway_opening(const Grid& before, const Grid& after, Cell player, const std::vector<Cell>& bombs) {
    int before_reach = reachable_floor_count(before, player);
    int after_reach = reachable_floor_count(after, player);
    int score = std::max(0, after_reach - before_reach) * 12;

    for (Cell bomb : bombs) {
        if (!is_floor(after, bomb.x, bomb.y)) continue;
        int before_bomb_reach = reachable_floor_count(before, bomb);
        int after_bomb_reach = reachable_floor_count(after, bomb);
        if (after_bomb_reach > before_bomb_reach + 4) {
            score += std::min(after_bomb_reach - before_bomb_reach, 30) * 18;
        }
    }
    return score;
}

double score_map(const GeneratedMap& map) {
    double score = 0.0;
    score += map.avg_pair_pushes * 18.0;
    score += map.max_pair_pushes * 4.0;
    score += map.min_pair_pushes * 8.0;
    score += map.wall_count * 0.8;
    score += map.dead_end_count * 14.0;
    score += map.corridor_count * 10.0;
    score += map.pocket_count * 8.0;
    score -= map.wall_block_count * 30.0;
    score += map.cul_de_sac_count * 9.0;
    score += map.thin_wall_count * 10.0;
    score += map.wall_component_count * 18.0;
    score -= map.thick_wall_count * 22.0;
    score -= std::max(0, map.largest_wall_component - 10) * 24.0;
    score += map.entity_mix_score * 2.0;

    int min_box_target = std::numeric_limits<int>::max();
    for (Cell box : map.boxes) {
        for (Cell target : map.targets) {
            min_box_target = std::min(min_box_target, manhattan(box, target));
        }
    }
    if (min_box_target != std::numeric_limits<int>::max()) score += min_box_target * 2.0;
    score += map.bomb_score;
    score += map.bomb_required_pairs * 95.0;
    score += map.bomb_improved_pairs * 60.0;
    score += map.gateway_score * 0.35;
    return score;
}

int count_bits4(int mask) {
    int count = 0;
    for (int i = 0; i < 4; ++i) {
        if (mask & (1 << i)) ++count;
    }
    return count;
}

int quadrant_mask(Cell c) {
    int q = (c.x >= MAP_MAX_WIDTH / 2 ? 1 : 0) +
        (c.y >= MAP_MAX_HEIGHT / 2 ? 2 : 0);
    return 1 << q;
}

void cell_bounds(const std::vector<Cell>& cells, int& min_x, int& max_x, int& min_y, int& max_y) {
    min_x = MAP_MAX_WIDTH;
    max_x = -1;
    min_y = MAP_MAX_HEIGHT;
    max_y = -1;
    for (Cell c : cells) {
        min_x = std::min(min_x, c.x);
        max_x = std::max(max_x, c.x);
        min_y = std::min(min_y, c.y);
        max_y = std::max(max_y, c.y);
    }
}

int axis_overlap_or_gap_score(int a_min, int a_max, int b_min, int b_max) {
    int overlap = std::min(a_max, b_max) - std::max(a_min, b_min) + 1;
    if (overlap > 0) return overlap * 14;
    int gap = std::max(a_min, b_min) - std::min(a_max, b_max) - 1;
    return -gap * 18;
}

int score_type_transitions(const std::vector<Cell>& boxes, const std::vector<Cell>& targets, bool by_x) {
    struct TaggedCell {
        Cell c;
        int type = 0;
    };
    std::vector<TaggedCell> cells;
    for (Cell c : boxes) cells.push_back({c, 0});
    for (Cell c : targets) cells.push_back({c, 1});
    std::sort(cells.begin(), cells.end(), [by_x](const TaggedCell& a, const TaggedCell& b) {
        int av = by_x ? a.c.x : a.c.y;
        int bv = by_x ? b.c.x : b.c.y;
        if (av != bv) return av < bv;
        int ao = by_x ? a.c.y : a.c.x;
        int bo = by_x ? b.c.y : b.c.x;
        return ao < bo;
    });

    int transitions = 0;
    for (int i = 1; i < static_cast<int>(cells.size()); ++i) {
        if (cells[i].type != cells[i - 1].type) ++transitions;
    }
    return transitions * 12;
}

int score_entity_distribution(const Grid& grid, const std::vector<Cell>& boxes, const std::vector<Cell>& targets) {
    if (boxes.empty() || targets.empty()) return 0;

    int score = 0;
    int box_mask = 0;
    int target_mask = 0;
    double box_x = 0.0;
    double box_y = 0.0;
    double target_x = 0.0;
    double target_y = 0.0;

    for (Cell box : boxes) {
        box_mask |= quadrant_mask(box);
        box_x += box.x;
        box_y += box.y;
        int nearest = 999;
        for (Cell target : targets) nearest = std::min(nearest, manhattan(box, target));
        if (nearest <= 1) score -= 60;
        else if (nearest <= 4) score += 34;
        else if (nearest <= 8) score += 22;
        else score -= nearest * 2;
        if (count_floor_neighbors(grid, box) <= 2) score += 10;
    }
    for (Cell target : targets) {
        target_mask |= quadrant_mask(target);
        target_x += target.x;
        target_y += target.y;
        int nearest = 999;
        for (Cell box : boxes) nearest = std::min(nearest, manhattan(target, box));
        if (nearest <= 1) score -= 45;
        else if (nearest <= 4) score += 24;
        else if (nearest <= 8) score += 16;
        else score -= nearest;
        if (count_floor_neighbors(grid, target) <= 2) score += 8;
    }

    box_x /= boxes.size();
    box_y /= boxes.size();
    target_x /= targets.size();
    target_y /= targets.size();
    double centroid_gap = std::abs(box_x - target_x) + std::abs(box_y - target_y);
    if (centroid_gap <= 5.0) score += 80;
    else if (centroid_gap <= 8.0) score += 30;
    else score -= static_cast<int>((centroid_gap - 8.0) * 18.0);

    int bx0, bx1, by0, by1;
    int tx0, tx1, ty0, ty1;
    cell_bounds(boxes, bx0, bx1, by0, by1);
    cell_bounds(targets, tx0, tx1, ty0, ty1);
    score += axis_overlap_or_gap_score(bx0, bx1, tx0, tx1);
    score += axis_overlap_or_gap_score(by0, by1, ty0, ty1);

    score += count_bits4(box_mask | target_mask) * 18;
    score += count_bits4(box_mask & target_mask) * 45;
    score += score_type_transitions(boxes, targets, true);
    score += score_type_transitions(boxes, targets, false);

    int min_pair = 999;
    int max_pair = 0;
    for (Cell box : boxes) {
        for (Cell target : targets) {
            int d = manhattan(box, target);
            min_pair = std::min(min_pair, d);
            max_pair = std::max(max_pair, d);
        }
    }
    score += std::min(max_pair - min_pair, 12) * 8;
    return score;
}

void fill_map_stats(GeneratedMap& map) {
    map.wall_count = count_walls(map.grid);
    map.dead_end_count = count_dead_ends(map.grid);
    map.corridor_count = count_corridor_cells(map.grid);
    map.pocket_count = count_corner_pockets(map.grid);
    map.wall_block_count = count_wall_blocks_2x2(map.grid);
    map.cul_de_sac_count = count_cul_de_sac_cells(map.grid);
    count_wall_components(map.grid, map.wall_component_count, map.largest_wall_component);
    map.thin_wall_count = count_thin_wall_cells(map.grid);
    map.thick_wall_count = count_thick_wall_cells(map.grid);
    map.entity_mix_score = score_entity_distribution(map.grid, map.boxes, map.targets);
}

bool wall_layout_quality_ok(const GeneratedMap& map, const GeneratorConfig& cfg) {
    bool hard = cfg.difficulty == DifficultyMode::HARD;
    if (cfg.style == GeneratorStyle::OFFICIAL) {
        if (map.wall_block_count > 0) return false;
        if (map.thick_wall_count > (hard ? 8 : 2)) return false;
        if (map.largest_wall_component > (hard ? 14 : 8)) return false;
        return map.wall_component_count >= 3;
    }
    if (map.largest_wall_component > (hard ? 34 : 18)) return false;
    if (map.thick_wall_count > (hard ? 30 : 18)) return false;
    if (map.wall_block_count > (hard ? 18 : 10)) return false;
    if (map.wall_component_count < (hard ? 2 : 3)) return false;
    return true;
}

bool occupied_by_map_entity(const GeneratedMap& map, Cell c) {
    return same_cell(map.player, c) ||
        contains_cell(map.boxes, c) ||
        contains_cell(map.targets, c) ||
        contains_cell(map.bombs, c);
}

bool near_wall_frontier(const Grid& grid, Cell c) {
    for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
            if (std::abs(dx) + std::abs(dy) > 2) continue;
            int nx = c.x + dx;
            int ny = c.y + dy;
            if (is_inner_wall(grid, nx, ny)) return true;
        }
    }
    return false;
}

struct BlastWallCandidate {
    Cell wall;
    int score = 0;
};

struct BombPlacementCandidate {
    Cell bomb;
    Cell wall;
    int push_dist = 0;
    int score = 0;
};

std::vector<BlastWallCandidate> collect_blast_wall_candidates(const Grid& grid) {
    std::vector<BlastWallCandidate> candidates;
    for (int y = 1; y < MAP_MAX_HEIGHT - 1; ++y) {
        for (int x = 1; x < MAP_MAX_WIDTH - 1; ++x) {
            int score = score_blast_wall(grid, {x, y});
            if (score >= 35) candidates.push_back({{x, y}, score});
        }
    }
    std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
        return a.score > b.score;
    });
    if (candidates.size() > 36) candidates.resize(36);
    return candidates;
}

bool place_bombs(GeneratedMap& map, std::mt19937& rng, const GeneratorConfig& cfg) {
    if (cfg.mode == GeneratorMode::NO_BOMB) return true;

    const int target_bombs = std::min(cfg.bombs, MAX_BOMBS);
    if (target_bombs <= 0) return false;

    std::vector<BlastWallCandidate> walls = collect_blast_wall_candidates(map.grid);
    if (walls.empty()) return false;

    std::vector<Cell> floor_candidates;
    for (Cell c : collect_floor_cells(map.grid)) {
        if (c.x <= 0 || c.x >= MAP_MAX_WIDTH - 1 || c.y <= 0 || c.y >= MAP_MAX_HEIGHT - 1) continue;
        if (is_reserved_start_cell(c)) continue;
        if (occupied_by_map_entity(map, c)) continue;
        if (count_floor_neighbors(map.grid, c) < 2) continue;
        if (!has_two_sided_push_axis(map.grid, c)) continue;
        if (!near_wall_frontier(map.grid, c)) continue;
        floor_candidates.push_back(c);
    }
    if (floor_candidates.empty()) return false;

    std::uniform_int_distribution<int> jitter_dist(0, 31);
    double total_bomb_score = 0.0;
    std::vector<Cell> selected_walls;
    Grid cumulative_grid = map.grid;
    PairReachStats current_pair_stats = compute_pair_reach_stats(
        cumulative_grid,
        map.player,
        map.boxes,
        map.targets,
        cfg.min_pair_pushes
    );

    for (int b = 0; b < target_bombs; ++b) {
        std::vector<BombPlacementCandidate> placements;
        for (Cell bomb : floor_candidates) {
            if (occupied_by_map_entity(map, bomb)) continue;
            if (!spacing_ok(map.bombs, bomb, 2)) continue;

            for (const BlastWallCandidate& wall_candidate : walls) {
                if (!spacing_ok(selected_walls, wall_candidate.wall, 3)) continue;
                const Grid& route_grid =
                    cfg.difficulty == DifficultyMode::HARD ? cumulative_grid : map.grid;
                int push_dist = bomb_push_wall_distance(route_grid, map.player, bomb, wall_candidate.wall);
                if (push_dist < 1 || push_dist > 48) continue;

                int entity_pressure = 0;
                for (Cell box : map.boxes) {
                    if (manhattan(box, wall_candidate.wall) <= 3) entity_pressure += 16;
                }
                for (Cell target : map.targets) {
                    if (manhattan(target, wall_candidate.wall) <= 3) entity_pressure += 12;
                }

                int hard_bonus = 0;
                if (cfg.difficulty == DifficultyMode::HARD) {
                    // 铺路炸弹允许先扩展弱连通区，不要求每一炸都立刻修复 pair
                    Grid probe = cumulative_grid;
                    apply_blast(probe, wall_candidate.wall);
                    PairReachStats after_stats = compute_pair_reach_stats(
                        probe,
                        map.player,
                        map.boxes,
                        map.targets,
                        cfg.min_pair_pushes
                    );
                    int improved_pairs = count_pair_improvements(current_pair_stats, after_stats);
                    std::vector<Cell> probe_bombs = map.bombs;
                    probe_bombs.push_back(bomb);
                    int gateway = score_gateway_opening(cumulative_grid, probe, map.player, probe_bombs);
                    if (improved_pairs <= 0 && gateway < 24) continue;
                    hard_bonus =
                        improved_pairs * 900 +
                        std::max(0, current_pair_stats.bad_pairs - after_stats.bad_pairs) * 650 +
                        gateway * 2;
                }

                int score = wall_candidate.score + entity_pressure
                    + count_floor_neighbors(map.grid, bomb) * 10
                    - push_dist * 5
                    - manhattan(bomb, wall_candidate.wall) * 2
                    + hard_bonus
                    + jitter_dist(rng);
                placements.push_back({bomb, wall_candidate.wall, push_dist, score});
            }
        }

        if (placements.empty()) break;
        std::sort(placements.begin(), placements.end(), [](const auto& a, const auto& b) {
            return a.score > b.score;
        });

        // 从高分候选中随机取一个，避免生成器完全贴着当前策略的偏好
        int pool_size = std::min<int>(8, placements.size());
        std::uniform_int_distribution<int> pick_dist(0, pool_size - 1);
        BombPlacementCandidate picked = placements[pick_dist(rng)];
        map.bombs.push_back(picked.bomb);
        map.bomb_targets.push_back(picked.wall);
        selected_walls.push_back(picked.wall);
        total_bomb_score += picked.score;
        if (cfg.difficulty == DifficultyMode::HARD) {
            apply_blast(cumulative_grid, picked.wall);
            current_pair_stats = compute_pair_reach_stats(
                cumulative_grid,
                map.player,
                map.boxes,
                map.targets,
                cfg.min_pair_pushes
            );
        }
    }

    if (static_cast<int>(map.bombs.size()) != target_bombs) return false;
    map.bomb_score = total_bomb_score / std::max(1, target_bombs);

    if (cfg.difficulty == DifficultyMode::HARD) {
        PairReachStats before_stats = compute_pair_reach_stats(
            map.grid,
            map.player,
            map.boxes,
            map.targets,
            cfg.min_pair_pushes
        );
        Grid after_grid = blasted_grid(map);
        PairReachStats after_stats = compute_pair_reach_stats(
            after_grid,
            map.player,
            map.boxes,
            map.targets,
            cfg.min_pair_pushes
        );
        map.bomb_required_pairs = before_stats.bad_pairs;
        map.bomb_improved_pairs = count_pair_improvements(before_stats, after_stats);
        map.bomb_after_bad_pairs = after_stats.bad_pairs;
        map.gateway_score = score_gateway_opening(map.grid, after_grid, map.player, map.bombs);
        apply_pair_stats(map, after_stats);
        int matched_after = max_bipartite_matching_size(
            after_grid,
            map.player,
            map.boxes,
            map.targets,
            cfg.min_pair_pushes
        );
        map.bomb_after_matching = matched_after;

        // Default hard mode restores all pairs; phase2-specific mode keeps only a solvable semantic matching.
        if (before_stats.bad_pairs < cfg.min_bomb_required_pairs) return false;
        if (matched_after < static_cast<int>(map.boxes.size())) return false;
        if (cfg.require_phase2_specific_bomb) {
            if (after_stats.bad_pairs <= 0) return false;
        } else if (after_stats.bad_pairs > 0) {
            return false;
        }
        if (map.bomb_improved_pairs < cfg.min_bomb_required_pairs) return false;
    }
    return true;
}

bool sample_entities(const Grid& grid,
                    std::mt19937& rng,
                    const GeneratorConfig& cfg,
                    GeneratedMap& out) {
    std::vector<Cell> target_candidates;
    std::vector<Cell> box_candidates;
    for (Cell c : collect_floor_cells(grid)) {
        if (c.x <= 0 || c.x >= MAP_MAX_WIDTH - 1 || c.y <= 0 || c.y >= MAP_MAX_HEIGHT - 1) continue;
        if (is_reserved_start_cell(c)) continue;
        if (!has_two_sided_push_axis(grid, c)) continue;
        target_candidates.push_back(c);
        if (cfg.difficulty != DifficultyMode::HARD || !is_near_immutable_border(c)) {
            box_candidates.push_back(c);
        }
    }
    if (static_cast<int>(target_candidates.size()) < cfg.boxes + 1) return false;
    if (static_cast<int>(box_candidates.size()) < cfg.boxes + 1) return false;

    std::shuffle(target_candidates.begin(), target_candidates.end(), rng);
    std::shuffle(box_candidates.begin(), box_candidates.end(), rng);
    std::uniform_int_distribution<int> target_pick_dist(0, static_cast<int>(target_candidates.size()) - 1);
    std::uniform_int_distribution<int> box_pick_dist(0, static_cast<int>(box_candidates.size()) - 1);

    GeneratedMap best;
    int valid_count = 0;
    const int entity_spacing = cfg.style == GeneratorStyle::OFFICIAL
        ? 1 : cfg.min_entity_spacing;

    auto pick_entity_set = [&](std::vector<Cell>& dst,
                               const std::vector<Cell>& blocked,
                               const std::vector<Cell>& source,
                               std::uniform_int_distribution<int>& pick_dist) {
        dst.clear();
        for (int guard = 0; static_cast<int>(dst.size()) < cfg.boxes && guard < 1200; ++guard) {
            Cell picked = source[pick_dist(rng)];
            if (contains_cell(blocked, picked) || contains_cell(dst, picked)) continue;
            if (!spacing_ok(dst, picked, entity_spacing)) continue;
            dst.push_back(picked);
        }
        return static_cast<int>(dst.size()) == cfg.boxes;
    };

    for (int trial = 0; trial < 1600; ++trial) {
        GeneratedMap candidate;
        candidate.grid = grid;

        const std::vector<Cell> empty_blocked;
        if (!pick_entity_set(candidate.targets, empty_blocked, target_candidates, target_pick_dist)) continue;
        if (!pick_entity_set(candidate.boxes, candidate.targets, box_candidates, box_pick_dist)) continue;

        candidate.player = fixed_player_start_file_cell();
        if (!is_floor(grid, candidate.player.x, candidate.player.y)) continue;
        if (contains_cell(candidate.boxes, candidate.player) || contains_cell(candidate.targets, candidate.player)) continue;

        if (cfg.difficulty == DifficultyMode::HARD && cfg.mode == GeneratorMode::WITH_BOMB) {
            PairReachStats stats = compute_pair_reach_stats(
                grid,
                candidate.player,
                candidate.boxes,
                candidate.targets,
                cfg.min_pair_pushes
            );
            if (stats.bad_pairs < cfg.min_bomb_required_pairs) continue;
            if (stats.good_pairs == 0) continue;
            apply_pair_stats(candidate, stats);
        } else {
            if (!evaluate_pair_reachability(grid, candidate.player, candidate.boxes, candidate.targets, cfg, candidate)) {
                continue;
            }
        }

        fill_map_stats(candidate);
        if (!wall_layout_quality_ok(candidate, cfg)) continue;
        candidate.score = score_map(candidate);
        ++valid_count;
        if (candidate.score > best.score) best = candidate;
        if (valid_count >= 18) {
            out = best;
            return true;
        }
    }

    if (best.score >= 0.0) {
        out = best;
        return true;
    }
    return false;
}

bool generate_one_map(std::mt19937& rng, const GeneratorConfig& cfg, GeneratedMap& out) {
    GeneratedMap best;
    int valid_count = 0;

    for (int attempt = 0; attempt < cfg.max_attempts; ++attempt) {
        Grid grid = make_random_grid(rng, cfg);
        if (!grid_wall_layout_quality_ok(grid, cfg)) continue;
        GeneratedMap candidate;
        if (!sample_entities(grid, rng, cfg, candidate)) continue;
        if (!place_bombs(candidate, rng, cfg)) continue;
        candidate.score = score_map(candidate);

        ++valid_count;
        if (candidate.score > best.score) best = candidate;
        if (valid_count >= 3 && attempt >= cfg.max_attempts / 12) {
            out = best;
            return true;
        }
        if (valid_count >= cfg.quality_candidates) {
            out = best;
            return true;
        }
    }

    if (best.score >= 0.0) {
        out = best;
        return true;
    }
    return false;
}

std::vector<std::string> render_map(const GeneratedMap& map) {
    std::vector<std::string> rows(MAP_MAX_HEIGHT, std::string(MAP_MAX_WIDTH, '-'));
    for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
            rows[y][x] = map.grid.wall[y][x] ? '#' : '-';
        }
    }

    for (Cell bomb : map.bombs) rows[bomb.y][bomb.x] = '*';
    for (Cell target : map.targets) rows[target.y][target.x] = '.';
    for (Cell box : map.boxes) rows[box.y][box.x] = '$';
    return rows;
}

bool write_map_file(const GeneratedMap& map, const std::filesystem::path& path) {
    std::ofstream out(path);
    if (!out.is_open()) return false;

    std::vector<std::string> rows = render_map(map);
    for (const std::string& row : rows) {
        out << row << "\n";
    }
    return true;
}

void write_cell_line(std::ofstream& out, const char* label, int index, Cell c) {
    out << label << " " << index
        << " file=" << c.x << "," << c.y
        << " logical=" << c.x << "," << (MAP_MAX_HEIGHT - 1 - c.y)
        << "\n";
}

bool write_meta_file(const GeneratedMap& map, const std::filesystem::path& path) {
    std::ofstream out(path);
    if (!out.is_open()) return false;

    out << "score " << static_cast<int>(map.score) << "\n";
    out << "min_pair_push " << map.min_pair_pushes << "\n";
    out << "avg_pair_push " << map.avg_pair_pushes << "\n";
    out << "max_pair_push " << map.max_pair_pushes << "\n";
    out << "bomb_required_pairs " << map.bomb_required_pairs << "\n";
    out << "bomb_improved_pairs " << map.bomb_improved_pairs << "\n";
    out << "bomb_after_bad_pairs " << map.bomb_after_bad_pairs << "\n";
    out << "bomb_after_matching " << map.bomb_after_matching << "\n";
    out << "gateway_score " << map.gateway_score << "\n";
    out << "coordinate_note file=x,row logical=x," << (MAP_MAX_HEIGHT - 1) << "-row\n";
    write_cell_line(out, "player", 0, map.player);
    for (int i = 0; i < static_cast<int>(map.boxes.size()); ++i) {
        write_cell_line(out, "box", i, map.boxes[i]);
    }
    for (int i = 0; i < static_cast<int>(map.targets.size()); ++i) {
        write_cell_line(out, "target", i, map.targets[i]);
    }
    for (int i = 0; i < static_cast<int>(map.bombs.size()); ++i) {
        write_cell_line(out, "bomb", i, map.bombs[i]);
    }
    for (int i = 0; i < static_cast<int>(map.bomb_targets.size()); ++i) {
        write_cell_line(out, "expected_blast_wall", i, map.bomb_targets[i]);
    }
    return true;
}

std::string format_index(int index) {
    std::ostringstream ss;
    ss.width(3);
    ss.fill('0');
    ss << index;
    return ss.str();
}

void print_usage(const char* exe) {
    std::cout
        << "Usage: " << exe << " [options]\n"
        << "  --count N                 number of maps, default 10\n"
        << "  --boxes N                 box and target count, default 3\n"
        << "  --bombs N                 bomb count in with-bomb mode, default 3\n"
        << "  --out-dir PATH            output directory, default map/map_generated\n"
        << "  --prefix NAME             output file prefix, default gen\n"
        << "  --seed N                  random seed, default current time\n"
        << "  --wall-density F          internal wall density, default 0.14\n"
        << "  --min-pair-pushes N       every box-target pair minimum pushes, default 4\n"
        << "  --min-bomb-required-pairs N hard mode requires at least this many initially blocked pairs, default 1\n"
        << "  --require-phase2-specific-bomb hard bomb maps keep partial pair reachability, requiring semantic-specific phase2 choice\n"
        << "  --quality-candidates N    keep best map from this many valid candidates, default 56\n"
        << "  --max-attempts N          max attempts per map, default 50000\n"
        << "  --mode no-bomb|with-bomb  generation mode, default no-bomb\n"
        << "  --difficulty normal|hard  hard mode generates bomb-required maps\n"
        << "  --style official|legacy   wall layout style, default official\n"
        << "  --write-meta              write sidecar diagnostics for generated maps\n";
}

bool parse_int_arg(const char* text, int& out) {
    char* end = nullptr;
    long value = std::strtol(text, &end, 10);
    if (end == text || *end != '\0') return false;
    out = static_cast<int>(value);
    return true;
}

bool parse_uint_arg(const char* text, uint32_t& out) {
    char* end = nullptr;
    unsigned long value = std::strtoul(text, &end, 10);
    if (end == text || *end != '\0') return false;
    out = static_cast<uint32_t>(value);
    return true;
}

bool parse_double_arg(const char* text, double& out) {
    char* end = nullptr;
    double value = std::strtod(text, &end);
    if (end == text || *end != '\0') return false;
    out = value;
    return true;
}

bool parse_args(int argc, char** argv, GeneratorConfig& cfg) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto need_value = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << "\n";
                return nullptr;
            }
            return argv[++i];
        };

        if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            std::exit(0);
        } else if (arg == "--count") {
            const char* value = need_value("--count");
            if (!value || !parse_int_arg(value, cfg.count)) return false;
        } else if (arg == "--boxes") {
            const char* value = need_value("--boxes");
            if (!value || !parse_int_arg(value, cfg.boxes)) return false;
        } else if (arg == "--bombs") {
            const char* value = need_value("--bombs");
            if (!value || !parse_int_arg(value, cfg.bombs)) return false;
        } else if (arg == "--out-dir") {
            const char* value = need_value("--out-dir");
            if (!value) return false;
            cfg.out_dir = value;
        } else if (arg == "--prefix") {
            const char* value = need_value("--prefix");
            if (!value) return false;
            cfg.prefix = value;
        } else if (arg == "--seed") {
            const char* value = need_value("--seed");
            if (!value || !parse_uint_arg(value, cfg.seed)) return false;
        } else if (arg == "--wall-density") {
            const char* value = need_value("--wall-density");
            if (!value || !parse_double_arg(value, cfg.wall_density)) return false;
        } else if (arg == "--min-pair-pushes") {
            const char* value = need_value("--min-pair-pushes");
            if (!value || !parse_int_arg(value, cfg.min_pair_pushes)) return false;
        } else if (arg == "--min-bomb-required-pairs") {
            const char* value = need_value("--min-bomb-required-pairs");
            if (!value || !parse_int_arg(value, cfg.min_bomb_required_pairs)) return false;
        } else if (arg == "--quality-candidates") {
            const char* value = need_value("--quality-candidates");
            if (!value || !parse_int_arg(value, cfg.quality_candidates)) return false;
        } else if (arg == "--max-attempts") {
            const char* value = need_value("--max-attempts");
            if (!value || !parse_int_arg(value, cfg.max_attempts)) return false;
        } else if (arg == "--mode") {
            const char* value = need_value("--mode");
            if (!value) return false;
            std::string mode = value;
            if (mode == "no-bomb") {
                cfg.mode = GeneratorMode::NO_BOMB;
            } else if (mode == "with-bomb" || mode == "bomb") {
                cfg.mode = GeneratorMode::WITH_BOMB;
            } else {
                std::cerr << "--mode must be no-bomb or with-bomb\n";
                return false;
            }
        } else if (arg == "--difficulty") {
            const char* value = need_value("--difficulty");
            if (!value) return false;
            std::string difficulty = value;
            if (difficulty == "normal") {
                cfg.difficulty = DifficultyMode::NORMAL;
            } else if (difficulty == "hard") {
                cfg.difficulty = DifficultyMode::HARD;
            } else {
                std::cerr << "--difficulty must be normal or hard\n";
                return false;
            }
        } else if (arg == "--style") {
            const char* value = need_value("--style");
            if (!value) return false;
            std::string style = value;
            if (style == "official" || style == "game") {
                cfg.style = GeneratorStyle::OFFICIAL;
            } else if (style == "legacy" || style == "random") {
                cfg.style = GeneratorStyle::LEGACY;
            } else {
                std::cerr << "--style must be official or legacy\n";
                return false;
            }
        } else if (arg == "--write-meta") {
            cfg.write_meta = true;
        } else if (arg == "--require-phase2-specific-bomb") {
            cfg.require_phase2_specific_bomb = true;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            return false;
        }
    }

    if (cfg.count <= 0) {
        std::cerr << "--count must be positive\n";
        return false;
    }
    if (cfg.boxes <= 0 || cfg.boxes > MAX_BOXES) {
        std::cerr << "--boxes must be in [1, " << MAX_BOXES << "]\n";
        return false;
    }
    if (cfg.mode == GeneratorMode::NO_BOMB) {
        cfg.bombs = 0;
        cfg.difficulty = DifficultyMode::NORMAL;
    } else if (cfg.bombs == 0) {
        cfg.bombs = std::min(3, MAX_BOMBS);
    }
    if (cfg.require_phase2_specific_bomb &&
        (cfg.mode != GeneratorMode::WITH_BOMB || cfg.difficulty != DifficultyMode::HARD)) {
        std::cerr << "--require-phase2-specific-bomb requires --mode with-bomb --difficulty hard\n";
        return false;
    }
    if (cfg.bombs < 0 || cfg.bombs > MAX_BOMBS) {
        std::cerr << "--bombs must be in [0, " << MAX_BOMBS << "]\n";
        return false;
    }
    if (cfg.min_pair_pushes < 1) {
        std::cerr << "--min-pair-pushes must be positive\n";
        return false;
    }
    if (cfg.min_bomb_required_pairs < 1) {
        std::cerr << "--min-bomb-required-pairs must be positive\n";
        return false;
    }
    if (cfg.quality_candidates <= 0 || cfg.max_attempts <= 0) {
        std::cerr << "--quality-candidates and --max-attempts must be positive\n";
        return false;
    }
    if (cfg.wall_density < 0.0 || cfg.wall_density > 0.55) {
        std::cerr << "--wall-density must be in [0, 0.55]\n";
        return false;
    }
    double max_wall_density = 0.0;
    if (cfg.style == GeneratorStyle::OFFICIAL) {
        max_wall_density = cfg.difficulty == DifficultyMode::HARD
            ? OFFICIAL_HARD_MAX_WALL_DENSITY : OFFICIAL_MAX_WALL_DENSITY;
    } else {
        max_wall_density = cfg.difficulty == DifficultyMode::HARD
            ? HARD_MAX_WALL_DENSITY : MAX_DISPERSED_WALL_DENSITY;
    }
    if (cfg.wall_density > max_wall_density) {
        std::cerr << "Notice: selected wall style clamps --wall-density from "
                << cfg.wall_density << " to " << max_wall_density
                << "\n";
        cfg.wall_density = max_wall_density;
    }
    int max_min_pair_pushes =
        cfg.difficulty == DifficultyMode::HARD ? HARD_MAX_MIN_PAIR_PUSHES : MAX_DISPERSED_MIN_PAIR_PUSHES;
    if (cfg.min_pair_pushes > max_min_pair_pushes) {
        std::cerr << "Notice: selected wall style caps --min-pair-pushes from "
                << cfg.min_pair_pushes << " to " << max_min_pair_pushes
                << "\n";
        cfg.min_pair_pushes = max_min_pair_pushes;
    }
    int max_quality_candidates =
        cfg.difficulty == DifficultyMode::HARD ? HARD_MAX_EFFECTIVE_QUALITY_CANDIDATES : MAX_EFFECTIVE_QUALITY_CANDIDATES;
    if (cfg.quality_candidates > max_quality_candidates) {
        std::cerr << "Notice: selected wall style caps --quality-candidates from "
                << cfg.quality_candidates << " to " << max_quality_candidates
                << "\n";
        cfg.quality_candidates = max_quality_candidates;
    }
    return true;
}

} // namespace

int main(int argc, char** argv) {
    GeneratorConfig cfg;
    cfg.seed = static_cast<uint32_t>(
        std::chrono::high_resolution_clock::now().time_since_epoch().count()
    );

    if (!parse_args(argc, argv, cfg)) {
        print_usage(argv[0]);
        return 1;
    }

    std::filesystem::create_directories(cfg.out_dir);

    std::mt19937 rng(cfg.seed);
    const char* mode_name = cfg.mode == GeneratorMode::WITH_BOMB ? "with-bomb" : "no-bomb";
    const char* style_name = cfg.style == GeneratorStyle::OFFICIAL ? "official" : "legacy";
    std::cout << "MapGenerator seed=" << cfg.seed
            << " count=" << cfg.count
            << " boxes=" << cfg.boxes
            << " bombs=" << cfg.bombs
            << " mode=" << mode_name
            << " style=" << style_name
            << " phase2_specific_bomb=" << (cfg.require_phase2_specific_bomb ? "yes" : "no")
            << "\n";

    for (int i = 1; i <= cfg.count; ++i) {
        GeneratedMap map;
        if (!generate_one_map(rng, cfg, map)) {
            std::cerr << "Failed to generate map " << i
                    << " after " << cfg.max_attempts << " attempts\n";
            return 3;
        }

        std::filesystem::path path = std::filesystem::path(cfg.out_dir) /
            (cfg.prefix + "_" + format_index(i) + ".txt");
        if (!write_map_file(map, path)) {
            std::cerr << "Failed to write " << path.string() << "\n";
            return 4;
        }
        if (cfg.write_meta) {
            std::filesystem::path meta_path = path;
            meta_path.replace_extension(".meta");
            if (!write_meta_file(map, meta_path)) {
                std::cerr << "Failed to write " << meta_path.string() << "\n";
                return 4;
            }
        }

        std::cout << path.string()
                << " score=" << static_cast<int>(map.score)
                << " min_pair_push=" << map.min_pair_pushes
                << " avg_pair_push=" << map.avg_pair_pushes
                << " max_pair_push=" << map.max_pair_pushes
                << " bombs=" << map.bombs.size()
                << " walls=" << map.wall_count
                << " dead_ends=" << map.dead_end_count
                << " corridors=" << map.corridor_count
                << " pockets=" << map.pocket_count
                << " wall_blocks=" << map.wall_block_count
                << " cul_de_sacs=" << map.cul_de_sac_count
                << " wall_components=" << map.wall_component_count
                << " largest_wall=" << map.largest_wall_component
                << " thin_walls=" << map.thin_wall_count
                << " thick_walls=" << map.thick_wall_count
                << " entity_mix=" << map.entity_mix_score
                << " bomb_required_pairs=" << map.bomb_required_pairs
                << " bomb_improved_pairs=" << map.bomb_improved_pairs
                << " bomb_after_bad_pairs=" << map.bomb_after_bad_pairs
                << " bomb_after_matching=" << map.bomb_after_matching
                << " gateway_score=" << map.gateway_score
                << "\n";
    }

    return 0;
}

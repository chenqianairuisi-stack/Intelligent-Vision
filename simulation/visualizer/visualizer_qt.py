import json
import os
import random
import re
import subprocess
import sys
from dataclasses import dataclass, field

from PySide6.QtCore import QPointF, QRectF, QSize, QThread, QTimer, Qt, Signal
from PySide6.QtGui import QColor, QFont, QIcon, QPainter, QPainterPath, QPen, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QButtonGroup,
    QCheckBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpacerItem,
    QVBoxLayout,
    QWidget,
)


CELL_SIZE = 44
MAP_W, MAP_H = 12, 16
PLAN_START_X, PLAN_START_Y = 4, 1
VISUALIZER_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(VISUALIZER_DIR)
SOLVER_PATH = os.path.join(PROJECT_DIR, "solver.exe")
MAP_INPUT_PATH = os.path.join(VISUALIZER_DIR, "map_input.txt")
PATH_OUTPUT_PATH = os.path.join(VISUALIZER_DIR, "path_output.txt")
SETTINGS_PATH = os.path.join(VISUALIZER_DIR, "visualizer_settings.json")
DEFAULT_MAP_DIR = os.path.join(PROJECT_DIR, "map", "map_game")
SOLVER_TIMEOUT_SECONDS = 4.0

MOVE_STEP_COST = 1
STOP_NODE_COST = 2
OBSERVE_EXTRA_COST = 1
TURN_EXTRA_COST = 2
DEFAULT_COST_MODEL = (
    MOVE_STEP_COST,
    STOP_NODE_COST,
    OBSERVE_EXTRA_COST,
    TURN_EXTRA_COST,
)

BOMB_COLORS = ["#e67e22", "#1abc9c", "#e74c3c", "#f39c12", "#8e44ad"]
PLAYBACK_SPEEDS = [1.0, 2.0, 4.0, 0.5]
PLAYBACK_SPEED_LABELS = ["x1", "x2", "x4", "x0.5"]


@dataclass
class BombTask:
    bomb: list
    pushes: list = field(default_factory=list)


def append_patrol_move_events(events, current_pos, target):
    """轴对齐航段逐格回放，任意斜向航段保留为单次移动"""
    dx = target[0] - current_pos[0]
    dy = target[1] - current_pos[1]
    if dx != 0 and dy != 0:
        events.append(["MOVE", target])
        return list(target)

    distance = abs(dx) + abs(dy)
    if distance <= 1:
        events.append(["MOVE", target])
        return list(target)

    step_x = 0 if dx == 0 else (1 if dx > 0 else -1)
    step_y = 0 if dy == 0 else (1 if dy > 0 else -1)
    for step in range(1, distance + 1):
        events.append([
            "MOVE",
            [current_pos[0] + step_x * step, current_pos[1] + step_y * step],
        ])
    return list(target)


class SolverThread(QThread):
    finished_ok = Signal()
    failed = Signal(str)
    timed_out = Signal()

    def __init__(self, solver_path, solver_args, cwd, parent=None, timeout_seconds=SOLVER_TIMEOUT_SECONDS):
        super().__init__(parent)
        self.solver_path = solver_path
        self.solver_args = solver_args
        self.cwd = cwd
        self.timeout_seconds = timeout_seconds
        self.process = None
        self.stop_requested = False

    def kill_process(self):
        process = self.process
        if process is None or process.poll() is not None:
            return
        try:
            process.kill()
            process.wait(timeout=1.0)
        except Exception:
            pass

    def run(self):
        try:
            if self.stop_requested:
                return
            self.process = subprocess.Popen(
                [self.solver_path, *self.solver_args],
                cwd=self.cwd,
            )
            if self.stop_requested:
                self.kill_process()
                return
            try:
                return_code = self.process.wait(timeout=self.timeout_seconds)
            except subprocess.TimeoutExpired:
                self.kill_process()
                if not self.stop_requested:
                    self.timed_out.emit()
                return
        except Exception as exc:
            if not self.stop_requested:
                self.failed.emit(str(exc))
            return
        finally:
            self.process = None

        if self.stop_requested:
            return
        if return_code != 0:
            self.failed.emit(f"求解器异常退出，返回码 {return_code}")
            return
        self.finished_ok.emit()

    def stop(self):
        self.stop_requested = True
        self.kill_process()


class MapCanvas(QWidget):
    def __init__(self, visualizer):
        super().__init__()
        self.visualizer = visualizer
        self.setFixedSize(MAP_W * CELL_SIZE, MAP_H * CELL_SIZE)
        self.setObjectName("mapCanvas")

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        self.visualizer.draw_map(painter)


class SokobanVisualizerQt(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("智能车多阶融合 SIL 仿真平台")
        self.resize(1120, 840)
        self.setMinimumSize(1040, 760)

        self.user_settings = self.load_user_settings()
        self.has_saved_car_start = "car_start" in self.user_settings
        car_start = self.user_settings.get("car_start", [PLAN_START_X, PLAN_START_Y])

        self.map_data = []
        self.player = None
        self.boxes, self.targets, self.bombs = [], [], []
        self.box_ids = []
        self.semantic_defaults = []
        self.output_semantics = []
        self.animation_box_semantics = []
        self.animation_target_semantics = []
        self.box_letter_map = {}
        self.target_letter_map = {}

        self.patrol_events = []
        self.sokoban_path = []
        self.bomb_tasks_1, self.bomb_tasks_2 = [], []
        self.chosen_obs = []
        self.obs_clear_pushes = []
        self.semantic_match_cache = {}
        self.last_semantic_mapping = []
        self.semantic_random_mode = True

        self.sokoban_failed = False
        self.sokoban_invalid_path = False
        self.is_solving = False
        self.is_solver_running = False
        self.current_phase = 1
        self.discovered_entities = set()
        self.build_info = ""
        self.cost_model = DEFAULT_COST_MODEL
        self.anim_step = 0
        self.playback_paused = False
        self.animation_finished = True
        self.playback_speed_index = 0
        self.show_debug_overlays = True
        self.dark_mode = bool(self.user_settings.get("dark_mode", False))
        self.status_kind = "idle"
        self.last_animation_delay = 60
        self.sidebar_fit_done = False

        self.map_files = []
        self.current_map_index = -1
        self.current_map_path = ""
        self.current_map_dir = DEFAULT_MAP_DIR

        self.solver_thread = None
        self.anim_timer = QTimer(self)
        self.anim_timer.setSingleShot(True)
        self.anim_timer.timeout.connect(self.animate_next_step)

        self.car_x_edit = QLineEdit(str(car_start[0]))
        self.car_y_edit = QLineEdit(str(car_start[1]))
        self.box_semantic_edit = QLineEdit()
        self.target_semantic_edit = QLineEdit()

        self.time_value_labels = []
        self.cost_value_labels = {}

        self.setup_ui()
        self.apply_style()

        self.car_x_edit.textChanged.connect(self.on_car_start_changed)
        self.car_y_edit.textChanged.connect(self.on_car_start_changed)

        self.scan_map_directory(DEFAULT_MAP_DIR)
        if self.map_files:
            self.load_map_by_index(0)

    def showEvent(self, event):
        super().showEvent(event)
        if not self.sidebar_fit_done:
            self.sidebar_fit_done = True
            QTimer.singleShot(0, self.resize_to_fit_sidebar)

    def setup_ui(self):
        root = QWidget()
        root.setObjectName("root")
        main_layout = QHBoxLayout(root)
        main_layout.setContentsMargins(18, 18, 18, 18)
        main_layout.setSpacing(18)

        sidebar = QFrame()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(360)
        sidebar_layout = QVBoxLayout(sidebar)
        sidebar_layout.setContentsMargins(10, 10, 10, 10)
        sidebar_layout.setSpacing(8)

        self.map_name_label = QLabel("未加载地图")
        self.map_name_label.setObjectName("mapName")
        sidebar_layout.addWidget(self.build_section("地图", self.build_map_controls(), title_extra=self.map_name_label))
        sidebar_layout.addWidget(self.build_section(None, self.build_semantic_controls()))
        sidebar_layout.addWidget(self.build_section("仿真参数", self.build_simulation_controls()))
        playback_controls = self.build_playback_controls()
        sidebar_layout.addWidget(self.build_section(
            "播放",
            playback_controls,
            title_extra=self.build_playback_title_controls(),
            title_extra_right=True,
        ))

        sidebar_layout.addWidget(self.build_section("规划时间", self.build_time_stats()))
        sidebar_layout.addWidget(self.build_section("路径代价", self.build_cost_stats()))
        sidebar_layout.addItem(QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding))

        workspace = QFrame()
        workspace.setObjectName("workspace")
        workspace_layout = QVBoxLayout(workspace)
        workspace_layout.setContentsMargins(18, 18, 18, 18)
        workspace_layout.setSpacing(0)

        canvas_row = QHBoxLayout()
        canvas_row.addStretch(1)
        self.canvas = MapCanvas(self)
        canvas_row.addWidget(self.canvas)
        canvas_row.addStretch(1)
        workspace_layout.addLayout(canvas_row)

        sidebar_scroll = QScrollArea()
        self.sidebar_scroll = sidebar_scroll
        self.sidebar_scroll.setObjectName("sidebarScroll")
        self.sidebar_scroll.setWidgetResizable(True)
        self.sidebar_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.sidebar_scroll.setFrameShape(QFrame.NoFrame)
        self.sidebar_scroll.setFixedWidth(380)
        self.sidebar_scroll.setWidget(sidebar)

        main_layout.addWidget(self.sidebar_scroll)
        main_layout.addWidget(workspace, 1)
        self.setCentralWidget(root)

    def resize_to_fit_sidebar(self):
        self.ensurePolished()
        self.centralWidget().layout().activate()
        if hasattr(self, "sidebar_scroll"):
            self.sidebar_scroll.widget().layout().activate()
            self.sidebar_scroll.widget().adjustSize()

        sidebar_needed = self.sidebar_scroll.widget().sizeHint().height()
        viewport_height = self.sidebar_scroll.viewport().height()
        extra_height = max(0, sidebar_needed - viewport_height + 12)
        canvas_needed = MAP_H * CELL_SIZE + 72
        target_height = max(self.height() + extra_height, canvas_needed, self.minimumHeight())
        target_height = min(int(target_height), 980)
        self.resize(max(self.width(), 1120), target_height)

    def build_section(self, title, content, extra=None, title_extra=None, title_extra_right=False):
        frame = QFrame()
        frame.setObjectName("section")
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(12, 8, 12, 9)
        layout.setSpacing(7)
        if title is not None:
            title_row = QHBoxLayout()
            title_row.setSpacing(8)
            label = QLabel(title)
            label.setObjectName("sectionTitle")
            title_row.addWidget(label)
            if title_extra_right:
                title_row.addStretch(1)
                if title_extra is not None:
                    title_row.addWidget(title_extra)
            else:
                if title_extra is not None:
                    title_row.addWidget(title_extra)
                title_row.addStretch(1)
            layout.addLayout(title_row)
        if extra is not None:
            layout.addWidget(extra)
        layout.addWidget(content)
        return frame

    def build_map_controls(self):
        row = QHBoxLayout()
        row.setSpacing(8)
        widget = QWidget()
        widget.setLayout(row)

        self.choose_btn = QPushButton()
        self.choose_btn.setObjectName("iconButton")
        self.choose_btn.setIcon(self.make_folder_icon())
        self.choose_btn.setIconSize(QSize(24, 24))
        self.choose_btn.setFixedWidth(52)
        self.prev_btn = QPushButton("上一张")
        self.reload_btn = QPushButton("重载")
        self.next_btn = QPushButton("下一张")
        for btn in [self.choose_btn, self.prev_btn, self.reload_btn, self.next_btn]:
            btn.setMinimumHeight(30)
            row.addWidget(btn)
        self.choose_btn.clicked.connect(self.choose_map)
        self.prev_btn.clicked.connect(self.prev_map)
        self.reload_btn.clicked.connect(self.reload_map)
        self.next_btn.clicked.connect(self.next_map)
        return widget

    def make_folder_icon(self):
        pixmap = QPixmap(28, 24)
        pixmap.fill(Qt.transparent)
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing, True)

        tab = QPainterPath()
        tab.addRoundedRect(QRectF(4, 4, 10, 7), 2, 2)
        painter.fillPath(tab, QColor("#2f80ed"))

        body = QPainterPath()
        body.addRoundedRect(QRectF(3, 8, 22, 13), 3, 3)
        painter.fillPath(body, QColor("#d9ecff"))
        painter.setPen(QPen(QColor("#2f80ed"), 2))
        painter.drawPath(body)
        painter.drawLine(QPointF(5, 9), QPointF(13, 9))
        painter.end()
        return QIcon(pixmap)

    def make_eye_icon(self, crossed=False):
        pixmap = QPixmap(24, 24)
        pixmap.fill(Qt.transparent)
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing, True)
        color = QColor("#f3f7fb" if self.dark_mode else "#314457")
        painter.setPen(QPen(color, 2))
        eye = QPainterPath()
        eye.moveTo(3, 12)
        eye.cubicTo(7, 6, 17, 6, 21, 12)
        eye.cubicTo(17, 18, 7, 18, 3, 12)
        painter.drawPath(eye)
        painter.setBrush(color)
        painter.drawEllipse(QPointF(12, 12), 3, 3)
        if crossed:
            painter.setPen(QPen(QColor("#d14b4b"), 2))
            painter.drawLine(QPointF(5, 20), QPointF(20, 4))
        painter.end()
        return QIcon(pixmap)

    def make_theme_icon(self):
        pixmap = QPixmap(24, 24)
        pixmap.fill(Qt.transparent)
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing, True)
        if self.dark_mode:
            moon = QPainterPath()
            moon.addEllipse(QRectF(5, 5, 14, 14))
            cutout = QPainterPath()
            cutout.addEllipse(QRectF(10, 3, 14, 14))
            painter.setPen(QPen(Qt.NoPen))
            painter.setBrush(QColor("#d8e3f0"))
            painter.fillPath(moon.subtracted(cutout), QColor("#d8e3f0"))
        else:
            painter.setBrush(QColor("#f6d365"))
            painter.setPen(QPen(QColor("#f2c14e"), 2))
            painter.drawEllipse(QPointF(12, 12), 5, 5)
            for dx, dy in [(0, -8), (6, -6), (8, 0), (6, 6), (0, 8), (-6, 6), (-8, 0), (-6, -6)]:
                painter.drawLine(QPointF(12 + dx * 0.65, 12 + dy * 0.65), QPointF(12 + dx, 12 + dy))
        painter.end()
        return QIcon(pixmap)

    def build_semantic_controls(self):
        wrapper = QWidget()
        layout = QVBoxLayout(wrapper)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)

        car_row = QHBoxLayout()
        car_row.setSpacing(6)
        car_label = QLabel("Car start")
        car_label.setObjectName("fieldLabel")
        self.car_x_edit.setFixedWidth(54)
        self.car_y_edit.setFixedWidth(54)
        self.car_x_edit.setMinimumHeight(26)
        self.car_y_edit.setMinimumHeight(26)
        car_row.addWidget(car_label)
        car_row.addStretch(1)
        car_row.addWidget(QLabel("X"))
        car_row.addWidget(self.car_x_edit)
        car_row.addWidget(QLabel("Y"))
        car_row.addWidget(self.car_y_edit)
        layout.addLayout(car_row)

        mode_row = QHBoxLayout()
        mode_row.setSpacing(6)
        mode_row.addWidget(QLabel("语义标签"))
        mode_row.addStretch(1)
        self.semantic_mode_btn = QPushButton(self.semantic_mode_label())
        self.semantic_mode_btn.setFixedWidth(70)
        self.semantic_mode_btn.setMinimumHeight(26)
        self.semantic_mode_btn.clicked.connect(self.toggle_semantic_mode)
        mode_row.addWidget(self.semantic_mode_btn)
        layout.addLayout(mode_row)

        layout.addLayout(self.build_labeled_edit("箱子语义", self.box_semantic_edit))
        layout.addLayout(self.build_labeled_edit("目标语义", self.target_semantic_edit))
        return wrapper

    def build_labeled_edit(self, text, edit):
        row = QHBoxLayout()
        row.setSpacing(6)
        label = QLabel(text)
        label.setObjectName("fieldLabel")
        edit.setMinimumWidth(160)
        edit.setMinimumHeight(26)
        row.addWidget(label)
        row.addStretch(1)
        row.addWidget(edit)
        return row

    def build_simulation_controls(self):
        wrapper = QWidget()
        layout = QVBoxLayout(wrapper)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)

        self.diagonal_move_checkbox = QCheckBox("斜向移动")
        self.box_extra_observe_checkbox = QCheckBox("箱子额外观测位")
        self.target_extra_observe_checkbox = QCheckBox("目标点额外观测位")
        switches = [
            (self.diagonal_move_checkbox, "simulation_diagonal_move"),
            (self.box_extra_observe_checkbox, "simulation_box_extra_observe"),
            (self.target_extra_observe_checkbox, "simulation_target_extra_observe"),
        ]
        for checkbox, setting_key in switches:
            checkbox.setChecked(bool(self.user_settings.get(setting_key, True)))
            checkbox.setToolTip("仅影响仿真求解器进程")
            checkbox.toggled.connect(self.save_simulation_switches)
            layout.addWidget(checkbox)
        return wrapper

    def build_playback_controls(self):
        wrapper = QWidget()
        layout = QVBoxLayout(wrapper)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(7)

        self.solve_btn = QPushButton("全自动解算并演示")
        self.solve_btn.setObjectName("primaryButton")
        layout.addWidget(self.solve_btn)

        bottom = QHBoxLayout()
        bottom.setSpacing(8)
        self.pause_btn = QPushButton("Pause")
        self.step_btn = QPushButton("Step")
        self.pause_btn.setMinimumHeight(32)
        self.step_btn.setMinimumHeight(32)
        bottom.addWidget(self.pause_btn)
        bottom.addWidget(self.step_btn)
        layout.addLayout(bottom)

        self.status_label = QLabel("等待操作")
        self.status_label.setObjectName("statusLabel")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setMinimumHeight(30)
        layout.addWidget(self.status_label)

        self.solve_btn.clicked.connect(self.solve_and_animate)
        self.pause_btn.clicked.connect(self.toggle_playback)
        self.step_btn.clicked.connect(self.step_once)
        return wrapper

    def build_playback_title_controls(self):
        wrapper = QWidget()
        row = QHBoxLayout(wrapper)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(4)
        self.speed_btn = QPushButton(self.playback_speed_label())
        self.overlay_btn = QPushButton()
        self.theme_btn = QPushButton()
        for btn in [self.speed_btn, self.overlay_btn, self.theme_btn]:
            btn.setObjectName("titleTinyButton")
            btn.setFixedSize(34, 22)
        self.speed_btn.setFixedWidth(44)
        self.overlay_btn.setIconSize(QSize(18, 18))
        self.theme_btn.setIconSize(QSize(18, 18))
        row.addWidget(self.speed_btn)
        row.addWidget(self.overlay_btn)
        row.addWidget(self.theme_btn)
        self.speed_btn.clicked.connect(self.toggle_playback_speed)
        self.overlay_btn.clicked.connect(self.toggle_debug_overlays)
        self.theme_btn.clicked.connect(self.toggle_theme_mode)
        self.update_playback_controls()
        return wrapper

    def build_time_stats(self):
        widget = QWidget()
        grid = QGridLayout(widget)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(3)
        for row, name in enumerate(["阶段 1 炸弹评估", "巡图规划", "阶段 2 炸弹评估", "推箱规划"]):
            key = QLabel(name)
            value = QLabel("-- ms")
            key.setObjectName("metricKey")
            value.setObjectName("metricValue")
            value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            key.setMinimumHeight(18)
            value.setMinimumHeight(18)
            grid.addWidget(key, row, 0)
            grid.addWidget(value, row, 1)
            self.time_value_labels.append(value)
        self.build_label = QLabel("")
        self.build_label.setObjectName("buildLabel")
        grid.addWidget(self.build_label, 4, 0, 1, 2)
        return widget

    def build_cost_stats(self):
        widget = QWidget()
        grid = QGridLayout(widget)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(6)
        headers = ["", "步数", "拐点", "观测", "转向", "总代价"]
        for col, text in enumerate(headers):
            label = QLabel(text)
            label.setObjectName("tableHeader")
            label.setAlignment(Qt.AlignRight if col > 0 else Qt.AlignLeft)
            label.setMinimumHeight(24)
            grid.addWidget(label, 0, col)

        for row, name in enumerate(["寻图", "推箱", "总计"], start=1):
            label = QLabel(name)
            label.setObjectName("tableName")
            label.setMinimumHeight(24)
            grid.addWidget(label, row, 0)
            self.cost_value_labels[name] = []
            for col in range(1, 6):
                value = QLabel("--")
                value.setObjectName("tableValue")
                value.setAlignment(Qt.AlignRight)
                value.setMinimumHeight(24)
                grid.addWidget(value, row, col)
                self.cost_value_labels[name].append(value)
        return widget

    def apply_style(self):
        if self.dark_mode:
            colors = {
                "root": "#111827",
                "text": "#e5edf6",
                "panel": "#172033",
                "panel_border": "#334155",
                "section": "#1f2937",
                "section_border": "#3a4658",
                "title": "#d9e4f0",
                "map_name": "#7db7ff",
                "status_bg": "#1e3554",
                "status_border": "#355f8c",
                "status_text": "#b9d9ff",
                "muted": "#b8c4d4",
                "metric": "#f4f7fb",
                "danger": "#f06f7a",
                "build": "#92a2b6",
                "input_bg": "#111827",
                "input_border": "#44556a",
                "button_bg": "#263244",
                "button_border": "#4b5f78",
                "button_hover": "#314158",
                "button_pressed": "#223047",
                "button_disabled_bg": "#202a39",
                "button_disabled_text": "#718096",
                "canvas_bg": "#182233",
                "canvas_border": "#46576c",
            }
        else:
            colors = {
                "root": "#eef2f6",
                "text": "#26313f",
                "panel": "#f8fafc",
                "panel_border": "#d9e1ea",
                "section": "#ffffff",
                "section_border": "#e1e7ef",
                "title": "#506174",
                "map_name": "#1469b8",
                "status_bg": "#eaf2ff",
                "status_border": "#c9dcf8",
                "status_text": "#225b98",
                "muted": "#4d5c6c",
                "metric": "#17202c",
                "danger": "#b23b3b",
                "build": "#738193",
                "input_bg": "#f8fafc",
                "input_border": "#cad4df",
                "button_bg": "#ffffff",
                "button_border": "#c9d3df",
                "button_hover": "#f0f5fb",
                "button_pressed": "#e6eef8",
                "button_disabled_bg": "#edf1f5",
                "button_disabled_text": "#9aa6b2",
                "canvas_bg": "#dfe6ed",
                "canvas_border": "#b9c5d1",
            }
        self.setStyleSheet(
            """
            QWidget#root {
                background: %(root)s;
                color: %(text)s;
                font-family: "Microsoft YaHei", "Segoe UI", Arial;
                font-size: 14px;
            }
            QLabel {
                color: %(text)s;
            }
            QScrollArea#sidebarScroll {
                background: transparent;
                border: none;
            }
            QFrame#sidebar, QFrame#workspace {
                background: %(panel)s;
                border: 1px solid %(panel_border)s;
                border-radius: 10px;
            }
            QFrame#section {
                background: %(section)s;
                border: 1px solid %(section_border)s;
                border-radius: 8px;
            }
            QLabel#sectionTitle {
                color: %(title)s;
                font-size: 15px;
                font-weight: 700;
            }
            QLabel#mapName {
                color: %(map_name)s;
                font-size: 14px;
                font-weight: 700;
            }
            QLabel#statusLabel {
                background: %(status_bg)s;
                border: 1px solid %(status_border)s;
                border-radius: 8px;
                color: %(status_text)s;
                font-size: 14px;
                font-weight: 700;
                padding: 4px 8px;
            }
            QLabel#metricKey, QLabel#fieldLabel {
                color: %(muted)s;
                font-weight: 600;
            }
            QLabel#metricKey, QLabel#metricValue {
                font-size: 12px;
            }
            QLabel#metricValue, QLabel#tableValue {
                color: %(metric)s;
                font-family: Consolas, "Microsoft YaHei";
                font-weight: 700;
            }
            QLabel#tableHeader, QLabel#tableName {
                color: %(danger)s;
                font-size: 13px;
                font-weight: 700;
            }
            QLabel#tableValue {
                font-size: 12px;
            }
            QLabel#buildLabel {
                color: %(build)s;
                font-family: Consolas, "Microsoft YaHei";
            }
            QLineEdit {
                background: %(input_bg)s;
                color: %(text)s;
                border: 1px solid %(input_border)s;
                border-radius: 6px;
                padding: 4px 8px;
                min-height: 24px;
                selection-background-color: #2f80ed;
            }
            QPushButton {
                background: %(button_bg)s;
                color: %(text)s;
                border: 1px solid %(button_border)s;
                border-radius: 7px;
                padding: 5px 10px;
                min-height: 24px;
                font-weight: 600;
            }
            QPushButton:hover {
                background: %(button_hover)s;
                border-color: %(button_border)s;
            }
            QPushButton:pressed {
                background: %(button_pressed)s;
            }
            QPushButton:disabled {
                color: %(button_disabled_text)s;
                background: %(button_disabled_bg)s;
            }
            QPushButton#primaryButton {
                background: #1f9d63;
                color: white;
                border-color: #188452;
                font-size: 15px;
                padding: 7px 12px;
                min-height: 28px;
            }
            QPushButton#primaryButton:hover {
                background: #188d58;
            }
            QPushButton#iconButton {
                padding: 3px 8px;
            }
            QPushButton#titleTinyButton {
                padding: 0px 4px;
                min-height: 18px;
                border-radius: 5px;
                font-size: 12px;
                font-weight: 700;
            }
            QWidget#mapCanvas {
                background: %(canvas_bg)s;
                border: 1px solid %(canvas_border)s;
                border-radius: 8px;
            }
            """ % colors
        )

    def load_user_settings(self):
        try:
            with open(SETTINGS_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError):
            return {}
        car_start = data.get("car_start")
        if not isinstance(car_start, list) or len(car_start) != 2:
            data.pop("car_start", None)
            return data
        try:
            x = max(0, min(MAP_W - 1, int(car_start[0])))
            y = max(0, min(MAP_H - 1, int(car_start[1])))
        except (TypeError, ValueError):
            data.pop("car_start", None)
            return data
        data["car_start"] = [x, y]
        data["dark_mode"] = bool(data.get("dark_mode", False))
        return data

    def save_user_settings(self):
        try:
            with open(SETTINGS_PATH, "w", encoding="utf-8") as f:
                json.dump(self.user_settings, f, ensure_ascii=False, indent=2)
        except OSError:
            pass

    def save_simulation_switches(self, _checked=False):
        self.user_settings["simulation_diagonal_move"] = self.diagonal_move_checkbox.isChecked()
        self.user_settings["simulation_box_extra_observe"] = self.box_extra_observe_checkbox.isChecked()
        self.user_settings["simulation_target_extra_observe"] = self.target_extra_observe_checkbox.isChecked()
        self.save_user_settings()

    def save_current_car_start(self):
        try:
            x = max(0, min(MAP_W - 1, int(self.car_x_edit.text())))
            y = max(0, min(MAP_H - 1, int(self.car_y_edit.text())))
        except ValueError:
            return False
        self.user_settings["car_start"] = [x, y]
        self.has_saved_car_start = True
        self.save_user_settings()
        return True

    def on_car_start_changed(self, *_):
        self.save_current_car_start()

    def closeEvent(self, event):
        self.save_current_car_start()
        self.cancel_animation_timer()
        self.stop_solver_thread()
        super().closeEvent(event)

    def stop_solver_thread(self):
        thread = self.solver_thread
        if thread is None:
            return
        thread.stop()
        if thread.isRunning():
            thread.wait(1000)
        if self.solver_thread is thread:
            self.solver_thread = None

    def scan_map_directory(self, directory):
        directory = os.path.abspath(directory)
        self.current_map_dir = directory
        if os.path.exists(directory) and os.path.isdir(directory):
            files = [f for f in os.listdir(directory) if f.endswith(".txt")]
            files.sort(key=lambda var: [
                (0, int(x)) if x.isdigit() else (1, x.lower())
                for x in re.findall(r"[0-9]+|[^0-9]+", var)
            ])
            self.map_files = [os.path.abspath(os.path.join(directory, f)) for f in files]
        else:
            self.map_files = []

    def normalize_path(self, path):
        return os.path.normcase(os.path.abspath(path))

    def load_map_by_path(self, filepath):
        filepath = os.path.abspath(filepath)
        directory = os.path.dirname(filepath)
        self.scan_map_directory(directory)
        target = self.normalize_path(filepath)
        for idx, candidate in enumerate(self.map_files):
            if self.normalize_path(candidate) == target:
                self.load_map_by_index(idx)
                return
        self.map_files.append(filepath)
        self.load_map_by_index(len(self.map_files) - 1)

    def choose_map(self):
        filepath, _ = QFileDialog.getOpenFileName(
            self,
            "选择地图文件",
            self.current_map_dir,
            "Text Files (*.txt)",
        )
        if filepath:
            self.load_map_by_path(filepath)

    def prev_map(self):
        if self.map_files and self.current_map_index > 0:
            self.load_map_by_index(self.current_map_index - 1)
        elif self.map_files:
            QMessageBox.information(self, "提示", "已经是文件夹里的第一张地图了")

    def next_map(self):
        if self.map_files and self.current_map_index < len(self.map_files) - 1:
            self.load_map_by_index(self.current_map_index + 1)
        elif self.map_files:
            QMessageBox.information(self, "提示", "已经是文件夹里的最后一张地图了")

    def reload_map(self):
        if self.current_map_path:
            self.load_map_by_index(self.current_map_index)

    def load_map_by_index(self, index):
        if not self.map_files or index < 0 or index >= len(self.map_files):
            return
        self.save_current_semantic_mapping()
        if self.has_saved_car_start:
            self.save_current_car_start()

        self.current_map_index = index
        self.current_map_path = self.map_files[index]
        self.cancel_animation_timer()

        self.boxes.clear()
        self.targets.clear()
        self.bombs.clear()
        try:
            car_x = max(0, min(MAP_W - 1, int(self.car_x_edit.text())))
            car_y = max(0, min(MAP_H - 1, int(self.car_y_edit.text())))
        except ValueError:
            car_x, car_y = PLAN_START_X, PLAN_START_Y
        # 界面坐标使用左下角规划坐标，显示位置保持编辑框中的坐标
        self.player = [car_x, car_y]
        self.is_solving = False
        self.playback_paused = False
        self.animation_finished = True
        self.update_playback_controls()

        self.update_cost_table()
        self.update_time_stats()
        self.build_label.setText("")
        self.build_info = ""
        file_name = os.path.basename(self.current_map_path)
        self.map_name_label.setText(file_name)
        self.set_status("地图已加载", "idle")

        with open(self.current_map_path, "r", encoding="utf-8-sig") as f:
            lines = [line.strip() for line in f.readlines() if line.strip()]
            map_lines = lines[:MAP_H]
            while len(map_lines) < MAP_H:
                map_lines.append("-" * MAP_W)
            self.map_data = [list(line.ljust(MAP_W, "-"))[:MAP_W] for line in map_lines]

        for file_y in range(MAP_H):
            grid_y = MAP_H - 1 - file_y
            for x in range(MAP_W):
                c = self.map_data[file_y][x]
                if c == "@":
                    if not self.has_saved_car_start:
                        self.player = [x, grid_y]
                        self.car_x_edit.setText(str(x))
                        self.car_y_edit.setText(str(grid_y))
                elif c == "$":
                    self.boxes.append([x, grid_y])
                elif c == ".":
                    self.targets.append([x, grid_y])
                elif c == "*":
                    self.bombs.append([x, grid_y])

        self.semantic_defaults = self.build_default_semantics()
        self.refresh_ui_inputs()
        self.canvas.update()

    def semantic_mode_label(self):
        return "随机" if self.semantic_random_mode else "固定"

    def update_semantic_mode_button(self):
        self.semantic_mode_btn.setText(self.semantic_mode_label())

    def toggle_semantic_mode(self):
        self.semantic_random_mode = not self.semantic_random_mode
        self.update_semantic_mode_button()
        if self.semantic_random_mode:
            self.apply_default_semantics(randomize_boxes=True)
        else:
            self.save_current_semantic_mapping()

    def build_ordered_semantics(self, count):
        return "".join(str(i % 10) for i in range(count))

    def build_random_box_semantics(self, target_sem, previous=""):
        if len(target_sem) != len(self.boxes):
            target_sem = self.build_ordered_semantics(len(self.boxes))
        values = list(target_sem)
        if len(values) <= 1:
            return "".join(values)
        for _ in range(8):
            random.shuffle(values)
            candidate = "".join(values)
            if candidate != previous:
                return candidate
        return "".join(values)

    def build_default_semantics(self, previous_box_sem=""):
        target_sem = self.build_ordered_semantics(len(self.targets))
        box_sem = self.build_random_box_semantics(target_sem, previous_box_sem)
        return box_sem, target_sem

    def apply_default_semantics(self, randomize_boxes):
        target_sem = self.build_ordered_semantics(len(self.targets))
        if randomize_boxes:
            box_sem = self.build_random_box_semantics(target_sem, self.box_semantic_edit.text().strip())
        else:
            box_sem = self.build_ordered_semantics(len(self.boxes))
        self.box_semantic_edit.setText(box_sem)
        self.target_semantic_edit.setText(target_sem)
        if self.current_map_path:
            self.semantic_match_cache[self.current_map_path] = (box_sem, target_sem)

    def refresh_ui_inputs(self):
        if self.semantic_random_mode:
            box_sem, target_sem = self.build_default_semantics(self.box_semantic_edit.text().strip())
        else:
            cached_values = self.semantic_match_cache.get(self.current_map_path)
            if cached_values is None:
                cached_values = (
                    self.build_ordered_semantics(len(self.boxes)),
                    self.build_ordered_semantics(len(self.targets)),
                )
            if not isinstance(cached_values, (tuple, list)) or len(cached_values) != 2:
                cached_values = self.semantic_defaults
            box_sem, target_sem = cached_values
            if len(box_sem) != len(self.boxes) or len(target_sem) != len(self.targets):
                box_sem = self.build_ordered_semantics(len(self.boxes))
                target_sem = self.build_ordered_semantics(len(self.targets))
        self.box_semantic_edit.setText(box_sem)
        self.target_semantic_edit.setText(target_sem)
        if self.current_map_path:
            self.semantic_match_cache[self.current_map_path] = (box_sem, target_sem)

    def save_current_semantic_mapping(self):
        values = (self.box_semantic_edit.text(), self.target_semantic_edit.text())
        if len(values[0]) != len(self.boxes) or len(values[1]) != len(self.targets):
            return
        self.last_semantic_mapping = values
        if self.current_map_path:
            self.semantic_match_cache[self.current_map_path] = values

    def validate_semantic_inputs(self):
        box_sem = self.box_semantic_edit.text().strip()
        target_sem = self.target_semantic_edit.text().strip()
        if len(box_sem) != len(self.boxes):
            QMessageBox.critical(self, "语义错误", f"箱子语义长度应为 {len(self.boxes)}，当前为 {len(box_sem)}")
            return None
        if len(target_sem) != len(self.targets):
            QMessageBox.critical(self, "语义错误", f"目标点语义长度应为 {len(self.targets)}，当前为 {len(target_sem)}")
            return None
        if not all(ch in "0123456789" for ch in box_sem + target_sem):
            QMessageBox.critical(self, "语义错误", "语义只能包含 0-9，例如箱子 012、目标 012")
            return None

        box_counts = {str(i): box_sem.count(str(i)) for i in range(10)}
        target_counts = {str(i): target_sem.count(str(i)) for i in range(10)}
        if box_counts != target_counts:
            QMessageBox.critical(self, "语义错误", "箱子语义和目标点语义数量对不上")
            return None
        return box_sem, target_sem

    def update_time_stats(self, values=None):
        values = values or ["--", "--", "--", "--"]
        for label, value in zip(self.time_value_labels, values):
            label.setText(f"{value} ms")

    def update_cost_table(self, patrol=None, sokoban=None, total=None):
        data = {
            "寻图": patrol or ("--", "--", "--", "--", "--"),
            "推箱": sokoban or ("--", "--", "--", "--", "--"),
            "总计": total or ("--", "--", "--", "--", "--"),
        }
        for name, values in data.items():
            for label, value in zip(self.cost_value_labels[name], values):
                label.setText(str(value))

    def update_playback_controls(self):
        can_control = self.is_solving and not self.animation_finished
        self.pause_btn.setEnabled(can_control)
        self.pause_btn.setText("Resume" if self.playback_paused else "Pause")
        self.step_btn.setEnabled(can_control)
        self.solve_btn.setEnabled(not self.is_solver_running)
        for checkbox in [
            self.diagonal_move_checkbox,
            self.box_extra_observe_checkbox,
            self.target_extra_observe_checkbox,
        ]:
            checkbox.setEnabled(not self.is_solver_running)
        self.update_title_button_icons()

    def playback_speed_label(self):
        return PLAYBACK_SPEED_LABELS[self.playback_speed_index]

    def playback_speed(self):
        return PLAYBACK_SPEEDS[self.playback_speed_index]

    def update_title_button_icons(self):
        if hasattr(self, "overlay_btn"):
            self.overlay_btn.setIcon(self.make_eye_icon(crossed=not self.show_debug_overlays))
        if hasattr(self, "theme_btn"):
            self.theme_btn.setIcon(self.make_theme_icon())

    def scaled_animation_delay(self, delay):
        if delay <= 0:
            return 0
        return max(1, int(round(delay / self.playback_speed())))

    def toggle_playback_speed(self):
        self.playback_speed_index = (self.playback_speed_index + 1) % len(PLAYBACK_SPEEDS)
        self.speed_btn.setText(self.playback_speed_label())
        if self.is_solving and not self.animation_finished and not self.playback_paused:
            self.schedule_next_step(self.last_animation_delay)

    def toggle_debug_overlays(self):
        self.show_debug_overlays = not self.show_debug_overlays
        self.update_title_button_icons()
        self.canvas.update()

    def toggle_theme_mode(self):
        self.dark_mode = not self.dark_mode
        self.user_settings["dark_mode"] = self.dark_mode
        self.save_user_settings()
        self.apply_style()
        self.update_title_button_icons()
        self.set_status(self.status_label.text(), self.status_kind)
        self.canvas.update()

    def cancel_animation_timer(self):
        if self.anim_timer.isActive():
            self.anim_timer.stop()

    def schedule_next_step(self, delay=60):
        self.cancel_animation_timer()
        self.last_animation_delay = delay
        if not self.playback_paused and not self.animation_finished:
            self.anim_timer.start(self.scaled_animation_delay(delay))

    def toggle_playback(self):
        if not self.is_solving or self.animation_finished:
            return
        self.playback_paused = not self.playback_paused
        if self.playback_paused:
            self.cancel_animation_timer()
        else:
            self.schedule_next_step(0)
        self.update_playback_controls()

    def step_once(self):
        if not self.is_solving or self.animation_finished:
            return
        self.playback_paused = True
        self.cancel_animation_timer()
        self.update_playback_controls()
        self.animate_next_step(manual=True)

    def set_status(self, text, kind="idle"):
        self.status_kind = kind
        colors = ({
            "idle": ("#1e3554", "#355f8c", "#b9d9ff"),
            "running": ("#4a3414", "#8a6428", "#ffd38a"),
            "phase1": ("#173657", "#315f91", "#acd4ff"),
            "phase2": ("#163d2c", "#2c704f", "#a8e7c4"),
            "success": ("#34224f", "#6d55a2", "#ddc9ff"),
            "error": ("#4a211f", "#8a4039", "#ffbbb5"),
        } if self.dark_mode else {
            "idle": ("#eaf2ff", "#c9dcf8", "#225b98"),
            "running": ("#fff2dc", "#f4cf91", "#9a5a00"),
            "phase1": ("#e7f1ff", "#b9d7f6", "#2368a2"),
            "phase2": ("#e7f7ef", "#b7e0c9", "#20724a"),
            "success": ("#f1eafd", "#dac8f4", "#6f3ca3"),
            "error": ("#ffe8e6", "#f2beb8", "#ad3328"),
        })
        bg, border, fg = colors.get(kind, colors["idle"])
        self.status_label.setText(text)
        self.status_label.setStyleSheet(
            f"background: {bg}; border: 1px solid {border}; border-radius: 8px; "
            f"color: {fg}; font-size: 14px; font-weight: 700; padding: 4px 8px;"
        )

    def solve_and_animate(self):
        if self.current_map_path and self.is_solving and self.animation_finished and not self.sokoban_failed:
            box_sem = self.box_semantic_edit.text().strip()
            target_sem = self.target_semantic_edit.text().strip()
            # 成功演示后的终局会改动箱子、目标点和墙体，重跑前恢复原图
            self.load_map_by_index(self.current_map_index)
            # 随机模式交给 refresh_ui_inputs 重新生成语义，固定模式才保留当前输入
            if not self.semantic_random_mode and len(box_sem) == len(self.boxes) and len(target_sem) == len(self.targets):
                self.box_semantic_edit.setText(box_sem)
                self.target_semantic_edit.setText(target_sem)
                self.semantic_match_cache[self.current_map_path] = (box_sem, target_sem)

        semantic_values = self.validate_semantic_inputs()
        if semantic_values is None:
            return
        box_sem, target_sem = semantic_values
        self.save_current_semantic_mapping()
        self.cancel_animation_timer()
        self.set_status("C++ M7 引擎爆算中", "running")
        QApplication.processEvents()

        try:
            car_x = max(0, min(MAP_W - 1, int(self.car_x_edit.text())))
            car_y = max(0, min(MAP_H - 1, int(self.car_y_edit.text())))
        except ValueError:
            QMessageBox.critical(self, "Error", "Car start X/Y must be integers")
            return
        self.save_current_car_start()
        # 界面和求解器使用同一套左下角规划坐标
        self.player = [car_x, car_y]

        with open(MAP_INPUT_PATH, "w", encoding="utf-8") as f:
            for row in self.map_data:
                f.write("".join("-" if c == "@" else c for c in row) + "\n")
            # 界面和规划器都使用左下角原点，坐标直接传递
            f.write(f"START {car_x} {car_y}\n")
            self.box_semantics = [int(ch) for ch in box_sem]
            self.target_semantics = [int(ch) for ch in target_sem]
            f.write("SEMANTICS BOXES " + box_sem + "\n")
            f.write("TARGETS " + target_sem + "\n")

        if not os.path.exists(SOLVER_PATH):
            QMessageBox.critical(self, "错误", f"未找到 {SOLVER_PATH}，请先编译 C++ 代码")
            return

        self.stop_solver_thread()
        try:
            os.remove(PATH_OUTPUT_PATH)
        except FileNotFoundError:
            pass
        except OSError as exc:
            self.set_status("无法清理旧求解结果", "error")
            QMessageBox.critical(self, "错误", str(exc))
            return

        self.is_solver_running = True
        self.update_playback_controls()
        solver_args = [
            f"--diagonal-move={int(self.diagonal_move_checkbox.isChecked())}",
            f"--box-extra-observe={int(self.box_extra_observe_checkbox.isChecked())}",
            f"--target-extra-observe={int(self.target_extra_observe_checkbox.isChecked())}",
        ]
        self.solver_thread = SolverThread(SOLVER_PATH, solver_args, VISUALIZER_DIR, self)
        self.solver_thread.finished_ok.connect(self.parse_and_play)
        self.solver_thread.failed.connect(self.on_solver_failed)
        self.solver_thread.timed_out.connect(self.on_solver_timed_out)
        self.solver_thread.finished.connect(self.on_solver_thread_finished)
        self.solver_thread.start()

    def on_solver_failed(self, message):
        self.sokoban_failed = True
        self.set_status("求解失败", "error")
        QMessageBox.critical(self, "错误", message)

    def on_solver_timed_out(self):
        self.sokoban_failed = True
        self.set_status(f"求解超过 {SOLVER_TIMEOUT_SECONDS:g} 秒，判定失败", "error")
        self.canvas.update()

    def on_solver_thread_finished(self):
        thread = self.sender()
        if thread is not self.solver_thread:
            return
        self.is_solver_running = False
        self.solver_thread = None
        self.update_playback_controls()

    def parse_and_play(self):
        if not os.path.exists(PATH_OUTPUT_PATH):
            self.set_status("求解器没有输出路径", "error")
            return

        self.bomb_tasks_1, self.bomb_tasks_2, self.chosen_obs = [], [], []
        self.obs_clear_pushes = []
        self.patrol_events, self.sokoban_path = [], []
        self.sokoban_failed = False
        self.sokoban_invalid_path = False
        self.build_info = ""
        self.cost_model = DEFAULT_COST_MODEL
        self.output_semantics = []
        self.box_ids = list(range(len(self.boxes)))

        with open(PATH_OUTPUT_PATH, "r", encoding="utf-8") as f:
            lines = [line.strip() for line in f.readlines() if line.strip()]

        def from_output_coord(x, y):
            # 求解器输出已经是左下角规划坐标，界面内部也使用同一坐标
            return [x, y]

        def from_output_quad(vals):
            start = from_output_coord(vals[0], vals[1])
            end = from_output_coord(vals[2], vals[3])
            return start + end

        def parse_bomb_task(line):
            vals = list(map(int, line.split()))
            task = BombTask(bomb=from_output_quad(vals[:4]), pushes=[])
            if len(vals) >= 5:
                push_count = vals[4]
                offset = 5
                for _ in range(push_count):
                    if offset + 3 >= len(vals):
                        break
                    task.pushes.append(from_output_quad(vals[offset:offset + 4]))
                    offset += 4
            return task

        patrol_parse_pos = list(self.player)
        i = 0
        while i < len(lines):
            line = lines[i]
            if line.startswith("TIMES"):
                t = line.split()
                self.update_time_stats(t[1:5])
            elif line.startswith("BUILD"):
                self.build_info = line[len("BUILD"):].strip()
            elif line.startswith("COST_MODEL"):
                values = [int(v) for v in line.split()[1:5]]
                if len(values) == 4:
                    self.cost_model = tuple(values)
            elif line.startswith("SEMANTICS"):
                self.output_semantics = [int(v) for v in line.split()[1:]]
            elif line.startswith("MATCHED_IDS"):
                self.box_ids = [int(v) for v in line.split()[1:]]
            elif line.startswith("BOMB_TASKS_1"):
                cnt = int(line.split()[1])
                for _ in range(cnt):
                    i += 1
                    self.bomb_tasks_1.append(parse_bomb_task(lines[i]))
            elif line.startswith("BOMB_TASKS_2"):
                cnt = int(line.split()[1])
                for _ in range(cnt):
                    i += 1
                    self.bomb_tasks_2.append(parse_bomb_task(lines[i]))
            elif line.startswith("CHOSEN_OBS"):
                cnt = int(line.split()[1])
                for _ in range(cnt):
                    i += 1
                    x, y = map(int, lines[i].split())
                    self.chosen_obs.append(from_output_coord(x, y))
            elif line.startswith("OBS_CLEAR_PUSHES"):
                cnt = int(line.split()[1])
                for _ in range(cnt):
                    i += 1
                    vals = list(map(int, lines[i].split()))
                    if len(vals) >= 4:
                        self.obs_clear_pushes.append(from_output_quad(vals[:4]))
            elif line == "PATROL":
                i += 1
                while i < len(lines) and lines[i] not in ["SOKOBAN", "FAILED", "SOKOBAN_REPLAY_BEGIN", "MCU_TRACE"]:
                    parts = lines[i].split()
                    if parts[0] == "OBSERVE":
                        observe_yaw = int(parts[3]) if len(parts) >= 4 else None
                        self.patrol_events.append(
                            ["OBSERVE", int(parts[1]), parts[2] == "1", observe_yaw]
                        )
                    else:
                        target = from_output_coord(int(parts[0]), int(parts[1]))
                        patrol_parse_pos = append_patrol_move_events(
                            self.patrol_events, patrol_parse_pos, target
                        )
                    i += 1
                continue
            elif line == "SOKOBAN_REPLAY_BEGIN":
                i += 1
                while i < len(lines) and lines[i] != "SOKOBAN_REPLAY_END":
                    i += 1
                if i < len(lines):
                    i += 1
                continue
            elif line == "SOKOBAN_PATH_INVALID":
                self.sokoban_invalid_path = True
            elif line == "SOKOBAN":
                i += 1
                while i < len(lines) and lines[i] != "MCU_TRACE":
                    if lines[i] == "FAILED":
                        self.sokoban_failed = True
                    else:
                        x, y = map(int, lines[i].split())
                        self.sokoban_path.append(from_output_coord(x, y))
                    i += 1
                continue
            elif line == "MCU_TRACE":
                # MCU/ART2 调试事件不参与路径回放
                i += 1
                while i < len(lines) and lines[i] not in [
                    "PATROL", "SOKOBAN", "SOKOBAN_PATH_INVALID", "SOKOBAN_REPLAY_BEGIN"
                ]:
                    i += 1
                continue
            i += 1

        if self.build_info:
            self.build_label.setText(f"BUILD: {self.build_info}")

        self.prepare_semantic_labels()
        self.update_cost_metrics()

        self.is_solving = True
        self.current_phase = 1
        self.discovered_entities = set()
        self.anim_step = 0
        self.playback_paused = False
        self.animation_finished = False
        self.update_playback_controls()
        self.canvas.update()
        self.schedule_next_step(0)

    def prepare_semantic_labels(self):
        self.box_letter_map = {}
        self.target_letter_map = {}
        semantic_values = self.output_semantics
        if len(semantic_values) < len(self.boxes) + len(self.targets):
            box_sem = self.box_semantic_edit.text().strip()
            target_sem = self.target_semantic_edit.text().strip()
            semantic_values = [int(ch) for ch in box_sem + target_sem if ch.isdigit()]
        semantic_letters = {}
        next_letter = 0
        for sem in semantic_values:
            if sem not in semantic_letters:
                semantic_letters[sem] = chr(65 + (next_letter % 26))
                next_letter += 1
        for b_id in range(len(self.boxes)):
            sem = semantic_values[b_id] if b_id < len(semantic_values) else b_id
            self.box_letter_map[b_id] = semantic_letters.get(sem, "?")
        for t_id in range(len(self.targets)):
            idx = len(self.boxes) + t_id
            sem = semantic_values[idx] if idx < len(semantic_values) else t_id
            self.target_letter_map[t_id] = semantic_letters.get(sem, "?")
        self.animation_box_semantics = [
            semantic_values[i] if i < len(semantic_values) else i
            for i in range(len(self.boxes))
        ]
        self.animation_target_semantics = [
            semantic_values[len(self.boxes) + i]
            if len(self.boxes) + i < len(semantic_values) else i
            for i in range(len(self.targets))
        ]

    def update_cost_metrics(self):
        def count_turns(path_coords):
            turn_count = 0
            prev_dir = None
            for i in range(1, len(path_coords)):
                p1, p2 = path_coords[i - 1], path_coords[i]
                dx, dy = p2[0] - p1[0], p2[1] - p1[1]
                if dx == 0 and dy == 0:
                    continue
                curr_dir = (dx, dy)
                if prev_dir is not None:
                    cross = prev_dir[0] * curr_dir[1] - prev_dir[1] * curr_dir[0]
                    dot = prev_dir[0] * curr_dir[0] + prev_dir[1] * curr_dir[1]
                    if cross != 0 or dot <= 0:
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

        patrol_path = [self.player]
        for ev in self.patrol_events:
            if ev[0] == "MOVE":
                patrol_path.append(ev[1])

        sokoban_start = patrol_path[-1] if patrol_path else self.player
        sokoban_path = [sokoban_start]
        for pt in self.sokoban_path:
            sokoban_path.append(pt)

        patrol_turns = count_turns(patrol_path)
        sokoban_turns = count_turns(sokoban_path)

        patrol_rotations = 0
        current_yaw = 90
        virtual_pos = self.player
        obs_group = []
        valid_fov = {
            0: [(1, 0), (2, 0), (1, 1), (1, -1), (2, 1), (2, -1)],
            90: [(0, 1), (0, 2), (-1, 1), (1, 1), (-1, 2), (1, 2)],
            180: [(-1, 0), (-2, 0), (-1, -1), (-1, 1), (-2, -1), (-2, 1)],
            270: [(0, -1), (0, -2), (1, -1), (-1, -1), (1, -2), (-1, -2)],
        }

        def process_obs_group(group, pos, curr_y):
            if not group:
                return curr_y, 0
            objects_rel = []
            for ent_id, is_box in group:
                if is_box:
                    ox, oy = self.boxes[ent_id]
                else:
                    target_idx = ent_id - len(self.boxes) if ent_id >= len(self.boxes) else ent_id
                    ox, oy = self.targets[target_idx]
                dx = ox - pos[0]
                dy = pos[1] - oy
                objects_rel.append((dx, dy))

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
            if curr_y in possible_yaws:
                return curr_y, 0
            return possible_yaws[0], 1

        for ev in self.patrol_events:
            if ev[0] == "OBSERVE":
                observe_yaw = ev[3] if len(ev) >= 4 else None
                if observe_yaw is None:
                    # 兼容旧输出：缺少真实 yaw 时才按同一驻留点的实体位置推断
                    obs_group.append((ev[1], ev[2]))
                    continue

                # 新输出逐次记录宏动作的 ALIGN_YAW，不能把同格连续观测合并
                if obs_group:
                    current_yaw, added_rot = process_obs_group(obs_group, virtual_pos, current_yaw)
                    patrol_rotations += added_rot
                    obs_group = []
                observe_yaw %= 360
                if current_yaw != observe_yaw:
                    patrol_rotations += 1
                    current_yaw = observe_yaw
            elif ev[0] == "MOVE":
                if obs_group:
                    current_yaw, added_rot = process_obs_group(obs_group, virtual_pos, current_yaw)
                    patrol_rotations += added_rot
                    obs_group = []
                virtual_pos = ev[1]
        if obs_group:
            current_yaw, added_rot = process_obs_group(obs_group, virtual_pos, current_yaw)
            patrol_rotations += added_rot

        patrol_steps = path_length(patrol_path)
        sokoban_steps = len(self.sokoban_path)
        patrol_observations = len(self.chosen_obs)
        sokoban_observations = 0
        total_steps = patrol_steps + sokoban_steps
        total_observations = patrol_observations + sokoban_observations
        sokoban_rotations = 0
        total_turns = patrol_turns + sokoban_turns
        total_rotations = patrol_rotations + sokoban_rotations

        def calc_cost(steps, turns, observations, rotations):
            move_cost, stop_cost, observe_extra, turn_extra = self.cost_model
            observe_cost = stop_cost + observe_extra
            return (
                steps * move_cost
                + turns * stop_cost
                + observations * observe_cost
                + rotations * turn_extra
            )

        patrol_cost = calc_cost(
            patrol_steps, patrol_turns, patrol_observations, patrol_rotations
        )
        sokoban_cost = calc_cost(
            sokoban_steps, sokoban_turns, sokoban_observations, sokoban_rotations
        )
        total_cost = calc_cost(
            total_steps, total_turns, total_observations, total_rotations
        )
        self.update_cost_table(
            (patrol_steps, patrol_turns, patrol_observations, patrol_rotations, patrol_cost),
            (sokoban_steps, sokoban_turns, sokoban_observations, sokoban_rotations, sokoban_cost),
            (total_steps, total_turns, total_observations, total_rotations, total_cost),
        )

    def animate_next_step(self, manual=False):
        next_pos = None
        if self.anim_step < len(self.patrol_events):
            self.set_status("阶段 1: 巡图盲区解锁中", "phase1")
            self.current_phase = 1
            event = self.patrol_events[self.anim_step]
            if event[0] == "OBSERVE":
                ent_id, is_box = event[1], event[2]
                if not is_box and ent_id >= len(self.boxes):
                    ent_id -= len(self.boxes)
                self.discovered_entities.add(f"{'B' if is_box else 'T'}{ent_id}")
                self.canvas.update()
                self.anim_step += 1
                if not manual:
                    self.schedule_next_step(50)
                self.update_playback_controls()
                return
            next_pos = event[1]
        elif self.anim_step - len(self.patrol_events) < len(self.sokoban_path):
            self.set_status("阶段 2: 终局推箱融合中", "phase2")
            self.current_phase = 2
            if self.anim_step == len(self.patrol_events):
                for i in range(len(self.boxes)):
                    self.discovered_entities.add(f"B{i}")
                for i in range(len(self.targets)):
                    self.discovered_entities.add(f"T{i}")
            idx = self.anim_step - len(self.patrol_events)
            next_pos = self.sokoban_path[idx]
        else:
            if self.sokoban_invalid_path:
                self.set_status("推箱路径超出固定容量", "error")
                QMessageBox.warning(self, "结束", "推箱路径无效：路径长度超过固定容量，未执行完整解。")
            elif self.sokoban_failed:
                self.set_status("推箱子无解", "error")
                QMessageBox.warning(self, "结束", "巡图完毕，但推箱子无解！(可能是墙壁死锁)")
            else:
                self.set_status("挑战成功", "success")
            self.animation_finished = True
            self.cancel_animation_timer()
            self.update_playback_controls()
            return

        dx, dy = next_pos[0] - self.player[0], next_pos[1] - self.player[1]
        for i, bx in enumerate(self.boxes):
            if bx and bx == next_pos:
                bx[0] += dx
                bx[1] += dy
                box_sem = self.animation_box_semantics[i] if i < len(self.animation_box_semantics) else i
                finished_target = -1
                for target_id, target_pos in enumerate(self.targets):
                    if target_pos != bx:
                        continue
                    target_sem = (
                        self.animation_target_semantics[target_id]
                        if target_id < len(self.animation_target_semantics) else target_id
                    )
                    if target_sem == box_sem:
                        finished_target = target_id
                        break
                if finished_target == -1:
                    target_id = self.box_ids[i] if i < len(self.box_ids) else i
                    if 0 <= target_id < len(self.targets) and bx == self.targets[target_id]:
                        finished_target = target_id
                if finished_target != -1:
                    self.boxes[i] = None
                    self.targets[finished_target] = [-1, -1]

        for i, b in enumerate(self.bombs):
            if b and b == next_pos:
                nx, ny = b[0] + dx, b[1] + dy
                file_ny = MAP_H - 1 - ny
                if 0 <= ny < MAP_H and 0 <= nx < MAP_W and self.map_data[file_ny][nx] == "#":
                    for dy_w in [-1, 0, 1]:
                        for dx_w in [-1, 0, 1]:
                            ey, ex = ny + dy_w, nx + dx_w
                            file_ey = MAP_H - 1 - ey
                            if 0 < ey < MAP_H - 1 and 0 < ex < MAP_W - 1 and self.map_data[file_ey][ex] == "#":
                                self.map_data[file_ey][ex] = "-"
                    self.bombs[i] = None
                else:
                    b[0] += dx
                    b[1] += dy

        self.player = next_pos
        self.canvas.update()
        self.anim_step += 1
        if not manual:
            self.schedule_next_step(60)
        self.update_playback_controls()

    def draw_map(self, painter):
        canvas_bg = "#182233" if self.dark_mode else "#dfe6ed"
        wall_fill = "#53606b" if self.dark_mode else "#778487"
        wall_stroke = "#738292" if self.dark_mode else "#5a6970"
        floor_fill = "#223047" if self.dark_mode else "#f4f7f9"
        floor_stroke = "#3b4a5e" if self.dark_mode else "#c8d0d8"
        painter.fillRect(self.canvas.rect(), QColor(canvas_bg))
        if not self.map_data:
            return
        for y in range(MAP_H):
            for x in range(MAP_W):
                rect = QRectF(x * CELL_SIZE, self.screen_y(y) * CELL_SIZE, CELL_SIZE, CELL_SIZE)
                if self.map_data[MAP_H - 1 - y][x] == "#":
                    self.draw_round_rect(painter, rect.adjusted(1, 1, -1, -1), wall_fill, wall_stroke, 5)
                else:
                    self.draw_round_rect(painter, rect.adjusted(1, 1, -1, -1), floor_fill, floor_stroke, 4)
                if [x, y] in self.targets:
                    tid = self.targets.index([x, y])
                    self.draw_target(painter, x, y, tid)

        if self.is_solving and self.show_debug_overlays:
            self.draw_overlays(painter)

        for i, b in enumerate(self.bombs):
            if b is not None:
                self.draw_bomb(painter, b[0], b[1])
        for i, bx in enumerate(self.boxes):
            if bx is not None:
                self.draw_box(painter, bx[0], bx[1], i)
        if self.player:
            self.draw_car(painter, self.player[0], self.player[1])

    def cell_rect(self, x, y, margin=5):
        return QRectF(
            x * CELL_SIZE + margin,
            self.screen_y(y) * CELL_SIZE + margin,
            CELL_SIZE - margin * 2,
            CELL_SIZE - margin * 2,
        )

    @staticmethod
    def screen_y(grid_y):
        """把左下角规划坐标转换为画布的屏幕行号"""
        return MAP_H - 1 - grid_y

    def draw_round_rect(self, painter, rect, fill, stroke, radius):
        path = QPainterPath()
        path.addRoundedRect(rect, radius, radius)
        painter.fillPath(path, QColor(fill))
        painter.setPen(QPen(QColor(stroke), 1))
        painter.drawPath(path)

    def draw_target(self, painter, x, y, tid):
        rect = self.cell_rect(x, y, 7)
        self.draw_round_rect(painter, rect, "#9b59b6", "#7e3fa0", 6)
        self.draw_entity_text(painter, x, y, f"T{tid}", tid, False)

    def draw_box(self, painter, x, y, idx):
        rect = self.cell_rect(x, y, 6)
        painter.setBrush(QColor("#f4c542"))
        painter.setPen(QPen(QColor("#d79509"), 2))
        painter.drawRoundedRect(rect, 6, 6)
        self.draw_entity_text(painter, x, y, f"B{idx}", idx, True)

    def draw_bomb(self, painter, x, y):
        center = QPointF(x * CELL_SIZE + CELL_SIZE / 2, self.screen_y(y) * CELL_SIZE + CELL_SIZE / 2)
        painter.setBrush(QColor("#222831"))
        painter.setPen(QPen(QColor("#111820"), 2))
        painter.drawEllipse(center, 14, 14)
        painter.setPen(QPen(QColor("#ff4d4d"), 2))
        font = QFont("Arial", 16, QFont.Bold)
        painter.setFont(font)
        painter.drawText(self.cell_rect(x, y, 0), Qt.AlignCenter, "*")

    def draw_car(self, painter, x, y):
        color = "#2f80ed" if self.current_phase == 1 else "#1f9d63"
        rect = self.cell_rect(x, y, 5)
        painter.setBrush(QColor(color))
        painter.setPen(QPen(QColor("#183b59"), 2))
        painter.drawEllipse(rect)
        painter.setPen(QColor("#ffffff"))
        painter.setFont(QFont("Arial", 9, QFont.Bold))
        painter.drawText(rect, Qt.AlignCenter, "Car")

    def draw_entity_text(self, painter, x, y, raw_name, entity_id, is_box):
        text = ""
        if not self.is_solving:
            text = raw_name
        elif raw_name in self.discovered_entities:
            text = self.box_letter_map.get(entity_id, "?") if is_box else self.target_letter_map.get(entity_id, "?")
        if not text:
            return
        painter.setPen(QColor("#111111" if is_box else "#ffffff"))
        painter.setFont(QFont("Arial", 11, QFont.Bold))
        painter.drawText(self.cell_rect(x, y, 0), Qt.AlignCenter, text)

    def draw_overlays(self, painter):
        painter.setRenderHint(QPainter.Antialiasing, True)
        for ox, oy in self.chosen_obs:
            cx, cy = ox * CELL_SIZE + CELL_SIZE / 2, self.screen_y(oy) * CELL_SIZE + CELL_SIZE / 2
            painter.setPen(QPen(QColor("#2f80ed"), 3))
            painter.drawLine(QPointF(cx - 9, cy - 9), QPointF(cx + 9, cy + 9))
            painter.drawLine(QPointF(cx - 9, cy + 9), QPointF(cx + 9, cy - 9))

        dash_pen = QPen(QColor("#2f80ed"), 3)
        dash_pen.setDashPattern([5, 3])
        painter.setPen(dash_pen)
        for sx, sy, tx, ty in self.obs_clear_pushes:
            self.draw_arrow(painter, sx, sy, tx, ty, QColor("#2f80ed"), dashed=True)

        tasks = self.bomb_tasks_1 if self.current_phase == 1 else self.bomb_tasks_2
        for idx, task in enumerate(tasks):
            color = QColor(BOMB_COLORS[idx % len(BOMB_COLORS)])
            bx, by, wx, wy = task.bomb
            pen = QPen(color, 3)
            pen.setDashPattern([4, 4])
            painter.setPen(pen)
            painter.setBrush(Qt.NoBrush)
            painter.drawRoundedRect(self.cell_rect(bx, by, 3), 5, 5)
            painter.drawRoundedRect(self.cell_rect(wx, wy, 3), 5, 5)
            painter.drawLine(
                QPointF(bx * CELL_SIZE + CELL_SIZE / 2, self.screen_y(by) * CELL_SIZE + CELL_SIZE / 2),
                QPointF(wx * CELL_SIZE + CELL_SIZE / 2, self.screen_y(wy) * CELL_SIZE + CELL_SIZE / 2),
            )
            for sx, sy, tx, ty in task.pushes:
                self.draw_arrow(painter, sx, sy, tx, ty, color)

    def draw_arrow(self, painter, sx, sy, tx, ty, color, dashed=False):
        pen = QPen(color, 3)
        if dashed:
            pen.setDashPattern([5, 3])
        painter.setPen(pen)
        start = QPointF(sx * CELL_SIZE + CELL_SIZE / 2, self.screen_y(sy) * CELL_SIZE + CELL_SIZE / 2)
        end = QPointF(tx * CELL_SIZE + CELL_SIZE / 2, self.screen_y(ty) * CELL_SIZE + CELL_SIZE / 2)
        painter.drawLine(start, end)
        dx = end.x() - start.x()
        dy = end.y() - start.y()
        length = max((dx * dx + dy * dy) ** 0.5, 1.0)
        ux, uy = dx / length, dy / length
        left = QPointF(end.x() - ux * 10 - uy * 5, end.y() - uy * 10 + ux * 5)
        right = QPointF(end.x() - ux * 10 + uy * 5, end.y() - uy * 10 - ux * 5)
        painter.setBrush(color)
        path = QPainterPath()
        path.moveTo(end)
        path.lineTo(left)
        path.lineTo(right)
        path.closeSubpath()
        painter.fillPath(path, color)


def main():
    app = QApplication(sys.argv)
    window = SokobanVisualizerQt()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())

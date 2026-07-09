# app.py
# PAI-Car Step 5-3 PyQtGraph telemetry dashboard + course_map + profiles
#
# 목적:
#   1. Pico 2 W의 기존 UDP telemetry packet VERSION 1을 PC에서 수신한다.
#   2. 실시간 plot과 CSV 저장을 수행한다.
#   3. PC 키 입력으로 PING / NEXT_SECTION / EMERGENCY_STOP / RUN command를 Pico로 전송한다.
#   4. Space 입력 시 현재 telemetry snapshot을 section_marks.csv에 저장한다.
#   5. course_map.json을 읽어 현재 section 정보를 표시하고 기록한다.
#   6. profiles.json을 읽어 현재 section의 profile 값을 표시하고 기록한다.
#
# 실행:
#   python app.py
#
# 필요 패키지:
#   python -m pip install pyqtgraph PyQt6
#
# 키:
#   P       PING
#   Space   NEXT_SECTION
#   Z       EMERGENCY_STOP
#   Enter   RUN

import csv
import json
import os
import socket
import struct
import sys
import time
from collections import deque
from datetime import datetime

import pyqtgraph as pg

try:
    from PyQt6 import QtCore, QtWidgets
except ImportError:
    try:
        from PySide6 import QtCore, QtWidgets
    except ImportError:
        from PyQt5 import QtCore, QtWidgets

from command_sender import (
    CommandSender,
    CMD_PING,
    CMD_STOP,
    CMD_NEXT_SECTION,
    CMD_RUN,
)


# ------------------------------------------------------------
# Qt compatibility
# ------------------------------------------------------------

QT_KEY = getattr(QtCore.Qt, "Key", QtCore.Qt)

KEY_P = getattr(QT_KEY, "Key_P")
KEY_Z = getattr(QT_KEY, "Key_Z")
KEY_SPACE = getattr(QT_KEY, "Key_Space")
KEY_RETURN = getattr(QT_KEY, "Key_Return")
KEY_ENTER = getattr(QT_KEY, "Key_Enter")


def text_selectable_flag():
    if hasattr(QtCore.Qt, "TextInteractionFlag"):
        return QtCore.Qt.TextInteractionFlag.TextSelectableByMouse
    return QtCore.Qt.TextSelectableByMouse


def run_qt_app(app):
    if hasattr(app, "exec"):
        return app.exec()
    return app.exec_()


# ------------------------------------------------------------
# Paths
# ------------------------------------------------------------

APP_DIR = os.path.dirname(os.path.abspath(__file__))

COURSE_MAP_PATH = os.path.join(APP_DIR, "course_map.json")
PROFILES_PATH = os.path.join(APP_DIR, "profiles.json")
LOG_DIR = os.path.join(APP_DIR, "logs")


# ------------------------------------------------------------
# UDP telemetry receive settings
# ------------------------------------------------------------

LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 5005
RECV_BUFFER_SIZE = 2048

COMMAND_PORT = 5006


# ------------------------------------------------------------
# Pico telemetry packet VERSION 1
# ------------------------------------------------------------

MAGIC = 0x5041
VERSION = 1
PACKET_FORMAT = "<HBBHIHHh8HHhhhhBB"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

CSV_HEADER = [
    "wall_time",
    "seq",
    "t_ms",
    "control_ms",
    "send_ms",
    "base_speed",
    "n0", "n1", "n2", "n3", "n4", "n5", "n6", "n7",
    "position",
    "error",
    "d_error",
    "left_cmd",
    "right_cmd",
    "on_line",
    "is_marker",
    "packet_loss_count",
    "bad_packet_count",
    "last_packet_age_ms",
]

SECTION_MARK_HEADER = [
    "wall_time",
    "event_type",
    "old_section_id",
    "new_section_id",
    "section_display_no",
    "section_name",
    "section_label_ko",
    "section_type",
    "profile_key",
    "section_role",
    "section_note",

    "profile_label_ko",
    "profile_base_speed",
    "profile_curve_speed",
    "profile_sharp_curve_speed",
    "profile_min_run_speed",
    "profile_kp",
    "profile_kd",
    "profile_max_correction",
    "profile_reverse_allow",
    "profile_reverse_pwm_mid",
    "profile_reverse_pwm_high",
    "profile_error_curve_threshold",
    "profile_error_sharp_threshold",
    "profile_d_error_curve_threshold",
    "profile_d_error_sharp_threshold",
    "profile_search_pwm",
    "profile_line_loss_max_ms",

    "cmd_seq",
    "ack",
    "status",
    "rtt_ms",
    "seq",
    "t_ms",
    "base_speed",
    "position",
    "error",
    "d_error",
    "left_cmd",
    "right_cmd",
    "on_line",
    "is_marker",
    "packet_loss_count",
]

MAX_POINTS = 1500
RECV_TIMER_MS = 5
COMMAND_TIMER_MS = 20
PLOT_TIMER_MS = 33
FLUSH_EVERY_ROWS = 20

SECTION_ACK_TIMEOUT_MS = 1000


# ------------------------------------------------------------
# Course map / profiles
# ------------------------------------------------------------

def load_json_file(path, fallback_name):
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)

        return data, "loaded"

    except FileNotFoundError:
        return {
            "name": fallback_name,
            "sections": [],
            "profiles": {},
        }, "missing: {}".format(path)

    except Exception as exc:
        return {
            "name": fallback_name,
            "sections": [],
            "profiles": {},
        }, "error: {}".format(exc)


def build_section_index(course_map):
    sections_by_id = {}

    for section in course_map.get("sections", []):
        try:
            section_id = int(section.get("section_id"))
        except Exception:
            continue

        sections_by_id[section_id] = section

    section_ids = sorted(sections_by_id.keys())

    return sections_by_id, section_ids


def default_section_info(section_id):
    return {
        "section_id": section_id,
        "display_no": "",
        "name": "unknown_section",
        "label_ko": "알 수 없는 구간",
        "type": "UNKNOWN",
        "profile_key": "UNKNOWN",
        "role": "UNKNOWN",
        "note": "",
    }


def default_profile_info(profile_key):
    return {
        "label_ko": "알 수 없는 profile",
        "base_speed": "",
        "curve_speed": "",
        "sharp_curve_speed": "",
        "min_run_speed": "",
        "kp": "",
        "kd": "",
        "max_correction": "",
        "reverse_allow": "",
        "reverse_pwm_mid": "",
        "reverse_pwm_high": "",
        "error_curve_threshold": "",
        "error_sharp_threshold": "",
        "d_error_curve_threshold": "",
        "d_error_sharp_threshold": "",
        "search_pwm": "",
        "line_loss_max_ms": "",
        "note": "missing profile: {}".format(profile_key),
    }


# ------------------------------------------------------------
# Packet parsing
# ------------------------------------------------------------

def parse_packet(data: bytes):
    if len(data) != PACKET_SIZE:
        return None, "bad_size"

    try:
        values = struct.unpack(PACKET_FORMAT, data)
    except struct.error:
        return None, "struct_error"

    magic = values[0]
    version = values[1]

    if magic != MAGIC:
        return None, "bad_magic"

    if version != VERSION:
        return None, "bad_version"

    seq = values[3]
    t_ms = values[4]
    control_ms = values[5]
    send_ms = values[6]
    base_speed = values[7]
    norm = values[8:16]
    position = values[16]
    error = values[17]
    d_error = values[18]
    left_cmd = values[19]
    right_cmd = values[20]
    on_line = values[21]
    is_marker = values[22]

    row = {
        "seq": seq,
        "t_ms": t_ms,
        "control_ms": control_ms,
        "send_ms": send_ms,
        "base_speed": base_speed,
        "n0": norm[0],
        "n1": norm[1],
        "n2": norm[2],
        "n3": norm[3],
        "n4": norm[4],
        "n5": norm[5],
        "n6": norm[6],
        "n7": norm[7],
        "position": position,
        "error": error,
        "d_error": d_error,
        "left_cmd": left_cmd,
        "right_cmd": right_cmd,
        "on_line": on_line,
        "is_marker": is_marker,
    }
    return row, None


# ------------------------------------------------------------
# Main window
# ------------------------------------------------------------

class Dashboard(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("PAI-Car Step 5-3 Telemetry Dashboard + Profiles")
        self.resize(1200, 920)

        # --------------------------------------------------------
        # Course map
        # --------------------------------------------------------

        self.course_map, self.course_map_status = load_json_file(
            COURSE_MAP_PATH,
            "missing_course_map",
        )
        self.sections_by_id, self.section_ids = build_section_index(self.course_map)

        if self.section_ids:
            self.current_section_id = self.section_ids[0]
        else:
            self.current_section_id = 0

        self.pending_section_marks = {}

        # --------------------------------------------------------
        # Profiles
        # --------------------------------------------------------

        self.profile_doc, self.profile_status = load_json_file(
            PROFILES_PATH,
            "missing_profiles",
        )
        self.profiles = self.profile_doc.get("profiles", {})

        # --------------------------------------------------------
        # UDP sockets
        # --------------------------------------------------------

        self.telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_sock.bind((LISTEN_IP, LISTEN_PORT))
        self.telemetry_sock.setblocking(False)

        self.command_sender = CommandSender(
            target_ip=None,
            target_port=COMMAND_PORT,
        )

        # --------------------------------------------------------
        # CSV logs
        # --------------------------------------------------------

        os.makedirs(LOG_DIR, exist_ok=True)

        self.run_id = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.csv_path = os.path.join(
            LOG_DIR,
            "pai_car_pyqt_{}.csv".format(self.run_id),
        )
        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_HEADER)
        self.csv_writer.writeheader()

        self.section_csv_path = os.path.join(
            LOG_DIR,
            "section_marks_{}.csv".format(self.run_id),
        )
        self.section_csv_file = open(
            self.section_csv_path,
            "w",
            newline="",
            encoding="utf-8",
        )
        self.section_csv_writer = csv.DictWriter(
            self.section_csv_file,
            fieldnames=SECTION_MARK_HEADER,
        )
        self.section_csv_writer.writeheader()

        # --------------------------------------------------------
        # Runtime state
        # --------------------------------------------------------

        self.received_count = 0
        self.bad_packet_count = 0
        self.packet_loss_count = 0
        self.last_seq = None
        self.last_packet_time = None
        self.last_row = None
        self.last_bad_reason = ""
        self.last_pico_ip = None

        self.closed = False

        # --------------------------------------------------------
        # Plot buffers
        # --------------------------------------------------------

        self.t_buf = deque(maxlen=MAX_POINTS)
        self.position_buf = deque(maxlen=MAX_POINTS)
        self.error_buf = deque(maxlen=MAX_POINTS)
        self.d_error_buf = deque(maxlen=MAX_POINTS)
        self.left_buf = deque(maxlen=MAX_POINTS)
        self.right_buf = deque(maxlen=MAX_POINTS)
        self.base_buf = deque(maxlen=MAX_POINTS)
        self.on_line_buf = deque(maxlen=MAX_POINTS)
        self.marker_buf = deque(maxlen=MAX_POINTS)

        self._build_ui()

        self.recv_timer = QtCore.QTimer(self)
        self.recv_timer.timeout.connect(self.receive_available_packets)
        self.recv_timer.start(RECV_TIMER_MS)

        self.command_timer = QtCore.QTimer(self)
        self.command_timer.timeout.connect(self.poll_command_acks)
        self.command_timer.start(COMMAND_TIMER_MS)

        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start(PLOT_TIMER_MS)

    def _build_ui(self):
        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        self.setCentralWidget(central)

        self.status_label = QtWidgets.QLabel()
        self.status_label.setTextInteractionFlags(text_selectable_flag())
        layout.addWidget(self.status_label)

        self.graph = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graph, stretch=1)

        self.plot_position = self.graph.addPlot(row=0, col=0, title="position")
        self.plot_error = self.graph.addPlot(row=1, col=0, title="error / d_error")
        self.plot_motor = self.graph.addPlot(
            row=2, col=0, title="left_cmd / right_cmd / base_speed"
        )
        self.plot_state = self.graph.addPlot(row=3, col=0, title="on_line / is_marker")

        self.plot_position.setYRange(0, 7000)
        self.plot_error.setYRange(-3500, 3500)
        self.plot_motor.setYRange(-1100, 1100)
        self.plot_state.setYRange(-0.1, 1.2)

        for p in [
            self.plot_position,
            self.plot_error,
            self.plot_motor,
            self.plot_state,
        ]:
            p.showGrid(x=True, y=True)
            p.setLabel("bottom", "time", units="s")

        self.center_line = pg.InfiniteLine(pos=3500, angle=0, movable=False)
        self.zero_error_line = pg.InfiniteLine(pos=0, angle=0, movable=False)
        self.plot_position.addItem(self.center_line)
        self.plot_error.addItem(self.zero_error_line)

        self.curve_position = self.plot_position.plot(name="position")
        self.curve_error = self.plot_error.plot(name="error")
        self.curve_d_error = self.plot_error.plot(name="d_error")
        self.curve_left = self.plot_motor.plot(name="left_cmd")
        self.curve_right = self.plot_motor.plot(name="right_cmd")
        self.curve_base = self.plot_motor.plot(name="base_speed")
        self.curve_on_line = self.plot_state.plot(name="on_line")
        self.curve_marker = self.plot_state.plot(name="is_marker")

        self.update_status_label()

    # ------------------------------------------------------------
    # Section / profile helpers
    # ------------------------------------------------------------

    def get_section_info(self, section_id=None):
        if section_id is None:
            section_id = self.current_section_id

        return self.sections_by_id.get(
            section_id,
            default_section_info(section_id),
        )

    def get_profile_info(self, profile_key):
        return self.profiles.get(
            profile_key,
            default_profile_info(profile_key),
        )

    def get_current_profile_info(self):
        section = self.get_section_info()
        profile_key = section.get("profile_key", "UNKNOWN")
        return self.get_profile_info(profile_key)

    def get_next_section_id(self):
        if not self.section_ids:
            return (self.current_section_id + 1) & 0xFFFF

        for section_id in self.section_ids:
            if section_id > self.current_section_id:
                return section_id

        return self.current_section_id

    def current_section_text(self):
        section = self.get_section_info()

        return (
            "course={}  map_status={}  "
            "section_id={}  no={}  label={}  type={}  profile={}  role={}"
        ).format(
            self.course_map.get("course_name", "unknown"),
            self.course_map_status,
            self.current_section_id,
            section.get("display_no", ""),
            section.get("label_ko", ""),
            section.get("type", ""),
            section.get("profile_key", ""),
            section.get("role", ""),
        )

    def current_profile_text(self):
        section = self.get_section_info()
        profile_key = section.get("profile_key", "UNKNOWN")
        profile = self.get_profile_info(profile_key)

        return (
            "profiles_status={}  "
            "profile={}  label={}  base={}  curve={}  sharp={}  "
            "kp={}  kd={}  max_corr={}  reverse={}  search={}  loss_ms={}"
        ).format(
            self.profile_status,
            profile_key,
            profile.get("label_ko", ""),
            profile.get("base_speed", ""),
            profile.get("curve_speed", ""),
            profile.get("sharp_curve_speed", ""),
            profile.get("kp", ""),
            profile.get("kd", ""),
            profile.get("max_correction", ""),
            profile.get("reverse_allow", ""),
            profile.get("search_pwm", ""),
            profile.get("line_loss_max_ms", ""),
        )

    # ------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------

    def update_packet_loss(self, seq):
        if self.last_seq is None:
            self.last_seq = seq
            return

        expected = (self.last_seq + 1) & 0xFFFF
        if seq != expected:
            self.packet_loss_count += (seq - expected) & 0xFFFF

        self.last_seq = seq

    def last_packet_age_ms(self):
        if self.last_packet_time is None:
            return -1
        return int((time.monotonic() - self.last_packet_time) * 1000)

    def receive_available_packets(self):
        while True:
            try:
                data, addr = self.telemetry_sock.recvfrom(RECV_BUFFER_SIZE)
            except BlockingIOError:
                break
            except OSError:
                break

            row, reason = parse_packet(data)
            if row is None:
                self.bad_packet_count += 1
                self.last_bad_reason = reason or "unknown"
                continue

            pico_ip = addr[0]
            self.last_pico_ip = pico_ip
            self.command_sender.set_target_ip(pico_ip)

            now_wall = datetime.now().isoformat(timespec="milliseconds")
            self.last_packet_time = time.monotonic()
            self.received_count += 1
            self.update_packet_loss(row["seq"])

            t_sec = row["t_ms"] / 1000.0
            self.t_buf.append(t_sec)
            self.position_buf.append(row["position"])
            self.error_buf.append(row["error"])
            self.d_error_buf.append(row["d_error"])
            self.left_buf.append(row["left_cmd"])
            self.right_buf.append(row["right_cmd"])
            self.base_buf.append(row["base_speed"])
            self.on_line_buf.append(row["on_line"])
            self.marker_buf.append(row["is_marker"])

            csv_row = dict(row)
            csv_row["wall_time"] = now_wall
            csv_row["packet_loss_count"] = self.packet_loss_count
            csv_row["bad_packet_count"] = self.bad_packet_count
            csv_row["last_packet_age_ms"] = self.last_packet_age_ms()
            self.csv_writer.writerow(csv_row)

            if self.received_count % FLUSH_EVERY_ROWS == 0:
                self.csv_file.flush()

            self.last_row = row

    # ------------------------------------------------------------
    # Command / section mark
    # ------------------------------------------------------------

    def poll_command_acks(self):
        acks = self.command_sender.poll_acks()

        for ack in acks:
            self.handle_command_ack(ack)

        self.expire_pending_section_marks()

    def send_command(self, cmd_type):
        info = self.command_sender.send(cmd_type)
        if info is None:
            print("command not sent:", self.command_sender.last_error)
            return

        print(
            "sent command: seq={} cmd={} target={}:{}".format(
                info["seq"],
                info["cmd_name"],
                info["target_ip"],
                info["target_port"],
            )
        )

    def request_next_section(self):
        if self.last_row is None:
            print("NEXT_SECTION ignored: no telemetry yet")
            return

        if self.pending_section_marks:
            print("NEXT_SECTION ignored: waiting previous ACK")
            return

        old_section_id = self.current_section_id
        new_section_id = self.get_next_section_id()

        if new_section_id == old_section_id:
            print("NEXT_SECTION ignored: already at last section")
            return

        snapshot = dict(self.last_row)
        new_section_info = dict(self.get_section_info(new_section_id))
        profile_key = new_section_info.get("profile_key", "UNKNOWN")
        new_profile_info = dict(self.get_profile_info(profile_key))

        info = self.command_sender.send(CMD_NEXT_SECTION)
        if info is None:
            print("NEXT_SECTION not sent:", self.command_sender.last_error)
            return

        self.pending_section_marks[info["seq"]] = {
            "snapshot": snapshot,
            "old_section_id": old_section_id,
            "new_section_id": new_section_id,
            "new_section_info": new_section_info,
            "new_profile_info": new_profile_info,
            "sent_wall_time": datetime.now().isoformat(timespec="milliseconds"),
            "sent_time": time.monotonic(),
        }

        print(
            "sent NEXT_SECTION: seq={} {} -> {}  {} / {} / profile={}".format(
                info["seq"],
                old_section_id,
                new_section_id,
                new_section_info.get("label_ko", ""),
                new_section_info.get("type", ""),
                profile_key,
            )
        )

    def handle_command_ack(self, ack):
        if ack.get("cmd_type") != CMD_NEXT_SECTION:
            return

        pending = self.pending_section_marks.pop(ack["cmd_seq"], None)
        if pending is None:
            return

        ok = ack.get("status_name") == "OK"

        if ok:
            self.current_section_id = pending["new_section_id"]

        self.write_section_mark(
            pending=pending,
            ack=ack,
            ok=ok,
        )

    def expire_pending_section_marks(self):
        now = time.monotonic()
        expired_seqs = []

        for seq, pending in self.pending_section_marks.items():
            elapsed_ms = int((now - pending["sent_time"]) * 1000)
            if elapsed_ms >= SECTION_ACK_TIMEOUT_MS:
                expired_seqs.append(seq)

        for seq in expired_seqs:
            pending = self.pending_section_marks.pop(seq)

            ack = {
                "cmd_seq": seq,
                "cmd_type": CMD_NEXT_SECTION,
                "cmd_name": "NEXT_SECTION",
                "status": "",
                "status_name": "TIMEOUT",
                "rtt_ms": "",
            }

            self.write_section_mark(
                pending=pending,
                ack=ack,
                ok=False,
            )

            print("NEXT_SECTION timeout: seq={}".format(seq))

    def write_section_mark(self, pending, ack, ok):
        snapshot = pending["snapshot"]
        section = pending["new_section_info"]
        profile = pending["new_profile_info"]

        row = {
            "wall_time": datetime.now().isoformat(timespec="milliseconds"),
            "event_type": "NEXT_SECTION",
            "old_section_id": pending["old_section_id"],
            "new_section_id": pending["new_section_id"],
            "section_display_no": section.get("display_no", ""),
            "section_name": section.get("name", ""),
            "section_label_ko": section.get("label_ko", ""),
            "section_type": section.get("type", ""),
            "profile_key": section.get("profile_key", ""),
            "section_role": section.get("role", ""),
            "section_note": section.get("note", ""),

            "profile_label_ko": profile.get("label_ko", ""),
            "profile_base_speed": profile.get("base_speed", ""),
            "profile_curve_speed": profile.get("curve_speed", ""),
            "profile_sharp_curve_speed": profile.get("sharp_curve_speed", ""),
            "profile_min_run_speed": profile.get("min_run_speed", ""),
            "profile_kp": profile.get("kp", ""),
            "profile_kd": profile.get("kd", ""),
            "profile_max_correction": profile.get("max_correction", ""),
            "profile_reverse_allow": profile.get("reverse_allow", ""),
            "profile_reverse_pwm_mid": profile.get("reverse_pwm_mid", ""),
            "profile_reverse_pwm_high": profile.get("reverse_pwm_high", ""),
            "profile_error_curve_threshold": profile.get("error_curve_threshold", ""),
            "profile_error_sharp_threshold": profile.get("error_sharp_threshold", ""),
            "profile_d_error_curve_threshold": profile.get("d_error_curve_threshold", ""),
            "profile_d_error_sharp_threshold": profile.get("d_error_sharp_threshold", ""),
            "profile_search_pwm": profile.get("search_pwm", ""),
            "profile_line_loss_max_ms": profile.get("line_loss_max_ms", ""),

            "cmd_seq": ack.get("cmd_seq", ""),
            "ack": 1 if ok else 0,
            "status": ack.get("status_name", ""),
            "rtt_ms": ack.get("rtt_ms", ""),
            "seq": snapshot.get("seq", ""),
            "t_ms": snapshot.get("t_ms", ""),
            "base_speed": snapshot.get("base_speed", ""),
            "position": snapshot.get("position", ""),
            "error": snapshot.get("error", ""),
            "d_error": snapshot.get("d_error", ""),
            "left_cmd": snapshot.get("left_cmd", ""),
            "right_cmd": snapshot.get("right_cmd", ""),
            "on_line": snapshot.get("on_line", ""),
            "is_marker": snapshot.get("is_marker", ""),
            "packet_loss_count": self.packet_loss_count,
        }

        self.section_csv_writer.writerow(row)
        self.section_csv_file.flush()

        print(
            "section mark: {} -> {}  {} / {}  profile={} base={} kp={} kd={} ack={} status={} rtt={}ms".format(
                row["old_section_id"],
                row["new_section_id"],
                row["section_label_ko"],
                row["section_type"],
                row["profile_key"],
                row["profile_base_speed"],
                row["profile_kp"],
                row["profile_kd"],
                row["ack"],
                row["status"],
                row["rtt_ms"],
            )
        )

    # ------------------------------------------------------------
    # Key input
    # ------------------------------------------------------------

    def keyPressEvent(self, event):
        key = event.key()

        if key == KEY_P:
            self.send_command(CMD_PING)
            return

        if key == KEY_SPACE:
            self.request_next_section()
            return

        if key == KEY_Z:
            self.send_command(CMD_STOP)
            return

        if key == KEY_RETURN or key == KEY_ENTER:
            self.send_command(CMD_RUN)
            return

        super().keyPressEvent(event)

    # ------------------------------------------------------------
    # UI update
    # ------------------------------------------------------------

    def update_status_label(self):
        if self.received_count > 0:
            denom = self.received_count + self.packet_loss_count
            loss_rate = self.packet_loss_count * 100.0 / denom if denom > 0 else 0.0
        else:
            loss_rate = 0.0

        if self.last_row is None:
            current = "no telemetry packet yet"
        else:
            current = (
                "seq={seq}  t_ms={t_ms}  control_ms={control_ms}  send_ms={send_ms}  "
                "base={base_speed}  pos={position}  err={error}  d_err={d_error}  "
                "left={left_cmd}  right={right_cmd}  on_line={on_line}  marker={is_marker}"
            ).format(**self.last_row)

        command_help = "keys: P=PING  Space=NEXT_SECTION  Z=EMERGENCY_STOP  Enter=RUN"

        command_status = "{}  pending_section_marks={}".format(
            self.command_sender.stats_text(),
            len(self.pending_section_marks),
        )

        section_status = self.current_section_text()
        profile_status = self.current_profile_text()

        self.status_label.setText(
            "telemetry_listen={}:{}  packet_size={}  version={}  "
            "received={}  lost={} ({:.2f}%)  bad={}  last_age={}ms  bad_reason={}\n"
            "{}\n"
            "{}\n"
            "{}\n"
            "{}\n"
            "CSV={}\n"
            "SECTION_CSV={}\n"
            "{}".format(
                LISTEN_IP,
                LISTEN_PORT,
                PACKET_SIZE,
                VERSION,
                self.received_count,
                self.packet_loss_count,
                loss_rate,
                self.bad_packet_count,
                self.last_packet_age_ms(),
                self.last_bad_reason,
                command_status,
                section_status,
                profile_status,
                command_help,
                self.csv_path,
                self.section_csv_path,
                current,
            )
        )

    def update_plots(self):
        t = list(self.t_buf)
        if not t:
            self.update_status_label()
            return

        self.curve_position.setData(t, list(self.position_buf))
        self.curve_error.setData(t, list(self.error_buf))
        self.curve_d_error.setData(t, list(self.d_error_buf))
        self.curve_left.setData(t, list(self.left_buf))
        self.curve_right.setData(t, list(self.right_buf))
        self.curve_base.setData(t, list(self.base_buf))
        self.curve_on_line.setData(t, list(self.on_line_buf))
        self.curve_marker.setData(t, list(self.marker_buf))

        if len(t) >= 2:
            x_min = t[0]
            x_max = t[-1]
            if x_max <= x_min:
                x_max = x_min + 1.0
            for p in [
                self.plot_position,
                self.plot_error,
                self.plot_motor,
                self.plot_state,
            ]:
                p.setXRange(x_min, x_max, padding=0)

        self.update_status_label()

    # ------------------------------------------------------------
    # Close
    # ------------------------------------------------------------

    def closeEvent(self, event):
        self.close_resources()
        event.accept()

    def close_resources(self):
        if self.closed:
            return
        self.closed = True

        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass

        try:
            self.section_csv_file.flush()
            self.section_csv_file.close()
        except Exception:
            pass

        try:
            self.telemetry_sock.close()
        except Exception:
            pass

        try:
            self.command_sender.close()
        except Exception:
            pass

        print("CSV saved:", self.csv_path)
        print("section CSV saved:", self.section_csv_path)
        print("received:", self.received_count)
        print("lost:", self.packet_loss_count)
        print("bad:", self.bad_packet_count)


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = Dashboard()
    window.show()
    code = run_qt_app(app)
    window.close_resources()
    sys.exit(code)


if __name__ == "__main__":
    main()
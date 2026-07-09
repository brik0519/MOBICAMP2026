# app.py
# PAI-Car PyQtGraph telemetry V2 dashboard
#
# 목적:
#   - Pico 2 W telemetry V2 packet 수신
#   - Pico 실제 section/profile/command 상태 표시
#   - ACK TIMEOUT이 있어도 telemetry actual_section_id로 PC section 동기화
#   - profiles.json의 profile 값을 각 telemetry row에 snapshot으로 기록
#   - Space: NEXT_SECTION, Z: STOP, Enter: RUN, P: PING
#   - 주행 CSV와 section mark CSV 저장
#
# 실행:
#   python app.py
#
# 필요 패키지:
#   python -m pip install pyqtgraph PyQt6

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


# ------------------------------------------------------------
# Paths
# ------------------------------------------------------------

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(BASE_DIR, "logs")
COURSE_MAP_PATH = os.path.join(BASE_DIR, "course_map.json")
PROFILES_PATH = os.path.join(BASE_DIR, "profiles.json")


# ------------------------------------------------------------
# UDP receive settings
# ------------------------------------------------------------

LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 5005
RECV_BUFFER_SIZE = 2048


# ------------------------------------------------------------
# Telemetry protocol
# ------------------------------------------------------------

MAGIC = 0x5041

VERSION_V1 = 1
PACKET_FORMAT_V1 = "<HBBHIHHh8HHhhhhBB"
PACKET_SIZE_V1 = struct.calcsize(PACKET_FORMAT_V1)

VERSION_V2 = 2
PACKET_FORMAT_V2 = "<HBBHIHHh8HHhhhhBBBBBHBB"
PACKET_SIZE_V2 = struct.calcsize(PACKET_FORMAT_V2)

PROFILE_ID_TO_KEY = {
    0: "SAFE",
    1: "STRAIGHT",
    2: "WIDE_S",
    3: "NARROW_S",
    4: "HAIRPIN_U",
    5: "WIDE_U",
    255: "UNKNOWN",
}

RUN_STATE_NAMES = {
    0: "STOP",
    1: "RUN",
    255: "UNKNOWN",
}


# ------------------------------------------------------------
# Command protocol: PC -> Pico
# ------------------------------------------------------------

COMMAND_MAGIC = 0x5043
ACK_MAGIC = 0x4341
COMMAND_VERSION = 1

COMMAND_FORMAT = "<HBBHBBhi"
COMMAND_SIZE = struct.calcsize(COMMAND_FORMAT)

ACK_FORMAT = "<HBBHBB"
ACK_SIZE = struct.calcsize(ACK_FORMAT)

COMMAND_PORT = 5006
COMMAND_ACK_TIMEOUT_MS = 450

CMD_PING = 1
CMD_STOP = 2
CMD_SAFE_MODE = 3
CMD_RUN = 4
CMD_NEXT_SECTION = 5

CMD_NAMES = {
    CMD_PING: "PING",
    CMD_STOP: "STOP",
    CMD_SAFE_MODE: "SAFE_MODE",
    CMD_RUN: "RUN",
    CMD_NEXT_SECTION: "NEXT_SECTION",
}

STATUS_NAMES = {
    0: "OK",
    1: "BAD_MAGIC",
    2: "BAD_VERSION",
    3: "BAD_SIZE",
    4: "UNKNOWN_CMD",
    5: "ERROR",
}

NEXT_SECTION_MIN_INTERVAL_MS = 250


# ------------------------------------------------------------
# Dashboard settings
# ------------------------------------------------------------

MAX_POINTS = 1500
RECV_TIMER_MS = 5
PLOT_TIMER_MS = 33
COMMAND_TIMER_MS = 20
FLUSH_EVERY_ROWS = 20


# ------------------------------------------------------------
# CSV headers
# ------------------------------------------------------------

CSV_HEADER = [
    "wall_time",
    "telemetry_version",
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
    "run_state",
    "run_state_name",
    "actual_section_id",
    "active_profile_id",
    "active_profile_key",
    "last_cmd_seq",
    "last_cmd_type",
    "last_cmd_type_name",
    "last_cmd_status",
    "last_cmd_status_name",
    "pc_section_id",
    "pc_section_label",
    "pc_profile_key",
    "section_mismatch",
    "packet_loss_count",
    "bad_packet_count",
    "last_packet_age_ms",
]

SECTION_MARK_HEADER = [
    "wall_time",
    "event",
    "cmd_seq",
    "cmd_type",
    "cmd_type_name",
    "cmd_status",
    "cmd_status_name",
    "ack_rtt_ms",
    "pc_from_section_id",
    "pc_from_label",
    "pc_from_profile_key",
    "pc_to_section_id",
    "pc_to_label",
    "pc_to_profile_key",
    "pc_section_id_after",
    "pico_actual_section_id",
    "pico_active_profile_id",
    "pico_active_profile_key",
    "pico_last_cmd_seq",
    "pico_last_cmd_type",
    "pico_last_cmd_status",
    "t_ms",
    "seq",
    "base_speed",
    "position",
    "error",
    "d_error",
    "left_cmd",
    "right_cmd",
    "on_line",
    "is_marker",
    "packet_loss_count",
    "bad_packet_count",
]

PROFILE_SNAPSHOT_HEADER = [
    "profile_set",
    "profile_version",
    "profile_source_key",
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
]

CSV_HEADER += PROFILE_SNAPSHOT_HEADER
SECTION_MARK_HEADER += PROFILE_SNAPSHOT_HEADER


# ------------------------------------------------------------
# Qt key compatibility
# ------------------------------------------------------------

def get_qt_key(name):
    if hasattr(QtCore.Qt, "Key"):
        return getattr(QtCore.Qt.Key, name)

    return getattr(QtCore.Qt, name)


KEY_P = get_qt_key("Key_P")
KEY_SPACE = get_qt_key("Key_Space")
KEY_Z = get_qt_key("Key_Z")
KEY_RETURN = get_qt_key("Key_Return")
KEY_ENTER = get_qt_key("Key_Enter")


def set_text_selectable(label):
    try:
        label.setTextInteractionFlags(
            QtCore.Qt.TextInteractionFlag.TextSelectableByMouse
        )
    except Exception:
        label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)


# ------------------------------------------------------------
# Course/profile loading
# ------------------------------------------------------------

FALLBACK_SECTIONS = [
    {
        "section_id": 0,
        "display_no": 1,
        "name": "start_zone",
        "label_ko": "시작구간",
        "type": "WIDE_S",
        "profile_key": "WIDE_S",
        "role": "START",
    },
    {
        "section_id": 1,
        "display_no": 2,
        "name": "long_straight",
        "label_ko": "긴 직진 구간",
        "type": "STRAIGHT",
        "profile_key": "STRAIGHT",
        "role": "NORMAL",
    },
    {
        "section_id": 2,
        "display_no": 3,
        "name": "short_s_1",
        "label_ko": "짧은 S자",
        "type": "NARROW_S",
        "profile_key": "NARROW_S",
        "role": "NORMAL",
    },
    {
        "section_id": 3,
        "display_no": 4,
        "name": "short_straight_1",
        "label_ko": "짧은 직진",
        "type": "STRAIGHT",
        "profile_key": "STRAIGHT",
        "role": "NORMAL",
    },
    {
        "section_id": 4,
        "display_no": 5,
        "name": "wide_s_1",
        "label_ko": "넓은 S자",
        "type": "WIDE_S",
        "profile_key": "WIDE_S",
        "role": "NORMAL",
    },
    {
        "section_id": 5,
        "display_no": 6,
        "name": "narrow_s_1",
        "label_ko": "좁은 S자",
        "type": "NARROW_S",
        "profile_key": "NARROW_S",
        "role": "NORMAL",
    },
    {
        "section_id": 6,
        "display_no": 7,
        "name": "hairpin_entry_and_u",
        "label_ko": "헤어핀 진입+헤어핀",
        "type": "HAIRPIN_U",
        "profile_key": "HAIRPIN_U",
        "role": "NORMAL",
    },
    {
        "section_id": 7,
        "display_no": 8,
        "name": "middle_straight",
        "label_ko": "중간 직진구간",
        "type": "STRAIGHT",
        "profile_key": "STRAIGHT",
        "role": "NORMAL",
    },
    {
        "section_id": 8,
        "display_no": 9,
        "name": "wide_u_1",
        "label_ko": "완만한 U턴",
        "type": "WIDE_U",
        "profile_key": "WIDE_U",
        "role": "NORMAL",
    },
    {
        "section_id": 9,
        "display_no": 10,
        "name": "short_straight_2",
        "label_ko": "짧은 직진",
        "type": "STRAIGHT",
        "profile_key": "STRAIGHT",
        "role": "NORMAL",
    },
    {
        "section_id": 10,
        "display_no": 11,
        "name": "half_narrow_s",
        "label_ko": "좁은 S자 절반",
        "type": "NARROW_S",
        "profile_key": "NARROW_S",
        "role": "NORMAL",
    },
    {
        "section_id": 11,
        "display_no": 12,
        "name": "finish_short_straight",
        "label_ko": "Finish 전 짧은 직진",
        "type": "STRAIGHT",
        "profile_key": "STRAIGHT",
        "role": "FINISH_APPROACH",
    },
]


def load_json_file(path, default_value):
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)

    except Exception:
        return default_value


class CourseModel:
    def __init__(self):
        course = load_json_file(COURSE_MAP_PATH, {})
        profiles_doc = load_json_file(PROFILES_PATH, {})

        self.course_name = course.get("course_name", "unknown_course")
        self.sections = course.get("sections") or FALLBACK_SECTIONS

        self.profile_set = profiles_doc.get("profile_set", "")
        self.profile_version = profiles_doc.get("version", "")
        self.profiles = profiles_doc.get("profiles", {})

        self.sections_by_id = {}

        for section in self.sections:
            section_id = int(section.get("section_id", len(self.sections_by_id)))
            self.sections_by_id[section_id] = section

        self.max_section_id = max(self.sections_by_id.keys()) if self.sections_by_id else 0

    def clamp_section_id(self, section_id):
        try:
            section_id = int(section_id)
        except Exception:
            section_id = 0

        if section_id < 0:
            return 0

        if section_id > self.max_section_id:
            return self.max_section_id

        return section_id

    def get_section(self, section_id):
        section_id = self.clamp_section_id(section_id)

        return self.sections_by_id.get(section_id, FALLBACK_SECTIONS[0])

    def get_label(self, section_id):
        return self.get_section(section_id).get("label_ko", "")

    def get_profile_key(self, section_id):
        return self.get_section(section_id).get("profile_key", "")

    def get_profile(self, profile_key):
        return self.profiles.get(profile_key, {})

    def get_profile_snapshot(self, profile_key):
        profile = self.get_profile(profile_key)

        return {
            "profile_set": self.profile_set,
            "profile_version": self.profile_version,
            "profile_source_key": profile_key,
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
        }


# ------------------------------------------------------------
# Packet parsing
# ------------------------------------------------------------

def parse_packet(data: bytes):
    if len(data) == PACKET_SIZE_V2:
        packet_format = PACKET_FORMAT_V2
        expected_version = VERSION_V2

    elif len(data) == PACKET_SIZE_V1:
        packet_format = PACKET_FORMAT_V1
        expected_version = VERSION_V1

    else:
        return None, "bad_size:{}".format(len(data))

    try:
        values = struct.unpack(packet_format, data)

    except struct.error:
        return None, "struct_error"

    magic = values[0]
    version = values[1]

    if magic != MAGIC:
        return None, "bad_magic"

    if version != expected_version:
        return None, "bad_version:{}".format(version)

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

    if version == VERSION_V2:
        run_state = values[23]
        actual_section_id = values[24]
        active_profile_id = values[25]
        last_cmd_seq = values[26]
        last_cmd_type = values[27]
        last_cmd_status = values[28]

    else:
        run_state = 255
        actual_section_id = 255
        active_profile_id = 255
        last_cmd_seq = 0
        last_cmd_type = 0
        last_cmd_status = 0

    active_profile_key = PROFILE_ID_TO_KEY.get(active_profile_id, "UNKNOWN")

    row = {
        "telemetry_version": version,
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
        "run_state": run_state,
        "run_state_name": RUN_STATE_NAMES.get(run_state, str(run_state)),
        "actual_section_id": actual_section_id,
        "active_profile_id": active_profile_id,
        "active_profile_key": active_profile_key,
        "last_cmd_seq": last_cmd_seq,
        "last_cmd_type": last_cmd_type,
        "last_cmd_type_name": CMD_NAMES.get(last_cmd_type, str(last_cmd_type)),
        "last_cmd_status": last_cmd_status,
        "last_cmd_status_name": STATUS_NAMES.get(last_cmd_status, str(last_cmd_status)),
    }

    return row, None


# ------------------------------------------------------------
# Command sender
# ------------------------------------------------------------

class CommandSender:
    def __init__(self, target_port=COMMAND_PORT):
        self.target_ip = None
        self.target_port = target_port

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

        self.cmd_seq = 0
        self.sent_count = 0
        self.ack_count = 0
        self.bad_ack_count = 0
        self.timeout_count = 0

        self.pending = {}

        self.last_sent_text = "none"
        self.last_ack_text = "none"
        self.last_error_text = "none"
        self.last_rtt_ms = -1

    def set_target_ip(self, ip):
        if ip:
            self.target_ip = ip

    def send(self, cmd_type, target_id=0, param_id=0, value=0, meta=None):
        if not self.target_ip:
            self.last_error_text = "NO_TARGET"
            return None

        self.cmd_seq = (self.cmd_seq + 1) & 0xFFFF

        try:
            packet = struct.pack(
                COMMAND_FORMAT,
                COMMAND_MAGIC,
                COMMAND_VERSION,
                COMMAND_SIZE,
                self.cmd_seq,
                int(cmd_type) & 0xFF,
                int(target_id) & 0xFF,
                int(param_id),
                int(value),
            )

        except struct.error as exc:
            self.last_error_text = "PACK_ERROR {}".format(exc)
            return None

        try:
            self.sock.sendto(packet, (self.target_ip, self.target_port))

        except OSError as exc:
            self.last_error_text = "SOCKET_ERROR {}".format(exc)
            return None

        now = time.monotonic()

        info = {
            "cmd_seq": self.cmd_seq,
            "cmd_type": cmd_type,
            "target_id": target_id,
            "param_id": param_id,
            "value": value,
            "sent_time": now,
            "meta": meta or {},
        }

        self.pending[self.cmd_seq] = info

        self.sent_count += 1
        self.last_rtt_ms = -1
        self.last_sent_text = "{} seq={} value={}".format(
            CMD_NAMES.get(cmd_type, str(cmd_type)),
            self.cmd_seq,
            value,
        )
        self.last_error_text = ""

        return info

    def send_ping(self):
        return self.send(CMD_PING)

    def send_stop(self):
        return self.send(CMD_STOP)

    def send_run(self):
        return self.send(CMD_RUN)

    def send_next_section(self, from_section_id, to_section_id, snapshot_row):
        meta = {
            "from_section_id": from_section_id,
            "to_section_id": to_section_id,
            "snapshot_row": dict(snapshot_row) if snapshot_row else {},
            "event": "NEXT_SECTION",
        }

        return self.send(CMD_NEXT_SECTION, value=to_section_id, meta=meta)

    def receive_acks(self):
        acks = []

        while True:
            try:
                data, addr = self.sock.recvfrom(128)

            except BlockingIOError:
                break

            except OSError:
                break

            ack = self._parse_ack(data)

            if ack is None:
                self.bad_ack_count += 1
                continue

            ack["addr"] = addr
            self.ack_count += 1

            pending_info = self.pending.pop(ack["cmd_seq"], None)
            ack["pending"] = pending_info

            if pending_info is not None:
                self.last_rtt_ms = int(
                    (time.monotonic() - pending_info["sent_time"]) * 1000
                )

            else:
                self.last_rtt_ms = -1

            ack["rtt_ms"] = self.last_rtt_ms

            self.last_ack_text = "{} seq={} status={} rtt={}ms".format(
                CMD_NAMES.get(ack["cmd_type"], str(ack["cmd_type"])),
                ack["cmd_seq"],
                STATUS_NAMES.get(ack["status"], str(ack["status"])),
                self.last_rtt_ms,
            )

            acks.append(ack)

        return acks

    def check_timeouts(self):
        now = time.monotonic()
        timed_out = []

        for cmd_seq, info in list(self.pending.items()):
            elapsed_ms = int((now - info["sent_time"]) * 1000)

            if elapsed_ms >= COMMAND_ACK_TIMEOUT_MS:
                self.pending.pop(cmd_seq, None)
                self.timeout_count += 1

                timeout_info = {
                    "cmd_seq": info["cmd_seq"],
                    "cmd_type": info["cmd_type"],
                    "status": "TIMEOUT",
                    "rtt_ms": elapsed_ms,
                    "pending": info,
                }

                self.last_ack_text = "{} seq={} status=TIMEOUT elapsed={}ms".format(
                    CMD_NAMES.get(info["cmd_type"], str(info["cmd_type"])),
                    info["cmd_seq"],
                    elapsed_ms,
                )

                timed_out.append(timeout_info)

        return timed_out

    def _parse_ack(self, data):
        if len(data) != ACK_SIZE:
            return None

        try:
            magic, version, packet_size, cmd_seq, cmd_type, status = struct.unpack(
                ACK_FORMAT,
                data,
            )

        except struct.error:
            return None

        if magic != ACK_MAGIC:
            return None

        if version != COMMAND_VERSION:
            return None

        if packet_size != ACK_SIZE:
            return None

        return {
            "cmd_seq": int(cmd_seq) & 0xFFFF,
            "cmd_type": int(cmd_type) & 0xFF,
            "status": int(status) & 0xFF,
        }

    def has_pending_next_section(self):
        for info in self.pending.values():
            if info.get("cmd_type") == CMD_NEXT_SECTION:
                return True

        return False

    def close(self):
        try:
            self.sock.close()

        except OSError:
            pass


# ------------------------------------------------------------
# Main window
# ------------------------------------------------------------

class Dashboard(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.course = CourseModel()

        self.setWindowTitle("PAI-Car Telemetry V2 Dashboard")
        self.resize(1280, 900)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((LISTEN_IP, LISTEN_PORT))
        self.sock.setblocking(False)

        self.command = CommandSender()

        os.makedirs(LOG_DIR, exist_ok=True)
        now_name = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.csv_path = os.path.join(
            LOG_DIR,
            "pai_car_pyqt_{}.csv".format(now_name),
        )
        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_HEADER)
        self.csv_writer.writeheader()

        self.section_marks_path = os.path.join(
            LOG_DIR,
            "section_marks_{}.csv".format(now_name),
        )
        self.section_marks_file = open(
            self.section_marks_path,
            "w",
            newline="",
            encoding="utf-8",
        )
        self.section_marks_writer = csv.DictWriter(
            self.section_marks_file,
            fieldnames=SECTION_MARK_HEADER,
        )
        self.section_marks_writer.writeheader()

        self.received_count = 0
        self.bad_packet_count = 0
        self.packet_loss_count = 0
        self.sync_fix_count = 0

        self.last_seq = None
        self.last_packet_time = None
        self.last_row = None
        self.last_bad_reason = ""
        self.last_pico_ip = ""
        self.last_sync_text = "none"
        self.closed = False

        self.pc_section_id = 0
        self.last_next_section_time = 0.0

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

        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start(PLOT_TIMER_MS)

        self.command_timer = QtCore.QTimer(self)
        self.command_timer.timeout.connect(self.receive_command_acks)
        self.command_timer.start(COMMAND_TIMER_MS)

    def _build_ui(self):
        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        self.setCentralWidget(central)

        self.status_label = QtWidgets.QLabel()
        set_text_selectable(self.status_label)
        layout.addWidget(self.status_label)

        self.graph = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graph, stretch=1)

        self.plot_position = self.graph.addPlot(row=0, col=0, title="position")
        self.plot_error = self.graph.addPlot(row=1, col=0, title="error / d_error")
        self.plot_motor = self.graph.addPlot(
            row=2,
            col=0,
            title="left_cmd / right_cmd / base_speed",
        )
        self.plot_state = self.graph.addPlot(row=3, col=0, title="on_line / is_marker")

        self.plot_position.setYRange(0, 7000)
        self.plot_error.setYRange(-3500, 3500)
        self.plot_motor.setYRange(-1100, 1100)
        self.plot_state.setYRange(-0.1, 1.2)

        for plot in [
            self.plot_position,
            self.plot_error,
            self.plot_motor,
            self.plot_state,
        ]:
            plot.showGrid(x=True, y=True)
            plot.setLabel("bottom", "time", units="s")

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
                data, addr = self.sock.recvfrom(RECV_BUFFER_SIZE)

            except BlockingIOError:
                break

            except OSError:
                break

            row, reason = parse_packet(data)

            if row is None:
                self.bad_packet_count += 1
                self.last_bad_reason = reason or "unknown"
                continue

            if addr and addr[0] != self.last_pico_ip:
                self.last_pico_ip = addr[0]
                self.command.set_target_ip(self.last_pico_ip)

            now_wall = datetime.now().isoformat(timespec="milliseconds")

            self.last_packet_time = time.monotonic()
            self.received_count += 1

            self.update_packet_loss(row["seq"])
            self.sync_pc_section_from_pico(row)

            section = self.course.get_section(self.pc_section_id)

            row["pc_section_id"] = self.pc_section_id
            row["pc_section_label"] = section.get("label_ko", "")
            row["pc_profile_key"] = section.get("profile_key", "")
            row["section_mismatch"] = self.is_section_mismatch(row)

            # profiles.json 기준 profile snapshot을 telemetry row에 함께 저장한다.
            # 우선 Pico가 보내준 active_profile_key를 사용하고,
            # V1 packet 또는 UNKNOWN이면 PC section의 profile_key로 fallback한다.
            profile_key_for_snapshot = row.get("active_profile_key", "")

            if profile_key_for_snapshot in ("", "UNKNOWN"):
                profile_key_for_snapshot = row["pc_profile_key"]

            row.update(
                self.course.get_profile_snapshot(profile_key_for_snapshot)
            )

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

            self.csv_writer.writerow(self.filter_csv_row(csv_row, CSV_HEADER))

            if self.received_count % FLUSH_EVERY_ROWS == 0:
                self.csv_file.flush()
                self.section_marks_file.flush()

            self.last_row = row

    def filter_csv_row(self, row, header):
        return {key: row.get(key, "") for key in header}

    def is_section_mismatch(self, row):
        if row.get("telemetry_version") != VERSION_V2:
            return 0

        actual_section_id = row.get("actual_section_id", 255)

        if actual_section_id == 255:
            return 0

        return 1 if int(actual_section_id) != int(self.pc_section_id) else 0

    def sync_pc_section_from_pico(self, row):
        if row.get("telemetry_version") != VERSION_V2:
            return

        actual_section_id = row.get("actual_section_id", 255)

        if actual_section_id == 255:
            return

        actual_section_id = self.course.clamp_section_id(actual_section_id)

        if actual_section_id != self.pc_section_id:
            old_id = self.pc_section_id
            self.pc_section_id = actual_section_id
            self.sync_fix_count += 1
            self.last_sync_text = "PC section sync {} -> {} by telemetry".format(
                old_id,
                actual_section_id,
            )

    def receive_command_acks(self):
        acks = self.command.receive_acks()

        for ack in acks:
            self.handle_command_result(ack, is_timeout=False)

        timeouts = self.command.check_timeouts()

        for timeout_info in timeouts:
            self.handle_command_result(timeout_info, is_timeout=True)

        if acks or timeouts:
            self.update_status_label()

    def handle_command_result(self, result, is_timeout=False):
        pending = result.get("pending")

        if not pending:
            return

        cmd_type = int(result.get("cmd_type", pending.get("cmd_type", 0)))
        status = result.get("status")

        if cmd_type == CMD_NEXT_SECTION:
            meta = pending.get("meta", {})
            from_id = self.course.clamp_section_id(
                meta.get("from_section_id", self.pc_section_id)
            )
            to_id = self.course.clamp_section_id(
                meta.get("to_section_id", from_id)
            )

            if not is_timeout and int(status) == 0:
                # telemetry가 이미 actual_section_id로 보정했다면 여기서 다시 증가시키지 않는다.
                if self.pc_section_id == from_id:
                    self.pc_section_id = to_id

            self.write_section_mark(result, is_timeout=is_timeout)

    def write_section_mark(self, result, is_timeout=False):
        pending = result.get("pending") or {}
        meta = pending.get("meta", {})
        snapshot = meta.get("snapshot_row") or self.last_row or {}

        from_id = self.course.clamp_section_id(
            meta.get("from_section_id", self.pc_section_id)
        )
        to_id = self.course.clamp_section_id(
            meta.get("to_section_id", from_id)
        )

        from_section = self.course.get_section(from_id)
        to_section = self.course.get_section(to_id)

        if is_timeout:
            status_value = "TIMEOUT"
            status_name = "TIMEOUT"

        else:
            status_value = int(result.get("status", -1))
            status_name = STATUS_NAMES.get(status_value, str(status_value))

        row = {
            "wall_time": datetime.now().isoformat(timespec="milliseconds"),
            "event": meta.get("event", CMD_NAMES.get(result.get("cmd_type", 0), "CMD")),
            "cmd_seq": result.get("cmd_seq", pending.get("cmd_seq", "")),
            "cmd_type": result.get("cmd_type", pending.get("cmd_type", "")),
            "cmd_type_name": CMD_NAMES.get(
                result.get("cmd_type", pending.get("cmd_type", 0)),
                "",
            ),
            "cmd_status": status_value,
            "cmd_status_name": status_name,
            "ack_rtt_ms": result.get("rtt_ms", ""),
            "pc_from_section_id": from_id,
            "pc_from_label": from_section.get("label_ko", ""),
            "pc_from_profile_key": from_section.get("profile_key", ""),
            "pc_to_section_id": to_id,
            "pc_to_label": to_section.get("label_ko", ""),
            "pc_to_profile_key": to_section.get("profile_key", ""),
            "pc_section_id_after": self.pc_section_id,
            "pico_actual_section_id": snapshot.get("actual_section_id", ""),
            "pico_active_profile_id": snapshot.get("active_profile_id", ""),
            "pico_active_profile_key": snapshot.get("active_profile_key", ""),
            "pico_last_cmd_seq": snapshot.get("last_cmd_seq", ""),
            "pico_last_cmd_type": snapshot.get("last_cmd_type", ""),
            "pico_last_cmd_status": snapshot.get("last_cmd_status", ""),
            "t_ms": snapshot.get("t_ms", ""),
            "seq": snapshot.get("seq", ""),
            "base_speed": snapshot.get("base_speed", ""),
            "position": snapshot.get("position", ""),
            "error": snapshot.get("error", ""),
            "d_error": snapshot.get("d_error", ""),
            "left_cmd": snapshot.get("left_cmd", ""),
            "right_cmd": snapshot.get("right_cmd", ""),
            "on_line": snapshot.get("on_line", ""),
            "is_marker": snapshot.get("is_marker", ""),
            "packet_loss_count": self.packet_loss_count,
            "bad_packet_count": self.bad_packet_count,
        }

        for key in PROFILE_SNAPSHOT_HEADER:
            row[key] = snapshot.get(key, "")

        self.section_marks_writer.writerow(
            self.filter_csv_row(row, SECTION_MARK_HEADER)
        )
        self.section_marks_file.flush()

    def keyPressEvent(self, event):
        try:
            if event.isAutoRepeat():
                return

        except Exception:
            pass

        key = event.key()

        if key == KEY_P:
            self.command.send_ping()

        elif key == KEY_SPACE:
            self.handle_next_section_key()

        elif key == KEY_Z:
            self.command.send_stop()

        elif key in (KEY_RETURN, KEY_ENTER):
            self.command.send_run()

        else:
            super().keyPressEvent(event)
            return

        self.update_status_label()

    def handle_next_section_key(self):
        now = time.monotonic()
        elapsed_ms = int((now - self.last_next_section_time) * 1000)

        if elapsed_ms < NEXT_SECTION_MIN_INTERVAL_MS:
            self.command.last_error_text = (
                "NEXT_SECTION ignored: debounce {}ms".format(elapsed_ms)
            )
            return

        if self.command.has_pending_next_section():
            self.command.last_error_text = "NEXT_SECTION ignored: pending ACK"
            return

        from_id = self.course.clamp_section_id(self.pc_section_id)
        to_id = self.course.clamp_section_id(from_id + 1)

        if to_id == from_id:
            self.command.last_error_text = "NEXT_SECTION ignored: already last section"
            return

        result = self.command.send_next_section(
            from_id,
            to_id,
            self.last_row,
        )

        if result is not None:
            self.last_next_section_time = now

    def update_status_label(self):
        if self.received_count > 0:
            denom = self.received_count + self.packet_loss_count
            loss_rate = self.packet_loss_count * 100.0 / denom if denom > 0 else 0.0

        else:
            loss_rate = 0.0

        pc_section = self.course.get_section(self.pc_section_id)

        pc_section_text = "PC section={} no={} label={} profile={}".format(
            self.pc_section_id,
            pc_section.get("display_no", ""),
            pc_section.get("label_ko", ""),
            pc_section.get("profile_key", ""),
        )

        if self.last_row is None:
            current = "no packet yet"
            pico_section_text = "Pico section=unknown"
            mismatch_text = ""

        else:
            pico_section_text = (
                "Pico actual_section={actual_section_id} profile={active_profile_key} "
                "run_state={run_state_name} last_cmd={last_cmd_type_name} "
                "seq={last_cmd_seq} status={last_cmd_status_name} "
                "profile_set={profile_set} kp={profile_kp} kd={profile_kd}"
            ).format(**self.last_row)

            mismatch = self.is_section_mismatch(self.last_row)
            mismatch_text = "SECTION MISMATCH" if mismatch else "section sync OK"

            current = (
                "seq={seq}  t_ms={t_ms}  base={base_speed}  pos={position}  "
                "err={error}  d_err={d_error}  left={left_cmd}  right={right_cmd}  "
                "on_line={on_line}  marker={is_marker}"
            ).format(**self.last_row)

        cmd_target = "{}:{}".format(
            self.command.target_ip if self.command.target_ip else "unknown",
            self.command.target_port,
        )

        self.status_label.setText(
            "listen={}:{}  packet_size_v2={}  received={}  lost={} ({:.2f}%)  "
            "bad={}  last_age={}ms  bad_reason={}\n"
            "{}\n{}\n{}  sync_fix_count={}  last_sync={}\n"
            "command_target={}  sent={}  ack={}  timeout={}  bad_ack={}  "
            "last_sent={}  last_ack={}  last_error={}\n"
            "keys: P=PING  Space=NEXT_SECTION  Z=STOP  Enter=RUN\n"
            "CSV={}\nSECTION_CSV={}\n{}".format(
                LISTEN_IP,
                LISTEN_PORT,
                PACKET_SIZE_V2,
                self.received_count,
                self.packet_loss_count,
                loss_rate,
                self.bad_packet_count,
                self.last_packet_age_ms(),
                self.last_bad_reason,
                pc_section_text,
                pico_section_text,
                mismatch_text,
                self.sync_fix_count,
                self.last_sync_text,
                cmd_target,
                self.command.sent_count,
                self.command.ack_count,
                self.command.timeout_count,
                self.command.bad_ack_count,
                self.command.last_sent_text,
                self.command.last_ack_text,
                self.command.last_error_text,
                self.csv_path,
                self.section_marks_path,
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

            for plot in [
                self.plot_position,
                self.plot_error,
                self.plot_motor,
                self.plot_state,
            ]:
                plot.setXRange(x_min, x_max, padding=0)

        self.update_status_label()

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
            self.section_marks_file.flush()
            self.section_marks_file.close()

        except Exception:
            pass

        try:
            self.sock.close()

        except Exception:
            pass

        try:
            self.command.close()

        except Exception:
            pass

        print("CSV saved:", self.csv_path)
        print("SECTION CSV saved:", self.section_marks_path)
        print("received:", self.received_count)
        print("lost:", self.packet_loss_count)
        print("bad:", self.bad_packet_count)
        print("sync_fix:", self.sync_fix_count)


def main():
    app = QtWidgets.QApplication(sys.argv)

    window = Dashboard()
    window.show()

    if hasattr(app, "exec"):
        code = app.exec()

    else:
        code = app.exec_()

    window.close_resources()

    sys.exit(code)


if __name__ == "__main__":
    main()
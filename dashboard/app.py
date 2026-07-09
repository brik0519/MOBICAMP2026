# app.py
# PAI-Car Step 3 PyQtGraph telemetry dashboard + UDP command sender
#
# 목적:
#   1. Pico 2 W의 기존 UDP telemetry packet VERSION 1을 PC에서 수신한다.
#   2. 실시간 plot과 CSV 저장을 수행한다.
#   3. PC 키 입력으로 PING / STOP / SAFE_MODE / RUN command를 Pico로 전송한다.
#
# 실행:
#   python app.py
#
# 필요 패키지:
#   python -m pip install pyqtgraph PyQt6
#
# 키:
#   P       PING
#   Space   STOP
#   Z       SAFE_MODE
#   Enter   RUN

import csv
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

LOG_DIR = "logs"
MAX_POINTS = 1500
RECV_TIMER_MS = 5
COMMAND_TIMER_MS = 20
PLOT_TIMER_MS = 33
FLUSH_EVERY_ROWS = 20


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

        self.setWindowTitle("PAI-Car Step 3 Telemetry Dashboard + Command Sender")
        self.resize(1200, 880)

        self.telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_sock.bind((LISTEN_IP, LISTEN_PORT))
        self.telemetry_sock.setblocking(False)

        self.command_sender = CommandSender(
            target_ip=None,
            target_port=COMMAND_PORT,
        )

        os.makedirs(LOG_DIR, exist_ok=True)
        self.csv_path = os.path.join(
            LOG_DIR,
            "pai_car_pyqt_{}.csv".format(datetime.now().strftime("%Y%m%d_%H%M%S")),
        )
        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_HEADER)
        self.csv_writer.writeheader()

        self.received_count = 0
        self.bad_packet_count = 0
        self.packet_loss_count = 0
        self.last_seq = None
        self.last_packet_time = None
        self.last_row = None
        self.last_bad_reason = ""
        self.last_pico_ip = None
        self.closed = False

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

    def poll_command_acks(self):
        self.command_sender.poll_acks()

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

    def keyPressEvent(self, event):
        key = event.key()

        if key == KEY_P:
            self.send_command(CMD_PING)
            return

        if key == KEY_SPACE:
            self.send_command(CMD_NEXT_SECTION)
            return

        if key == KEY_Z:
            self.send_command(CMD_STOP)
            return

        if key == KEY_RETURN or key == KEY_ENTER:
            self.send_command(CMD_RUN)
            return

        super().keyPressEvent(event)

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

        self.status_label.setText(
            "telemetry_listen={}:{}  packet_size={}  version={}  "
            "received={}  lost={} ({:.2f}%)  bad={}  last_age={}ms  bad_reason={}\n"
            "{}\n"
            "{}\n"
            "CSV={}\n"
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
                self.command_sender.stats_text(),
                command_help,
                self.csv_path,
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
            self.telemetry_sock.close()
        except Exception:
            pass

        try:
            self.command_sender.close()
        except Exception:
            pass

        print("CSV saved:", self.csv_path)
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
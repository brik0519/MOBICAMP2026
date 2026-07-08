# pc_udp_monitor.py
# PAI-Car UDP binary data receiver + CSV logger + realtime monitor
#
# 기능:
#   1. Pico 2 W에서 보내는 UDP binary packet을 수신한다.
#   2. packet을 unpack해서 CSV 파일로 저장한다.
#   3. 하나의 figure에 3개의 subplot을 표시한다.
#
# subplot 구성:
#   1) position
#   2) error
#   3) left/right motor command
#
# 주의:
#   Pico 쪽 pai_udp_telemetry.py의 PACKET_FORMAT과 반드시 같아야 한다.
#
# 실행:
#   python pc_udp_monitor.py
#
# 종료:
#   그래프 창을 닫거나 Ctrl+C

import socket
import struct
import csv
import os
from datetime import datetime
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# ------------------------------------------------------------
# UDP receive settings
# ------------------------------------------------------------

LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 5005

RECV_BUFFER_SIZE = 2048


# ------------------------------------------------------------
# Binary packet settings
# ------------------------------------------------------------
# Pico 쪽 pai_udp_telemetry.py와 동일해야 한다.

MAGIC = 0x5041
VERSION = 1

PACKET_FORMAT = "<HBBHIHHh8HHhhhhBB"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)


# ------------------------------------------------------------
# CSV settings
# ------------------------------------------------------------

CSV_HEADER = [
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
]

LOG_DIR = "logs"

os.makedirs(LOG_DIR, exist_ok=True)

CSV_FILENAME = os.path.join(
    LOG_DIR,
    "pai_car_run_{}.csv".format(
        datetime.now().strftime("%Y%m%d_%H%M%S")
    )
)

FLUSH_EVERY_ROWS = 20


# ------------------------------------------------------------
# Plot settings
# ------------------------------------------------------------

MAX_POINTS = 1500       # 50Hz 기준 약 30초 표시
PLOT_INTERVAL_MS = 100  # 그래프 갱신 간격

# 현재 Pico 코드에서는 left_cmd/right_cmd가 이미 최종 모터 명령이다.
#
# False:
#   세 번째 subplot에 left_cmd, right_cmd를 그대로 표시한다.
#
# True:
#   세 번째 subplot에 base_speed + left_cmd,
#   base_speed + right_cmd를 표시한다.
#
# 일반적으로는 False가 맞다.
PLOT_LITERAL_BASE_PLUS_CMD = False


# ------------------------------------------------------------
# Runtime buffers
# ------------------------------------------------------------

time_buf = deque(maxlen=MAX_POINTS)
pos_buf = deque(maxlen=MAX_POINTS)
error_buf = deque(maxlen=MAX_POINTS)
left_buf = deque(maxlen=MAX_POINTS)
right_buf = deque(maxlen=MAX_POINTS)

received_count = 0
bad_packet_count = 0
lost_packet_count = 0
last_seq = None

closed = False


# ------------------------------------------------------------
# Packet parser
# ------------------------------------------------------------

def parse_packet(data):
    """
    UDP binary packet을 dict로 변환한다.

    정상 packet이 아니면 None을 반환한다.
    """

    if len(data) != PACKET_SIZE:
        return None

    try:
        values = struct.unpack(PACKET_FORMAT, data)
    except struct.error:
        return None

    magic = values[0]
    version = values[1]

    if magic != MAGIC:
        return None

    if version != VERSION:
        return None

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

    return row


def update_lost_packet_count(seq):
    """
    seq 번호를 이용해 중간에 빠진 UDP packet 수를 추정한다.
    UDP는 손실 가능성이 있으므로 참고용이다.
    """

    global last_seq
    global lost_packet_count

    if last_seq is None:
        last_seq = seq
        return

    expected = (last_seq + 1) & 0xFFFF

    if seq != expected:
        diff = (seq - expected) & 0xFFFF
        lost_packet_count += diff

    last_seq = seq


# ------------------------------------------------------------
# CSV helper
# ------------------------------------------------------------

def write_csv_row(writer, row):
    writer.writerow([row[name] for name in CSV_HEADER])


# ------------------------------------------------------------
# UDP socket setup
# ------------------------------------------------------------

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LISTEN_IP, LISTEN_PORT))
sock.setblocking(False)

print()
print("PAI-Car UDP binary monitor")
print("Listening on {}:{}".format(LISTEN_IP, LISTEN_PORT))
print("Expected packet size:", PACKET_SIZE)
print("CSV file:", CSV_FILENAME)
print()


# ------------------------------------------------------------
# CSV file setup
# ------------------------------------------------------------

csv_file = open(CSV_FILENAME, "w", newline="", encoding="utf-8")
csv_writer = csv.writer(csv_file)
csv_writer.writerow(CSV_HEADER)


# ------------------------------------------------------------
# Plot setup
# ------------------------------------------------------------

fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))

ax_pos = axes[0]
ax_error = axes[1]
ax_motor = axes[2]

line_pos, = ax_pos.plot([], [], label="position")
line_error, = ax_error.plot([], [], label="error")
line_left, = ax_motor.plot([], [], label="left")
line_right, = ax_motor.plot([], [], label="right")

ax_pos.axhline(3500, linestyle="--", linewidth=1, label="center")
ax_error.axhline(0, linestyle="--", linewidth=1)

ax_pos.set_ylabel("position")
ax_error.set_ylabel("error")
ax_motor.set_ylabel("motor cmd")
ax_motor.set_xlabel("time (s)")

ax_pos.set_ylim(0, 7000)
ax_error.set_ylim(-3500, 3500)

if not PLOT_LITERAL_BASE_PLUS_CMD:
    ax_motor.set_ylim(-1100, 1100)

ax_pos.grid(True)
ax_error.grid(True)
ax_motor.grid(True)

ax_pos.legend(loc="upper right")
ax_error.legend(loc="upper right")
ax_motor.legend(loc="upper right")

fig.tight_layout()


# ------------------------------------------------------------
# Main receive/update functions
# ------------------------------------------------------------

def receive_available_packets():
    """
    현재 도착해 있는 UDP packet을 모두 읽는다.
    """

    global received_count
    global bad_packet_count

    while True:
        try:
            data, addr = sock.recvfrom(RECV_BUFFER_SIZE)

        except BlockingIOError:
            break

        except OSError:
            break

        row = parse_packet(data)

        if row is None:
            bad_packet_count += 1
            continue

        received_count += 1
        update_lost_packet_count(row["seq"])

        write_csv_row(csv_writer, row)

        if received_count % FLUSH_EVERY_ROWS == 0:
            csv_file.flush()

        t_sec = row["t_ms"] / 1000.0

        if PLOT_LITERAL_BASE_PLUS_CMD:
            left_value = row["base_speed"] + row["left_cmd"]
            right_value = row["base_speed"] + row["right_cmd"]
        else:
            left_value = row["left_cmd"]
            right_value = row["right_cmd"]

        time_buf.append(t_sec)
        pos_buf.append(row["position"])
        error_buf.append(row["error"])
        left_buf.append(left_value)
        right_buf.append(right_value)


def update_plot(frame):
    """
    matplotlib animation callback.
    """

    receive_available_packets()

    line_pos.set_data(time_buf, pos_buf)
    line_error.set_data(time_buf, error_buf)
    line_left.set_data(time_buf, left_buf)
    line_right.set_data(time_buf, right_buf)

    if len(time_buf) >= 2:
        x_min = time_buf[0]
        x_max = time_buf[-1]

        if x_max <= x_min:
            x_max = x_min + 1.0

        ax_pos.set_xlim(x_min, x_max)

    if PLOT_LITERAL_BASE_PLUS_CMD:
        ax_motor.relim()
        ax_motor.autoscale_view(scalex=False, scaley=True)

    fig.suptitle(
        "received: {}   lost: {}   bad: {}   csv: {}".format(
            received_count,
            lost_packet_count,
            bad_packet_count,
            CSV_FILENAME
        )
    )

    return line_pos, line_error, line_left, line_right


# ------------------------------------------------------------
# Cleanup
# ------------------------------------------------------------

def close_resources():
    """
    파일과 socket을 정리한다.
    """

    global closed

    if closed:
        return

    closed = True

    try:
        csv_file.flush()
        csv_file.close()
    except Exception:
        pass

    try:
        sock.close()
    except Exception:
        pass

    print()
    print("Stopped.")
    print("CSV saved:", CSV_FILENAME)
    print("received:", received_count)
    print("lost:", lost_packet_count)
    print("bad:", bad_packet_count)


def on_close(event):
    close_resources()


fig.canvas.mpl_connect("close_event", on_close)


# ------------------------------------------------------------
# Run
# ------------------------------------------------------------

try:
    ani = FuncAnimation(
        fig,
        update_plot,
        interval=PLOT_INTERVAL_MS,
        blit=False,
        cache_frame_data=False
    )

    plt.show()

except KeyboardInterrupt:
    close_resources()

finally:
    close_resources()
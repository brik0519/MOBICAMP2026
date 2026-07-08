# pc_udp_monitor.py
# PAI-Car UDP binary telemetry receiver
# + telemetry CSV logger
# + realtime graph
# + wireless debug text receiver
# + parsed debug CSV logger
#
# Ports:
#   5005: binary telemetry
#   5006: debug text
#
# Binary telemetry:
#   current_speed
#   normalized sensors
#   position
#   error
#   filtered error rate
#   left/right final motor commands
#   line and marker states
#
# Debug telemetry:
#   target/current speed
#   curve score
#   controller gains
#   P/D terms
#   steering
#   saturation
#   execution time
#
# Run:
#   python pc_udp_monitor.py
#
# Stop:
#   Close graph window or press Ctrl+C

import socket
import struct
import csv
import os

from datetime import datetime
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# ============================================================
# 1. UDP receive settings
# ============================================================

LISTEN_IP = "0.0.0.0"

TELEMETRY_LISTEN_PORT = 5005
DEBUG_LISTEN_PORT = 5006

RECV_BUFFER_SIZE = 4096


# ============================================================
# 2. Binary packet settings
# ============================================================
# Must match modules/pai_udp_telemetry.py.

MAGIC = 0x5041
VERSION = 1

PACKET_FORMAT = "<HBBHIHHh8HHhhhhBB"

PACKET_SIZE = struct.calcsize(
    PACKET_FORMAT
)


# ============================================================
# 3. Log settings
# ============================================================

LOG_DIR = "logs"

os.makedirs(
    LOG_DIR,
    exist_ok=True,
)

RUN_TIMESTAMP = datetime.now().strftime(
    "%Y%m%d_%H%M%S"
)

TELEMETRY_CSV_FILENAME = os.path.join(
    LOG_DIR,
    "pai_car_run_{}.csv".format(
        RUN_TIMESTAMP
    ),
)

DEBUG_LOG_FILENAME = os.path.join(
    LOG_DIR,
    "pai_car_debug_{}.log".format(
        RUN_TIMESTAMP
    ),
)

DEBUG_CSV_FILENAME = os.path.join(
    LOG_DIR,
    "pai_car_debug_{}.csv".format(
        RUN_TIMESTAMP
    ),
)

FLUSH_EVERY_ROWS = 20
DEBUG_FLUSH_EVERY_ROWS = 5


# ============================================================
# 4. Telemetry CSV format
# ============================================================

TELEMETRY_CSV_HEADER = [
    "seq",
    "t_ms",
    "control_ms",
    "send_ms",
    "current_speed",

    "n0",
    "n1",
    "n2",
    "n3",
    "n4",
    "n5",
    "n6",
    "n7",

    "position",
    "error",
    "d_error",
    "left_cmd",
    "right_cmd",
    "on_line",
    "is_marker",
]


# ============================================================
# 5. Debug CSV format
# ============================================================

DEBUG_CSV_HEADER = [
    "pc_time",
    "sender",
    "type",

    "loop",
    "drive",
    "speed_state",

    "target",
    "current",

    "compute_ms",
    "max_ms",
    "overrun",

    "error",
    "filtered_error",
    "filtered_rate",

    "confidence",
    "active",
    "wide",
    "curve",

    "kp",
    "kd",
    "p",
    "d",

    "steering",
    "scale",

    "left",
    "right",

    "finished",
    "mode",
    "control_ms",
    "loops",
    "average_compute_ms",
    "max_compute_ms",
    "overrun_count",
    "overrun_rate",
    "line_lost_entry",

    "raw_message",
]


# ============================================================
# 6. Plot settings
# ============================================================

# At approximately 50 Hz, 1500 points represent about 30 seconds.
MAX_POINTS = 1500

PLOT_INTERVAL_MS = 100

# left_cmd and right_cmd are already final motor commands.
PLOT_LITERAL_BASE_PLUS_CMD = False


# ============================================================
# 7. Runtime plot buffers
# ============================================================

time_buf = deque(
    maxlen=MAX_POINTS
)

position_buf = deque(
    maxlen=MAX_POINTS
)

error_buf = deque(
    maxlen=MAX_POINTS
)

left_buf = deque(
    maxlen=MAX_POINTS
)

right_buf = deque(
    maxlen=MAX_POINTS
)

current_speed_buf = deque(
    maxlen=MAX_POINTS
)


# ============================================================
# 8. Runtime counters and state
# ============================================================

received_count = 0
bad_packet_count = 0
lost_packet_count = 0

debug_received_count = 0
debug_decode_error_count = 0

last_seq = None

last_debug_message = ""
last_debug_sender = ""

closed = False


# ============================================================
# 9. Binary packet parser
# ============================================================

def parse_packet(data):
    """
    Convert one binary UDP packet to a dictionary.

    Return None if the packet is invalid.
    """

    if len(data) != PACKET_SIZE:
        return None

    try:
        values = struct.unpack(
            PACKET_FORMAT,
            data,
        )

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

    # The revised main.py transmits current_speed here.
    current_speed = values[7]

    norm = values[8:16]

    position = values[16]
    error = values[17]
    d_error = values[18]
    left_cmd = values[19]
    right_cmd = values[20]
    on_line = values[21]
    is_marker = values[22]

    return {
        "seq": seq,
        "t_ms": t_ms,
        "control_ms": control_ms,
        "send_ms": send_ms,
        "current_speed": current_speed,

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


def update_lost_packet_count(seq):
    """
    Estimate lost packets using the 16-bit sequence number.

    Large jumps are treated as reboot or sequence reset.
    """

    global last_seq
    global lost_packet_count

    if last_seq is None:
        last_seq = seq
        return

    expected = (
        last_seq + 1
    ) & 0xFFFF

    if seq != expected:
        diff = (
            seq - expected
        ) & 0xFFFF

        if diff < 10000:
            lost_packet_count += diff

    last_seq = seq


# ============================================================
# 10. Debug message parser
# ============================================================

def parse_debug_message(text):
    """
    Parse a comma-separated key=value UDP debug message.

    Example:
        type=status,loop=100,target=220,current=310
    """

    result = {}

    parts = text.strip().split(",")

    for part in parts:
        if "=" not in part:
            continue

        key, value = part.split(
            "=",
            1,
        )

        result[
            key.strip()
        ] = value.strip()

    return result


def create_debug_csv_row(
    parsed,
    timestamp,
    sender,
    raw_message,
):
    return {
        "pc_time": timestamp,
        "sender": sender,
        "type": parsed.get(
            "type",
            "",
        ),

        "loop": parsed.get(
            "loop",
            "",
        ),
        "drive": parsed.get(
            "drive",
            "",
        ),
        "speed_state": parsed.get(
            "speed_state",
            "",
        ),

        "target": parsed.get(
            "target",
            "",
        ),
        "current": parsed.get(
            "current",
            "",
        ),

        "compute_ms": parsed.get(
            "compute_ms",
            "",
        ),
        "max_ms": parsed.get(
            "max_ms",
            "",
        ),
        "overrun": parsed.get(
            "overrun",
            "",
        ),

        "error": parsed.get(
            "error",
            "",
        ),
        "filtered_error": parsed.get(
            "filtered_error",
            "",
        ),
        "filtered_rate": parsed.get(
            "filtered_rate",
            "",
        ),

        "confidence": parsed.get(
            "confidence",
            "",
        ),
        "active": parsed.get(
            "active",
            "",
        ),
        "wide": parsed.get(
            "wide",
            "",
        ),
        "curve": parsed.get(
            "curve",
            "",
        ),

        "kp": parsed.get(
            "kp",
            "",
        ),
        "kd": parsed.get(
            "kd",
            "",
        ),
        "p": parsed.get(
            "p",
            "",
        ),
        "d": parsed.get(
            "d",
            "",
        ),

        "steering": parsed.get(
            "steering",
            "",
        ),
        "scale": parsed.get(
            "scale",
            "",
        ),

        "left": parsed.get(
            "left",
            "",
        ),
        "right": parsed.get(
            "right",
            "",
        ),

        "finished": parsed.get(
            "finished",
            "",
        ),
        "mode": parsed.get(
            "mode",
            "",
        ),
        "control_ms": parsed.get(
            "control_ms",
            "",
        ),
        "loops": parsed.get(
            "loops",
            "",
        ),
        "average_compute_ms": parsed.get(
            "average_compute_ms",
            "",
        ),
        "max_compute_ms": parsed.get(
            "max_compute_ms",
            "",
        ),
        "overrun_count": parsed.get(
            "overrun_count",
            "",
        ),
        "overrun_rate": parsed.get(
            "overrun_rate",
            "",
        ),
        "line_lost_entry": parsed.get(
            "line_lost_entry",
            "",
        ),

        "raw_message": raw_message,
    }


# ============================================================
# 11. CSV helpers
# ============================================================

def write_telemetry_csv_row(
    writer,
    row,
):
    writer.writerow(
        [
            row[name]
            for name in TELEMETRY_CSV_HEADER
        ]
    )


def write_debug_csv_row(
    writer,
    row,
):
    writer.writerow(
        [
            row[name]
            for name in DEBUG_CSV_HEADER
        ]
    )


# ============================================================
# 12. UDP socket setup
# ============================================================

telemetry_sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM,
)

telemetry_sock.bind(
    (
        LISTEN_IP,
        TELEMETRY_LISTEN_PORT,
    )
)

telemetry_sock.setblocking(
    False
)


debug_sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM,
)

debug_sock.bind(
    (
        LISTEN_IP,
        DEBUG_LISTEN_PORT,
    )
)

debug_sock.setblocking(
    False
)


# ============================================================
# 13. Startup information
# ============================================================

print()
print("PAI-Car UDP monitor")
print()

print(
    "Binary telemetry : {}:{}".format(
        LISTEN_IP,
        TELEMETRY_LISTEN_PORT,
    )
)

print(
    "Debug text      : {}:{}".format(
        LISTEN_IP,
        DEBUG_LISTEN_PORT,
    )
)

print(
    "Packet size     : {}".format(
        PACKET_SIZE
    )
)

print(
    "Telemetry CSV   : {}".format(
        TELEMETRY_CSV_FILENAME
    )
)

print(
    "Debug CSV       : {}".format(
        DEBUG_CSV_FILENAME
    )
)

print(
    "Debug raw log   : {}".format(
        DEBUG_LOG_FILENAME
    )
)

print()


# ============================================================
# 14. Log file setup
# ============================================================

telemetry_csv_file = open(
    TELEMETRY_CSV_FILENAME,
    "w",
    newline="",
    encoding="utf-8",
)

telemetry_csv_writer = csv.writer(
    telemetry_csv_file
)

telemetry_csv_writer.writerow(
    TELEMETRY_CSV_HEADER
)


debug_csv_file = open(
    DEBUG_CSV_FILENAME,
    "w",
    newline="",
    encoding="utf-8",
)

debug_csv_writer = csv.writer(
    debug_csv_file
)

debug_csv_writer.writerow(
    DEBUG_CSV_HEADER
)


debug_log_file = open(
    DEBUG_LOG_FILENAME,
    "w",
    encoding="utf-8",
)


# ============================================================
# 15. Plot setup
# ============================================================

fig, axes = plt.subplots(
    3,
    1,
    sharex=True,
    figsize=(11, 8),
)

ax_position = axes[0]
ax_error = axes[1]
ax_motor = axes[2]


line_position, = ax_position.plot(
    [],
    [],
    label="position",
)

line_error, = ax_error.plot(
    [],
    [],
    label="error",
)

line_left, = ax_motor.plot(
    [],
    [],
    label="left command",
)

line_right, = ax_motor.plot(
    [],
    [],
    label="right command",
)

line_current_speed, = ax_motor.plot(
    [],
    [],
    linestyle="--",
    label="current speed",
)


ax_position.axhline(
    3500,
    linestyle="--",
    linewidth=1,
    label="center",
)

ax_error.axhline(
    0,
    linestyle="--",
    linewidth=1,
)

ax_motor.axhline(
    0,
    linestyle="--",
    linewidth=1,
)


ax_position.set_ylabel(
    "position"
)

ax_error.set_ylabel(
    "error"
)

ax_motor.set_ylabel(
    "motor command"
)

ax_motor.set_xlabel(
    "time (s)"
)


ax_position.set_ylim(
    0,
    7000,
)

ax_error.set_ylim(
    -3500,
    3500,
)

if not PLOT_LITERAL_BASE_PLUS_CMD:
    ax_motor.set_ylim(
        -1100,
        1100,
    )


ax_position.grid(
    True
)

ax_error.grid(
    True
)

ax_motor.grid(
    True
)


ax_position.legend(
    loc="upper right"
)

ax_error.legend(
    loc="upper right"
)

ax_motor.legend(
    loc="upper right"
)


fig.tight_layout()


# ============================================================
# 16. Binary telemetry receiver
# ============================================================

def receive_available_packets():
    """
    Receive all currently available binary UDP packets.
    """

    global received_count
    global bad_packet_count

    while True:
        try:
            data, addr = telemetry_sock.recvfrom(
                RECV_BUFFER_SIZE
            )

        except BlockingIOError:
            break

        except OSError:
            break

        row = parse_packet(
            data
        )

        if row is None:
            bad_packet_count += 1
            continue

        received_count += 1

        update_lost_packet_count(
            row["seq"]
        )

        write_telemetry_csv_row(
            telemetry_csv_writer,
            row,
        )

        if (
            received_count
            % FLUSH_EVERY_ROWS
            == 0
        ):
            telemetry_csv_file.flush()

        t_sec = (
            row["t_ms"]
            / 1000.0
        )

        if PLOT_LITERAL_BASE_PLUS_CMD:
            left_value = (
                row["current_speed"]
                + row["left_cmd"]
            )

            right_value = (
                row["current_speed"]
                + row["right_cmd"]
            )

        else:
            left_value = row[
                "left_cmd"
            ]

            right_value = row[
                "right_cmd"
            ]

        time_buf.append(
            t_sec
        )

        position_buf.append(
            row["position"]
        )

        error_buf.append(
            row["error"]
        )

        left_buf.append(
            left_value
        )

        right_buf.append(
            right_value
        )

        current_speed_buf.append(
            row["current_speed"]
        )


# ============================================================
# 17. Debug text receiver
# ============================================================

def receive_available_debug_messages():
    """
    Receive all available human-readable UDP debug messages.

    Raw messages are stored in a log file.
    Parsed key-value fields are stored in a CSV file.
    """

    global debug_received_count
    global debug_decode_error_count
    global last_debug_message
    global last_debug_sender

    while True:
        try:
            data, addr = debug_sock.recvfrom(
                RECV_BUFFER_SIZE
            )

        except BlockingIOError:
            break

        except OSError:
            break

        try:
            text = data.decode(
                "utf-8",
                errors="replace",
            )

        except Exception:
            debug_decode_error_count += 1

            text = repr(
                data
            )

        debug_received_count += 1

        last_debug_message = text

        last_debug_sender = (
            "{}:{}".format(
                addr[0],
                addr[1],
            )
        )

        timestamp = datetime.now().strftime(
            "%Y-%m-%d %H:%M:%S.%f"
        )[:-3]

        log_line = (
            "[{}] [{}] {}\n".format(
                timestamp,
                last_debug_sender,
                text,
            )
        )

        print(
            log_line,
            end="",
        )

        debug_log_file.write(
            log_line
        )

        parsed = parse_debug_message(
            text
        )

        debug_row = create_debug_csv_row(
            parsed,
            timestamp,
            last_debug_sender,
            text.strip(),
        )

        write_debug_csv_row(
            debug_csv_writer,
            debug_row,
        )

        if (
            debug_received_count
            % DEBUG_FLUSH_EVERY_ROWS
            == 0
        ):
            debug_log_file.flush()
            debug_csv_file.flush()


# ============================================================
# 18. Plot update
# ============================================================

def update_plot(frame):
    """
    Matplotlib animation callback.
    """

    receive_available_packets()
    receive_available_debug_messages()

    line_position.set_data(
        time_buf,
        position_buf,
    )

    line_error.set_data(
        time_buf,
        error_buf,
    )

    line_left.set_data(
        time_buf,
        left_buf,
    )

    line_right.set_data(
        time_buf,
        right_buf,
    )

    line_current_speed.set_data(
        time_buf,
        current_speed_buf,
    )

    if len(time_buf) >= 2:
        x_min = time_buf[0]
        x_max = time_buf[-1]

        if x_max <= x_min:
            x_max = (
                x_min + 1.0
            )

        ax_position.set_xlim(
            x_min,
            x_max,
        )

    if PLOT_LITERAL_BASE_PLUS_CMD:
        ax_motor.relim()

        ax_motor.autoscale_view(
            scalex=False,
            scaley=True,
        )

    title = (
        "binary: {}   "
        "debug: {}   "
        "lost: {}   "
        "bad: {}"
    ).format(
        received_count,
        debug_received_count,
        lost_packet_count,
        bad_packet_count,
    )

    if last_debug_message:
        if len(last_debug_message) <= 140:
            short_debug = (
                last_debug_message
            )
        else:
            short_debug = (
                last_debug_message[:137]
                + "..."
            )

        title += (
            "\nlast debug: "
            + short_debug
        )

    fig.suptitle(
        title
    )

    return (
        line_position,
        line_error,
        line_left,
        line_right,
        line_current_speed,
    )


# ============================================================
# 19. Cleanup
# ============================================================

def close_resources():
    """
    Flush and close files and sockets.
    """

    global closed

    if closed:
        return

    closed = True

    try:
        telemetry_csv_file.flush()
        telemetry_csv_file.close()
    except Exception:
        pass

    try:
        debug_csv_file.flush()
        debug_csv_file.close()
    except Exception:
        pass

    try:
        debug_log_file.flush()
        debug_log_file.close()
    except Exception:
        pass

    try:
        telemetry_sock.close()
    except Exception:
        pass

    try:
        debug_sock.close()
    except Exception:
        pass

    print()
    print("Stopped.")

    print(
        "Telemetry CSV   : {}".format(
            TELEMETRY_CSV_FILENAME
        )
    )

    print(
        "Debug CSV       : {}".format(
            DEBUG_CSV_FILENAME
        )
    )

    print(
        "Debug raw log   : {}".format(
            DEBUG_LOG_FILENAME
        )
    )

    print(
        "Binary received : {}".format(
            received_count
        )
    )

    print(
        "Debug received  : {}".format(
            debug_received_count
        )
    )

    print(
        "Lost packets    : {}".format(
            lost_packet_count
        )
    )

    print(
        "Bad packets     : {}".format(
            bad_packet_count
        )
    )

    print(
        "Decode errors   : {}".format(
            debug_decode_error_count
        )
    )


def on_close(event):
    close_resources()


fig.canvas.mpl_connect(
    "close_event",
    on_close,
)


# ============================================================
# 20. Run
# ============================================================

try:
    animation = FuncAnimation(
        fig,
        update_plot,
        interval=PLOT_INTERVAL_MS,
        blit=False,
        cache_frame_data=False,
    )

    plt.show()

except KeyboardInterrupt:
    close_resources()

finally:
    close_resources()
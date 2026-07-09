# main.py
# PAI-Car high-speed simplified PD controller
# - aggressive speed mode
# - single KP / KD tuning
# - error-based speed control
# - straight boost up to 1000
# - hard corner mode for right-angle / sharp corners
# - late U-turn / emergency curvature mode
# - simple motor mixer
# - short strong line-loss recovery
# - UDP telemetry/debug compatible

from time import ticks_ms, ticks_diff
import socket

import modules.pai_car_run_support as run_support
import modules.pai_udp_telemetry as udp_telemetry

from modules.pai_car_run_support import (
    create_lap_timer,
    setup_paicar,
    self_calibrate_or_stop,
    wait_button_start,
    limit_cmd,
    wait_control_period,
)

from modules.pai_udp_telemetry import (
    PAIUdpTelemetry,
    read_line_detail,
    is_t_marker_area,
)

from modules.pai_car_wifi_config import (
    PC_IP,
    PC_PORT,
)


# ============================================================
# TUNING PANEL
# ============================================================

DEBUG_MODE = True
CONTROL_MS = 10


# ------------------------------------------------------------
# Speed
# ------------------------------------------------------------

BASE_SPEED = 850
MAX_SPEED = 1000
MIN_SPEED = 420

# 일반 고속 주행 감속 기준.
SLOW_ERROR = 750
HARD_ERROR = 1500

SLOW_SPEED = 700
HARD_SPEED = 520

BOOST_ERROR = 220
BOOST_D_ERROR = 70

SPEED_RISE = 25
SPEED_FALL = 150


# ------------------------------------------------------------
# Steering
# ------------------------------------------------------------

KP = 0.24
KD = 0.42

STEERING_LIMIT = 620
ERROR_DEADBAND = 60

ERROR_ALPHA = 0.55
D_ALPHA = 0.18

STRAIGHT_DAMP_ERROR_1 = 300
STRAIGHT_DAMP_D_1 = 90
STRAIGHT_DAMP_GAIN_1 = 0.55

STRAIGHT_DAMP_ERROR_2 = 500
STRAIGHT_DAMP_D_2 = 140
STRAIGHT_DAMP_GAIN_2 = 0.75


# ------------------------------------------------------------
# Hard corner mode
# ------------------------------------------------------------

# 중간 직각/급코너용.
# UTURN보다 덜 과격하게 회전한다.
HARD_CORNER_MIN_SPEED = 620

HARD_CORNER_ENTRY_ERROR = 1050
HARD_CORNER_ENTRY_D = 90
HARD_CORNER_HARD_ERROR = 1900

HARD_CORNER_HOLD_MS = 260

HARD_CORNER_SPEED = 460
HARD_CORNER_STEERING_LIMIT = 740
HARD_CORNER_INNER_FLOOR = 60
HARD_CORNER_OUTER_BOOST = 900


# ------------------------------------------------------------
# U-turn / emergency curvature mode
# ------------------------------------------------------------

# 마지막 U턴이 있는 후반 구간 전까지는 UTURN 모드 금지.
# 중간 직각 코너가 UTURN처럼 처리되는 것을 막는다.
UTURN_ENABLE_AFTER_MS = 32000

UTURN_MIN_SPEED = 650

UTURN_ENTRY_ERROR = 1300
UTURN_ENTRY_D = 120

UTURN_ENTRY_ERROR_STRONG = 1650
UTURN_ENTRY_D_STRONG = 170

UTURN_HARD_ERROR = 2300
UTURN_EDGE_SUM = 1100

UTURN_HOLD_MS = 420

UTURN_SPEED = 380
UTURN_STEERING_LIMIT = 850
UTURN_INNER_FLOOR = 0
UTURN_OUTER_BOOST = 1000


# ------------------------------------------------------------
# Motor
# ------------------------------------------------------------

MOTOR_MAX_CMD = 1000

LEFT_GAIN = 1.00
RIGHT_GAIN = 1.00

ALLOW_REVERSE = False

# 일반 추종 중 안쪽 바퀴 최소 출력.
INNER_FLOOR = 120


# ------------------------------------------------------------
# Line loss recovery
# ------------------------------------------------------------

LOST_FORWARD = 250
LOST_TURN = 900
LOST_PIVOT_AFTER_MS = 120


# ------------------------------------------------------------
# Finish guard
# ------------------------------------------------------------

MIN_FINISH_MS = 1500
FINISH_CONFIRM_COUNT = 2


# ------------------------------------------------------------
# Sensor
# ------------------------------------------------------------

SENSOR_ACTIVE_THRESHOLD = 300
SENSOR_SUM_FULL = 4000


# ------------------------------------------------------------
# Debug / telemetry
# ------------------------------------------------------------

DEBUG_PC_PORT = PC_PORT + 1
DEBUG_REPORT_MS = 500

OVERRUN_THRESHOLD_MS = CONTROL_MS

MAX_NETWORK_SEND_COST_MS = 2
NETWORK_COOLDOWN_MS = 120

TELEMETRY_SKIP_COMPUTE_MS = 7
DEBUG_REPORT_MAX_LAG_MS = 1500


# Apply control period to support modules.
run_support.CONTROL_MS = CONTROL_MS
udp_telemetry.CONTROL_MS = CONTROL_MS


# ============================================================
# Helpers
# ============================================================

def clamp(value, minimum, maximum):
    if value < minimum:
        return minimum

    if value > maximum:
        return maximum

    return value


def move_speed(current, target):
    if target > current:
        return min(
            current + SPEED_RISE,
            target,
        )

    return max(
        current - SPEED_FALL,
        target,
    )


def read_sensor_data(line):
    (
        error,
        position,
        norm,
        on_line,
    ) = read_line_detail(line)

    is_marker = is_t_marker_area(
        norm,
        on_line,
    )

    return (
        error,
        position,
        norm,
        on_line,
        is_marker,
    )


def count_active_sensors(norm):
    count = 0
    total = 0

    for value in norm:
        total += value

        if value >= SENSOR_ACTIVE_THRESHOLD:
            count += 1

    if SENSOR_SUM_FULL > 0:
        confidence = total / SENSOR_SUM_FULL
    else:
        confidence = 0.0

    confidence = clamp(
        confidence,
        0.0,
        1.0,
    )

    return (
        count,
        total,
        confidence,
    )


def edge_sensor_sums(norm):
    left_edge = norm[0] + norm[1]
    right_edge = norm[6] + norm[7]

    return (
        left_edge,
        right_edge,
    )


# ============================================================
# Filter / speed / steering
# ============================================================

def update_filters(
    error,
    on_line,
    previous_filtered_error,
    previous_filtered_d,
):
    if on_line:
        filtered_error = (
            previous_filtered_error
            + ERROR_ALPHA
            * (
                error
                - previous_filtered_error
            )
        )
    else:
        filtered_error = previous_filtered_error

    if on_line:
        raw_d = (
            filtered_error
            - previous_filtered_error
        ) * (
            10.0
            / CONTROL_MS
        )
    else:
        raw_d = 0.0

    filtered_d = (
        previous_filtered_d
        + D_ALPHA
        * (
            raw_d
            - previous_filtered_d
        )
    )

    return (
        filtered_error,
        filtered_d,
    )


def calculate_target_speed(
    filtered_error,
    filtered_d,
):
    abs_error = abs(
        filtered_error
    )

    if abs_error >= HARD_ERROR:
        target = HARD_SPEED

    elif abs_error >= SLOW_ERROR:
        target = SLOW_SPEED

    else:
        target = BASE_SPEED

    if (
        abs_error < BOOST_ERROR
        and abs(filtered_d) < BOOST_D_ERROR
    ):
        target = MAX_SPEED

    return clamp(
        target,
        MIN_SPEED,
        MAX_SPEED,
    )


def apply_error_deadband(error):
    if abs(error) <= ERROR_DEADBAND:
        return 0.0

    if error > 0:
        return error - ERROR_DEADBAND

    return error + ERROR_DEADBAND


def damp_steering_on_straight(
    steering,
    filtered_error,
    filtered_d,
):
    abs_error = abs(
        filtered_error
    )

    abs_d = abs(
        filtered_d
    )

    if (
        abs_error < STRAIGHT_DAMP_ERROR_1
        and abs_d < STRAIGHT_DAMP_D_1
    ):
        return int(
            steering
            * STRAIGHT_DAMP_GAIN_1
        )

    if (
        abs_error < STRAIGHT_DAMP_ERROR_2
        and abs_d < STRAIGHT_DAMP_D_2
    ):
        return int(
            steering
            * STRAIGHT_DAMP_GAIN_2
        )

    return int(
        steering
    )


def calculate_steering(
    filtered_error,
    filtered_d,
    steering_limit,
    allow_straight_damping=True,
):
    control_error = apply_error_deadband(
        filtered_error
    )

    steering = (
        KP
        * control_error
        + KD
        * filtered_d
    )

    steering = clamp(
        steering,
        -steering_limit,
        steering_limit,
    )

    if allow_straight_damping:
        steering = damp_steering_on_straight(
            steering,
            filtered_error,
            filtered_d,
        )

    return int(
        steering
    )


# ============================================================
# Corner detection
# ============================================================

def detect_hard_corner_entry(
    filtered_error,
    filtered_d,
    current_speed,
):
    if current_speed < HARD_CORNER_MIN_SPEED:
        return False

    abs_error = abs(
        filtered_error
    )

    abs_d = abs(
        filtered_d
    )

    moving_away = (
        filtered_error
        * filtered_d
        > 0
    )

    if abs_error >= HARD_CORNER_HARD_ERROR:
        return True

    if (
        abs_error >= HARD_CORNER_ENTRY_ERROR
        and abs_d >= HARD_CORNER_ENTRY_D
        and moving_away
    ):
        return True

    return False


def detect_uturn_entry(
    filtered_error,
    filtered_d,
    current_speed,
    norm,
):
    if current_speed < UTURN_MIN_SPEED:
        return False

    abs_error = abs(
        filtered_error
    )

    abs_d = abs(
        filtered_d
    )

    (
        left_edge,
        right_edge,
    ) = edge_sensor_sums(
        norm
    )

    edge_strong = (
        left_edge >= UTURN_EDGE_SUM
        or right_edge >= UTURN_EDGE_SUM
    )

    moving_away = (
        filtered_error
        * filtered_d
        > 0
    )

    if abs_error >= UTURN_HARD_ERROR:
        return True

    if (
        abs_error >= UTURN_ENTRY_ERROR
        and abs_d >= UTURN_ENTRY_D
        and moving_away
        and edge_strong
    ):
        return True

    if (
        abs_error >= UTURN_ENTRY_ERROR_STRONG
        and abs_d >= UTURN_ENTRY_D_STRONG
        and moving_away
    ):
        return True

    return False


# ============================================================
# Motor / recovery
# ============================================================

def mix_motor(
    speed,
    steering,
    inner_floor,
    outer_boost=None,
):
    left = (
        speed
        + steering
    )

    right = (
        speed
        - steering
    )

    left *= LEFT_GAIN
    right *= RIGHT_GAIN

    if not ALLOW_REVERSE:
        if left < 0:
            left = 0

        if right < 0:
            right = 0

    if speed > 0:
        if left <= 0:
            left = inner_floor
        elif left < inner_floor:
            left = inner_floor

        if right <= 0:
            right = inner_floor
        elif right < inner_floor:
            right = inner_floor

    if outer_boost is not None:
        if steering > 0:
            left = max(
                left,
                outer_boost,
            )
        elif steering < 0:
            right = max(
                right,
                outer_boost,
            )

    left = clamp(
        left,
        0,
        MOTOR_MAX_CMD,
    )

    right = clamp(
        right,
        0,
        MOTOR_MAX_CMD,
    )

    return (
        limit_cmd(
            int(left)
        ),
        limit_cmd(
            int(right)
        ),
    )


def calculate_recovery_drive(
    last_error,
    lost_elapsed_ms,
):
    if lost_elapsed_ms < LOST_PIVOT_AFTER_MS:
        if last_error < 0:
            return (
                LOST_FORWARD,
                LOST_TURN,
            )

        return (
            LOST_TURN,
            LOST_FORWARD,
        )

    if last_error < 0:
        return (
            0,
            LOST_TURN,
        )

    return (
        LOST_TURN,
        0,
    )


# ============================================================
# Debug UDP
# ============================================================

def create_debug_socket():
    if not DEBUG_MODE:
        return None

    try:
        debug_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )

        try:
            debug_socket.setblocking(False)
        except Exception:
            pass

        return debug_socket

    except OSError:
        return None


def send_debug_message(
    debug_socket,
    message,
):
    if debug_socket is None:
        return (
            False,
            0,
        )

    start_ms = ticks_ms()

    try:
        debug_socket.sendto(
            message.encode(),
            (
                PC_IP,
                DEBUG_PC_PORT,
            ),
        )

        cost_ms = ticks_diff(
            ticks_ms(),
            start_ms,
        )

        return (
            True,
            cost_ms,
        )

    except OSError:
        cost_ms = ticks_diff(
            ticks_ms(),
            start_ms,
        )

        return (
            False,
            cost_ms,
        )


def send_debug_report(
    debug_socket,
    loop_count,
    overrun_count,
    max_loop_ms,
    last_loop_ms,
    target_speed,
    current_speed,
    error,
    filtered_error,
    filtered_d,
    confidence,
    active_count,
    steering,
    left_cmd,
    right_cmd,
    on_line,
    in_hard_corner,
    in_uturn,
    uturn_detection_enabled,
):
    if debug_socket is None:
        return (
            False,
            0,
        )

    message = (
        "type=status,"
        "loop={},"
        "target={},"
        "current={},"
        "compute_ms={},"
        "max_ms={},"
        "overrun={},"
        "error={},"
        "filtered_error={:.1f},"
        "filtered_rate={:.1f},"
        "confidence={:.2f},"
        "active={},"
        "kp={:.3f},"
        "kd={:.3f},"
        "steering={},"
        "left={},"
        "right={},"
        "on_line={},"
        "hard={},"
        "uturn={},"
        "uturn_enabled={}"
    ).format(
        loop_count,
        target_speed,
        current_speed,
        last_loop_ms,
        max_loop_ms,
        overrun_count,
        error,
        filtered_error,
        filtered_d,
        confidence,
        active_count,
        KP,
        KD,
        steering,
        left_cmd,
        right_cmd,
        1 if on_line else 0,
        1 if in_hard_corner else 0,
        1 if in_uturn else 0,
        1 if uturn_detection_enabled else 0,
    )

    return send_debug_message(
        debug_socket,
        message,
    )


def send_final_report(
    debug_socket,
    finished,
    loop_count,
    overrun_count,
    max_loop_ms,
    total_compute_ms,
    line_lost_count,
    hard_corner_entry_count,
    uturn_entry_count,
    telemetry_sent_count,
    telemetry_skip_count,
    debug_skip_count,
    network_slow_count,
):
    if debug_socket is None:
        return

    if loop_count > 0:
        average_compute_ms = (
            total_compute_ms
            / loop_count
        )

        overrun_rate = (
            overrun_count
            * 100.0
            / loop_count
        )
    else:
        average_compute_ms = 0.0
        overrun_rate = 0.0

    message = (
        "type=final,"
        "finished={},"
        "mode=HIGH_SPEED_HARD_UTURN,"
        "control_ms={},"
        "loops={},"
        "average_compute_ms={:.3f},"
        "max_compute_ms={},"
        "overrun_count={},"
        "overrun_rate={:.2f},"
        "line_lost_entry={},"
        "hard_entry={},"
        "uturn_entry={},"
        "telemetry_sent={},"
        "telemetry_skip={},"
        "debug_skip={},"
        "network_slow={}"
    ).format(
        finished,
        CONTROL_MS,
        loop_count,
        average_compute_ms,
        max_loop_ms,
        overrun_count,
        overrun_rate,
        line_lost_count,
        hard_corner_entry_count,
        uturn_entry_count,
        telemetry_sent_count,
        telemetry_skip_count,
        debug_skip_count,
        network_slow_count,
    )

    send_debug_message(
        debug_socket,
        message,
    )


# ============================================================
# OLED
# ============================================================

def show_running_mode(lap_timer):
    try:
        lap_timer.show(
            "HIGH HARD U",
            "BASE {}".format(BASE_SPEED),
            "MAX {}".format(MAX_SPEED),
            "U after {}".format(UTURN_ENABLE_AFTER_MS),
        )
    except Exception:
        pass


# ============================================================
# Main
# ============================================================

def run():
    lap_timer = None
    motors = None

    telemetry = None
    debug_socket = None

    finished = False

    loop_count = 0
    overrun_count = 0

    max_loop_ms = 0
    total_compute_ms = 0
    last_loop_ms = 0

    line_lost_count = 0
    hard_corner_entry_count = 0
    uturn_entry_count = 0

    telemetry_sent_count = 0
    telemetry_skip_count = 0
    debug_skip_count = 0
    network_slow_count = 0

    network_cooldown_until_ms = 0

    try:
        lap_timer = create_lap_timer()

        (
            line,
            motors,
            button,
        ) = setup_paicar(
            lap_timer
        )

        if DEBUG_MODE:
            telemetry_ok = False

            telemetry = PAIUdpTelemetry(
                lap_timer
            )

            telemetry_ok = telemetry.begin()

            if telemetry_ok:
                try:
                    if telemetry.sock is not None:
                        telemetry.sock.setblocking(
                            False
                        )
                except Exception:
                    pass
            else:
                telemetry = None

                try:
                    lap_timer.show(
                        "UDP FAILED",
                        "WiFi/IP check",
                        "No telemetry",
                        "",
                    )
                except Exception:
                    pass

            debug_socket = create_debug_socket()

            (
                ok,
                cost_ms,
            ) = send_debug_message(
                debug_socket,
                (
                    "type=boot,"
                    "telemetry_ok={},"
                    "mode=HIGH_SPEED_HARD_UTURN,"
                    "control_ms={},"
                    "base_speed={},"
                    "max_speed={},"
                    "kp={:.3f},"
                    "kd={:.3f},"
                    "slow_error={},"
                    "hard_error={},"
                    "hard_speed={},"
                    "uturn_after={},"
                    "uturn_speed={}"
                ).format(
                    telemetry_ok,
                    CONTROL_MS,
                    BASE_SPEED,
                    MAX_SPEED,
                    KP,
                    KD,
                    SLOW_ERROR,
                    HARD_ERROR,
                    HARD_CORNER_SPEED,
                    UTURN_ENABLE_AFTER_MS,
                    UTURN_SPEED,
                ),
            )

            if cost_ms > MAX_NETWORK_SEND_COST_MS:
                network_slow_count += 1
                network_cooldown_until_ms = (
                    ticks_ms()
                    + NETWORK_COOLDOWN_MS
                )

        self_calibrate_or_stop(
            line,
            motors,
            lap_timer,
        )

        show_running_mode(
            lap_timer
        )

        wait_button_start(
            button,
            lap_timer,
        )

        lap_timer.start()

        run_start_ms = ticks_ms()

        if telemetry is not None:
            telemetry.reset_timer()

        send_debug_message(
            debug_socket,
            "type=start",
        )

        previous_filtered_error = 0.0
        previous_filtered_d = 0.0

        current_speed = MIN_SPEED
        target_speed = BASE_SPEED

        steering = 0

        left_cmd = 0
        right_cmd = 0

        last_valid_error = 0.0
        lost_start_ms = None
        previous_on_line = True

        hard_corner_until_ms = 0
        uturn_until_ms = 0

        last_debug_report_ms = ticks_ms()

        finish_confirm_count = 0

        while True:
            loop_start = ticks_ms()

            loop_count += 1

            (
                error,
                position,
                norm,
                on_line,
                is_marker,
            ) = read_sensor_data(
                line
            )

            (
                active_count,
                sensor_sum,
                confidence,
            ) = count_active_sensors(
                norm
            )

            (
                filtered_error,
                filtered_d,
            ) = update_filters(
                error,
                on_line,
                previous_filtered_error,
                previous_filtered_d,
            )

            previous_filtered_error = filtered_error
            previous_filtered_d = filtered_d

            now_ms = ticks_ms()

            elapsed_run_ms = ticks_diff(
                now_ms,
                run_start_ms,
            )

            uturn_detection_enabled = (
                elapsed_run_ms >= UTURN_ENABLE_AFTER_MS
            )

            # HARD_CORNER는 초반부터 허용한다.
            if (
                on_line
                and detect_hard_corner_entry(
                    filtered_error,
                    filtered_d,
                    current_speed,
                )
            ):
                if ticks_diff(
                    hard_corner_until_ms,
                    now_ms,
                ) <= 0:
                    hard_corner_entry_count += 1

                hard_corner_until_ms = (
                    now_ms
                    + HARD_CORNER_HOLD_MS
                )

            # UTURN은 후반에서만 허용한다.
            if (
                on_line
                and uturn_detection_enabled
                and detect_uturn_entry(
                    filtered_error,
                    filtered_d,
                    current_speed,
                    norm,
                )
            ):
                if ticks_diff(
                    uturn_until_ms,
                    now_ms,
                ) <= 0:
                    uturn_entry_count += 1

                uturn_until_ms = (
                    now_ms
                    + UTURN_HOLD_MS
                )

            in_hard_corner = (
                ticks_diff(
                    hard_corner_until_ms,
                    now_ms,
                )
                > 0
            )

            in_uturn = (
                ticks_diff(
                    uturn_until_ms,
                    now_ms,
                )
                > 0
            )

            if elapsed_run_ms >= MIN_FINISH_MS:
                if lap_timer.check_finish(
                    norm,
                    on_line,
                ):
                    finish_confirm_count += 1
                else:
                    finish_confirm_count = 0
            else:
                finish_confirm_count = 0

            if finish_confirm_count >= FINISH_CONFIRM_COUNT:
                motors.stop()

                left_cmd = 0
                right_cmd = 0

                finished = True

                if telemetry is not None:
                    try:
                        now = ticks_ms()

                        if ticks_diff(
                            now,
                            network_cooldown_until_ms,
                        ) >= 0:
                            ok = telemetry.send_now(
                                current_speed,
                                norm,
                                position,
                                error,
                                int(filtered_d),
                                0,
                                0,
                                on_line,
                                is_marker,
                            )

                            if ok:
                                telemetry_sent_count += 1
                            else:
                                telemetry_skip_count += 1
                        else:
                            telemetry_skip_count += 1

                    except Exception:
                        telemetry_skip_count += 1

                (
                    ok,
                    cost_ms,
                ) = send_debug_message(
                    debug_socket,
                    (
                        "type=finish,"
                        "loops={},"
                        "speed={},"
                        "error={},"
                        "filtered_error={:.1f},"
                        "hard_entry={},"
                        "uturn_entry={}"
                    ).format(
                        loop_count,
                        current_speed,
                        error,
                        filtered_error,
                        hard_corner_entry_count,
                        uturn_entry_count,
                    ),
                )

                if cost_ms > MAX_NETWORK_SEND_COST_MS:
                    network_slow_count += 1

                break

            if on_line:
                last_valid_error = filtered_error
                lost_start_ms = None

                if in_uturn:
                    target_speed = UTURN_SPEED
                    current_speed = UTURN_SPEED

                    steering = calculate_steering(
                        filtered_error,
                        filtered_d,
                        UTURN_STEERING_LIMIT,
                        allow_straight_damping=False,
                    )

                    (
                        left_cmd,
                        right_cmd,
                    ) = mix_motor(
                        current_speed,
                        steering,
                        UTURN_INNER_FLOOR,
                        outer_boost=UTURN_OUTER_BOOST,
                    )

                elif in_hard_corner:
                    target_speed = HARD_CORNER_SPEED
                    current_speed = HARD_CORNER_SPEED

                    steering = calculate_steering(
                        filtered_error,
                        filtered_d,
                        HARD_CORNER_STEERING_LIMIT,
                        allow_straight_damping=False,
                    )

                    (
                        left_cmd,
                        right_cmd,
                    ) = mix_motor(
                        current_speed,
                        steering,
                        HARD_CORNER_INNER_FLOOR,
                        outer_boost=HARD_CORNER_OUTER_BOOST,
                    )

                else:
                    target_speed = calculate_target_speed(
                        filtered_error,
                        filtered_d,
                    )

                    current_speed = move_speed(
                        current_speed,
                        target_speed,
                    )

                    steering = calculate_steering(
                        filtered_error,
                        filtered_d,
                        STEERING_LIMIT,
                        allow_straight_damping=True,
                    )

                    (
                        left_cmd,
                        right_cmd,
                    ) = mix_motor(
                        current_speed,
                        steering,
                        INNER_FLOOR,
                        outer_boost=None,
                    )

                motors.drive(
                    left_cmd,
                    right_cmd,
                )

            else:
                if previous_on_line:
                    line_lost_count += 1
                    lost_start_ms = ticks_ms()

                if lost_start_ms is None:
                    lost_start_ms = ticks_ms()

                lost_elapsed_ms = ticks_diff(
                    ticks_ms(),
                    lost_start_ms,
                )

                (
                    left_cmd,
                    right_cmd,
                ) = calculate_recovery_drive(
                    last_valid_error,
                    lost_elapsed_ms,
                )

                target_speed = LOST_FORWARD
                current_speed = LOST_FORWARD
                steering = 0

                motors.drive(
                    left_cmd,
                    right_cmd,
                )

            previous_on_line = on_line

            compute_mid = ticks_ms()

            compute_before_network_ms = ticks_diff(
                compute_mid,
                loop_start,
            )

            if telemetry is not None:
                now = ticks_ms()

                if compute_before_network_ms >= TELEMETRY_SKIP_COMPUTE_MS:
                    telemetry_skip_count += 1

                elif ticks_diff(
                    now,
                    network_cooldown_until_ms,
                ) < 0:
                    telemetry_skip_count += 1

                else:
                    send_start = ticks_ms()

                    try:
                        ok = telemetry.send_if_due(
                            current_speed,
                            norm,
                            position,
                            error,
                            int(filtered_d),
                            left_cmd,
                            right_cmd,
                            on_line,
                            is_marker,
                        )

                        if ok:
                            telemetry_sent_count += 1

                    except Exception:
                        telemetry_skip_count += 1
                        ok = False

                    send_cost_ms = ticks_diff(
                        ticks_ms(),
                        send_start,
                    )

                    if send_cost_ms > MAX_NETWORK_SEND_COST_MS:
                        network_slow_count += 1
                        network_cooldown_until_ms = (
                            ticks_ms()
                            + NETWORK_COOLDOWN_MS
                        )

            lap_timer.update()

            compute_end = ticks_ms()

            last_loop_ms = ticks_diff(
                compute_end,
                loop_start,
            )

            total_compute_ms += last_loop_ms

            if last_loop_ms > max_loop_ms:
                max_loop_ms = last_loop_ms

            if last_loop_ms >= OVERRUN_THRESHOLD_MS:
                overrun_count += 1

            if DEBUG_MODE:
                now = ticks_ms()

                debug_elapsed_ms = ticks_diff(
                    now,
                    last_debug_report_ms,
                )

                if debug_elapsed_ms >= DEBUG_REPORT_MS:
                    if debug_elapsed_ms > DEBUG_REPORT_MAX_LAG_MS:
                        debug_skip_count += 1
                        last_debug_report_ms = now

                    elif ticks_diff(
                        now,
                        network_cooldown_until_ms,
                    ) < 0:
                        debug_skip_count += 1
                        last_debug_report_ms = now

                    else:
                        (
                            ok,
                            cost_ms,
                        ) = send_debug_report(
                            debug_socket,
                            loop_count,
                            overrun_count,
                            max_loop_ms,
                            last_loop_ms,
                            target_speed,
                            current_speed,
                            error,
                            filtered_error,
                            filtered_d,
                            confidence,
                            active_count,
                            steering,
                            left_cmd,
                            right_cmd,
                            on_line,
                            in_hard_corner,
                            in_uturn,
                            uturn_detection_enabled,
                        )

                        if cost_ms > MAX_NETWORK_SEND_COST_MS:
                            network_slow_count += 1
                            network_cooldown_until_ms = (
                                ticks_ms()
                                + NETWORK_COOLDOWN_MS
                            )

                        last_debug_report_ms = now

            wait_control_period(
                loop_start
            )

    except KeyboardInterrupt:
        send_debug_message(
            debug_socket,
            "type=stop,reason=keyboard_interrupt",
        )

    except Exception as exc:
        send_debug_message(
            debug_socket,
            (
                "type=error,"
                "exception={}"
            ).format(
                exc.__class__.__name__
            ),
        )

        if lap_timer is not None:
            try:
                lap_timer.show(
                    "ERROR",
                    exc.__class__.__name__[:16],
                    "Motor stopped",
                    "",
                )
            except Exception:
                pass

        raise

    finally:
        if motors is not None:
            motors.stop()

        if telemetry is not None:
            try:
                (
                    tele_sent,
                    tele_drop_lag,
                    tele_drop_busy,
                    tele_slow,
                ) = telemetry.get_stats()

                if tele_sent > telemetry_sent_count:
                    telemetry_sent_count = tele_sent

                telemetry_skip_count += (
                    tele_drop_lag
                    + tele_drop_busy
                )

                network_slow_count += tele_slow

            except Exception:
                pass

        send_final_report(
            debug_socket,
            finished,
            loop_count,
            overrun_count,
            max_loop_ms,
            total_compute_ms,
            line_lost_count,
            hard_corner_entry_count,
            uturn_entry_count,
            telemetry_sent_count,
            telemetry_skip_count,
            debug_skip_count,
            network_slow_count,
        )

        if telemetry is not None:
            telemetry.close()

        if debug_socket is not None:
            try:
                debug_socket.close()
            except Exception:
                pass

        if lap_timer is not None and not finished:
            try:
                lap_timer.show_stopped()
            except Exception:
                pass


# ============================================================
# Entry point
# ============================================================

run()
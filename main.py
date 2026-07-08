# main.py
# PAI-Car stabilized PD controller
#
# Features:
# - center deadband
# - filtered PD control
# - gain scheduling
# - curve-based speed planning
# - staged line-loss recovery
# - consecutive line reacquisition confirmation
# - filter reset after reacquisition
# - independent binary/text UDP options

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
# 1. Running mode
# ============================================================

DEBUG_MODE = True

# Binary telemetry is relatively lightweight.
ENABLE_BINARY_TELEMETRY = True

# Text formatting and UDP transmission caused long loop overruns.
# Keep this False for normal tuning runs.
ENABLE_TEXT_DEBUG = False

DEBUG_CONTROL_MS = 10
RACE_CONTROL_MS = 10

if DEBUG_MODE:
    CONTROL_PERIOD_MS = DEBUG_CONTROL_MS
else:
    CONTROL_PERIOD_MS = RACE_CONTROL_MS

run_support.CONTROL_MS = CONTROL_PERIOD_MS
udp_telemetry.CONTROL_MS = CONTROL_PERIOD_MS


# ============================================================
# 2. Wireless debug settings
# ============================================================

DEBUG_PC_PORT = PC_PORT + 1

# Only used when ENABLE_TEXT_DEBUG is True.
DEBUG_REPORT_MS = 3000

OVERRUN_THRESHOLD_MS = CONTROL_PERIOD_MS


# ============================================================
# 3. Speed planner settings
# ============================================================

MAX_TRACK_SPEED = 760
MEDIUM_CURVE_SPEED = 620
SHARP_CURVE_SPEED = 430
VERY_SHARP_SPEED = 300
LOW_CONFIDENCE_SPEED = 300

SPEED_RISE_STEP = 5
SPEED_FALL_STEP = 35

CURVE_SCORE_MEDIUM = 0.30
CURVE_SCORE_SHARP = 0.58
CURVE_SCORE_VERY_SHARP = 0.82

LOW_CONFIDENCE_THRESHOLD = 0.30

CURVE_ERROR_REFERENCE = 2200.0
CURVE_RATE_REFERENCE = 360.0
CURVE_ACCEL_REFERENCE = 160.0

CURVE_ERROR_WEIGHT = 0.60
CURVE_RATE_WEIGHT = 0.30
CURVE_ACCEL_WEIGHT = 0.10


# ============================================================
# 4. PD controller settings
# ============================================================

KP_STRAIGHT = 0.90
KP_CURVE = 0.80
KP_SHARP = 0.62

KD_STRAIGHT = 0.85
KD_CURVE = 0.75
KD_SHARP = 0.55

ERROR_DEADBAND = 90.0

MAX_STEERING_STRAIGHT = 360
MAX_STEERING_CURVE = 520
MAX_STEERING_SHARP = 680

STEERING_FILTER_ALPHA = 0.35


# ============================================================
# 5. Sensor settings
# ============================================================

SENSOR_ACTIVE_THRESHOLD = 300
SENSOR_SUM_FULL = 4000

WIDE_LINE_COUNT = 6
EDGE_SENSOR_COUNT = 2


# ============================================================
# 6. Filter settings
# ============================================================

ERROR_FILTER_ALPHA = 0.50
DERIVATIVE_FILTER_ALPHA = 0.22


# ============================================================
# 7. Motor mixer settings
# ============================================================

MOTOR_MAX_CMD = 1000

LEFT_GAIN = 1.00
RIGHT_GAIN = 1.00

LEFT_MIN_CMD = 0
RIGHT_MIN_CMD = 0

MOTOR_RISE_STEP = 120
MOTOR_FALL_STEP = 260

ALLOW_REVERSE_TRACKING = False


# ============================================================
# 8. Recovery settings
# ============================================================

# Keep previous commands for very short sensor dropouts.
LOST_GRACE_MS = 25

# Forward differential search.
SOFT_SEARCH_MS = 180

# One-wheel pivot search is allowed only until this time.
HARD_SEARCH_MS = 550

# Recovery motor commands.
RECOVERY_FORWARD_SPEED = 180
RECOVERY_TURN_SPEED = 330
RECOVERY_PIVOT_SPEED = 260

# After HARD_SEARCH_MS, stop rotating aggressively and move forward
# with a weaker steering difference.
RECOVERY_WIDE_INNER_SPEED = 130
RECOVERY_WIDE_OUTER_SPEED = 260

# A line must be detected this many consecutive loops before recovery
# is considered successful.
REACQUIRE_CONFIRM_COUNT = 5

# Reacquisition stabilization phase.
REACQUIRE_MS = 150
REACQUIRE_SPEED = 240
REACQUIRE_MAX_STEERING = 300

# While confirming the line, use a gentle correction.
REACQUIRE_CONFIRM_INNER_SPEED = 160
REACQUIRE_CONFIRM_OUTER_SPEED = 240


# ============================================================
# 9. States
# ============================================================

STATE_TRACKING = 0
STATE_LOST_GRACE = 1
STATE_SOFT_SEARCH = 2
STATE_HARD_SEARCH = 3
STATE_WIDE_SEARCH = 4
STATE_REACQUIRE_CONFIRM = 5
STATE_REACQUIRE = 6

SPEED_STATE_FAST = 0
SPEED_STATE_CURVE = 1
SPEED_STATE_SHARP = 2
SPEED_STATE_VERY_SHARP = 3
SPEED_STATE_LOW_CONFIDENCE = 4
SPEED_STATE_RECOVERY = 5


# ============================================================
# 10. Common helpers
# ============================================================

def clamp(
    value,
    minimum,
    maximum,
):
    if value < minimum:
        return minimum

    if value > maximum:
        return maximum

    return value


def move_toward(
    current,
    target,
    rise_step,
    fall_step,
):
    if target > current:
        return min(
            current + rise_step,
            target,
        )

    return max(
        current - fall_step,
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


# ============================================================
# 11. Sensor feature extraction
# ============================================================

def calculate_line_width(norm):
    longest = 0
    current = 0

    for value in norm:
        if (
            value
            >= SENSOR_ACTIVE_THRESHOLD
        ):
            current += 1

            if current > longest:
                longest = current
        else:
            current = 0

    return longest


def extract_features(
    error,
    on_line,
    norm,
    previous_filtered_error,
    previous_filtered_derivative,
):
    sensor_sum = 0
    active_count = 0

    for value in norm:
        sensor_sum += value

        if (
            value
            >= SENSOR_ACTIVE_THRESHOLD
        ):
            active_count += 1

    if SENSOR_SUM_FULL > 0:
        confidence = (
            sensor_sum
            / SENSOR_SUM_FULL
        )
    else:
        confidence = 0.0

    confidence = clamp(
        confidence,
        0.0,
        1.0,
    )

    if not on_line:
        confidence = 0.0

    if on_line:
        filtered_error = (
            previous_filtered_error
            + ERROR_FILTER_ALPHA
            * (
                error
                - previous_filtered_error
            )
        )
    else:
        filtered_error = (
            previous_filtered_error
        )

    # Normalize the derivative to a 10 ms reference period.
    period_scale = (
        10.0
        / CONTROL_PERIOD_MS
    )

    if on_line:
        raw_rate = (
            filtered_error
            - previous_filtered_error
        ) * period_scale
    else:
        raw_rate = 0.0

    filtered_error_rate = (
        previous_filtered_derivative
        + DERIVATIVE_FILTER_ALPHA
        * (
            raw_rate
            - previous_filtered_derivative
        )
    )

    error_accel = (
        filtered_error_rate
        - previous_filtered_derivative
    ) * period_scale

    edge_left = 0
    edge_right = 0

    sensor_count = len(norm)

    for index in range(
        EDGE_SENSOR_COUNT
    ):
        if index < sensor_count:
            edge_left += norm[index]

        right_index = (
            sensor_count
            - 1
            - index
        )

        if right_index >= 0:
            edge_right += norm[
                right_index
            ]

    line_width = calculate_line_width(
        norm
    )

    wide_line = (
        active_count
        >= WIDE_LINE_COUNT
    )

    return {
        "error": error,
        "filtered_error": filtered_error,
        "filtered_error_rate": (
            filtered_error_rate
        ),
        "error_accel": error_accel,
        "sensor_sum": sensor_sum,
        "confidence": confidence,
        "active_count": active_count,
        "line_width": line_width,
        "edge_left": edge_left,
        "edge_right": edge_right,
        "wide_line": wide_line,
    }


def create_empty_features():
    return {
        "error": 0,
        "filtered_error": 0.0,
        "filtered_error_rate": 0.0,
        "error_accel": 0.0,
        "sensor_sum": 0,
        "confidence": 0.0,
        "active_count": 0,
        "line_width": 0,
        "edge_left": 0,
        "edge_right": 0,
        "wide_line": False,
    }


def reset_features_after_reacquire(
    error,
    norm,
):
    """
    Rebuild features without carrying the old derivative or steering
    state across the recovery boundary.
    """

    return extract_features(
        error,
        True,
        norm,
        error,
        0.0,
    )


# ============================================================
# 12. Curve score and speed planner
# ============================================================

def calculate_curve_score(
    features
):
    error_component = clamp(
        abs(
            features[
                "filtered_error"
            ]
        )
        / CURVE_ERROR_REFERENCE,
        0.0,
        1.0,
    )

    rate_component = clamp(
        abs(
            features[
                "filtered_error_rate"
            ]
        )
        / CURVE_RATE_REFERENCE,
        0.0,
        1.0,
    )

    accel_component = clamp(
        abs(
            features[
                "error_accel"
            ]
        )
        / CURVE_ACCEL_REFERENCE,
        0.0,
        1.0,
    )

    curve_score = (
        CURVE_ERROR_WEIGHT
        * error_component
        + CURVE_RATE_WEIGHT
        * rate_component
        + CURVE_ACCEL_WEIGHT
        * accel_component
    )

    return clamp(
        curve_score,
        0.0,
        1.0,
    )


def calculate_target_speed(
    curve_score,
    features,
    on_line,
):
    if not on_line:
        return (
            RECOVERY_FORWARD_SPEED,
            SPEED_STATE_RECOVERY,
        )

    if (
        features["confidence"]
        < LOW_CONFIDENCE_THRESHOLD
    ):
        return (
            LOW_CONFIDENCE_SPEED,
            SPEED_STATE_LOW_CONFIDENCE,
        )

    if (
        curve_score
        >= CURVE_SCORE_VERY_SHARP
    ):
        return (
            VERY_SHARP_SPEED,
            SPEED_STATE_VERY_SHARP,
        )

    if (
        curve_score
        >= CURVE_SCORE_SHARP
    ):
        return (
            SHARP_CURVE_SPEED,
            SPEED_STATE_SHARP,
        )

    if (
        curve_score
        >= CURVE_SCORE_MEDIUM
    ):
        return (
            MEDIUM_CURVE_SPEED,
            SPEED_STATE_CURVE,
        )

    return (
        MAX_TRACK_SPEED,
        SPEED_STATE_FAST,
    )


def get_speed_state_name(
    speed_state
):
    if (
        speed_state
        == SPEED_STATE_FAST
    ):
        return "FAST"

    if (
        speed_state
        == SPEED_STATE_CURVE
    ):
        return "CURVE"

    if (
        speed_state
        == SPEED_STATE_SHARP
    ):
        return "SHARP"

    if (
        speed_state
        == SPEED_STATE_VERY_SHARP
    ):
        return "VSHARP"

    if (
        speed_state
        == SPEED_STATE_LOW_CONFIDENCE
    ):
        return "LOWCONF"

    return "RECOVERY"


# ============================================================
# 13. Gain scheduling and steering
# ============================================================

def get_control_gains(
    curve_score
):
    if (
        curve_score
        >= CURVE_SCORE_SHARP
    ):
        return (
            KP_SHARP,
            KD_SHARP,
            MAX_STEERING_SHARP,
        )

    if (
        curve_score
        >= CURVE_SCORE_MEDIUM
    ):
        return (
            KP_CURVE,
            KD_CURVE,
            MAX_STEERING_CURVE,
        )

    return (
        KP_STRAIGHT,
        KD_STRAIGHT,
        MAX_STEERING_STRAIGHT,
    )


def apply_error_deadband(error):
    if (
        abs(error)
        <= ERROR_DEADBAND
    ):
        return 0.0

    if error > 0:
        return (
            error
            - ERROR_DEADBAND
        )

    return (
        error
        + ERROR_DEADBAND
    )


def calculate_steering(
    features,
    curve_score,
    previous_steering,
):
    (
        kp,
        kd,
        max_steering,
    ) = get_control_gains(
        curve_score
    )

    control_error = (
        apply_error_deadband(
            features[
                "filtered_error"
            ]
        )
    )

    p_term = (
        kp
        * control_error
    )

    d_term = (
        kd
        * features[
            "filtered_error_rate"
        ]
    )

    raw_steering = clamp(
        p_term + d_term,
        -max_steering,
        max_steering,
    )

    filtered_steering = (
        previous_steering
        + STEERING_FILTER_ALPHA
        * (
            raw_steering
            - previous_steering
        )
    )

    return (
        int(filtered_steering),
        p_term,
        d_term,
        kp,
        kd,
    )


# ============================================================
# 14. Motor mixer
# ============================================================

def apply_motor_deadzone(
    command,
    minimum_command,
):
    if command == 0:
        return 0.0

    if minimum_command <= 0:
        return command

    magnitude = clamp(
        abs(command),
        0,
        MOTOR_MAX_CMD,
    )

    corrected_magnitude = (
        minimum_command
        + magnitude
        * (
            MOTOR_MAX_CMD
            - minimum_command
        )
        / MOTOR_MAX_CMD
    )

    if command < 0:
        return (
            -corrected_magnitude
        )

    return corrected_magnitude


def preserve_ratio_saturation(
    left_cmd,
    right_cmd,
):
    peak = max(
        abs(left_cmd),
        abs(right_cmd),
    )

    if peak <= MOTOR_MAX_CMD:
        return (
            left_cmd,
            right_cmd,
            1.0,
        )

    scale = (
        MOTOR_MAX_CMD
        / peak
    )

    return (
        left_cmd * scale,
        right_cmd * scale,
        scale,
    )


def slew_limit(
    target_cmd,
    previous_cmd,
    rise_step,
    fall_step,
):
    delta = (
        target_cmd
        - previous_cmd
    )

    if (
        abs(target_cmd)
        > abs(previous_cmd)
    ):
        step_limit = rise_step
    else:
        step_limit = fall_step

    if delta > step_limit:
        return (
            previous_cmd
            + step_limit
        )

    if delta < -step_limit:
        return (
            previous_cmd
            - step_limit
        )

    return target_cmd


def mix_motor_commands(
    base_speed,
    steering,
    previous_left_cmd,
    previous_right_cmd,
    emergency=False,
):
    left_cmd = (
        base_speed
        + steering
    )

    right_cmd = (
        base_speed
        - steering
    )

    if not ALLOW_REVERSE_TRACKING:
        if left_cmd < 0:
            left_cmd = 0

        if right_cmd < 0:
            right_cmd = 0

    left_cmd *= LEFT_GAIN
    right_cmd *= RIGHT_GAIN

    left_cmd = apply_motor_deadzone(
        left_cmd,
        LEFT_MIN_CMD,
    )

    right_cmd = apply_motor_deadzone(
        right_cmd,
        RIGHT_MIN_CMD,
    )

    (
        left_cmd,
        right_cmd,
        saturation_scale,
    ) = preserve_ratio_saturation(
        left_cmd,
        right_cmd,
    )

    if not emergency:
        period_scale = (
            CONTROL_PERIOD_MS
            / 10.0
        )

        left_cmd = slew_limit(
            left_cmd,
            previous_left_cmd,
            MOTOR_RISE_STEP
            * period_scale,
            MOTOR_FALL_STEP
            * period_scale,
        )

        right_cmd = slew_limit(
            right_cmd,
            previous_right_cmd,
            MOTOR_RISE_STEP
            * period_scale,
            MOTOR_FALL_STEP
            * period_scale,
        )

    left_cmd = limit_cmd(
        int(left_cmd)
    )

    right_cmd = limit_cmd(
        int(right_cmd)
    )

    return (
        left_cmd,
        right_cmd,
        saturation_scale,
    )


# ============================================================
# 15. Line-loss recovery
# ============================================================

def calculate_recovery_commands(
    lost_elapsed_ms,
    last_valid_error,
    previous_left_cmd,
    previous_right_cmd,
):
    # Brief sensor dropout: keep the previous motor commands.
    if (
        lost_elapsed_ms
        <= LOST_GRACE_MS
    ):
        return (
            STATE_LOST_GRACE,
            previous_left_cmd,
            previous_right_cmd,
        )

    # Soft forward differential search.
    if (
        lost_elapsed_ms
        <= SOFT_SEARCH_MS
    ):
        if last_valid_error < 0:
            return (
                STATE_SOFT_SEARCH,
                RECOVERY_FORWARD_SPEED,
                RECOVERY_TURN_SPEED,
            )

        return (
            STATE_SOFT_SEARCH,
            RECOVERY_TURN_SPEED,
            RECOVERY_FORWARD_SPEED,
        )

    # Strong pivot search, but only for a limited duration.
    if (
        lost_elapsed_ms
        <= HARD_SEARCH_MS
    ):
        if last_valid_error < 0:
            return (
                STATE_HARD_SEARCH,
                0,
                RECOVERY_PIVOT_SPEED,
            )

        return (
            STATE_HARD_SEARCH,
            RECOVERY_PIVOT_SPEED,
            0,
        )

    # After the pivot timeout, avoid continuing to rotate indefinitely.
    # Move forward with a weaker steering difference.
    if last_valid_error < 0:
        return (
            STATE_WIDE_SEARCH,
            RECOVERY_WIDE_INNER_SPEED,
            RECOVERY_WIDE_OUTER_SPEED,
        )

    return (
        STATE_WIDE_SEARCH,
        RECOVERY_WIDE_OUTER_SPEED,
        RECOVERY_WIDE_INNER_SPEED,
    )


def calculate_reacquire_confirm_commands(
    error
):
    """
    Use the currently detected error only for a gentle confirmation
    movement. Normal PD control resumes only after consecutive
    detections have been confirmed.
    """

    if error < 0:
        return (
            REACQUIRE_CONFIRM_INNER_SPEED,
            REACQUIRE_CONFIRM_OUTER_SPEED,
        )

    return (
        REACQUIRE_CONFIRM_OUTER_SPEED,
        REACQUIRE_CONFIRM_INNER_SPEED,
    )


# ============================================================
# 16. Wireless debug UDP
# ============================================================

def create_debug_socket():
    if not ENABLE_TEXT_DEBUG:
        return None

    try:
        debug_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )

        try:
            debug_socket.setblocking(
                False
            )
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
        return False

    try:
        debug_socket.sendto(
            message.encode(),
            (
                PC_IP,
                DEBUG_PC_PORT,
            ),
        )

        return True

    except OSError:
        return False


def send_debug_report(
    debug_socket,
    loop_count,
    overrun_count,
    max_loop_ms,
    last_loop_ms,
    drive_state,
    recovering,
    reacquire_count,
    features,
    target_speed,
    current_speed,
    speed_state,
    curve_score,
    steering,
    p_term,
    d_term,
    kp,
    kd,
    saturation_scale,
    left_cmd,
    right_cmd,
):
    if debug_socket is None:
        return

    message = (
        "type=status,"
        "loop={},"
        "drive={},"
        "recovering={},"
        "reacquire_count={},"
        "speed_state={},"
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
        "wide={},"
        "curve={:.3f},"
        "kp={:.3f},"
        "kd={:.3f},"
        "p={:.1f},"
        "d={:.1f},"
        "steering={},"
        "scale={:.3f},"
        "left={},"
        "right={}"
    ).format(
        loop_count,
        drive_state,
        int(recovering),
        reacquire_count,
        get_speed_state_name(
            speed_state
        ),
        target_speed,
        current_speed,
        last_loop_ms,
        max_loop_ms,
        overrun_count,
        features["error"],
        features[
            "filtered_error"
        ],
        features[
            "filtered_error_rate"
        ],
        features["confidence"],
        features["active_count"],
        features["wide_line"],
        curve_score,
        kp,
        kd,
        p_term,
        d_term,
        steering,
        saturation_scale,
        left_cmd,
        right_cmd,
    )

    send_debug_message(
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
        "mode={},"
        "control_ms={},"
        "loops={},"
        "average_compute_ms={:.3f},"
        "max_compute_ms={},"
        "overrun_count={},"
        "overrun_rate={:.2f},"
        "line_lost_entry={}"
    ).format(
        finished,
        (
            "DEBUG"
            if DEBUG_MODE
            else "RACE"
        ),
        CONTROL_PERIOD_MS,
        loop_count,
        average_compute_ms,
        max_loop_ms,
        overrun_count,
        overrun_rate,
        line_lost_count,
    )

    send_debug_message(
        debug_socket,
        message,
    )


# ============================================================
# 17. OLED
# ============================================================

def show_running_mode(lap_timer):
    if DEBUG_MODE:
        if ENABLE_TEXT_DEBUG:
            mode_name = "DEBUG FULL"
        elif ENABLE_BINARY_TELEMETRY:
            mode_name = "DEBUG BIN"
        else:
            mode_name = "DEBUG OFF"
    else:
        mode_name = "RACE"

    try:
        lap_timer.show(
            mode_name,
            "{} ms".format(
                CONTROL_PERIOD_MS
            ),
            "Stable PD",
            "",
        )
    except Exception:
        pass


# ============================================================
# 18. Main
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

    wifi_enabled = (
        DEBUG_MODE
        and (
            ENABLE_BINARY_TELEMETRY
            or ENABLE_TEXT_DEBUG
        )
    )

    try:
        lap_timer = create_lap_timer()

        (
            line,
            motors,
            button,
        ) = setup_paicar(
            lap_timer
        )

        if wifi_enabled:
            telemetry = PAIUdpTelemetry(
                lap_timer
            )

            telemetry.begin()

            if ENABLE_TEXT_DEBUG:
                debug_socket = (
                    create_debug_socket()
                )

                send_debug_message(
                    debug_socket,
                    (
                        "type=boot,"
                        "control_ms={},"
                        "max_speed={},"
                        "kp_straight={:.3f},"
                        "kd_straight={:.3f},"
                        "binary={},"
                        "text={}"
                    ).format(
                        CONTROL_PERIOD_MS,
                        MAX_TRACK_SPEED,
                        KP_STRAIGHT,
                        KD_STRAIGHT,
                        int(
                            ENABLE_BINARY_TELEMETRY
                        ),
                        int(
                            ENABLE_TEXT_DEBUG
                        ),
                    ),
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

        if telemetry is not None:
            telemetry.reset_timer()

        send_debug_message(
            debug_socket,
            "type=start",
        )

        features = (
            create_empty_features()
        )

        previous_filtered_error = 0.0
        previous_filtered_derivative = 0.0

        previous_left_cmd = 0
        previous_right_cmd = 0
        previous_steering = 0

        current_speed = (
            VERY_SHARP_SPEED
        )

        target_speed = (
            VERY_SHARP_SPEED
        )

        curve_score = 0.0

        speed_state = (
            SPEED_STATE_VERY_SHARP
        )

        drive_state = (
            STATE_TRACKING
        )

        p_term = 0.0
        d_term = 0.0

        kp = KP_STRAIGHT
        kd = KD_STRAIGHT

        steering = 0
        saturation_scale = 1.0

        left_cmd = 0
        right_cmd = 0

        last_valid_error = 0.0

        recovering = False
        lost_start_ms = None

        reacquire_count = 0
        reacquire_start_ms = None

        last_debug_report_ms = (
            ticks_ms()
        )

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

            if lap_timer.check_finish(
                norm,
                on_line,
            ):
                motors.stop()

                left_cmd = 0
                right_cmd = 0

                previous_left_cmd = 0
                previous_right_cmd = 0

                finished = True

                if (
                    telemetry is not None
                    and ENABLE_BINARY_TELEMETRY
                ):
                    telemetry.send_now(
                        current_speed,
                        norm,
                        position,
                        error,
                        0,
                        0,
                        0,
                        on_line,
                        is_marker,
                    )

                send_debug_message(
                    debug_socket,
                    (
                        "type=finish,"
                        "loops={},"
                        "speed={},"
                        "error={},"
                        "curve={:.3f}"
                    ).format(
                        loop_count,
                        current_speed,
                        error,
                        curve_score,
                    ),
                )

                break

            # ------------------------------------------------
            # A. Start a physical recovery event
            # ------------------------------------------------

            if (
                not recovering
                and not on_line
            ):
                recovering = True

                line_lost_count += 1

                lost_start_ms = (
                    ticks_ms()
                )

                reacquire_count = 0
                reacquire_start_ms = None

            # ------------------------------------------------
            # B. Recovery and reacquisition confirmation
            # ------------------------------------------------

            if recovering:
                if on_line:
                    reacquire_count += 1
                else:
                    reacquire_count = 0

                if (
                    on_line
                    and reacquire_count
                    >= REACQUIRE_CONFIRM_COUNT
                ):
                    # Confirmed reacquisition.
                    recovering = False

                    drive_state = (
                        STATE_REACQUIRE
                    )

                    reacquire_start_ms = (
                        ticks_ms()
                    )

                    lost_start_ms = None
                    reacquire_count = 0

                    # Reset stale controller state.
                    previous_filtered_error = (
                        error
                    )

                    previous_filtered_derivative = (
                        0.0
                    )

                    previous_steering = 0

                    features = (
                        reset_features_after_reacquire(
                            error,
                            norm,
                        )
                    )

                    previous_filtered_error = (
                        features[
                            "filtered_error"
                        ]
                    )

                    previous_filtered_derivative = (
                        features[
                            "filtered_error_rate"
                        ]
                    )

                    last_valid_error = (
                        features[
                            "filtered_error"
                        ]
                    )

                else:
                    # Recovery is still active.
                    if (
                        lost_start_ms
                        is None
                    ):
                        lost_start_ms = (
                            ticks_ms()
                        )

                    lost_elapsed_ms = (
                        ticks_diff(
                            ticks_ms(),
                            lost_start_ms,
                        )
                    )

                    if on_line:
                        drive_state = (
                            STATE_REACQUIRE_CONFIRM
                        )

                        (
                            left_cmd,
                            right_cmd,
                        ) = (
                            calculate_reacquire_confirm_commands(
                                error
                            )
                        )
                    else:
                        (
                            drive_state,
                            left_cmd,
                            right_cmd,
                        ) = calculate_recovery_commands(
                            lost_elapsed_ms,
                            last_valid_error,
                            previous_left_cmd,
                            previous_right_cmd,
                        )

                    target_speed = (
                        RECOVERY_FORWARD_SPEED
                    )

                    current_speed = (
                        RECOVERY_FORWARD_SPEED
                    )

                    speed_state = (
                        SPEED_STATE_RECOVERY
                    )

                    steering = 0
                    p_term = 0.0
                    d_term = 0.0
                    kp = 0.0
                    kd = 0.0

                    saturation_scale = 1.0

                    motors.drive(
                        left_cmd,
                        right_cmd,
                    )

            # ------------------------------------------------
            # C. Normal tracking or stabilized reacquisition
            # ------------------------------------------------

            if not recovering:
                if (
                    drive_state
                    != STATE_REACQUIRE
                ):
                    features = extract_features(
                        error,
                        on_line,
                        norm,
                        previous_filtered_error,
                        previous_filtered_derivative,
                    )

                    previous_filtered_error = (
                        features[
                            "filtered_error"
                        ]
                    )

                    previous_filtered_derivative = (
                        features[
                            "filtered_error_rate"
                        ]
                    )

                last_valid_error = (
                    features[
                        "filtered_error"
                    ]
                )

                curve_score = (
                    calculate_curve_score(
                        features
                    )
                )

                (
                    target_speed,
                    speed_state,
                ) = calculate_target_speed(
                    curve_score,
                    features,
                    True,
                )

                in_reacquire_phase = False

                if (
                    reacquire_start_ms
                    is not None
                ):
                    reacquire_elapsed_ms = (
                        ticks_diff(
                            ticks_ms(),
                            reacquire_start_ms,
                        )
                    )

                    if (
                        reacquire_elapsed_ms
                        < REACQUIRE_MS
                    ):
                        in_reacquire_phase = (
                            True
                        )

                        drive_state = (
                            STATE_REACQUIRE
                        )

                        target_speed = min(
                            target_speed,
                            REACQUIRE_SPEED,
                        )
                    else:
                        reacquire_start_ms = (
                            None
                        )

                        drive_state = (
                            STATE_TRACKING
                        )
                else:
                    drive_state = (
                        STATE_TRACKING
                    )

                if features["wide_line"]:
                    steering = int(
                        previous_steering
                        * 0.8
                    )

                    p_term = 0.0
                    d_term = 0.0
                    kp = 0.0
                    kd = 0.0
                else:
                    (
                        steering,
                        p_term,
                        d_term,
                        kp,
                        kd,
                    ) = calculate_steering(
                        features,
                        curve_score,
                        previous_steering,
                    )

                if in_reacquire_phase:
                    steering = int(
                        clamp(
                            steering,
                            -REACQUIRE_MAX_STEERING,
                            REACQUIRE_MAX_STEERING,
                        )
                    )

                current_speed = move_toward(
                    current_speed,
                    target_speed,
                    SPEED_RISE_STEP,
                    SPEED_FALL_STEP,
                )

                emergency = (
                    speed_state
                    == SPEED_STATE_VERY_SHARP
                )

                (
                    left_cmd,
                    right_cmd,
                    saturation_scale,
                ) = mix_motor_commands(
                    current_speed,
                    steering,
                    previous_left_cmd,
                    previous_right_cmd,
                    emergency=emergency,
                )

                motors.drive(
                    left_cmd,
                    right_cmd,
                )

                previous_steering = (
                    steering
                )

            previous_left_cmd = left_cmd
            previous_right_cmd = right_cmd

            # ------------------------------------------------
            # D. Binary telemetry
            # ------------------------------------------------

            if (
                telemetry is not None
                and ENABLE_BINARY_TELEMETRY
            ):
                telemetry.send_if_due(
                    current_speed,
                    norm,
                    position,
                    error,
                    int(
                        features[
                            "filtered_error_rate"
                        ]
                    ),
                    left_cmd,
                    right_cmd,
                    on_line,
                    is_marker,
                )

            lap_timer.update()

            # ------------------------------------------------
            # E. Loop execution-time measurement
            # ------------------------------------------------

            compute_end = ticks_ms()

            last_loop_ms = ticks_diff(
                compute_end,
                loop_start,
            )

            total_compute_ms += (
                last_loop_ms
            )

            if (
                last_loop_ms
                > max_loop_ms
            ):
                max_loop_ms = (
                    last_loop_ms
                )

            if (
                last_loop_ms
                >= OVERRUN_THRESHOLD_MS
            ):
                overrun_count += 1

            # ------------------------------------------------
            # F. Optional text debug
            # ------------------------------------------------

            if (
                DEBUG_MODE
                and ENABLE_TEXT_DEBUG
            ):
                now = ticks_ms()

                if ticks_diff(
                    now,
                    last_debug_report_ms,
                ) >= DEBUG_REPORT_MS:

                    send_debug_report(
                        debug_socket,
                        loop_count,
                        overrun_count,
                        max_loop_ms,
                        last_loop_ms,
                        drive_state,
                        recovering,
                        reacquire_count,
                        features,
                        target_speed,
                        current_speed,
                        speed_state,
                        curve_score,
                        steering,
                        p_term,
                        d_term,
                        kp,
                        kd,
                        saturation_scale,
                        left_cmd,
                        right_cmd,
                    )

                    last_debug_report_ms = (
                        now
                    )

            wait_control_period(
                loop_start
            )

    except KeyboardInterrupt:
        send_debug_message(
            debug_socket,
            (
                "type=stop,"
                "reason=keyboard_interrupt"
            ),
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
                    exc.__class__.__name__[
                        :16
                    ],
                    "Motor stopped",
                    "",
                )
            except Exception:
                pass

        raise

    finally:
        if motors is not None:
            motors.stop()

        send_final_report(
            debug_socket,
            finished,
            loop_count,
            overrun_count,
            max_loop_ms,
            total_compute_ms,
            line_lost_count,
        )

        if telemetry is not None:
            telemetry.close()

        if debug_socket is not None:
            try:
                debug_socket.close()
            except Exception:
                pass

        if (
            lap_timer is not None
            and not finished
        ):
            try:
                lap_timer.show_stopped()
            except Exception:
                pass


# ============================================================
# 19. Program entry point
# ============================================================

run()
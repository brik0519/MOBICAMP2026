# main.py
# PAI-Car filtered PD + motor mixer + curve speed planner
# + wireless UDP debug report

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

DEBUG_CONTROL_MS = 10
RACE_CONTROL_MS = 5

if DEBUG_MODE:
    CONTROL_PERIOD_MS = DEBUG_CONTROL_MS
else:
    CONTROL_PERIOD_MS = RACE_CONTROL_MS

run_support.CONTROL_MS = CONTROL_PERIOD_MS
udp_telemetry.CONTROL_MS = CONTROL_PERIOD_MS


# ============================================================
# 2. Wireless debug settings
# ============================================================

# 기존 바이너리 텔레메트리 포트와 분리
DEBUG_PC_PORT = PC_PORT + 1

# 사람이 읽는 디버그 상태 전송 주기
DEBUG_REPORT_MS = 1000

OVERRUN_THRESHOLD_MS = CONTROL_PERIOD_MS


# ============================================================
# 3. Speed planner settings
# ============================================================

MAX_STRAIGHT_SPEED = 550
CURVE_SPEED = 420
SHARP_CURVE_SPEED = 300
LOW_CONFIDENCE_SPEED = 260
LINE_LOST_SPEED = 0

CURVE_THRESHOLD = 0.20
SHARP_CURVE_THRESHOLD = 0.48

LOW_CONFIDENCE_THRESHOLD = 0.35

CURVE_ERROR_REFERENCE = 3000.0
CURVE_RATE_REFERENCE = 450.0
CURVE_ACCEL_REFERENCE = 180.0

CURVE_ERROR_WEIGHT = 0.55
CURVE_RATE_WEIGHT = 0.30
CURVE_ACCEL_WEIGHT = 0.15
CURVE_CONFIDENCE_WEIGHT = 0.25


# ============================================================
# 4. Filtered PD settings
# ============================================================

KP = 0.55
KD = 0.35

MAX_STEERING = 500


# ============================================================
# 5. Sensor feature settings
# ============================================================

SENSOR_ACTIVE_THRESHOLD = 300
SENSOR_SUM_FULL = 4000

EDGE_SENSOR_COUNT = 2
WIDE_LINE_COUNT = 6


# ============================================================
# 6. Filter settings
# ============================================================

ERROR_FILTER_ALPHA = 0.65
DERIVATIVE_FILTER_ALPHA = 0.30


# ============================================================
# 7. Motor mixer settings
# ============================================================

MOTOR_MAX_CMD = 1000

LEFT_GAIN = 1.00
RIGHT_GAIN = 1.00

LEFT_MIN_CMD = 0
RIGHT_MIN_CMD = 0

MOTOR_RISE_STEP = 35
MOTOR_FALL_STEP = 90

ALLOW_REVERSE_TRACKING = False


# ============================================================
# 8. States
# ============================================================

STATE_TRACKING = 0
STATE_LINE_LOST = 1

SPEED_STATE_STRAIGHT = 0
SPEED_STATE_CURVE = 1
SPEED_STATE_SHARP_CURVE = 2
SPEED_STATE_LOW_CONFIDENCE = 3
SPEED_STATE_LINE_LOST = 4


# ============================================================
# 9. Common helpers
# ============================================================

def clamp(value, minimum, maximum):
    if value < minimum:
        return minimum

    if value > maximum:
        return maximum

    return value


def read_sensor_data(line):
    error, position, norm, on_line = read_line_detail(line)

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
# 10. Sensor feature extraction
# ============================================================

def calculate_line_width(norm):
    first_active = -1
    last_active = -1

    for index in range(len(norm)):
        if norm[index] >= SENSOR_ACTIVE_THRESHOLD:
            if first_active < 0:
                first_active = index

            last_active = index

    if first_active < 0:
        return 0

    return last_active - first_active + 1


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

        if value >= SENSOR_ACTIVE_THRESHOLD:
            active_count += 1

    if SENSOR_SUM_FULL > 0:
        confidence = sensor_sum / SENSOR_SUM_FULL
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
        filtered_error = previous_filtered_error

    if on_line:
        error_rate = (
            filtered_error
            - previous_filtered_error
        )
    else:
        error_rate = 0.0

    filtered_error_rate = (
        previous_filtered_derivative
        + DERIVATIVE_FILTER_ALPHA
        * (
            error_rate
            - previous_filtered_derivative
        )
    )

    error_accel = (
        filtered_error_rate
        - previous_filtered_derivative
    )

    line_width = calculate_line_width(norm)

    edge_left = 0
    edge_right = 0

    sensor_count = len(norm)

    for index in range(EDGE_SENSOR_COUNT):
        if index < sensor_count:
            edge_left += norm[index]

        right_index = sensor_count - 1 - index

        if right_index >= 0:
            edge_right += norm[right_index]

    wide_line = (
        active_count >= WIDE_LINE_COUNT
    )

    return {
        "error": error,
        "filtered_error": filtered_error,
        "error_rate": error_rate,
        "filtered_error_rate": filtered_error_rate,
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
        "error_rate": 0.0,
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


# ============================================================
# 11. Curve-risk speed planner
# ============================================================

def calculate_curve_score(features):
    error_component = (
        abs(features["filtered_error"])
        / CURVE_ERROR_REFERENCE
    )

    rate_component = (
        abs(features["filtered_error_rate"])
        / CURVE_RATE_REFERENCE
    )

    accel_component = (
        abs(features["error_accel"])
        / CURVE_ACCEL_REFERENCE
    )

    error_component = clamp(
        error_component,
        0.0,
        1.0,
    )

    rate_component = clamp(
        rate_component,
        0.0,
        1.0,
    )

    accel_component = clamp(
        accel_component,
        0.0,
        1.0,
    )

    confidence_component = clamp(
        1.0 - features["confidence"],
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
        + CURVE_CONFIDENCE_WEIGHT
        * confidence_component
    )

    curve_score = clamp(
        curve_score,
        0.0,
        1.0,
    )

    return (
        curve_score,
        error_component,
        rate_component,
        accel_component,
        confidence_component,
    )


def calculate_target_speed(
    curve_score,
    features,
    on_line,
):
    if not on_line:
        return (
            LINE_LOST_SPEED,
            SPEED_STATE_LINE_LOST,
        )

    if (
        features["confidence"]
        < LOW_CONFIDENCE_THRESHOLD
    ):
        return (
            LOW_CONFIDENCE_SPEED,
            SPEED_STATE_LOW_CONFIDENCE,
        )

    if curve_score >= SHARP_CURVE_THRESHOLD:
        return (
            SHARP_CURVE_SPEED,
            SPEED_STATE_SHARP_CURVE,
        )

    if curve_score >= CURVE_THRESHOLD:
        return (
            CURVE_SPEED,
            SPEED_STATE_CURVE,
        )

    return (
        MAX_STRAIGHT_SPEED,
        SPEED_STATE_STRAIGHT,
    )


def get_speed_state_name(speed_state):
    if speed_state == SPEED_STATE_STRAIGHT:
        return "STRAIGHT"

    if speed_state == SPEED_STATE_CURVE:
        return "CURVE"

    if speed_state == SPEED_STATE_SHARP_CURVE:
        return "SHARP"

    if speed_state == SPEED_STATE_LOW_CONFIDENCE:
        return "LOW_CONF"

    return "LOST"


# ============================================================
# 12. Filtered PD controller
# ============================================================

def calculate_steering(
    filtered_error,
    filtered_error_rate,
):
    p_term = (
        KP
        * filtered_error
    )

    d_term = (
        KD
        * filtered_error_rate
    )

    steering = int(
        p_term
        + d_term
    )

    steering = clamp(
        steering,
        -MAX_STEERING,
        MAX_STEERING,
    )

    return (
        steering,
        p_term,
        d_term,
    )


def calculate_pd_control(features):
    return calculate_steering(
        features["filtered_error"],
        features["filtered_error_rate"],
    )


# ============================================================
# 13. Motor mixer
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
        return -corrected_magnitude

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

    if abs(target_cmd) > abs(previous_cmd):
        step_limit = rise_step
    else:
        step_limit = fall_step

    if delta > step_limit:
        return previous_cmd + step_limit

    if delta < -step_limit:
        return previous_cmd - step_limit

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
        left_cmd = slew_limit(
            left_cmd,
            previous_left_cmd,
            MOTOR_RISE_STEP,
            MOTOR_FALL_STEP,
        )

        right_cmd = slew_limit(
            right_cmd,
            previous_right_cmd,
            MOTOR_RISE_STEP,
            MOTOR_FALL_STEP,
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
# 14. Wireless debug UDP
# ============================================================

def create_debug_socket():
    """
    telemetry.begin()으로 Wi-Fi 연결이 완료된 뒤 호출한다.
    """
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
    """
    디버그 문자열을 별도 UDP 포트로 전송한다.

    송신 실패가 주행 제어를 중단시키지 않도록
    모든 네트워크 오류를 무시한다.
    """
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
    line_lost_count,
    line_lost_loop_count,
    drive_state,
    features,
    target_speed,
    speed_state,
    curve_score,
    steering,
    p_term,
    d_term,
    saturation_scale,
    left_cmd,
    right_cmd,
):
    if debug_socket is None:
        return

    if drive_state == STATE_TRACKING:
        drive_name = "TRACK"
    else:
        drive_name = "LOST"

    speed_name = get_speed_state_name(
        speed_state
    )

    message = (
        "type=status,"
        "loop={},"
        "drive={},"
        "speed_state={},"
        "target={},"
        "compute_ms={},"
        "max_ms={},"
        "overrun={},"
        "lost_entry={},"
        "lost_loop={},"
        "error={},"
        "filtered_error={:.1f},"
        "filtered_rate={:.1f},"
        "error_accel={:.1f},"
        "sensor_sum={},"
        "confidence={:.2f},"
        "active={},"
        "width={},"
        "wide={},"
        "curve={:.3f},"
        "p={:.1f},"
        "d={:.1f},"
        "steering={},"
        "scale={:.3f},"
        "left={},"
        "right={}"
    ).format(
        loop_count,
        drive_name,
        speed_name,
        target_speed,
        last_loop_ms,
        max_loop_ms,
        overrun_count,
        line_lost_count,
        line_lost_loop_count,
        features["error"],
        features["filtered_error"],
        features["filtered_error_rate"],
        features["error_accel"],
        features["sensor_sum"],
        features["confidence"],
        features["active_count"],
        features["line_width"],
        features["wide_line"],
        curve_score,
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
    line_lost_loop_count,
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
        "line_lost_entry={},"
        "line_lost_loops={}"
    ).format(
        finished,
        "DEBUG" if DEBUG_MODE else "RACE",
        CONTROL_PERIOD_MS,
        loop_count,
        average_compute_ms,
        max_loop_ms,
        overrun_count,
        overrun_rate,
        line_lost_count,
        line_lost_loop_count,
    )

    send_debug_message(
        debug_socket,
        message,
    )


# ============================================================
# 15. OLED
# ============================================================

def show_running_mode(lap_timer):
    if DEBUG_MODE:
        mode_name = "DEBUG UDP"
    else:
        mode_name = "RACE"

    try:
        lap_timer.show(
            mode_name,
            "{} ms".format(
                CONTROL_PERIOD_MS
            ),
            "Curve Speed",
            "",
        )
    except Exception:
        pass


# ============================================================
# 16. Main
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
    line_lost_loop_count = 0

    previous_on_line = True
    last_debug_report_ms = 0

    try:
        lap_timer = create_lap_timer()

        line, motors, button = setup_paicar(
            lap_timer
        )

        if DEBUG_MODE:
            telemetry = PAIUdpTelemetry(
                lap_timer
            )

            # 여기에서 Wi-Fi 연결
            telemetry.begin()

            # Wi-Fi 연결 후 디버그 UDP 소켓 생성
            debug_socket = create_debug_socket()

            send_debug_message(
                debug_socket,
                (
                    "type=boot,"
                    "mode=DEBUG,"
                    "control_ms={},"
                    "telemetry_port={},"
                    "debug_port={}"
                ).format(
                    CONTROL_PERIOD_MS,
                    PC_PORT,
                    DEBUG_PC_PORT,
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

        drive_state = STATE_TRACKING

        steering = 0
        p_term = 0.0
        d_term = 0.0

        error = 0
        position = 0

        target_speed = SHARP_CURVE_SPEED
        speed_state = SPEED_STATE_SHARP_CURVE

        curve_score = 0.0

        left_cmd = 0
        right_cmd = 0

        previous_left_cmd = 0
        previous_right_cmd = 0

        saturation_scale = 1.0

        previous_filtered_error = 0.0
        previous_filtered_derivative = 0.0

        features = create_empty_features()

        previous_on_line = True
        last_debug_report_ms = ticks_ms()

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

            features = extract_features(
                error,
                on_line,
                norm,
                previous_filtered_error,
                previous_filtered_derivative,
            )

            previous_filtered_error = (
                features["filtered_error"]
            )

            previous_filtered_derivative = (
                features["filtered_error_rate"]
            )

            (
                curve_score,
                curve_error_component,
                curve_rate_component,
                curve_accel_component,
                curve_confidence_component,
            ) = calculate_curve_score(
                features
            )

            (
                target_speed,
                speed_state,
            ) = calculate_target_speed(
                curve_score,
                features,
                on_line,
            )

            if not on_line:
                line_lost_loop_count += 1

                if previous_on_line:
                    line_lost_count += 1

            previous_on_line = on_line

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

                if telemetry is not None:
                    telemetry.send_now(
                        target_speed,
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
                        "target={},"
                        "error={},"
                        "curve={:.3f}"
                    ).format(
                        loop_count,
                        target_speed,
                        error,
                        curve_score,
                    ),
                )

                break

            if on_line:
                drive_state = STATE_TRACKING

                (
                    steering,
                    p_term,
                    d_term,
                ) = calculate_pd_control(
                    features
                )

                (
                    left_cmd,
                    right_cmd,
                    saturation_scale,
                ) = mix_motor_commands(
                    target_speed,
                    steering,
                    previous_left_cmd,
                    previous_right_cmd,
                    emergency=False,
                )

                motors.drive(
                    left_cmd,
                    right_cmd,
                )

                previous_left_cmd = left_cmd
                previous_right_cmd = right_cmd

            else:
                drive_state = STATE_LINE_LOST

                motors.stop()

                target_speed = LINE_LOST_SPEED
                speed_state = SPEED_STATE_LINE_LOST

                steering = 0
                p_term = 0.0
                d_term = 0.0

                left_cmd = 0
                right_cmd = 0

                previous_left_cmd = 0
                previous_right_cmd = 0

                saturation_scale = 1.0

            if telemetry is not None:
                telemetry.send_if_due(
                    target_speed,
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
                        line_lost_count,
                        line_lost_loop_count,
                        drive_state,
                        features,
                        target_speed,
                        speed_state,
                        curve_score,
                        steering,
                        p_term,
                        d_term,
                        saturation_scale,
                        left_cmd,
                        right_cmd,
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

        # 소켓을 닫기 전에 최종 통계를 무선 전송
        send_final_report(
            debug_socket,
            finished,
            loop_count,
            overrun_count,
            max_loop_ms,
            total_compute_ms,
            line_lost_count,
            line_lost_loop_count,
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
# 17. Program entry point
# ============================================================

run()
# main.py
# PAI-Car high-speed simplified PD controller
# - aggressive speed mode
# - single KP / KD tuning
# - error-based speed control
# - straight boost up to 1000
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

# 공격형 시작값.
# 완주율보다 기록 단축 우선.
BASE_SPEED = 850
MAX_SPEED = 1000
MIN_SPEED = 420

# 오차 기반 감속.
SLOW_ERROR = 850
HARD_ERROR = 1800

SLOW_SPEED = 680
HARD_SPEED = 470

# 직선 안정 시 1000 boost.
BOOST_ERROR = 220
BOOST_D_ERROR = 70

# 속도 변화량.
# 기존 5보다 훨씬 빠르게 가속.
SPEED_RISE = 25
SPEED_FALL = 120


# ------------------------------------------------------------
# Steering
# ------------------------------------------------------------

KP = 0.24
KD = 0.42

STEERING_LIMIT = 620
ERROR_DEADBAND = 60

# 필터.
ERROR_ALPHA = 0.55
D_ALPHA = 0.18

# 직선 안정화.
# 코너 탈출 후 잔류 조향이 직선에서 흔들림으로 번지는 것을 줄임.
STRAIGHT_DAMP_ERROR_1 = 300
STRAIGHT_DAMP_D_1 = 90
STRAIGHT_DAMP_GAIN_1 = 0.55

STRAIGHT_DAMP_ERROR_2 = 500
STRAIGHT_DAMP_D_2 = 140
STRAIGHT_DAMP_GAIN_2 = 0.75


# ------------------------------------------------------------
# Motor
# ------------------------------------------------------------

MOTOR_MAX_CMD = 1000

LEFT_GAIN = 1.00
RIGHT_GAIN = 1.00

ALLOW_REVERSE = False

# 0이면 극공격형 회전.
# 120은 속도형 안정 절충.
INNER_FLOOR = 120


# ------------------------------------------------------------
# Line loss recovery
# ------------------------------------------------------------

LOST_FORWARD = 300
LOST_TURN = 650
LOST_PIVOT_AFTER_MS = 180


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


def move_speed(
    current,
    target,
):
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

    # 직선 안정 구간 boost.
    # 직선이 긴 코스에서 기록 단축 핵심.
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

    # 중심 근처이고 변화율도 작으면 직선으로 보고 잔류 조향을 빠르게 죽인다.
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
        -STEERING_LIMIT,
        STEERING_LIMIT,
    )

    steering = damp_steering_on_straight(
        steering,
        filtered_error,
        filtered_d,
    )

    return int(
        steering
    )


# ============================================================
# Motor / recovery
# ============================================================

def mix_motor(
    speed,
    steering,
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

    # 안쪽 바퀴가 완전히 죽으면 코너 탈출 흔들림이 커질 수 있음.
    if speed > 0:
        if left <= 0:
            left = INNER_FLOOR
        elif left < INNER_FLOOR:
            left = INNER_FLOOR

        if right <= 0:
            right = INNER_FLOOR
        elif right < INNER_FLOOR:
            right = INNER_FLOOR

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
        "on_line={}"
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
        "mode=HIGH_SPEED,"
        "control_ms={},"
        "loops={},"
        "average_compute_ms={:.3f},"
        "max_compute_ms={},"
        "overrun_count={},"
        "overrun_rate={:.2f},"
        "line_lost_entry={},"
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
            "HIGH SPEED",
            "BASE {}".format(BASE_SPEED),
            "MAX {}".format(MAX_SPEED),
            "KP {:.2f} KD {:.2f}".format(
                KP,
                KD,
            ),
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
                    "mode=HIGH_SPEED,"
                    "control_ms={},"
                    "base_speed={},"
                    "max_speed={},"
                    "kp={:.3f},"
                    "kd={:.3f}"
                ).format(
                    telemetry_ok,
                    CONTROL_MS,
                    BASE_SPEED,
                    MAX_SPEED,
                    KP,
                    KD,
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

            elapsed_run_ms = ticks_diff(
                ticks_ms(),
                run_start_ms,
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
                        "filtered_error={:.1f}"
                    ).format(
                        loop_count,
                        current_speed,
                        error,
                        filtered_error,
                    ),
                )

                if cost_ms > MAX_NETWORK_SEND_COST_MS:
                    network_slow_count += 1

                break

            if on_line:
                last_valid_error = filtered_error
                lost_start_ms = None

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
                )

                (
                    left_cmd,
                    right_cmd,
                ) = mix_motor(
                    current_speed,
                    steering,
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
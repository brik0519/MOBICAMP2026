# main.py
# PAI-Car v1.0 PD 제어 라인트레이싱 + 주행 시간 측정 + UDP 데이터 전송
#
# 무엔코더 안정 버전 1단계:
#   - 엔코더 없는 모터 기준
#   - encoder speed feedback 제거
#   - distance_ticks 기반 slow zone 제거
#   - error / d_error 기반 속도 제어 유지
#   - 라인 미검출 시 마지막 error 방향 저속 탐색 추가
#   - UDP 확장 포맷은 유지하되 encoder 관련 값은 0으로 전송
#
# 현재 목표:
#   1. 48.1초대 완주 안정성 유지
#   2. 40~42초 부근 급커브 라인 미검출 복구
#   3. 이후 시간 기반 slow zone을 제한적으로 적용

from time import ticks_ms, ticks_diff

from modules.pai_car_run_support import (
    CONTROL_MS,
    create_lap_timer,
    setup_paicar,
    self_calibrate_or_stop,
    wait_button_start,
    wait_control_period,
)

from modules.pai_udp_telemetry import (
    PAIUdpTelemetry,
    read_line_detail,
)


# ------------------------------------------------------------
# PD-control settings
# ------------------------------------------------------------

BASE_SPEED = 1000

KP = 0.52
KD = 0.21

MAX_CORRECTION = 1100


# ------------------------------------------------------------
# Speed settings
# ------------------------------------------------------------

STRAIGHT_SPEED = 1000

# 기존 encoder boost +30이 사실상 고정으로 들어가던 상태를 반영한다.
# 기존: CURVE 800 + boost 30 = 830
# 기존: SHARP 650 + boost 30 = 680
CURVE_SPEED = 830
SHARP_CURVE_SPEED = 680

MIN_RUN_SPEED = 500


# ------------------------------------------------------------
# Optional time-based slow zones
# ------------------------------------------------------------
#
# 엔코더가 없으므로 distance 기반 slow zone은 사용할 수 없다.
# 대신 elapsed_ms 기준 slow zone을 제한적으로 사용할 수 있다.
#
# 주의:
#   시간 기반 slow zone은 랩타임이 바뀌면 위치가 밀린다.
#   따라서 처음에는 비활성으로 둔다.
#
# 40~42초 부근에서 계속 라인을 잃으면 아래 예시를 활성화한다.
#
# TIME_SLOW_ZONES = [
#     (40000, 42000, 650),
# ]

TIME_SLOW_ZONES = []


# ------------------------------------------------------------
# Line-loss recovery settings
# ------------------------------------------------------------

SEARCH_PWM = 280
LINE_LOSS_MAX_MS = 250


# ------------------------------------------------------------
# Safe start / finish marker settings
# ------------------------------------------------------------

T_MARKER_TH = 700
T_MARKER_MIN_COUNT = 6

MARKER_CONFIRM_COUNT = 3
MARKER_RELEASE_COUNT = 5

MIN_FINISH_MS = 3000

DEBUG_MARKER = False


# ------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------

def clamp_value(value, low, high):
    if value < low:
        return low

    if value > high:
        return high

    return value


def clamp_correction(value):
    if value > MAX_CORRECTION:
        return MAX_CORRECTION

    if value < -MAX_CORRECTION:
        return -MAX_CORRECTION

    return int(value)


def limit_drive_cmd(value, error):
    """
    정상 라인트레이싱 중 모터 출력 제한.

    error가 작을 때는 역회전 금지.
    error가 클 때만 제한적 역회전 허용.
    """

    ae = abs(error)

    if ae >= 2200:
        min_cmd = -450
    elif ae >= 1500:
        min_cmd = -250
    else:
        min_cmd = 0

    if value > 1000:
        return 1000

    if value < min_cmd:
        return min_cmd

    return int(value)


def count_black_sensors(norm):
    count = 0

    for v in norm:
        if v >= T_MARKER_TH:
            count += 1

    return count


def marker_detected_now(norm, on_line):
    black_count = count_black_sensors(norm)

    return on_line and (black_count >= T_MARKER_MIN_COUNT)


def speed_from_time(elapsed_ms):
    speed = STRAIGHT_SPEED

    for start_ms, end_ms, zone_speed in TIME_SLOW_ZONES:
        if start_ms <= elapsed_ms <= end_ms:
            if zone_speed < speed:
                speed = zone_speed

    return speed


def speed_from_error(base_speed, error, d_error):
    ae = abs(error)
    ad = abs(d_error)

    speed = base_speed

    if ae > 2000 or ad > 1400:
        speed = min(speed, SHARP_CURVE_SPEED)

    elif ae > 1200 or ad > 800:
        speed = min(speed, CURVE_SPEED)

    if speed < MIN_RUN_SPEED:
        speed = MIN_RUN_SPEED

    return speed


def marker_event_from_norm(norm, on_line):
    global marker_active
    global marker_detect_count
    global marker_release_count

    detected = marker_detected_now(norm, on_line)

    if detected:
        marker_release_count = 0

        if marker_detect_count < MARKER_CONFIRM_COUNT:
            marker_detect_count += 1

        if marker_detect_count >= MARKER_CONFIRM_COUNT:
            if not marker_active:
                marker_active = True
                return True

    else:
        marker_detect_count = 0

        if marker_active:
            marker_release_count += 1

            if marker_release_count >= MARKER_RELEASE_COUNT:
                marker_active = False
                marker_release_count = 0

    return False


# ------------------------------------------------------------
# Setup
# ------------------------------------------------------------

lap_timer = create_lap_timer()

line, motors, button = setup_paicar(lap_timer)

telemetry = PAIUdpTelemetry(lap_timer)
telemetry.begin()


# ------------------------------------------------------------
# Self calibration
# ------------------------------------------------------------

self_calibrate_or_stop(line, motors, lap_timer)


# ------------------------------------------------------------
# Button start
# ------------------------------------------------------------

wait_button_start(button, lap_timer)

lap_timer.start()
telemetry.reset_timer()

run_start_ms = ticks_ms()


# ------------------------------------------------------------
# PD-control line tracing
# ------------------------------------------------------------

finished = False

last_error = 0
left_cmd = 0
right_cmd = 0
d_error = 0

was_on_line = False

target_speed = BASE_SPEED

line_lost_start_ms = None

last_udp_loop_ms = 0
last_udp_send_cost_ms = 0
last_udp_overrun = 0

marker_count = 0
marker_active = False
marker_detect_count = 0
marker_release_count = 0

try:
    while True:
        loop_start = ticks_ms()
        now_ms = ticks_ms()

        elapsed_ms = ticks_diff(now_ms, run_start_ms)

        # --------------------------------------------------------
        # 1. 라인센서 읽기
        # --------------------------------------------------------

        error, position, norm, on_line = read_line_detail(line)

        is_marker = marker_detected_now(norm, on_line)

        # --------------------------------------------------------
        # 2. Start / Finish marker 확인
        # --------------------------------------------------------

        marker_event = marker_event_from_norm(norm, on_line)

        if marker_event:
            marker_count += 1

            black_count = count_black_sensors(norm)

            if DEBUG_MARKER:
                print(
                    "MARKER",
                    marker_count,
                    "t=",
                    elapsed_ms,
                    "black=",
                    black_count
                )

            if marker_count == 1:
                pass

            else:
                finish_allowed = True

                if elapsed_ms < MIN_FINISH_MS:
                    finish_allowed = False

                if finish_allowed:
                    d_error = 0
                    left_cmd = 0
                    right_cmd = 0
                    target_speed = 0

                    motors.stop()
                    finished = True

                    telemetry.send_now(
                        target_speed,
                        norm,
                        position,
                        error,
                        d_error,
                        left_cmd,
                        right_cmd,
                        on_line,
                        is_marker,

                        0,  # distance_ticks
                        0,  # left_speed
                        0,  # right_speed
                        0,  # dl
                        0,  # dr
                        0,  # heading_ticks

                        last_udp_loop_ms,
                        last_udp_send_cost_ms,
                        last_udp_overrun
                    )

                    break

                else:
                    marker_count = 1

                    if DEBUG_MARKER:
                        print(
                            "EARLY_MARKER_IGNORED",
                            "t=",
                            elapsed_ms
                        )

        # --------------------------------------------------------
        # 3. PD 제어 + 라인 미검출 복구
        # --------------------------------------------------------

        if on_line:
            line_lost_start_ms = None

            if was_on_line:
                d_error = error - last_error
            else:
                d_error = 0

            last_error = error
            was_on_line = True

            time_speed = speed_from_time(elapsed_ms)

            target_speed = speed_from_error(
                time_speed,
                error,
                d_error
            )

            correction = int(KP * error + KD * d_error)
            correction = clamp_correction(correction)

            left_cmd = limit_drive_cmd(
                target_speed + correction,
                error
            )

            right_cmd = limit_drive_cmd(
                target_speed - correction,
                error
            )

            motors.drive(left_cmd, right_cmd)

        else:
            now = ticks_ms()

            if line_lost_start_ms is None:
                line_lost_start_ms = now

            loss_ms = ticks_diff(now, line_lost_start_ms)

            d_error = 0
            target_speed = 0
            was_on_line = False

            # 마지막으로 보았던 error 방향으로 저속 제자리 탐색한다.
            # error < 0 상태에서 라인을 잃었다면 왼쪽으로 치우친 것으로 보고
            # 왼쪽 바퀴 후진, 오른쪽 바퀴 전진으로 회전한다.
            if loss_ms <= LINE_LOSS_MAX_MS:
                if last_error < 0:
                    left_cmd = -SEARCH_PWM
                    right_cmd = SEARCH_PWM
                else:
                    left_cmd = SEARCH_PWM
                    right_cmd = -SEARCH_PWM

                motors.drive(left_cmd, right_cmd)

            else:
                left_cmd = 0
                right_cmd = 0
                motors.stop()

        # --------------------------------------------------------
        # 4. 주행 데이터 전송
        # --------------------------------------------------------

        sent = telemetry.send_if_due(
            target_speed,
            norm,
            position,
            error,
            d_error,
            left_cmd,
            right_cmd,
            on_line,
            is_marker,

            0,  # distance_ticks
            0,  # left_speed
            0,  # right_speed
            0,  # dl
            0,  # dr
            0,  # heading_ticks

            last_udp_loop_ms,
            last_udp_send_cost_ms,
            last_udp_overrun
        )

        loop_after_udp_ms = ticks_diff(ticks_ms(), loop_start)

        if sent:
            last_udp_loop_ms = loop_after_udp_ms
            last_udp_send_cost_ms = telemetry.last_send_cost_ms

            if loop_after_udp_ms > CONTROL_MS:
                last_udp_overrun = 1
            else:
                last_udp_overrun = 0

        # --------------------------------------------------------
        # 5. OLED 갱신
        # --------------------------------------------------------

        lap_timer.update()

        # --------------------------------------------------------
        # 6. 제어 주기 맞추기
        # --------------------------------------------------------

        wait_control_period(loop_start)


finally:
    motors.stop()
    telemetry.close()

    if not finished:
        lap_timer.show_stopped()
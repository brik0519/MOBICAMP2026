# main.py
# PAI-Car v1.0 PD 제어 라인트레이싱 + 주행 시간 측정 + UDP 데이터 전송
#
# 무엔코더 안정 버전 + 기본 finish 판별 롤백
#
# 유지:
#   - 엔코더 없는 모터 기준
#   - error / d_error 기반 속도 제어
#   - 라인 미검출 시 마지막 error 방향 저속 탐색
#   - UDP 확장 포맷 유지, encoder 관련 값은 0으로 전송
#
# 롤백:
#   - finish 판별은 기본 lap_timer.check_finish(norm, on_line) 사용
#   - 공격적인 black_count 기반 즉시 종료 판정 제거
#
# 현재 목표:
#   1. 코스 중간 오정지 방지
#   2. 48~49초대 완주 안정성 유지
#   3. 라인 미검출 시 복구 시도
#   4. UDP 로그 분석 유지

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
CURVE_SPEED = 830
SHARP_CURVE_SPEED = 680

MIN_RUN_SPEED = 500


# ------------------------------------------------------------
# Optional time-based slow zones
# ------------------------------------------------------------
#
# 엔코더가 없으므로 distance 기반 slow zone은 사용할 수 없다.
# elapsed_ms 기반 slow zone은 위치가 밀릴 수 있으므로 기본 비활성.

TIME_SLOW_ZONES = []


# ------------------------------------------------------------
# Line-loss recovery settings
# ------------------------------------------------------------

SEARCH_PWM = 280
LINE_LOSS_MAX_MS = 250


# ------------------------------------------------------------
# Marker display / logging settings
# ------------------------------------------------------------
#
# finish 판별 자체는 lap_timer.check_finish()에 맡긴다.
# 아래 값은 CSV의 is_marker 표시용으로만 사용한다.

T_MARKER_TH = 700
T_MARKER_MIN_COUNT = 6


# ------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------

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

    # 급커브 판정을 늦춘 speed_from_error()와 맞춰 역회전 허용도 늦춘다.
    if ae >= 2400:
        min_cmd = -420
    elif ae >= 1650:
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
    """
    CSV 표시용 marker 판정.
    정지 판정에는 사용하지 않는다.
    """

    if not on_line:
        return False

    black_count = count_black_sensors(norm)

    return black_count >= T_MARKER_MIN_COUNT


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

    # 최신 의도 기준:
    #   - base_speed=680 구간을 줄인다.
    #   - 불필요한 역회전을 줄인다.
    #   - 단, speed 값 자체는 아직 올리지 않는다.
    if ae > 2400 or ad > 1700:
        speed = min(speed, SHARP_CURVE_SPEED)

    elif ae > 1350 or ad > 950:
        speed = min(speed, CURVE_SPEED)

    if speed < MIN_RUN_SPEED:
        speed = MIN_RUN_SPEED

    return speed


def send_stop_packet(
    telemetry,
    target_speed,
    norm,
    position,
    error,
    on_line,
    is_marker,
    last_udp_loop_ms,
    last_udp_send_cost_ms,
    last_udp_overrun
):
    telemetry.send_now(
        target_speed,
        norm,
        position,
        error,
        0,  # d_error
        0,  # left_cmd
        0,  # right_cmd
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

try:
    while True:
        loop_start = ticks_ms()
        now_ms = ticks_ms()

        elapsed_ms = ticks_diff(now_ms, run_start_ms)

        # --------------------------------------------------------
        # 1. 라인센서 읽기
        # --------------------------------------------------------

        error, position, norm, on_line = read_line_detail(line)

        # CSV 표시용 marker.
        # 실제 finish 정지는 lap_timer.check_finish()만 사용한다.
        is_marker = marker_detected_now(norm, on_line)

        # --------------------------------------------------------
        # 2. Finish 확인: 기본 방식으로 롤백
        # --------------------------------------------------------
        #
        # 중요:
        #   black_count 기반 즉시 종료 판정은 사용하지 않는다.
        #   코스 중간 복수 센서 감지로 오정지한 문제가 있었으므로
        #   기본 lap_timer.check_finish()에 맡긴다.

        if lap_timer.check_finish(norm, on_line):
            d_error = 0
            left_cmd = 0
            right_cmd = 0
            target_speed = 0

            motors.stop()
            finished = True

            send_stop_packet(
                telemetry,
                target_speed,
                norm,
                position,
                error,
                on_line,
                is_marker,
                last_udp_loop_ms,
                last_udp_send_cost_ms,
                last_udp_overrun
            )

            break

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
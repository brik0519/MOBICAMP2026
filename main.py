# main.py
# PAI-Car v1.0 PD 제어 라인트레이싱 + 주행 시간 측정 + UDP 데이터 전송
#
# 무엔코더 안정 버전 + 기본 finish 판별 롤백
#
# 유지:
#   - 엔코더 없는 모터 기준
#   - error / d_error 기반 속도 제어
#   - 라인 미검출 시 마지막 error 방향 저속 탐색
#   - UDP 기본 V1 telemetry 포맷 유지
#
# 추가:
#   - PC -> Pico UDP command 수신
#   - PING / STOP / SAFE_MODE / RUN 지원
#   - STOP / SAFE_MODE 상태에서는 최종 모터 출력만 0으로 덮어씀
#   - 버튼 직후 시작선 중복 finish 오검출 방지

from time import ticks_ms, ticks_diff

from modules.pai_car_run_support import (
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

from modules.pai_udp_command import PAIUdpCommand


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

TIME_SLOW_ZONES = []


# ------------------------------------------------------------
# Line-loss recovery settings
# ------------------------------------------------------------

SEARCH_PWM = 280
LINE_LOSS_MAX_MS = 250


# ------------------------------------------------------------
# Marker display / logging settings
# ------------------------------------------------------------

T_MARKER_TH = 700
T_MARKER_MIN_COUNT = 6

# 시작선 위에서 버튼을 누른 직후,
# 같은 시작선을 finish로 중복 인식하지 않도록
# marker가 사라진 상태를 연속 몇 회 확인할지 결정한다.
START_MARKER_RELEASE_COUNT = 5


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
    정지 판정에는 직접 사용하지 않는다.
    """

    if not on_line:
        return False

    black_count = count_black_sensors(norm)

    return black_count >= T_MARKER_MIN_COUNT


def arm_start_marker_if_needed(lap_timer, norm, on_line):
    """
    버튼을 누른 직후 차량이 시작선 위에 있으면,
    시작 marker를 이미 1회 본 상태로 둔다.

    이렇게 해야 finish 지점에서 다음 marker가 2번째 marker로 처리된다.
    """

    if marker_detected_now(norm, on_line):
        lap_timer.t_marker_count = 1
        lap_timer.t_marker_active = True
        lap_timer.t_marker_release_count = 0
        return True

    return False


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
    is_marker
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
        is_marker
    )


# ------------------------------------------------------------
# Setup
# ------------------------------------------------------------

lap_timer = create_lap_timer()

line, motors, button = setup_paicar(lap_timer)

telemetry = PAIUdpTelemetry(lap_timer)
telemetry.begin()

cmd = PAIUdpCommand(require_heartbeat=False)
cmd.begin()


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
# Start marker guard
# ------------------------------------------------------------

_start_error, _start_position, _start_norm, _start_on_line = read_line_detail(line)

start_marker_armed = arm_start_marker_if_needed(
    lap_timer,
    _start_norm,
    _start_on_line
)

start_marker_released = not start_marker_armed
start_marker_release_count = 0


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

try:
    while True:
        loop_start = ticks_ms()
        now_ms = ticks_ms()

        elapsed_ms = ticks_diff(now_ms, run_start_ms)

        # --------------------------------------------------------
        # 0. PC command 수신
        # --------------------------------------------------------

        cmd.poll()
        force_stop = cmd.should_force_stop()

        # --------------------------------------------------------
        # 1. 라인센서 읽기
        # --------------------------------------------------------

        error, position, norm, on_line = read_line_detail(line)

        is_marker = marker_detected_now(norm, on_line)

        # --------------------------------------------------------
        # 2. Finish 확인
        # --------------------------------------------------------
        #
        # 시작선 위에서 버튼을 누른 경우,
        # 시작선을 완전히 벗어나기 전까지 finish 판별을 막는다.
        #
        # 단, 시작선을 그냥 무시하는 것이 아니라
        # start marker를 이미 1회 본 상태로 둔다.
        # 그래야 실제 finish 선에서 정지할 수 있다.

        if not start_marker_released:
            if marker_detected_now(norm, on_line):
                start_marker_release_count = 0
            else:
                start_marker_release_count += 1

                if start_marker_release_count >= START_MARKER_RELEASE_COUNT:
                    start_marker_released = True
                    lap_timer.t_marker_active = False
                    lap_timer.t_marker_release_count = 0

        else:
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
                    is_marker
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
        # 3-1. PC STOP / SAFE_MODE 강제 정지
        # --------------------------------------------------------
        #
        # 기존 라인트레이싱 계산과 복구 로직은 그대로 수행한 뒤,
        # STOP 또는 SAFE_MODE 상태일 때만 최종 모터 출력을 0으로 덮어쓴다.

        if force_stop:
            target_speed = 0
            left_cmd = 0
            right_cmd = 0
            motors.stop()

        # --------------------------------------------------------
        # 4. 주행 데이터 전송
        # --------------------------------------------------------
        #
        # 현재 pai_udp_telemetry.py는 기본 V1 telemetry 인자만 받는다.
        # 따라서 distance_ticks, udp_loop_ms 등 확장 인자는 보내지 않는다.

        telemetry.send_if_due(
            target_speed,
            norm,
            position,
            error,
            d_error,
            left_cmd,
            right_cmd,
            on_line,
            is_marker
        )

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
    cmd.close()

    if not finished:
        lap_timer.show_stopped()
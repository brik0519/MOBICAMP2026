# pd_control_udp.py
# PAI-Car v1.0 PD 제어 라인트레이싱 + 주행 시간 측정 + UDP 데이터 전송
#
# 랩타임 단축 조합 1단계:
#   - 조건부 역회전 허용 유지
#   - KP = 0.52, KD = 0.21 유지
#   - MAX_CORRECTION = 1100 유지
#   - CURVE_SPEED = 800 유지
#   - SHARP_CURVE_SPEED = 650 유지
#   - 커브 구간에서도 encoder boost를 +30까지만 허용
#   - 시작선 / 종료선 오검출 방지 유지
#
# 아직 추가하지 않은 것:
#   - 라인 미검출 시 마지막 오차 방향 저속 탐색
#   - UDP packet에 encoder 값 추가

from time import ticks_ms, ticks_diff

from modules.pai_encoder import WheelEncoders

from modules.pai_car_run_support import (
    CONTROL_MS,
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
)


# ------------------------------------------------------------
# PD-control settings
# ------------------------------------------------------------

BASE_SPEED = 1000

KP = 0.52
KD = 0.21

MAX_CORRECTION = 1100


# ------------------------------------------------------------
# Encoder-based speed settings
# ------------------------------------------------------------

STRAIGHT_SPEED = 1000

CURVE_SPEED = 800
SHARP_CURVE_SPEED = 650

MIN_RUN_SPEED = 500


# ------------------------------------------------------------
# Encoder speed feedback settings
# ------------------------------------------------------------

REF_PWM = 900
REF_TICK_SPEED = 80

SPEED_KP = 1
MAX_SPEED_BOOST = 60

# 커브 구간에서 허용할 양수 speed boost 상한
# 0이면 안정 조합, 30이면 랩타임 단축 1단계
CURVE_POSITIVE_BOOST_LIMIT = 30


# ------------------------------------------------------------
# Distance-based slow zones
# ------------------------------------------------------------

SLOW_ZONES = [
    # (120, 180, 720),
    # (360, 430, 650),
    # (590, 660, 620),
]


# ------------------------------------------------------------
# Safe start / finish marker settings
# ------------------------------------------------------------

T_MARKER_TH = 700
T_MARKER_MIN_COUNT = 6

MARKER_CONFIRM_COUNT = 3
MARKER_RELEASE_COUNT = 5

MIN_FINISH_MS = 3000
MIN_FINISH_DISTANCE_TICKS = 0

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


def speed_from_distance(distance_ticks):
    speed = STRAIGHT_SPEED

    for start_tick, end_tick, zone_speed in SLOW_ZONES:
        if start_tick <= distance_ticks <= end_tick:
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


def target_tick_speed_from_pwm(target_pwm):
    if REF_PWM <= 0:
        return REF_TICK_SPEED

    return target_pwm * REF_TICK_SPEED // REF_PWM


def encoder_speed_boost(encoders, target_pwm):
    actual_tick_speed = (
        encoders.left_speed + encoders.right_speed
    ) // 2

    target_tick_speed = target_tick_speed_from_pwm(target_pwm)

    speed_error = target_tick_speed - actual_tick_speed
    boost = SPEED_KP * speed_error

    boost = clamp_value(
        boost,
        -MAX_SPEED_BOOST,
        MAX_SPEED_BOOST
    )

    return boost


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

encoders = WheelEncoders(left_pin=16, right_pin=17)

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
encoders.reset()

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

        # --------------------------------------------------------
        # 0. 엔코더 상태 업데이트
        # --------------------------------------------------------

        encoders.update()

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

            elapsed_ms = ticks_diff(ticks_ms(), run_start_ms)
            black_count = count_black_sensors(norm)

            if DEBUG_MARKER:
                print(
                    "MARKER",
                    marker_count,
                    "t=",
                    elapsed_ms,
                    "dist=",
                    encoders.distance_ticks,
                    "black=",
                    black_count
                )

            if marker_count == 1:
                pass

            else:
                finish_allowed = True

                if elapsed_ms < MIN_FINISH_MS:
                    finish_allowed = False

                if encoders.distance_ticks < MIN_FINISH_DISTANCE_TICKS:
                    finish_allowed = False

                if finish_allowed:
                    motors.stop()
                    finished = True

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

                        encoders.distance_ticks,
                        encoders.left_speed,
                        encoders.right_speed,
                        encoders.dl,
                        encoders.dr,
                        encoders.heading_ticks,

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
                            elapsed_ms,
                            "dist=",
                            encoders.distance_ticks
                        )

        # --------------------------------------------------------
        # 3. PD 제어 + 엔코더 기반 속도 보정
        # --------------------------------------------------------

        if on_line:
            if was_on_line:
                d_error = error - last_error
            else:
                d_error = 0

            last_error = error
            was_on_line = True

            distance_speed = speed_from_distance(
                encoders.distance_ticks
            )

            safe_speed = speed_from_error(
                distance_speed,
                error,
                d_error
            )

            speed_boost = encoder_speed_boost(
                encoders,
                safe_speed
            )

            # 랩타임 단축 1단계:
            # 커브 감속 상태에서도 양수 boost를 +30까지만 허용한다.
            if safe_speed < STRAIGHT_SPEED:
                if speed_boost > CURVE_POSITIVE_BOOST_LIMIT:
                    speed_boost = CURVE_POSITIVE_BOOST_LIMIT

            target_speed = safe_speed + speed_boost

            if target_speed > STRAIGHT_SPEED:
                target_speed = STRAIGHT_SPEED

            if target_speed < MIN_RUN_SPEED:
                target_speed = MIN_RUN_SPEED

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

            encoders.set_direction_from_cmd(left_cmd, right_cmd)

        else:
            # 아직 마지막 오차 방향 저속 탐색은 넣지 않는다.
            motors.stop()
            encoders.set_direction_from_cmd(0, 0)

            d_error = 0
            left_cmd = 0
            right_cmd = 0
            was_on_line = False

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

            encoders.distance_ticks,
            encoders.left_speed,
            encoders.right_speed,
            encoders.dl,
            encoders.dr,
            encoders.heading_ticks,

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
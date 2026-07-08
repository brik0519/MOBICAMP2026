# pd_control_udp.py
# PAI-Car v1.0 PD 제어 라인트레이싱 + 주행 시간 측정 + UDP 데이터 전송

from time import ticks_ms, ticks_diff, sleep_ms

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


# ------------------------------------------------------------
# PD-control settings
# ------------------------------------------------------------
BASE_SPEED = 1000
KP = 0.55
KD = 0.22


# ------------------------------------------------------------
# Motor calibration settings
# ------------------------------------------------------------
MOTOR_CAL_SPEED = 1000
MOTOR_CAL_DURATION_MS = 2500
MOTOR_CAL_IGNORE_MS = 700
MOTOR_CAL_KP = 0.15

# 평균 오차 절댓값에 따른 강한 모터 감쇠 기준
MOTOR_CAL_ERROR_MILD = 100
MOTOR_CAL_ERROR_STRONG = 300

MOTOR_SCALE_MILD = 0.95
MOTOR_SCALE_STRONG = 0.90

# 유효 표본 수 기준
MOTOR_CAL_MIN_SAMPLES = 140


# ------------------------------------------------------------
# Motor Calibration Function
# ------------------------------------------------------------
def motor_calibrate(line, motors, button, lap_timer):
    """
    직선 구간을 풀스로틀로 주행하면서 평균 편향을 측정합니다.

    약한 모터는 100% 출력을 유지하고,
    강한 모터만 95% 또는 90%로 낮춥니다.

    반드시 self_calibrate_or_stop() 이후 실행해야 합니다.

    반환값:
        left_scale, right_scale
    """

    if lap_timer:
        lap_timer.show_hold(
            "MOTOR CAL",
            "Align Straight",
            "Press Button...",
            ""
        )

    # 차량을 긴 직선 구간 중앙에 정렬한 뒤 버튼 입력
    wait_button_start(button, lap_timer)

    if lap_timer:
        lap_timer.show_hold(
            "MOTOR CAL",
            "Running 2.5 sec",
            "Do not touch",
            ""
        )

    sleep_ms(300)

    start_time = ticks_ms()

    accumulated_error = 0
    accumulated_abs_error = 0
    sample_count = 0
    calibration_failed = False

    try:
        while ticks_diff(ticks_ms(), start_time) < MOTOR_CAL_DURATION_MS:
            loop_start = ticks_ms()

            error, position, norm, on_line = read_line_detail(line)

            # 라인 이탈 시 즉시 중단
            if not on_line:
                print("[!] Motor Cal: Line lost. Calibration disabled.")
                calibration_failed = True
                break

            # 풀스로틀을 유지하면서 최소한의 P 보정만 적용
            correction = int(MOTOR_CAL_KP * error)

            left_cmd = limit_cmd(MOTOR_CAL_SPEED + correction)
            right_cmd = limit_cmd(MOTOR_CAL_SPEED - correction)

            motors.drive(left_cmd, right_cmd)

            elapsed_ms = ticks_diff(ticks_ms(), start_time)

            # 출발 직후 기동 및 정렬 흔들림은 평균에서 제외
            if elapsed_ms >= MOTOR_CAL_IGNORE_MS:
                accumulated_error += error
                accumulated_abs_error += abs(error)
                sample_count += 1

            wait_control_period(loop_start)

    finally:
        motors.stop()

    # 실패 시 기본값
    left_scale = 1.00
    right_scale = 1.00
    avg_error = 0
    avg_abs_error = 0

    calibration_ok = (
        not calibration_failed
        and sample_count >= MOTOR_CAL_MIN_SAMPLES
    )

    if calibration_ok:
        avg_error = accumulated_error / sample_count
        avg_abs_error = accumulated_abs_error / sample_count

        abs_avg_error = abs(avg_error)

        # 편차 크기에 따라 강한 모터 감쇠 비율 결정
        if abs_avg_error >= MOTOR_CAL_ERROR_STRONG:
            strong_scale = MOTOR_SCALE_STRONG

        elif abs_avg_error >= MOTOR_CAL_ERROR_MILD:
            strong_scale = MOTOR_SCALE_MILD

        else:
            strong_scale = 1.00

        if strong_scale < 1.00:
            if avg_error > 0:
                # 양의 평균 오차:
                # 오른쪽 모터가 강하다고 가정
                right_scale = strong_scale

            elif avg_error < 0:
                # 음의 평균 오차:
                # 왼쪽 모터가 강하다고 가정
                left_scale = strong_scale

    if lap_timer:
        if calibration_ok:
            lap_timer.show_hold(
                "CAL DONE!",
                "Err: {}".format(int(avg_error)),
                "L:{:.2f} R:{:.2f}".format(
                    left_scale,
                    right_scale
                ),
                "N:{}".format(sample_count)
            )
        else:
            lap_timer.show_hold(
                "CAL FAILED",
                "No correction",
                "L:1.00 R:1.00",
                "N:{}".format(sample_count)
            )

        sleep_ms(3000)

    print(
        "[MOTOR CAL]",
        "ok=", calibration_ok,
        "avg_error=", avg_error,
        "avg_abs_error=", avg_abs_error,
        "samples=", sample_count,
        "left_scale=", left_scale,
        "right_scale=", right_scale
    )

    return left_scale, right_scale


# ------------------------------------------------------------
# Setup
# ------------------------------------------------------------
lap_timer = create_lap_timer()
line, motors, button = setup_paicar(lap_timer)

telemetry = PAIUdpTelemetry(lap_timer)
telemetry.begin()


# ------------------------------------------------------------
# 1. Self calibration
# ------------------------------------------------------------
# 차량이 제자리에서 좌우로 흔들리며 바닥 반사율을 측정합니다.
self_calibrate_or_stop(line, motors, lap_timer)


# ------------------------------------------------------------
# 2. Motor calibration
# ------------------------------------------------------------
# 화면에 "MOTOR CAL"이 뜨면 긴 직선 구간 시작점으로 옮긴 뒤
# 차량을 라인 중앙에 정렬하고 버튼을 누릅니다.
left_scale, right_scale = motor_calibrate(
    line,
    motors,
    button,
    lap_timer
)


# ------------------------------------------------------------
# 3. Button start
# ------------------------------------------------------------
# "RACE READY"가 뜨면 출발선에 놓고 시작 버튼을 누릅니다.
if lap_timer:
    lap_timer.show_hold(
        "RACE READY",
        "L:{:.2f} R:{:.2f}".format(
            left_scale,
            right_scale
        ),
        "Press Button",
        "to START!"
    )

wait_button_start(button, lap_timer)
wait_button_start(button, lap_timer)

lap_timer.start()
telemetry.reset_timer()


# ------------------------------------------------------------
# PD-control line tracing
# ------------------------------------------------------------
finished = False
last_error = 0
left_cmd = 0
right_cmd = 0
d_error = 0

try:
    while True:
        loop_start = ticks_ms()

        # --------------------------------------------------------
        # 1. 라인센서 읽기
        # --------------------------------------------------------
        error, position, norm, on_line = read_line_detail(line)
        is_marker = is_t_marker_area(norm, on_line)

        # --------------------------------------------------------
        # 2. Finish 확인
        # --------------------------------------------------------
        if lap_timer.check_finish(norm, on_line):
            motors.stop()
            finished = True

            telemetry.send_now(
                BASE_SPEED,
                norm,
                position,
                error,
                0,
                0,
                0,
                on_line,
                is_marker
            )
            break

        # --------------------------------------------------------
        # 3. PD 제어
        # --------------------------------------------------------
        if on_line:
            d_error = error - last_error
            correction = int(KP * error + KD * d_error)

            last_error = error

            raw_left_cmd = BASE_SPEED + correction
            raw_right_cmd = BASE_SPEED - correction

            # 약한 모터는 100% 유지하고 강한 모터만 감쇠
            left_cmd = limit_cmd(
                int(raw_left_cmd * left_scale)
            )
            right_cmd = limit_cmd(
                int(raw_right_cmd * right_scale)
            )

            motors.drive(left_cmd, right_cmd)

        else:
            motors.stop()

            d_error = 0
            left_cmd = 0
            right_cmd = 0
            last_error = 0

        # --------------------------------------------------------
        # 4. 주행 데이터 전송
        # --------------------------------------------------------
        telemetry.send_if_due(
            BASE_SPEED,
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

    if not finished:
        lap_timer.show_stopped()
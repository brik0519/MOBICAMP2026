# pd_control_udp.py
# PAI-Car v1.0 PD 제어 라인트레이싱 + 주행 시간 측정 + UDP 데이터 전송
#
# 이 파일은 실제 실행되는 main 파일이다.
#
# 학생들이 보는 핵심 흐름:
#   1. PAI-Car 준비
#   2. 라인센서 셀프 캘리브레이션
#   3. 버튼을 눌렀다가 떼면 출발
#   4. 라인센서 값을 읽는다.
#   5. PD 제어로 좌우 모터 속도를 계산한다.
#   6. 20ms마다 주행 데이터를 PC로 보낸다.
#   7. Finish T 마커를 만나면 정지한다.

from time import ticks_ms

from pai_car_run_support import (
    create_lap_timer,
    setup_paicar,
    self_calibrate_or_stop,
    wait_button_start,
    limit_cmd,
    wait_control_period,
)

from pai_udp_telemetry import (
    PAIUdpTelemetry,
    read_line_detail,
    is_t_marker_area,
)


# ------------------------------------------------------------
# PD-control settings
# ------------------------------------------------------------
# 학생들이 우선 조정해 볼 값은 아래 세 가지이다.

BASE_SPEED = 740


# KP:
#   현재 error에 대한 보정 강도이다.
#   값이 클수록 라인에서 벗어났을 때 더 강하게 방향을 수정한다.
KP = 0.22


# KD:
#   error 변화량에 대한 보정 강도이다.
#   값이 적절하면 좌우 흔들림을 줄이고 코너에서 움직임을 부드럽게 만든다.
KD = 0.17


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

            left_cmd = limit_cmd(BASE_SPEED + correction)
            right_cmd = limit_cmd(BASE_SPEED - correction)

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
        # 실제 전송 주기, packet format, Wi-Fi/UDP 처리는
        # pai_udp_telemetry.py 안에서 관리한다.

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
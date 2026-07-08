# ai_control_udp.py
# PAI-Car v1.0 AI 모델 기반 라인트레이싱 + 주행 시간 측정 + UDP 데이터 전송
#
# 이 파일은 Linear Regression 모델을 이용해 PAI-Car를 주행시키는 main 파일이다.
#
# 동작 흐름:
#   1. PAI-Car 하드웨어 준비
#   2. UDP telemetry 준비
#   3. 라인센서 셀프 캘리브레이션
#   4. 버튼을 눌렀다가 떼면 출발
#   5. 라인센서 값을 읽는다.
#   6. AI 모델에 입력 features를 넣는다.
#   7. AI 모델이 left_cmd, right_cmd를 예측한다.
#   8. 예측된 모터 명령으로 주행한다.
#   9. 20ms마다 주행 데이터를 PC로 보낸다.
#   10. Finish T 마커를 만나면 정지한다.
#
# 필요 파일:
#   - pai_car_lr_model.py
#   - pai_udp_telemetry.py
#   - pai_car_wifi_config.py
#   - pai_car_run_support.py
#   - pai_motor.py
#   - pai_line_sensor.py
#   - pai_self_calibration.py
#   - tla2528.py

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

from pai_car_lr_model import predict


# ------------------------------------------------------------
# AI-control settings
# ------------------------------------------------------------
# 학습 데이터 수집 때 사용한 BASE_SPEED와 같은 값으로 시작하는 것이 좋다.
# 나중에 여러 base_speed를 섞어 학습했다면 이 값을 바꿔가며 실험할 수 있다.

BASE_SPEED = 740


# ------------------------------------------------------------
# Utility
# ------------------------------------------------------------

def make_features(norm, position, error, d_error, base_speed):
    """
    Linear Regression 모델에 넣을 입력 features를 만든다.

    주의:
        이 순서는 train_pai_car_lr.py에서 사용한 FEATURE_COLUMNS 순서와
        반드시 같아야 한다.

    FEATURE_COLUMNS:
        n0, n1, n2, n3, n4, n5, n6, n7,
        position,
        error,
        d_error,
        base_speed
    """

    return [
        norm[0],
        norm[1],
        norm[2],
        norm[3],
        norm[4],
        norm[5],
        norm[6],
        norm[7],
        position,
        error,
        d_error,
        base_speed,
    ]


def to_motor_cmd(value):
    """
    AI 모델이 예측한 값을 모터 명령 정수로 변환하고,
    허용 범위 -1000~1000으로 제한한다.
    """

    return limit_cmd(int(value))


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
# AI-control line tracing
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
        # 3. AI 모델 기반 제어
        # --------------------------------------------------------

        if on_line:
            d_error = error - last_error
            last_error = error

            features = make_features(norm,position,error,d_error,BASE_SPEED)

            left_pred, right_pred = predict(features)

            left_cmd = to_motor_cmd(left_pred)
            right_cmd = to_motor_cmd(right_pred)

            motors.drive(left_cmd, right_cmd)

        else:
            # 라인을 잃으면 안전을 위해 정지한다.
            motors.stop()

            d_error = 0
            left_cmd = 0
            right_cmd = 0
            last_error = 0

        # --------------------------------------------------------
        # 4. 주행 데이터 전송
        # --------------------------------------------------------
        # AI 주행 중에도 PD 데이터 수집 때와 같은 형식으로 전송한다.
        # PC에서는 같은 pc_udp_monitor.py로 수신/저장/그래프 표시가 가능하다.

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
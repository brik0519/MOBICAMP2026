# p_control.py
# PAI-Car v1.0 P 제어 라인트레이싱 + 주행 시간 측정
#
# 이 코드는 PAI-Car를 트랙 위에 올려놓고,
# 라인센서 값을 이용하여 검은색 라인을 따라가도록 주행시키는 코드이다.
#
# 추가 기능:
#   - 버튼을 눌렀다가 떼면 주행 시간을 측정하기 시작한다.
#   - 주행 중 OLED에 경과 시간을 표시한다.
#   - 첫 번째 T 마커는 Start 지점으로 보고 무시한다.
#   - 두 번째 T 마커는 Finish 지점으로 보고 정지한다.
#   - 정지 후 OLED에 최종 주행 시간을 표시한다.
#
# 참고:
#   자주 바꾸지 않는 하드웨어 설정, 센서 설정, 캘리브레이션 설정,
#   OLED 설정, T 마커 검출 설정 등은 pai_car_run_support.py 파일에
#   기본값으로 정리되어 있다.
#
#   이 파일에서는 학생들이 조정해 볼 만한 값과
#   P 제어의 핵심 흐름만 보이도록 구성하였다.

from time import ticks_ms

from pai_car_run_support import (
    create_lap_timer,
    setup_paicar,
    self_calibrate_or_stop,
    wait_button_start,
    read_line,
    limit_cmd,
    wait_control_period,
)


# ------------------------------------------------------------
# P-control settings
# ------------------------------------------------------------
# 학생들이 우선 조정해 볼 값은 아래 두 가지이다.

# BASE_SPEED:
#   PAI-Car가 기본적으로 앞으로 나아가려는 속도이다.
#   값이 커질수록 빠르게 주행하지만, 너무 크면 코너에서 라인을 벗어나기 쉽다.
BASE_SPEED = 300


# KP:
#   P 제어에서 error에 곱해지는 비례 제어 계수이다.
#
#   correction = KP × error
#
#   KP가 클수록 라인에서 벗어났을 때 더 강하게 방향을 수정한다.
#   너무 작으면 코너에서 밀려나고, 너무 크면 좌우로 흔들릴 수 있다.
KP = 0.11


# ------------------------------------------------------------
# Setup
# ------------------------------------------------------------
# OLED 주행 시간 표시 기능을 준비한다.
# OLED가 없거나 ssd1306.py 파일이 없어도 주행 자체는 가능하다.

lap_timer = create_lap_timer()


# PAI-Car 주행에 필요한 장치들을 준비한다.
#
# setup_paicar() 함수 안에서는 다음 작업이 수행된다.
#   - TLA2528 ADC 초기화
#   - 라인센서 객체 생성
#   - 모터 객체 생성
#   - 사용자 버튼 설정

line, motors, button = setup_paicar(lap_timer)


# ------------------------------------------------------------
# Self calibration
# ------------------------------------------------------------
# 라인센서 셀프 캘리브레이션을 수행한다.
#
# 캘리브레이션은 각 라인센서가 흰색 바닥과 검은색 라인을
# 어느 정도 값으로 읽는지 기준을 잡는 과정이다.
#
# 캘리브레이션이 실패하면 안전을 위해 주행하지 않고 정지 상태를 유지한다.

self_calibrate_or_stop(line, motors, lap_timer)


# ------------------------------------------------------------
# Button start
# ------------------------------------------------------------
# 셀프 캘리브레이션이 끝나면 바로 출발하지 않고,
# 사용자가 버튼을 눌렀다가 뗄 때까지 기다린다.

wait_button_start(button, lap_timer)


# 버튼을 눌렀다가 떼면 주행 시간 측정을 시작한다.
# 이 시점을 Start 시간으로 본다.

lap_timer.start()


# ------------------------------------------------------------
# P-control line tracing
# ------------------------------------------------------------
# 여기부터가 실제 라인트레이싱 주행 코드이다.
#
# 반복문 안의 핵심 흐름:
#   1. 라인센서로부터 error 값을 읽는다.
#   2. T 마커를 확인하여 Finish인지 판단한다.
#   3. error에 KP를 곱해 correction 값을 계산한다.
#   4. correction 값으로 왼쪽/오른쪽 모터 속도를 다르게 만든다.
#   5. 계산된 속도로 모터를 구동한다.
#   6. OLED에 경과 시간을 표시한다.
#   7. 일정한 제어 주기가 되도록 기다린다.

finished = False

try:
    while True:
        # 현재 반복이 시작된 시간을 저장한다.
        # 반복문 마지막에서 제어 주기를 일정하게 맞추는 데 사용된다.
        loop_start = ticks_ms()


        # 라인센서 값을 읽는다.
        #
        # error:
        #   라인의 중심에서 PAI-Car가 얼마나 벗어났는지를 나타내는 값이다.
        #
        # on_line:
        #   라인을 정상적으로 감지했는지 여부를 나타낸다.
        #
        # norm:
        #   8개 라인센서의 정규화된 값이다.
        #   T 마커 검출에 사용된다.
        error, on_line, norm = read_line(line)


        # T 마커를 확인한다.
        #
        # 첫 번째 T 마커는 Start 지점으로 보고 무시한다.
        # 두 번째 T 마커는 Finish 지점으로 보고 True를 반환한다.
        if lap_timer.check_finish(norm, on_line):
            motors.stop()
            finished = True
            break


        # 라인을 정상적으로 감지한 경우에만 P 제어를 수행한다.
        if on_line:

            # P 제어의 핵심 계산이다.
            #
            # correction = KP × error
            #
            # error가 크면 라인 중심에서 많이 벗어난 것이므로
            # correction 값도 커진다.
            correction = int(KP * error)


            # 기본 속도(BASE_SPEED)에 correction 값을 더하거나 빼서
            # 양쪽 바퀴의 속도 차이를 만든다.
            left_speed = limit_cmd(BASE_SPEED + correction)
            right_speed = limit_cmd(BASE_SPEED - correction)


            # 계산된 속도로 왼쪽/오른쪽 모터를 구동한다.
            motors.drive(left_speed, right_speed)


        # 라인을 정상적으로 감지하지 못한 경우
        else:
            # 첫 번째 P 제어 테스트에서는 라인을 잃으면 정지하도록 한다.
            motors.stop()


        # OLED에 경과 시간을 표시한다.
        #
        # OLED 출력은 I2C 통신 시간이 걸리므로 매 반복마다 갱신하지 않고,
        # pai_car_run_support.py에 정해진 시간 간격으로만 갱신한다.
        lap_timer.update()


        # 제어 주기를 일정하게 맞춘다.
        wait_control_period(loop_start)


finally:
    # 프로그램이 중간에 멈추거나 오류가 발생하더라도
    # 마지막에는 반드시 모터를 정지시킨다.
    motors.stop()

    # Finish로 정상 종료된 경우에는 OLED에 최종 시간이 이미 표시되어 있다.
    # 그 외의 경우에는 정지 상태를 OLED에 표시한다.
    if not finished:
        lap_timer.show_stopped()
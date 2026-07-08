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
# [신규 추가] Motor Calibration Function
# ------------------------------------------------------------
def motor_calibrate(line, motors, button, lap_timer):
    """
    직선 구간에서 모터 편차를 측정하고 좌/우 가중치를 반환합니다.
    (반드시 self_calibrate_or_stop 이후에 실행되어야 on_line 판정이 정상 동작합니다.)
    """
    if lap_timer:
        lap_timer.show_hold("MOTOR CAL", "Align Straight", "Press Button...", "")

    # 사용자가 차를 직선에 똑바로 정렬하고 버튼을 누를 때까지 대기
    wait_button_start(button, lap_timer)

    if lap_timer:
        lap_timer.show_hold("MOTOR CAL", "Running...", "Do not touch", "")
        
    sleep_ms(300)

    CAL_SPEED = 1000
    CAL_DURATION = 2000       # 주행 시간 2초 (트랙 길이에 맞춰 1500~2000 권장)
    CAL_KP = 0.15
    IGNORE_INITIAL_MS = 700   # 초기 0.7초 흔들림 무시

    start_time = ticks_ms()
    end_time = start_time + CAL_DURATION
    
    accumulated_error = 0
    sample_count = 0

    while ticks_ms() < end_time:
        error, position, norm, on_line = read_line_detail(line)
        
        # 라인 이탈 시 즉시 중단 (안전 장치)
        if not on_line:
            print("[!] Motor Cal: Line lost. Stopping early.")
            break
            
        # 출발 후 안정화 시간이 지난 후부터 데이터 누적
        if ticks_diff(ticks_ms(), start_time) > IGNORE_INITIAL_MS:
            accumulated_error += error
            sample_count += 1
            
        correction = int(CAL_KP * error)
        l_cmd = limit_cmd(CAL_SPEED + correction)
        r_cmd = limit_cmd(CAL_SPEED - correction)
        
        motors.drive(l_cmd, r_cmd)
        sleep_ms(10)

    motors.stop()

    # 분석 및 가중치 계산
    l_gain, r_gain = 1.0, 1.0
    avg_error = 0
    if sample_count > 0:
        avg_error = accumulated_error / sample_count
        if avg_error > 50:
            l_gain = 1.0 + (avg_error / 3500.0)
        elif avg_error < -50:
            r_gain = 1.0 + (abs(avg_error) / 3500.0)

    if lap_timer:
        # 결과를 OLED에 3초간 표시
        lap_timer.show_hold("CAL DONE!", f"Err: {int(avg_error)}", f"L:{l_gain:.2f} R:{r_gain:.2f}", "")
        sleep_ms(3000)

    return l_gain, r_gain


# ------------------------------------------------------------
# Setup
# ------------------------------------------------------------
lap_timer = create_lap_timer()
line, motors, button = setup_paicar(lap_timer)

telemetry = PAIUdpTelemetry(lap_timer)
telemetry.begin()

# ------------------------------------------------------------
# 1. Self calibration (IR 센서 영점 맞추기) - ★ 무조건 1순위
# ------------------------------------------------------------
# 차량이 제자리에서 좌우로 흔들리며 바닥 반사율을 측정합니다.
self_calibrate_or_stop(line, motors, lap_timer)


# ------------------------------------------------------------
# 2. Motor calibration (모터 편차 측정) - ★ 센서가 정상화된 후 2순위
# ------------------------------------------------------------
# 화면에 "MOTOR CAL"이 뜨면 차를 긴 직선 구간 시작점으로 옮기고 버튼을 누릅니다.
left_gain, right_gain = motor_calibrate(line, motors, button, lap_timer)


# ------------------------------------------------------------
# 3. Button start (실제 본선 레이스 시작)
# ------------------------------------------------------------
# "RACE READY"가 뜨면 다시 출발선에 놓고 시작 버튼을 누릅니다.
if lap_timer:
    lap_timer.show_hold("RACE READY", "Press Button", "to START!", "")

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
            telemetry.send_now(BASE_SPEED, norm, position, error, 0, 0, 0, on_line, is_marker)
            break

        # --------------------------------------------------------
        # 3. PD 제어
        # --------------------------------------------------------
        if on_line:
            d_error = error - last_error
            correction = int(KP * error + KD * d_error)

            last_error = error

            # 기본 제어값 계산
            raw_left_cmd = BASE_SPEED + correction
            raw_right_cmd = BASE_SPEED - correction

            # [핵심] 캘리브레이션 가중치 적용 후 모터 출력 범위 제한
            left_cmd = limit_cmd(int(raw_left_cmd * left_gain))
            right_cmd = limit_cmd(int(raw_right_cmd * right_gain))

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
        telemetry.send_if_due(BASE_SPEED, norm, position, error, d_error, left_cmd, right_cmd, on_line, is_marker)

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

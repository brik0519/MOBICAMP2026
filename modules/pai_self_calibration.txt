# pai_self_calibration.py
# PAI-Car v1.0 self-calibration helper
#
# 역할:
#   - 모터를 이용해 PAI-Car를 좌우로 회전시킨다.
#   - 회전하는 동안 라인센서 raw 값을 읽어 cal_min / cal_max를 갱신한다.
#   - 캘리브레이션이 성공하면 line_cal.json으로 저장한다.
#   - 함수 종료 시 반드시 모터를 정지한다.
#
# 주의:
#   - 이 파일은 버튼 입력을 처리하지 않는다.
#   - 이 파일은 PID 주행을 시작하지 않는다.
#   - 버튼 처리와 주행 시작은 main.py에서 담당한다.

from time import sleep_ms, ticks_ms, ticks_diff


# ------------------------------------------------------------
# 기본 설정값
# ------------------------------------------------------------

CAL_SPEED = 280
SEGMENT_MS = 500
SAMPLE_INTERVAL_MS = 5
PAUSE_MS = 100


def _collect_calibration(line_sensor, duration_ms, interval_ms):
    """
    지정한 시간 동안 센서 raw 값을 읽어 캘리브레이션 값을 갱신한다.
    모터는 이 함수 밖에서 이미 움직이고 있어야 한다.
    """

    start = ticks_ms()

    while ticks_diff(ticks_ms(), start) < duration_ms:
        raw = line_sensor.read_raw()
        line_sensor.update_calibration(raw)
        sleep_ms(interval_ms)


def _stop_and_pause(motors, pause_ms):
    """
    모터를 정지하고 잠깐 기다린다.
    방향 전환 시 차체 흔들림과 기계적 충격을 줄이기 위한 처리이다.
    """

    motors.stop()
    sleep_ms(pause_ms)


def self_calibrate(
    line_sensor,
    motors,
    cal_speed=CAL_SPEED,
    segment_ms=SEGMENT_MS,
    sample_interval_ms=SAMPLE_INTERVAL_MS,
    pause_ms=PAUSE_MS,
    save=True
):
    """
    PAI-Car 셀프 캘리브레이션 함수.

    동작 순서:
        1. 오른쪽 제자리 회전
        2. 왼쪽 제자리 회전
        3. 오른쪽 제자리 회전
        4. 캘리브레이션 성공 시 line_cal.json 저장
        5. 모터 정지

    반환값:
        True  : 캘리브레이션 성공
        False : 캘리브레이션 실패
    """

    print()
    print("Self calibration start")
    print("Place PAI-Car on the center of the black line.")
    print()

    line_sensor.reset_calibration()

    try:
        _stop_and_pause(motors, 300)

        print("Step 1: pivot right")
        motors.pivot_right(cal_speed)
        _collect_calibration(line_sensor, segment_ms, sample_interval_ms)
        _stop_and_pause(motors, pause_ms)

        print("Step 2: pivot left")
        motors.pivot_left(cal_speed)
        _collect_calibration(line_sensor, segment_ms, sample_interval_ms)
        _stop_and_pause(motors, pause_ms)

        print("Step 3: pivot left")
        motors.pivot_left(cal_speed)
        _collect_calibration(line_sensor, segment_ms, sample_interval_ms)
        _stop_and_pause(motors, pause_ms)

        print("Step 4: pivot right")
        motors.pivot_right(cal_speed)
        _collect_calibration(line_sensor, segment_ms, sample_interval_ms)
        _stop_and_pause(motors, pause_ms)

    finally:
        motors.stop()

    print()
    print("===== Self calibration result =====")
    print("cal_min:")
    print(line_sensor.cal_min)
    print("cal_max:")
    print(line_sensor.cal_max)

    ok = line_sensor.is_calibrated()
    print("calibration ok:", ok)

    if ok and save:
        line_sensor.save_calibration()
        print("line_cal.json saved.")

    if not ok:
        print("Self calibration failed.")
        print("Check whether all sensors passed over both white floor and black line.")

    motors.stop()
    return ok
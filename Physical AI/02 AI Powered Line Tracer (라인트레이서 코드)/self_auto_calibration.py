# test_self_auto_calibration_only.py
# PAI-Car v1.0 self auto calibration only test
#
# 동작:
#   1. 코드 실행
#   2. 0.5초 대기
#   3. 오른쪽 제자리 턴 0.5초
#   4. 왼쪽 제자리 턴 0.5초
#   5. 왼쪽 제자리 턴 0.5초
#   6. 오른쪽 제자리 턴 0.5초
#   7. 모터 정지
#   8. cal_min, cal_max, span 출력
#
# 주의:
#   - PID 주행 없음
#   - 버튼 처리 없음
#   - 동작 확인용이므로 기본값은 line_cal.json 저장 안 함
#
# 필요 파일:
#   - tla2528.py
#   - pai_line_sensor.py
#   - pai_motor.py
#   - pai_self_calibration.py

from time import sleep_ms

from tla2528 import TLA2528
from pai_line_sensor import PaiLineSensor
from pai_motor import PAICarMotors
from pai_self_calibration import self_calibrate


# ------------------------------------------------------------
# 테스트 설정값
# ------------------------------------------------------------

START_DELAY_MS = 500

# 셀프 캘리브레이션 동작 설정
CAL_SPEED = 280
CAL_SEGMENT_MS = 500
CAL_SAMPLE_INTERVAL_MS = 5
CAL_PAUSE_MS = 100

# 라인 검출 기준값
MIN_TOTAL = 500
NOISE_CUTOFF = 100

# 테스트 단계에서는 기존 line_cal.json을 덮어쓰지 않도록 False 권장
# 동작과 결과가 안정적이면 True로 변경
SAVE_CALIBRATION = False


# ------------------------------------------------------------
# ADC / 라인센서 초기화
# ------------------------------------------------------------

adc = TLA2528(
    i2c_id=0,
    sda_pin=4,
    scl_pin=5,
    freq=400_000,
    address=None,
    startup_delay_ms=300
)

adc.begin(verbose=True)

line = PaiLineSensor(
    adc,
    sensor_count=8,
    dark_is_low=True,
    cal_file="line_cal.json",
    min_range=30,
    min_total=MIN_TOTAL,
    noise_cutoff=NOISE_CUTOFF
)


# ------------------------------------------------------------
# 모터 초기화
# ------------------------------------------------------------

motors = PAICarMotors()
motors.stop()


# ------------------------------------------------------------
# 셀프 오토 캘리브레이션 동작 테스트
# ------------------------------------------------------------

try:
    print()
    print("====================================")
    print("PAI-Car self auto calibration test")
    print("====================================")
    print()
    print("Place PAI-Car on the center of the black line.")
    print("Self calibration will start after", START_DELAY_MS, "ms.")
    print()
    print("Pattern:")
    print("  1. pivot right 0.5 s")
    print("  2. pivot left  0.5 s")
    print("  3. pivot left  0.5 s")
    print("  4. pivot right 0.5 s")
    print()

    sleep_ms(START_DELAY_MS)

    ok = self_calibrate(
        line,
        motors,
        cal_speed=CAL_SPEED,
        segment_ms=CAL_SEGMENT_MS,
        sample_interval_ms=CAL_SAMPLE_INTERVAL_MS,
        pause_ms=CAL_PAUSE_MS,
        save=SAVE_CALIBRATION
    )

    motors.stop()

    print()
    print("====================================")
    print("Self calibration test finished")
    print("====================================")
    print()

    print("calibration ok:", ok)
    print()

    print("cal_min:")
    print(line.cal_min)
    print()

    print("cal_max:")
    print(line.cal_max)
    print()

    span = []
    for i in range(line.sensor_count):
        span.append(line.cal_max[i] - line.cal_min[i])

    print("span = cal_max - cal_min:")
    print(span)
    print()

    if SAVE_CALIBRATION:
        print("line_cal.json was saved.")
    else:
        print("line_cal.json was NOT saved.")
        print("If the movement and span values are good, set SAVE_CALIBRATION = True.")
    print()

    # 캘리브레이션 직후 현재 위치값도 몇 번 확인
    print("Line position check after self calibration")
    print("Press Ctrl+C to stop.")
    print()

    while True:
        position, norm, on_line = line.read_line(
            min_total=MIN_TOTAL,
            noise_cutoff=NOISE_CUTOFF
        )

        print(
            "pos:", position,
            "on:", on_line,
            "total:", line.last_total,
            "ftotal:", line.last_filtered_total,
            "peak:", line.last_peak,
            "ch:", line.last_peak_index,
            "norm:", norm
        )

        sleep_ms(300)

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    motors.stop()
    print("Motor stopped")
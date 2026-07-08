# benchmark_tla2528_read_all.py
#
# 목적:
#   TLA2528 모듈의 read_all_raw10() 호출 시간이 얼마나 걸리는지 측정한다.
#
# 측정 대상:
#   adc.read_all_raw10()
#
# 제외한 것:
#   - UDP 전송
#   - 문자열 생성
#   - 라인 위치 계산
#   - PD 제어 계산
#   - 모터 출력
#
# 실행 전 권장:
#   - Wi-Fi 사용 코드가 실행 중이었다면 Pico를 완전히 재부팅한 뒤 실행
#   - Thonny 콘솔 출력은 측정 루프 안에서 하지 않음

import time
import gc

try:
    import network
except ImportError:
    network = None

from tla2528 import TLA2528


# ==============================
# 사용자 설정
# ==============================

I2C_ID = 0
SDA_PIN = 4
SCL_PIN = 5
I2C_FREQ = 400_000

ADC_ADDRESS = None
STARTUP_DELAY_MS = 300

WARMUP_COUNT = 20
TEST_COUNT = 1000

# 이 값 이상이면 느린 읽기로 표시
SLOW_READ_US = 1000


# ==============================
# Wi-Fi 비활성화
# ==============================

def disable_wifi():
    if network is None:
        return

    try:
        wlan = network.WLAN(network.STA_IF)
        wlan.active(False)

        ap = network.WLAN(network.AP_IF)
        ap.active(False)

        print("Wi-Fi disabled")
    except Exception as e:
        print("Wi-Fi disable skipped:", e)


# ==============================
# 벤치마크
# ==============================

def benchmark_read_all(adc):
    print()
    print("Warm-up...")

    for _ in range(WARMUP_COUNT):
        adc.read_all_raw10()

    gc.collect()

    print("Benchmark start")
    print("TEST_COUNT =", TEST_COUNT)
    print("SLOW_READ_US =", SLOW_READ_US)
    print()

    min_us = 999999999
    max_us = 0
    sum_us = 0

    slow_count = 0

    bin_500 = 0
    bin_700 = 0
    bin_1000 = 0
    bin_1500 = 0
    bin_2000 = 0
    bin_over_2000 = 0

    last_raw = None

    t_total_start = time.ticks_us()

    for i in range(TEST_COUNT):
        t0 = time.ticks_us()
        raw = adc.read_all_raw10()
        t1 = time.ticks_us()

        dt = time.ticks_diff(t1, t0)

        last_raw = raw

        sum_us += dt

        if dt < min_us:
            min_us = dt

        if dt > max_us:
            max_us = dt

        if dt >= SLOW_READ_US:
            slow_count += 1

        if dt < 500:
            bin_500 += 1
        elif dt < 700:
            bin_700 += 1
        elif dt < 1000:
            bin_1000 += 1
        elif dt < 1500:
            bin_1500 += 1
        elif dt < 2000:
            bin_2000 += 1
        else:
            bin_over_2000 += 1

    t_total_end = time.ticks_us()
    total_us = time.ticks_diff(t_total_end, t_total_start)

    avg_us = sum_us // TEST_COUNT

    print("========== TLA2528 read_all_raw10 benchmark ==========")
    print("I2C_FREQ              =", I2C_FREQ)
    print("TEST_COUNT            =", TEST_COUNT)
    print("total_us              =", total_us)
    print("avg_us_per_read       =", avg_us)
    print("min_us_per_read       =", min_us)
    print("max_us_per_read       =", max_us)
    print("slow_count            =", slow_count)
    print("last_raw              =", last_raw)
    print()
    print("---- histogram ----")
    print("read_us < 500         =", bin_500)
    print("500 <= read_us < 700  =", bin_700)
    print("700 <= read_us < 1000 =", bin_1000)
    print("1000 <= read_us <1500 =", bin_1500)
    print("1500 <= read_us <2000 =", bin_2000)
    print("read_us >= 2000       =", bin_over_2000)
    print("======================================================")


# ==============================
# main
# ==============================

def main():
    disable_wifi()

    print()
    print("Initializing TLA2528...")

    adc = TLA2528(
        i2c_id=I2C_ID,
        sda_pin=SDA_PIN,
        scl_pin=SCL_PIN,
        freq=I2C_FREQ,
        address=ADC_ADDRESS,
        startup_delay_ms=STARTUP_DELAY_MS
    )

    adc.begin(verbose=True)

    benchmark_read_all(adc)


main()
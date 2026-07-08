# main_calibrate.py

from machine import Pin, I2C
from time import sleep
from tla2528 import TLA2528
from pai_line_sensor import PaiLineSensor

i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=400000)

adc = TLA2528(i2c)
adc.begin()

line_sensor = PaiLineSensor(adc, dark_is_low=True)

print("5초 동안 PAI-Car를 검은 라인과 흰색 바닥 위에서 좌우로 움직이세요.")
print("2초 후 측정을 시작합니다.")
sleep(2)

ok = line_sensor.calibrate_for(duration_ms=5000)

print("===== 센서 출력값 범위 측정 결과 =====")
print("센서별 최소값 cal_min:")
print(line_sensor.cal_min)
print("센서별 최대값 cal_max:")
print(line_sensor.cal_max)


if ok:
    line_sensor.save_calibration()
    print("센서 출력값 범위 측정 및 저장 완료")
    print("저장 파일: line_cal.json")
else:
    print("센서 출력값 범위 측정 실패")
    print("PAI-Car를 검은 라인과 흰색 바닥 위에서 충분히 움직였는지 확인하세요.")
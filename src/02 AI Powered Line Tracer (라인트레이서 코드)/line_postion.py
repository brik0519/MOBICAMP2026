from time import sleep_ms
from tla2528 import TLA2528
from pai_line_sensor import PaiLineSensor

adc = TLA2528(i2c_id=0, sda_pin=4, scl_pin=5)    # ADC 객체 생성
adc.begin()                                      # ADC 초기화(동작 준비)
line = PaiLineSensor(adc)                        # 라인 센서 값을 라인 위치 정보로 바꿔주는 객체 생성

print("라인센서 출력값 범위 측정 시작")
print("5초 동안 PAI-Car를 검은 라인과 흰색 바닥 위에서 움직이세요.")

ok = line.calibrate_for(duration_ms=5000)        # 각 센서의 최소 최대 범위 측정
print("cal_min:", line.cal_min)
print("cal_max:", line.cal_max)

if ok:
    line.save_calibration()                      # 각 센서의 최소 최대 범위 저장
    print("line_cal.json 저장 완료")

print("라인 위치 읽기 시작")
while True:                                      # 라인의 위치 계산 
    error, position, norm, on_line = line.read_error()
    print("norm:", norm, "pos:", position, "err:", error, "on:", on_line)
    sleep_ms(100)
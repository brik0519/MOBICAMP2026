from machine import Pin
import time

button = Pin(22, Pin.IN)

while True:
    value = button.value()        # 사용자 스위치가 연결된 GP22 핀의 상태를 읽음
    print(value, end=':')

    if value == 1:
        print("Button pressed")
    else:
        print("Button released")

    time.sleep(0.1)

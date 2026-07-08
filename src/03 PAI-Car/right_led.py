from machine import Pin
import time

# 오른쪽 LED: GP18
right_led = Pin(18, Pin.OUT)


while True:
    right_led.value(1)   # 오른쪽 LED 켜기
    time.sleep(0.5)

    right_led.value(0)   # 오른쪽 LED 끄기
    time.sleep(0.5)

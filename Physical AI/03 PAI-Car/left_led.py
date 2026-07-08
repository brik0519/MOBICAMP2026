from machine import Pin
import time

# 왼쪽 LED: GP19
left_led = Pin(19, Pin.OUT)

while True:
    left_led.value(1)   # 왼쪽 LED 켜기
    time.sleep(0.5)

    left_led.value(0)   # 왼쪽 LED 끄기
    time.sleep(0.5)

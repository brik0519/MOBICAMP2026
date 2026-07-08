from machine import Pin
import time

# 오른쪽 LED: GP18, 왼쪽 LED: GP19
right_led = Pin(18, Pin.OUT)
left_led = Pin(19, Pin.OUT)
try:
    while True:
        right_led.value(1); left_led.value(1)   # 오른쪽 LED 켜기, 왼쪽 LED 켜기
        time.sleep(0.4)

        right_led.value(0); left_led.value(0)   # 오른쪽 LED 끄기, 왼쪽 LED 끄기
        time.sleep(0.4)
except:
    right_led.value(0)
    left_led.value(0)
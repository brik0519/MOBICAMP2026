from machine import Pin
import time

HIGH = 1
LOW  = 0
button_pressed = False

led = Pin("LED", Pin.OUT)
button = Pin(22, Pin.IN)

blink_interval = 400
last_time = time.ticks_ms()

while True:
    now = time.ticks_ms()
    if time.ticks_diff(now, last_time) >= blink_interval:
        last_time = now
        led.toggle()

    btn = button.value()
    if btn == HIGH:
        button_pressed = True
    if button_pressed == True and btn == LOW:
        led.value(0)
        break

print("출발")

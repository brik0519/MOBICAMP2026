from machine import Pin
import time
HIGH = 1
LOW  = 0
button_pressed = False         # 버튼을 누른 적이 있다면 True를 할당하고 누른적이 없다면 False를 할당

right_led = Pin(18, Pin.OUT)
left_led = Pin(19, Pin.OUT)
button = Pin(22, Pin.IN)       # 안 누름 = HIGH, 누름 = LOW
builtin_LED = Pin("LED", Pin.OUT)
builtin_LED.value(0)

blink_interval = 400
last_time = time.ticks_ms()

while True:
    now = time.ticks_ms()
    if time.ticks_diff(now, last_time) >= blink_interval:   # 일정 시간이 지나면 LED 토글
        last_time = now
        right_led.toggle();         left_led.toggle()
        
    btn = button.value()        # 버튼 상태 읽기
    if btn == HIGH:               # 버튼이 눌리면 "누른 적 있음"으로 기억
        button_pressed = True
    if button_pressed == True and btn == LOW:  # 버튼을 누른 적이 있고, 지금은 버튼을 뗀 상태이면 무한 루프 탈출
        right_led.value(0);         left_led.value(0)
        break
    
builtin_LED.value(1)
print("출발")

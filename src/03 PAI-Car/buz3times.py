from machine import Pin, PWM
import time
BUZZER_PIN = 21
buzzer = PWM(Pin(BUZZER_PIN))
buzzer.freq(4000)

for i in range(3):
    buzzer.duty_u16(32768)
    time.sleep(0.2)
    buzzer.duty_u16(0)
    time.sleep(0.6)

buzzer.deinit()

# GP21을 확실히 LOW로 고정
# buzzer_pin = Pin(BUZZER_PIN, Pin.OUT)
# buzzer_pin.value(0)
Pin(BUZZER_PIN, Pin.OUT).value(0)

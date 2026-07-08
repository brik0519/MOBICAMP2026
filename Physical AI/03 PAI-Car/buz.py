from machine import Pin, PWM
import time
BUZZER_PIN = 21
buzzer = PWM(Pin(BUZZER_PIN))

# 4kHz 소리
buzzer.freq(500)
buzzer.freq(1000)
buzzer.freq(1500)
buzzer.duty_u16(32768)   # 50% duty
time.sleep(1)

# 소리 끄기
buzzer.duty_u16(0)         # 0% duty
buzzer.deinit()

# GP21을 확실히 LOW로 고정
# buzzer_pin = Pin(BUZZER_PIN, Pin.OUT)
# buzzer_pin.value(0)
Pin(BUZZER_PIN, Pin.OUT).value(0)
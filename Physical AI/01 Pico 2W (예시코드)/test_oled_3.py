from machine import Pin, I2C
import ssd1306
import time

# I2C0: SDA=GP0, SCL=GP1
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)

# OLED: 128 x 32, address 0x3C
oled = ssd1306.SSD1306_I2C(128, 32, i2c, addr=0x3C)

count = 0

# 마지막으로 OLED를 갱신한 시간
last_oled_time = time.ticks_ms()

while True:
    now = time.ticks_ms()

    # 1000ms, 즉 1초가 지났는지 확인
    if time.ticks_diff(now, last_oled_time) >= 1000:
        last_oled_time = now

        oled.fill(0)
        oled.text("PAI-Car", 0, 0)
        oled.text("No sleep demo", 0, 8)
        oled.text("Count: " + str(count), 0, 16)
        oled.text("OLED update", 0, 24)
        oled.show()

        count += 1

    # 여기에 다른 작업을 계속 넣을 수 있다.
    # 예: 센서 읽기, 버튼 확인, 모터 제어 등
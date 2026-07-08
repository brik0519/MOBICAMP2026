from machine import Pin, I2C
import ssd1306
import time

# I2C1: SDA=GP2, SCL=GP3
i2c = I2C(1, sda=Pin(2), scl=Pin(3), freq=400_000)

# OLED: 128 x 32, address 0x3C
oled = ssd1306.SSD1306_I2C(128, 32, i2c, addr=0x3C)

# 마지막으로 OLED를 갱신한 시간
start_time = time.ticks_ms()
last_oled_time = start_time

try:
    while True:
        now = time.ticks_ms()

        # 1000ms, 즉 1초가 지났는지 확인
        if time.ticks_diff(now, last_oled_time) >= 1000:
            last_oled_time = now
            elapsed_time = time.ticks_diff(now, start_time)
            sec_str = "{:.2f}".format(elapsed_time/1000)
            oled.fill(0)
            oled.text("PAI-Car", 0, 0)
            oled.text("No sleep demo", 0, 8)
            oled.text("Elapsed:"+sec_str, 0, 16)
            oled.show()

except:
    elapsed_time = time.ticks_diff(now, start_time)
    sec_str = "{:.2f}".format(elapsed_time/1000)
    oled.fill(0)
    oled.text("PAI-Car", 0, 0)
    oled.text("No sleep demo", 0, 8)
    oled.text("Elapsed:"+sec_str, 0, 16)
    oled.show()
    print(sec_str)
    

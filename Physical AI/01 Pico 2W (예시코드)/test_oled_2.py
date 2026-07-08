from machine import Pin, I2C
import ssd1306
import time

# I2C0: SDA=GP0, SCL=GP1
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)

# OLED: 128 x 32, address 0x3C
oled = ssd1306.SSD1306_I2C(128, 32, i2c, addr=0x3C)

count = 0

while True:
    oled.fill(0)                          # 화면 지우기
    oled.text("PAI-Car OLED", 0, 0)       # 첫 번째 줄
    oled.text("Count:", 0, 12)            # 두 번째 줄
    oled.text(str(count), 56, 12)         # count 값 출력
    oled.show()                           # OLED에 반영

    count += 1
    time.sleep(1)                         # 1초 대기
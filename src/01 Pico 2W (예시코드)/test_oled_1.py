from machine import Pin, I2C
import ssd1306
import time

# 브레드보드 테스트 기준
# I2C0: SDA=GP0, SCL=GP1
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)
time.sleep_ms(500)

# oled 0.91인치 : 128 x 32
oled = ssd1306.SSD1306_I2C(128, 32, i2c, addr=0x3C)

oled.fill(0)
oled.text("PAI-Car", 0, 0)        # 0, 0위치에 출력
oled.text("OLED 128x32", 0, 12)   # x = 0, y = 12 픽셀 위치에 출력
oled.text("ADDR 0x3C", 0, 24)     # x = 0, y = 24 픽셀 위치에 출력
oled.show()

#oled.fill(0)
#oled.text("PAI-Car", 0, 0)
#oled.text("AI Powered Car", 0, 8)
#oled.text("by Riatech", 0, 16)
#oled.text("Seoungpil Kim", 0, 24)
#oled.show()
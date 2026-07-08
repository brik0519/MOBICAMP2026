from machine import Pin, I2C
import time
from ria_bh1750 import BH1750

# Pico 2 W의 기본 I2C0 (SDA:GP0, SCL:GP1)
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)
sensor = BH1750(i2c, addr=0x23)

while True:
    lux = sensor.read_lux()
    print(f"Lux: {lux:.2f}[lx]")
    time.sleep(1)
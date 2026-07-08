# test_line_raw.py

from machine import Pin, I2C
from time import sleep_ms
from tla2528 import TLA2528

i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=400000)

adc = TLA2528(i2c)
adc.begin()

while True:
    raw = adc.read_all_raw10()
    print(raw)
    sleep_ms(200)
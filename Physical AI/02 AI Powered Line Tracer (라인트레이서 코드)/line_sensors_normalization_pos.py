# test_line_position.py

from machine import Pin, I2C
from time import sleep_ms
from tla2528 import TLA2528
from pai_line_sensor import PaiLineSensor

i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=400000)

adc = TLA2528(i2c)
adc.begin()

line_sensor = PaiLineSensor(adc, dark_is_low=True)
line_sensor.load_calibration()

while True:
    position, norm, on_line = line_sensor.read_line()
    print("norm:", norm)
    print("position:", position, "on_line:", on_line, end="\n\n")
    sleep_ms(200)
from machine import Pin, I2C
import time

# I2C1: SDA = GP2, SCL = GP3
i2c = I2C(1, sda=Pin(2), scl=Pin(3), freq=100000)

while True:
    devices = i2c.scan()

    if devices:
        print("I2C devices found:")
        for addr in devices:
            print("Address: ", hex(addr))
    else:
        print("No I2C device found.")

    print("----------------------")
    time.sleep(2)
# ria_bh1750.py
from machine import I2C
import time

class BH1750:
    # 명령어들
    _POWER_ON  = 0x01
    _RESET     = 0x07
    _CONT_HRES = 0x10  # Continuous High Resolution Mode (1 lx)

    def __init__(self, i2c: I2C, addr: int = 0x23):
        self.i2c = i2c
        self.addr = addr
        self._init_sensor()

    def _init_sensor(self):
        # Power On -> Reset -> Mode
        self.i2c.writeto(self.addr, bytes([self._POWER_ON]))
        time.sleep_ms(10)
        self.i2c.writeto(self.addr, bytes([self._RESET]))
        time.sleep_ms(10)
        self.i2c.writeto(self.addr, bytes([self._CONT_HRES]))
        time.sleep_ms(180)  # 첫 측정 대기

    def read_lux(self) -> float:
        """현재 조도(lux)를 읽어서 float로 반환"""
        data = self.i2c.readfrom(self.addr, 2)
        raw = (data[0] << 8) | data[1]
        return raw / 1.2

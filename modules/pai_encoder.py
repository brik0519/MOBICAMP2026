# modules/pai_encoder.py

from machine import Pin
from time import ticks_ms, ticks_diff


class WheelEncoders:
    def __init__(self, left_pin=16, right_pin=17, pullup=True):
        mode = Pin.PULL_UP if pullup else None

        self.left_pin = Pin(left_pin, Pin.IN, mode)
        self.right_pin = Pin(right_pin, Pin.IN, mode)

        self.left_ticks = 0
        self.right_ticks = 0

        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.last_ms = ticks_ms()

        self.dl = 0
        self.dr = 0
        self.dt_ms = 0

        self.left_speed = 0
        self.right_speed = 0
        self.distance_ticks = 0
        self.heading_ticks = 0

        self.left_dir = 1
        self.right_dir = 1

        self.left_pin.irq(
            trigger=Pin.IRQ_FALLING,
            handler=self._left_irq
        )
        self.right_pin.irq(
            trigger=Pin.IRQ_FALLING,
            handler=self._right_irq
        )

    def _left_irq(self, pin):
        self.left_ticks += self.left_dir

    def _right_irq(self, pin):
        self.right_ticks += self.right_dir

    def reset(self):
        self.left_ticks = 0
        self.right_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.last_ms = ticks_ms()

        self.dl = 0
        self.dr = 0
        self.dt_ms = 0

        self.left_speed = 0
        self.right_speed = 0
        self.distance_ticks = 0
        self.heading_ticks = 0

    def set_direction_from_cmd(self, left_cmd, right_cmd):
        if left_cmd >= 0:
            self.left_dir = 1
        else:
            self.left_dir = -1

        if right_cmd >= 0:
            self.right_dir = 1
        else:
            self.right_dir = -1

    def update(self):
        now = ticks_ms()
        dt = ticks_diff(now, self.last_ms)

        if dt <= 0:
            return

        lt = self.left_ticks
        rt = self.right_ticks

        self.dl = lt - self.last_left_ticks
        self.dr = rt - self.last_right_ticks
        self.dt_ms = dt

        self.left_speed = self.dl * 1000 // dt
        self.right_speed = self.dr * 1000 // dt

        self.distance_ticks = (lt + rt) // 2
        self.heading_ticks = rt - lt

        self.last_left_ticks = lt
        self.last_right_ticks = rt
        self.last_ms = now
# modules/pai_encoder.py

from machine import Pin, disable_irq, enable_irq
from time import ticks_ms, ticks_diff


class WheelEncoders:
    def __init__(self, left_pin=16, right_pin=17, pullup=True):
        if pullup:
            self.left_pin = Pin(left_pin, Pin.IN, Pin.PULL_UP)
            self.right_pin = Pin(right_pin, Pin.IN, Pin.PULL_UP)
        else:
            self.left_pin = Pin(left_pin, Pin.IN)
            self.right_pin = Pin(right_pin, Pin.IN)

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

        # 트랙 진행거리 추정용
        self.distance_ticks = 0

        # 좌우 회전 차이 추정용
        self.heading_ticks = 0

        # signed speed 평균이 아니라 실제 바퀴 회전량 기반 속도
        self.progress_speed = 0

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
        state = disable_irq()

        self.left_ticks = 0
        self.right_ticks = 0

        enable_irq(state)

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
        self.progress_speed = 0

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

        # IRQ가 tick 값을 바꾸는 중간에 읽지 않도록 짧게 보호
        state = disable_irq()

        lt = self.left_ticks
        rt = self.right_ticks

        enable_irq(state)

        self.dl = lt - self.last_left_ticks
        self.dr = rt - self.last_right_ticks
        self.dt_ms = dt

        # signed wheel speed
        self.left_speed = self.dl * 1000 // dt
        self.right_speed = self.dr * 1000 // dt

        # 진행거리:
        # 역회전이 섞여도 트랙 위치 추정을 위해 바퀴 회전량 절댓값을 누적한다.
        self.distance_ticks += (
            abs(self.dl) + abs(self.dr)
        ) // 2

        # 회전 방향성:
        # signed tick 차이는 차량 자세 변화 추정에 사용한다.
        self.heading_ticks = rt - lt

        # 실제 바퀴 움직임 기반 속도
        self.progress_speed = (
            abs(self.dl) + abs(self.dr)
        ) * 1000 // (2 * dt)

        self.last_left_ticks = lt
        self.last_right_ticks = rt
        self.last_ms = now
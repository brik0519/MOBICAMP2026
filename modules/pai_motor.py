# pai_motor.py
# PAI-Car motor control module
# MCU: Raspberry Pi Pico 2 W
# Motor driver: TB6612FNG
#
# Latest PAI-Car motor pin map:
#   A channel = Left motor
#   B channel = Right motor
#
# Test result:
#   Left motor direction  = normal
#   Right motor direction = inverted
#
# Speed range:
#   -1000 ~ +1000
#   + value: forward
#   - value: reverse
#    0     : stop / coast

from machine import Pin, PWM
from time import sleep_ms


# ------------------------------------------------------------
# User adjustable constants
# ------------------------------------------------------------

PWM_FREQ = 20000          # 20 kHz
MAX_SPEED = 1000          # external speed command range
MAX_DUTY = 65535          # MicroPython PWM duty_u16 max

DIR_CHANGE_DELAY_MS = 3   # delay when motor direction changes directly


# ------------------------------------------------------------
# Pin map for PAI-Car
# ------------------------------------------------------------

# Left motor, TB6612FNG A channel
PWMA_PIN = 11
AIN1_PIN = 10
AIN2_PIN = 15

# Right motor, TB6612FNG B channel
PWMB_PIN = 8
BIN1_PIN = 9
BIN2_PIN = 14


# ------------------------------------------------------------
# 모터의 회전 방향 조정
# ------------------------------------------------------------
# 예를 들어, drive(300,300)을 실행했을 때
#   오른쪽 모터 : 정상
#   왼쪽 모터   : 반대 반향으로 
#
# 왼쪽 모터의 회전 방향을 현 설정에서 반대로 변경하시오

LEFT_INVERT = True
RIGHT_INVERT = False


# ------------------------------------------------------------
# Utility functions
# ------------------------------------------------------------

def _clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def _speed_to_duty(speed):
    """
    Convert speed magnitude 0~1000 to PWM duty 0~65535.
    Integer-only version.
    """
    speed = abs(speed)

    if speed > MAX_SPEED:
        speed = MAX_SPEED

    return speed * MAX_DUTY // MAX_SPEED


# ------------------------------------------------------------
# Motor channel class
# ------------------------------------------------------------

class MotorChannel:
    def __init__(self, in1_pin, in2_pin, pwm_pin,
                 pwm_freq=PWM_FREQ, invert=False):
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)

        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(pwm_freq)
        self.pwm.duty_u16(0)

        self.invert = invert

        # Last logical direction:
        #   +1 = forward
        #   -1 = reverse
        #    0 = stopped / coast / brake
        self._last_dir = 0

        self.coast()

    def _set_forward_dir(self):
        """
        Set physical pin state for logical forward direction.

        Base direction:
            IN1=0, IN2=1

        If invert=True:
            IN1=1, IN2=0
        """
        if self.invert:
            self.in1.value(1)
            self.in2.value(0)
        else:
            self.in1.value(0)
            self.in2.value(1)

    def _set_reverse_dir(self):
        """
        Set physical pin state for logical reverse direction.

        Base reverse:
            IN1=1, IN2=0

        If invert=True:
            IN1=0, IN2=1
        """
        if self.invert:
            self.in1.value(0)
            self.in2.value(1)
        else:
            self.in1.value(1)
            self.in2.value(0)

    def drive(self, speed):
        """
        Drive one motor.

        speed range:
            +1000 : full forward
                0 : stop/coast
            -1000 : full reverse
        """
        speed = int(speed)

        if speed > MAX_SPEED:
            speed = MAX_SPEED
        elif speed < -MAX_SPEED:
            speed = -MAX_SPEED

        if speed == 0:
            self.coast()
            return

        if speed > 0:
            new_dir = 1
        else:
            new_dir = -1

        # Change direction pins only when direction changes.
        if new_dir != self._last_dir:
            if self._last_dir != 0:
                self.pwm.duty_u16(0)
                sleep_ms(DIR_CHANGE_DELAY_MS)

            if new_dir > 0:
                self._set_forward_dir()
            else:
                self._set_reverse_dir()

            self._last_dir = new_dir

        self.pwm.duty_u16(_speed_to_duty(speed))

    def coast(self):
        """
        Coast stop.
        Motor output is released.
        """
        self.pwm.duty_u16(0)
        self.in1.value(0)
        self.in2.value(0)
        self._last_dir = 0

    def brake(self):
        """
        Active brake.
        """
        self.pwm.duty_u16(MAX_DUTY)
        self.in1.value(1)
        self.in2.value(1)
        self._last_dir = 0

    def set_pwm_freq(self, freq):
        self.pwm.freq(freq)

    def deinit(self):
        self.coast()
        self.pwm.deinit()


# ------------------------------------------------------------
# PAI-Car motor controller
# ------------------------------------------------------------

class PAICarMotors:
    def __init__(self, pwm_freq=PWM_FREQ):
        # A channel = left motor
        self.left = MotorChannel(
            AIN1_PIN,
            AIN2_PIN,
            PWMA_PIN,
            pwm_freq,
            invert=LEFT_INVERT
        )

        # B channel = right motor
        self.right = MotorChannel(
            BIN1_PIN,
            BIN2_PIN,
            PWMB_PIN,
            pwm_freq,
            invert=RIGHT_INVERT
        )

    def drive(self, left_speed, right_speed):
        """
        Drive left and right motors.

        left_speed, right_speed:
            -1000 ~ +1000

        Example:
            drive(300, 300)     # forward
            drive(-300, -300)   # backward
            drive(200, 500)     # turn left while moving
            drive(500, 200)     # turn right while moving
        """
        self.left.drive(left_speed)
        self.right.drive(right_speed)

    def stop(self, mode="coast"):
        """
        Stop both motors.

        mode:
            "coast" : free stop
            "brake" : active brake
        """
        if mode == "brake":
            self.left.brake()
            self.right.brake()
        else:
            self.left.coast()
            self.right.coast()

    def forward(self, speed):
        speed = abs(int(speed))
        self.drive(speed, speed)

    def backward(self, speed):
        speed = abs(int(speed))
        self.drive(-speed, -speed)

    def turn_left(self, speed, ratio=0.5):
        """
        Smooth left turn while moving forward.
        """
        speed = abs(int(speed))
        inner = int(speed * ratio)
        self.drive(inner, speed)

    def turn_right(self, speed, ratio=0.5):
        """
        Smooth right turn while moving forward.
        """
        speed = abs(int(speed))
        inner = int(speed * ratio)
        self.drive(speed, inner)

    def pivot_left(self, speed):
        """
        Rotate left in place.
        Left motor reverse, right motor forward.
        """
        speed = abs(int(speed))
        self.drive(-speed, speed)

    def pivot_right(self, speed):
        """
        Rotate right in place.
        Left motor forward, right motor reverse.
        """
        speed = abs(int(speed))
        self.drive(speed, -speed)

    def drive_with_correction(self, base_speed, correction):
        """
        Helper for PID line tracing.

        Default formula:
            left  = base_speed + correction
            right = base_speed - correction

        For the fastest control loop, calculate left/right speed
        in the main control code and call drive(left, right) directly.
        """
        left_speed = int(base_speed + correction)
        right_speed = int(base_speed - correction)

        left_speed = _clamp(left_speed, -MAX_SPEED, MAX_SPEED)
        right_speed = _clamp(right_speed, -MAX_SPEED, MAX_SPEED)

        self.drive(left_speed, right_speed)

    def set_pwm_freq(self, freq):
        self.left.set_pwm_freq(freq)
        self.right.set_pwm_freq(freq)

    def deinit(self):
        self.left.deinit()
        self.right.deinit()


# ------------------------------------------------------------
# Simple test function
# ------------------------------------------------------------

def motor_test():
    """
    Simple motor test.
    Run manually from main.py or Thonny shell:

        import pai_motor
        pai_motor.motor_test()
    """
    motors = PAICarMotors()

    try:
        print("Forward 300")
        motors.forward(300)
        sleep_ms(1000)

        print("Stop")
        motors.stop()
        sleep_ms(1000)

        print("Backward 300")
        motors.backward(300)
        sleep_ms(1000)

        print("Stop")
        motors.stop()
        sleep_ms(1000)

        print("Left motor only")
        motors.drive(300, 0)
        sleep_ms(1000)

        print("Stop")
        motors.stop()
        sleep_ms(1000)

        print("Right motor only")
        motors.drive(0, 300)
        sleep_ms(1000)

        print("Stop")
        motors.stop()

    except KeyboardInterrupt:
        motors.stop()
        print("Motor test interrupted")
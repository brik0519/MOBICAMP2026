from time import sleep_ms
from pai_motor import PAICarMotors

motors = PAICarMotors()
motors.stop();

try:
    # 전진하면서 좌회전
    print("forward left"); motors.drive(100, 400);     sleep_ms(2000)
    print("stop");           motors.drive(0, 0);            sleep_ms(100)

except KeyboardInterrupt:
    print("Interrupted")

finally:
    motors.stop()
    print("Motor stopped")

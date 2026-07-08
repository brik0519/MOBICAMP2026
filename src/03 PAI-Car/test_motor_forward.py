from time import sleep_ms
from pai_motor import PAICarMotors

motors = PAICarMotors()
motors.stop();

try:
    while True:
        print("forward");    motors.drive(300, 300)
        sleep_ms(200)
        print("stop");       motors.drive(0, 0)
        sleep_ms(1000)

except KeyboardInterrupt:
    print("Interrupted")

finally:
    motors.stop()
    print("Motor stopped")

from time import sleep_ms
from pai_motor import PAICarMotors

motors = PAICarMotors()
motors.stop();

try:
    print("forward");    motors.drive(300, 300);     sleep_ms(200)
    print("stop");         motors.drive(0, 0);       sleep_ms(20)
    print("backward");  motors.drive(-300, -300);    sleep_ms(200)
    print("stop");         motors.drive(0, 0);       sleep_ms(20)

    # 전진하면서 좌회전
    #print("forward left"); motors.drive(100, 400);     sleep_ms(800)
    #print("stop");            motors.drive(0, 0);           sleep_ms(100)

    # 전진하면서 우회전
    #print("forward right"); motors.drive(400, 100);      sleep_ms(800)
    #print("stop");             motors.drive(0, 0)

except KeyboardInterrupt:
    print("Interrupted")

finally:
    motors.stop()
    print("Motor stopped")

# main.py
# PAI-Car Step 3 main
#
# 기능:
#   1. 기존 라인트레이싱 주행 유지
#   2. Pico -> PC UDP telemetry 송신 유지
#   3. PC -> Pico UDP command 수신 추가
#
# PC 키:
#   P       PING
#   Space   STOP
#   Z       SAFE_MODE
#   Enter   RUN
#
# 주의:
#   3단계에서는 SAFE_MODE를 저속 주행이 아니라 안전 정지로 처리한다.
#   속도/PID 원격 변경은 아직 하지 않는다.

from time import ticks_ms, ticks_diff, sleep_ms

from modules.pai_car_run_support import (
    create_lap_timer,
    setup_paicar,
    self_calibrate_or_stop,
    wait_button_start,
    limit_cmd,
    wait_control_period,
)

from modules.pai_udp_telemetry import (
    PAIUdpTelemetry,
    read_line_detail,
    is_t_marker_area,
)

from modules.pai_udp_command import PAIUdpCommand


# ------------------------------------------------------------
# User tunable baseline values
# ------------------------------------------------------------
# BASELINE 확보 때 사용한 값이 다르면 이 값만 기존값으로 바꿔라.

BASE_SPEED = 260

# 정수 연산용 PD gain
# steer = (KP_X1000 * error + KD_X1000 * d_error) // 1000
KP_X1000 = 180
KD_X1000 = 75


# ------------------------------------------------------------
# Run options
# ------------------------------------------------------------

# 이미 line_cal.json이 있고 baseline을 확보했다면 False 권장
RUN_SELF_CALIBRATION = False

# 버튼을 눌러 출발하려면 True
WAIT_FOR_BUTTON = True


# ------------------------------------------------------------
# Helper
# ------------------------------------------------------------

def compute_steer(error, d_error):
    return (KP_X1000 * error + KD_X1000 * d_error) // 1000


def apply_pd_control(base_speed, error, d_error):
    steer = compute_steer(error, d_error)

    left_cmd = limit_cmd(base_speed + steer)
    right_cmd = limit_cmd(base_speed - steer)

    return left_cmd, right_cmd


def stop_motors(motors):
    motors.stop("brake")


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

def main():
    motors = None
    telemetry = None
    cmd = None

    try:
        lap_timer = create_lap_timer()

        line, motors, button = setup_paicar(lap_timer)

        if RUN_SELF_CALIBRATION:
            self_calibrate_or_stop(line, motors, lap_timer)

        # Wi-Fi 연결 및 Pico -> PC telemetry 준비
        telemetry = PAIUdpTelemetry(lap_timer)
        telemetry.begin()

        # PC -> Pico command 수신 준비
        # 3단계에서는 heartbeat timeout을 강제하지 않는다.
        cmd = PAIUdpCommand(require_heartbeat=False)
        cmd.begin()

        if WAIT_FOR_BUTTON:
            wait_button_start(button, lap_timer)

        lap_timer.start()
        telemetry.reset_timer()

        prev_error = 0

        # STOP/SAFE 상태에서 brake 명령을 매 loop마다 반복하지 않기 위한 플래그
        force_stop_applied = False

        while True:
            loop_start = ticks_ms()

            # ------------------------------------------------
            # 1. PC command 수신
            # ------------------------------------------------
            # non-blocking poll이므로 주행 루프를 막지 않는다.
            cmd.poll()

            # ------------------------------------------------
            # 2. 센서 읽기
            # ------------------------------------------------
            error, position, norm, on_line = read_line_detail(line)

            d_error = error - prev_error
            prev_error = error

            is_marker = is_t_marker_area(norm, on_line)

            # ------------------------------------------------
            # 3. Finish 판별
            # ------------------------------------------------
            if lap_timer.check_finish(norm, on_line):
                left_cmd = 0
                right_cmd = 0

                stop_motors(motors)

                telemetry.send_now(
                    BASE_SPEED,
                    norm,
                    position,
                    error,
                    d_error,
                    left_cmd,
                    right_cmd,
                    on_line,
                    is_marker,
                )

                break

            # ------------------------------------------------
            # 4. 기본 PD 제어값 계산
            # ------------------------------------------------
            left_cmd, right_cmd = apply_pd_control(
                BASE_SPEED,
                error,
                d_error,
            )

            # ------------------------------------------------
            # 5. STOP / SAFE_MODE 명령 반영
            # ------------------------------------------------
            if cmd.should_force_stop():
                left_cmd = 0
                right_cmd = 0

                if not force_stop_applied:
                    stop_motors(motors)
                    force_stop_applied = True

            else:
                motors.drive(left_cmd, right_cmd)
                force_stop_applied = False

            # ------------------------------------------------
            # 6. Telemetry 송신
            # ------------------------------------------------
            telemetry.send_if_due(
                BASE_SPEED,
                norm,
                position,
                error,
                d_error,
                left_cmd,
                right_cmd,
                on_line,
                is_marker,
            )

            # ------------------------------------------------
            # 7. OLED 시간 표시 및 제어 주기 유지
            # ------------------------------------------------
            lap_timer.update()
            wait_control_period(loop_start)

        # Finish 이후 정지 유지
        stop_motors(motors)
        lap_timer.show_stopped()

        while True:
            if cmd is not None:
                cmd.poll()
            sleep_ms(100)

    except KeyboardInterrupt:
        print("KeyboardInterrupt")

    except Exception as exc:
        print("main error:", exc)

    finally:
        if motors is not None:
            try:
                motors.stop("brake")
            except Exception:
                pass

        if telemetry is not None:
            try:
                telemetry.close()
            except Exception:
                pass

        if cmd is not None:
            try:
                cmd.close()
            except Exception:
                pass

        print("PAI-Car stopped")


main()
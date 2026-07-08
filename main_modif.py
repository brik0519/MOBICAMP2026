# main.py
# PAI-Car line tracing
#
# DEBUG_MODE = True
#   - 제어 주기 10 ms
#   - Wi-Fi 연결
#   - UDP 텔레메트리 전송
#
# DEBUG_MODE = False
#   - 제어 주기 5 ms
#   - Wi-Fi 및 UDP 비활성화
#   - 레이스 전용

from time import ticks_ms

import modules.pai_car_run_support as run_support

from modules.pai_car_run_support import (
    create_lap_timer,
    setup_paicar,
    self_calibrate_or_stop,
    wait_button_start,
    limit_cmd,
    wait_control_period,
)


# ------------------------------------------------------------
# 1. Running mode
# ------------------------------------------------------------
DEBUG_MODE = True

DEBUG_CONTROL_MS = 10
RACE_CONTROL_MS = 5

if DEBUG_MODE:
    CONTROL_PERIOD_MS = DEBUG_CONTROL_MS
else:
    CONTROL_PERIOD_MS = RACE_CONTROL_MS

# wait_control_period()가 참조하는 제어 주기 변경
run_support.CONTROL_MS = CONTROL_PERIOD_MS


# ------------------------------------------------------------
# 2. UDP telemetry import
# ------------------------------------------------------------
# 레이스 모드에서는 Wi-Fi 관련 모듈 자체를 불러오지 않는다.

if DEBUG_MODE:
    import modules.pai_udp_telemetry as udp_telemetry

    from modules.pai_udp_telemetry import (
        PAIUdpTelemetry,
        read_line_detail,
        is_t_marker_area,
    )

    # pai_udp_telemetry.py 내부에서 CONTROL_MS를 별도로 import했다면
    # 실제 제어 주기와 동일한 값으로 맞춘다.
    udp_telemetry.CONTROL_MS = CONTROL_PERIOD_MS

else:
    PAIUdpTelemetry = None


# ------------------------------------------------------------
# 3. PD-control settings
# ------------------------------------------------------------
BASE_SPEED = 1000

KP = 0.55
KD = 0.22


# ------------------------------------------------------------
# 4. Constants and states
# ------------------------------------------------------------
STATE_TRACKING = 0
STATE_LINE_LOST = 1


# ------------------------------------------------------------
# 5. Helper functions
# ------------------------------------------------------------
def calculate_pd_commands(error, last_error):
    """
    PD 제어로 좌우 모터 명령을 계산한다.
    """
    d_error = error - last_error

    correction = int(
        KP * error
        + KD * d_error
    )

    left_cmd = limit_cmd(
        BASE_SPEED + correction
    )

    right_cmd = limit_cmd(
        BASE_SPEED - correction
    )

    return (
        left_cmd,
        right_cmd,
        d_error,
    )


def read_sensor_data(line):
    """
    실행 모드와 관계없이 같은 형태의 센서 데이터를 반환한다.

    반환값:
        error
        position
        norm
        on_line
        is_marker
    """
    if DEBUG_MODE:
        error, position, norm, on_line = read_line_detail(line)

        is_marker = is_t_marker_area(
            norm,
            on_line,
        )

        return (
            error,
            position,
            norm,
            on_line,
            is_marker,
        )

    # 레이스 모드에서도 read_line_detail()을 사용하려면
    # pai_udp_telemetry 모듈을 import해야 하므로,
    # 여기서는 line 객체의 기존 API를 직접 사용한다.
    error, position, norm, on_line = line.read_error()

    # 레이스 중에는 UDP 마커 정보가 필요하지 않다.
    is_marker = False

    return (
        error,
        position,
        norm,
        on_line,
        is_marker,
    )


def show_running_mode(lap_timer):
    """
    출발 전에 현재 모드를 OLED에 표시한다.
    """
    if DEBUG_MODE:
        mode_name = "DEBUG UDP"
    else:
        mode_name = "RACE"

    try:
        lap_timer.show(
            mode_name,
            "{} ms".format(CONTROL_PERIOD_MS),
            "Ready",
            "",
        )
    except Exception:
        pass


def run():
    """
    차량 초기화 및 PD 제어 루프 실행.
    """
    lap_timer = None
    motors = None
    telemetry = None

    finished = False

    try:
        # ----------------------------------------------------
        # 6. Setup
        # ----------------------------------------------------
        lap_timer = create_lap_timer()

        line, motors, button = setup_paicar(
            lap_timer
        )

        # ----------------------------------------------------
        # 7. UDP telemetry setup
        # ----------------------------------------------------
        if DEBUG_MODE:
            telemetry = PAIUdpTelemetry(
                lap_timer
            )

            telemetry.begin()

        # ----------------------------------------------------
        # 8. Self calibration
        # ----------------------------------------------------
        self_calibrate_or_stop(
            line,
            motors,
            lap_timer,
        )

        show_running_mode(lap_timer)

        # ----------------------------------------------------
        # 9. Button start
        # ----------------------------------------------------
        wait_button_start(
            button,
            lap_timer,
        )

        lap_timer.start()

        if telemetry is not None:
            telemetry.reset_timer()

        # ----------------------------------------------------
        # 10. Controller state initialization
        # ----------------------------------------------------
        drive_state = STATE_TRACKING

        last_error = 0
        d_error = 0

        left_cmd = 0
        right_cmd = 0

        # ----------------------------------------------------
        # 11. PD-control line tracing
        # ----------------------------------------------------
        while True:
            loop_start = ticks_ms()

            # ------------------------------------------------
            # 11-1. Sensor input
            # ------------------------------------------------
            (
                error,
                position,
                norm,
                on_line,
                is_marker,
            ) = read_sensor_data(line)

            # ------------------------------------------------
            # 11-2. Finish detection
            # ------------------------------------------------
            if lap_timer.check_finish(
                norm,
                on_line,
            ):
                motors.stop()

                finished = True

                if telemetry is not None:
                    telemetry.send_now(
                        BASE_SPEED,
                        norm,
                        position,
                        error,
                        0,
                        0,
                        0,
                        on_line,
                        is_marker,
                    )

                break

            # ------------------------------------------------
            # 11-3. PD control
            # ------------------------------------------------
            if on_line:
                drive_state = STATE_TRACKING

                (
                    left_cmd,
                    right_cmd,
                    d_error,
                ) = calculate_pd_commands(
                    error,
                    last_error,
                )

                last_error = error

                motors.drive(
                    left_cmd,
                    right_cmd,
                )

            else:
                drive_state = STATE_LINE_LOST

                motors.stop()

                d_error = 0
                left_cmd = 0
                right_cmd = 0
                last_error = 0

            # ------------------------------------------------
            # 11-4. UDP telemetry
            # ------------------------------------------------
            if telemetry is not None:
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
            # 11-5. OLED update
            # ------------------------------------------------
            lap_timer.update()

            # ------------------------------------------------
            # 11-6. Control period
            # ------------------------------------------------
            # DEBUG_MODE = True  -> 10 ms
            # DEBUG_MODE = False -> 5 ms
            wait_control_period(
                loop_start
            )

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        if lap_timer is not None:
            try:
                lap_timer.show(
                    "ERROR",
                    exc.__class__.__name__[:16],
                    "Motor stopped",
                    "",
                )
            except Exception:
                pass

        raise

    finally:
        if motors is not None:
            motors.stop()

        if telemetry is not None:
            telemetry.close()

        if (
            lap_timer is not None
            and not finished
        ):
            try:
                lap_timer.show_stopped()
            except Exception:
                pass


# ------------------------------------------------------------
# 12. Program entry point
# ------------------------------------------------------------
run()
# main.py
# PAI-Car line tracing
# Stage 2: execution structure refactoring only
#
# This version preserves the existing support-function call signatures.
# The control law remains a simple P controller so that architecture can be
# changed without simultaneously changing vehicle behavior.

from time import ticks_ms

from modules.pai_car_run_support import (
    create_lap_timer,
    setup_paicar,
    self_calibrate_or_stop,
    wait_button_start,
    read_line,
    limit_cmd,
    wait_control_period,
)


# ------------------------------------------------------------
# 1. User parameters
# ------------------------------------------------------------
# Replace these two values with the values used by the current stable main.py.
BASE_SPEED = 350
KP = 0.20

# Behavior when the line is not detected.
# Keep this equal to the current stable implementation during Stage 2.
LINE_LOST_LEFT_CMD = 0
LINE_LOST_RIGHT_CMD = 0


# ------------------------------------------------------------
# 2. Constants and states
# ------------------------------------------------------------
STATE_TRACKING = 0
STATE_LINE_LOST = 1


# ------------------------------------------------------------
# 3. Helper functions defined in main.py
# ------------------------------------------------------------
def calculate_p_commands(error):
    """
    Calculate motor commands using the existing P-control behavior.

    Positive correction increases the left motor command and decreases
    the right motor command.
    """
    correction = KP * error

    left_cmd = BASE_SPEED + correction
    right_cmd = BASE_SPEED - correction

    return limit_cmd(left_cmd), limit_cmd(right_cmd)


def calculate_line_lost_commands():
    """
    Preserve the current line-loss behavior during Stage 2.

    Stage 7 will replace this function with a recovery state machine.
    """
    return (
        limit_cmd(LINE_LOST_LEFT_CMD),
        limit_cmd(LINE_LOST_RIGHT_CMD),
    )


def calculate_drive_commands(error, on_line):
    """
    Select the current controller without changing library APIs.
    """
    if on_line:
        return STATE_TRACKING, calculate_p_commands(error)

    return STATE_LINE_LOST, calculate_line_lost_commands()


def run():
    """
    Initialize the vehicle and execute the deterministic control loop.
    """
    lap_timer = None
    motors = None

    try:
        # ----------------------------------------------------
        # 4. Hardware setup
        # ----------------------------------------------------
        lap_timer = create_lap_timer()
        line, motors, button = setup_paicar(lap_timer)

        # ----------------------------------------------------
        # 5. Calibration and start wait
        # ----------------------------------------------------
        self_calibrate_or_stop(line, motors, lap_timer)
        wait_button_start(button, lap_timer)

        # ----------------------------------------------------
        # 6. Controller state initialization
        # ----------------------------------------------------
        drive_state = STATE_TRACKING
        lap_timer.start()

        # ----------------------------------------------------
        # 7. Deterministic control loop
        # ----------------------------------------------------
        while True:
            loop_start = ticks_ms()

            # Sensor input
            error, on_line, norm = read_line(line)

            # Finish detection uses the same sensor sample.
            if lap_timer.check_finish(norm, on_line):
                motors.stop()
                break

            # Current P-control behavior
            drive_state, commands = calculate_drive_commands(
                error,
                on_line,
            )

            left_cmd, right_cmd = commands

            # Final motor output
            motors.drive(left_cmd, right_cmd)

            # Low-frequency OLED update
            lap_timer.update()

            # Keep the existing control-period function call
            wait_control_period(loop_start)

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        # Show the error when possible, but do not hide it.
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
        # ----------------------------------------------------
        # 8. Guaranteed motor stop
        # ----------------------------------------------------
        if motors is not None:
            motors.stop()

        if lap_timer is not None:
            try:
                lap_timer.show_stopped()
            except Exception:
                pass


# ------------------------------------------------------------
# 9. Program entry point
# ------------------------------------------------------------
run()
# main.py
# PAI-Car v1.0 profile 기반 PD 제어 라인트레이싱 + 주행 시간 측정 + UDP telemetry V2
#
# 유지:
#   - 엔코더 없는 모터 기준
#   - 버튼 직후 시작선 중복 finish 오검출 방지
#   - Z 긴급 정지 / Enter 재개
#   - Space section advance
#
# 변경:
#   - telemetry V2에 Pico 실제 section/profile 상태 전송
#   - telemetry V2에 마지막 command 처리 결과 전송
#   - PC/Pico section sync 확인 가능

from time import ticks_ms, ticks_diff, ticks_add

from modules.pai_car_run_support import (
    create_lap_timer,
    setup_paicar,
    self_calibrate_or_stop,
    wait_button_start,
    wait_control_period,
)

from modules.pai_udp_telemetry import (
    PAIUdpTelemetry,
    RUN_STATE_STOP,
    RUN_STATE_RUN,
    read_line_detail,
)

from modules.pai_udp_command import PAIUdpCommand
from modules.pai_drive_profiles import DriveProfileManager


# ------------------------------------------------------------
# Marker display / logging settings
# ------------------------------------------------------------

T_MARKER_TH = 700
T_MARKER_MIN_COUNT = 6

START_MARKER_RELEASE_COUNT = 5


# ------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------

def count_black_sensors(norm):
    count = 0

    for v in norm:
        if v >= T_MARKER_TH:
            count += 1

    return count


def marker_detected_now(norm, on_line):
    if not on_line:
        return False

    black_count = count_black_sensors(norm)

    return black_count >= T_MARKER_MIN_COUNT


def arm_start_marker_if_needed(lap_timer, norm, on_line):
    if marker_detected_now(norm, on_line):
        lap_timer.t_marker_count = 1
        lap_timer.t_marker_active = True
        lap_timer.t_marker_release_count = 0
        return True

    return False


def update_telemetry_command_echo(
    telemetry,
    cmd,
    drive_profiles,
    run_state
):
    telemetry.set_command_echo(
        run_state,
        drive_profiles.get_section_id(),
        drive_profiles.get_profile_key(),
        cmd.get_last_cmd_seq(),
        cmd.get_last_cmd_type(),
        cmd.get_last_cmd_status()
    )


def send_stop_packet(
    telemetry,
    target_speed,
    norm,
    position,
    error,
    on_line,
    is_marker
):
    telemetry.send_now(
        target_speed,
        norm,
        position,
        error,
        0,  # d_error
        0,  # left_cmd
        0,  # right_cmd
        on_line,
        is_marker
    )


def enter_pause(lap_timer):
    motors.stop()
    lap_timer.show("PAUSE", "Z STOP", "Enter: RUN", "")


def exit_pause(lap_timer):
    lap_timer.t_marker_active = False
    lap_timer.t_marker_release_count = 0
    lap_timer.show("RUN", "RESUME", "", "")


# ------------------------------------------------------------
# Setup
# ------------------------------------------------------------

lap_timer = create_lap_timer()

line, motors, button = setup_paicar(lap_timer)

telemetry = PAIUdpTelemetry(lap_timer)
telemetry.begin()

cmd = PAIUdpCommand(require_heartbeat=False)
cmd.begin()

drive_profiles = DriveProfileManager()


# ------------------------------------------------------------
# Self calibration
# ------------------------------------------------------------

self_calibrate_or_stop(line, motors, lap_timer)


# ------------------------------------------------------------
# Button start
# ------------------------------------------------------------

wait_button_start(button, lap_timer)

lap_timer.start()
telemetry.reset_timer()

drive_profiles.reset()

update_telemetry_command_echo(
    telemetry,
    cmd,
    drive_profiles,
    RUN_STATE_RUN
)

print(drive_profiles.debug_text())

run_start_ms = ticks_ms()


# ------------------------------------------------------------
# Start marker guard
# ------------------------------------------------------------

_start_error, _start_position, _start_norm, _start_on_line = read_line_detail(line)

start_marker_armed = arm_start_marker_if_needed(
    lap_timer,
    _start_norm,
    _start_on_line
)

start_marker_released = not start_marker_armed
start_marker_release_count = 0


# ------------------------------------------------------------
# Profile-based line tracing
# ------------------------------------------------------------

finished = False

last_error = 0
left_cmd = 0
right_cmd = 0
d_error = 0

was_on_line = False

target_speed = drive_profiles.get_base_speed()

line_lost_start_ms = None

paused = False
pause_start_ms = 0

try:
    while True:
        loop_start = ticks_ms()
        now_ms = loop_start

        # --------------------------------------------------------
        # 0. PC command 수신
        # --------------------------------------------------------

        cmd.poll()
        force_stop = cmd.should_force_stop()

        section_changed = drive_profiles.sync_section_id(
            cmd.get_track_section_id()
        )

        if section_changed:
            print(drive_profiles.debug_text())

        # --------------------------------------------------------
        # 0-1. Z emergency stop pause / Enter resume
        # --------------------------------------------------------

        if force_stop and not paused:
            paused = True
            pause_start_ms = now_ms

            target_speed = 0
            left_cmd = 0
            right_cmd = 0
            d_error = 0

            motors.stop()
            enter_pause(lap_timer)

        elif paused and not force_stop:
            paused_ms = ticks_diff(now_ms, pause_start_ms)

            run_start_ms = ticks_add(run_start_ms, paused_ms)
            lap_timer.start_ms = ticks_add(lap_timer.start_ms, paused_ms)
            lap_timer.last_update_ms = ticks_add(lap_timer.last_update_ms, paused_ms)

            exit_pause(lap_timer)

            paused = False
            pause_start_ms = 0

        elapsed_ms = ticks_diff(now_ms, run_start_ms)

        # elapsed_ms는 현재 telemetry/log 분석용으로 유지한다.
        # 실제 속도 결정은 section profile이 담당한다.
        _ = elapsed_ms

        # --------------------------------------------------------
        # 1. 라인센서 읽기
        # --------------------------------------------------------

        error, position, norm, on_line = read_line_detail(line)

        is_marker = marker_detected_now(norm, on_line)

        # --------------------------------------------------------
        # 2. Pause 상태
        # --------------------------------------------------------

        if paused:
            target_speed = 0
            left_cmd = 0
            right_cmd = 0
            d_error = 0
            motors.stop()

        else:
            # ----------------------------------------------------
            # 3. Finish 확인
            # ----------------------------------------------------

            if not start_marker_released:
                if marker_detected_now(norm, on_line):
                    start_marker_release_count = 0
                else:
                    start_marker_release_count += 1

                    if start_marker_release_count >= START_MARKER_RELEASE_COUNT:
                        start_marker_released = True
                        lap_timer.t_marker_active = False
                        lap_timer.t_marker_release_count = 0

            else:
                if lap_timer.check_finish(norm, on_line):
                    d_error = 0
                    left_cmd = 0
                    right_cmd = 0
                    target_speed = 0

                    motors.stop()
                    finished = True

                    update_telemetry_command_echo(
                        telemetry,
                        cmd,
                        drive_profiles,
                        RUN_STATE_STOP
                    )

                    send_stop_packet(
                        telemetry,
                        target_speed,
                        norm,
                        position,
                        error,
                        on_line,
                        is_marker
                    )

                    break

            # ----------------------------------------------------
            # 4. Profile 기반 PD 제어 + 라인 미검출 복구
            # ----------------------------------------------------

            if on_line:
                line_lost_start_ms = None

                if was_on_line:
                    d_error = error - last_error
                else:
                    d_error = 0

                last_error = error
                was_on_line = True

                target_speed = drive_profiles.compute_target_speed(
                    error,
                    d_error
                )

                correction = drive_profiles.compute_correction(
                    error,
                    d_error
                )

                left_cmd = drive_profiles.limit_drive_cmd(
                    target_speed + correction,
                    error
                )

                right_cmd = drive_profiles.limit_drive_cmd(
                    target_speed - correction,
                    error
                )

                motors.drive(left_cmd, right_cmd)

            else:
                now = ticks_ms()

                if line_lost_start_ms is None:
                    line_lost_start_ms = now

                loss_ms = ticks_diff(now, line_lost_start_ms)

                d_error = 0
                target_speed = 0
                was_on_line = False

                search_pwm = drive_profiles.get_search_pwm()
                line_loss_max_ms = drive_profiles.get_line_loss_max_ms()

                if loss_ms <= line_loss_max_ms:
                    if last_error < 0:
                        left_cmd = -search_pwm
                        right_cmd = search_pwm
                    else:
                        left_cmd = search_pwm
                        right_cmd = -search_pwm

                    motors.drive(left_cmd, right_cmd)

                else:
                    left_cmd = 0
                    right_cmd = 0
                    motors.stop()

        # --------------------------------------------------------
        # 5. Telemetry V2 command/profile echo 갱신
        # --------------------------------------------------------

        if paused or force_stop:
            run_state = RUN_STATE_STOP
        else:
            run_state = RUN_STATE_RUN

        update_telemetry_command_echo(
            telemetry,
            cmd,
            drive_profiles,
            run_state
        )

        # --------------------------------------------------------
        # 6. 주행 데이터 전송
        # --------------------------------------------------------

        telemetry.send_if_due(
            target_speed,
            norm,
            position,
            error,
            d_error,
            left_cmd,
            right_cmd,
            on_line,
            is_marker
        )

        # --------------------------------------------------------
        # 7. OLED 갱신
        # --------------------------------------------------------

        if not paused:
            lap_timer.update()

        # --------------------------------------------------------
        # 8. 제어 주기 맞추기
        # --------------------------------------------------------

        wait_control_period(loop_start)


finally:
    motors.stop()

    update_telemetry_command_echo(
        telemetry,
        cmd,
        drive_profiles,
        RUN_STATE_STOP
    )

    telemetry.close()
    cmd.close()

    if not finished:
        lap_timer.show_stopped()
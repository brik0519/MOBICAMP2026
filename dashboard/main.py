# ai_control_udp.py
# PAI-Car autonomous AI line tracing with UDP telemetry
#
# 목적:
#   - train.py가 만든 paicar_lr_model.py Ridge Linear Regression 모델 사용
#   - PC Space 입력 없이 elapsed time 기반으로 section/profile을 자동 진행
#   - 현재 section/profile 값을 AI feature에 넣어 left_cmd/right_cmd 예측
#   - AI 출력은 기존 PD/profile 계산값 주변으로 제한해 급격한 이탈 방지
#   - UDP telemetry는 기존 dashboard/app.py로 계속 수신 가능
#
# 필요 파일:
#   - paicar_lr_model.py        # train.py 출력 파일
#   - modules/pai_car_run_support.py
#   - modules/pai_udp_telemetry.py
#   - modules/pai_drive_profiles.py
#   - 기타 hardware module들
#
# 선택 파일:
#   - modules/pai_udp_command.py
#     있으면 Z=STOP, Enter=RUN 비상 제어만 사용한다.
#     Space/NEXT_SECTION은 자동 section schedule 때문에 사용하지 않는다.

from time import ticks_ms, ticks_diff

try:
    from pai_car_run_support import (
        create_lap_timer,
        setup_paicar,
        self_calibrate_or_stop,
        wait_button_start,
        limit_cmd,
        wait_control_period,
    )
except ImportError:
    from modules.pai_car_run_support import (
        create_lap_timer,
        setup_paicar,
        self_calibrate_or_stop,
        wait_button_start,
        limit_cmd,
        wait_control_period,
    )

try:
    from pai_udp_telemetry import (
        PAIUdpTelemetry,
        read_line_detail,
        is_t_marker_area,
    )
except ImportError:
    from modules.pai_udp_telemetry import (
        PAIUdpTelemetry,
        read_line_detail,
        is_t_marker_area,
    )

try:
    from pai_drive_profiles import DriveProfileManager
except ImportError:
    from modules.pai_drive_profiles import DriveProfileManager

try:
    import paicar_lr_model as lr_model
except ImportError:
    import pai_car_lr_model as lr_model

try:
    from pai_udp_command import (
        PAIUdpCommand,
        RUN_STATE_STOP,
        RUN_STATE_RUN,
    )
except ImportError:
    try:
        from modules.pai_udp_command import (
            PAIUdpCommand,
            RUN_STATE_STOP,
            RUN_STATE_RUN,
        )
    except ImportError:
        PAIUdpCommand = None
        RUN_STATE_STOP = 0
        RUN_STATE_RUN = 1


# ------------------------------------------------------------
# Auto-section schedule settings
# ------------------------------------------------------------

# FAST_V5 48.87s 로그 기준 section 시작 시각표.
# 단위: 주행 시작 후 ms
AUTO_SECTION_START_MS = [
    0,
    3560,
    10170,
    13770,
    15350,
    24300,
    34080,
    35030,
    39950,
    41110,
    44980,
    46700,
]

# 1000 = 원본
# 980  = 전체 section 전환을 2% 빠르게
# 1020 = 전체 section 전환을 2% 느리게
SCHEDULE_SCALE_X1000 = 1000

# +값이면 section 전환을 늦춤, -값이면 앞당김
SECTION_SHIFT_MS = 0

# Finish marker 오검출 방지용 최소 주행 시간
MIN_FINISH_TIME_MS = 25000


# ------------------------------------------------------------
# AI-control settings
# ------------------------------------------------------------

PROFILE_VERSION = 6

# 첫 실차 테스트는 80 권장.
# 안정적이면 90, 100으로 올린다.
ML_BLEND_PERCENT = 80

# AI가 기존 PD/profile 출력에서 너무 멀리 벗어나지 못하게 제한
USE_PD_DELTA_GUARD = True
ML_MAX_DELTA_FROM_PD = 220

# AI raw 출력 이상치 방지
AI_RAW_ABS_LIMIT = 2500

# 라인 미검출 복구 fallback
LINE_LOSS_STOP_MS_FALLBACK = 300

# UDP command는 비상 STOP/RUN 용도
ENABLE_UDP_COMMAND = True

OLED_SECTION_UPDATE_MS = 500

PROFILE_ID = {
    "SAFE": 0,
    "STRAIGHT": 1,
    "WIDE_S": 2,
    "NARROW_S": 3,
    "HAIRPIN_U": 4,
    "WIDE_U": 5,
    "UNKNOWN": 255,
}


# ------------------------------------------------------------
# Small utilities
# ------------------------------------------------------------

def clamp_int(value, min_value, max_value):
    value = int(value)

    if value > max_value:
        return int(max_value)

    if value < min_value:
        return int(min_value)

    return value


def is_finite_number(value):
    try:
        value = float(value)
    except Exception:
        return False

    return value == value and value != float("inf") and value != -float("inf")


def limit_near_reference(value, reference, max_delta):
    low = int(reference) - int(max_delta)
    high = int(reference) + int(max_delta)
    return clamp_int(value, low, high)


def get_optional_attr(obj, name, default_value=0):
    if obj is None:
        return default_value

    try:
        value = getattr(obj, name)
    except Exception:
        return default_value

    if callable(value):
        try:
            return value()
        except Exception:
            return default_value

    return value


# ------------------------------------------------------------
# Auto section scheduler
# ------------------------------------------------------------

class AutoSectionScheduler:
    def __init__(self, start_ms_table, scale_x1000=1000, shift_ms=0):
        self.raw_table = start_ms_table
        self.scale_x1000 = int(scale_x1000)
        self.shift_ms = int(shift_ms)
        self.table = []
        self.start_ms = 0
        self.section_id = 0
        self._build_table()

    def _build_table(self):
        self.table = []

        for i, t in enumerate(self.raw_table):
            if i == 0:
                scaled = 0
            else:
                scaled = (int(t) * self.scale_x1000) // 1000 + self.shift_ms
                if scaled < 0:
                    scaled = 0

            self.table.append(int(scaled))

        for i in range(1, len(self.table)):
            if self.table[i] <= self.table[i - 1]:
                self.table[i] = self.table[i - 1] + 1

    def reset(self):
        self.start_ms = ticks_ms()
        self.section_id = 0

    def elapsed_ms(self):
        return ticks_diff(ticks_ms(), self.start_ms)

    def get_section_for_elapsed(self, elapsed_ms):
        section_id = 0

        for i in range(len(self.table) - 1, -1, -1):
            if elapsed_ms >= self.table[i]:
                section_id = i
                break

        if section_id < 0:
            section_id = 0

        if section_id >= len(self.table):
            section_id = len(self.table) - 1

        return section_id

    def update(self, drive_profiles):
        elapsed = self.elapsed_ms()
        section_id = self.get_section_for_elapsed(elapsed)
        self.section_id = section_id
        drive_profiles.set_section_id(section_id)
        return section_id, elapsed


# ------------------------------------------------------------
# Profile feature helpers
# ------------------------------------------------------------

def get_profile_float(profile, key, default_value=0.0):
    try:
        return float(profile.get(key, default_value))
    except Exception:
        return float(default_value)


def get_profile_int(profile, key, default_value=0):
    try:
        return int(profile.get(key, default_value))
    except Exception:
        return int(default_value)


def get_profile_feature_value(profile_key, profile, feature_name):
    if feature_name == "profile_version":
        return PROFILE_VERSION

    if feature_name == "profile_base_speed":
        return get_profile_int(profile, "base_speed", 0)

    if feature_name == "profile_curve_speed":
        return get_profile_int(profile, "curve_speed", 0)

    if feature_name == "profile_sharp_curve_speed":
        return get_profile_int(profile, "sharp_curve_speed", 0)

    if feature_name == "profile_min_run_speed":
        return get_profile_int(profile, "min_run_speed", 0)

    if feature_name == "profile_kp":
        return get_profile_float(profile, "kp_x1000", 0) / 1000.0

    if feature_name == "profile_kd":
        return get_profile_float(profile, "kd_x1000", 0) / 1000.0

    if feature_name == "profile_max_correction":
        return get_profile_int(profile, "max_correction", 0)

    if feature_name == "profile_reverse_allow":
        return 1 if get_profile_int(profile, "reverse_allow", 0) else 0

    if feature_name == "profile_reverse_pwm_mid":
        return get_profile_int(profile, "reverse_pwm_mid", 0)

    if feature_name == "profile_reverse_pwm_high":
        return get_profile_int(profile, "reverse_pwm_high", 0)

    if feature_name == "profile_error_curve_threshold":
        return get_profile_int(profile, "error_curve_threshold", 0)

    if feature_name == "profile_error_sharp_threshold":
        return get_profile_int(profile, "error_sharp_threshold", 0)

    if feature_name == "profile_d_error_curve_threshold":
        return get_profile_int(profile, "d_error_curve_threshold", 0)

    if feature_name == "profile_d_error_sharp_threshold":
        return get_profile_int(profile, "d_error_sharp_threshold", 0)

    if feature_name == "profile_search_pwm":
        return get_profile_int(profile, "search_pwm", 0)

    if feature_name == "profile_line_loss_max_ms":
        return get_profile_int(profile, "line_loss_max_ms", 0)

    return 0


def make_feature_dict(
    norm,
    position,
    error,
    d_error,
    target_speed,
    drive_profiles,
):
    profile_key = drive_profiles.get_profile_key()
    profile = drive_profiles.get_active_profile()
    section_id = drive_profiles.get_section_id()

    features = {}

    for i in range(8):
        features["n{}".format(i)] = int(norm[i])

    features["position"] = int(position)
    features["error"] = int(error)
    features["d_error"] = int(d_error)
    features["base_speed"] = int(target_speed)
    features["actual_section_id"] = int(section_id)
    features["active_profile_id"] = PROFILE_ID.get(profile_key, 255)

    for name in getattr(lr_model, "FEATURE_COLUMNS", []):
        if name.startswith("profile_"):
            features[name] = get_profile_feature_value(profile_key, profile, name)

    return features


# ------------------------------------------------------------
# Controller helpers
# ------------------------------------------------------------

def compute_pd_drive(error, d_error, drive_profiles):
    target_speed = drive_profiles.compute_target_speed(error, d_error)
    correction = drive_profiles.compute_correction(error, d_error)

    left = target_speed + correction
    right = target_speed - correction

    left = drive_profiles.limit_drive_cmd(left, error)
    right = drive_profiles.limit_drive_cmd(right, error)

    return int(target_speed), int(left), int(right)


def predict_ai_drive(features, pd_left, pd_right, drive_profiles, error):
    try:
        raw_left, raw_right = lr_model.predict_raw(features)
    except Exception:
        return int(pd_left), int(pd_right), False

    if not is_finite_number(raw_left) or not is_finite_number(raw_right):
        return int(pd_left), int(pd_right), False

    if abs(float(raw_left)) > AI_RAW_ABS_LIMIT or abs(float(raw_right)) > AI_RAW_ABS_LIMIT:
        return int(pd_left), int(pd_right), False

    ai_left = int(raw_left)
    ai_right = int(raw_right)

    ai_left = drive_profiles.limit_drive_cmd(ai_left, error)
    ai_right = drive_profiles.limit_drive_cmd(ai_right, error)

    if USE_PD_DELTA_GUARD:
        ai_left = limit_near_reference(ai_left, pd_left, ML_MAX_DELTA_FROM_PD)
        ai_right = limit_near_reference(ai_right, pd_right, ML_MAX_DELTA_FROM_PD)

    blend = clamp_int(ML_BLEND_PERCENT, 0, 100)

    left = (ai_left * blend + pd_left * (100 - blend)) // 100
    right = (ai_right * blend + pd_right * (100 - blend)) // 100

    left = drive_profiles.limit_drive_cmd(left, error)
    right = drive_profiles.limit_drive_cmd(right, error)

    return int(left), int(right), True


def compute_line_loss_drive(last_error, drive_profiles):
    search_pwm = drive_profiles.get_search_pwm()

    if last_error >= 0:
        left = search_pwm
        right = -search_pwm
    else:
        left = -search_pwm
        right = search_pwm

    left = limit_cmd(left)
    right = limit_cmd(right)

    return int(left), int(right)


# ------------------------------------------------------------
# Optional UDP command helpers
# ------------------------------------------------------------

def create_optional_command():
    if not ENABLE_UDP_COMMAND:
        return None

    if PAIUdpCommand is None:
        return None

    try:
        cmd = PAIUdpCommand(require_heartbeat=False)
        cmd.begin()
        return cmd
    except Exception:
        return None


def poll_optional_command(cmd):
    if cmd is None:
        return RUN_STATE_RUN

    try:
        cmd.poll()
        return cmd.get_run_state()
    except Exception:
        return RUN_STATE_RUN


def update_telemetry_state(telemetry, cmd, run_state, drive_profiles):
    if not hasattr(telemetry, "set_command_echo"):
        return

    last_cmd_seq = get_optional_attr(cmd, "get_last_cmd_seq", 0)
    last_cmd_type = get_optional_attr(cmd, "get_last_cmd_type", 0)
    last_cmd_status = get_optional_attr(cmd, "get_last_cmd_status", 0)

    try:
        telemetry.set_command_echo(
            run_state,
            drive_profiles.get_section_id(),
            drive_profiles.get_profile_key(),
            last_cmd_seq,
            last_cmd_type,
            last_cmd_status,
        )
    except Exception:
        pass


# ------------------------------------------------------------
# Setup
# ------------------------------------------------------------

lap_timer = create_lap_timer()
line, motors, button = setup_paicar(lap_timer)

drive_profiles = DriveProfileManager()
scheduler = AutoSectionScheduler(
    AUTO_SECTION_START_MS,
    scale_x1000=SCHEDULE_SCALE_X1000,
    shift_ms=SECTION_SHIFT_MS,
)

telemetry = PAIUdpTelemetry(lap_timer)
telemetry.begin()

cmd = create_optional_command()

try:
    lap_timer.show(
        "AI AUTO READY",
        "blend {}%".format(ML_BLEND_PERCENT),
        "No Space input",
        "Press button",
    )
except Exception:
    pass


# ------------------------------------------------------------
# Self calibration
# ------------------------------------------------------------

self_calibrate_or_stop(line, motors, lap_timer)


# ------------------------------------------------------------
# Button start
# ------------------------------------------------------------

wait_button_start(button, lap_timer)

drive_profiles.reset()
scheduler.reset()

lap_timer.start()
telemetry.reset_timer()


# ------------------------------------------------------------
# Autonomous AI line tracing
# ------------------------------------------------------------

finished = False
last_error = 0
last_line_seen_ms = ticks_ms()
last_oled_section_ms = ticks_ms()

left_cmd = 0
right_cmd = 0
d_error = 0
base_speed = 0
run_state = RUN_STATE_RUN
ai_ok = False

try:
    while True:
        loop_start = ticks_ms()

        # --------------------------------------------------------
        # 0. 자동 section/profile 갱신
        # --------------------------------------------------------
        section_id, elapsed_ms = scheduler.update(drive_profiles)

        # UDP command는 비상 STOP/RUN만 사용한다.
        # Space/NEXT_SECTION은 자동 schedule 때문에 주행 제어에 사용하지 않는다.
        run_state = poll_optional_command(cmd)
        update_telemetry_state(telemetry, cmd, run_state, drive_profiles)

        # --------------------------------------------------------
        # 1. STOP 상태 처리
        # --------------------------------------------------------
        if run_state == RUN_STATE_STOP:
            motors.stop()

            error, position, norm, on_line = read_line_detail(line)
            is_marker = is_t_marker_area(norm, on_line)

            left_cmd = 0
            right_cmd = 0
            d_error = 0
            base_speed = 0

            telemetry.send_if_due(
                base_speed,
                norm,
                position,
                error,
                d_error,
                left_cmd,
                right_cmd,
                on_line,
                is_marker,
            )

            lap_timer.update()
            wait_control_period(loop_start)
            continue

        # --------------------------------------------------------
        # 2. 라인센서 읽기
        # --------------------------------------------------------
        error, position, norm, on_line = read_line_detail(line)
        is_marker = is_t_marker_area(norm, on_line)

        # --------------------------------------------------------
        # 3. Finish 확인
        # --------------------------------------------------------
        if elapsed_ms >= MIN_FINISH_TIME_MS and lap_timer.check_finish(norm, on_line):
            motors.stop()
            finished = True

            update_telemetry_state(telemetry, cmd, RUN_STATE_STOP, drive_profiles)

            telemetry.send_now(
                0,
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

        # --------------------------------------------------------
        # 4. AI 제어
        # --------------------------------------------------------
        if on_line:
            now_ms = ticks_ms()
            last_line_seen_ms = now_ms

            d_error = error - last_error
            last_error = error

            base_speed, pd_left, pd_right = compute_pd_drive(
                error,
                d_error,
                drive_profiles,
            )

            features = make_feature_dict(
                norm,
                position,
                error,
                d_error,
                base_speed,
                drive_profiles,
            )

            left_cmd, right_cmd, ai_ok = predict_ai_drive(
                features,
                pd_left,
                pd_right,
                drive_profiles,
                error,
            )

            motors.drive(left_cmd, right_cmd)

        else:
            line_loss_ms = ticks_diff(ticks_ms(), last_line_seen_ms)
            max_loss_ms = drive_profiles.get_line_loss_max_ms()

            if max_loss_ms <= 0:
                max_loss_ms = LINE_LOSS_STOP_MS_FALLBACK

            if line_loss_ms <= max_loss_ms:
                left_cmd, right_cmd = compute_line_loss_drive(
                    last_error,
                    drive_profiles,
                )
                motors.drive(left_cmd, right_cmd)
            else:
                motors.stop()
                left_cmd = 0
                right_cmd = 0

            d_error = 0
            base_speed = drive_profiles.get_search_pwm()
            ai_ok = False

        # --------------------------------------------------------
        # 5. Telemetry 전송
        # --------------------------------------------------------
        update_telemetry_state(telemetry, cmd, run_state, drive_profiles)

        telemetry.send_if_due(
            base_speed,
            norm,
            position,
            error,
            d_error,
            left_cmd,
            right_cmd,
            on_line,
            is_marker,
        )

        # --------------------------------------------------------
        # 6. OLED 갱신
        # --------------------------------------------------------
        now_ms = ticks_ms()

        if ticks_diff(now_ms, last_oled_section_ms) >= OLED_SECTION_UPDATE_MS:
            try:
                lap_timer.show(
                    "AI AUTO",
                    "sec {} {}".format(
                        drive_profiles.get_section_id(),
                        drive_profiles.get_profile_key(),
                    ),
                    "{}ms".format(elapsed_ms),
                    "AI" if ai_ok else "PD fallback",
                )
            except Exception:
                pass

            last_oled_section_ms = now_ms

        else:
            lap_timer.update()

        # --------------------------------------------------------
        # 7. 제어 주기 맞추기
        # --------------------------------------------------------
        wait_control_period(loop_start)

finally:
    motors.stop()

    try:
        telemetry.close()
    except Exception:
        pass

    if cmd is not None:
        try:
            cmd.close()
        except Exception:
            pass

    if not finished:
        lap_timer.show_stopped()
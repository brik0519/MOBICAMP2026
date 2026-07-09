# modules/pai_drive_profiles.py
# PAI-Car Pico 내부 코스 profile table
#
# FAST_V2
#
# 기준:
#   - SPEED=1000, KP=0.55, KD=0.22로 전 구간 통과 가능했던 결과 반영
#   - FAST_V1 48.79s 완주 성공 결과 반영
#   - 속도는 더 올리고, KP/KD는 0.55/0.22 중심으로 정리
#
# 목적:
#   - 경기 코스의 section 순서를 Pico 내부에 고정한다.
#   - 각 section에 대응하는 profile 값을 Pico RAM에서 즉시 참조한다.
#   - PC는 Space로 NEXT_SECTION만 보내고, profile 값 묶음은 주행 중 매번 전송하지 않는다.
#
# 주의:
#   - kp, kd는 MicroPython float 전송을 피하기 위해 x1000 정수로 저장한다.
#   - 예: kp=0.55 -> kp_x1000=550
#   - 이 파일은 flash 저장/수정을 하지 않는다.


MOTOR_CMD_MAX = 1000


# ------------------------------------------------------------
# Profile table
# ------------------------------------------------------------

PROFILES = {
    "STRAIGHT": {
        "label_ko": "직진",
        "base_speed": 1000,
        "curve_speed": 1000,
        "sharp_curve_speed": 940,
        "min_run_speed": 600,
        "kp_x1000": 550,
        "kd_x1000": 200,
        "max_correction": 900,
        "reverse_allow": 0,
        "reverse_pwm_mid": 0,
        "reverse_pwm_high": 0,
        "error_curve_threshold": 2000,
        "error_sharp_threshold": 3200,
        "d_error_curve_threshold": 1500,
        "d_error_sharp_threshold": 2400,
        "search_pwm": 260,
        "line_loss_max_ms": 220,
    },

    "WIDE_S": {
        "label_ko": "넓은 S자",
        "base_speed": 1000,
        "curve_speed": 950,
        "sharp_curve_speed": 830,
        "min_run_speed": 580,
        "kp_x1000": 550,
        "kd_x1000": 220,
        "max_correction": 950,
        "reverse_allow": 1,
        "reverse_pwm_mid": -60,
        "reverse_pwm_high": -180,
        "error_curve_threshold": 1750,
        "error_sharp_threshold": 3000,
        "d_error_curve_threshold": 1350,
        "d_error_sharp_threshold": 2250,
        "search_pwm": 280,
        "line_loss_max_ms": 240,
    },

    "NARROW_S": {
        "label_ko": "좁은 S자",
        "base_speed": 1000,
        "curve_speed": 920,
        "sharp_curve_speed": 780,
        "min_run_speed": 560,
        "kp_x1000": 560,
        "kd_x1000": 240,
        "max_correction": 1050,
        "reverse_allow": 1,
        "reverse_pwm_mid": -120,
        "reverse_pwm_high": -260,
        "error_curve_threshold": 1450,
        "error_sharp_threshold": 2500,
        "d_error_curve_threshold": 1000,
        "d_error_sharp_threshold": 1800,
        "search_pwm": 300,
        "line_loss_max_ms": 280,
    },

    "HAIRPIN_U": {
        "label_ko": "헤어핀",
        "base_speed": 1000,
        "curve_speed": 930,
        "sharp_curve_speed": 820,
        "min_run_speed": 560,
        "kp_x1000": 550,
        "kd_x1000": 220,
        "max_correction": 1000,
        "reverse_allow": 1,
        "reverse_pwm_mid": -80,
        "reverse_pwm_high": -240,
        "error_curve_threshold": 1600,
        "error_sharp_threshold": 2800,
        "d_error_curve_threshold": 1150,
        "d_error_sharp_threshold": 2000,
        "search_pwm": 320,
        "line_loss_max_ms": 320,
    },

    "WIDE_U": {
        "label_ko": "완만한 U턴",
        "base_speed": 930,
        "curve_speed": 830,
        "sharp_curve_speed": 700,
        "min_run_speed": 530,
        "kp_x1000": 560,
        "kd_x1000": 240,
        "max_correction": 1050,
        "reverse_allow": 1,
        "reverse_pwm_mid": -120,
        "reverse_pwm_high": -260,
        "error_curve_threshold": 1350,
        "error_sharp_threshold": 2300,
        "d_error_curve_threshold": 950,
        "d_error_sharp_threshold": 1700,
        "search_pwm": 290,
        "line_loss_max_ms": 260,
    },

    "SAFE": {
        "label_ko": "안전 주행",
        "base_speed": 560,
        "curve_speed": 520,
        "sharp_curve_speed": 470,
        "min_run_speed": 420,
        "kp_x1000": 520,
        "kd_x1000": 260,
        "max_correction": 1000,
        "reverse_allow": 1,
        "reverse_pwm_mid": -180,
        "reverse_pwm_high": -300,
        "error_curve_threshold": 1200,
        "error_sharp_threshold": 2200,
        "d_error_curve_threshold": 800,
        "d_error_sharp_threshold": 1500,
        "search_pwm": 260,
        "line_loss_max_ms": 300,
    },
}

# ------------------------------------------------------------
# Course section sequence
# ------------------------------------------------------------

COURSE_SECTIONS = [
    {
        "section_id": 0,
        "display_no": 1,
        "name": "start_zone",
        "label_ko": "시작구간",
        "type": "WIDE_S",
        "profile_key": "WIDE_S",
        "role": "START",
    },
    {
        "section_id": 1,
        "display_no": 2,
        "name": "long_straight",
        "label_ko": "긴 직진 구간",
        "type": "STRAIGHT",
        "profile_key": "STRAIGHT",
        "role": "NORMAL",
    },
    {
        "section_id": 2,
        "display_no": 3,
        "name": "short_s_1",
        "label_ko": "짧은 S자",
        "type": "NARROW_S",
        "profile_key": "NARROW_S",
        "role": "NORMAL",
    },
    {
        "section_id": 3,
        "display_no": 4,
        "name": "short_straight_1",
        "label_ko": "짧은 직진",
        "type": "STRAIGHT",
        "profile_key": "STRAIGHT",
        "role": "NORMAL",
    },
    {
        "section_id": 4,
        "display_no": 5,
        "name": "wide_s_1",
        "label_ko": "넓은 S자",
        "type": "WIDE_S",
        "profile_key": "WIDE_S",
        "role": "NORMAL",
    },
    {
        "section_id": 5,
        "display_no": 6,
        "name": "narrow_s_1",
        "label_ko": "좁은 S자",
        "type": "NARROW_S",
        "profile_key": "NARROW_S",
        "role": "NORMAL",
    },
    {
        "section_id": 6,
        "display_no": 7,
        "name": "hairpin_entry_and_u",
        "label_ko": "헤어핀 진입+헤어핀",
        "type": "HAIRPIN_U",
        "profile_key": "HAIRPIN_U",
        "role": "NORMAL",
    },
    {
        "section_id": 7,
        "display_no": 8,
        "name": "middle_straight",
        "label_ko": "중간 직진구간",
        "type": "STRAIGHT",
        "profile_key": "STRAIGHT",
        "role": "NORMAL",
    },
    {
        "section_id": 8,
        "display_no": 9,
        "name": "wide_u_1",
        "label_ko": "완만한 U턴",
        "type": "WIDE_U",
        "profile_key": "WIDE_U",
        "role": "NORMAL",
    },
    {
        "section_id": 9,
        "display_no": 10,
        "name": "short_straight_2",
        "label_ko": "짧은 직진",
        "type": "STRAIGHT",
        "profile_key": "STRAIGHT",
        "role": "NORMAL",
    },
    {
        "section_id": 10,
        "display_no": 11,
        "name": "half_narrow_s",
        "label_ko": "좁은 S자 절반",
        "type": "NARROW_S",
        "profile_key": "NARROW_S",
        "role": "NORMAL",
    },
    {
        "section_id": 11,
        "display_no": 12,
        "name": "finish_short_straight",
        "label_ko": "Finish 전 짧은 직진",
        "type": "STRAIGHT",
        "profile_key": "STRAIGHT",
        "role": "FINISH_APPROACH",
    },
]


# ------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------

def clamp_int(value, min_value, max_value):
    if value > max_value:
        return max_value

    if value < min_value:
        return min_value

    return int(value)


def div1000_trunc(value):
    if value >= 0:
        return value // 1000

    return -((-value) // 1000)


def get_section_count():
    return len(COURSE_SECTIONS)


def get_max_section_id():
    return len(COURSE_SECTIONS) - 1


def get_section_info(section_id):
    if section_id < 0:
        section_id = 0

    max_section_id = get_max_section_id()

    if section_id > max_section_id:
        section_id = max_section_id

    return COURSE_SECTIONS[section_id]


def get_profile_by_key(profile_key):
    profile = PROFILES.get(profile_key)

    if profile is None:
        return PROFILES["SAFE"]

    return profile


# ------------------------------------------------------------
# Profile manager
# ------------------------------------------------------------

class DriveProfileManager:
    def __init__(self):
        self.section_id = 0
        self.section_info = get_section_info(0)
        self.profile_key = self.section_info["profile_key"]
        self.profile = get_profile_by_key(self.profile_key)

    def set_section_id(self, section_id):
        try:
            section_id = int(section_id)
        except Exception:
            section_id = 0

        section_id = clamp_int(section_id, 0, get_max_section_id())

        changed = section_id != self.section_id

        self.section_id = section_id
        self.section_info = get_section_info(section_id)
        self.profile_key = self.section_info["profile_key"]
        self.profile = get_profile_by_key(self.profile_key)

        return changed

    def sync_section_id(self, section_id):
        return self.set_section_id(section_id)

    def next_section(self):
        return self.set_section_id(self.section_id + 1)

    def reset(self):
        return self.set_section_id(0)

    def get_section_id(self):
        return self.section_id

    def get_section_info(self):
        return self.section_info

    def get_profile_key(self):
        return self.profile_key

    def get_active_profile(self):
        return self.profile

    def get_label(self):
        return self.section_info.get("label_ko", "")

    def get_type(self):
        return self.section_info.get("type", "")

    def get_role(self):
        return self.section_info.get("role", "")

    def is_finish_approach(self):
        return self.get_role() == "FINISH_APPROACH"

    def get_value(self, key, default_value=0):
        return self.profile.get(key, default_value)

    def get_base_speed(self):
        return self.get_value("base_speed", 0)

    def get_search_pwm(self):
        return self.get_value("search_pwm", 260)

    def get_line_loss_max_ms(self):
        return self.get_value("line_loss_max_ms", 250)

    def compute_target_speed(self, error, d_error):
        ae = abs(error)
        ad = abs(d_error)

        speed = self.profile["base_speed"]

        if (
            ae > self.profile["error_sharp_threshold"]
            or ad > self.profile["d_error_sharp_threshold"]
        ):
            speed = self.profile["sharp_curve_speed"]

        elif (
            ae > self.profile["error_curve_threshold"]
            or ad > self.profile["d_error_curve_threshold"]
        ):
            speed = self.profile["curve_speed"]

        if speed < self.profile["min_run_speed"]:
            speed = self.profile["min_run_speed"]

        return int(speed)

    def compute_correction(self, error, d_error):
        total = (
            self.profile["kp_x1000"] * error
            + self.profile["kd_x1000"] * d_error
        )

        correction = div1000_trunc(total)
        max_correction = self.profile["max_correction"]

        return clamp_int(
            correction,
            -max_correction,
            max_correction,
        )

    def limit_drive_cmd(self, value, error):
        ae = abs(error)

        if self.profile["reverse_allow"]:
            if ae >= self.profile["error_sharp_threshold"]:
                min_cmd = self.profile["reverse_pwm_high"]
            elif ae >= self.profile["error_curve_threshold"]:
                min_cmd = self.profile["reverse_pwm_mid"]
            else:
                min_cmd = 0
        else:
            min_cmd = 0

        return clamp_int(value, min_cmd, MOTOR_CMD_MAX)

    def debug_text(self):
        return (
            "section={} no={} label={} type={} profile={} "
            "base={} curve={} sharp={} kp={} kd={} max_corr={}"
        ).format(
            self.section_id,
            self.section_info.get("display_no", ""),
            self.section_info.get("label_ko", ""),
            self.section_info.get("type", ""),
            self.profile_key,
            self.profile.get("base_speed", ""),
            self.profile.get("curve_speed", ""),
            self.profile.get("sharp_curve_speed", ""),
            self.profile.get("kp_x1000", ""),
            self.profile.get("kd_x1000", ""),
            self.profile.get("max_correction", ""),
        )
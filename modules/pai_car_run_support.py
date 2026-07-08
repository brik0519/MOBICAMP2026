# pai_car_run_support.py
# Support functions and default settings for PAI-Car line tracing examples
#
# 이 파일에는 학생들이 자주 수정하지 않아도 되는 설정과 보조 기능을 모아 두었다.
#
# 포함된 기능:
#   - ADC / 라인센서 / 모터 / 버튼 초기화
#   - 셀프 캘리브레이션
#   - 버튼 시작 대기
#   - OLED 초기화 및 표시
#   - 주행 시간 측정
#   - T 마커 검출
#   - 모터 명령 제한
#   - 제어 주기 맞추기

from machine import Pin, I2C
from time import sleep_ms, ticks_ms, ticks_diff

from modules.tla2528 import TLA2528
from modules.pai_line_sensor import PaiLineSensor
from modules.pai_motor import PAICarMotors
from modules.pai_self_calibration import self_calibrate

try:
    import modules.ssd1306 as ssd1306
except ImportError:
    ssd1306 = None


# ------------------------------------------------------------
# Default hardware settings
# ------------------------------------------------------------

BUTTON_PIN = 22

ADC_I2C_ID = 0
ADC_SDA_PIN = 4
ADC_SCL_PIN = 5
ADC_FREQ = 400_000
ADC_STARTUP_DELAY_MS = 300

LINE_SENSOR_COUNT = 8
LINE_CAL_FILE = "line_cal.json"

DARK_IS_LOW = True
MIN_RANGE = 30


# ------------------------------------------------------------
# Default line detection settings
# ------------------------------------------------------------

MIN_TOTAL = 500
NOISE_CUTOFF = 100


# ------------------------------------------------------------
# Default P-control support settings
# ------------------------------------------------------------

CONTROL_MS = 10     # 제어 주기는 10ms 또는 5ms 둘 중 하나 선택
                    # 전송 주기는 제어 주기의 정수 배
                    # 전송 주기 = n x CONTROL_MS

MAX_CMD = 1000


# ------------------------------------------------------------
# Default self-calibration settings
# ------------------------------------------------------------
# 캘리브레이션 관련 기본값이다.
# 자주 바꿀 필요가 없으므로 학생용 p_control.py에서는 숨긴다.

CAL_SPEED = 280
CAL_SEGMENT_MS = 500
CAL_SAMPLE_INTERVAL_MS = 5
CAL_PAUSE_MS = 100


# ------------------------------------------------------------
# Default OLED settings
# ------------------------------------------------------------

OLED_WIDTH = 128
OLED_HEIGHT = 32
OLED_ADDR = 0x3C

OLED_I2C_ID = 1
OLED_SDA_PIN = 2
OLED_SCL_PIN = 3
OLED_FREQ = 400_000

OLED_UPDATE_MS = 1000

# 초기 상태 메시지를 사람이 읽을 수 있도록 유지하는 시간
# 주행 제어 루프가 시작되기 전 단계에서만 사용한다.
OLED_STATUS_HOLD_MS = 700


# ------------------------------------------------------------
# Default T-marker settings
# ------------------------------------------------------------
# T 마커 검출 기준이다.
#
# norm 값이 T_MARKER_TH 이상인 센서가
# T_MARKER_MIN_COUNT개 이상이면 T 마커로 판단한다.
#
# 같은 T 마커 위에 머무르는 동안 여러 번 카운트되지 않도록
# T_MARKER_RELEASE_COUNT를 이용해 마커에서 벗어났는지 확인한다.

T_MARKER_TH = 700
T_MARKER_MIN_COUNT = 6
T_MARKER_RELEASE_COUNT = 3


# ------------------------------------------------------------
# OLED utility functions
# ------------------------------------------------------------

def format_time_ms(elapsed_ms):
    """
    ms 단위 시간을 초 단위 문자열로 변환한다.
    소수점 아래 둘째 자리까지 표시한다.

    예:
        12345 ms -> "12.35s"

    float 연산을 피하기 위해 정수 연산으로 처리한다.
    """

    total_hundredths = (elapsed_ms + 5) // 10
    sec = total_hundredths // 100
    frac = total_hundredths % 100

    return "{}.{:02d}s".format(sec, frac)


def init_oled():
    """
    SSD1306 OLED를 초기화한다.

    OLED 초기화에 실패해도 주행 자체는 가능하도록
    실패 시 None을 반환한다.
    """

    if ssd1306 is None:
        return None

    try:
        i2c_oled = I2C(
            OLED_I2C_ID,
            sda=Pin(OLED_SDA_PIN),
            scl=Pin(OLED_SCL_PIN),
            freq=OLED_FREQ
        )

        oled = ssd1306.SSD1306_I2C(
            OLED_WIDTH,
            OLED_HEIGHT,
            i2c_oled,
            addr=OLED_ADDR
        )

        return oled

    except Exception:
        return None


def oled_show_lines(oled, line0="", line1="", line2="", line3=""):
    """
    128x32 OLED에 최대 4줄을 출력한다.
    OLED가 없으면 아무 작업도 하지 않는다.
    """

    if oled is None:
        return

    try:
        oled.fill(0)
        oled.text(line0[:16], 0, 0)
        oled.text(line1[:16], 0, 8)
        oled.text(line2[:16], 0, 16)
        oled.text(line3[:16], 0, 24)
        oled.show()

    except Exception:
        pass


# ------------------------------------------------------------
# Lap timer
# ------------------------------------------------------------

class LapTimer:
    """
    PAI-Car 주행 시간 측정과 OLED 표시를 담당하는 클래스이다.

    동작 방식:
        - start()가 호출된 시점을 주행 시작 시각으로 저장한다.
        - 주행 중 update()가 OLED에 경과 시간을 표시한다.
        - check_finish()가 T 마커를 검사한다.
        - 첫 번째 T 마커는 Start 지점으로 보고 무시한다.
        - 두 번째 T 마커는 Finish 지점으로 보고 최종 시간을 표시한다.
    """

    def __init__(self, oled):
        self.oled = oled

        self.start_ms = 0
        self.last_update_ms = 0

        self.t_marker_count = 0
        self.t_marker_active = False
        self.t_marker_release_count = 0

        self.finished = False
        self.finish_ms = 0

    def show(self, line0="", line1="", line2="", line3=""):
        """
        OLED에 상태 메시지를 즉시 표시한다.

        주행 중에도 사용할 수 있도록 이 함수에는 시간 지연을 넣지 않는다.
        """

        oled_show_lines(self.oled, line0, line1, line2, line3)

    def show_hold(
        self,
        line0="",
        line1="",
        line2="",
        line3="",
        hold_ms=OLED_STATUS_HOLD_MS
    ):
        """
        OLED에 상태 메시지를 표시하고 일정 시간 유지한다.

        주의:
            이 함수는 초기 설정, 캘리브레이션 안내, 버튼 대기 전 상태 표시처럼
            주행 제어 루프가 시작되기 전 단계에서만 사용한다.
        """

        oled_show_lines(self.oled, line0, line1, line2, line3)

        # OLED가 없으면 불필요하게 기다리지 않는다.
        if self.oled is not None and hold_ms > 0:
            sleep_ms(hold_ms)

    def start(self):
        self.start_ms = ticks_ms()
        self.last_update_ms = self.start_ms

        self.t_marker_count = 0
        self.t_marker_active = False
        self.t_marker_release_count = 0

        self.finished = False
        self.finish_ms = 0

        self.show("RUN", "0.00s", "T: 0/2", "")

    def update(self):
        if self.finished:
            return

        now = ticks_ms()

        if ticks_diff(now, self.last_update_ms) >= OLED_UPDATE_MS:
            elapsed_ms = ticks_diff(now, self.start_ms)

            self.show(
                "RUN",
                format_time_ms(elapsed_ms),
                "T: {}/2".format(self.t_marker_count),
                ""
            )

            self.last_update_ms = now

    def check_finish(self, norm, on_line):
        """
        T 마커를 확인하고 Finish 도달 여부를 반환한다.

        반환값:
            True  -> Finish 도달
            False -> 아직 Finish가 아님
        """

        if self.finished:
            return True

        t_event = self._check_t_marker_event(norm, on_line)

        if not t_event:
            return False

        self.t_marker_count += 1

        elapsed_ms = ticks_diff(ticks_ms(), self.start_ms)

        if self.t_marker_count == 1:
            # 첫 번째 T 마커는 Start 지점이므로 무시한다.
            self.show(
                "RUN",
                format_time_ms(elapsed_ms),
                "T: 1/2",
                ""
            )

            return False

        # 두 번째 T 마커는 Finish 지점이다.
        self.finished = True
        self.finish_ms = ticks_ms()

        final_ms = ticks_diff(self.finish_ms, self.start_ms)

        self.show(
            "FINISH",
            format_time_ms(final_ms),
            "T: 2/2",
            "STOP"
        )

        return True

    def show_stopped(self):
        if self.finished:
            return

        if self.start_ms == 0:
            self.show("STOPPED", "", "", "")
        else:
            elapsed_ms = ticks_diff(ticks_ms(), self.start_ms)

            self.show(
                "STOPPED",
                format_time_ms(elapsed_ms),
                "T: {}/2".format(self.t_marker_count),
                ""
            )

    def _check_t_marker_event(self, norm, on_line):
        """
        새로운 T 마커에 처음 진입한 순간만 True를 반환한다.
        같은 T 마커 위에 계속 머무르는 동안에는 다시 True를 반환하지 않는다.
        """

        black_count = 0

        for v in norm:
            if v >= T_MARKER_TH:
                black_count += 1

        detected = on_line and (black_count >= T_MARKER_MIN_COUNT)

        if detected:
            self.t_marker_release_count = 0

            if not self.t_marker_active:
                self.t_marker_active = True
                return True

        else:
            if self.t_marker_active:
                self.t_marker_release_count += 1

                if self.t_marker_release_count >= T_MARKER_RELEASE_COUNT:
                    self.t_marker_active = False
                    self.t_marker_release_count = 0

        return False


def create_lap_timer():
    oled = init_oled()
    timer = LapTimer(oled)

    timer.show_hold("PAI-Car", "Ready", "", "")

    return timer


# ------------------------------------------------------------
# Setup
# ------------------------------------------------------------

def setup_paicar(lap_timer=None):
    """
    PAI-Car 주행에 필요한 장치들을 준비한다.

    수행 작업:
        - TLA2528 ADC 초기화
        - 라인센서 객체 생성
        - 모터 객체 생성
        - 사용자 버튼 설정
    """

    if lap_timer is not None:
        lap_timer.show_hold("PAI-Car", "ADC init...", "", "")

    adc = TLA2528(
        i2c_id=ADC_I2C_ID,
        sda_pin=ADC_SDA_PIN,
        scl_pin=ADC_SCL_PIN,
        freq=ADC_FREQ,
        address=None,
        startup_delay_ms=ADC_STARTUP_DELAY_MS
    )

    adc.begin(verbose=False)

    if lap_timer is not None:
        lap_timer.show_hold("PAI-Car", "Line sensor", "", "")

    line = PaiLineSensor(
        adc,
        sensor_count=LINE_SENSOR_COUNT,
        dark_is_low=DARK_IS_LOW,
        cal_file=LINE_CAL_FILE,
        min_range=MIN_RANGE,
        min_total=MIN_TOTAL,
        noise_cutoff=NOISE_CUTOFF
    )

    if lap_timer is not None:
        lap_timer.show_hold("PAI-Car", "Motor init...", "", "")

    motors = PAICarMotors()
    motors.stop()

    button = Pin(BUTTON_PIN, Pin.IN)

    if lap_timer is not None:
        lap_timer.show_hold("PAI-Car", "Setup OK", "", "")

    return line, motors, button


# ------------------------------------------------------------
# Self calibration
# ------------------------------------------------------------

def self_calibrate_or_stop(line, motors, lap_timer=None):
    if lap_timer is not None:
        lap_timer.show_hold("SELF CAL", "Running...", "", "")

    ok = self_calibrate(
        line,
        motors,
        cal_speed=CAL_SPEED,
        segment_ms=CAL_SEGMENT_MS,
        sample_interval_ms=CAL_SAMPLE_INTERVAL_MS,
        pause_ms=CAL_PAUSE_MS,
        save=True
    )

    motors.stop()

    if not ok:
        if lap_timer is not None:
            lap_timer.show("CAL FAILED", "Check line", "Motor stopped", "")

        while True:
            motors.stop()
            sleep_ms(1000)

    if lap_timer is not None:
        # 이 메시지는 이후 wait_button_start()에서도 다시 표시되고,
        # 버튼을 누르기 전까지 유지되므로 show_hold()가 꼭 필요하지 않다.
        lap_timer.show("CAL OK", "WAIT BUTTON", "", "")


# ------------------------------------------------------------
# Button
# ------------------------------------------------------------

def wait_button_start(button, lap_timer=None):
    """
    사용자 버튼을 눌렀다가 떼는 동작을 기다린다.

    PAI-Car v1.0 user switch:
        released = LOW  = 0
        pressed  = HIGH = 1
    """

    if lap_timer is not None:
        lap_timer.show("CAL OK", "WAIT BUTTON", "", "")

    # 버튼이 이미 눌린 상태라면 먼저 뗄 때까지 기다림
    while button.value() == 1:
        sleep_ms(20)

    # 버튼이 눌릴 때까지 기다림
    while button.value() == 0:
        sleep_ms(20)

    sleep_ms(50)

    # 버튼이 떼어질 때까지 기다림
    while button.value() == 1:
        sleep_ms(20)

    sleep_ms(50)


# ------------------------------------------------------------
# Line sensor
# ------------------------------------------------------------

def read_line(line):
    """
    라인센서 값을 읽는다.

    반환값:
        error, on_line, norm

    error:
        라인 중심에서 벗어난 정도

    on_line:
        라인을 감지했는지 여부

    norm:
        8개 라인센서의 정규화 값
        T 마커 검출에 사용된다.
    """

    error, _, norm, on_line = line.read_error(
        min_total=MIN_TOTAL,
        noise_cutoff=NOISE_CUTOFF
    )

    return error, on_line, norm


def read_line_error(line):
    """
    P/PD 제어 예제에서 사용할 error와 on_line만 반환한다.

    read_line()은 error, on_line, norm을 반환한다.
    이 중 기본 P 제어와 PD 제어에서는 norm을 사용하지 않으므로
    학생용 예제에서는 error와 on_line만 보이도록 한 번 감싼다.
    """

    error, on_line, _ = read_line(line)
    return error, on_line


# ------------------------------------------------------------
# Motor command
# ------------------------------------------------------------

def limit_cmd(value):
    """
    모터 명령값이 허용 범위를 넘지 않도록 제한한다.
    """

    if value > MAX_CMD:
        return MAX_CMD

    if value < -MAX_CMD:
        return -MAX_CMD

    return int(value)


# ------------------------------------------------------------
# Control period
# ------------------------------------------------------------

def wait_control_period(loop_start):
    """
    제어 주기를 일정하게 맞춘다.
    """

    elapsed = ticks_diff(ticks_ms(), loop_start)
    remain = CONTROL_MS - elapsed

    if remain > 0:
        sleep_ms(remain)
# pai_udp_telemetry.py
# PAI-Car UDP binary telemetry module
#
# 역할:
#   - Pico 2 W Wi-Fi 연결
#   - UDP socket 생성
#   - 주행 데이터를 binary packet으로 변환
#   - 20ms 주기로 PC에 전송
#   - telemetry V2에서 Pico 실제 section/profile 및 command 적용 결과를 함께 전송
#
# 주의:
#   - VERSION=2 packet은 기존 V1 dashboard parser가 바로 읽을 수 없다.
#   - main.py에서 set_command_echo(...)를 호출해야 실제 command 상태가 packet에 반영된다.
#   - main.py 수정 전에는 기본값이 전송된다.

from time import ticks_ms, ticks_diff, sleep_ms
import network
import socket
import struct

from modules.pai_car_run_support import (
    CONTROL_MS,
    MIN_TOTAL,
    NOISE_CUTOFF,
    T_MARKER_TH,
    T_MARKER_MIN_COUNT,
)

from modules.pai_car_wifi_config import (
    WIFI_SSID,
    WIFI_PASSWORD,
    PC_IP,
    PC_PORT,
)


# ------------------------------------------------------------
# Telemetry settings
# ------------------------------------------------------------

SEND_MS = 20

WIFI_TIMEOUT_MS = 15000

MAGIC = 0x5041     # packet identifier: 'PA'
VERSION = 2
RESERVED = 0


# ------------------------------------------------------------
# Runtime state / profile id mapping
# ------------------------------------------------------------

RUN_STATE_STOP = 0
RUN_STATE_RUN = 1
RUN_STATE_UNKNOWN = 255

PROFILE_ID_SAFE = 0
PROFILE_ID_STRAIGHT = 1
PROFILE_ID_WIDE_S = 2
PROFILE_ID_NARROW_S = 3
PROFILE_ID_HAIRPIN_U = 4
PROFILE_ID_WIDE_U = 5
PROFILE_ID_UNKNOWN = 255

PROFILE_ID_BY_KEY = {
    "SAFE": PROFILE_ID_SAFE,
    "STRAIGHT": PROFILE_ID_STRAIGHT,
    "WIDE_S": PROFILE_ID_WIDE_S,
    "NARROW_S": PROFILE_ID_NARROW_S,
    "HAIRPIN_U": PROFILE_ID_HAIRPIN_U,
    "WIDE_U": PROFILE_ID_WIDE_U,
}


# ------------------------------------------------------------
# Binary packet format V2
# ------------------------------------------------------------
#
# <      : little-endian
# H      : magic, uint16
# B      : version, uint8
# B      : reserved, uint8
# H      : seq, uint16
# I      : t_ms, uint32
# H      : control_ms, uint16
# H      : send_ms, uint16
# h      : base_speed, int16
# 8H     : n0~n7, uint16 x 8
# H      : position, uint16
# h      : error, int16
# h      : d_error, int16
# h      : left_cmd, int16
# h      : right_cmd, int16
# B      : on_line, uint8
# B      : is_marker, uint8
#
# V2 추가:
# B      : run_state, uint8
# B      : actual_section_id, uint8
# B      : active_profile_id, uint8
# H      : last_cmd_seq, uint16
# B      : last_cmd_type, uint8
# B      : last_cmd_status, uint8
#
# Total V1: 44 bytes
# Total V2: 51 bytes

PACKET_FORMAT = "<HBBHIHHh8HHhhhhBBBBBHBB"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)


# ------------------------------------------------------------
# Clamp helpers
# ------------------------------------------------------------

def clamp_int(value, min_value, max_value):
    try:
        value = int(value)
    except Exception:
        value = min_value

    if value < min_value:
        return min_value

    if value > max_value:
        return max_value

    return value


def clamp_u8(value):
    return clamp_int(value, 0, 255)


def clamp_u16(value):
    return clamp_int(value, 0, 65535)


def clamp_u32(value):
    return clamp_int(value, 0, 4294967295)


def clamp_i16(value):
    return clamp_int(value, -32768, 32767)


def profile_key_to_id(profile_key):
    profile_id = PROFILE_ID_BY_KEY.get(profile_key)

    if profile_id is None:
        return PROFILE_ID_UNKNOWN

    return profile_id


# ------------------------------------------------------------
# Line sensor helper
# ------------------------------------------------------------

def read_line_detail(line):
    """
    라인센서 값을 읽고 제어/전송에 필요한 값을 반환한다.

    반환값:
        error, position, norm, on_line

    error:
        position - 3500

    position:
        왼쪽 끝 0, 중앙 3500, 오른쪽 끝 7000

    norm:
        8개 라인센서 정규화값, 0~1000

    on_line:
        라인 감지 여부
    """

    error, position, norm, on_line = line.read_error(
        min_total=MIN_TOTAL,
        noise_cutoff=NOISE_CUTOFF
    )

    return error, position, norm, on_line


def is_t_marker_area(norm, on_line):
    """
    현재 순간이 T 마커 구간인지 판단한다.

    lap_timer.check_finish()는 T 마커 '이벤트'를 세는 함수이고,
    이 함수는 현재 센서 상태가 T 마커 위인지 여부만 반환한다.
    """

    if not on_line:
        return False

    black_count = 0

    for v in norm:
        if v >= T_MARKER_TH:
            black_count += 1

    return black_count >= T_MARKER_MIN_COUNT


# ------------------------------------------------------------
# Wi-Fi / UDP telemetry class
# ------------------------------------------------------------

class PAIUdpTelemetry:
    """
    PAI-Car 주행 데이터를 PC로 전송하는 클래스.

    main 파일에서는 다음 정도만 사용하면 된다.

        telemetry = PAIUdpTelemetry(lap_timer)
        telemetry.begin()
        telemetry.reset_timer()
        telemetry.set_command_echo(...)
        telemetry.send_if_due(...)
        telemetry.send_now(...)
        telemetry.close()
    """

    def __init__(self, lap_timer=None):
        self.lap_timer = lap_timer

        self.wlan = None
        self.sock = None

        self.seq = 0
        self.run_start_ms = ticks_ms()
        self.last_send_ms = ticks_ms()

        self.enabled = False

        # Telemetry V2 command/profile echo fields.
        self.run_state = RUN_STATE_RUN
        self.actual_section_id = 0
        self.active_profile_id = PROFILE_ID_UNKNOWN
        self.last_cmd_seq = 0
        self.last_cmd_type = 0
        self.last_cmd_status = 0

    def begin(self):
        """
        Wi-Fi에 연결하고 UDP socket을 준비한다.

        실패해도 주행 자체는 가능하도록 enabled=False 상태로 둔다.
        """

        if not self._connect_wifi():
            self.enabled = False
            return False

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.enabled = True

            print("UDP telemetry ready")
            print("Target:", PC_IP, PC_PORT)
            print("Telemetry version:", VERSION)
            print("Packet size:", PACKET_SIZE)

            if self.lap_timer is not None:
                self.lap_timer.show(
                    "UDP READY V{}".format(VERSION),
                    PC_IP[:16],
                    "port {}".format(PC_PORT),
                    "{} bytes".format(PACKET_SIZE)
                )

            return True

        except OSError as e:
            print("UDP socket error:", e)

            if self.lap_timer is not None:
                self.lap_timer.show("UDP ERROR", "socket failed", "", "")

            self.enabled = False
            return False

    def reset_timer(self):
        """
        주행 시작 시점에 맞춰 전송 시간 기준을 초기화한다.
        """

        now = ticks_ms()
        self.run_start_ms = now
        self.last_send_ms = now
        self.seq = 0

    def set_command_echo(
        self,
        run_state,
        actual_section_id,
        active_profile_key,
        last_cmd_seq,
        last_cmd_type,
        last_cmd_status
    ):
        """
        telemetry V2에 실어 보낼 Pico 실제 command/profile 상태를 갱신한다.

        run_state:
            0 STOP, 1 RUN

        actual_section_id:
            Pico가 실제 적용 중인 DriveProfileManager section_id

        active_profile_key:
            "STRAIGHT", "WIDE_S", "NARROW_S", "HAIRPIN_U", "WIDE_U", "SAFE"

        last_cmd_seq / last_cmd_type / last_cmd_status:
            Pico command receiver가 마지막으로 적용한 command 결과
        """

        self.run_state = clamp_u8(run_state)
        self.actual_section_id = clamp_u8(actual_section_id)
        self.active_profile_id = profile_key_to_id(active_profile_key)
        self.last_cmd_seq = clamp_u16(last_cmd_seq)
        self.last_cmd_type = clamp_u8(last_cmd_type)
        self.last_cmd_status = clamp_u8(last_cmd_status)

    def send_if_due(
        self,
        base_speed,
        norm,
        position,
        error,
        d_error,
        left_cmd,
        right_cmd,
        on_line,
        is_marker
    ):
        """
        SEND_MS가 지났으면 주행 데이터를 한 번 전송한다.
        """

        if not self.enabled:
            return

        now = ticks_ms()

        if ticks_diff(now, self.last_send_ms) < SEND_MS:
            return

        t_ms = ticks_diff(now, self.run_start_ms)

        self._send_packet(
            t_ms,
            base_speed,
            norm,
            position,
            error,
            d_error,
            left_cmd,
            right_cmd,
            on_line,
            is_marker
        )

        self.last_send_ms = now

    def send_now(
        self,
        base_speed,
        norm,
        position,
        error,
        d_error,
        left_cmd,
        right_cmd,
        on_line,
        is_marker
    ):
        """
        전송 주기와 관계없이 주행 데이터를 즉시 한 번 전송한다.
        Finish 순간의 마지막 상태를 보낼 때 사용한다.
        """

        if not self.enabled:
            return

        now = ticks_ms()
        t_ms = ticks_diff(now, self.run_start_ms)

        self._send_packet(
            t_ms,
            base_speed,
            norm,
            position,
            error,
            d_error,
            left_cmd,
            right_cmd,
            on_line,
            is_marker
        )

        self.last_send_ms = now

    def close(self):
        """
        UDP socket을 닫는다.
        """

        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass

        self.sock = None
        self.enabled = False

    def _connect_wifi(self):
        """
        Pico 2 W를 Wi-Fi에 연결한다.
        """

        if WIFI_SSID == "" or WIFI_SSID == "YOUR_WIFI_SSID":
            print("Wi-Fi SSID is not set. UDP disabled.")

            if self.lap_timer is not None:
                self.lap_timer.show("UDP OFF", "Set WiFi info", "", "")

            return False

        if self.lap_timer is not None:
            self.lap_timer.show("WiFi", "Connecting...", "", "")

        self.wlan = network.WLAN(network.STA_IF)
        self.wlan.active(True)

        if not self.wlan.isconnected():
            self.wlan.connect(WIFI_SSID, WIFI_PASSWORD)

            start = ticks_ms()

            while not self.wlan.isconnected():
                if ticks_diff(ticks_ms(), start) >= WIFI_TIMEOUT_MS:
                    print("Wi-Fi connection failed. UDP disabled.")

                    if self.lap_timer is not None:
                        self.lap_timer.show("WiFi FAILED", "UDP disabled", "", "")

                    return False

                sleep_ms(200)

        ip = self.wlan.ifconfig()[0]
        print("Wi-Fi connected:", ip)

        if self.lap_timer is not None:
            self.lap_timer.show("WiFi OK", ip[:16], "", "")

        return True

    def _send_packet(
        self,
        t_ms,
        base_speed,
        norm,
        position,
        error,
        d_error,
        left_cmd,
        right_cmd,
        on_line,
        is_marker
    ):
        """
        실제 binary packet을 만들어 PC로 전송한다.
        """

        try:
            packet = struct.pack(
                PACKET_FORMAT,
                MAGIC,
                VERSION,
                RESERVED,
                clamp_u16(self.seq),
                clamp_u32(t_ms),
                clamp_u16(CONTROL_MS),
                clamp_u16(SEND_MS),
                clamp_i16(base_speed),

                clamp_u16(norm[0]),
                clamp_u16(norm[1]),
                clamp_u16(norm[2]),
                clamp_u16(norm[3]),
                clamp_u16(norm[4]),
                clamp_u16(norm[5]),
                clamp_u16(norm[6]),
                clamp_u16(norm[7]),

                clamp_u16(position),
                clamp_i16(error),
                clamp_i16(d_error),
                clamp_i16(left_cmd),
                clamp_i16(right_cmd),
                1 if on_line else 0,
                1 if is_marker else 0,

                clamp_u8(self.run_state),
                clamp_u8(self.actual_section_id),
                clamp_u8(self.active_profile_id),
                clamp_u16(self.last_cmd_seq),
                clamp_u8(self.last_cmd_type),
                clamp_u8(self.last_cmd_status),
            )

            self.sock.sendto(packet, (PC_IP, PC_PORT))
            self.seq = (self.seq + 1) & 0xFFFF

        except OSError:
            # 전송 실패가 발생해도 주행이 멈추지 않도록 무시한다.
            pass
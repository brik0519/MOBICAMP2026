# pai_udp_telemetry.py
# PAI-Car UDP binary telemetry module
#
# 확장 내용:
#   - encoder distance_ticks, left_speed, right_speed, dl, dr, heading_ticks 전송
#   - UDP 송신이 포함된 루프의 loop_ms 전송
#   - UDP 송신 비용 send_cost_ms 전송
#   - loop_ms > CONTROL_MS 여부 overrun 전송
#
# 주의:
#   - PC 수신 코드의 PACKET_FORMAT과 CSV_HEADER도 반드시 같이 바꿔야 한다.

from time import ticks_ms, ticks_us, ticks_diff, sleep_ms
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

MAGIC = 0x5041
VERSION = 2
RESERVED = 0


# ------------------------------------------------------------
# Binary packet format
# ------------------------------------------------------------
#
# 기존 44 bytes에서 encoder/timing 필드를 추가한다.
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
# i      : distance_ticks, int32
# h      : left_speed, int16
# h      : right_speed, int16
# h      : dl, int16
# h      : dr, int16
# i      : heading_ticks, int32
# H      : udp_loop_ms, uint16
# H      : udp_send_cost_ms, uint16
# B      : udp_overrun, uint8
# B      : reserved2, uint8
#
# Total: 66 bytes

PACKET_FORMAT = "<HBBHIHHh8HHhhhhBBihhhhihhBB"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)


# ------------------------------------------------------------
# Line sensor helper
# ------------------------------------------------------------

def read_line_detail(line):
    error, position, norm, on_line = line.read_error(
        min_total=MIN_TOTAL,
        noise_cutoff=NOISE_CUTOFF
    )

    return error, position, norm, on_line


def is_t_marker_area(norm, on_line):
    if not on_line:
        return False

    black_count = 0

    for v in norm:
        if v >= T_MARKER_TH:
            black_count += 1

    return black_count >= T_MARKER_MIN_COUNT


# ------------------------------------------------------------
# Safe conversion helper
# ------------------------------------------------------------

def clamp_i16(value):
    value = int(value)

    if value > 32767:
        return 32767

    if value < -32768:
        return -32768

    return value


def clamp_u16(value):
    value = int(value)

    if value > 65535:
        return 65535

    if value < 0:
        return 0

    return value


def clamp_i32(value):
    value = int(value)

    if value > 2147483647:
        return 2147483647

    if value < -2147483648:
        return -2147483648

    return value


# ------------------------------------------------------------
# Wi-Fi / UDP telemetry class
# ------------------------------------------------------------

class PAIUdpTelemetry:
    def __init__(self, lap_timer=None):
        self.lap_timer = lap_timer

        self.wlan = None
        self.sock = None

        self.seq = 0
        self.run_start_ms = ticks_ms()
        self.last_send_ms = ticks_ms()

        self.enabled = False

        # 마지막 UDP 송신 비용.
        # 현재 패킷에는 직전 송신 비용이 기록된다.
        self.last_send_cost_ms = 0

        # send_if_due()가 직전 호출에서 실제 전송했는지 여부
        self.last_sent = False

    def begin(self):
        if not self._connect_wifi():
            self.enabled = False
            return False

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            # 매우 중요:
            # UDP sendto가 제어 루프를 오래 붙잡지 않도록 non-blocking으로 둔다.
            try:
                self.sock.setblocking(False)
            except Exception:
                pass

            self.enabled = True

            print("UDP telemetry ready")
            print("Target:", PC_IP, PC_PORT)
            print("Packet size:", PACKET_SIZE)
            print("Packet version:", VERSION)

            if self.lap_timer is not None:
                self.lap_timer.show(
                    "UDP READY",
                    PC_IP[:16],
                    "port {}".format(PC_PORT),
                    "size {}".format(PACKET_SIZE)
                )

            return True

        except OSError as e:
            print("UDP socket error:", e)

            if self.lap_timer is not None:
                self.lap_timer.show("UDP ERROR", "socket failed", "", "")

            self.enabled = False
            return False

    def reset_timer(self):
        now = ticks_ms()
        self.run_start_ms = now
        self.last_send_ms = now
        self.last_send_cost_ms = 0
        self.last_sent = False

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
        is_marker,
        distance_ticks=0,
        left_speed=0,
        right_speed=0,
        dl=0,
        dr=0,
        heading_ticks=0,
        udp_loop_ms=0,
        udp_send_cost_ms=0,
        udp_overrun=0
    ):
        """
        SEND_MS가 지났으면 주행 데이터를 한 번 전송한다.

        반환값:
            True  -> 이번 호출에서 실제 전송함
            False -> 전송하지 않음
        """

        self.last_sent = False

        if not self.enabled:
            return False

        now = ticks_ms()

        if ticks_diff(now, self.last_send_ms) < SEND_MS:
            return False

        t_ms = ticks_diff(now, self.run_start_ms)

        ok = self._send_packet(
            t_ms,
            base_speed,
            norm,
            position,
            error,
            d_error,
            left_cmd,
            right_cmd,
            on_line,
            is_marker,
            distance_ticks,
            left_speed,
            right_speed,
            dl,
            dr,
            heading_ticks,
            udp_loop_ms,
            udp_send_cost_ms,
            udp_overrun
        )

        self.last_send_ms = now
        self.last_sent = ok

        return ok

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
        is_marker,
        distance_ticks=0,
        left_speed=0,
        right_speed=0,
        dl=0,
        dr=0,
        heading_ticks=0,
        udp_loop_ms=0,
        udp_send_cost_ms=0,
        udp_overrun=0
    ):
        if not self.enabled:
            return False

        now = ticks_ms()
        t_ms = ticks_diff(now, self.run_start_ms)

        ok = self._send_packet(
            t_ms,
            base_speed,
            norm,
            position,
            error,
            d_error,
            left_cmd,
            right_cmd,
            on_line,
            is_marker,
            distance_ticks,
            left_speed,
            right_speed,
            dl,
            dr,
            heading_ticks,
            udp_loop_ms,
            udp_send_cost_ms,
            udp_overrun
        )

        self.last_send_ms = now
        self.last_sent = ok

        return ok

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass

        self.sock = None
        self.enabled = False

    def _connect_wifi(self):
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
                        self.lap_timer.show(
                            "WiFi FAILED",
                            "UDP disabled",
                            "",
                            ""
                        )

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
        is_marker,
        distance_ticks,
        left_speed,
        right_speed,
        dl,
        dr,
        heading_ticks,
        udp_loop_ms,
        udp_send_cost_ms,
        udp_overrun
    ):
        start_us = ticks_us()

        try:
            packet = struct.pack(
                PACKET_FORMAT,
                MAGIC,
                VERSION,
                RESERVED,
                self.seq,
                int(t_ms),
                CONTROL_MS,
                SEND_MS,
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

                clamp_i32(distance_ticks),
                clamp_i16(left_speed),
                clamp_i16(right_speed),
                clamp_i16(dl),
                clamp_i16(dr),
                clamp_i32(heading_ticks),
                clamp_u16(udp_loop_ms),
                clamp_u16(udp_send_cost_ms),
                1 if udp_overrun else 0,
                0
            )

            self.sock.sendto(packet, (PC_IP, PC_PORT))
            self.seq = (self.seq + 1) & 0xFFFF

            cost_us = ticks_diff(ticks_us(), start_us)
            self.last_send_cost_ms = clamp_u16((cost_us + 999) // 1000)

            return True

        except OSError:
            # non-blocking UDP에서 전송 실패가 발생해도 주행을 멈추면 안 된다.
            cost_us = ticks_diff(ticks_us(), start_us)
            self.last_send_cost_ms = clamp_u16((cost_us + 999) // 1000)

            return False